# -*- coding: utf-8 -*-
"""
[통합 전처리 및 판정 파이프라인 - 포팅용 설계도]
- 대상: 데이터 취득, 시그널 전처리, 특징 추출, ML 예측 및 하이브리드 판정
- 특징: 클래스 없이 함수형으로 구성, IoT Edge 의존성 제거, 상세 파라미터 변수화
"""

import os
import re
import h5py
import pickle
import itertools
import numpy as np
import pandas as pd
import noisereduce as nr
from scipy.fft import fft, ifft
from scipy.signal import get_window, butter, filtfilt, lfilter, hilbert
from obspy.signal.trigger import carl_sta_trig
from datetime import datetime

# =================================================================
# [SECTION 1] 전역 설정 및 파라미터 (Global Configuration)
# =================================================================

# 1. 공통 하드웨어/데이터 파라미터 (from feature051_parameters.json)
SAMPLE_RATE = 42000             # 샘플링 주파수 (Hz)
TARGET_TEST_NO = [1, 2]         # 3회 테스트 중 분석 대상 (인덱스 1, 2)
FFT_PADDING = 8400              # FFT/Cepstrum 연산 시 패딩 크기
DB_REFERENCE = 1e-6             # dB 변환 기준값

# 2. 전처리 상세 파라미터 (from feature051_parameters.json)
PRE_EMPHASIS_ALPHA = 0.97       # 고주파 강조 계수
VALID_INTERVAL = {"from": 0.295, "to": 0.495}  # 유효 신호 구간
TRIGGER_INTERVAL = {"from": 0.0, "to": 0.2}    # 트리거 검출 구간
NOISE_INTERVAL = {"from": 0.0, "to": 0.15}     # 노이즈 프로파일 구간

# 3. ANC(능동 소음 제거) 상세 설정 (from feature051_parameters.json)
ANC_PARAMS = {
    "n_fft": 5600,              # ANC용 FFT 윈도우 크기
    "win_length": 420,          # ANC 분석 윈도우 길이
    "hop_length": 210,          # ANC 윈도우 간격
    "n_grad_freq": 2,           # 주파수 축 평활화
    "n_grad_time": 4,           # 시간 축 평활화
    "n_std_thresh": 1.5         # 소음 판단 임계치
}

# 4. 라인(LID)별 가변 설정 (from cutoffs.json)
ANC_PROP_DECREASE = {1: 1.0, 2: 1.0, 3: 1.0, 4: 1.0, 5: 0.8, 6: 1.0, 7: 1.0, 8: 1.0}
ML_CUTOFFS = {1: 0.5, 2: 0.5, 3: 0.5, 4: 0.6, 5: 0.6, 6: 0.47, 7: 0.5, 8: 0.5}

# 5. 규칙 기반 판정 임계치 (Rule Cutoffs) (from cutoffs.json)
RULE_THRESHOLD_ENRG = 0.00003   # 에너지 임계치
RULE_THRESHOLD_STDDEV = 0.12    # 파형 표준편차 임계치
RULE_THRESHOLD_TRGER = 1        # 최소 트리거 발생 수

# 6. 모델 입력 피처 리스트 (from model_03_051_01_features.json)
MODEL_FEATURE_NAMES = [
    "FTUR_CPSR_MAX_N1", "FTUR_CPSR_MAX_N2", "FTUR_CPSR_MAX_N3", "FTUR_CPSR_MAX_N4",
    "FTUR_CPSR_MXRMS_N1", "FTUR_CPSR_MXRMS_N2", "FTUR_CPSR_MXRMS_N3", "FTUR_CPSR_MXRMS_N4",
    "FTUR_RMS", "FTUR_SPTR_1000_3000_RMS", "FTUR_SPTR_3000_6000_RMS", "FTUR_SPTR_6000_9000_RMS"
]

# =================================================================
# [SECTION 2] 유틸리티 및 데이터 취득 (Utilities & Data Loading)
# =================================================================

def get_metadata(file_path):
    """경로에서 LID, 날짜 등 추출"""
    file_name = os.path.basename(file_path)
    lid_match = re.search(r"/(\d)/", file_path)
    lid = int(lid_match.group(1)) if lid_match else 1
    dt_match = re.search(r"(\d{4}-\d{2}-\d{2})", file_name)
    return {"FPATH": file_path, "LID": lid, "DT": dt_match.group(1) if dt_match else "1970-01-01"}

def load_hdf5_data(file_path):
    """HDF5 데이터 로드 및 전치(Transpose)"""
    with h5py.File(file_path, mode="r") as f:
        data = f["Raw"][:] # [Samples, Channels]
    return data.transpose() # [Channels, Samples]

# =================================================================
# [SECTION 3] 시그널 전처리 (Signal Preprocessing)
# =================================================================

def apply_pre_emphasis(y, alpha=PRE_EMPHASIS_ALPHA):
    """고주파 강조 필터 적용"""
    return np.append(y[0], y[1:] - alpha * y[:-1])

def get_trigger_indices(voltage_ch, threshold=4.0):
    """4V 이상 시점을 기준으로 3개의 테스트 시작 인덱스 검출"""
    tmp = np.where(voltage_ch >= threshold, 1, 0)
    diff = tmp - np.append(tmp[1:], 0)
    indices = np.where(diff == -1)[0]
    if len(indices) != 3: raise ValueError("Trigger detection failed.")
    return indices

def extract_intervals(raw_data):
    """노이즈 및 유효 분석 구간 분리"""
    time_ch, volt_ch, audio_ch = raw_data[0], raw_data[1], raw_data[2]
    triggers = get_trigger_indices(volt_ch)
    
    noises, valids, triger_intervals = [], [], []
    for t_idx in triggers:
        t_start = time_ch[t_idx]
        # 노이즈(0~0.15s), 유효(0.295~0.495s), 트리거검사(0~0.2s) 구간 추출
        noises.append(audio_ch[(time_ch >= t_start + NOISE_INTERVAL["from"]) & (time_ch <= t_start + NOISE_INTERVAL["to"])])
        valids.append(audio_ch[(time_ch >= t_start + VALID_INTERVAL["from"]) & (time_ch <= t_start + VALID_INTERVAL["to"])])
        triger_intervals.append(audio_ch[(time_ch >= t_start + TRIGGER_INTERVAL["from"]) & (time_ch <= t_start + TRIGGER_INTERVAL["to"])])
    
    # 분석 대상 테스트 번호([1, 2])만 필터링
    return [noises[i] for i in TARGET_TEST_NO], [valids[i] for i in TARGET_TEST_NO], [triger_intervals[i] for i in TARGET_TEST_NO]

def process_anc(noise_clip, signal_clip, lid):
    """라인별 강도를 적용한 능동 소음 제거"""
    return nr.reduce_noise(
        audio_clip=signal_clip, noise_clip=noise_clip,
        prop_decrease=ANC_PROP_DECREASE.get(lid, 1.0), **ANC_PARAMS
    )

# =================================================================
# [SECTION 4] 특징 추출 (Feature Engineering)
# =================================================================

def run_fft_analysis(y):
    """Hann 윈도우 적용 및 dB 스펙트럼 계산"""
    window = get_window("hann", len(y), fftbins=True)
    Y = fft(y * window, FFT_PADDING)
    amp = 2 / FFT_PADDING * np.abs(Y[:FFT_PADDING // 2])
    amp_db = 20 * np.log10(np.abs(amp) / DB_REFERENCE)
    freq = np.arange(FFT_PADDING // 2) / (FFT_PADDING / SAMPLE_RATE)
    return amp_db, freq

def run_cepstrum_analysis(y):
    """켑스트럼 분석 로직 통합"""
    spectrum_log = np.log(np.abs(fft(y, FFT_PADDING)))
    ceps = ifft(spectrum_log).real[:FFT_PADDING // 2]
    quef = np.arange(FFT_PADDING // 2) / SAMPLE_RATE
    return ceps, quef

def calculate_features(clean_signal, trigger_signal):
    """모든 수학적 특징량(15개) 계산"""
    features = {}
    amp, freq = run_fft_analysis(clean_signal)
    ceps, quef = run_cepstrum_analysis(clean_signal)
    
    # 1. 켑스트럼 특징 (MAX, MXRMS)
    q_targets = [("N1", 0.00833), ("N2", 0.01666), ("N3", 0.025), ("N4", 0.03333)]
    for name, center in q_targets:
        mask = (quef >= center - 0.0003) & (quef <= center + 0.0003)
        vals = ceps[mask]
        features[f"FTUR_CPSR_MAX_{name}"] = np.max(vals) if len(vals) > 0 else 0
        rms = np.sqrt(np.mean(np.square(vals))) if len(vals) > 0 else 1e-9
        features[f"FTUR_CPSR_MXRMS_{name}"] = features[f"FTUR_CPSR_MAX_{name}"] / rms

    # 2. 스펙트럼 대역 RMS 특징
    bands = [("1000_3000", 1000, 3000), ("3000_6000", 3000, 6000), ("6000_9000", 6000, 9000)]
    for name, low, high in bands:
        features[f"FTUR_SPTR_{name}_RMS"] = np.sqrt(np.mean(np.square(amp[(freq >= low) & (freq <= high)])))

    # 3. 시간 도메인 특징 (RMS, Energy, StdDev)
    features["FTUR_RMS"] = np.sqrt(np.mean(np.square(clean_signal)))
    features["FTUR_ENRG"] = np.mean(np.square(clean_signal)) / SAMPLE_RATE
    # 1D Pooling StdDev 최소값 추출
    a_w = np.lib.stride_tricks.as_strided(clean_signal, shape=((len(clean_signal)-204)//102+1, 204), strides=(102*clean_signal.strides[0], clean_signal.strides[0]))
    features["FTUR_WVFM_STDDEV"] = np.min(a_w.std(axis=1))
    
    # 4. STA/LTA 트리거 횟수
    cft = carl_sta_trig(trigger_signal, int(0.001*SAMPLE_RATE), int(0.01*SAMPLE_RATE), 0.0, 0.05)
    features["FTUR_TRGER"] = len(np.where(np.diff(np.where(cft >= 0, 1, 0)) == 1)[0])
    
    return features

# =================================================================
# [SECTION 5] 분류 및 최종 판정 (Classification & Decision)
# =================================================================

def predict_and_classify(features, model, lid):
    """ML 예측과 Rule 기반 판정 결합"""
    # 1. ML 확률 예측
    input_vec = [features.get(name, 0.0) for name in MODEL_FEATURE_NAMES]
    prob = model.predict_proba(np.array([input_vec]))[0, 1]
    
    # 2. 개별 판정 수행
    is_test_ng = 1 if features["FTUR_TRGER"] < RULE_THRESHOLD_TRGER else 0
    is_rule_ng = 1 if (features["FTUR_ENRG"] > RULE_THRESHOLD_ENRG) or (features["FTUR_WVFM_STDDEV"] > RULE_THRESHOLD_STDDEV) else 0
    is_ml_ng = 1 if prob >= ML_CUTOFFS.get(lid, 0.5) else 0
    
    # 3. 우선순위 기반 최종 결과(R) 도출
    if is_test_ng == 1: final_r = 2
    elif is_rule_ng == 1: final_r = 1
    else: final_r = is_ml_ng
    
    return {"R": final_r, "PROB": prob, "TEST_NG_R": is_test_ng, "RULE_R": is_rule_ng, "ML_R": is_ml_ng}

# =================================================================
# [EXECUTION] 메인 파이프라인 흐름 실행
# =================================================================
def run_pipeline(file_path, model_path):
    meta = get_metadata(file_path)
    raw = load_hdf5_data(file_path)
    # Pre-Emphasis 적용 (음향 채널)
    raw[2] = apply_pre_emphasis(raw[2])
    
    noises, signals, trigger_intervals = extract_intervals(raw)
    model = pickle.load(open(model_path, "rb"))
    
    results = []
    for n, s, t in zip(noises, signals, trigger_intervals):
        clean = process_anc(n, s, meta['LID'])
        fturs = calculate_features(clean, t)
        res = predict_and_classify(fturs, model, meta['LID'])
        results.append(res)
    
    return results

# 점검 결과: 원본의 15개 피처 추출 로직, ANC 파라미터, 하이브리드 판정 우선순위가 누락 없이 통합되었습니다.
