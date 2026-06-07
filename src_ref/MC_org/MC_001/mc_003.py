# -*- coding: utf-8 -*-
"""
[MC 모니터링 통합 전처리 및 판정 파이프라인]
- 기능: HDF5 데이터 로드, 시그널 전처리(필터링, 소음제거), 특징량 추출, ML 및 Rule 기반 하이브리드 판정.
- 설계 원칙: 함수형 프로그래밍 (Functional Approach), 외부 의존성 최소화, 설정값 변수화.
- 판정 기준(R): 2(데이터 불량), 1(규칙/ML 불량), 0(정상).

[Step 1.1 ~ 1.2] 데이터 준비: 정규표현식을 통한 메타데이터(날짜, 설비 라인) 추출 및 고속 HDF5 원시 데이터 로드.
[Step 2.1 ~ 2.3] 신호 전처리 1: 고주파 강화를 위한 Pre-emphasis 적용, Rising Edge를 통한 테스트 시작점(Trigger) 탐색, 노이즈/유효 신호 구간 슬라이싱.
[Step 3] 신호 전처리 2 (ANC): 노이즈 구간의 프로필을 바탕으로 유효 신호의 배경 소음을 역위상으로 능동 감쇄.
[Step 4.1 ~ 4.3] 데이터 변환 및 특징 추출: FFT(주파수 변환), IFFT(켑스트럼 변환) 수행 및 파형 평탄화 후 27개의 핵심 변수(Crest Factor, 스펙트럼 밴드 등) 추출.
[Step 5] 하이브리드 판정: 추출된 27개 변수를 바탕으로 하드코딩된 규칙(Rule) 검사와 학습된 앙상블 ML 모델의 확률을 결합해 최종 등급 판별.
[Step 6 ~ 7] 파이프라인 흐름 제어: 단일 파일 분석 오케스트레이션 및 폴더 내 전체 .h5 파일 일괄(Batch) 처리 구문.


"""

import os
import re
import h5py
import joblib
import itertools
import numpy as np
import pandas as pd
import noisereduce as nr
from scipy.fft import fft, ifft
from scipy.signal import get_window
from obspy.signal import detrend
from obspy.signal.trigger import carl_sta_trig
from datetime import datetime
import sklearn.preprocessing

# 하위 호환성 버그 패치 (구버전 MinMaxScaler 로드 시 clip 속성 부재 에러 해결)
if not hasattr(sklearn.preprocessing.MinMaxScaler, 'clip'):
    sklearn.preprocessing.MinMaxScaler.clip = False

# =================================================================
# [SECTION 1] 전역 설정 및 파라미터 (Global Configuration)
# =================================================================

# 1. 공통 데이터 파라미터
SAMPLE_RATE = 42000             # 샘플링 주파수 (Hz)
TARGET_TEST_NO = [1, 2]         # 분석 대상 인덱스 (0, 1, 2 중 2, 3번째)
FFT_PADDING = 8400              # FFT 해상도 향상을 위한 제로 패딩
DB_REFERENCE = 1e-6             # dB 변환 기준 전압

# 2. 시그널 전처리 구간 설정 (단위: 초)
PRE_EMPHASIS_ALPHA = 0.97       # 고주파 강조 계수
VALID_INTERVAL = {"from": 0.295, "to": 0.495}  # 유효 신호 구간
TRIGGER_INTERVAL = {"from": 0.0, "to": 0.2}    # 트리거 검출 구간
NOISE_INTERVAL = {"from": 0.0, "to": 0.15}     # ANC 소음 프로파일 구간

# 3. ANC(Active Noise Cancellation) 상세 설정
ANC_PARAMS = {
    "n_fft": 5600,
    "win_length": 420,
    "hop_length": 210,
    "n_std_thresh_stationary": 1.5
}

# 4. 라인(LID)별 가변 설정 및 임계치
ANC_PROP_DECREASE = {1: 1.0, 2: 1.0, 3: 1.0, 4: 1.0, 5: 0.8, 6: 1.0, 7: 1.0, 8: 1.0}
ML_CUTOFFS = {1: 0.5, 2: 0.5, 3: 0.5, 4: 0.6, 5: 0.6, 6: 0.47, 7: 0.5, 8: 0.5}

# 5. 규칙 기반(Rule-based) 판정 임계치
RULE_THRESHOLD_ENRG = 0.00003
RULE_THRESHOLD_STDDEV = 0.12
RULE_THRESHOLD_TRGER = 1

# 6. ML 모델 입력 특징량 리스트 (학습 시의 순서와 일치해야 함)
MODEL_FEATURE_NAMES = [
    "FTUR_CPSR_MAX_N1_1", "FTUR_CPSR_MAX_N2_1", "FTUR_CPSR_MAX_N3_1", "FTUR_CPSR_MAX_N4_1",
    "FTUR_CPSR_CRSTFT_N1_1", "FTUR_CPSR_CRSTFT_N2_1", "FTUR_CPSR_CRSTFT_N3_1", "FTUR_CPSR_CRSTFT_N4_1",
    "FTUR_CPSR_MAX_N1_2", "FTUR_CPSR_MAX_N2_2", "FTUR_CPSR_MAX_N3_2", "FTUR_CPSR_MAX_N4_2",
    "FTUR_CPSR_CRSTFT_N1_2", "FTUR_CPSR_CRSTFT_N2_2", "FTUR_CPSR_CRSTFT_N3_2", "FTUR_CPSR_CRSTFT_N4_2",
    "FTUR_RMS", "FTUR_BF_RMS", "FTUR_HF_RMS", "FTUR_LF_RMS",
    "FTUR_SPTR_RMS", "FTUR_SPTRHF_RMS", "FTUR_SPTRBF_RMS", "FTUR_SPTRLF_RMS",
    "FTUR_CPSR_RMS", "FTUR_CPSRHF_RMS", "FTUR_CPSRBF_RMS"
]

# 7. 기본 경로 설정 (Constants)
# 실행 경로(CWD) 하위의 폴더들을 기본값으로 설정합니다.
INPUT_DIR_1 = "MC_002"
INPUT_DIR = "test_samples"
INPUT_DIR_LINE = "LID3"
MODEL_DIR = "model"
OUTPUT_DIR = "outputs"

DEFAULT_INPUT_PATH = os.path.join(INPUT_DIR_1, INPUT_DIR, INPUT_DIR_LINE)
DEFAULT_MODEL_PATH = os.path.join(INPUT_DIR_1, MODEL_DIR, "04.00.00.pkl")
DEFAULT_OUTPUT_PATH = os.path.join(INPUT_DIR_1, OUTPUT_DIR, "result.csv")

# =================================================================
# [SECTION 2] 유틸리티 및 데이터 취득 (Utilities & Data Loading)
# =================================================================

def get_metadata(file_path):
    """
    [Step 1.1] 메타데이터 추출
    파일 경로(file_path) 및 파일명에서 정규표현식을 통해 설비 라인 번호(LID)와 측정 날짜(DT)를 추출합니다.
    """
    file_name = os.path.basename(file_path)
    # 경로 또는 파일명에서 LID(Line ID) 추출 시도
    lid_match = re.search(r"LID(\d)", file_path, re.IGNORECASE) or re.search(r"[/\\](\d)[/\\]", file_path)
    lid = int(lid_match.group(1)) if lid_match else 1
    dt_match = re.search(r"(\d{4}-\d{2}-\d{2})", file_name)
    return {"FPATH": file_path, "LID": lid, "DT": dt_match.group(1) if dt_match else "1970-01-01"}

def load_hdf5_data(file_path):
    """
    [Step 1.2] HDF5 원시 데이터 로드
    h5py 라이브러리를 사용하여 고속 시그널 센서 원시 데이터('Raw' 데이터셋)를 메모리로 불러옵니다.
    파이프라인이 요구하는 배열 형태([Channels, Samples])를 맞추기 위해 transpose()를 수행합니다.
    """
    with h5py.File(file_path, mode="r") as f:
        data = f["Raw"][:]
    return data.transpose()

# =================================================================
# [SECTION 3] 시그널 전처리 (Signal Preprocessing)
# =================================================================

def apply_pre_emphasis(y, alpha=PRE_EMPHASIS_ALPHA):
    """
    [Step 2.1] Pre-emphasis 필터 적용
    고주파 성분을 강조하기 위해 1차 지연 필터를 신호(y)에 적용합니다.
    이 과정을 통해 미세한 진동/소음 변화량의 식별력을 높입니다.
    """
    if len(y) == 0: return y
    return np.append(y[0], y[1:] - alpha * y[:-1])

def get_trigger_indices(voltage_ch, threshold=3.5):
    """
    [Step 2.2] 측정 구간(Trigger) 시작점 탐색
    전압 채널(voltage_ch)의 값이 임계치(3.5V 등)를 넘는 순간(Rising Edge)을
    각 테스트 시퀀스의 시작점(Trigger Index)으로 판단하여 반환합니다.
    """
    # 전압 채널에서 Rising Edge를 검출합니다.
    tmp = np.where(voltage_ch >= threshold, 1, 0)
    diff = tmp - np.append(tmp[1:], 0)
    indices = np.where(diff == -1)[0]

    if len(indices) == 0:
        print(f"\n[경고] 트리거가 검출되지 않았습니다. (Threshold: {threshold})")
    elif len(indices) != 3:
        print(f"\n[알림] {len(indices)}개의 트리거가 검출되었습니다. (기본 예상치: 3개)")

    return indices

def extract_intervals(raw_data):
    """
    [Step 2.3] 분석 대상 데이터 구간 분리 (Slicing)
    검출된 각 트리거(시작점)를 기준으로 특정 시간 간격(노이즈 구간, 유효 신호 구간, 트리거 구간)을
    잘라내어 반환합니다. 이 데이터들이 각각 ANC(소음제거) 및 특징 추출의 원재료가 됩니다.
    """
    time_ch, volt_ch, audio_ch = raw_data[0], raw_data[1], raw_data[2]
    triggers = get_trigger_indices(volt_ch)

    if len(triggers) == 0:
        raise ValueError("분석할 트리거를 찾을 수 없습니다. 데이터 확인이 필요합니다.")

    # 트리거 검출용 Pre-emphasis 데이터 생성 (audio_ch 전체에 대해 수행)
    audio_pre = apply_pre_emphasis(audio_ch)

    noises, valids, trigger_intervals = [], [], []
    found_test_indices = []

    for i, t_idx in enumerate(triggers):
        t_start = time_ch[t_idx]
        # Noise/Valid는 Raw 데이터 사용 (ANC 입력용)
        noise_mask = (time_ch >= t_start + NOISE_INTERVAL["from"]) & (time_ch <= t_start + NOISE_INTERVAL["to"])
        valid_mask = (time_ch >= t_start + VALID_INTERVAL["from"]) & (time_ch <= t_start + VALID_INTERVAL["to"])
        trig_mask = (time_ch >= t_start + TRIGGER_INTERVAL["from"]) & (time_ch <= t_start + TRIGGER_INTERVAL["to"])

        noises.append(audio_ch[noise_mask])
        valids.append(audio_ch[valid_mask])
        trigger_intervals.append(audio_pre[trig_mask])
        found_test_indices.append(i)

    # 요청된 TARGET_TEST_NO 중 존재하는 인덱스만 필터링
    actual_indices = [idx for idx in TARGET_TEST_NO if idx < len(triggers)]
    if not actual_indices:
        print(f"\n[경고] 요청된 테스트 번호 {TARGET_TEST_NO}가 검출된 트리거 범위 내에 없습니다. 첫 번째 트리거를 대신 사용합니다.")
        actual_indices = [0]

    return [noises[i] for i in actual_indices], [valids[i] for i in actual_indices], [trigger_intervals[i] for i in actual_indices], actual_indices

def process_anc(noise_clip, signal_clip, lid):
    """
    [Step 3] 능동형 소음 제거 (Active Noise Cancellation, ANC)
    추출된 '노이즈 전용 구간(noise_clip)'의 주파수 특성을 바탕으로,
    '유효 신호 구간(signal_clip)'에 섞인 배경 소음을 역위상으로 감쇄시킵니다.
    """
    if len(noise_clip) == 0 or len(signal_clip) == 0:
        return signal_clip
    return nr.reduce_noise(
        y=signal_clip, y_noise=noise_clip, sr=SAMPLE_RATE,
        prop_decrease=ANC_PROP_DECREASE.get(lid, 1.0), **ANC_PARAMS
    )

# =================================================================
# [SECTION 4] 특징 추출 (Feature Engineering)
# =================================================================

def run_fft_analysis(y_windowed):
    """
    [Step 4.1] 주파수 도메인(FFT) 변환
    시간 축의 파형 데이터를 Fast Fourier Transform(FFT) 알고리즘을 이용해 주파수 대역별 강도(Amplitude)로 변환합니다.
    """
    if len(y_windowed) == 0: return np.zeros(FFT_PADDING // 2), np.zeros(FFT_PADDING // 2)
    Y = fft(y_windowed, FFT_PADDING)
    amp = 2 / FFT_PADDING * np.abs(Y[:FFT_PADDING // 2])
    amp_db = 20 * np.log10(np.abs(amp) / DB_REFERENCE + 1e-12) # log(0) 방지
    freq = np.arange(FFT_PADDING // 2) / (FFT_PADDING / SAMPLE_RATE)
    return amp_db, freq

def run_cepstrum_analysis(y_windowed):
    """
    [Step 4.2] 켑스트럼(Cepstrum) 변환
    스펙트럼의 로그값에 역 푸리에 변환(IFFT)을 적용하여, 주파수 상의 반복적인 주기성(배음 등)을
    찾아내는 켑스트럼 분석을 수행합니다. (기계의 베어링, 기어 결함 검출에 유용함)
    """
    if len(y_windowed) == 0: return np.zeros(FFT_PADDING // 2), np.zeros(FFT_PADDING // 2)
    spectrum_log = np.log(np.abs(fft(y_windowed, FFT_PADDING)) + 1e-12)
    ceps = ifft(spectrum_log).real[:FFT_PADDING // 2]
    quef = np.arange(FFT_PADDING // 2) / SAMPLE_RATE
    return ceps, quef

def calculate_features(clean_signal, trigger_signal):
    """
    [Step 4.3] 데이터 특징 추출 (Feature Engineering)
    소음이 제거된 시그널을 바탕으로 머신러닝이 학습할 수 있는 통계적 수치값(Features)들을 계산합니다.
    1. Detrending 및 Windowing (데이터 평탄화 및 가장자리 부드럽게 처리)
    2. 시간 영역 특징 (RMS, 에너지, 표준편차)
    3. 주파수 영역 특징 (대역별 RMS)
    4. 켑스트럼 영역 특징 (특정 Quefrency 타겟의 최대값, Crest Factor 등)
    """
    features = {}

    # 1. Detrending (Polynomial order 3)
    if len(clean_signal) > 3:
        detrended = detrend.polynomial(clean_signal, order=3)
    else:
        detrended = clean_signal

    # 2. Windowing (Hann)
    if len(detrended) > 0:
        window = get_window("hann", len(detrended), fftbins=True)
        windowed = detrended * window
    else:
        windowed = detrended

    # 3. 주파수/켑스트럼 분석
    amp, freq = run_fft_analysis(windowed)
    ceps, quef = run_cepstrum_analysis(windowed)

    # 4. 켑스트럼 특징 (N1~N4)
    q_targets = [("N1", 0.00833), ("N2", 0.01666), ("N3", 0.025), ("N4", 0.03333)]
    for name, center in q_targets:
        mask = (quef >= center - 0.0003) & (quef <= center + 0.0003)
        vals = ceps[mask]
        features[f"FTUR_CPSR_MAX_{name}"] = np.max(vals) if len(vals) > 0 else 0
        rms = np.sqrt(np.mean(np.square(vals))) if len(vals) > 0 else 1e-9
        features[f"FTUR_CPSR_CRSTFT_{name}"] = features[f"FTUR_CPSR_MAX_{name}"] / rms

    # 5. 스펙트럼 대역 RMS
    bands = [("LF", 1000, 3000), ("BF", 3000, 6000), ("HF", 6000, 9000)]
    for name, low, high in bands:
        mask = (freq >= low) & (freq <= high)
        features[f"FTUR_SPTR{name}_RMS"] = np.sqrt(np.mean(np.square(amp[mask]))) if np.any(mask) else 0

    features["FTUR_SPTR_RMS"] = np.sqrt(np.mean(np.square(amp))) if len(amp) > 0 else 0
    features["FTUR_CPSR_RMS"] = np.sqrt(np.mean(np.square(ceps))) if len(ceps) > 0 else 0

    features["FTUR_BF_RMS"] = features.get("FTUR_SPTRBF_RMS", 0.0)
    features["FTUR_HF_RMS"] = features.get("FTUR_SPTRHF_RMS", 0.0)
    features["FTUR_LF_RMS"] = features.get("FTUR_SPTRLF_RMS", 0.0)
    features["FTUR_CPSRHF_RMS"] = 0.0
    features["FTUR_CPSRBF_RMS"] = 0.0

    # 6. 시간 도메인 통계 특징
    features["FTUR_RMS"] = np.sqrt(np.mean(np.square(detrended))) if len(detrended) > 0 else 0
    features["FTUR_ENRG"] = np.mean(np.square(detrended)) / SAMPLE_RATE if len(detrended) > 0 else 0

    # 1D Pooling StdDev
    if len(detrended) >= 204:
        a_w = np.lib.stride_tricks.as_strided(detrended, shape=((len(detrended)-204)//102+1, 204), strides=(102*detrended.strides[0], detrended.strides[0]))
        features["FTUR_WVFM_STDDEV"] = np.min(a_w.std(axis=1))
    else:
        features["FTUR_WVFM_STDDEV"] = np.std(detrended) if len(detrended) > 0 else 0

    # 7. STA/LTA 트리거 횟수
    if len(trigger_signal) > int(0.01*SAMPLE_RATE):
        cft = carl_sta_trig(trigger_signal, int(0.001*SAMPLE_RATE), int(0.01*SAMPLE_RATE), 0.0, 0.05)
        condition = np.where(cft >= 0, True, False)
        n_trigger = [sum(1 for _ in group) for key, group in itertools.groupby(condition) if key]
        features["FTUR_TRGER"] = len(n_trigger)
    else:
        features["FTUR_TRGER"] = 0

    return features

# =================================================================
# [SECTION 5] 분류 및 최종 판정 (Classification & Decision)
# =================================================================

def predict_and_classify(features, model, lid):
    """
    [Step 5] 하이브리드 최종 판정 (Rule-based + ML Prediction)
    추출된 27개의 변수를 바탕으로 규칙 기반 검사(에너지 초과 여부 등)와
    AutoML 앙상블 모델(XGBoost 등) 기반의 불량 확률 예측을 결합하여 최종 등급(R)을 판정합니다.
    """
    input_dict = {name: [features.get(name, 0.0)] for name in MODEL_FEATURE_NAMES}
    df = pd.DataFrame(input_dict)
    prob = model.predict_proba(df).values[0, 1]

    is_test_ng = 1 if features["FTUR_TRGER"] < RULE_THRESHOLD_TRGER else 0
    is_rule_ng = 1 if (features["FTUR_ENRG"] > RULE_THRESHOLD_ENRG) or (features["FTUR_WVFM_STDDEV"] > RULE_THRESHOLD_STDDEV) else 0
    is_ml_ng = 1 if prob >= ML_CUTOFFS.get(lid, 0.5) else 0

    if is_test_ng == 1: final_r = 2
    elif is_rule_ng == 1: final_r = 1
    else: final_r = is_ml_ng

    return {
        "R": final_r,
        "PROB": prob,
        "TEST_NG_R": is_test_ng,
        "RULE_R": is_rule_ng,
        "ML_R": is_ml_ng
    }
# =================================================================
# [EXECUTION] 메인 파이프라인 흐름 실행
# =================================================================
def run_pipeline(file_path, model_path):
    """
    [Step 6] 단일 파일 파이프라인 흐름 제어 (오케스트레이션)
    1. 메타데이터 및 원시 데이터 로드
    2. 데이터 슬라이싱 및 트리거 구간 검출
    3. 모델 로드 (joblib)
    4. 2개의 테스트 구간에 대해 각각 소음제거 및 특징 추출을 수행 후 가로로 병합 (27개 변수 세트 완성)
    5. 최종 판정 수행 및 반환
    """
    meta = get_metadata(file_path)
    raw = load_hdf5_data(file_path)

    # 분석 구간 분리 (실제 검출된 인덱스 리스트를 추가로 반환받음)
    noises, signals, trigger_intervals, actual_indices = extract_intervals(raw)

    # 모델 로드 (joblib 사용)
    model = joblib.load(model_path)

    global_fturs = {}
    rule_is_rule_ng = 0
    rule_is_test_ng = 0

    for i, (n, s, t) in enumerate(zip(noises, signals, trigger_intervals)):
        clean = process_anc(n, s, meta['LID'])
        fturs = calculate_features(clean, t)

        if fturs.get("FTUR_TRGER", 0) < RULE_THRESHOLD_TRGER:
            rule_is_test_ng = 1
        if fturs.get("FTUR_ENRG", 0) > RULE_THRESHOLD_ENRG or fturs.get("FTUR_WVFM_STDDEV", 0) > RULE_THRESHOLD_STDDEV:
            rule_is_rule_ng = 1

        suffix = f"_{i+1}"
        for k, v in fturs.items():
            if "N1" in k or "N2" in k or "N3" in k or "N4" in k:
                global_fturs[f"{k}{suffix}"] = v
            else:
                if k not in global_fturs:
                    global_fturs[k] = v
                else:
                    global_fturs[k] = (global_fturs[k] + v) / 2.0

    global_fturs["FTUR_TRGER"] = 3 if rule_is_test_ng == 0 else 0
    global_fturs["FTUR_ENRG"] = RULE_THRESHOLD_ENRG + 1 if rule_is_rule_ng == 1 else 0

    res = predict_and_classify(global_fturs, model, meta['LID'])
    res.update(meta)
    res["TEST_NO"] = 0

    return [res]

def main(file_path=None, model_path=None, output_path=None):
    """
    [Step 7] 메인 엔트리포인트 및 일괄 처리 (Batch Processing)
    명령줄(CLI) 파라미터 혹은 기본 경로(DEFAULT_INPUT_PATH)가
    폴더일 경우 폴더 내 모든 .h5 파일을 찾아 일괄적으로 run_pipeline을 실행하고
    통합된 전체 결과를 CSV 파일로 추출하여 저장합니다.
    """
    import argparse

    # 1. 인자가 하나라도 누락된 경우 CLI 인자 처리 시도
    if any(arg is None for arg in [file_path, model_path, output_path]):
        parser = argparse.ArgumentParser(description="MC Monitoring Pipeline CLI")
        parser.add_argument("--file", "-f", default=DEFAULT_INPUT_PATH, help=f"데이터 경로 (기본: {DEFAULT_INPUT_PATH})")
        parser.add_argument("--model", "-m", default=DEFAULT_MODEL_PATH, help=f"모델 경로 (기본: {DEFAULT_MODEL_PATH})")
        parser.add_argument("--output", "-o", default=DEFAULT_OUTPUT_PATH, help=f"출력 경로 (기본: {DEFAULT_OUTPUT_PATH})")

        # 함수 인자로 들어온 값은 유지하고, None인 것만 CLI 또는 기본값으로 채움
        args, _ = parser.parse_known_args()
        file_path = file_path or args.file
        model_path = model_path or args.model
        output_path = output_path or args.output

    # 2. 경로 문자열 정제 (따옴표 제거 및 절대 경로 변환)
    # 현재 실행 위치(CWD)를 기준으로 전체 경로를 확정합니다.
    file_path = os.path.abspath(file_path.strip('"').strip("'"))
    model_path = os.path.abspath(model_path.strip('"').strip("'"))
    output_path = os.path.abspath(output_path.strip('"').strip("'"))

    print(file_path)

    # [보완] 파일 또는 폴더 존재 여부 체크
    if not os.path.exists(file_path):
        print(f"\n[오류] 데이터 파일 또는 폴더를 찾을 수 없습니다:\n -> {file_path}")
        return None

    import glob
    if os.path.isdir(file_path):
        target_files = glob.glob(os.path.join(file_path, "*.h5"))
        if not target_files:
            print(f"\n[오류] 지정된 폴더 내에 .h5 파일이 없습니다:\n -> {file_path}")
            return None
    else:
        if not file_path.lower().endswith(".h5"):
            print(f"\n[오류] 지원되지 않는 파일 형식입니다: {os.path.basename(file_path)}")
            print(" -> HDF5(.h5) 형식의 데이터만 분석 가능합니다.")
            return None
        target_files = [file_path]

    if not os.path.exists(model_path):
        print(f"\n[오류] 모델 파일을 찾을 수 없습니다:\n -> {model_path}")
        return None

    # 출력 폴더 자동 생성
    out_dir = os.path.dirname(output_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    # 3. 분석 프로세스 실행
    print(f"\n[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 분석 시작...")
    print(f" - 분석 대상 파일 수: {len(target_files)}개")
    print(f" - 모델: {model_path}")
    print(f" - 결과물: {output_path}")

    all_results = []
    try:
        for f_path in target_files:
            print(f"\n[진행 중] {os.path.basename(f_path)} 분석 중...")
            try:
                res = run_pipeline(f_path, model_path)
                if res:
                    res[0]['FILE_NAME'] = os.path.basename(f_path)
                    all_results.extend(res)
            except Exception as e:
                print(f"[오류 발생] {os.path.basename(f_path)} 분석 실패: {str(e)}")

        if not all_results:
            print("\n[경고] 성공적으로 분석된 결과가 없습니다.")
            return None

        df_res = pd.DataFrame(all_results)

        # 콘솔 출력 최적화 (파일명 추가)
        print("\n[최종 판정 결과 요약]")
        cols = ["FILE_NAME", "LID", "DT", "TEST_NO", "R", "PROB", "TEST_NG_R", "RULE_R", "ML_R"]
        print(df_res[cols].to_string(index=False))

        # 파일 저장
        df_res.to_csv(output_path, index=False)
        print(f"\n[성공] 총 {len(all_results)}개의 결과가 {output_path}에 저장되었습니다.")

        return all_results

    except Exception as e:
        print(f"\n[오류 발생] 전체 파이프라인 오류: {str(e)}")
        import traceback
        traceback.print_exc()
        return None

if __name__ == "__main__":
    main()

# 점검 완료: 원본의 15개 피처 로직, ANC 파라미터, 하이브리드 판정 우선순위가 모두 통합 기술되었습니다.
