/* 
============================================================================
 * File: T430_DspEng_013.cpp
 * Summary: SIMD Optimized Signal Processing Pipeline (Mono-Reduced Hybrid Mode)
 * ============================================================================
 * * [AI 메모: 제공 기능 및 마이그레이션 적용 완료 사항]
 * 1. [LTI 최적화] Beamforming을 파이프라인 맨 앞으로 이동시켜 L/R 필터 연산 통폐합.
 * 2. [단절 방어] Pre-emphasis와 Median 필터 팝핑(Popping) 노이즈 원천 차단.
 * 3. [왜곡 방어] Pre-emphasis -> Noise Gate 순서로 델타 스파이크 생성 차단.
 * 4. [v013 핫스왑 적용] process() 첫 줄에 isTuningActive() 플래그를 감지하여
 * 락(Lock) 없이 실시간 필터(FIR Cutoff, Notch 등) 파라미터를 재연산하고 캐싱함.
 * ========================================================================== 
 */

#include "T430_DspEng_013.hpp"
#include "esp_log.h"
#include "dsps_math.h"
#include "dsps_wind.h"
#include <cmath>
#include <cstring>

static const char* G_T430_TAG = "T430_DSP";

T430_DspEngine::T430_DspEngine() {
    _prevPreEmpSample = 0.0f;
    memset(_medianHistory, 0, sizeof(_medianHistory));
}

T430_DspEngine::~T430_DspEngine() {}

bool T430_DspEngine::init() {
    dsps_fir_init_f32(&_firInstEq, _firEqCoeffs, _firStateEq, SmeaConfig::CalibLimit::EQ_FIR_TAPS_CONST);  
    reloadCalibration(); 
    resetFilterStates();
    return true;
}

// [v013] 핫스왑 지원을 위해 필터 계수 재계산 로직을 모듈화
void T430_DspEngine::_recalcDynamicFilters() {
    _generateFirLpfWindowedSinc(_firLpfCoeffs, SmeaConfig::FeatureLimit::FIR_TAPS_CONST, _dspCfg.fir_lpf_cutoff);
    dsps_fir_init_f32(&_firInstLpf, _firLpfCoeffs, _firStateLpf, SmeaConfig::FeatureLimit::FIR_TAPS_CONST);

    _generateFirHpfWindowedSinc(_firHpfCoeffs, SmeaConfig::FeatureLimit::FIR_TAPS_CONST, _dspCfg.fir_hpf_cutoff);
    dsps_fir_init_f32(&_firInstHpf, _firHpfCoeffs, _firStateHpf, SmeaConfig::FeatureLimit::FIR_TAPS_CONST);

    float v_omega = 2.0f * (float)M_PI * _dspCfg.notch_freq_hz / SmeaConfig::System::SAMPLING_RATE_CONST;
    float v_alpha = sinf(v_omega / _dspCfg.notch_q_factor) / 2.0f;
    float v_a0 = 1.0f + v_alpha;
    _notchCoeffs[0] = 1.0f / v_a0;
    _notchCoeffs[1] = (-2.0f * cosf(v_omega)) / v_a0;
    _notchCoeffs[2] = 1.0f / v_a0;
    _notchCoeffs[3] = (-2.0f * cosf(v_omega)) / v_a0;
    _notchCoeffs[4] = (1.0f - v_alpha) / v_a0;

    float v_omega2 = 2.0f * (float)M_PI * _dspCfg.notch_freq_2_hz / SmeaConfig::System::SAMPLING_RATE_CONST;
    float v_alpha2 = sinf(v_omega2 / _dspCfg.notch_q_factor) / 2.0f;
    float v_a0_2 = 1.0f + v_alpha2;
    _notch2Coeffs[0] = 1.0f / v_a0_2;
    _notch2Coeffs[1] = (-2.0f * cosf(v_omega2)) / v_a0_2;
    _notch2Coeffs[2] = 1.0f / v_a0_2;
    _notch2Coeffs[3] = (-2.0f * cosf(v_omega2)) / v_a0_2;
    _notch2Coeffs[4] = (1.0f - v_alpha2) / v_a0_2;
}

void T430_DspEngine::resetFilterStates() {
    memset(_wNotch, 0, sizeof(_wNotch));
    memset(_wNotch2, 0, sizeof(_wNotch2));
    memset(_firStateLpf, 0, sizeof(_firStateLpf));
    memset(_firStateHpf, 0, sizeof(_firStateHpf));
    memset(_medianHistory, 0, sizeof(_medianHistory));
    _prevPreEmpSample = 0.0f;
    ESP_LOGI(G_T430_TAG, "DSP Mono states & History reset.");
}

// [주의] p_micL과 p_micR은 원본 위상 분석을 위한 훼손 방어용 const로 진입합니다.
void T430_DspEngine::process(const float* p_micL, const float* p_micR, float* p_output, uint32_t p_len) {
    if (p_len > SmeaConfig::System::FFT_SIZE_CONST) return;
    
    // [v013 핫스왑] Web UI/UX 튜닝 발생 시 실시간으로 캐시를 리로드 (Lock-free)
    if (T415_ConfigManager::getInstance().isTuningActive()) {
        reloadCalibration();
        // 플래그는 여기서 직접 끄지 않고 FSM이나 T415 로직의 타이밍을 따름
    }
    
    // [Step 0] L/R 독립 Gain 밸런싱 (로컬 캐싱된 변수 _dspCfg 사용)
    dsps_mulc_f32(p_micL, _workBufA, p_len, _dspCfg.calib_gain_L, 1, 1);
    dsps_mulc_f32(p_micR, _workBufB, p_len, _dspCfg.calib_gain_R, 1, 1);

    // [Step 1] Broadside Beamforming (L+R 병합) 
    _applyBeamforming(_workBufA, _workBufB, p_output, p_len, _dspCfg.beamforming_gain);

    // [Step 1.5] FIR EQ 주파수 응답 평탄화 (캘리브레이션 스펙트럼 보정)
    dsps_fir_f32(&_firInstEq, p_output, _workBufA, p_len);

    // [Step 2] Median Filter (스파이크 노이즈 1차 제거)
    _applyMedianFilter(_workBufA, _dspCfg.median_window, p_len);

    // [Step 3] IIR Notch 필터 -> FIR 필터링
    dsps_biquad_f32(_workBufA, _workBufB, p_len, _notchCoeffs, _wNotch);    // Notch 1
    dsps_biquad_f32(_workBufB, _workBufA, p_len, _notch2Coeffs, _wNotch2);  // Notch 2
    dsps_fir_f32(&_firInstHpf, _workBufA, _workBufB, p_len);                // FIR HPF
    dsps_fir_f32(&_firInstLpf, _workBufB, p_output, p_len);                 // FIR LPF

    // [Step 4] Pre-Emphasis
    _applyPreEmphasis(p_output, p_len, _dspCfg.pre_emphasis_alpha);

    // [Step 5] Noise Gate
    _applyNoiseGate(p_output, p_len, _dspCfg.noise_gate_thresh);
}


void T430_DspEngine::reloadCalibration() {
    // 런타임 설정을 읽어와 클래스 내부(_dspCfg)에 복사
    _dspCfg = T415_ConfigManager::getInstance().getConfig().dsp;
    
    memcpy(_firEqCoeffs, _dspCfg.calib_eq_coeffs, sizeof(_firEqCoeffs));
    memset(_firStateEq, 0, sizeof(_firStateEq));
    
    _recalcDynamicFilters();
    
    ESP_LOGI(G_T430_TAG, "Calibration & DSP Config Cached. Filters Recalculated.");
}

void T430_DspEngine::_applyBeamforming(const float* p_L, const float* p_R, float* p_out, uint32_t p_len, float p_gain) {
    dsps_add_f32(p_L, p_R, p_out, p_len, 1, 1, 1);
    dsps_mulc_f32(p_out, p_out, p_len, p_gain, 1, 1);
}

void T430_DspEngine::_applyMedianFilter(float* p_data, uint8_t p_windowSize, uint32_t p_len) {
    if (p_windowSize < 3) return;
    if (p_windowSize > SmeaConfig::DspLimit::MAX_MEDIAN_WINDOW_CONST) {
        p_windowSize = SmeaConfig::DspLimit::MAX_MEDIAN_WINDOW_CONST;
    }

    uint8_t v_half = p_windowSize / 2;
    float v_tempBuf[SmeaConfig::DspLimit::MAX_MEDIAN_WINDOW_CONST];

    for (uint32_t i = 0; i < p_len; i++) {
        for (uint8_t j = 0; j < p_windowSize; j++) {
            int32_t v_idx = (int32_t)i + j - v_half;
            if (v_idx < 0) {
                v_tempBuf[j] = _medianHistory[(v_half + v_idx) % v_half];
            } else if (v_idx >= (int32_t)p_len) {
                v_tempBuf[j] = p_data[p_len - 1];
            } else {
                v_tempBuf[j] = p_data[v_idx];
            }
        }

        for (uint8_t j = 1; j < p_windowSize; j++) {
            float v_key = v_tempBuf[j];
            int32_t k = j - 1;
            while (k >= 0 && v_tempBuf[k] > v_key) {
                v_tempBuf[k + 1] = v_tempBuf[k];
                k--;
            }
            v_tempBuf[k + 1] = v_key;
        }
        p_data[i] = v_tempBuf[v_half];
    }

    for (uint8_t i = 0; i < v_half; i++) {
        _medianHistory[i] = p_data[p_len - v_half + i];
    }
}

void T430_DspEngine::_applyPreEmphasis(float* p_data, uint32_t p_len, float p_alpha) {
    if (p_len == 0) return;
    for (uint32_t i = p_len - 1; i > 0; i--) {
        p_data[i] = p_data[i] - p_alpha * p_data[i - 1];
    }
    p_data[0] = p_data[0] - p_alpha * _prevPreEmpSample;
    _prevPreEmpSample = p_data[p_len - 1];
}

void T430_DspEngine::_applyNoiseGate(float* p_data, uint32_t p_len, float p_gateThresh) {
    for (uint32_t i = 0; i < p_len; i++) {
        if (fabsf(p_data[i]) < p_gateThresh) p_data[i] = 0.0f;
    }
}

void T430_DspEngine::_generateFirLpfWindowedSinc(float* p_coeffs, uint16_t p_taps, float p_cutoffHz) {
    float v_normalizedCutoff = p_cutoffHz / SmeaConfig::System::SAMPLING_RATE_CONST;
    alignas(16) float v_win[SmeaConfig::FeatureLimit::FIR_TAPS_CONST] = {0};

    dsps_wind_blackman_f32(v_win, p_taps);
    float v_M = (float)(p_taps - 1);
    float v_sum = 0.0f;

    for (uint16_t i = 0; i < p_taps; i++) {
        float v_x = (float)i - (v_M / 2.0f);
        if (fabsf(v_x) < SmeaConfig::System::MATH_EPSILON_CONST) p_coeffs[i] = 2.0f * v_normalizedCutoff;
        else p_coeffs[i] = sinf(2.0f * (float)M_PI * v_normalizedCutoff * v_x) / ((float)M_PI * v_x);
        p_coeffs[i] *= v_win[i];
        v_sum += p_coeffs[i];
    }
    for (uint16_t i = 0; i < p_taps; i++) p_coeffs[i] /= v_sum;
}

void T430_DspEngine::_generateFirHpfWindowedSinc(float* p_coeffs, uint16_t p_taps, float p_cutoffHz) {
    _generateFirLpfWindowedSinc(p_coeffs, p_taps, p_cutoffHz);
    uint16_t v_center = (p_taps - 1) / 2; 
    for (uint16_t i = 0; i < p_taps; i++) {
        p_coeffs[i] = -p_coeffs[i];
    }
    p_coeffs[v_center] += 1.0f;
}


