/* ============================================================================
 * File: T230_DspEng_241.cpp
 * Summary: Hybrid DSP Engine Implementation
 * ============================================================================
 * [구현 상세] LTI (Linear Time-Invariant) 최적화
 * L채널과 R채널을 각각 필터링하면 연산량이 2배입니다. 빔포밍으로 L과 R을 합친 후
 * 필터링을 1회만 수행하여 배터리 소모와 발열을 절반으로 낮춥니다.
 * ========================================================================== */

#include "T230_DspEng_241.hpp"
#include "T215_ConfigMgr_241.hpp"

#include <cmath>
#include <cstring>

CL_T2_DspEngine::CL_T2_DspEngine() {
    _isInitialized = false;
}

bool CL_T2_DspEngine::init() {
    if (_isInitialized) return true;

    // Hanning 윈도우 사전 생성 (반복 연산 최소화)
    dsps_wind_hann_f32(_audWindow, T2_Config::System::AUD_FFT_SIZE_CONST);
    dsps_wind_hann_f32(_vibWindow, T2_Config::System::VIB_FFT_SIZE_CONST);

    reloadFilters();
    resetStates();

    _isInitialized = true;
    return true;
}


void CL_T2_DspEngine::reloadFilters() {
    T2_Type::DynamicConfig v_cfg = CL_T2_ConfigManager::getInstance().getConfig();

    // 1. Notch 필터 갱신
    _calcNotchCoeffs(v_cfg.dsp.notch.target_freq_hz, v_cfg.dsp.notch.q_factor, _notch60Coeffs, T2_Config::System::AUD_SAMPLING_RATE_CONST);

    // 2. FIR 필터 계수 동적 갱신
    _generateFirLpfWindowedSinc(_firLpfAudioCoeffs, T2_Config::FeatureLimit::FIR_TAPS_CONST, v_cfg.dsp.fir_lpf.cutoff_hz, (float)T2_Config::System::AUD_SAMPLING_RATE_CONST);
    dsps_fir_init_f32(&_firInstLpfAudio, _firLpfAudioCoeffs, _firStateLpfAudio, T2_Config::FeatureLimit::FIR_TAPS_CONST);

    _generateFirHpfWindowedSinc(_firHpfAudioCoeffs, T2_Config::FeatureLimit::FIR_TAPS_CONST, v_cfg.dsp.fir_hpf.cutoff_hz, (float)T2_Config::System::AUD_SAMPLING_RATE_CONST);
    dsps_fir_init_f32(&_firInstHpfAudio, _firHpfAudioCoeffs, _firStateHpfAudio, T2_Config::FeatureLimit::FIR_TAPS_CONST);

    // 3. IIR 필터 계수 갱신
    _calcIirHpCoeffs(v_cfg.dsp.iir_hpf.cutoff_hz, v_cfg.dsp.iir_hpf.q_factor, _iirHpfCoeffs, T2_Config::System::AUD_SAMPLING_RATE_CONST);
    _calcIirLpCoeffs(v_cfg.dsp.iir_lpf.cutoff_hz, v_cfg.dsp.iir_lpf.q_factor, _iirLpfCoeffs, T2_Config::System::AUD_SAMPLING_RATE_CONST);
}


void CL_T2_DspEngine::resetStates() {
    memset(_notch60State, 0, sizeof(_notch60State));
    memset(_notch120State, 0, sizeof(_notch120State));
    memset(_firEqState, 0, sizeof(_firEqState));

    // IIR 상태 초기화
    memset(_iirHpfState, 0, sizeof(_iirHpfState));
    memset(_iirLpfState, 0, sizeof(_iirLpfState));

}

void CL_T2_DspEngine::_calcNotchCoeffs(float p_freq, float p_q, float* p_coeffs, uint32_t p_sampleRate) {
    float v_w0 = 2.0f * (float)M_PI * p_freq / (float)p_sampleRate;
    float v_alpha = sinf(v_w0) / (2.0f * p_q);
    float v_b0 = 1.0f;
    float v_b1 = -2.0f * cosf(v_w0);
    float v_b2 = 1.0f;
    float v_a0 = 1.0f + v_alpha;
    float v_a1 = -2.0f * cosf(v_w0);
    float v_a2 = 1.0f - v_alpha;

    // ESP-DSP Biquad 포맷: [b0, b1, b2, a1, a2] (a0로 정규화됨)
    p_coeffs[0] = v_b0 / v_a0;
    p_coeffs[1] = v_b1 / v_a0;
    p_coeffs[2] = v_b2 / v_a0;
    p_coeffs[3] = v_a1 / v_a0;
    p_coeffs[4] = v_a2 / v_a0;
}

void CL_T2_DspEngine::processAcoustic(const float* p_audL, const float* p_audR, float* p_outBeam) {
    T2_Type::DynamicConfig v_cfg = CL_T2_ConfigManager::getInstance().getConfig();
    float v_gainL = v_cfg.dsp.calib_gain_L * v_cfg.dsp.beamforming_gain;
    float v_gainR = v_cfg.dsp.calib_gain_R * v_cfg.dsp.beamforming_gain;

    // 1. LTI 빔포밍 (L+R 병합)
    for (uint32_t i = 0; i < T2_Config::System::AUD_FFT_SIZE_CONST; i++) {
        p_outBeam[i] = (p_audL[i] * v_gainL) + (p_audR[i] * v_gainR);
    }

    // 2. [복원] Median Filter
    if (v_cfg.dsp.median_enabled) {
        _applyMedianFilter(p_outBeam, _medianHistAudio, v_cfg.dsp.median_window, T2_Config::System::AUD_FFT_SIZE_CONST);
    }

    // 3. Notch & FIR 필터링 (핑퐁 버퍼 사용)
    float* v_src = p_outBeam;
    float* v_dst = _audioBufA;

    if (v_cfg.dsp.notch.enabled) {
        dsps_biquad_f32(v_src, v_dst, T2_Config::System::AUD_FFT_SIZE_CONST, _notch60Coeffs, _notch60State);
        v_src = _audioBufA; v_dst = _audioBufB;
    }
    if (v_cfg.dsp.fir_hpf.enabled) {
        dsps_fir_f32(&_firInstHpfAudio, v_src, v_dst, T2_Config::System::AUD_FFT_SIZE_CONST);
        float* temp = v_src; v_src = v_dst; v_dst = temp;
    }
    if (v_cfg.dsp.fir_lpf.enabled) {
        dsps_fir_f32(&_firInstLpfAudio, v_src, v_dst, T2_Config::System::AUD_FFT_SIZE_CONST);
        float* temp = v_src; v_src = v_dst; v_dst = temp;
    }

    // FIR 필터링 ...
    if (v_cfg.dsp.fir_lpf.enabled) {
        dsps_fir_f32(&_firInstLpfAudio, v_src, v_dst, T2_Config::System::AUD_FFT_SIZE_CONST);
        float* temp = v_src; v_src = v_dst; v_dst = temp;
    }

    // IIR (Biquad) 필터링 파이프라인 병합
    if (v_cfg.dsp.iir_hpf.enabled) {
        dsps_biquad_f32(v_src, v_dst, T2_Config::System::AUD_FFT_SIZE_CONST, _iirHpfCoeffs, _iirHpfState);
        float* temp = v_src; v_src = v_dst; v_dst = temp;
    }
    if (v_cfg.dsp.iir_lpf.enabled) {
        dsps_biquad_f32(v_src, v_dst, T2_Config::System::AUD_FFT_SIZE_CONST, _iirLpfCoeffs, _iirLpfState);
        float* temp = v_src; v_src = v_dst; v_dst = temp;
    }


    if (v_src != p_outBeam) memcpy(p_outBeam, v_src, T2_Config::System::AUD_FFT_SIZE_CONST * sizeof(float));

    // 4. [복원] Pre-emphasis & Noise Gate & Windowing
    if (v_cfg.dsp.preemphasis_enable) _applyPreEmphasis(p_outBeam, _prevPreEmpAudio, T2_Config::System::AUD_FFT_SIZE_CONST, v_cfg.dsp.preemphasis_alpha);
    if (v_cfg.dsp.noise.enable_gate) _applyNoiseGate(p_outBeam, T2_Config::System::AUD_FFT_SIZE_CONST, v_cfg.dsp.noise.gate_threshold_abs);

    for (uint32_t i = 0; i < T2_Config::System::AUD_FFT_SIZE_CONST; i++) p_outBeam[i] *= _audWindow[i];
}


void CL_T2_DspEngine::processVibration(const float* p_vibX, const float* p_vibY, const float* p_vibZ,
                                       float* p_outX, float* p_outY, float* p_outZ, uint32_t p_len) {
    float v_meanX = 0, v_meanY = 0, v_meanZ = 0;

    // 1. Mean 계산 (p_len 방어 적용)
    for (uint32_t i = 0; i < p_len; i++) {
        v_meanX += p_vibX[i]; v_meanY += p_vibY[i]; v_meanZ += p_vibZ[i];
    }
    v_meanX /= p_len; v_meanY /= p_len; v_meanZ /= p_len;

    // 2. DC 오프셋 제거 후 윈도윙 (const p_vib에서 읽고 p_out에 쓰기)
    for (uint32_t i = 0; i < p_len; i++) {
        p_outX[i] = (p_vibX[i] - v_meanX) * _vibWindow[i];
        p_outY[i] = (p_vibY[i] - v_meanY) * _vibWindow[i];
        p_outZ[i] = (p_vibZ[i] - v_meanZ) * _vibWindow[i];
    }
}


void CL_T2_DspEngine::_applyMedianFilter(float* p_data, float* p_hist, uint8_t p_windowSize, uint32_t p_len) {
    if (p_windowSize < 3) return;
    uint8_t v_half = p_windowSize / 2;
    float v_tempBuf[16];

    for (uint32_t i = 0; i < p_len; i++) {
        for (uint8_t j = 0; j < p_windowSize; j++) {
            int32_t v_idx = (int32_t)i + j - v_half;
            if (v_idx < 0) v_tempBuf[j] = p_hist[(v_half + v_idx) % v_half];
            else if (v_idx >= (int32_t)p_len) v_tempBuf[j] = p_data[p_len - 1];
            else v_tempBuf[j] = p_data[v_idx];
        }
        for (uint8_t j = 1; j < p_windowSize; j++) {
            float v_key = v_tempBuf[j]; int32_t k = j - 1;
            while (k >= 0 && v_tempBuf[k] > v_key) { v_tempBuf[k + 1] = v_tempBuf[k]; k--; }
            v_tempBuf[k + 1] = v_key;
        }
        p_data[i] = v_tempBuf[v_half];
    }
    for (uint8_t i = 0; i < v_half; i++) p_hist[i] = p_data[p_len - v_half + i];
}

void CL_T2_DspEngine::_applyPreEmphasis(float* p_data, float& p_prevSample, uint32_t p_len, float p_alpha) {
    if (p_len == 0) return;
    for (uint32_t i = p_len - 1; i > 0; i--) p_data[i] = p_data[i] - p_alpha * p_data[i - 1];
    p_data[0] = p_data[0] - p_alpha * p_prevSample;
    p_prevSample = p_data[p_len - 1];
}

void CL_T2_DspEngine::_applyNoiseGate(float* p_data, uint32_t p_len, float p_gateThresh) {
    for (uint32_t i = 0; i < p_len; i++) {
        if (fabsf(p_data[i]) < p_gateThresh) p_data[i] = 0.0f;
    }
}

void CL_T2_DspEngine::_generateFirLpfWindowedSinc(float* p_coeffs, uint16_t p_taps, float p_cutoffHz, float p_sampleRate) {
    float v_normalizedCutoff = p_cutoffHz / p_sampleRate;
    alignas(16) float v_win[T2_Config::FeatureLimit::FIR_TAPS_CONST] = {0};
    dsps_wind_blackman_f32(v_win, p_taps);
    float v_M = (float)(p_taps - 1);
    float v_sum = 0.0f;

    for (uint16_t i = 0; i < p_taps; i++) {
        float v_x = (float)i - (v_M / 2.0f);
        if (fabsf(v_x) < T2_Config::System::MATH_EPSILON_12_CONST) p_coeffs[i] = 2.0f * v_normalizedCutoff;
        else p_coeffs[i] = sinf(2.0f * (float)M_PI * v_normalizedCutoff * v_x) / ((float)M_PI * v_x);
        p_coeffs[i] *= v_win[i];
        v_sum += p_coeffs[i];
    }
    for (uint16_t i = 0; i < p_taps; i++) p_coeffs[i] /= v_sum;
}

void CL_T2_DspEngine::_generateFirHpfWindowedSinc(float* p_coeffs, uint16_t p_taps, float p_cutoffHz, float p_sampleRate) {
    _generateFirLpfWindowedSinc(p_coeffs, p_taps, p_cutoffHz, p_sampleRate);
    uint16_t v_center = (p_taps - 1) / 2;
    for (uint16_t i = 0; i < p_taps; i++) p_coeffs[i] = -p_coeffs[i];
    p_coeffs[v_center] += 1.0f;
}

// ============================================================================
// [교정 2] 링커 에러 방어: IIR Biquad 계수 생성 함수 복원
// ============================================================================
void CL_T2_DspEngine::_calcIirHpCoeffs(float p_freq, float p_q, float* p_coeffs, uint32_t p_sampleRate) {
    float v_w0 = 2.0f * (float)M_PI * p_freq / (float)p_sampleRate;
    float v_alpha = sinf(v_w0) / (2.0f * p_q);
    float v_cosW0 = cosf(v_w0);

    float v_b0 =  (1.0f + v_cosW0) / 2.0f;
    float v_b1 = -(1.0f + v_cosW0);
    float v_b2 =  (1.0f + v_cosW0) / 2.0f;
    float v_a0 =   1.0f + v_alpha;
    float v_a1 =  -2.0f * v_cosW0;
    float v_a2 =   1.0f - v_alpha;

    p_coeffs[0] = v_b0 / v_a0; p_coeffs[1] = v_b1 / v_a0; p_coeffs[2] = v_b2 / v_a0;
    p_coeffs[3] = v_a1 / v_a0; p_coeffs[4] = v_a2 / v_a0;
}

void CL_T2_DspEngine::_calcIirLpCoeffs(float p_freq, float p_q, float* p_coeffs, uint32_t p_sampleRate) {
    float v_w0 = 2.0f * (float)M_PI * p_freq / (float)p_sampleRate;
    float v_alpha = sinf(v_w0) / (2.0f * p_q);
    float v_cosW0 = cosf(v_w0);

    float v_b0 =  (1.0f - v_cosW0) / 2.0f;
    float v_b1 =   1.0f - v_cosW0;
    float v_b2 =  (1.0f - v_cosW0) / 2.0f;
    float v_a0 =   1.0f + v_alpha;
    float v_a1 =  -2.0f * v_cosW0;
    float v_a2 =   1.0f - v_alpha;

    p_coeffs[0] = v_b0 / v_a0; p_coeffs[1] = v_b1 / v_a0; p_coeffs[2] = v_b2 / v_a0;
    p_coeffs[3] = v_a1 / v_a0; p_coeffs[4] = v_a2 / v_a0;
}


