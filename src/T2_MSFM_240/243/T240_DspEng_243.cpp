/* ============================================================================
 * File: T240_DspEng_243.cpp
 * Summary: 멀티모달(진동/소음) 하이브리드 SIMD 고속 신호 처리 엔진 구현부 (v243)
 * ========================================================================== */
#include "T240_DspEng_243.hpp"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <cmath>
#include <cstring>

static const char* TAG = "T243_DSP";

CL_T2_DspEngine::CL_T2_DspEngine() {
    _isInitialized = false;
    _audDsp = nullptr;
    _accDsp = nullptr;
    _gyrDsp = nullptr;
}

CL_T2_DspEngine::~CL_T2_DspEngine() {
    if (_audDsp) { heap_caps_free(_audDsp); _audDsp = nullptr; }
    if (_accDsp) { heap_caps_free(_accDsp); _accDsp = nullptr; }
    if (_gyrDsp) { heap_caps_free(_gyrDsp); _gyrDsp = nullptr; }
}

bool CL_T2_DspEngine::init(const T2_Type::DynamicConfig& p_cfg) {
    // 1. [사용여부 적용 동적 할당 및 오버헤드 최소화]
    // Audio Dsp Alloc
    if (p_cfg.system.audio_enable) {
        if (!_audDsp) {
            _audDsp = (ST_AudioDspRuntime*)heap_caps_aligned_alloc(16, sizeof(ST_AudioDspRuntime), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
            if (!_audDsp) {
                // SRAM 부족 시 PSRAM 폴백
                _audDsp = (ST_AudioDspRuntime*)heap_caps_aligned_alloc(16, sizeof(ST_AudioDspRuntime), MALLOC_CAP_SPIRAM);
            }
        }
        if (_audDsp) {
            dsps_wind_hann_f32(_audDsp->window, T2_Def::Audio::Sensor::FFT_SIZE_MAX);
        }
    } else {
        if (_audDsp) { heap_caps_free(_audDsp); _audDsp = nullptr; }
    }

    // Accel Dsp Alloc
    if (p_cfg.system.vib_enable && p_cfg.vib_sensor.accel_enable) {
        if (!_accDsp) {
            _accDsp = (ST_AccelDspRuntime*)heap_caps_aligned_alloc(16, sizeof(ST_AccelDspRuntime), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
            if (!_accDsp) {
                _accDsp = (ST_AccelDspRuntime*)heap_caps_aligned_alloc(16, sizeof(ST_AccelDspRuntime), MALLOC_CAP_SPIRAM);
            }
        }
        if (_accDsp) {
            dsps_wind_hann_f32(_accDsp->window, T2_Def::Vib::Sensor::FFT_SIZE_MAX);
        }
    } else {
        if (_accDsp) { heap_caps_free(_accDsp); _accDsp = nullptr; }
    }

    // Gyro Dsp Alloc
    if (p_cfg.system.vib_enable && p_cfg.vib_sensor.gyro_enable) {
        if (!_gyrDsp) {
            _gyrDsp = (ST_GyroDspRuntime*)heap_caps_aligned_alloc(16, sizeof(ST_GyroDspRuntime), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
            if (!_gyrDsp) {
                _gyrDsp = (ST_GyroDspRuntime*)heap_caps_aligned_alloc(16, sizeof(ST_GyroDspRuntime), MALLOC_CAP_SPIRAM);
            }
        }
        if (_gyrDsp) {
            dsps_wind_hann_f32(_gyrDsp->window, T2_Def::Vib::Sensor::FFT_SIZE_MAX);
        }
    } else {
        if (_gyrDsp) { heap_caps_free(_gyrDsp); _gyrDsp = nullptr; }
    }

    reloadFilters(p_cfg);
    resetStates();

    _isInitialized = true;
    ESP_LOGI(TAG, "DSP Engine v243 Initialized with Dynamic Allocations (Aud:%d, Acc:%d, Gyr:%d)",
             (int)(_audDsp != nullptr), (int)(_accDsp != nullptr), (int)(_gyrDsp != nullptr));
    return true;
}

void CL_T2_DspEngine::reloadFilters(const T2_Type::DynamicConfig& p_cfg) {
    // 1. Audio Filters
    if (_audDsp) {
        uint32_t p_audSampleRate = p_cfg.aud_sensor.sample_rate;
        if (p_cfg.shared_dsp.notch.en) {
            _calcNotchCoeffs(p_cfg.shared_dsp.notch.freq, p_cfg.shared_dsp.notch.q, _audDsp->notch_coeffs, p_audSampleRate);
        }
        if (p_cfg.shared_dsp.notch2.en) {
            _calcNotchCoeffs(p_cfg.shared_dsp.notch2.freq, p_cfg.shared_dsp.notch2.q, _audDsp->notch2_coeffs, p_audSampleRate);
        }
        if (p_cfg.shared_dsp.iir_hpf.en) {
            _calcIirCoeffs(p_cfg.shared_dsp.iir_hpf.cutoff, p_cfg.shared_dsp.iir_hpf.q, _audDsp->iir_hpf_coeffs, p_audSampleRate, true);
        }
        if (p_cfg.shared_dsp.iir_lpf.en) {
            _calcIirCoeffs(p_cfg.shared_dsp.iir_lpf.cutoff, p_cfg.shared_dsp.iir_lpf.q, _audDsp->iir_lpf_coeffs, p_audSampleRate, false);
        }

        // FIR Audio
        if (p_cfg.shared_dsp.hpf.en && p_cfg.shared_dsp.hpf.taps > 0) {
            _generateFirHpf(_audDsp->fir_hpf_coeffs, p_cfg.shared_dsp.hpf.taps, p_cfg.shared_dsp.hpf.cutoff, (float)p_audSampleRate);
            dsps_fir_init_f32(&_audDsp->fir_inst_hpf, _audDsp->fir_hpf_coeffs, _audDsp->fir_state_hpf, p_cfg.shared_dsp.hpf.taps);
        }
        if (p_cfg.shared_dsp.lpf.en && p_cfg.shared_dsp.lpf.taps > 0) {
            _generateFirLpf(_audDsp->fir_lpf_coeffs, p_cfg.shared_dsp.lpf.taps, p_cfg.shared_dsp.lpf.cutoff, p_audSampleRate);
            dsps_fir_init_f32(&_audDsp->fir_inst_lpf, _audDsp->fir_lpf_coeffs, _audDsp->fir_state_lpf, p_cfg.shared_dsp.lpf.taps);
        }
    }

    // 2. Accel Filters
    if (_accDsp) {
        uint32_t p_vibSampleRate = p_cfg.vib_sensor.sample_rate;
        if (p_cfg.shared_dsp.hpf.en && p_cfg.shared_dsp.hpf.taps > 0) {
            _generateFirHpf(_accDsp->fir_hpf_coeffs, p_cfg.shared_dsp.hpf.taps, p_cfg.shared_dsp.hpf.cutoff, p_vibSampleRate);
            for (int i = 0; i < T2_Def::Vib::Sensor::AXIS_MAX; i++) {
                dsps_fir_init_f32(&_accDsp->fir_inst_hpf[i], _accDsp->fir_hpf_coeffs, _accDsp->fir_state_hpf[i], p_cfg.shared_dsp.hpf.taps);
            }
        }
        if (p_cfg.shared_dsp.lpf.en && p_cfg.shared_dsp.lpf.taps > 0) {
            _generateFirLpf(_accDsp->fir_lpf_coeffs, p_cfg.shared_dsp.lpf.taps, p_cfg.shared_dsp.lpf.cutoff, p_vibSampleRate);
            for (int i = 0; i < T2_Def::Vib::Sensor::AXIS_MAX; i++) {
                dsps_fir_init_f32(&_accDsp->fir_inst_lpf[i], _accDsp->fir_lpf_coeffs, _accDsp->fir_state_lpf[i], p_cfg.shared_dsp.lpf.taps);
            }
        }
    }

    // 3. Gyro Filters
    if (_gyrDsp) {
        uint32_t p_vibSampleRate = p_cfg.vib_sensor.sample_rate;
        // 자이로 필터는 필요시 다른 특화 룰 적용 가능
        if (p_cfg.shared_dsp.hpf.en && p_cfg.shared_dsp.hpf.taps > 0) {
            _generateFirHpf(_gyrDsp->fir_hpf_coeffs, p_cfg.shared_dsp.hpf.taps, p_cfg.shared_dsp.hpf.cutoff, p_vibSampleRate);
            for (int i = 0; i < T2_Def::Vib::Sensor::AXIS_MAX; i++) {
                dsps_fir_init_f32(&_gyrDsp->fir_inst_hpf[i], _gyrDsp->fir_hpf_coeffs, _gyrDsp->fir_state_hpf[i], p_cfg.shared_dsp.hpf.taps);
            }
        }
        if (p_cfg.shared_dsp.lpf.en && p_cfg.shared_dsp.lpf.taps > 0) {
            _generateFirLpf(_gyrDsp->fir_lpf_coeffs, p_cfg.shared_dsp.lpf.taps, p_cfg.shared_dsp.lpf.cutoff, p_vibSampleRate);
            for (int i = 0; i < T2_Def::Vib::Sensor::AXIS_MAX; i++) {
                dsps_fir_init_f32(&_gyrDsp->fir_inst_lpf[i], _gyrDsp->fir_lpf_coeffs, _gyrDsp->fir_state_lpf[i], p_cfg.shared_dsp.lpf.taps);
            }
        }
    }

    ESP_LOGI(TAG, "Filters Reloaded (FIR Taps:%d)", p_cfg.shared_dsp.hpf.taps);
}

void CL_T2_DspEngine::resetStates() {
    if (_audDsp) {
        _audDsp->prev_pre_emp = 0.0f;
        memset(_audDsp->median_hist, 0, sizeof(_audDsp->median_hist));
        memset(_audDsp->notch_state, 0, sizeof(_audDsp->notch_state));
        memset(_audDsp->notch2_state, 0, sizeof(_audDsp->notch2_state));
        memset(_audDsp->iir_hpf_state, 0, sizeof(_audDsp->iir_hpf_state));
        memset(_audDsp->iir_lpf_state, 0, sizeof(_audDsp->iir_lpf_state));
        memset(_audDsp->fir_state_hpf, 0, sizeof(_audDsp->fir_state_hpf));
        memset(_audDsp->fir_state_lpf, 0, sizeof(_audDsp->fir_state_lpf));
    }
    if (_accDsp) {
        memset(_accDsp->median_hist, 0, sizeof(_accDsp->median_hist));
        memset(_accDsp->fir_state_hpf, 0, sizeof(_accDsp->fir_state_hpf));
        memset(_accDsp->fir_state_lpf, 0, sizeof(_accDsp->fir_state_lpf));
    }
    if (_gyrDsp) {
        memset(_gyrDsp->median_hist, 0, sizeof(_gyrDsp->median_hist));
        memset(_gyrDsp->fir_state_hpf, 0, sizeof(_gyrDsp->fir_state_hpf));
        memset(_gyrDsp->fir_state_lpf, 0, sizeof(_gyrDsp->fir_state_lpf));
    }
}

void CL_T2_DspEngine::processAudio(const float* p_audL, const float* p_audR, float* p_out, uint32_t p_len,
                                   const T2_Type::ST_Shared_Dsp& p_sharedCfg, const T2_Type::ST_Audio_Dsp& p_audCfg) {
    if (!_isInitialized || !_audDsp) return;

    // [Step 1] Beamforming / Channel Selection (L/R 융합 또는 단일 채널 통과)
    if (p_audL && p_audR) {
        // Stereo Mode: Beamforming
        for (uint32_t i = 0; i < p_len; i++) {
            p_out[i] = (p_audL[i] + p_audR[i]) * p_audCfg.beam_gain;
        }
    } else if (p_audL) {
        // Mono Mode (L or R-mapped-to-L): Bypass with Gain
        for (uint32_t i = 0; i < p_len; i++) {
            p_out[i] = p_audL[i] * p_audCfg.beam_gain;
        }
    } else if (p_audR) {
        // Mono Mode (R only fallback): Bypass with Gain
        for (uint32_t i = 0; i < p_len; i++) {
            p_out[i] = p_audR[i] * p_audCfg.beam_gain;
        }
    } else {
        memset(p_out, 0, p_len * sizeof(float));
    }

    // [Step 2] DC Removal (오디오 바이어스 제거)
    _removeDC(p_out, p_len);

    // [Step 3] Median Filter (스파이크 제거를 필터링 전단으로 이동)
    if (p_sharedCfg.med_en) {
        _applyMedianFilter(p_out, _audDsp->median_hist, p_sharedCfg.med_win, p_len);
    }

    // [Step 4] Pre-emphasis (고주파 강조)
    if (p_audCfg.pre_en) {
        _applyPreEmphasis(p_out, _audDsp->prev_pre_emp, p_len, p_audCfg.pre_alpha);
    }

    // [Step 5] IIR Notch Filter (특정 고주파 노이즈 제거)
    if (p_sharedCfg.notch.en) {
        dsps_biquad_f32_aes3(p_out, p_out, p_len, _audDsp->notch_coeffs, _audDsp->notch_state);
    }
    if (p_sharedCfg.notch2.en) {
        dsps_biquad_f32_aes3(p_out, p_out, p_len, _audDsp->notch2_coeffs, _audDsp->notch2_state);
    }

    // [Step 6] IIR HPF/LPF (가벼운 필터링)
    if (p_sharedCfg.iir_hpf.en) {
        dsps_biquad_f32_aes3(p_out, p_out, p_len, _audDsp->iir_hpf_coeffs, _audDsp->iir_hpf_state);
    }
    if (p_sharedCfg.iir_lpf.en) {
        dsps_biquad_f32_aes3(p_out, p_out, p_len, _audDsp->iir_lpf_coeffs, _audDsp->iir_lpf_state);
    }

    // [Step 7] FIR HPF/LPF (정밀 필터링 - esp-dsp SIMD 가속)
    if (p_sharedCfg.hpf.en) {
        dsps_fir_f32_aes3(&_audDsp->fir_inst_hpf, p_out, p_out, p_len);
    }
    if (p_sharedCfg.lpf.en) {
        dsps_fir_f32_aes3(&_audDsp->fir_inst_lpf, p_out, p_out, p_len);
    }

    // [Step 8] Noise Gate (미세 소음 차단)
    if (p_audCfg.noise.gate_en) {
        _applyNoiseGate(p_out, p_len, p_audCfg.noise.gate_thresh);
    }
}

void CL_T2_DspEngine::processAccel(const float* p_inX, const float* p_inY, const float* p_inZ,
                                   float* p_outX, float* p_outY, float* p_outZ, uint32_t p_len,
                                   const T2_Type::ST_Shared_Dsp& p_sharedCfg) {
    if (!_isInitialized || !_accDsp) return;

    float* v_out[3] = {p_outX, p_outY, p_outZ};
    const float* v_in[3] = {p_inX, p_inY, p_inZ};

    for (int i = 0; i < T2_Def::Vib::Sensor::AXIS_MAX; i++) {
        if (!v_in[i] || !v_out[i]) continue;

        // 원본 복사
        memcpy(v_out[i], v_in[i], p_len * sizeof(float));

        // [Step 1] Median Filter
        if (p_sharedCfg.med_en) {
            _applyMedianFilter(v_out[i], _accDsp->median_hist[i], p_sharedCfg.med_win, p_len);
        }

        // [Step 2] FIR Filtering (esp-dsp SIMD HPF/LPF)
        if (p_sharedCfg.hpf.en) {
            dsps_fir_f32_aes3(&_accDsp->fir_inst_hpf[i], v_out[i], v_out[i], p_len);
        }
        if (p_sharedCfg.lpf.en) {
            dsps_fir_f32_aes3(&_accDsp->fir_inst_lpf[i], v_out[i], v_out[i], p_len);
        }

        // [Step 3] DC Removal (Option)
        if (p_sharedCfg.rem_dc) {
            _removeDC(v_out[i], p_len);
        }
    }
}

void CL_T2_DspEngine::processGyro(const float* p_inX, const float* p_inY, const float* p_inZ,
                                  float* p_outX, float* p_outY, float* p_outZ, uint32_t p_len,
                                  const T2_Type::ST_Shared_Dsp& p_sharedCfg) {
    if (!_isInitialized || !_gyrDsp) return;

    float* v_out[3] = {p_outX, p_outY, p_outZ};
    const float* v_in[3] = {p_inX, p_inY, p_inZ};

    for (int i = 0; i < T2_Def::Vib::Sensor::AXIS_MAX; i++) {
        if (!v_in[i] || !v_out[i]) continue;

        // 원본 복사
        memcpy(v_out[i], v_in[i], p_len * sizeof(float));

        // [Step 1] Median Filter
        if (p_sharedCfg.med_en) {
            _applyMedianFilter(v_out[i], _gyrDsp->median_hist[i], p_sharedCfg.med_win, p_len);
        }

        // [Step 2] FIR Filtering (자이로 특화 고역 통과 필터로 드리프트 제거 등 대응)
        if (p_sharedCfg.hpf.en) {
            dsps_fir_f32_aes3(&_gyrDsp->fir_inst_hpf[i], v_out[i], v_out[i], p_len);
        }
        if (p_sharedCfg.lpf.en) {
            dsps_fir_f32_aes3(&_gyrDsp->fir_inst_lpf[i], v_out[i], v_out[i], p_len);
        }

        // [Step 3] DC Removal (자이로는 각속도의 누적 적분 오차(Drift) 방지를 위해 DC 오프셋 완전 차단이 핵심)
        if (p_sharedCfg.rem_dc) {
            _removeDC(v_out[i], p_len);
        }
    }
}

// --- Filter Generation Helpers ---

void CL_T2_DspEngine::_generateFirLpf(float* p_coeffs, uint16_t p_taps, float p_cutoffHz, float p_sampleRate) {
    float v_ft = p_cutoffHz / p_sampleRate;
    for (int i = 0; i < p_taps; i++) {
        float n = i - (p_taps - 1) / 2.0f;
        if (std::abs(n) < T2_Def::Global::System::MATH_EPSILON_12_CONST) {
            p_coeffs[i] = 2.0f * v_ft;
        } else {
            p_coeffs[i] = std::sin(2.0f * M_PI * v_ft * n) / (M_PI * n);
        }
        // Blackman Window 적용
        float w = 0.42f - 0.5f * std::cos(2.0f * M_PI * i / (p_taps - 1)) + 0.08f * std::cos(4.0f * M_PI * i / (p_taps - 1));
        p_coeffs[i] *= w;
    }
}

void CL_T2_DspEngine::_generateFirHpf(float* p_coeffs, uint16_t p_taps, float p_cutoffHz, float p_sampleRate) {
    _generateFirLpf(p_coeffs, p_taps, p_cutoffHz, p_sampleRate);
    for (int i = 0; i < p_taps; i++) {
        p_coeffs[i] = -p_coeffs[i];
    }
    p_coeffs[(p_taps - 1) / 2] += 1.0f;
}

void CL_T2_DspEngine::_calcIirCoeffs(float p_freq, float p_q, float* p_coeffs, uint32_t p_sampleRate, bool p_isHpf) {
    if (p_isHpf) dsps_biquad_gen_hpf_f32(p_coeffs, p_freq / p_sampleRate, p_q);
    else dsps_biquad_gen_lpf_f32(p_coeffs, p_freq / p_sampleRate, p_q);
}

void CL_T2_DspEngine::_calcNotchCoeffs(float p_freq, float p_q, float* p_coeffs, uint32_t p_sampleRate) {
    dsps_biquad_gen_notch_f32(p_coeffs, p_freq / (float)p_sampleRate, -60.0f, p_q);
}

// --- DSP Utilities ---

void CL_T2_DspEngine::_applyMedianFilter(float* p_data, float* p_hist, uint8_t p_windowSize, uint32_t p_len) {
    // 단순 3점/5점 메디안 필터 (속도 우선)
    for (uint32_t i = 0; i < p_len; i++) {
        // 히스토리 업데이트 (Shift)
        for (int k = p_windowSize - 1; k > 0; k--) p_hist[k] = p_hist[k - 1];
        p_hist[0] = p_data[i];

        // 정렬 및 중앙값 추출 (작은 윈도우에 최적화)
        float v_sort[T2_Def::Shared::Dsp::MEDIAN_WINDOW_MAX];
        memcpy(v_sort, p_hist, p_windowSize * sizeof(float));
        for (int j = 0; j < p_windowSize - 1; j++) {
            for (int k = j + 1; k < p_windowSize; k++) {
                if (v_sort[j] > v_sort[k]) {
                    float tmp = v_sort[j]; v_sort[j] = v_sort[k]; v_sort[k] = tmp;
                }
            }
        }
        p_data[i] = v_sort[p_windowSize / 2];
    }
}

void CL_T2_DspEngine::_applyPreEmphasis(float* p_data, float& p_prevSample, uint32_t p_len, float p_alpha) {
    for (uint32_t i = 0; i < p_len; i++) {
        float current = p_data[i];
        p_data[i] = current - p_alpha * p_prevSample;
        p_prevSample = current;
    }
}

void CL_T2_DspEngine::_applyNoiseGate(float* p_data, uint32_t p_len, float p_gateThresh) {
    for (uint32_t i = 0; i < p_len; i++) {
        if (std::abs(p_data[i]) < p_gateThresh) p_data[i] = 0.0f;
    }
}

void CL_T2_DspEngine::_removeDC(float* p_data, uint32_t p_len) {
    float v_sum = 0.0f;
    for (uint32_t i = 0; i < p_len; i++) {
        // NaN/Inf 방어 (v234 정합성)
        uint32_t v_bits;
        memcpy(&v_bits, &p_data[i], 4);
        if ((v_bits & 0x7F800000) == 0x7F800000) p_data[i] = 0.0f;
        v_sum += p_data[i];
    }
    float v_mean = v_sum / p_len;
    for (uint32_t i = 0; i < p_len; i++) p_data[i] -= v_mean;
}
