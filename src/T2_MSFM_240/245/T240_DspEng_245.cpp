/* ============================================================================
 * File: T240_DspEng_245.cpp
 * Summary: 멀티모달(진동/소음) SIMD 고속 신호 처리 엔진 구현부
 * ============================================================================ */

#include "T240_DspEng_245.hpp"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <cmath>
#include <cstring>

static const char* TAG = "T245_DSP";

/**
 * @brief FIR 차수에 따라 적합한 esp-dsp 구현체 함수를 안전하게 호출하는 인라인 래퍼
 * @param fir FIR 인스턴스 구조체
 * @param input 입력 신호 버퍼
 * @param output 결과 출력 버퍼
 * @param len 데이터 길이
 */
inline void safe_dsps_fir_f32(fir_f32_t* fir, const float* input, float* output, int len) {
    if (fir->N % 4 == 0) {
        dsps_fir_f32_aes3(fir, input, output, len);
    } else {
        dsps_fir_f32(fir, input, output, len);
    }
}

/**
 * @brief CL_T2_DspEngine 생성자
 */
CL_T2_DspEngine::CL_T2_DspEngine() {
    _isInitialized = false;
    _audDsp = nullptr;
    _accDsp = nullptr;
    _gyrDsp = nullptr;
}

/**
 * @brief CL_T2_DspEngine 소멸자 (동적 할당된 필터 내부 상태 메모리 해제)
 */
CL_T2_DspEngine::~CL_T2_DspEngine() {
    if (_audDsp) { heap_caps_free(_audDsp); _audDsp = nullptr; }
    if (_accDsp) { heap_caps_free(_accDsp); _accDsp = nullptr; }
    if (_gyrDsp) { heap_caps_free(_gyrDsp); _gyrDsp = nullptr; }
}

/**
 * @brief 활성화 상태에 매칭되는 필터 연산 상태 힙 영역 메모리 할당
 * @param p_cfg 전체 시스템 동적 설정
 * @return 초기화 성공 여부
 */
bool CL_T2_DspEngine::init(const T2_Type::ST_DynamicConfig_t& p_cfg) {
    // [처리 단위 1] 오디오(마이크) 필터 내부 구조체 상태 힙 할당 및 윈도우 생성
    if (p_cfg.audio.enable) {
        if (!_audDsp) {
            _audDsp = (ST_AudioDspRuntime*)heap_caps_aligned_alloc(16, sizeof(ST_AudioDspRuntime), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
            if (!_audDsp) {
                _audDsp = (ST_AudioDspRuntime*)heap_caps_aligned_alloc(16, sizeof(ST_AudioDspRuntime), MALLOC_CAP_SPIRAM);
            }
        }
        if (_audDsp) {
            switch (p_cfg.audio.dsp.win_type) {
                case T2_Type::EM_WindowType_t::HAMMING:  dsps_wind_hann_f32(_audDsp->window, p_cfg.audio.fft_size);  break;
                case T2_Type::EM_WindowType_t::BLACKMAN: dsps_wind_blackman_f32(_audDsp->window, p_cfg.audio.fft_size); break;
                default:                            dsps_wind_hann_f32(_audDsp->window, p_cfg.audio.fft_size);     break;
            }
        }
    } else {
        if (_audDsp) { heap_caps_free(_audDsp); _audDsp = nullptr; }
    }

    // [처리 단위 2] 가속도(진동) 필터 내부 구조체 상태 힙 할당
    if (p_cfg.accel.enable) {
        if (!_accDsp) {
            _accDsp = (ST_AccelDspRuntime*)heap_caps_aligned_alloc(16, sizeof(ST_AccelDspRuntime), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
            if (!_accDsp) {
                _accDsp = (ST_AccelDspRuntime*)heap_caps_aligned_alloc(16, sizeof(ST_AccelDspRuntime), MALLOC_CAP_SPIRAM);
            }
        }
        if (_accDsp) {
            dsps_wind_hann_f32(_accDsp->window, p_cfg.accel.fft_size);
        }
    } else {
        if (_accDsp) { heap_caps_free(_accDsp); _accDsp = nullptr; }
    }

    // [처리 단위 3] 자이로 필터 내부 구조체 상태 힙 할당 및 윈도우 생성
    if (p_cfg.gyro.enable) {
        if (!_gyrDsp) {
            _gyrDsp = (ST_GyroDspRuntime*)heap_caps_aligned_alloc(16, sizeof(ST_GyroDspRuntime), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
            if (!_gyrDsp) {
                _gyrDsp = (ST_GyroDspRuntime*)heap_caps_aligned_alloc(16, sizeof(ST_GyroDspRuntime), MALLOC_CAP_SPIRAM);
            }
        }
        if (_gyrDsp) {
            switch (p_cfg.gyro.dsp.win_type) {
                case T2_Type::EM_WindowType_t::HAMMING:  dsps_wind_hann_f32(_gyrDsp->window, p_cfg.gyro.fft_size);  break;
                case T2_Type::EM_WindowType_t::BLACKMAN: dsps_wind_blackman_f32(_gyrDsp->window, p_cfg.gyro.fft_size); break;
                default:                            dsps_wind_hann_f32(_gyrDsp->window, p_cfg.gyro.fft_size);     break;
            }
        }
    } else {
        if (_gyrDsp) { heap_caps_free(_gyrDsp); _gyrDsp = nullptr; }
    }

    reloadFilters(p_cfg);
    resetStates();

    _isInitialized = true;
    ESP_LOGI(TAG, "DSP Engine Initialized (Aud:%d, Acc:%d, Gyr:%d)",
             (int)(_audDsp != nullptr), (int)(_accDsp != nullptr), (int)(_gyrDsp != nullptr));
    return true;
}

/**
 * @brief 동적 필터 설정(컷오프 차수, 탭 크기 등) 변경 시 호출되어 계수 재지정
 * @param p_cfg 최신 설정 구조체
 */
void CL_T2_DspEngine::reloadFilters(const T2_Type::ST_DynamicConfig_t& p_cfg) {
    // [처리 단위 1] 오디오 필터(IIR Notch, HPF, LPF 및 FIR) 계수 연산 및 초기 설정
    if (_audDsp) {
        uint32_t p_audSampleRate = p_cfg.audio.sample_rate;
        const T2_Type::ST_Dsp_Config_t& v_dsp = p_cfg.audio.dsp;

        if (v_dsp.notch.en) {
            _calcNotchCoeffs(v_dsp.notch.freq, v_dsp.notch.q, _audDsp->notch_coeffs, p_audSampleRate);
        }
        if (v_dsp.notch2.en) {
            _calcNotchCoeffs(v_dsp.notch2.freq, v_dsp.notch2.q, _audDsp->notch2_coeffs, p_audSampleRate);
        }
        if (v_dsp.iir_hpf.en) {
            _calcIirCoeffs(v_dsp.iir_hpf.cutoff, v_dsp.iir_hpf.q, _audDsp->iir_hpf_coeffs, p_audSampleRate, true);
        }
        if (v_dsp.iir_lpf.en) {
            _calcIirCoeffs(v_dsp.iir_lpf.cutoff, v_dsp.iir_lpf.q, _audDsp->iir_lpf_coeffs, p_audSampleRate, false);
        }

        uint16_t hpf_taps = v_dsp.hpf.taps;
        if (v_dsp.hpf.en && (hpf_taps % 2 == 0)) {
            ESP_LOGW(TAG, "FIR HPF taps must be odd, adjusting %d -> %d", hpf_taps, hpf_taps+1);
            hpf_taps += 1;
        }

        if (v_dsp.hpf.en && hpf_taps > 0) {
            _generateFirHpf(_audDsp->fir_hpf_coeffs, hpf_taps, v_dsp.hpf.cutoff, (float)p_audSampleRate);
            for (int ch = 0; ch < 2; ch++) {
                memset(_audDsp->fir_state_hpf[ch], 0, sizeof(_audDsp->fir_state_hpf[ch]));
                dsps_fir_init_f32(&_audDsp->fir_inst_hpf[ch], _audDsp->fir_hpf_coeffs, _audDsp->fir_state_hpf[ch], hpf_taps);
            }
        }
        if (v_dsp.lpf.en && v_dsp.lpf.taps > 0) {
            _generateFirLpf(_audDsp->fir_lpf_coeffs, v_dsp.lpf.taps, v_dsp.lpf.cutoff, p_audSampleRate);
            for (int ch = 0; ch < 2; ch++) {
                memset(_audDsp->fir_state_lpf[ch], 0, sizeof(_audDsp->fir_state_lpf[ch]));
                dsps_fir_init_f32(&_audDsp->fir_inst_lpf[ch], _audDsp->fir_lpf_coeffs, _audDsp->fir_state_lpf[ch], v_dsp.lpf.taps);
            }
        }
    }

    // [처리 단위 2] 가속도 필터(IIR Notch, HPF, LPF 및 FIR) 계수 연산 및 초기 설정 (3축 동일 필터 적용)
    if (_accDsp) {
        uint32_t p_accSampleRate = p_cfg.accel.sample_rate;
        const T2_Type::ST_Dsp_Config_t& v_dsp = p_cfg.accel.dsp;

        if (v_dsp.notch.en) {
            _calcNotchCoeffs(v_dsp.notch.freq, v_dsp.notch.q, _accDsp->notch_coeffs, p_accSampleRate);
        }
        if (v_dsp.notch2.en) {
            _calcNotchCoeffs(v_dsp.notch2.freq, v_dsp.notch2.q, _accDsp->notch2_coeffs, p_accSampleRate);
        }
        if (v_dsp.iir_hpf.en) {
            _calcIirCoeffs(v_dsp.iir_hpf.cutoff, v_dsp.iir_hpf.q, _accDsp->iir_hpf_coeffs, p_accSampleRate, true);
        }
        if (v_dsp.iir_lpf.en) {
            _calcIirCoeffs(v_dsp.iir_lpf.cutoff, v_dsp.iir_lpf.q, _accDsp->iir_lpf_coeffs, p_accSampleRate, false);
        }

        uint16_t hpf_taps = v_dsp.hpf.taps;
        if (v_dsp.hpf.en && (hpf_taps % 2 == 0)) {
            ESP_LOGW(TAG, "FIR HPF taps must be odd, adjusting %d -> %d", hpf_taps, hpf_taps+1);
            hpf_taps += 1;
        }

        if (v_dsp.hpf.en && hpf_taps > 0) {
            _generateFirHpf(_accDsp->fir_hpf_coeffs[0], hpf_taps, v_dsp.hpf.cutoff, p_accSampleRate);
            memcpy(_accDsp->fir_hpf_coeffs[1], _accDsp->fir_hpf_coeffs[0], hpf_taps * sizeof(float));
            memcpy(_accDsp->fir_hpf_coeffs[2], _accDsp->fir_hpf_coeffs[0], hpf_taps * sizeof(float));
            for (int i = 0; i < T2_Def::Accel::Sensor::AXIS_MAX; i++) {
                memset(_accDsp->fir_state_hpf[i], 0, sizeof(_accDsp->fir_state_hpf[i]));
                dsps_fir_init_f32(&_accDsp->fir_inst_hpf[i], _accDsp->fir_hpf_coeffs[i], _accDsp->fir_state_hpf[i], hpf_taps);
            }
        }
        if (v_dsp.lpf.en && v_dsp.lpf.taps > 0) {
            _generateFirLpf(_accDsp->fir_lpf_coeffs[0], v_dsp.lpf.taps, v_dsp.lpf.cutoff, p_accSampleRate);
            memcpy(_accDsp->fir_lpf_coeffs[1], _accDsp->fir_lpf_coeffs[0], v_dsp.lpf.taps * sizeof(float));
            memcpy(_accDsp->fir_lpf_coeffs[2], _accDsp->fir_lpf_coeffs[0], v_dsp.lpf.taps * sizeof(float));
            for (int i = 0; i < T2_Def::Accel::Sensor::AXIS_MAX; i++) {
                memset(_accDsp->fir_state_lpf[i], 0, sizeof(_accDsp->fir_state_lpf[i]));
                dsps_fir_init_f32(&_accDsp->fir_inst_lpf[i], _accDsp->fir_lpf_coeffs[i], _accDsp->fir_state_lpf[i], v_dsp.lpf.taps);
            }
        }
    }

    // [처리 단위 3] 자이로 필터(IIR Notch, HPF, LPF 및 FIR) 계수 연산 및 초기 설정 (3축 개별)
    if (_gyrDsp) {
        uint32_t p_gyrSampleRate = p_cfg.gyro.sample_rate;
        const T2_Type::ST_Dsp_Config_t& v_dsp = p_cfg.gyro.dsp;

        if (v_dsp.notch.en) {
            _calcNotchCoeffs(v_dsp.notch.freq, v_dsp.notch.q, _gyrDsp->notch_coeffs, p_gyrSampleRate);
        }
        if (v_dsp.notch2.en) {
            _calcNotchCoeffs(v_dsp.notch2.freq, v_dsp.notch2.q, _gyrDsp->notch2_coeffs, p_gyrSampleRate);
        }
        if (v_dsp.iir_hpf.en) {
            _calcIirCoeffs(v_dsp.iir_hpf.cutoff, v_dsp.iir_hpf.q, _gyrDsp->iir_hpf_coeffs, p_gyrSampleRate, true);
        }
        if (v_dsp.iir_lpf.en) {
            _calcIirCoeffs(v_dsp.iir_lpf.cutoff, v_dsp.iir_lpf.q, _gyrDsp->iir_lpf_coeffs, p_gyrSampleRate, false);
        }

        uint16_t hpf_taps = v_dsp.hpf.taps;
        if (v_dsp.hpf.en && (hpf_taps % 2 == 0)) {
            ESP_LOGW(TAG, "FIR HPF taps must be odd, adjusting %d -> %d", hpf_taps, hpf_taps+1);
            hpf_taps += 1;
        }

        if (v_dsp.hpf.en && hpf_taps > 0) {
            for (int i = 0; i < T2_Def::Gyro::Sensor::AXIS_MAX; i++) {
                _generateFirHpf(_gyrDsp->fir_hpf_coeffs[i], hpf_taps, v_dsp.hpf.cutoff, p_gyrSampleRate);
                dsps_fir_init_f32(&_gyrDsp->fir_inst_hpf[i], _gyrDsp->fir_hpf_coeffs[i], _gyrDsp->fir_state_hpf[i], hpf_taps);
            }
        }
        if (v_dsp.lpf.en && v_dsp.lpf.taps > 0) {
            for (int i = 0; i < T2_Def::Gyro::Sensor::AXIS_MAX; i++) {
                _generateFirLpf(_gyrDsp->fir_lpf_coeffs[i], v_dsp.lpf.taps, v_dsp.lpf.cutoff, p_gyrSampleRate);
                dsps_fir_init_f32(&_gyrDsp->fir_inst_lpf[i], _gyrDsp->fir_lpf_coeffs[i], _gyrDsp->fir_state_lpf[i], v_dsp.lpf.taps);
            }
        }
    }

    uint16_t v_taps = 0;
    if (_accDsp) v_taps = p_cfg.accel.dsp.hpf.taps;
    else if (_audDsp) v_taps = p_cfg.audio.dsp.hpf.taps;
    ESP_LOGI(TAG, "Filters Reloaded (Taps:%d)", v_taps);
}

/**
 * @brief 필터 상태 지연 버퍼 리셋 (이력 초기화)
 */
void CL_T2_DspEngine::resetStates() {
    if (_audDsp) {
        _audDsp->prev_pre_emp[0] = 0.0f;
        _audDsp->prev_pre_emp[1] = 0.0f;
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
        memset(_accDsp->notch_state, 0, sizeof(_accDsp->notch_state));
        memset(_accDsp->notch2_state, 0, sizeof(_accDsp->notch2_state));
        memset(_accDsp->iir_hpf_state, 0, sizeof(_accDsp->iir_hpf_state));
        memset(_accDsp->iir_lpf_state, 0, sizeof(_accDsp->iir_lpf_state));
        memset(_accDsp->fir_state_hpf, 0, sizeof(_accDsp->fir_state_hpf));
        memset(_accDsp->fir_state_lpf, 0, sizeof(_accDsp->fir_state_lpf));
    }
    if (_gyrDsp) {
        memset(_gyrDsp->median_hist, 0, sizeof(_gyrDsp->median_hist));
        memset(_gyrDsp->notch_state, 0, sizeof(_gyrDsp->notch_state));
        memset(_gyrDsp->notch2_state, 0, sizeof(_gyrDsp->notch2_state));
        memset(_gyrDsp->iir_hpf_state, 0, sizeof(_gyrDsp->iir_hpf_state));
        memset(_gyrDsp->iir_lpf_state, 0, sizeof(_gyrDsp->iir_lpf_state));
        memset(_gyrDsp->fir_state_hpf, 0, sizeof(_gyrDsp->fir_state_hpf));
        memset(_gyrDsp->fir_state_lpf, 0, sizeof(_gyrDsp->fir_state_lpf));
    }
}

/**
 * @brief 오디오 채널 신호 필터 연산 및 빔포밍 합성 파이프라인
 * @param p_audL, p_audR 입력 신호 버퍼
 * @param p_outL, p_outR 결과 출력 버퍼
 * @param p_len 데이터 크기
 * @param p_audCfg 오디오 세부 설정
 */
void CL_T2_DspEngine::processAudio(const float* p_audL, const float* p_audR, float* p_outL, float* p_outR, uint32_t p_len,
                                  const T2_Type::ST_Audio_Config_t& p_audCfg) {
    if (!_isInitialized || !_audDsp) return;

    static bool s_prevBeamActive = false;
    bool currentBeam = (p_audL && p_audR && p_audCfg.beam_gain > 0.001f);
    if (currentBeam != s_prevBeamActive) {
        resetStates();
        s_prevBeamActive = currentBeam;
    }

    bool v_useBeam = (p_audL && p_audR && p_audCfg.beam_gain > 0.001f);
    const T2_Type::ST_Dsp_Config_t& v_dsp = p_audCfg.dsp;

    // [처리 단위 1] 빔포밍(두 마이크 평균 합성)이 활성화된 경우
    if (v_useBeam) {
        uint32_t v_len = (p_len > T2_Def::Audio::Sensor::FFT_SIZE_MAX) ? T2_Def::Audio::Sensor::FFT_SIZE_MAX : p_len;

        if (p_outL) {
            for (uint32_t i = 0; i < v_len; i++) {
                p_outL[i] = (p_audL[i] + p_audR[i]) * p_audCfg.beam_gain;
            }
            _removeDC(p_outL, v_len);
            if (v_dsp.med_en) _applyMedianFilter(p_outL, _audDsp->median_hist[0], v_dsp.med_win, v_len);
            if (p_audCfg.pre_en) _applyPreEmphasis(p_outL, _audDsp->prev_pre_emp[0], v_len, p_audCfg.pre_alpha);
            if (v_dsp.notch.en)  dsps_biquad_f32_aes3(p_outL, p_outL, v_len, _audDsp->notch_coeffs, _audDsp->notch_state[0]);
            if (v_dsp.notch2.en) dsps_biquad_f32_aes3(p_outL, p_outL, v_len, _audDsp->notch2_coeffs, _audDsp->notch2_state[0]);
            if (v_dsp.iir_hpf.en) dsps_biquad_f32_aes3(p_outL, p_outL, v_len, _audDsp->iir_hpf_coeffs, _audDsp->iir_hpf_state[0]);
            if (v_dsp.iir_lpf.en) dsps_biquad_f32_aes3(p_outL, p_outL, v_len, _audDsp->iir_lpf_coeffs, _audDsp->iir_lpf_state[0]);
            if (v_dsp.hpf.en) safe_dsps_fir_f32(&_audDsp->fir_inst_hpf[0], p_outL, p_outL, v_len);
            if (v_dsp.lpf.en) safe_dsps_fir_f32(&_audDsp->fir_inst_lpf[0], p_outL, p_outL, v_len);
            if (p_audCfg.noise.gate_en) _applyNoiseGate(p_outL, v_len, p_audCfg.noise.gate_thresh);
        }
        if (p_outR && p_outL) memcpy(p_outR, p_outL, v_len * sizeof(float));

    } else {
        // [처리 단위 2] 빔포밍 미사용 (개별 좌/우 마이크 독립 필터링 처리)
        uint32_t v_len = (p_len > T2_Def::Audio::Sensor::FFT_SIZE_MAX) ? T2_Def::Audio::Sensor::FFT_SIZE_MAX : p_len;

        // LEFT 마이크 채널
        if (p_audL && p_outL) {
            memcpy(p_outL, p_audL, v_len * sizeof(float));
            _removeDC(p_outL, v_len);

            if (v_dsp.med_en) {
                _applyMedianFilter(p_outL, _audDsp->median_hist[0], v_dsp.med_win, v_len);
            }
            if (p_audCfg.pre_en) {
                _applyPreEmphasis(p_outL, _audDsp->prev_pre_emp[0], v_len, p_audCfg.pre_alpha);
            }
            if (v_dsp.notch.en) {
                dsps_biquad_f32_aes3(p_outL, p_outL, v_len, _audDsp->notch_coeffs, _audDsp->notch_state[0]);
            }
            if (v_dsp.notch2.en) {
                dsps_biquad_f32_aes3(p_outL, p_outL, v_len, _audDsp->notch2_coeffs, _audDsp->notch2_state[0]);
            }
            if (v_dsp.iir_hpf.en) {
                dsps_biquad_f32_aes3(p_outL, p_outL, v_len, _audDsp->iir_hpf_coeffs, _audDsp->iir_hpf_state[0]);
            }
            if (v_dsp.iir_lpf.en) {
                dsps_biquad_f32_aes3(p_outL, p_outL, v_len, _audDsp->iir_lpf_coeffs, _audDsp->iir_lpf_state[0]);
            }
            if (v_dsp.hpf.en) {
                safe_dsps_fir_f32(&_audDsp->fir_inst_hpf[0], p_outL, p_outL, v_len);
            }
            if (v_dsp.lpf.en) {
                safe_dsps_fir_f32(&_audDsp->fir_inst_lpf[0], p_outL, p_outL, v_len);
            }
            if (p_audCfg.noise.gate_en) {
                _applyNoiseGate(p_outL, v_len, p_audCfg.noise.gate_thresh);
            }
        } else if (p_outL) {
            memset(p_outL, 0, v_len * sizeof(float));
        }

        // RIGHT 마이크 채널
        if (p_audR && p_outR) {
            memcpy(p_outR, p_audR, v_len * sizeof(float));
            _removeDC(p_outR, v_len);

            if (v_dsp.med_en) {
                _applyMedianFilter(p_outR, _audDsp->median_hist[1], v_dsp.med_win, v_len);
            }
            if (p_audCfg.pre_en) {
                _applyPreEmphasis(p_outR, _audDsp->prev_pre_emp[1], v_len, p_audCfg.pre_alpha);
            }
            if (v_dsp.notch.en) {
                dsps_biquad_f32_aes3(p_outR, p_outR, v_len, _audDsp->notch_coeffs, _audDsp->notch_state[1]);
            }
            if (v_dsp.notch2.en) {
                dsps_biquad_f32_aes3(p_outR, p_outR, v_len, _audDsp->notch2_coeffs, _audDsp->notch2_state[1]);
            }
            if (v_dsp.iir_hpf.en) {
                dsps_biquad_f32_aes3(p_outR, p_outR, v_len, _audDsp->iir_hpf_coeffs, _audDsp->iir_hpf_state[1]);
            }
            if (v_dsp.iir_lpf.en) {
                dsps_biquad_f32_aes3(p_outR, p_outR, v_len, _audDsp->iir_lpf_coeffs, _audDsp->iir_lpf_state[1]);
            }
            if (v_dsp.hpf.en) {
                safe_dsps_fir_f32(&_audDsp->fir_inst_hpf[1], p_outR, p_outR, v_len);
            }
            if (v_dsp.lpf.en) {
                safe_dsps_fir_f32(&_audDsp->fir_inst_lpf[1], p_outR, p_outR, v_len);
            }
            if (p_audCfg.noise.gate_en) {
                _applyNoiseGate(p_outR, v_len, p_audCfg.noise.gate_thresh);
            }
        } else if (p_outR) {
            memset(p_outR, 0, v_len * sizeof(float));
        }
    }
}

/**
 * @brief 가속도 3축 개별 데이터 필터링 연산 파이프라인
 * @param p_inX, p_inY, p_inZ 입력 신호
 * @param p_outX, p_outY, p_outZ 결과 출력
 * @param p_len 데이터 크기
 * @param p_accCfg 가속도 세부 설정
 */
void CL_T2_DspEngine::processAccel(const float* p_inX, const float* p_inY, const float* p_inZ,
                                  float* p_outX, float* p_outY, float* p_outZ, uint32_t p_len,
                                  const T2_Type::ST_Accel_Config_t& p_accCfg) {
    if (!_isInitialized || !_accDsp) return;

    float* v_out[3] = {p_outX, p_outY, p_outZ};
    const float* v_in[3] = {p_inX, p_inY, p_inZ};
    const T2_Type::ST_Dsp_Config_t& v_dsp = p_accCfg.dsp;

    for (int i = 0; i < T2_Def::Accel::Sensor::AXIS_MAX; i++) {
        if (!v_in[i] || !v_out[i]) continue;

        memcpy(v_out[i], v_in[i], p_len * sizeof(float));

        // 1. Median 필터링
        if (v_dsp.med_en) {
            _applyMedianFilter(v_out[i], _accDsp->median_hist[i], v_dsp.med_win, p_len);
        }

        // 2. IIR Notch 필터링 (가속도 공용 notch_coeffs 사용)
        if (v_dsp.notch.en) {
            dsps_biquad_f32_aes3(v_out[i], v_out[i], p_len, _accDsp->notch_coeffs, _accDsp->notch_state[i]);
        }
        if (v_dsp.notch2.en) {
            dsps_biquad_f32_aes3(v_out[i], v_out[i], p_len, _accDsp->notch2_coeffs, _accDsp->notch2_state[i]);
        }

        // 3. IIR HPF/LPF
        if (v_dsp.iir_hpf.en) {
            dsps_biquad_f32_aes3(v_out[i], v_out[i], p_len, _accDsp->iir_hpf_coeffs, _accDsp->iir_hpf_state[i]);
        }
        if (v_dsp.iir_lpf.en) {
            dsps_biquad_f32_aes3(v_out[i], v_out[i], p_len, _accDsp->iir_lpf_coeffs, _accDsp->iir_lpf_state[i]);
        }

        // 4. FIR HPF/LPF
        if (v_dsp.hpf.en) {
            safe_dsps_fir_f32(&_accDsp->fir_inst_hpf[i], v_out[i], v_out[i], p_len);
        }
        if (v_dsp.lpf.en) {
            safe_dsps_fir_f32(&_accDsp->fir_inst_lpf[i], v_out[i], v_out[i], p_len);
        }

        // 5. DC 성분 차감
        if (v_dsp.rem_dc) {
            _removeDC(v_out[i], p_len);
        }
    }
}

/**
 * @brief 자이로 3축 개별 데이터 필터링 연산 파이프라인
 * @param p_inX, p_inY, p_inZ 입력 신호
 * @param p_outX, p_outY, p_outZ 결과 출력
 * @param p_len 데이터 크기
 * @param p_gyrCfg 자이로 세부 설정
 */
void CL_T2_DspEngine::processGyro(const float* p_inX, const float* p_inY, const float* p_inZ,
                                 float* p_outX, float* p_outY, float* p_outZ, uint32_t p_len,
                                  const T2_Type::ST_Gyro_Config_t& p_gyrCfg) {
    if (!_isInitialized || !_gyrDsp) return;

    float* v_out[3] = {p_outX, p_outY, p_outZ};
    const float* v_in[3] = {p_inX, p_inY, p_inZ};
    const T2_Type::ST_Dsp_Config_t& v_dsp = p_gyrCfg.dsp;

    for (int i = 0; i < T2_Def::Gyro::Sensor::AXIS_MAX; i++) {
        if (!v_in[i] || !v_out[i]) continue;

        memcpy(v_out[i], v_in[i], p_len * sizeof(float));

        // 1. Median 필터링
        if (v_dsp.med_en) {
            _applyMedianFilter(v_out[i], _gyrDsp->median_hist[i], v_dsp.med_win, p_len);
        }

        // 2. IIR Notch 필터링 (자이로 공용 notch_coeffs 사용)
        if (v_dsp.notch.en) {
            dsps_biquad_f32_aes3(v_out[i], v_out[i], p_len, _gyrDsp->notch_coeffs, _gyrDsp->notch_state[i]);
        }
        if (v_dsp.notch2.en) {
            dsps_biquad_f32_aes3(v_out[i], v_out[i], p_len, _gyrDsp->notch2_coeffs, _gyrDsp->notch2_state[i]);
        }

        // 3. IIR HPF/LPF
        if (v_dsp.iir_hpf.en) {
            dsps_biquad_f32_aes3(v_out[i], v_out[i], p_len, _gyrDsp->iir_hpf_coeffs, _gyrDsp->iir_hpf_state[i]);
        }
        if (v_dsp.iir_lpf.en) {
            dsps_biquad_f32_aes3(v_out[i], v_out[i], p_len, _gyrDsp->iir_lpf_coeffs, _gyrDsp->iir_lpf_state[i]);
        }

        // 4. FIR HPF/LPF
        if (v_dsp.hpf.en) {
            safe_dsps_fir_f32(&_gyrDsp->fir_inst_hpf[i], v_out[i], v_out[i], p_len);
        }
        if (v_dsp.lpf.en) {
            safe_dsps_fir_f32(&_gyrDsp->fir_inst_lpf[i], v_out[i], v_out[i], p_len);
        }

        // 5. DC 성분 차감 (자이로 드리프트 누적 억제)
        if (v_dsp.rem_dc) {
            _removeDC(v_out[i], p_len);
        }
    }
}

/**
 * @brief FIR 저역 통과 필터(LPF) 설계 및 계수 산출 (Blackman 창 함수 적용)
 */
void CL_T2_DspEngine::_generateFirLpf(float* p_coeffs, uint16_t p_taps, float p_cutoffHz, float p_sampleRate) {
    float v_ft = p_cutoffHz / p_sampleRate;
    for (int i = 0; i < p_taps; i++) {
        float n = (float)i - (float)(p_taps - 1) / 2.0f;
        if (fabsf(n) < T2_Def::Global::System::MATH_EPSILON_12_CONST) {
            p_coeffs[i] = 2.0f * v_ft;
        } else {
            p_coeffs[i] = sinf(2.0f * (float)M_PI * v_ft * n) / ((float)M_PI * n);
        }
        float w = 0.42f - 0.5f * cosf(2.0f * (float)M_PI * i / (float)(p_taps - 1))
                        + 0.08f * cosf(4.0f * (float)M_PI * i / (float)(p_taps - 1));
        p_coeffs[i] *= w;
    }
}

/**
 * @brief LPF 설계를 바탕으로 주파수 천이를 통한 고역 통과 필터(HPF) 설계
 */
void CL_T2_DspEngine::_generateFirHpf(float* p_coeffs, uint16_t p_taps, float p_cutoffHz, float p_sampleRate) {
    _generateFirLpf(p_coeffs, p_taps, p_cutoffHz, p_sampleRate);
    for (int i = 0; i < p_taps; i++) {
        p_coeffs[i] = -p_coeffs[i];
    }
    p_coeffs[(p_taps - 1) / 2] += 1.0f;
}

/**
 * @brief IIR HPF/LPF Biquad 계수 연산 (esp-dsp 빌트인 제너레이터 활용)
 */
void CL_T2_DspEngine::_calcIirCoeffs(float p_freq, float p_q, float* p_coeffs, uint32_t p_sampleRate, bool p_isHpf) {
    if (p_isHpf) dsps_biquad_gen_hpf_f32(p_coeffs, p_freq / p_sampleRate, p_q);
    else dsps_biquad_gen_lpf_f32(p_coeffs, p_freq / p_sampleRate, p_q);
}

/**
 * @brief 특정 주파수의 IIR Notch 계수 연산 (esp-dsp notch 제너레이터 활용)
 */
void CL_T2_DspEngine::_calcNotchCoeffs(float p_freq, float p_q, float* p_coeffs, uint32_t p_sampleRate) {
    dsps_biquad_gen_notch_f32(p_coeffs, p_freq / (float)p_sampleRate, -60.0f, p_q);
}

/**
 * @brief 메디안(Median) 임시 정렬 필터 적용을 통한 충격성 노이즈 억제 (삽입 정렬 사용)
 */
void CL_T2_DspEngine::_applyMedianFilter(float* p_data, float* p_hist, uint8_t p_windowSize, uint32_t p_len) {
    uint8_t v_ws = p_windowSize;
    if (v_ws > T2_Def::Global::Dsp::MEDIAN_WINDOW_MAX)
        v_ws = T2_Def::Global::Dsp::MEDIAN_WINDOW_MAX;
    if (v_ws == 0) v_ws = 1;

    for (uint32_t i = 0; i < p_len; i++) {
        for (int k = v_ws - 1; k > 0; k--) p_hist[k] = p_hist[k - 1];
        p_hist[0] = p_data[i];

        float v_sort[T2_Def::Global::Dsp::MEDIAN_WINDOW_MAX];
        memcpy(v_sort, p_hist, v_ws * sizeof(float));

        // 고속 삽입 정렬 적용
        for (int j = 1; j < v_ws; j++) {
            float key = v_sort[j];
            int m = j - 1;
            while (m >= 0 && v_sort[m] > key) {
                v_sort[m + 1] = v_sort[m];
                m--;
            }
            v_sort[m + 1] = key;
        }
        p_data[i] = v_sort[v_ws / 2];
    }
}

/**
 * @brief 프리엠파시스(Pre-Emphasis) 고역 보정 필터링 적용 (고주파 성분 증폭)
 */
void CL_T2_DspEngine::_applyPreEmphasis(float* p_data, float& p_prevSample, uint32_t p_len, float p_alpha) {
    for (uint32_t i = 0; i < p_len; i++) {
        float current = p_data[i];
        p_data[i] = current - p_alpha * p_prevSample;
        p_prevSample = current;
    }
}

/**
 * @brief 입력 데이터 진폭이 임계치 미만일 시 0으로 제어하는 노이즈 게이트 적용
 */
void CL_T2_DspEngine::_applyNoiseGate(float* p_data, uint32_t p_len, float p_gateThresh) {
    for (uint32_t i = 0; i < p_len; i++) {
         if (fabsf(p_data[i]) < p_gateThresh) p_data[i] = 0.0f;
    }
}

/**
 * @brief 데이터셋의 평균값을 산출하여 전체 바이어스를 해제하는 DC offset 제거 적용
 */
void CL_T2_DspEngine::_removeDC(float* p_data, uint32_t p_len) {
    float v_sum = 0.0f;
    for (uint32_t i = 0; i < p_len; i++) {
        if (std::isnan(p_data[i]) || std::isinf(p_data[i])) p_data[i] = 0.0f;
        v_sum += p_data[i];
    }
    float v_mean = v_sum / (float)p_len;
    for (uint32_t i = 0; i < p_len; i++) p_data[i] -= v_mean;
}
