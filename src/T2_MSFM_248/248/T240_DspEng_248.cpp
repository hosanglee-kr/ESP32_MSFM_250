/* ============================================================================
 * File: T240_DspEng_248.cpp
 * Summary: 멀티모달(진동/소음) SIMD 고속 신호 처리 엔진 구현부 (v245 개정판)
 * ============================================================================ */

#include "T240_DspEng_248.hpp"
#include "esp_heap_caps.h"
#include "dsps_fft2r.h"
#include "esp_log.h"
#include <cmath>
#include <cstring>


static const char* TAG = "T245_DSP";

inline void safe_dsps_fir_f32(fir_f32_t* fir, const float* input, float* output, int len) {
    if (fir->N % 4 == 0) {
        dsps_fir_f32_aes3(fir, input, output, len);
    } else {
        dsps_fir_f32(fir, input, output, len);
    }
}

// 함수설명: DSP 엔진 생성자이며, 초기화 상태 플래그 및 포인터 변수들을 NULL로 리셋합니다.
CL_T2_DspEngine::CL_T2_DspEngine() {
    _isInitialized = false;
    _audDsp = nullptr;
    _accDsp = nullptr;
    _gyrDsp = nullptr;
    _capBufL = nullptr;
    _capBufR = nullptr;
    _prcBufL = nullptr;
    _prcBufR = nullptr;
    _accBufX = nullptr;
    _accBufY = nullptr;
    _accBufZ = nullptr;
    _gyrBufX = nullptr;
    _gyrBufY = nullptr;
    _gyrBufZ = nullptr;
}

// 함수설명: DSP 엔진 소멸자이며, 동적으로 할당된 멀티모달 DSP 런타임 메모리 영역을 해제합니다.
CL_T2_DspEngine::~CL_T2_DspEngine() {
    if (_audDsp) { heap_caps_free(_audDsp); _audDsp = nullptr; }
    if (_accDsp) { heap_caps_free(_accDsp); _accDsp = nullptr; }
    if (_gyrDsp) { heap_caps_free(_gyrDsp); _gyrDsp = nullptr; }

    if (_capBufL) { heap_caps_free(_capBufL); _capBufL = nullptr; }
    if (_capBufR) { heap_caps_free(_capBufR); _capBufR = nullptr; }
    if (_prcBufL) { heap_caps_free(_prcBufL); _prcBufL = nullptr; }
    if (_prcBufR) { heap_caps_free(_prcBufR); _prcBufR = nullptr; }

    if (_accBufX) { heap_caps_free(_accBufX); _accBufX = nullptr; }
    if (_accBufY) { heap_caps_free(_accBufY); _accBufY = nullptr; }
    if (_accBufZ) { heap_caps_free(_accBufZ); _accBufZ = nullptr; }
    if (_gyrBufX) { heap_caps_free(_gyrBufX); _gyrBufX = nullptr; }
    if (_gyrBufY) { heap_caps_free(_gyrBufY); _gyrBufY = nullptr; }
    if (_gyrBufZ) { heap_caps_free(_gyrBufZ); _gyrBufZ = nullptr; }
}

// 함수설명: 설정 구조체를 기반으로 마이크, 가속도, 자이로 채널별 DSP 메모리 할당 및 창 함수를 초기 배정합니다. (p_cfg: 설정 스냅샷 구조체, 반환값: 초기화 성공 여부)
bool CL_T2_DspEngine::init(const T2_Type::ST_DynamicConfig_t& p_cfg) {
    // esp-dsp FFT 하드웨어 가속/테이블 초기화 (누락 시 Core Panic 방지)
    esp_err_t v_fftRet = dsps_fft2r_init_fc32(NULL, T2_Def::Audio::Sensor::FFT_SIZE_MAX);
    if (v_fftRet != ESP_OK) {
        ESP_LOGE(TAG, "esp-dsp FFT Initialization Failed! Code: %d", v_fftRet);
    }

    size_t v_fSizeAudio = sizeof(float) * T2_Def::Audio::Sensor::FFT_SIZE_MAX;
    size_t v_fSizeVib   = sizeof(float) * T2_Def::Accel::Sensor::FFT_SIZE_MAX;

    // [처리 단위 1] 마이크 센서용 DSP 메모리 동적 할당 및 윈도우 초기화
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

        if (!_capBufL) _capBufL = (float*)heap_caps_aligned_alloc(16, v_fSizeAudio, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!_prcBufL) _prcBufL = (float*)heap_caps_aligned_alloc(16, v_fSizeAudio, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!_capBufR) _capBufR = (float*)heap_caps_aligned_alloc(16, v_fSizeAudio, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!_prcBufR) _prcBufR = (float*)heap_caps_aligned_alloc(16, v_fSizeAudio, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    } else {
        if (_audDsp) { heap_caps_free(_audDsp); _audDsp = nullptr; }
        if (_capBufL) { heap_caps_free(_capBufL); _capBufL = nullptr; }
        if (_prcBufL) { heap_caps_free(_prcBufL); _prcBufL = nullptr; }
        if (_capBufR) { heap_caps_free(_capBufR); _capBufR = nullptr; }
        if (_prcBufR) { heap_caps_free(_prcBufR); _prcBufR = nullptr; }
    }

    // [처리 단위 2] 가속도 센서용 DSP 메모리 동적 할당 및 윈도우 초기화
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

    // [처리 단위 3] 자이로 센서용 DSP 메모리 동적 할당 및 윈도우 초기화
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

    // IMU 관련 통합 버퍼 할당
    if (p_cfg.accel.enable || p_cfg.gyro.enable) {
        if (!_accBufX) _accBufX = (float*)heap_caps_aligned_alloc(16, v_fSizeVib, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!_accBufY) _accBufY = (float*)heap_caps_aligned_alloc(16, v_fSizeVib, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!_accBufZ) _accBufZ = (float*)heap_caps_aligned_alloc(16, v_fSizeVib, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!_gyrBufX) _gyrBufX = (float*)heap_caps_aligned_alloc(16, v_fSizeVib, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!_gyrBufY) _gyrBufY = (float*)heap_caps_aligned_alloc(16, v_fSizeVib, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!_gyrBufZ) _gyrBufZ = (float*)heap_caps_aligned_alloc(16, v_fSizeVib, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    } else {
        if (_accBufX) { heap_caps_free(_accBufX); _accBufX = nullptr; }
        if (_accBufY) { heap_caps_free(_accBufY); _accBufY = nullptr; }
        if (_accBufZ) { heap_caps_free(_accBufZ); _accBufZ = nullptr; }
        if (_gyrBufX) { heap_caps_free(_gyrBufX); _gyrBufX = nullptr; }
        if (_gyrBufY) { heap_caps_free(_gyrBufY); _gyrBufY = nullptr; }
        if (_gyrBufZ) { heap_caps_free(_gyrBufZ); _gyrBufZ = nullptr; }
    }

    reloadFilters(p_cfg);
    resetStates();

    _isInitialized = true;
    ESP_LOGI(TAG, "DSP Engine Initialized (Aud:%d, Acc:%d, Gyr:%d)",
             (int)(_audDsp != nullptr), (int)(_accDsp != nullptr), (int)(_gyrDsp != nullptr));
    return true;
}

// 함수설명: 오디오, 가속도, 자이로별로 주파수 차단대역 및 차수 조건에 맞춰 필터 계수들을 재생성 재배치합니다. (p_cfg: 재생성용 설정 정보)
void CL_T2_DspEngine::reloadFilters(const T2_Type::ST_DynamicConfig_t& p_cfg) {
    // [처리 단위 1] 마이크 오디오 채널의 대역 필터 계수 갱신
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

    // [처리 단위 2] 가속도 진동 센서 채널의 대역 필터 계수 갱신
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
            _generateFirHpf(_accDsp->fir_hpf_coeffs, hpf_taps, v_dsp.hpf.cutoff, p_accSampleRate);
            for (int i = 0; i < T2_Def::Accel::Sensor::AXIS_MAX; i++) {
                memset(_accDsp->fir_state_hpf[i], 0, sizeof(_accDsp->fir_state_hpf[i]));
                dsps_fir_init_f32(&_accDsp->fir_inst_hpf[i], _accDsp->fir_hpf_coeffs, _accDsp->fir_state_hpf[i], hpf_taps);
            }
        }
        if (v_dsp.lpf.en && v_dsp.lpf.taps > 0) {
            _generateFirLpf(_accDsp->fir_lpf_coeffs, v_dsp.lpf.taps, v_dsp.lpf.cutoff, p_accSampleRate);
            for (int i = 0; i < T2_Def::Accel::Sensor::AXIS_MAX; i++) {
                memset(_accDsp->fir_state_lpf[i], 0, sizeof(_accDsp->fir_state_lpf[i]));
                dsps_fir_init_f32(&_accDsp->fir_inst_lpf[i], _accDsp->fir_lpf_coeffs, _accDsp->fir_state_lpf[i], v_dsp.lpf.taps);
            }
        }
    }

    // [처리 단위 3] 자이로 센서 채널의 대역 필터 계수 갱신
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
            _generateFirHpf(_gyrDsp->fir_hpf_coeffs, hpf_taps, v_dsp.hpf.cutoff, p_gyrSampleRate);
            for (int i = 0; i < T2_Def::Gyro::Sensor::AXIS_MAX; i++) {
                memset(_gyrDsp->fir_state_hpf[i], 0, sizeof(_gyrDsp->fir_state_hpf[i]));
                dsps_fir_init_f32(&_gyrDsp->fir_inst_hpf[i], _gyrDsp->fir_hpf_coeffs, _gyrDsp->fir_state_hpf[i], hpf_taps);
            }
        }
        if (v_dsp.lpf.en && v_dsp.lpf.taps > 0) {
            _generateFirLpf(_gyrDsp->fir_lpf_coeffs, v_dsp.lpf.taps, v_dsp.lpf.cutoff, p_gyrSampleRate);
            for (int i = 0; i < T2_Def::Gyro::Sensor::AXIS_MAX; i++) {
                memset(_gyrDsp->fir_state_lpf[i], 0, sizeof(_gyrDsp->fir_state_lpf[i]));
                dsps_fir_init_f32(&_gyrDsp->fir_inst_lpf[i], _gyrDsp->fir_lpf_coeffs, _gyrDsp->fir_state_lpf[i], v_dsp.lpf.taps);
            }
        }
    }

    uint16_t v_taps = 0;
    if (_accDsp) v_taps = p_cfg.accel.dsp.hpf.taps;
    else if (_audDsp) v_taps = p_cfg.audio.dsp.hpf.taps;
    ESP_LOGI(TAG, "Filters Reloaded (Taps:%d)", v_taps);
}

// 함수설명: 할당된 오디오, 가속도, 자이로 DSP 필터 인스턴스들의 상태 레지스터(과거 샘플 버퍼)를 모두 0으로 비웁니다.
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


// 함수설명: 빔포밍 연산 모드 여부에 맞춰 입력 사운드 파형 데이터를 받아 빔포밍 합성 또는 좌우 채널별 개별 노이즈게이트 및 대역 처리를 가합니다. (p_audL/R: 수신 오디오, p_outL/R: 출력 대상 버퍼, p_len: 크기, p_audCfg: 마이크 센서 설정)
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

    if (v_useBeam) {
        // [처리 단위 1] 빔포밍 모드 적용: 좌우 채널 합성 후 단일 채널 필터 라인 처리 및 복사
        uint32_t v_len = (p_len > T2_Def::Audio::Sensor::FFT_SIZE_MAX) ? T2_Def::Audio::Sensor::FFT_SIZE_MAX : p_len;

        // 실시간 온도를 반영한 정밀 음속 계산 (m/s) 및 공간 지연 보정
        float v_soundSpeed = 331.5f + 0.6f * _tempC;
        float v_correctedBeamGain = p_audCfg.beam_gain * (340.0f / v_soundSpeed); // 340m/s(상온 기준) 기준 스케일 보정

        if (p_outL) {
            for (uint32_t i = 0; i < v_len; i++) {
                p_outL[i] = (p_audL[i] + p_audR[i]) * v_correctedBeamGain;
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
        // R채널: memcpy(p_outR, p_outL) 덮어쓰기 제거
        // 공간 지표(IPD, Coh)를 위해 R채널의 원본 위상을 보존하되, 최소한의 DC 오프셋만 제거하여 제공
        if (p_outR && p_audR) {
            memcpy(p_outR, p_audR, v_len * sizeof(float));
            _removeDC(p_outR, v_len);
        }
    } else {
        // [처리 단위 2] 일반 모드 적용: L 및 R 독립 필터링 파이프라인 구동
        uint32_t v_len = (p_len > T2_Def::Audio::Sensor::FFT_SIZE_MAX) ? T2_Def::Audio::Sensor::FFT_SIZE_MAX : p_len;

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

// 함수설명: 가속도 센서 3축 신호에 대해 메디안 정렬 필터, 노치 거부, 대역 IIR 및 FIR 필터, DC 오프셋 제거를 차례대로 병렬 처리 수행합니다. (p_inX/Y/Z: 원시 입력, p_outX/Y/Z: 출력 대상, p_len: 샘플 길이, p_accCfg: 센서 사양 구조체)
void CL_T2_DspEngine::processAccel(const float* p_inX, const float* p_inY, const float* p_inZ,
                                  float* p_outX, float* p_outY, float* p_outZ, uint32_t p_len,
                                  const T2_Type::ST_Accel_Config_t& p_accCfg) {
    if (!_isInitialized || !_accDsp) return;

    float* v_out[3] = {p_outX, p_outY, p_outZ};
    const float* v_in[3] = {p_inX, p_inY, p_inZ};
    const T2_Type::ST_Dsp_Config_t& v_dsp = p_accCfg.dsp;

    // [처리 단위 1] 가속도 각 축별 5단계 필터 체인(Median -> Notch -> IIR HPF/LPF -> FIR -> DC) 가동
    for (int i = 0; i < T2_Def::Accel::Sensor::AXIS_MAX; i++) {
        if (!v_in[i] || !v_out[i]) continue;

        memcpy(v_out[i], v_in[i], p_len * sizeof(float));

        if (v_dsp.med_en) {
            _applyMedianFilter(v_out[i], _accDsp->median_hist[i], v_dsp.med_win, p_len);
        }

        if (v_dsp.notch.en) {
            dsps_biquad_f32_aes3(v_out[i], v_out[i], p_len, _accDsp->notch_coeffs, _accDsp->notch_state[i]);
        }
        if (v_dsp.notch2.en) {
            dsps_biquad_f32_aes3(v_out[i], v_out[i], p_len, _accDsp->notch2_coeffs, _accDsp->notch2_state[i]);
        }

        if (v_dsp.iir_hpf.en) {
            dsps_biquad_f32_aes3(v_out[i], v_out[i], p_len, _accDsp->iir_hpf_coeffs, _accDsp->iir_hpf_state[i]);
        }
        if (v_dsp.iir_lpf.en) {
            dsps_biquad_f32_aes3(v_out[i], v_out[i], p_len, _accDsp->iir_lpf_coeffs, _accDsp->iir_lpf_state[i]);
        }

        if (v_dsp.hpf.en) {
            safe_dsps_fir_f32(&_accDsp->fir_inst_hpf[i], v_out[i], v_out[i], p_len);
        }
        if (v_dsp.lpf.en) {
            safe_dsps_fir_f32(&_accDsp->fir_inst_lpf[i], v_out[i], v_out[i], p_len);
        }

        if (v_dsp.rem_dc) {
            _removeDC(v_out[i], p_len);
        }
    }
}

// 함수설명: 자이로 센서 3축 신호에 대해 메디안 정렬 필터, 노치 거부, 대역 IIR 및 FIR 필터, DC 오프셋 제거를 차례대로 병렬 처리 수행합니다. (p_inX/Y/Z: 원시 입력, p_outX/Y/Z: 출력 대상, p_len: 샘플 길이, p_gyrCfg: 센서 사양 구조체)
void CL_T2_DspEngine::processGyro(const float* p_inX, const float* p_inY, const float* p_inZ,
                                 float* p_outX, float* p_outY, float* p_outZ, uint32_t p_len,
                                  const T2_Type::ST_Gyro_Config_t& p_gyrCfg) {
    if (!_isInitialized || !_gyrDsp) return;

    float* v_out[3] = {p_outX, p_outY, p_outZ};
    const float* v_in[3] = {p_inX, p_inY, p_inZ};
    const T2_Type::ST_Dsp_Config_t& v_dsp = p_gyrCfg.dsp;

    // [처리 단위 1] 자이로 각 축별 5단계 필터 체인(Median -> Notch -> IIR HPF/LPF -> FIR -> DC) 가동
    for (int i = 0; i < T2_Def::Gyro::Sensor::AXIS_MAX; i++) {
        if (!v_in[i] || !v_out[i]) continue;

        memcpy(v_out[i], v_in[i], p_len * sizeof(float));

        if (v_dsp.med_en) {
            _applyMedianFilter(v_out[i], _gyrDsp->median_hist[i], v_dsp.med_win, p_len);
        }

        if (v_dsp.notch.en) {
            dsps_biquad_f32_aes3(v_out[i], v_out[i], p_len, _gyrDsp->notch_coeffs, _gyrDsp->notch_state[i]);
        }
        if (v_dsp.notch2.en) {
            dsps_biquad_f32_aes3(v_out[i], v_out[i], p_len, _gyrDsp->notch2_coeffs, _gyrDsp->notch2_state[i]);
        }

        if (v_dsp.iir_hpf.en) {
            dsps_biquad_f32_aes3(v_out[i], v_out[i], p_len, _gyrDsp->iir_hpf_coeffs, _gyrDsp->iir_hpf_state[i]);
        }
        if (v_dsp.iir_lpf.en) {
            dsps_biquad_f32_aes3(v_out[i], v_out[i], p_len, _gyrDsp->iir_lpf_coeffs, _gyrDsp->iir_lpf_state[i]);
        }

        if (v_dsp.hpf.en) {
            safe_dsps_fir_f32(&_gyrDsp->fir_inst_hpf[i], v_out[i], v_out[i], p_len);
        }
        if (v_dsp.lpf.en) {
            safe_dsps_fir_f32(&_gyrDsp->fir_inst_lpf[i], v_out[i], v_out[i], p_len);
        }

        if (v_dsp.rem_dc) {
            _removeDC(v_out[i], p_len);
        }
    }
}

// 함수설명: 주파수 스펙트럼 윈도윙용 LPF FIR 필터의 계수 배열 데이터를 생성합니다. (p_coeffs: 출력 버퍼, p_taps: 차수, p_cutoffHz: 차단주파수, p_sampleRate: 샘플율)
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

// 함수설명: 주파수 스펙트럼 윈도윙용 HPF FIR 필터의 계수 배열 데이터를 생성합니다. (p_coeffs: 출력 버퍼, p_taps: 차수, p_cutoffHz: 차단주파수, p_sampleRate: 샘플율)
void CL_T2_DspEngine::_generateFirHpf(float* p_coeffs, uint16_t p_taps, float p_cutoffHz, float p_sampleRate) {
    _generateFirLpf(p_coeffs, p_taps, p_cutoffHz, p_sampleRate);
    for (int i = 0; i < p_taps; i++) {
        p_coeffs[i] = -p_coeffs[i];
    }
    p_coeffs[(p_taps - 1) / 2] += 1.0f;
}

// 함수설명: IIR 대역 필터(HPF 또는 LPF)의 바이쿼드 계수를 연산하여 배열에 채워넣습니다. (p_freq: 컷오프Hz, p_q: Q인자, p_coeffs: 계수 출력 대상, p_sampleRate: 샘플율, p_isHpf: 고역통과 필터 여부)
void CL_T2_DspEngine::_calcIirCoeffs(float p_freq, float p_q, float* p_coeffs, uint32_t p_sampleRate, bool p_isHpf) {
    if (p_isHpf) dsps_biquad_gen_hpf_f32(p_coeffs, p_freq / p_sampleRate, p_q);
    else dsps_biquad_gen_lpf_f32(p_coeffs, p_freq / p_sampleRate, p_q);
}

// 함수설명: Notch 대역 필터의 바이쿼드 차단 계수를 연산하여 배열에 채워넣습니다. (p_freq: 표적Hz, p_q: Q인자, p_coeffs: 계수 출력 대상, p_sampleRate: 샘플율)
void CL_T2_DspEngine::_calcNotchCoeffs(float p_freq, float p_q, float* p_coeffs, uint32_t p_sampleRate) {
    dsps_biquad_gen_notch_f32(p_coeffs, p_freq / (float)p_sampleRate, -60.0f, p_q);
}

// 함수설명: 대역 내 급격한 고주파 피크성 스파이크 노이즈를 제거하기 위해 삽입 정렬 기반의 1차원 메디안 필터를 구동합니다. (p_data: 정규화 대상 데이터군, p_hist: 메디안 윈도우용 이력 데이터 배열, p_windowSize: 창 크기, p_len: 샘플 길이)
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

// 함수설명: 오디오 주파수 대역의 고주파 성분을 증폭 강조하기 위해 프리엠파시스 수식을 수행합니다. (p_data: 입력 배열 데이터, p_prevSample: 직전 주기 샘플 값 참조, p_len: 크기, p_alpha: 엠파시스 계수)
void CL_T2_DspEngine::_applyPreEmphasis(float* p_data, float& p_prevSample, uint32_t p_len, float p_alpha) {
    for (uint32_t i = 0; i < p_len; i++) {
        float current = p_data[i];
        p_data[i] = current - p_alpha * p_prevSample;
        p_prevSample = current;
    }
}

// 함수설명: 수집한 데이터 중 오디오 임계 전력 레벨에 못 미치는 진폭을 소거 처리합니다. (p_data: 대상 데이터 버퍼, p_len: 크기, p_gateThresh: 임계 진폭 크기)
void CL_T2_DspEngine::_applyNoiseGate(float* p_data, uint32_t p_len, float p_gateThresh) {
    for (uint32_t i = 0; i < p_len; i++) {
         if (fabsf(p_data[i]) < p_gateThresh) p_data[i] = 0.0f;
    }
}

// 함수설명: 배열 내부의 잘못된 NaN/INF 값을 무효화하고 단일 패스로 평균 오프셋 DC 성분을 소거합니다. (p_data: 대상 데이터 버퍼, p_len: 크기)
void CL_T2_DspEngine::_removeDC(float* p_data, uint32_t p_len) {
    float v_sum = 0.0f;
    for (uint32_t i = 0; i < p_len; i++) {
        p_data[i] = SMEA_SAN_FLOAT(p_data[i]);
        v_sum += p_data[i];
    }
    float v_mean = v_sum / (float)p_len;
    for (uint32_t i = 0; i < p_len; i++) p_data[i] -= v_mean;
}
