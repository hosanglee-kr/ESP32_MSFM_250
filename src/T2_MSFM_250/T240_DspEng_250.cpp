/* ============================================================================
 * File: T240_DspEng_250.cpp
 * Summary: 멀티모달(진동/소음) SIMD 고속 신호 처리 엔진 구현부
 * ============================================================================ */

#include "T240_DspEng_250.hpp"
#include "esp_heap_caps.h"
#include "dsps_fft2r.h"
#include "esp_log.h"
#include <cmath>
#include <cstring>


static const char* TAG = "T240_DSP";

// --- 1/3 Octave band bin index map (32 bands x 3 resolutions x 2 indices = 192 bytes, 내부 SRAM 상주) ---
const uint16_t g_T2_40_Dsp_BandBinMap_arr[192] G_T2_10_Def_SRAM_ATTR = {
    // 1024 FFT 예시 (start, end bin)
    2, 2,  3, 3,  4, 4,  5, 5,  6, 6,  7, 8,  9, 10, 11, 13,
    14, 16, 17, 20, 21, 25, 26, 32, 33, 40, 41, 50, 51, 63, 64, 79,
    // 2048 FFT 예시
    4, 4,  6, 6,  8, 8,  10, 10, 12, 12, 14, 16, 18, 20, 22, 26,
    28, 32, 34, 40, 42, 50, 52, 64, 66, 80, 82, 100, 102, 126, 128, 158,
    // 4096 FFT 예시
    8, 8,  12, 12, 16, 16, 20, 20, 24, 24, 28, 32, 36, 40, 44, 52,
    56, 64, 68, 80, 84, 100, 104, 128, 132, 160, 164, 200, 204, 252, 256, 316
};

// --- 31차 힐버트 FIR 필터 계수 정의 (.rodata Flash XIP 상주) ---
const float g_T2_40_Dsp_HilbertCoeffs_arr[31] G_T2_10_Def_FLASH_ATTR_RODATA = {
    -0.0051f, 0.0f, -0.0084f, 0.0f, -0.0145f, 0.0f, -0.0262f, 0.0f,
    -0.0513f, 0.0f, -0.1132f, 0.0f, -0.3183f, 0.0f, 0.0f, 0.0f,
    0.3183f, 0.0f, 0.1132f, 0.0f, 0.0513f, 0.0f, 0.0262f, 0.0f,
    0.0145f, 0.0f, 0.0084f, 0.0f, 0.0051f
};

inline void T2_40_Dsp_safe_dsps_fir_f32(fir_f32_t* fir, const float* input, float* output, int len) {
    if (fir->N % 4 == 0) {
        dsps_fir_f32_aes3(fir, input, output, len);
    } else {
        dsps_fir_f32(fir, input, output, len);
    }
}

CL_T2_DspEngine::CL_T2_DspEngine() {
    _isInitialized = false;

    _audDsp = nullptr;                  // 오디오 DSP 런타임
    _accDsp = nullptr;                  // 가속도 DSP 런타임
    _gyrDsp = nullptr;                  // 자이로 DSP 런타임

    _capBufL = nullptr;                 // 오디오 캡처 버퍼 L
    _capBufR = nullptr;                 // 오디오 캡처 버퍼 R
    _prcBufL = nullptr;                 // 오디오 프로세스 버퍼 L
    _prcBufR = nullptr;                 // 오디오 프로세스 버퍼 R

    _accBufX = nullptr;                 // 가속도 버퍼 X
    _accBufY = nullptr;                 // 가속도 버퍼 Y
    _accBufZ = nullptr;                 // 가속도 버퍼 Z

    _gyrBufX = nullptr;                 // 자이로 버퍼 X
    _gyrBufY = nullptr;                 // 자이로 버퍼 Y
    _gyrBufZ = nullptr;                 // 자이로 버퍼 Z

    _accHilbertEnvX = nullptr;          // 가속도 힐버트 변환 버퍼 X
    _accHilbertEnvY = nullptr;          // 가속도 힐버트 변환 버퍼 Y
    _accHilbertEnvZ = nullptr;          // 가속도 힐버트 변환 버퍼 Z
}

// DSP 엔진 소멸자이며, 동적으로 할당된 멀티모달 DSP 런타임 메모리 영역을 해제함
CL_T2_DspEngine::~CL_T2_DspEngine() {

    // 오디오 DSP 메모리 해제
    if (_audDsp) { heap_caps_free(_audDsp); _audDsp = nullptr; }
    // 가속도 DSP 메모리 해제
    if (_accDsp) { heap_caps_free(_accDsp); _accDsp = nullptr; }
    // 자이로 DSP 메모리 해제
    if (_gyrDsp) { heap_caps_free(_gyrDsp); _gyrDsp = nullptr; }

    // 캡처 버퍼 메모리 해제
    if (_capBufL) { heap_caps_free(_capBufL); _capBufL = nullptr; }
    if (_capBufR) { heap_caps_free(_capBufR); _capBufR = nullptr; }

    // 프로세스 버퍼 메모리 해제
    if (_prcBufL) { heap_caps_free(_prcBufL); _prcBufL = nullptr; }
    if (_prcBufR) { heap_caps_free(_prcBufR); _prcBufR = nullptr; }

    // 가속도 센서 버퍼 메모리 해제
    if (_accBufX) { heap_caps_free(_accBufX); _accBufX = nullptr; }
    if (_accBufY) { heap_caps_free(_accBufY); _accBufY = nullptr; }
    if (_accBufZ) { heap_caps_free(_accBufZ); _accBufZ = nullptr; }

    // 자이로 센서 버퍼 메모리 해제
    if (_gyrBufX) { heap_caps_free(_gyrBufX); _gyrBufX = nullptr; }
    if (_gyrBufY) { heap_caps_free(_gyrBufY); _gyrBufY = nullptr; }
    if (_gyrBufZ) { heap_caps_free(_gyrBufZ); _gyrBufZ = nullptr; }

    // 힐베르트 변환 메모리 해제
    if (_accHilbertEnvX) { heap_caps_free(_accHilbertEnvX); _accHilbertEnvX = nullptr; }
    if (_accHilbertEnvY) { heap_caps_free(_accHilbertEnvY); _accHilbertEnvY = nullptr; }
    if (_accHilbertEnvZ) { heap_caps_free(_accHilbertEnvZ); _accHilbertEnvZ = nullptr; }
}

// 설정 구조체를 기반으로 마이크, 가속도, 자이로 채널별 DSP 메모리 할당 및 창 함수를 초기 배정합니다. (p_cfg: 설정 스냅샷 구조체, 반환값: 초기화 성공 여부)
bool CL_T2_DspEngine::init(const T2_Type::ST_DynamicConfig_t& p_cfg) {

    // esp-dsp FFT 하드웨어 가속/테이블 초기화 (누락 시 Core Panic 방지)
    esp_err_t v_fftRet = dsps_fft2r_init_fc32(NULL, T2_Def::Audio::Sensor::FFT_SIZE_MAX);
    if (v_fftRet != ESP_OK) {
        ESP_LOGE(TAG, "esp-dsp FFT Initialization Failed! Code: %d", v_fftRet);
    }

    // 사용할 메모리 크기 계산
    // 마이크 센서용 메모리 크기
    size_t v_fSizeAudio = sizeof(float) * T2_Def::Audio::Sensor::FFT_SIZE_MAX;
    // 가속도 및 자이로 공통 메모리 크기
    size_t v_fSizeVib   = sizeof(float) * T2_Def::Accel::Sensor::FFT_SIZE_MAX;

    // [처리 단위 1] 마이크 센서용 DSP 메모리 동적 할당 및 윈도우 초기화
    if (p_cfg.audio.enable) {
        // 마이크 DSP 런타임 구조체 메모리 할당 (IRAM 우선, 실패 시 SPIRAM 사용)
        if (!_audDsp) {
            // IRAM 우선으로 할당하고, 실패 시 SPIRAM 사용
            _audDsp = (ST_AudioDspRuntime*)heap_caps_aligned_alloc(16, sizeof(ST_AudioDspRuntime), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
            if (!_audDsp) {
                _audDsp = (ST_AudioDspRuntime*)heap_caps_aligned_alloc(16, sizeof(ST_AudioDspRuntime), MALLOC_CAP_SPIRAM);
            }
        }
        // 윈도우 함수 초기화
        if (_audDsp) {
            switch (p_cfg.audio.dsp.win_type) {
                // 윈도우 타입 설정
                case T2_Type::EM_WindowType_t::HAMMING:
                    dsps_wind_hann_f32(_audDsp->window, p_cfg.audio.fft_size);
                    break;
                case T2_Type::EM_WindowType_t::BLACKMAN:
                    dsps_wind_blackman_f32(_audDsp->window, p_cfg.audio.fft_size);
                    break;
                default:
                    dsps_wind_hann_f32(_audDsp->window, p_cfg.audio.fft_size);
                    break;
            }
        }

        // 캡처 버퍼 메모리 할당
        if (!_capBufL) _capBufL = (float*)heap_caps_aligned_alloc(16, v_fSizeAudio, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!_capBufR) _capBufR = (float*)heap_caps_aligned_alloc(16, v_fSizeAudio, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        // 프로세스 버퍼 메모리 할당
        if (!_prcBufL) _prcBufL = (float*)heap_caps_aligned_alloc(16, v_fSizeAudio, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!_prcBufR) _prcBufR = (float*)heap_caps_aligned_alloc(16, v_fSizeAudio, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    } else {
        // 오디오 DSP 메모리 해제
        if (_audDsp) { heap_caps_free(_audDsp); _audDsp = nullptr; }
        // 캡처 버퍼 메모리 해제
        if (_capBufL) { heap_caps_free(_capBufL); _capBufL = nullptr; }
        if (_capBufR) { heap_caps_free(_capBufR); _capBufR = nullptr; }
        // 프로세스 버퍼 메모리 해제
        if (_prcBufR) { heap_caps_free(_prcBufR); _prcBufR = nullptr; }
        if (_prcBufL) { heap_caps_free(_prcBufL); _prcBufL = nullptr; }
    }

    // [처리 단위 2] 가속도 센서용 DSP 메모리 동적 할당 및 윈도우 초기화
    if (p_cfg.accel.enable) {
        // 가속도 DSP 런타임 구조체 메모리 할당 (IRAM 우선, 실패 시 SPIRAM 사용)
        if (!_accDsp) {
            _accDsp = (ST_AccelDspRuntime*)heap_caps_aligned_alloc(16, sizeof(ST_AccelDspRuntime), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
            if (!_accDsp) {
                _accDsp = (ST_AccelDspRuntime*)heap_caps_aligned_alloc(16, sizeof(ST_AccelDspRuntime), MALLOC_CAP_SPIRAM);
            }
        }
        // 윈도우 함수 초기화
        if (_accDsp) {
            dsps_wind_hann_f32(_accDsp->window, p_cfg.accel.fft_size);

            // 힐버트 FIR 필터 인스턴스 초기화
            for (int i = 0; i < T2_Def::Accel::Sensor::AXIS_MAX; i++) {
                memset(_accDsp->fir_state_hilbert[i], 0, sizeof(_accDsp->fir_state_hilbert[i]));
                dsps_fir_init_f32(&_accDsp->fir_inst_hilbert[i], const_cast<float*>(g_T2_40_Dsp_HilbertCoeffs_arr), _accDsp->fir_state_hilbert[i], T2_Def::Accel::FeatureLimit::HILBERT_FIR_TAPS);
            }
        }
    } else {
        // 가속도 DSP 메모리 해제
        if (_accDsp) { heap_caps_free(_accDsp); _accDsp = nullptr; }
    }

    // [처리 단위 3] 자이로 센서용 DSP 메모리 동적 할당 및 윈도우 초기화
    if (p_cfg.gyro.enable) {
        // 자이로 DSP 런타임 구조체 메모리 할당 (IRAM 우선, 실패 시 SPIRAM 사용)
        if (!_gyrDsp) {
            _gyrDsp = (ST_GyroDspRuntime*)heap_caps_aligned_alloc(16, sizeof(ST_GyroDspRuntime), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
            if (!_gyrDsp) {
                _gyrDsp = (ST_GyroDspRuntime*)heap_caps_aligned_alloc(16, sizeof(ST_GyroDspRuntime), MALLOC_CAP_SPIRAM);
            }
        }
        // 윈도우 함수 초기화
        if (_gyrDsp) {
            switch (p_cfg.gyro.dsp.win_type) {
                case T2_Type::EM_WindowType_t::HAMMING:  dsps_wind_hann_f32(_gyrDsp->window, p_cfg.gyro.fft_size);  break;
                case T2_Type::EM_WindowType_t::BLACKMAN: dsps_wind_blackman_f32(_gyrDsp->window, p_cfg.gyro.fft_size); break;
                default:                            dsps_wind_hann_f32(_gyrDsp->window, p_cfg.gyro.fft_size);     break;
            }
        }
    } else {
        // 자이로 DSP 메모리 해제
        if (_gyrDsp) { heap_caps_free(_gyrDsp); _gyrDsp = nullptr; }
    }

    // IMU 관련 통합 버퍼 할당
    if (p_cfg.accel.enable || p_cfg.gyro.enable) {
        // 가속도 버퍼 메모리 할당
        if (!_accBufX) _accBufX = (float*)heap_caps_aligned_alloc(16, v_fSizeVib, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!_accBufY) _accBufY = (float*)heap_caps_aligned_alloc(16, v_fSizeVib, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!_accBufZ) _accBufZ = (float*)heap_caps_aligned_alloc(16, v_fSizeVib, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        // 자이로 버퍼 메모리 할당
        if (!_gyrBufX) _gyrBufX = (float*)heap_caps_aligned_alloc(16, v_fSizeVib, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!_gyrBufY) _gyrBufY = (float*)heap_caps_aligned_alloc(16, v_fSizeVib, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!_gyrBufZ) _gyrBufZ = (float*)heap_caps_aligned_alloc(16, v_fSizeVib, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

        // 힐버트 포락선 버퍼 할당
        if (!_accHilbertEnvX) _accHilbertEnvX = (float*)heap_caps_aligned_alloc(16, v_fSizeVib, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!_accHilbertEnvY) _accHilbertEnvY = (float*)heap_caps_aligned_alloc(16, v_fSizeVib, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!_accHilbertEnvZ) _accHilbertEnvZ = (float*)heap_caps_aligned_alloc(16, v_fSizeVib, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    } else {
        // 가속도 버퍼 메모리 해제
        if (_accBufX) { heap_caps_free(_accBufX); _accBufX = nullptr; }
        if (_accBufY) { heap_caps_free(_accBufY); _accBufY = nullptr; }
        if (_accBufZ) { heap_caps_free(_accBufZ); _accBufZ = nullptr; }
        // 자이로 버퍼 메모리 해제
        if (_gyrBufX) { heap_caps_free(_gyrBufX); _gyrBufX = nullptr; }
        if (_gyrBufY) { heap_caps_free(_gyrBufY); _gyrBufY = nullptr; }
        if (_gyrBufZ) { heap_caps_free(_gyrBufZ); _gyrBufZ = nullptr; }

        // 힐버트 포락선 메모리 해제
        if (_accHilbertEnvX) { heap_caps_free(_accHilbertEnvX); _accHilbertEnvX = nullptr; }
        if (_accHilbertEnvY) { heap_caps_free(_accHilbertEnvY); _accHilbertEnvY = nullptr; }
        if (_accHilbertEnvZ) { heap_caps_free(_accHilbertEnvZ); _accHilbertEnvZ = nullptr; }
    }

    // 필터 계수 갱신
    reloadFilters(p_cfg);
    // 상태 변수 리셋
    resetStates();

    _isInitialized = true;
    ESP_LOGI(TAG, "DSP Engine Initialized (Aud:%d, Acc:%d, Gyr:%d)",
             (int)(_audDsp != nullptr), (int)(_accDsp != nullptr), (int)(_gyrDsp != nullptr));
    return true;
}

// 오디오, 가속도, 자이로별로 주파수 차단대역 및 차수 조건에 맞춰 필터 계수들을 재생성 재배치합니다. (p_cfg: 재생성용 설정 정보)
void CL_T2_DspEngine::reloadFilters(const T2_Type::ST_DynamicConfig_t& p_cfg) {
    // [처리 단위 1] 마이크 오디오 채널의 대역 필터 계수 갱신
    if (_audDsp) {
        // 오디오 샘플 레이트
        uint32_t p_audSampleRate = p_cfg.audio.sample_rate;
        // 오디오 DSP 설정
        const T2_Type::ST_Dsp_Config_t& v_dsp = p_cfg.audio.dsp;

        //notch 필터 설정
        if (v_dsp.notch.en) {
            _calcNotchCoeffs(v_dsp.notch.freq, v_dsp.notch.q, _audDsp->notch_coeffs, p_audSampleRate);
        }
        //notch2 필터 설정
        if (v_dsp.notch2.en) {
            _calcNotchCoeffs(v_dsp.notch2.freq, v_dsp.notch2.q, _audDsp->notch2_coeffs, p_audSampleRate);
        }
        //iir_hpf 필터 설정
        if (v_dsp.iir_hpf.en) {
            _calcIirCoeffs(v_dsp.iir_hpf.cutoff, v_dsp.iir_hpf.q, _audDsp->iir_hpf_coeffs, p_audSampleRate, true);
        }
        //iir_lpf 필터 설정
        if (v_dsp.iir_lpf.en) {
            _calcIirCoeffs(v_dsp.iir_lpf.cutoff, v_dsp.iir_lpf.q, _audDsp->iir_lpf_coeffs, p_audSampleRate, false);
        }

		//hpf 필터 설정
		uint16_t hpf_taps = v_dsp.hpf.taps;
		if (v_dsp.hpf.en && (hpf_taps % 2 == 0)) {
			ESP_LOGW(TAG, "FIR HPF taps must be odd, adjusting %d -> %d", hpf_taps, hpf_taps+1);
			hpf_taps += 1;
		}

        //hpf 필터 계수 생성
        if (v_dsp.hpf.en && hpf_taps > 0) {
            // hpf 필터 계수 생성
            _generateFirHpf(_audDsp->fir_hpf_coeffs, hpf_taps, v_dsp.hpf.cutoff, (float)p_audSampleRate);
            for (int ch = 0; ch < 2; ch++) {
                // hpf 필터 상태 변수 초기화
                memset(_audDsp->fir_state_hpf[ch], 0, sizeof(_audDsp->fir_state_hpf[ch]));
                // hpf 필터 인스턴스 초기화
                dsps_fir_init_f32(&_audDsp->fir_inst_hpf[ch], _audDsp->fir_hpf_coeffs, _audDsp->fir_state_hpf[ch], hpf_taps);
            }
        }
        //lpf 필터 설정
        if (v_dsp.lpf.en && v_dsp.lpf.taps > 0) {
            // lpf 필터 계수 생성
            _generateFirLpf(_audDsp->fir_lpf_coeffs, v_dsp.lpf.taps, v_dsp.lpf.cutoff, p_audSampleRate);
            for (int ch = 0; ch < 2; ch++) {
                // lpf 필터 상태 변수 초기화
                memset(_audDsp->fir_state_lpf[ch], 0, sizeof(_audDsp->fir_state_lpf[ch]));
                // lpf 필터 인스턴스 초기화
                dsps_fir_init_f32(&_audDsp->fir_inst_lpf[ch], _audDsp->fir_lpf_coeffs, _audDsp->fir_state_lpf[ch], v_dsp.lpf.taps);
            }
        }
    }

    // [처리 단위 2] 가속도 진동 센서 채널의 대역 필터 계수 갱신
    if (_accDsp) {
        // 가속도 샘플 레이트
        uint32_t p_accSampleRate = p_cfg.accel.sample_rate;
        // 가속도 DSP 설정
        const T2_Type::ST_Dsp_Config_t& v_dsp = p_cfg.accel.dsp;

        //notch 필터 설정
        if (v_dsp.notch.en) {
            _calcNotchCoeffs(v_dsp.notch.freq, v_dsp.notch.q, _accDsp->notch_coeffs, p_accSampleRate);
        }
        //notch2 필터 설정
        if (v_dsp.notch2.en) {
            _calcNotchCoeffs(v_dsp.notch2.freq, v_dsp.notch2.q, _accDsp->notch2_coeffs, p_accSampleRate);
        }
        //iir_hpf 필터 설정
        if (v_dsp.iir_hpf.en) {
            _calcIirCoeffs(v_dsp.iir_hpf.cutoff, v_dsp.iir_hpf.q, _accDsp->iir_hpf_coeffs, p_accSampleRate, true);
        }
        //iir_lpf 필터 설정
        if (v_dsp.iir_lpf.en) {
            _calcIirCoeffs(v_dsp.iir_lpf.cutoff, v_dsp.iir_lpf.q, _accDsp->iir_lpf_coeffs, p_accSampleRate, false);
        }

		//hpf 필터 설정
		uint16_t hpf_taps = v_dsp.hpf.taps;
		if (v_dsp.hpf.en && (hpf_taps % 2 == 0)) {
			ESP_LOGW(TAG, "FIR HPF taps must be odd, adjusting %d -> %d", hpf_taps, hpf_taps+1);
			hpf_taps += 1;
		}

        //hpf 필터 계수 생성
        if (v_dsp.hpf.en && hpf_taps > 0) {
            _generateFirHpf(_accDsp->fir_hpf_coeffs, hpf_taps, v_dsp.hpf.cutoff, p_accSampleRate);
            for (int i = 0; i < T2_Def::Accel::Sensor::AXIS_MAX; i++) {
                // hpf 필터 상태 변수 초기화
                memset(_accDsp->fir_state_hpf[i], 0, sizeof(_accDsp->fir_state_hpf[i]));
                // hpf 필터 인스턴스 초기화
                dsps_fir_init_f32(&_accDsp->fir_inst_hpf[i], _accDsp->fir_hpf_coeffs, _accDsp->fir_state_hpf[i], hpf_taps);
            }
        }
        //lpf 필터 설정
        if (v_dsp.lpf.en && v_dsp.lpf.taps > 0) {
            // lpf 필터 계수 생성
            _generateFirLpf(_accDsp->fir_lpf_coeffs, v_dsp.lpf.taps, v_dsp.lpf.cutoff, p_accSampleRate);
            for (int i = 0; i < T2_Def::Accel::Sensor::AXIS_MAX; i++) {
                // lpf 필터 상태 변수 초기화
                memset(_accDsp->fir_state_lpf[i], 0, sizeof(_accDsp->fir_state_lpf[i]));
                // lpf 필터 인스턴스 초기화
                dsps_fir_init_f32(&_accDsp->fir_inst_lpf[i], _accDsp->fir_lpf_coeffs, _accDsp->fir_state_lpf[i], v_dsp.lpf.taps);
            }
        }
    }

    // [처리 단위 3] 자이로 센서 채널의 대역 필터 계수 갱신
    if (_gyrDsp) {
        // 자이로 샘플 레이트
        uint32_t p_gyrSampleRate = p_cfg.gyro.sample_rate;
        // 자이로 DSP 설정
        const T2_Type::ST_Dsp_Config_t& v_dsp = p_cfg.gyro.dsp;

        //notch 필터 설정
        if (v_dsp.notch.en) {
            _calcNotchCoeffs(v_dsp.notch.freq, v_dsp.notch.q, _gyrDsp->notch_coeffs, p_gyrSampleRate);
        }
        //notch2 필터 설정
        if (v_dsp.notch2.en) {
            _calcNotchCoeffs(v_dsp.notch2.freq, v_dsp.notch2.q, _gyrDsp->notch2_coeffs, p_gyrSampleRate);
        }
        //iir_hpf 필터 설정
        if (v_dsp.iir_hpf.en) {
            _calcIirCoeffs(v_dsp.iir_hpf.cutoff, v_dsp.iir_hpf.q, _gyrDsp->iir_hpf_coeffs, p_gyrSampleRate, true);
        }
        //iir_lpf 필터 설정
        if (v_dsp.iir_lpf.en) {
            _calcIirCoeffs(v_dsp.iir_lpf.cutoff, v_dsp.iir_lpf.q, _gyrDsp->iir_lpf_coeffs, p_gyrSampleRate, false);
        }
    }

    // hpf 필터 탭 수 확인
    uint16_t v_taps = 0;
    if (_accDsp) v_taps = p_cfg.accel.dsp.hpf.taps;
    else if (_audDsp) v_taps = p_cfg.audio.dsp.hpf.taps;
    // 탭 수 출력
    ESP_LOGI(TAG, "Filters Reloaded (Taps:%d)", v_taps);
}

// 할당된 오디오, 가속도, 자이로 DSP 필터 인스턴스들의 상태 레지스터(과거 샘플 버퍼)를 모두 0으로 비웁니다.
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
        // 힐버트 필터 지연 버퍼 초기화
        memset(_accDsp->fir_state_hilbert, 0, sizeof(_accDsp->fir_state_hilbert));
    }
    if (_gyrDsp) {
        memset(_gyrDsp->median_hist, 0, sizeof(_gyrDsp->median_hist));
        memset(_gyrDsp->notch_state, 0, sizeof(_gyrDsp->notch_state));
        memset(_gyrDsp->notch2_state, 0, sizeof(_gyrDsp->notch2_state));
        memset(_gyrDsp->iir_hpf_state, 0, sizeof(_gyrDsp->iir_hpf_state));
        memset(_gyrDsp->iir_lpf_state, 0, sizeof(_gyrDsp->iir_lpf_state));
    }
}


// 빔포밍 연산 모드 여부에 맞춰 입력 사운드 파형 데이터를 받아 빔포밍 합성 또는 좌우 채널별 개별 노이즈게이트 및 대역 처리를 가합니다. (p_audL/R: 수신 오디오, p_outL/R: 출력 대상 버퍼, p_len: 크기, p_audCfg: 마이크 센서 설정)
void CL_T2_DspEngine::processAudio(const float* p_audL, const float* p_audR, float* p_outL, float* p_outR, uint32_t p_len,
                                  const T2_Type::ST_Audio_Config_t& p_audCfg) {
    if (!_isInitialized || !_audDsp) return;

    // 빔포밍 모드 변경 감지
	static bool s_prevBeamActive = false;
	bool v_beamActive = (p_audL && p_audR && p_audCfg.beam_gain > 0.001f);

	if (v_beamActive != s_prevBeamActive) {
        // 빔포밍 모드 변경 시 상태 변수 초기화
		resetStates();
        // 이전 빔포밍 모드 상태 갱신
		s_prevBeamActive = v_beamActive;
	}
    // 빔포밍 모드 설정
    bool v_useBeam = (p_audL && p_audR && p_audCfg.beam_gain > 0.001f);

    const T2_Type::ST_Dsp_Config_t& v_dsp = p_audCfg.dsp;

    // 빔포밍 모드인 경우
    if (v_useBeam) {
        // 빔포밍 모드 적용: 좌우 채널 합성 후 단일 채널 필터 라인 처리 및 복사
        uint32_t v_len = (p_len > T2_Def::Audio::Sensor::FFT_SIZE_MAX) ? T2_Def::Audio::Sensor::FFT_SIZE_MAX : p_len;

        // 실시간 온도를 반영한 정밀 음속 계산 (m/s) 및 공간 지연 보정
        float v_soundSpeed = 331.5f + 0.6f * _tempC;
        // 빔포밍 게인 스케일 보정
        float v_correctedBeamGain = p_audCfg.beam_gain * (340.0f / v_soundSpeed); // 340m/s(상온 기준) 기준 스케일 보정

        // 출력 버퍼가 존재하는 경우
        if (p_outL) {
            // 빔포밍 합성 및 게인 적용
            for (uint32_t i = 0; i < v_len; i++) {
                p_outL[i] = (p_audL[i] + p_audR[i]) * v_correctedBeamGain;
            }
            // DC 제거
            _removeDC(p_outL, v_len);

            // 미디언 필터
            if (v_dsp.med_en) _applyMedianFilter(p_outL, _audDsp->median_hist[0], v_dsp.med_win, v_len);
            // 프리 엠퍼시스
            if (p_audCfg.pre_en) _applyPreEmphasis(p_outL, _audDsp->prev_pre_emp[0], v_len, p_audCfg.pre_alpha);
            // 노치 필터
            if (v_dsp.notch.en)  dsps_biquad_f32_aes3(p_outL, p_outL, v_len, _audDsp->notch_coeffs, _audDsp->notch_state[0]);
            // 노치2 필터
            if (v_dsp.notch2.en) dsps_biquad_f32_aes3(p_outL, p_outL, v_len, _audDsp->notch2_coeffs, _audDsp->notch2_state[0]);
            // IIR 하이패스 필터
            if (v_dsp.iir_hpf.en) dsps_biquad_f32_aes3(p_outL, p_outL, v_len, _audDsp->iir_hpf_coeffs, _audDsp->iir_hpf_state[0]);
            // IIR 로우패스 필터
            if (v_dsp.iir_lpf.en) dsps_biquad_f32_aes3(p_outL, p_outL, v_len, _audDsp->iir_lpf_coeffs, _audDsp->iir_lpf_state[0]);
            // FIR 하이패스 필터
            if (v_dsp.hpf.en) T2_40_Dsp_safe_dsps_fir_f32(&_audDsp->fir_inst_hpf[0], p_outL, p_outL, v_len);
            // FIR 로우패스 필터
            if (v_dsp.lpf.en) T2_40_Dsp_safe_dsps_fir_f32(&_audDsp->fir_inst_lpf[0], p_outL, p_outL, v_len);
            // 노이즈 게이트
            if (p_audCfg.noise.gate_en) _applyNoiseGate(p_outL, v_len, p_audCfg.noise.gate_thresh);
        }
        // 빔포밍 모드인 경우, R채널은 최소한의 DC 제거만 수행하여 공간 지표 계산용으로 제공
        if (p_outR && p_audR) {
            memcpy(p_outR, p_audR, v_len * sizeof(float));
            _removeDC(p_outR, v_len);
        }
    } else {
        // 일반 모드: L 및 R 독립 필터링 파이프라인 구동
        uint32_t v_len = (p_len > T2_Def::Audio::Sensor::FFT_SIZE_MAX) ? T2_Def::Audio::Sensor::FFT_SIZE_MAX : p_len;

        // L채널
        if (p_audL && p_outL) {
            // 입력 데이터 복사
            memcpy(p_outL, p_audL, v_len * sizeof(float));
            // DC 제거
            _removeDC(p_outL, v_len);

            // 미디언 필터
            if (v_dsp.med_en) {
                _applyMedianFilter(p_outL, _audDsp->median_hist[0], v_dsp.med_win, v_len);
            }
            // 프리 엠퍼시스
            if (p_audCfg.pre_en) {
                _applyPreEmphasis(p_outL, _audDsp->prev_pre_emp[0], v_len, p_audCfg.pre_alpha);
            }
            // 노치 필터
            if (v_dsp.notch.en) {
                dsps_biquad_f32_aes3(p_outL, p_outL, v_len, _audDsp->notch_coeffs, _audDsp->notch_state[0]);
            }
            // 노치2 필터
            if (v_dsp.notch2.en) {
                dsps_biquad_f32_aes3(p_outL, p_outL, v_len, _audDsp->notch2_coeffs, _audDsp->notch2_state[0]);
            }
            // IIR 하이패스 필터
            if (v_dsp.iir_hpf.en) {
                dsps_biquad_f32_aes3(p_outL, p_outL, v_len, _audDsp->iir_hpf_coeffs, _audDsp->iir_hpf_state[0]);
            }
            // IIR 로우패스 필터
            if (v_dsp.iir_lpf.en) {
                dsps_biquad_f32_aes3(p_outL, p_outL, v_len, _audDsp->iir_lpf_coeffs, _audDsp->iir_lpf_state[0]);
            }
            // FIR 하이패스 필터
            if (v_dsp.hpf.en) {
                T2_40_Dsp_safe_dsps_fir_f32(&_audDsp->fir_inst_hpf[0], p_outL, p_outL, v_len);
            }
            // FIR 로우패스 필터
            if (v_dsp.lpf.en) {
                T2_40_Dsp_safe_dsps_fir_f32(&_audDsp->fir_inst_lpf[0], p_outL, p_outL, v_len);
            }
            // 노이즈 게이트
            if (p_audCfg.noise.gate_en) {
                _applyNoiseGate(p_outL, v_len, p_audCfg.noise.gate_thresh);
            }
        } else if (p_outL) {
            memset(p_outL, 0, v_len * sizeof(float));
        }

        // R채널
        if (p_audR && p_outR) {
            // 입력 데이터 복사
            memcpy(p_outR, p_audR, v_len * sizeof(float));
            // DC 제거
            _removeDC(p_outR, v_len);

            // 미디언 필터
            if (v_dsp.med_en) {
                _applyMedianFilter(p_outR, _audDsp->median_hist[1], v_dsp.med_win, v_len);
            }
            // 프리 엠퍼시스
            if (p_audCfg.pre_en) {
                _applyPreEmphasis(p_outR, _audDsp->prev_pre_emp[1], v_len, p_audCfg.pre_alpha);
            }
            // 노치 필터
            if (v_dsp.notch.en) {
                dsps_biquad_f32_aes3(p_outR, p_outR, v_len, _audDsp->notch_coeffs, _audDsp->notch_state[1]);
            }
            // 노치2 필터
            if (v_dsp.notch2.en) {
                dsps_biquad_f32_aes3(p_outR, p_outR, v_len, _audDsp->notch2_coeffs, _audDsp->notch2_state[1]);
            }
            if (v_dsp.iir_hpf.en) {
                dsps_biquad_f32_aes3(p_outR, p_outR, v_len, _audDsp->iir_hpf_coeffs, _audDsp->iir_hpf_state[1]);
            }
            // IIR 로우패스 필터
            if (v_dsp.iir_lpf.en) {
                dsps_biquad_f32_aes3(p_outR, p_outR, v_len, _audDsp->iir_lpf_coeffs, _audDsp->iir_lpf_state[1]);
            }
            // FIR 하이패스 필터
            if (v_dsp.hpf.en) {
                T2_40_Dsp_safe_dsps_fir_f32(&_audDsp->fir_inst_hpf[1], p_outR, p_outR, v_len);
            }
            // FIR 로우패스 필터
            if (v_dsp.lpf.en) {
                T2_40_Dsp_safe_dsps_fir_f32(&_audDsp->fir_inst_lpf[1], p_outR, p_outR, v_len);
            }
            // 노이즈 게이트
            if (p_audCfg.noise.gate_en) {
                _applyNoiseGate(p_outR, v_len, p_audCfg.noise.gate_thresh);
            }
        } else if (p_outR) {
            memset(p_outR, 0, v_len * sizeof(float));
        }
    }
}


// 가속도 센서 3축 신호에 대해 메디안 정렬 필터, 노치 거부, 대역 IIR 및 FIR 필터, DC 오프셋 제거를 차례대로 병렬 처리 수행합니다. (p_inX/Y/Z: 원시 입력, p_outX/Y/Z: 출력 대상, p_len: 샘플 길이, p_accCfg: 센서 사양 구조체)
void CL_T2_DspEngine::processAccel(const  float* p_inX, const float* p_inY, const float* p_inZ,
                                          float* p_outX,      float* p_outY,      float* p_outZ, uint32_t p_len,
                                  const T2_Type::ST_Accel_Config_t& p_accCfg) {
    if (!_isInitialized || !_accDsp) return;

    // 가속도 각 축별 포인터 및 설정값 준비
    float*       v_out[3] = {p_outX, p_outY, p_outZ};     // 출력 버퍼 포인터 배열
    const float* v_in[3]  = {p_inX,  p_inY,  p_inZ};      // 입력 버퍼 포인터 배열
    
    const T2_Type::ST_Dsp_Config_t& v_dsp = p_accCfg.dsp; // DSP 설정 구조체

    // 가속도 각 축별 5단계 필터 체인(Median -> Notch -> IIR HPF/LPF -> FIR -> DC) 가동
    for (int i = 0; i < T2_Def::Accel::Sensor::AXIS_MAX; i++) {
        // 입력 또는 출력 버퍼가 없는 경우 건너뜀
        if (!v_in[i] || !v_out[i]) continue;

        // 입력 데이터 복사
        memcpy(v_out[i], v_in[i], p_len * sizeof(float));
        
        // DC 제거 1차
        _removeDC(v_out[i], p_len);

        // 적응형 2단계 메디안 필터링 : 돌발적인 임펄스 노이즈(스파이크)를 제거
        // - 풀스케일 95% 초과 데이터만 메디안필터 적용, 그 외 바이패스)
        if (v_dsp.med_en) {
            
            // 풀스케일 범위 계산
            float v_rawFullScale = (float)p_accCfg.range;
            // 95% 임계값 계산
            float v_esdThresh = v_rawFullScale * 0.95f;

            // 1차 메디안 필터용 임시 버퍼 준비
            alignas(16) float v_tempFiltered[T2_Def::Accel::Sensor::FFT_SIZE_MAX];
            memcpy(v_tempFiltered, v_out[i], p_len * sizeof(float));

            // 1차 메디안 필터 임시 버퍼에 적용
            _applyMedianFilter(v_tempFiltered, _accDsp->median_hist[i], v_dsp.med_win, p_len);

            // 2차 메디안 필터 적용 (임계값 초과 데이터만 1차 메디안 필터 출력값으로 대체)
            for (uint32_t j = 0; j < p_len; j++) {
                if (fabsf(v_out[i][j]) > v_esdThresh) {
                    v_out[i][j] = v_tempFiltered[j];
                }
            }
        }

        // 노치 1차 필터 (바이쿼드 IIR 필터 방식
        // 특정 협대역 주파수(예: 50/60Hz 전원 노이즈 또는 특정 기계적 공진 주파수)를 정밀하게 제거
        if (v_dsp.notch.en) {
            dsps_biquad_f32_aes3(v_out[i], v_out[i], p_len, _accDsp->notch_coeffs, _accDsp->notch_state[i]);
        }
        // 노치 2차 필터
        if (v_dsp.notch2.en) {
            dsps_biquad_f32_aes3(v_out[i], v_out[i], p_len, _accDsp->notch2_coeffs, _accDsp->notch2_state[i]);
        }
        
        // IIR 하이패스 필터 : 저주파 드리프트(걷거나 흔들릴 때 발생하는 느린 중력 성분 변화)를 차단하여 순수한 동적 가속도만 추출
        if (v_dsp.iir_hpf.en) {
            dsps_biquad_f32_aes3(v_out[i], v_out[i], p_len, _accDsp->iir_hpf_coeffs, _accDsp->iir_hpf_state[i]);
        }
        // IIR 로우패스 필터 : 고주파 전기적 잡음이나 진동 노이즈를 제거하여 신호를 매끄럽게 만듦
        if (v_dsp.iir_lpf.en) {
            dsps_biquad_f32_aes3(v_out[i], v_out[i], p_len, _accDsp->iir_lpf_coeffs, _accDsp->iir_lpf_state[i]);
        }

        // FIR 하이패스 필터
        if (v_dsp.hpf.en) {
            T2_40_Dsp_safe_dsps_fir_f32(&_accDsp->fir_inst_hpf[i], v_out[i], v_out[i], p_len);
        }
        // FIR 로우패스 필터
        if (v_dsp.lpf.en) {
            T2_40_Dsp_safe_dsps_fir_f32(&_accDsp->fir_inst_lpf[i], v_out[i], v_out[i], p_len);
        }

        // DC 제거 2차 : 긴 필터 체인을 거치면서 연산 오차나 필터 과도 응답으로 인해 다시 DC 성분 제거
        if (v_dsp.rem_dc) {
            _removeDC(v_out[i], p_len);
        }

        // 힐버트 변환 진폭 포락선 추출 및 군지연 보정 (31차 FIR, 군지연 15)
        // 임시 버퍼 준비
        alignas(16) float v_hilbertPhaseShift[T2_Def::Accel::Sensor::FFT_SIZE_MAX] = {0};
        // 힐버트 변환
        T2_40_Dsp_safe_dsps_fir_f32(&_accDsp->fir_inst_hilbert[i], v_out[i], v_hilbertPhaseShift, p_len);

        // 군지연 보정
        const uint16_t v_delay = T2_Def::Accel::FeatureLimit::HILBERT_GROUP_DELAY;
        
        // 진폭 포락선 추출
        float* v_envDest = (i == 0) ? _accHilbertEnvX : ((i == 1) ? _accHilbertEnvY : _accHilbertEnvZ);
        
        // 힐버트 변환 데이터에서 군지연만큼 지연된 원본 데이터와 90도 위상 변환된 데이터를 이용하여 진폭 포락선 추출
        for (uint32_t j = 0; j < p_len; j++) {
            // 군지연만큼 지연된 원본 데이터 인덱스 계산
            int v_srcIdx = (int)j - (int)v_delay;
            // 지연된 원본 데이터
            float v_rawDelayed = (v_srcIdx >= 0) ? v_out[i][v_srcIdx] : 0.0f;
            // 90도 위상 변환된 데이터
            float v_shiftVal = v_hilbertPhaseShift[j];
            // 진폭 포락선 계산
            v_envDest[j] = sqrtf(v_rawDelayed * v_rawDelayed + v_shiftVal * v_shiftVal);
        }
    }
}

// 자이로 센서 3축 신호에 대해 메디안 정렬 필터, 노치 거부, 대역 IIR 및 FIR 필터, DC 오프셋 제거를 차례대로 병렬 처리 수행합니다. (p_inX/Y/Z: 원시 입력, p_outX/Y/Z: 출력 대상, p_len: 샘플 길이, p_gyrCfg: 센서 사양 구조체)
void CL_T2_DspEngine::processGyro(const float* p_inX, const float* p_inY, const float* p_inZ,
                                 float* p_outX, float* p_outY, float* p_outZ, uint32_t p_len,
                                  const T2_Type::ST_Gyro_Config_t& p_gyrCfg) {
    if (!_isInitialized || !_gyrDsp) return;

    // 자이로 각 축별 포인터 및 설정값 준비
    float*       v_out[3] = {p_outX, p_outY, p_outZ};     // 출력 버퍼 포인터 배열
    const float* v_in[3]  = {p_inX,  p_inY,  p_inZ};      // 입력 버퍼 포인터 배열
    const T2_Type::ST_Dsp_Config_t& v_dsp = p_gyrCfg.dsp; // DSP 설정 구조체

    // 자이로 각 축별 5단계 필터 체인(Median -> Notch -> IIR HPF/LPF -> FIR -> DC) 가동
    for (int i = 0; i < T2_Def::Gyro::Sensor::AXIS_MAX; i++) {
        // 입력 또는 출력 버퍼가 없는 경우 건너뜀
        if (!v_in[i] || !v_out[i]) continue;

        // 입력 데이터 복사
        memcpy(v_out[i], v_in[i], p_len * sizeof(float));

        // 메디안 필터
        if (v_dsp.med_en) {
            _applyMedianFilter(v_out[i], _gyrDsp->median_hist[i], v_dsp.med_win, p_len);
        }

        // 노치 필터
        if (v_dsp.notch.en) {
            dsps_biquad_f32_aes3(v_out[i], v_out[i], p_len, _gyrDsp->notch_coeffs, _gyrDsp->notch_state[i]);
        }
        // 노치2 필터
        if (v_dsp.notch2.en) {
            dsps_biquad_f32_aes3(v_out[i], v_out[i], p_len, _gyrDsp->notch2_coeffs, _gyrDsp->notch2_state[i]);
        }

        // IIR 하이패스 필터
        if (v_dsp.iir_hpf.en) {
            dsps_biquad_f32_aes3(v_out[i], v_out[i], p_len, _gyrDsp->iir_hpf_coeffs, _gyrDsp->iir_hpf_state[i]);
        }
        // IIR 로우패스 필터
        if (v_dsp.iir_lpf.en) {
            dsps_biquad_f32_aes3(v_out[i], v_out[i], p_len, _gyrDsp->iir_lpf_coeffs, _gyrDsp->iir_lpf_state[i]);
        }

        // DC 제거
        if (v_dsp.rem_dc) {
            _removeDC(v_out[i], p_len);
        }
    }
}

// 주파수 스펙트럼 윈도윙용 LPF FIR 필터의 계수 배열 데이터를 생성 (p_coeffs: 출력 버퍼, p_taps: 차수, p_cutoffHz: 차단주파수, p_sampleRate: 샘플율)
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

// 주파수 스펙트럼 윈도윙용 HPF FIR 필터의 계수 배열 데이터를 생성 (p_coeffs: 출력 버퍼, p_taps: 차수, p_cutoffHz: 차단주파수, p_sampleRate: 샘플율)
void CL_T2_DspEngine::_generateFirHpf(float* p_coeffs, uint16_t p_taps, float p_cutoffHz, float p_sampleRate) {
    _generateFirLpf(p_coeffs, p_taps, p_cutoffHz, p_sampleRate);
    for (int i = 0; i < p_taps; i++) {
        p_coeffs[i] = -p_coeffs[i];
    }
    p_coeffs[(p_taps - 1) / 2] += 1.0f;
}

// IIR 대역 필터(HPF 또는 LPF)의 바이쿼드 계수를 연산하여 배열에 채움 (p_freq: 컷오프Hz, p_q: Q인자, p_coeffs: 계수 출력 대상, p_sampleRate: 샘플율, p_isHpf: 고역통과 필터 여부)
void CL_T2_DspEngine::_calcIirCoeffs(float p_freq, float p_q, float* p_coeffs, uint32_t p_sampleRate, bool p_isHpf) {
    if (p_isHpf) dsps_biquad_gen_hpf_f32(p_coeffs, p_freq / p_sampleRate, p_q);
    else dsps_biquad_gen_lpf_f32(p_coeffs, p_freq / p_sampleRate, p_q);
}

// Notch 대역 필터의 바이쿼드 차단 계수를 연산하여 배열에 채움 (p_freq: 표적Hz, p_q: Q인자, p_coeffs: 계수 출력 대상, p_sampleRate: 샘플율)
void CL_T2_DspEngine::_calcNotchCoeffs(float p_freq, float p_q, float* p_coeffs, uint32_t p_sampleRate) {
    dsps_biquad_gen_notch_f32(p_coeffs, p_freq / (float)p_sampleRate, -60.0f, p_q);
}

// 대역 내 급격한 고주파 피크성 스파이크 노이즈를 제거하기 위해 삽입 정렬 기반의 1차원 메디안 필터를 구동 
//   (p_data: 정규화 대상 데이터군, p_hist: 메디안 윈도우용 이력 데이터 배열, p_windowSize: 창 크기, p_len: 샘플 길이)
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

// 오디오 주파수 대역의 고주파 성분을 증폭 강조하기 위해 프리엠파시스 수식을 수행 (p_data: 입력 배열 데이터, p_prevSample: 직전 주기 샘플 값 참조, p_len: 크기, p_alpha: 엠파시스 계수)
void CL_T2_DspEngine::_applyPreEmphasis(float* p_data, float& p_prevSample, uint32_t p_len, float p_alpha) {
    for (uint32_t i = 0; i < p_len; i++) {
        float current = p_data[i];
        p_data[i] = current - p_alpha * p_prevSample;
        p_prevSample = current;
    }
}

// 수집한 데이터 중 오디오 임계 전력 레벨에 못 미치는 진폭을 소거 처리 (p_data: 대상 데이터 버퍼, p_len: 크기, p_gateThresh: 임계 진폭 크기)
void CL_T2_DspEngine::_applyNoiseGate(float* p_data, uint32_t p_len, float p_gateThresh) {
    for (uint32_t i = 0; i < p_len; i++) {
         if (fabsf(p_data[i]) < p_gateThresh) p_data[i] = 0.0f;
    }
}

// 배열 내부의 잘못된 NaN/INF 값을 무효화하고 단일 패스로 평균 오프셋 DC 성분을 소거 (p_data: 대상 데이터 버퍼, p_len: 크기)
void CL_T2_DspEngine::_removeDC(float* p_data, uint32_t p_len) {
    float v_sum = 0.0f;
    for (uint32_t i = 0; i < p_len; i++) {
        p_data[i] = G_T2_10_Def_FPU_SAN_FLOAT(p_data[i]);
        v_sum += p_data[i];
    }
    float v_mean = v_sum / (float)p_len;
    for (uint32_t i = 0; i < p_len; i++) p_data[i] -= v_mean;
}
