/* ============================================================================
 * File: T231_Dsp_234.h
 * Summary: Advanced MFCC & DSP Pipeline Engine (v230.011 / SIMD + Matrix Optimized)
 * * [AI 메모: 제공 기능 요약]
 * 1. ESP32-S3 SIMD 및 Matrix API(dspm)를 활용한 초고속 FFT 및 MFCC 행렬 연산 가속.
 * 2. 다중 필터(FIR, IIR, Adaptive Notch) 및 가변 Median/Window 함수 완벽 지원.
 * 3. 16바이트 정렬된 Internal SRAM 플랫 버퍼를 사용하여 연산 지연 및 메모리 병목 차단.
 * * [AI 메모: 구현 및 유지보수 주의사항]
 * 1. FSM 상태가 READY -> MONITORING으로 전환될 때 반드시 resetFilterStates()를 
 * 호출하여 FIR/IIR 필터의 딜레이 라인(Delay Line) 찌꺼기로 인한 오탐지를 방지해야 합니다.
 * 2. 필터는 위상 왜곡을 막기 위해 반드시 [Median -> DC제거 -> FIR -> IIR -> Notch] 
 * 순서의 파이프라인을 엄격히 준수합니다.
 * ==========================================================================
 * 1. ESP-DSP의 Blackman Window 가속 함수를 활용한 Windowed-Sinc FIR 계수 자체 생성.
 * 2. 스펙트럼 반전(Spectral Inversion)을 통한 초정밀 Linear Phase HPF 생성.
 * 3. 16바이트 정렬된 Internal SRAM 플랫 버퍼를 사용하여 연산 지연 원천 차단.
 * 4. https://docs.espressif.com/projects/esp-dsp/en/latest/esp32/esp-dsp-apis.html#support 
 * ========================================================================== */



#pragma once

#include "T210_Def_234.h"  // ST_T20_Config_t 정의 포함
#include "esp_dsp.h"


class CL_T20_DspPipeline {
   public:
	CL_T20_DspPipeline();
	~CL_T20_DspPipeline(); // 소멸자 추가 (freeBuffers 호출)


	// 엔진 초기화 (프리컴퓨팅 및 필터 생성)
	bool begin(const ST_T20_Config_t& cfg);

	// 단일 프레임 처리 (Time Domain -> 39D MFCC Vector)
    bool processFrame(const float* p_time_in, ST_T20_FeatureVector_t* p_vec_out, uint8_t axis_idx = 0);


	// 노이즈 학습 제어 (외부 버튼이나 API에서 호출)
	void setNoiseLearning(bool active) {
		_noise_learning_active = active;
	}

	void resetNoiseStats() {
	    memset(_noise_learned_frames, 0, sizeof(_noise_learned_frames));
	}

	bool isNoiseLearning() const {
		return _noise_learning_active;
	}
	
	// FSM 상태 전환 시 과거 찌꺼기로 인한 글리치(Glitch) 트리거 오탐지 방지
    void resetFilterStates();

    const float* getPowerSpectrum() const { return _power; }

	float getBandEnergy(float start_hz, float end_hz);
	float getLatestRMS(uint8_t axis_idx) const { return _current_rms[axis_idx]; }

   private:
    void _freeBuffers();

    // 전처리 파이프라인 (순서 분리)
    void _applyMedianFilter(float* p_data, int window_size);
    void _removeDC(float* p_data);                                 // [분리됨]
    void _calcRMS(const float* p_data, uint8_t axis_idx);          // [분리됨]
    void _applyAdaptiveNotch(float* p_data, uint8_t axis_idx);
    void _applyPreEmphasis(float* p_data, uint8_t axis_idx);
    void _applyNoiseGate(float* p_data);
    void _generateWindow(float* p_out, int n, EM_T20_WindowType_t type);

    // 주파수 도메인 변환 및 특징 추출
    void _computePowerSpectrum(const float* p_time);
    void _learnNoiseSpectrum(uint8_t axis_idx);
    void _applySpectralSubtraction(uint8_t axis_idx);
    void _applyMelFilterbank(float* p_log_mel_out);
    void _computeDCT2(const float* p_in, float* p_out);
    
    // Windowed-Sinc 기반 FIR 필터 계수 자체 생성기
    void _generateFirLpfWindowedSinc(float* coeffs, uint16_t num_taps, float cutoff_hz);
    void _generateFirHpfWindowedSinc(float* coeffs, uint16_t num_taps, float cutoff_hz);

    // 39차원 벡터 조립
    void _pushHistory(const float* p_mfcc, uint8_t axis_idx);
    void _build39DVector(ST_T20_FeatureVector_t* p_vec_out, uint8_t axis_idx);

   private:
	ST_T20_Config_t _cfg;

    // 동적 할당 버퍼 (SIMD 정렬)
    float* _work_frame     = nullptr;
    float* _window         = nullptr;
    float* _power          = nullptr;
    float* _noise_spectrum = nullptr;
    float* _mel_bank_flat  = nullptr;
    float* _fft_io_buf     = nullptr;

    // 행렬 연산 가속을 위한 플랫 버퍼
    float* _dct_matrix_flat = nullptr;

    alignas(16) float _log_mel[T20::C10_DSP::MEL_FILTERS];
    alignas(16) float _mfcc_history[T20::C10_DSP::AXIS_COUNT_MAX][T20::C10_DSP::MFCC_HISTORY_LEN][T20::C10_DSP::MFCC_COEFFS_MAX];

    // [정합성 보완] SIMD 가동을 위해 FIR 상태 변수(Delay line)에도 16바이트 정렬 강제 적용
    alignas(16) float _fir_hpf_coeffs[128]; // 최대 128 Tap 지원
    alignas(16) float _fir_lpf_coeffs[128];
    alignas(16) float _fir_hpf_state[T20::C10_DSP::AXIS_COUNT_MAX][128];
    alignas(16) float _fir_lpf_state[T20::C10_DSP::AXIS_COUNT_MAX][128];
    
    fir_f32_t _fir_hpf_inst[T20::C10_DSP::AXIS_COUNT_MAX]; // FIR 필터 인스턴스
    fir_f32_t _fir_lpf_inst[T20::C10_DSP::AXIS_COUNT_MAX];

    // IIR 및 Notch
    alignas(16) float _hpf_coeffs[8];
    alignas(16) float _lpf_coeffs[8];
    alignas(16) float _notch_coeffs[8];

    float _hpf_state[T20::C10_DSP::AXIS_COUNT_MAX][2];
    float _lpf_state[T20::C10_DSP::AXIS_COUNT_MAX][2];
    float _notch_state[T20::C10_DSP::AXIS_COUNT_MAX][2];

    float _prev_sample[T20::C10_DSP::AXIS_COUNT_MAX];
    float _current_rms[T20::C10_DSP::AXIS_COUNT_MAX];

    uint16_t _history_count[T20::C10_DSP::AXIS_COUNT_MAX];
    uint16_t _noise_learned_frames[T20::C10_DSP::AXIS_COUNT_MAX];

    bool     _noise_learning_active = false;
    uint16_t _current_fft_size = 0;
};



