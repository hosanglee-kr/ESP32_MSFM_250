/* ============================================================================
 * File: T240_DspEng_242.hpp
 * Summary: 멀티모달(진동/소음) 하이브리드 SIMD 고속 신호 처리 파이프라인
 * ============================================================================
 * [로직 상태 추적 (Logic State Tracker)]
 * - 유지: T2의 진동 3축(X,Y,Z) 독립 필터링(DC 차단 및 스파이크 제거) 파이프라인.
 * - 갱신: T4(SMEA-100)의 Beamforming, Notch, Pre-emphasis 기반 오디오 파이프라인 병합.
 * - 신규: 이종 샘플링 레이트(1.6kHz vs 42kHz)를 독립적으로 지원하는 필터 계수 이중화.
 *
 * [축소/누락 방어 체크리스트 (Omission Defense)]
 * 1. [SIMD 정렬]: 모든 내부 상태 버퍼 및 FIR 계수에 alignas(16) 적용 완료.
 * 2. [LTI 병합]: 오디오 파이프라인은 Beamforming을 최상단에 배치하여 50% 연산 절감.
 * 3. [비선형 배치]: Noise Gate 등 파형 절단 로직을 항상 파이프라인의 맨 마지막에 배치.
 * 4. [포인터 방어]: 원본 데이터 보존을 위해 입력 매개변수에 const float* 엄격 적용.
 * ========================================================================== */
#pragma once

#include "T210_Def_242.hpp"
#include "esp_dsp.h"
#include <cstdint>

class CL_T2_DspEngine {
private:
    T2_Type::ST_Config_Dsp _dspCfg;

	bool _isInitialized = false;

    // 윈도우 함수 버퍼
    alignas(16) float _audWindow[T2_Def::System::FFT_SIZE_AUDIO_CONST];
    alignas(16) float _vibWindow[T2_Def::System::FFT_SIZE_VIB_CONST];

    // Audio DSP 상태 변수 (SIMD 정렬)
    alignas(16) float _notch60Coeffs[5];
    alignas(16) float _notch60State[2];
    alignas(16) float _notch120Coeffs[5];
    alignas(16) float _notch120State[2];
    alignas(16) float _firEqState[T2_Def::FeatureLimit::FIR_TAPS_CONST];


    // --- Audio 전용 상태 변수 (1채널 병합) ---
    float _prevPreEmpAudio = 0.0f;
    alignas(16) float _medianHistAudio[T2_Def::Dsp::MEDIAN_WINDOW_DEF];
    alignas(16) float _audioBufA[T2_Def::System::FFT_SIZE_AUDIO_CONST];
    alignas(16) float _audioBufB[T2_Def::System::FFT_SIZE_AUDIO_CONST];

    alignas(16) float _notchCoeffs[5];
    alignas(16) float _wNotch[2];

    fir_f32_t         _firInstHpfAudio;
    fir_f32_t         _firInstLpfAudio;
    alignas(16) float _firHpfAudioCoeffs[T2_Def::FeatureLimit::FIR_TAPS_CONST];
    alignas(16) float _firLpfAudioCoeffs[T2_Def::FeatureLimit::FIR_TAPS_CONST];
    alignas(16) float _firStateHpfAudio[T2_Def::FeatureLimit::FIR_TAPS_CONST];
    alignas(16) float _firStateLpfAudio[T2_Def::FeatureLimit::FIR_TAPS_CONST];

    // IIR Biquad 필터 상태 및 계수 버퍼 (SIMD 정렬)
    alignas(16) float _iirHpfCoeffs[5];
    alignas(16) float _iirHpfState[2];
    alignas(16) float _iirLpfCoeffs[5];
    alignas(16) float _iirLpfState[2];

    // --- Vibration 전용 상태 변수 (3축 독립) ---
    alignas(16) float _medianHistVib[T2_Def::System::VIB_AXIS_CONST][T2_Def::Dsp::MEDIAN_WINDOW_DEF];
    alignas(16) float _vibBufA[T2_Def::System::VIB_AXIS_CONST][T2_Def::System::FFT_SIZE_VIB_CONST];

    fir_f32_t         _firInstHpfVib[T2_Def::System::VIB_AXIS_CONST];
    fir_f32_t         _firInstLpfVib[T2_Def::System::VIB_AXIS_CONST];
    alignas(16) float _firHpfVibCoeffs[T2_Def::FeatureLimit::FIR_TAPS_CONST];
    alignas(16) float _firLpfVibCoeffs[T2_Def::FeatureLimit::FIR_TAPS_CONST];
    alignas(16) float _firStateHpfVib[T2_Def::System::VIB_AXIS_CONST][T2_Def::FeatureLimit::FIR_TAPS_CONST];
    alignas(16) float _firStateLpfVib[T2_Def::System::VIB_AXIS_CONST][T2_Def::FeatureLimit::FIR_TAPS_CONST];


    void _recalcDynamicFilters();
    void _generateFirLpfWindowedSinc(float* p_coeffs, uint16_t p_taps, float p_cutoffHz, float p_sampleRate);
    void _generateFirHpfWindowedSinc(float* p_coeffs, uint16_t p_taps, float p_cutoffHz, float p_sampleRate);

    // IIR 계수 산출 헬퍼
    void _calcIirHpCoeffs(float p_freq, float p_q, float* p_coeffs, uint32_t p_sampleRate);
    void _calcIirLpCoeffs(float p_freq, float p_q, float* p_coeffs, uint32_t p_sampleRate);

    // 내부 공통 유틸리티
    void _applyBeamforming(const float* p_L, const float* p_R, float* p_out, uint32_t p_len, float p_gain);
    void _applyMedianFilter(float* p_data, float* p_hist, uint8_t p_windowSize, uint32_t p_len);
    void _applyPreEmphasis(float* p_data, float& p_prevSample, uint32_t p_len, float p_alpha);
    void _applyNoiseGate(float* p_data, uint32_t p_len, float p_gateThresh);

	void _calcNotchCoeffs(float p_freq, float p_q, float* p_coeffs, uint32_t p_sampleRate);
public:
    CL_T2_DspEngine();
    ~CL_T2_DspEngine() = default;

    bool init();
    // void resetFilterStates();
    void reloadFilters();
	void resetStates();

    // 이종 센서 독립 처리 파이프라인

	void processAcoustic(const float* p_audL, const float* p_audR, float* p_outBeam);
	// void processAudio(const float* p_micL, const float* p_micR, float* p_out, uint32_t p_len);
    void processVibration(const float* p_vibX, const float* p_vibY, const float* p_vibZ, float* p_outX, float* p_outY, float* p_outZ, uint32_t p_len);
	//void processVibration(const float* p_vibX, const float* p_vibY, const float* p_vibZ, float* p_outX, float* p_outY, float* p_outZ, uint32_t p_len);
};


