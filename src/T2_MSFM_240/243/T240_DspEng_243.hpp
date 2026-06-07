/* ============================================================================
 * File: T240_DspEng_243.hpp
 * Summary: 멀티모달(진동/소음) 하이브리드 SIMD 고속 신호 처리 엔진 (v243 고도화)
 * ============================================================================
 * [로직 상태 추적 (Logic State Tracker)]
 * - 유지: T2의 진동 3축 독립 필터링 및 T4의 오디오 빔포밍+필터 통합 파이프라인.
 * - 갱신: v243 4-Tier 설정 구조체(ST_Shared_Dsp, ST_Audio_Dsp) 완벽 연동.
 * - 신규: [정적 할당] _MAX 상수를 이용한 메모리 선언 및 _DEF 기반 런타임 루프 제어.
 *
 * [축소/누락 방어 체크리스트 (Omission Defense)]
 * 1. [SIMD 정렬]: 모든 내부 상태 버퍼 및 FIR/IIR 계수에 alignas(16) 강제 적용.
 * 2. [리소스 제약]: FIR TAPS는 FIR_TAPS_MAX(127)로 제한하여 스택 오버플로우 방지.
 * 3. [LTI 병합]: 오디오는 Beamforming -> Filter 순서로 연산량 50% 절감 유지.
 * 4. [방어적 루프]: p_len 매개변수를 통해 가변 샘플 길이에 대응.
 * ========================================================================== */
#pragma once

#include "T210_Def_243_9.hpp"
#include "T215_Type_243_8.hpp"
#include "esp_dsp.h"
#include <cstdint>

class CL_T2_DspEngine {
public:
    // [사용여부 대응 동적 구조체 정의]
    struct ST_AudioDspRuntime {
        alignas(16) float window[T2_Def::Audio::Sensor::FFT_SIZE_MAX];
        float             prev_pre_emp;
        alignas(16) float median_hist[T2_Def::Shared::Dsp::MEDIAN_WINDOW_MAX];
        alignas(16) float notch_coeffs[5];
        alignas(16) float notch_state[2];
        alignas(16) float notch2_coeffs[5];
        alignas(16) float notch2_state[2];
        alignas(16) float iir_hpf_coeffs[5];
        alignas(16) float iir_hpf_state[2];
        alignas(16) float iir_lpf_coeffs[5];
        alignas(16) float iir_lpf_state[2];
        fir_f32_t         fir_inst_hpf;
        fir_f32_t         fir_inst_lpf;
        alignas(16) float fir_hpf_coeffs[T2_Def::Shared::FeatureLimit::FIR_TAPS_MAX];
        alignas(16) float fir_lpf_coeffs[T2_Def::Shared::FeatureLimit::FIR_TAPS_MAX];
        alignas(16) float fir_state_hpf[T2_Def::Shared::FeatureLimit::FIR_TAPS_MAX];
        alignas(16) float fir_state_lpf[T2_Def::Shared::FeatureLimit::FIR_TAPS_MAX];
    };

    struct ST_AccelDspRuntime {
        alignas(16) float window[T2_Def::Vib::Sensor::FFT_SIZE_MAX];
        alignas(16) float median_hist[T2_Def::Vib::Sensor::AXIS_MAX][T2_Def::Shared::Dsp::MEDIAN_WINDOW_MAX];
        fir_f32_t         fir_inst_hpf[T2_Def::Vib::Sensor::AXIS_MAX];
        fir_f32_t         fir_inst_lpf[T2_Def::Vib::Sensor::AXIS_MAX];
        alignas(16) float fir_hpf_coeffs[T2_Def::Shared::FeatureLimit::FIR_TAPS_MAX];
        alignas(16) float fir_lpf_coeffs[T2_Def::Shared::FeatureLimit::FIR_TAPS_MAX];
        alignas(16) float fir_state_hpf[T2_Def::Vib::Sensor::AXIS_MAX][T2_Def::Shared::FeatureLimit::FIR_TAPS_MAX];
        alignas(16) float fir_state_lpf[T2_Def::Vib::Sensor::AXIS_MAX][T2_Def::Shared::FeatureLimit::FIR_TAPS_MAX];
    };

    struct ST_GyroDspRuntime {
        alignas(16) float window[T2_Def::Vib::Sensor::FFT_SIZE_MAX];
        alignas(16) float median_hist[T2_Def::Vib::Sensor::AXIS_MAX][T2_Def::Shared::Dsp::MEDIAN_WINDOW_MAX];
        fir_f32_t         fir_inst_hpf[T2_Def::Vib::Sensor::AXIS_MAX];
        fir_f32_t         fir_inst_lpf[T2_Def::Vib::Sensor::AXIS_MAX];
        alignas(16) float fir_hpf_coeffs[T2_Def::Shared::FeatureLimit::FIR_TAPS_MAX];
        alignas(16) float fir_lpf_coeffs[T2_Def::Shared::FeatureLimit::FIR_TAPS_MAX];
        alignas(16) float fir_state_hpf[T2_Def::Vib::Sensor::AXIS_MAX][T2_Def::Shared::FeatureLimit::FIR_TAPS_MAX];
        alignas(16) float fir_state_lpf[T2_Def::Vib::Sensor::AXIS_MAX][T2_Def::Shared::FeatureLimit::FIR_TAPS_MAX];
    };

private:
    bool _isInitialized = false;

    // 동적 할당 포인터들
    ST_AudioDspRuntime* _audDsp = nullptr;
    ST_AccelDspRuntime* _accDsp = nullptr;
    ST_GyroDspRuntime*  _gyrDsp = nullptr;

    // 내부 계산 헬퍼
    void _generateFirLpf(float* p_coeffs, uint16_t p_taps, float p_cutoffHz, float p_sampleRate);
    void _generateFirHpf(float* p_coeffs, uint16_t p_taps, float p_cutoffHz, float p_sampleRate);
    void _calcIirCoeffs(float p_freq, float p_q, float* p_coeffs, uint32_t p_sampleRate, bool p_isHpf);
    void _calcNotchCoeffs(float p_freq, float p_q, float* p_coeffs, uint32_t p_sampleRate);

    // 공통 유연성(DSP) 헬퍼
    void _applyMedianFilter(float* p_data, float* p_hist, uint8_t p_windowSize, uint32_t p_len);
    void _applyPreEmphasis(float* p_data, float& p_prevSample, uint32_t p_len, float p_alpha);
    void _applyNoiseGate(float* p_data, uint32_t p_len, float p_gateThresh);
    void _removeDC(float* p_data, uint32_t p_len);

public:
    CL_T2_DspEngine();
    ~CL_T2_DspEngine();

    // v243 설정 구조체 기반 초기화 및 런타임 필터 갱신 (DynamicConfig SSOT 직접 연동)
    bool init(const T2_Type::DynamicConfig& p_cfg);

    void reloadFilters(const T2_Type::DynamicConfig& p_cfg);

    void resetStates();

    /**
     * @brief 소음 데이터 처리 파이프라인 (Beamforming -> Filter -> Gate)
     */
    void processAudio(const float* p_audL, const float* p_audR, float* p_out, uint32_t p_len,
                      const T2_Type::ST_Shared_Dsp& p_sharedCfg, const T2_Type::ST_Audio_Dsp& p_audCfg);

    /**
     * @brief 가속도(Acceleration) 데이터 처리 파이프라인
     */
    void processAccel(const float* p_inX, const float* p_inY, const float* p_inZ,
                      float* p_outX, float* p_outY, float* p_outZ, uint32_t p_len,
                      const T2_Type::ST_Shared_Dsp& p_sharedCfg);

    /**
     * @brief 자이로(Gyro) 데이터 처리 파이프라인
     */
    void processGyro(const float* p_inX, const float* p_inY, const float* p_inZ,
                     float* p_outX, float* p_outY, float* p_outZ, uint32_t p_len,
                     const T2_Type::ST_Shared_Dsp& p_sharedCfg);

    // 하위 호환성 유지용 processVibration 래퍼
    void processVibration(const float* p_vibX, const float* p_vibY, const float* p_vibZ,
                          float* p_outX, float* p_outY, float* p_outZ, uint32_t p_len,
                          const T2_Type::ST_Shared_Dsp& p_sharedCfg) {
        processAccel(p_vibX, p_vibY, p_vibZ, p_outX, p_outY, p_outZ, p_len, p_sharedCfg);
    }

    const float* getAudioWindow() const { return _audDsp ? _audDsp->window : nullptr; }
    const float* getVibWindow() const { return _accDsp ? _accDsp->window : nullptr; }
};
