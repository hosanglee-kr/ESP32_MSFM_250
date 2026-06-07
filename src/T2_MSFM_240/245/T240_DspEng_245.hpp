/* ============================================================================
 * File: T240_DspEng_245.hpp
 * Summary: 멀티모달(진동/소음) SIMD 고속 신호 처리 엔진
 * ============================================================================ */

#pragma once

#include "T210_Def_245.hpp"
#include "T215_Type_245.hpp"
#include "esp_dsp.h"
#include <cstdint>

/**
 * @class CL_T2_DspEngine
 * @brief 오디오(마이크), 가속도, 자이로 신호에 대해 Biquad IIR, FIR, Median, DC 차감 필터링을 수행하는 엔진
 */
class CL_T2_DspEngine {
public:
    /**
     * @struct ST_AudioDspRuntime
     * @brief 오디오 필터 동작에 필요한 상태 버퍼 구조체
     */
    struct ST_AudioDspRuntime {
        alignas(16) float window[T2_Def::Audio::Sensor::FFT_SIZE_MAX];                          ///< FFT 윈도우 버퍼
        float             prev_pre_emp[2];                                                      ///< L/R 개별 프리엠파시스 직전 샘플 저장소
        alignas(16) float median_hist[2][T2_Def::Global::Dsp::MEDIAN_WINDOW_MAX];               ///< L/R 개별 메디안 필터 히스토리 버퍼
        alignas(16) float notch_coeffs[5];                                                      ///< IIR Notch 필터 계수
        alignas(16) float notch_state[2][2];                                                   ///< L/R 개별 Notch 지연 상태
        alignas(16) float notch2_coeffs[5];                                                     ///< IIR Notch2 필터 계수
        alignas(16) float notch2_state[2][2];                                                  ///< L/R 개별 Notch2 지연 상태
        alignas(16) float iir_hpf_coeffs[5];                                                    ///< IIR 고역필터 계수
        alignas(16) float iir_hpf_state[2][2];                                                 ///< L/R 개별 IIR HPF 지연 상태
        alignas(16) float iir_lpf_coeffs[5];                                                    ///< IIR 저역필터 계수
        alignas(16) float iir_lpf_state[2][2];                                                 ///< L/R 개별 IIR LPF 지연 상태
        fir_f32_t         fir_inst_hpf[2];                                                      ///< L/R 개별 FIR HPF 구조체 인스턴스
        fir_f32_t         fir_inst_lpf[2];                                                      ///< L/R 개별 FIR LPF 구조체 인스턴스
        alignas(16) float fir_hpf_coeffs[T2_Def::Audio::FeatureLimit::FIR_TAPS_MAX];            ///< FIR HPF 필터 계수 버퍼
        alignas(16) float fir_lpf_coeffs[T2_Def::Audio::FeatureLimit::FIR_TAPS_MAX];            ///< FIR LPF 필터 계수 버퍼
        alignas(16) float fir_state_hpf[2][T2_Def::Audio::FeatureLimit::FIR_TAPS_MAX];         ///< L/R 개별 FIR HPF 필터 지연 상태 버퍼
        alignas(16) float fir_state_lpf[2][T2_Def::Audio::FeatureLimit::FIR_TAPS_MAX];         ///< L/R 개별 FIR LPF 필터 지연 상태 버퍼
    };

    /**
     * @struct ST_AccelDspRuntime
     * @brief 가속도 3축 필터 동작에 필요한 상태 버퍼 구조체
     */
    struct ST_AccelDspRuntime {
        alignas(16) float window[T2_Def::Accel::Sensor::FFT_SIZE_MAX];
        alignas(16) float median_hist[T2_Def::Accel::Sensor::AXIS_MAX][T2_Def::Global::Dsp::MEDIAN_WINDOW_MAX];
        alignas(16) float notch_coeffs[5];
        alignas(16) float notch_state[T2_Def::Accel::Sensor::AXIS_MAX][2];
        alignas(16) float notch2_coeffs[5];
        alignas(16) float notch2_state[T2_Def::Accel::Sensor::AXIS_MAX][2];
        alignas(16) float iir_hpf_coeffs[5];
        alignas(16) float iir_hpf_state[T2_Def::Accel::Sensor::AXIS_MAX][2];
        alignas(16) float iir_lpf_coeffs[5];
        alignas(16) float iir_lpf_state[T2_Def::Accel::Sensor::AXIS_MAX][2];
        fir_f32_t         fir_inst_hpf[T2_Def::Accel::Sensor::AXIS_MAX];
        fir_f32_t         fir_inst_lpf[T2_Def::Accel::Sensor::AXIS_MAX];
        alignas(16) float fir_hpf_coeffs[T2_Def::Accel::Sensor::AXIS_MAX][T2_Def::Accel::FeatureLimit::FIR_TAPS_MAX];
        alignas(16) float fir_lpf_coeffs[T2_Def::Accel::Sensor::AXIS_MAX][T2_Def::Accel::FeatureLimit::FIR_TAPS_MAX];
        alignas(16) float fir_state_hpf[T2_Def::Accel::Sensor::AXIS_MAX][T2_Def::Accel::FeatureLimit::FIR_TAPS_MAX];
        alignas(16) float fir_state_lpf[T2_Def::Accel::Sensor::AXIS_MAX][T2_Def::Accel::FeatureLimit::FIR_TAPS_MAX];
    };

    /**
     * @struct ST_GyroDspRuntime
     * @brief 자이로 3축 필터 동작에 필요한 상태 버퍼 구조체
     */
    struct ST_GyroDspRuntime {
        alignas(16) float window[T2_Def::Gyro::Sensor::FFT_SIZE_MAX];
        alignas(16) float median_hist[T2_Def::Gyro::Sensor::AXIS_MAX][T2_Def::Global::Dsp::MEDIAN_WINDOW_MAX];
        alignas(16) float notch_coeffs[5];
        alignas(16) float notch_state[T2_Def::Gyro::Sensor::AXIS_MAX][2];
        alignas(16) float notch2_coeffs[5];
        alignas(16) float notch2_state[T2_Def::Gyro::Sensor::AXIS_MAX][2];
        alignas(16) float iir_hpf_coeffs[5];
        alignas(16) float iir_hpf_state[T2_Def::Gyro::Sensor::AXIS_MAX][2];
        alignas(16) float iir_lpf_coeffs[5];
        alignas(16) float iir_lpf_state[T2_Def::Gyro::Sensor::AXIS_MAX][2];
        fir_f32_t         fir_inst_hpf[T2_Def::Gyro::Sensor::AXIS_MAX];
        fir_f32_t         fir_inst_lpf[T2_Def::Gyro::Sensor::AXIS_MAX];
        alignas(16) float fir_hpf_coeffs[T2_Def::Gyro::Sensor::AXIS_MAX][T2_Def::Gyro::FeatureLimit::FIR_TAPS_MAX];
        alignas(16) float fir_lpf_coeffs[T2_Def::Gyro::Sensor::AXIS_MAX][T2_Def::Gyro::FeatureLimit::FIR_TAPS_MAX];
        alignas(16) float fir_state_hpf[T2_Def::Gyro::Sensor::AXIS_MAX][T2_Def::Gyro::FeatureLimit::FIR_TAPS_MAX];
        alignas(16) float fir_state_lpf[T2_Def::Gyro::Sensor::AXIS_MAX][T2_Def::Gyro::FeatureLimit::FIR_TAPS_MAX];
    };

private:
    bool _isInitialized = false;

    // 동적 할당 포인터들 (각 모듈 사용 시점에만 할당하여 힙 보존)
    ST_AudioDspRuntime* _audDsp = nullptr;
    ST_AccelDspRuntime* _accDsp = nullptr;
    ST_GyroDspRuntime*  _gyrDsp = nullptr;

    // 필터 계수 산출용 내부 헬퍼 함수
    void _generateFirLpf(float* p_coeffs, uint16_t p_taps, float p_cutoffHz, float p_sampleRate);
    void _generateFirHpf(float* p_coeffs, uint16_t p_taps, float p_cutoffHz, float p_sampleRate);
    void _calcIirCoeffs(float p_freq, float p_q, float* p_coeffs, uint32_t p_sampleRate, bool p_isHpf);
    void _calcNotchCoeffs(float p_freq, float p_q, float* p_coeffs, uint32_t p_sampleRate);

    // 공통 가공 헬퍼 함수
    void _applyMedianFilter(float* p_data, float* p_hist, uint8_t p_windowSize, uint32_t p_len);
    void _applyPreEmphasis(float* p_data, float& p_prevSample, uint32_t p_len, float p_alpha);
    void _applyNoiseGate(float* p_data, uint32_t p_len, float p_gateThresh);
    void _removeDC(float* p_data, uint32_t p_len);

public:
    CL_T2_DspEngine();
    ~CL_T2_DspEngine();

    /**
     * @brief 동적 설정을 적용하여 런타임 메모리 할당 및 초기화
     * @param p_cfg 적용할 전체 동적 설정 구조체
     * @return 초기화 성공 여부
     */
    bool init(const T2_Type::ST_DynamicConfig_t& p_cfg);

    /**
     * @brief 런타임 필터 차수, 컷오프 변경 등에 의한 필터 계수 리로드
     * @param p_cfg 최신 설정 정보 구조체
     */
    void reloadFilters(const T2_Type::ST_DynamicConfig_t& p_cfg);

    /**
     * @brief IIR/FIR 필터 지연 버퍼 및 이력 변수 초기 소거
     */
    void resetStates();

    /**
     * @brief 2채널 마이크 오디오 신호 필터링 연산 및 합성 실행
     * @param p_audL, p_audR 입력 신호 버퍼
     * @param p_outL, p_outR 결과 출력 버퍼
     * @param p_len 샘플 길이
     * @param p_audCfg 마이크 설정 파라미터 정보
     */
    void processAudio(const float* p_audL, const float* p_audR, float* p_outL, float* p_outR, uint32_t p_len,
                      const T2_Type::ST_Audio_Config_t& p_audCfg);

    /**
     * @brief 3축 가속도 신호 개별 축 필터링 연산 실행
     * @param p_inX, p_inY, p_inZ 입력 신호 버퍼
     * @param p_outX, p_outY, p_outZ 결과 출력 버퍼
     * @param p_len 샘플 길이
     * @param p_accCfg 가속도계 설정 파라미터 정보
     */
    void processAccel(const float* p_inX, const float* p_inY, const float* p_inZ,
                      float* p_outX, float* p_outY, float* p_outZ, uint32_t p_len,
                      const T2_Type::ST_Accel_Config_t& p_accCfg);

    /**
     * @brief 3축 자이로 신호 개별 축 필터링 연산 실행
     * @param p_inX, p_inY, p_inZ 입력 신호 버퍼
     * @param p_outX, p_outY, p_outZ 결과 출력 버퍼
     * @param p_len 샘플 길이
     * @param p_gyrCfg 자이로계 설정 파라미터 정보
     */
    void processGyro(const float* p_inX, const float* p_inY, const float* p_inZ,
                     float* p_outX, float* p_outY, float* p_outZ, uint32_t p_len,
                     const T2_Type::ST_Gyro_Config_t& p_gyrCfg);

    /**
     * @brief 진동 처리용 하위 호환 래퍼 (내부적으로 processAccel 호출)
     */
    void processVibration(const float* p_vibX, const float* p_vibY, const float* p_vibZ,
                          float* p_outX, float* p_outY, float* p_outZ, uint32_t p_len,
                          const T2_Type::ST_Accel_Config_t& p_accCfg) {
        processAccel(p_vibX, p_vibY, p_vibZ, p_outX, p_outY, p_outZ, p_len, p_accCfg);
    }

    /**
     * @brief 연산용 오디오 Hann 윈도우 스냅샷 포인터 획득
     */
    const float* getAudioWindow() const { return _audDsp ? _audDsp->window : nullptr; }

    /**
     * @brief 연산용 진동 Hann 윈도우 스냅샷 포인터 획득
     */
    const float* getVibWindow() const { return _accDsp ? _accDsp->window : nullptr; }
};
