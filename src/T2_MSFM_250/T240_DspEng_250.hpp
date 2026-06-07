/* ============================================================================
 * File: T240_DspEng_250.hpp
 * Summary: 멀티모달(진동/소음) SIMD 고속 신호 처리 엔진 헤더
 * ============================================================================ */

#pragma once

#include "T210_Def_250.hpp"
#include "T215_Type_250.hpp"
#include "esp_dsp.h"
#include <cstdint>

class CL_T2_DspEngine {
public:
    struct ST_AudioDspRuntime {
        alignas(16) float window[T2_Def::Audio::Sensor::FFT_SIZE_MAX];
        float             prev_pre_emp[2];                                                      // L/R 개별 프리엠파시스 샘플
        alignas(16) float median_hist[2][T2_Def::Global::Dsp::MEDIAN_WINDOW_MAX];               // L/R 개별 메디안 필터 히스토리
        alignas(16) float notch_coeffs[5];
        alignas(16) float notch_state[2][2];                                                   // L/R 개별 Notch 상태
        alignas(16) float notch2_coeffs[5];
        alignas(16) float notch2_state[2][2];                                                  // L/R 개별 Notch2 상태
        alignas(16) float iir_hpf_coeffs[5];
        alignas(16) float iir_hpf_state[2][2];                                                 // L/R 개별 IIR HPF 상태
        alignas(16) float iir_lpf_coeffs[5];
        alignas(16) float iir_lpf_state[2][2];                                                 // L/R 개별 IIR LPF 상태
        fir_f32_t         fir_inst_hpf[2];                                                      // L/R 개별 FIR HPF 인스턴스
        fir_f32_t         fir_inst_lpf[2];                                                      // L/R 개별 FIR LPF 인스턴스
        alignas(16) float fir_hpf_coeffs[T2_Def::Audio::FeatureLimit::FIR_TAPS_MAX];
        alignas(16) float fir_lpf_coeffs[T2_Def::Audio::FeatureLimit::FIR_TAPS_MAX];
        alignas(16) float fir_state_hpf[2][T2_Def::Audio::FeatureLimit::FIR_TAPS_MAX];         // L/R 개별 FIR HPF 상태 버퍼
        alignas(16) float fir_state_lpf[2][T2_Def::Audio::FeatureLimit::FIR_TAPS_MAX];         // L/R 개별 FIR LPF 상태 버퍼
    };

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
        alignas(16) float fir_hpf_coeffs[T2_Def::Accel::FeatureLimit::FIR_TAPS_MAX];
        alignas(16) float fir_lpf_coeffs[T2_Def::Accel::FeatureLimit::FIR_TAPS_MAX];
        alignas(16) float fir_state_hpf[T2_Def::Accel::Sensor::AXIS_MAX][T2_Def::Accel::FeatureLimit::FIR_TAPS_MAX];
        alignas(16) float fir_state_lpf[T2_Def::Accel::Sensor::AXIS_MAX][T2_Def::Accel::FeatureLimit::FIR_TAPS_MAX];
        
        // --- [신규] 가속도 힐버트 변환용 FIR 및 상태 버퍼 ---
        fir_f32_t         fir_inst_hilbert[T2_Def::Accel::Sensor::AXIS_MAX];
        alignas(16) float fir_state_hilbert[T2_Def::Accel::Sensor::AXIS_MAX][T2_Def::Accel::FeatureLimit::HILBERT_FIR_TAPS + 4];
    };

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
        alignas(16) float fir_hpf_coeffs[T2_Def::Gyro::FeatureLimit::FIR_TAPS_MAX];
        alignas(16) float fir_lpf_coeffs[T2_Def::Gyro::FeatureLimit::FIR_TAPS_MAX];
        alignas(16) float fir_state_hpf[T2_Def::Gyro::Sensor::AXIS_MAX][T2_Def::Gyro::FeatureLimit::FIR_TAPS_MAX];
        alignas(16) float fir_state_lpf[T2_Def::Gyro::Sensor::AXIS_MAX][T2_Def::Gyro::FeatureLimit::FIR_TAPS_MAX];
    };

private:
    bool _isInitialized = false;
    float _tempC = 23.0f; // 기본 상온 섭씨 23도

    // 동적 할당 포인터들 (각 모듈 사용 시점에만 할당하여 힙 보존)
    ST_AudioDspRuntime* _audDsp = nullptr;
    ST_AccelDspRuntime* _accDsp = nullptr;
    ST_GyroDspRuntime*  _gyrDsp = nullptr;

    // 연산 핫패스 및 수집 데이터 임시 가공/보관 버퍼들 (DspEngine 내부로 캡슐화 이관)
    float* _capBufL = nullptr;
    float* _capBufR = nullptr;
    float* _prcBufL = nullptr;
    float* _prcBufR = nullptr;

    float* _accBufX = nullptr;
    float* _accBufY = nullptr;
    float* _accBufZ = nullptr;
    float* _gyrBufX = nullptr;
    float* _gyrBufY = nullptr;
    float* _gyrBufZ = nullptr;

    // --- [신규] 힐버트 변환 진폭 포락선 임시 버퍼 ---
    float* _accHilbertEnvX = nullptr;
    float* _accHilbertEnvY = nullptr;
    float* _accHilbertEnvZ = nullptr;

    // 필터 계수 산출용 내부 헬퍼
    void _generateFirLpf(float* p_coeffs, uint16_t p_taps, float p_cutoffHz, float p_sampleRate);
    void _generateFirHpf(float* p_coeffs, uint16_t p_taps, float p_cutoffHz, float p_sampleRate);
    void _calcIirCoeffs(float p_freq, float p_q, float* p_coeffs, uint32_t p_sampleRate, bool p_isHpf);
    void _calcNotchCoeffs(float p_freq, float p_q, float* p_coeffs, uint32_t p_sampleRate);

    // 공통 가공 헬퍼
    void _applyMedianFilter(float* p_data, float* p_hist, uint8_t p_windowSize, uint32_t p_len);
    void _applyPreEmphasis(float* p_data, float& p_prevSample, uint32_t p_len, float p_alpha);
    void _applyNoiseGate(float* p_data, uint32_t p_len, float p_gateThresh);
    void _removeDC(float* p_data, uint32_t p_len);

public:
    CL_T2_DspEngine();
    ~CL_T2_DspEngine();

    // 함수설명: 설정 스냅샷을 받아 동적으로 DSP 처리용 런타임 메모리(SRAM) 구조체를 활성화 배치합니다. (p_cfg: 전체 설정 구조체, 반환값: 성공 여부)
    bool init(const T2_Type::ST_DynamicConfig_t& p_cfg);

    // 함수설명: 런타임 상태에서 갱신된 컷오프 주파수 등을 기반으로 각 필터 계수를 즉시 재계산 반영합니다. (p_cfg: 갱신된 설정 구조체)
    void reloadFilters(const T2_Type::ST_DynamicConfig_t& p_cfg);

    // 함수설명: 각 IIR 및 FIR 필터 인스턴스의 내부 이력 버퍼(지연 스테이지)들을 전부 0으로 재초기화합니다.
    void resetStates();

    // 함수설명: 수신된 스테레오 오디오 원시 파형 데이터에 대해 노이즈게이트, 프리엠파시스, 주파수 대역 필터를 적용합니다. (p_audL/R: 입력 오디오 데이터군, p_outL/R: 출력 대상 버퍼군, p_len: 크기, p_audCfg: 마이크 센서 설정)
    void processAudio(const float* p_audL, const float* p_audR, float* p_outL, float* p_outR, uint32_t p_len,
                      const T2_Type::ST_Audio_Config_t& p_audCfg);

    // 함수설명: 가속도 센서의 3축 원시 데이터에 대해 메디안, DC 제거 및 IIR/FIR 대역 필터를 연쇄 적용합니다. (p_inX/Y/Z: 가속도 입력 버퍼군, p_outX/Y/Z: 출력 대상 버퍼군, p_len: 크기, p_accCfg: 가속도 센서 설정)
    void processAccel(const float* p_inX, const float* p_inY, const float* p_inZ,
                      float* p_outX, float* p_outY, float* p_outZ, uint32_t p_len,
                      const T2_Type::ST_Accel_Config_t& p_accCfg);

    // 함수설명: 자이로 센서의 3축 원시 데이터에 대해 메디안, DC 제거 및 IIR/FIR 대역 필터를 연쇄 적용합니다. (p_inX/Y/Z: 자이로 입력 버퍼군, p_outX/Y/Z: 출력 대상 버퍼군, p_len: 크기, p_gyrCfg: 자이로 센서 설정)
    void processGyro(const float* p_inX, const float* p_inY, const float* p_inZ,
                     float* p_outX, float* p_outY, float* p_outZ, uint32_t p_len,
                     const T2_Type::ST_Gyro_Config_t& p_gyrCfg);

    // 함수설명: 현재 활성화되어 있는 마이크 신호용 창(Windowing) 함수 배열의 주소 포인터를 반환합니다. (반환값: 창 함수 데이터 포인터)
    const float* getAudioWindow() const { return _audDsp ? _audDsp->window : nullptr; }

    // 함수설명: 현재 활성화되어 있는 가속도 신호용 창(Windowing) 함수 배열의 주소 포인터를 반환합니다. (반환값: 창 함수 데이터 포인터)
    const float* getVibWindow() const { return _accDsp ? _accDsp->window : nullptr; }

    // [신규] 온도 데이터를 갱신하여 빔포밍 공간 보정에 반영되도록 합니다.
    void updateTemperature(float p_temp) { _tempC = p_temp; }

    // [신규] DspEngine 내부 버퍼 접근용 Getters
    float* getCapBufL() { return _capBufL; }
    float* getCapBufR() { return _capBufR; }
    float* getPrcBufL() { return _prcBufL; }
    float* getPrcBufR() { return _prcBufR; }
    float* getAccBufX() { return _accBufX; }
    float* getAccBufY() { return _accBufY; }
    float* getAccBufZ() { return _accBufZ; }
    float* getGyrBufX() { return _gyrBufX; }
    float* getGyrBufY() { return _gyrBufY; }
    float* getGyrBufZ() { return _gyrBufZ; }
    float* getAccHilbertEnvX() { return _accHilbertEnvX; }
    float* getAccHilbertEnvY() { return _accHilbertEnvY; }
    float* getAccHilbertEnvZ() { return _accHilbertEnvZ; }
};
