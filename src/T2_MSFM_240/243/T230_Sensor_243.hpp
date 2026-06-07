/* ============================================================================
 * File: T230_Sensor_243.hpp
 * Summary: BMI270 (SPI) & ICS43434 (I2S) 멀티모달 센서 융합 수집 엔진 (v243 고도화)
 * ============================================================================
 * [로직 상태 추적 (Logic State Tracker)]
 * - 유지: T2의 BMI270 1.6kHz FIFO 배치 수집 및 딥슬립 Any-Motion 인터럽트.
 * - 갱신: T4의 ICS43434 I2S DMA 32bit 수집 엔진 병합 (ESP_INTR_FLAG_LEVEL3 격상).
 * - 신규: [SIMD/Calibration] 가중치(Gain) 보정 추가 및 v243 4-Tier 설정 구조체 연동.
 *
 * [축소/누락 방어 체크리스트 (Omission Defense)]
 * 1. [RTOS 방어]: delay() 절대 사용 금지. 복귀 대기 시 vTaskDelay 적용.
 * 2. [기능 축소 방어]: runCalibration, applyStoredCalibration 메서드 완벽 유지 및 Gain 대응.
 * 3. [메모리 오염 방어]: I2S 읽기 요청 길이 클램핑 (FFT_SIZE_MAX 상수 기반).
 * 4. [SIMD 정렬]: DMA 임시 버퍼(_dmaBuffer) alignas(16) 강제 정렬 유지.
 * ========================================================================== */
#pragma once

#include "T210_Def_243_9.hpp"
#include "T215_Type_243_8.hpp"
#include <SPI.h>
#include <driver/i2s.h>
#include "SparkFun_BMI270_Arduino_Library.h"

class CL_T2_SensorEngine {
private:
    SPIClass& _spi;
    BMI270    _bmi;

    bool      _isI2sInit;
    bool      _isBmiInit;
    bool      _isPaused;

    // 소프트웨어 캘리브레이션 (v243: Offset + Gain)
    float     _offsetX, _offsetY, _offsetZ;
    float     _gainX, _gainY, _gainZ;
    float     _lsbToG; // 가속도 센서 Range에 따른 LSB 변환 계수
    uint8_t   _accelRange; // 현재 설정된 가속도 측정 범위 (G)
    uint8_t   _accelAxisCount; // 가속도 분석 축 개수 (1축 vs 3축)
    uint8_t   _accelAxis;      // 가속도 분석 타겟 축 (0:X, 1:Y, 2:Z)
    uint8_t   _gyroAxisCount;  // 자이로 분석 축 개수 (1축 vs 3축)
    uint8_t   _gyroAxis;       // 자이로 분석 타겟 축 (0:X, 1:Y, 2:Z)
    bool      _gyroEnabled; // Gyro 활성화 여부 (FIFO 파싱 분기용)
    T2_Type::AudioChannelMode _audioChannelMode; // 오디오 채널 모드 (Mono L/R, Stereo)

    // [SIMD 정렬] 32bit 오디오(L/R 교차) 수신용 DMA 스크래치 버퍼
    // T2_Def::Audio::Sensor::FFT_SIZE_MAX 기반으로 최대 크기 할당
    alignas(16) int32_t _dmaBuffer[T2_Def::Audio::Sensor::FFT_SIZE_MAX * 2];

    char _statusText[32];

    // BMI270 고속 SPI 접근용 내부 헬퍼
    bool _readRegs(uint8_t p_reg, uint8_t* p_data, uint16_t p_len);
    bool _writeRegs(uint8_t p_reg, const uint8_t* p_data, uint16_t p_len);
    uint8_t _mapAccelRange(uint8_t p_rangeG);
    uint8_t _mapGyroRange(uint8_t p_rangeDps);

public:
    CL_T2_SensorEngine(SPIClass& p_spiBus);
    ~CL_T2_SensorEngine() = default;

    // v243 4-Tier 설정 구조체로 초기화 (Global 스위치 및 모듈별 Enable 반영)
    bool init(const T2_Type::ST_Global_System& p_sysCfg, const T2_Type::ST_Vib_Sensor& p_vibCfg, const T2_Type::ST_Audio_Sensor& p_audCfg);

    void pause();
    void resume();
    void clearAudioBuffer();
    void resetHardware();

    // [오디오 수집 - Core 0 Blocking]
    uint32_t readAudioChunk(float* p_outL, float* p_outR, uint32_t p_reqSamples);

    // [진동 수집 - Core 0 Polling] (캘리브레이션 오프셋 및 게인 자동 적용)
    uint16_t readVibFifoBatch(float* p_outX, float* p_outY, float* p_outZ, uint16_t p_maxFrames);

    // [복원된 핵심 기능] 캘리브레이션 (영점 조절 및 전용 JSON 저장)
    bool runCalibration();
    bool applyStoredCalibration();

    // 하드웨어 인터럽트 기반 Any-Motion 활성화
    bool enableWakeOnMotion(float p_threshG, uint16_t p_duration);

    const char* getStatusText() const { return _statusText; }
};
