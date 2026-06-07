/* ============================================================================
 * File: T280_Sensor_241.hpp
 * Summary: BMI270 (SPI) & ICS43434 (I2S) 멀티모달 센서 융합 수집 엔진
 * ============================================================================
 * [로직 상태 추적 (Logic State Tracker)]
 * - 유지: T2의 BMI270 1.6kHz FIFO 배치 수집 및 딥슬립 Any-Motion 인터럽트.
 * - 갱신: T4의 ICS43434 I2S DMA 32bit 수집 엔진 병합 (ESP_INTR_FLAG_LEVEL3 격상).
 * - 신규: [자가 교정] 누락되었던 캘리브레이션(영점 보정) 로직 복원 및 소프트웨어 오프셋 적용.
 *
 * [축소/누락 방어 체크리스트 (Omission Defense)]
 * 1. [RTOS 방어]: delay() 절대 사용 금지. 복귀 대기 시 vTaskDelay 적용.
 * 2. [기능 축소 방어]: runCalibration, applyStoredCalibration 메서드 완벽 복원.
 * 3. [메모리 오염 방어]: I2S 읽기 요청 길이 클램핑 (FFT_SIZE 상수 기반).
 * 4. [SIMD 정렬]: DMA 임시 버퍼(_dmaBuffer) alignas(16) 강제 정렬 유지.
 * ========================================================================== */
#pragma once

#include "T210_Def_241.hpp"
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

    // 소프트웨어 캘리브레이션 오프셋
    float     _offsetX, _offsetY, _offsetZ;

    // [SIMD 정렬] 32bit 오디오(L/R 교차) 수신용 DMA 스크래치 버퍼
    alignas(16) int32_t _dmaBuffer[T2_Config::System::FFT_SIZE_AUDIO_CONST * 2];

    char _statusText[32];

    // BMI270 고속 SPI 접근용 내부 헬퍼
    bool _readRegs(uint8_t p_reg, uint8_t* p_data, uint16_t p_len);
    bool _writeRegs(uint8_t p_reg, const uint8_t* p_data, uint16_t p_len);
    uint8_t _mapAccelRange(uint8_t p_rangeG);

public:
    CL_T2_SensorEngine(SPIClass& p_spiBus);
    ~CL_T2_SensorEngine() = default;

    bool init(const T2_Type::ST_Config_Sensor& p_cfg);

    void pause();
    void resume();
    void clearAudioBuffer();

    // [오디오 수집 - Core 0 Blocking]
    uint32_t readAudioChunk(float* p_outL, float* p_outR, uint32_t p_reqSamples);

    // [진동 수집 - Core 0 Polling] (캘리브레이션 오프셋 자동 적용)
    uint16_t readVibFifoBatch(float* p_outX, float* p_outY, float* p_outZ, uint16_t p_maxFrames);

    // [복원된 핵심 기능] 캘리브레이션 (영점 조절 및 LittleFS 저장)
    bool runCalibration();
    bool applyStoredCalibration();

    bool enableWakeOnMotion(float p_threshG, uint16_t p_duration);

    const char* getStatusText() const { return _statusText; }
};
