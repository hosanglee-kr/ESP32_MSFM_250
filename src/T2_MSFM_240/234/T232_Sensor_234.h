/* ============================================================================
 * File: T232_Sensor_234.h
 * Summary: BMI270 Sensor Driver & Data Acquisition Engine (FSM Optimized)
 * * [AI 메모: 제공 기능 요약]
 * 1. SparkFun_BMI270_Arduino_Library 기반의 SPI 통신 엔진(10MHz).
 * 2. 1600Hz 고속 샘플링(ODR) 및 FIFO(Acc+Gyro 통합) 기반 데이터 배치 수집.
 * 3. 1축 타겟팅 및 3축 동시 분리(Interleaving -> Separation) 로직 지원.
 * 4. 절전 모드 진입 시 BMI270 자체 Any-Motion(충격/진동 감지) 하드웨어 인터럽트 설정.
 * 5. 공장 캘리브레이션 실행 및 오프셋 백업/복원(LittleFS).
 * * [AI 메모: 구현 및 유지보수 주의사항]
 * 1. FSM 상태가 READY(대기) 모드일 때 CPU가 허공에 데이터를 버리는 현상을 막기 위해 
 * pause() 메서드를 호출하여 센서 칩셋 자체를 저전력(Suspend) 상태로 전환합니다.
 * ========================================================================== */

/* 
============================================================================
 * File: T232_Sensor_234.h
 * Summary: BMI270 Sensor Driver & Data Acquisition Engine
 * * [AI 메모: 제공 기능 요약]
 * 1. SparkFun_BMI270_Arduino_Library 기반의 SPI 통신 엔진(10MHz).
 * 2. 1600Hz 고속 샘플링(ODR) 및 FIFO(Acc+Gyro 통합) 기반 데이터 배치 수집.
 * 3. 1축 타겟팅 및 3축 동시 분리(Interleaving -> Separation) 로직 지원.
 * 4. 절전 모드 진입 시 BMI270 자체 Any-Motion(충격/진동 감지) 하드웨어 인터럽트 설정.
 * 5. 공장 캘리브레이션 실행 및 오프셋 백업/복원(LittleFS).
 * * [AI 메모: 구현 및 유지보수 주의사항]
 * 1. FIFO 일괄 읽기 시 1프레임은 12바이트(Accel 6B + Gyro 6B)입니다. 최대 32프레임 
 * 배치 처리 시 메모리와 타이밍의 병목이 없도록 주의해야 합니다.
 * 2. Any-Motion 임계값(Threshold)은 11-bit 해상도(0~2047)로 0~1G를 표현하므로 
 * 계산 시 2047을 초과하지 않도록 반드시 클램핑(Clamping) 처리를 유지해야 합니다.
 * 3. 직접 레지스터를 읽는 _readRegs 로직에서 BMI270의 SPI 통신 규약에 따른 
 * Dummy Byte 전송(0x00)이 누락되지 않도록 유지합니다.
 * ========================================================================== 
 */
#pragma once

#include "SparkFun_BMI270_Arduino_Library.h"
#include <SPI.h>
#include "T210_Def_234.h"

class CL_T20_SensorEngine {
public:

    CL_T20_SensorEngine(SPIClass& spi_bus);
    ~CL_T20_SensorEngine() = default;

    // 센서 초기화 및 하드웨어 설정
    bool begin(const ST_T20_ConfigSensor_t& s_cfg);

    // FIFO 데이터 일괄 획득 (배치 처리용 - 1축 또는 3축 분리)
    uint16_t readFifoBatch(float* p_out_x, float* p_out_y, float* p_out_z,
                           uint16_t max_frames, EM_T20_AxisCount_t axis_count,
                           EM_T20_SensorAxis_t target_axis);

     // FSM 대기 상태 진입 시 호출: FIFO 정지 및 자이로 절전
    void pause();
    // FSM 가동 상태 진입 시 호출: 센서 정상 가동 복귀
    void resume();
 
    // 캘리브레이션 관리
    bool runCalibration();
    bool applyStoredCalibration();

    // 딥슬립 전 Any-Motion (충격 감지) 하드웨어 인터럽트 활성화
    bool enableWakeOnMotion(float threshold_g, uint16_t duration);

    // 상태 조회 및 초기화
    const char* getStatusText() const { return _status_text; }
    void resetHardware();

private:
    // Direct SPI Helpers (내부 캡슐화)
    bool _readRegs(uint8_t reg, uint8_t* data, uint16_t len);
    bool _writeRegs(uint8_t reg, const uint8_t* data, uint16_t len);

    uint8_t _mapAccelRange(EM_T20_AccelRange_t r);
    uint8_t _mapGyroRange(EM_T20_GyroRange_t r);

private:
    SPIClass& _spi;
    BMI270    _bmi;
    char      _status_text[32];
    bool      _initialized = false;
    bool      _is_paused = false; // 일시정지 상태 플래그
};
