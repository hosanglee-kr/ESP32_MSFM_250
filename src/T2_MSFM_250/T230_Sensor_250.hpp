/* ============================================================================
 * File: T230_Sensor_250.hpp
 * Summary: BMI270 (SPI) & ICS43434 (I2S) 멀티모달 센서 수집 엔진 헤더
 * ============================================================================ */

#pragma once

#include "T210_Def_250.hpp"
#include "T215_Type_250.hpp"
#include <SPI.h>
#include <driver/i2s.h>
#include "SparkFun_BMI270_Arduino_Library.h"

enum class EM_SpiPriority : uint8_t {
    IMU_FIFO_READ = 0,
    IMU_REG_WRITE,
    IMU_TEMP_READ
};

struct ST_SpiTransaction_t {
    EM_SpiPriority priority;
    uint8_t reg_addr;
    uint8_t* tx_data;
    uint8_t* rx_data;
    size_t length;
    SemaphoreHandle_t done_sem;
};


class CL_T2_SensorEngine {
private:
    SPIClass& _spi;
    BMI270    _bmi;

    SemaphoreHandle_t _spiLock; // [신규] 멀티코어 SPI 경합 방지용 뮤텍스


    bool      _isI2sInit;
    bool      _isBmiInit;
    bool      _isPaused;

    // 소프트웨어 캘리브레이션 오프셋 및 게인 변수
    float     _accOffsetX, 	_accOffsetY,	_accOffsetZ;
    float     _accGainX, 	_accGainY, 		_accGainZ;
    float     _gyrOffsetX, 	_gyrOffsetY, 	_gyrOffsetZ;
    float     _gyrGainX, 	_gyrGainY, 		_gyrGainZ;

    float     _lsbToG;     // 가속도 LSB 변환 계수
    float     _lsbToDps;   // 자이로 LSB 변환 계수

    // SPI MSB/LSB 뒤집힘 방어용 직전 유효값 보존 변수
    float     _lastAccX, _lastAccY, _lastAccZ;
    float     _lastGyrX, _lastGyrY, _lastGyrZ;

    uint8_t   _accelRange;
    uint8_t   _gyroRange;
    uint8_t   _accelAxisMask;
    uint8_t   _gyroAxisMask;
    bool      _gyroEnabled;
    T2_Type::EM_ChannelMask_t _audioChannelMask;

    // 가속도 FIFO 축별 누적 링버퍼
    float     _accumX[T2_Def::Accel::Sensor::FFT_SIZE_MAX];
    float     _accumY[T2_Def::Accel::Sensor::FFT_SIZE_MAX];
    float     _accumZ[T2_Def::Accel::Sensor::FFT_SIZE_MAX];
    uint16_t  _accumWriteIdx;
    uint16_t  _accumReadIdx;
    uint16_t  _accumCount;

    // 자이로 FIFO 축별 누적 링버퍼
    float     _accumGX[T2_Def::Gyro::Sensor::FFT_SIZE_MAX];
    float     _accumGY[T2_Def::Gyro::Sensor::FFT_SIZE_MAX];
    float     _accumGZ[T2_Def::Gyro::Sensor::FFT_SIZE_MAX];
    uint16_t  _accumGWriteIdx;
    uint16_t  _accumGReadIdx;
    uint16_t  _accumGCount;

    // 오디오 수신용 DMA 스크래치 버퍼
    alignas(16) int32_t _audioDmaBuffer[T2_Def::Audio::Sensor::FFT_SIZE_MAX * 2];

    char _statusText[32];

    // BMI270 고속 SPI 접근용 내부 헬퍼 함수
    bool _readRegs(uint8_t p_reg, uint8_t* p_data, uint16_t p_len);
    bool _writeRegs(uint8_t p_reg, const uint8_t* p_data, uint16_t p_len);
    uint8_t _mapAccelRange(uint8_t p_rangeG);
    uint8_t _mapGyroRange(uint16_t p_rangeDps);
    void _writeRegSingle(uint8_t p_reg, uint8_t p_val);

public:
    // 인터럽트 물리 분리에 따른 상태 확인용 직접 레지스터 읽기 헬퍼
    uint8_t _readRegSingle(uint8_t p_reg);

    CL_T2_SensorEngine(SPIClass& p_spiBus);
    ~CL_T2_SensorEngine() = default;

    // 센서 설정을 인자로 받아 가속도/자이로 및 마이크 수집 모듈들을 초기화합니다. (p_sysCfg: 글로벌 설정, p_accCfg: 가속도 설정, p_gyrCfg: 자이로 설정, p_audCfg: 마이크 설정, 반환값: 성공 여부)
    bool init(const T2_Type::ST_Global_System_t& p_sysCfg,
              const T2_Type::ST_Accel_Config_t& p_accCfg,
              const T2_Type::ST_Gyro_Config_t& p_gyrCfg,
              const T2_Type::ST_Audio_Config_t& p_audCfg);

    // 데이터 수집 및 DMA 전송 과정을 일시 정지시킵니다.
    void pause();

    // 일시 정지 상태에서 데이터 수집 및 DMA 전송 과정을 재개합니다.
    void resume();

    // 마이크용 DMA 버퍼 및 내부 I2S 대기 큐를 완전히 비워 초기화합니다.
    void clearAudioBuffer();

    // 센서 하드웨어를 재부팅하여 통신 상태를 리셋합니다.
    void resetHardware();

    // 마이크 센서로부터 16비트 오디오 샘플 청크를 가져와 채널 분리를 수행합니다. (p_outL: L채널 출력 버퍼, p_outR: R채널 출력 버퍼, p_reqSamples: 요구 샘플 개수, 반환값: 수신 완료된 샘플 개수)
    uint32_t readAudioChunk(float* p_outL, float* p_outR, uint32_t p_reqSamples);

    // BMI270 센서의 FIFO로부터 진동 원시 데이터를 다량으로 수집하여 가변 정밀도로 변환 적용합니다. (p_accX/Y/Z: 가속도 축별 출력 버퍼, p_gyrX/Y/Z: 자이로 축별 출력 버퍼, p_maxFrames: 최대 요청 프레임 수, 반환값: 변환 완료된 프레임 개수)
    uint16_t readVibFifoBatch(float* p_accX, float* p_accY, float* p_accZ,
                              float* p_gyrX, float* p_gyrY, float* p_gyrZ,
                              uint16_t p_maxFrames);

    // 센서 FIFO를 읽고 내부 축별 누적 링버퍼에 적재합니다. (반환값: 축적 완료된 진동 데이터 프레임 수)
    uint16_t accumulateFifo();

    // 누적된 가속도 데이터 링버퍼에서 지정 수량만큼의 샘플을 꺼내옵니다. (p_outX/Y/Z: 출력 버퍼, p_reqCount: 요청 개수, 반환값: 복사 완료된 개수)
    uint16_t getAccumulatedAccel(float* p_outX, float* p_outY, float* p_outZ, uint16_t p_reqCount);

    // 누적된 자이로 데이터 링버퍼에서 지정 수량만큼의 샘플을 꺼내옵니다. (p_outX/Y/Z: 출력 버퍼, p_reqCount: 요청 개수, 반환값: 복사 완료된 개수)
    uint16_t getAccumulatedGyro(float* p_outX, float* p_outY, float* p_outZ, uint16_t p_reqCount);

    // 누적 대기 상태인 가속도 데이터 프레임 개수를 반환합니다. (반환값: 누적 프레임 수)
    uint16_t getAccumulatedAccelCount() const { return _accumCount; }

    // 누적 대기 상태인 자이로 데이터 프레임 개수를 반환합니다. (반환값: 누적 프레임 수)
    uint16_t getAccumulatedGyroCount() const { return _accumGCount; }

    // BMI270 칩 내부 온도 센서에서 섭씨 온도를 읽어와 반환합니다. (반환값: 섭씨 온도값)
    float readTemperatureSensor();

    // 가속도 센서의 3축 소프트웨어 오프셋 캘리브레이션을 진행하고 결과를 파일로 저장합니다. (반환값: 성공 여부)
    bool runAccelCalibration();

    // 자이로 센서의 3축 소프트웨어 오프셋 캘리브레이션을 진행하고 결과를 파일로 저장합니다. (반환값: 성공 여부)
    bool runGyroCalibration();

    // 이전에 파일시스템에 저장되어 있던 가속도 캘리브레이션 설정값을 로드하여 적용합니다. (반환값: 복원 성공 여부)
    bool applyStoredAccelCalibration();

    // 이전에 파일시스템에 저장되어 있던 자이로 캘리브레이션 설정값을 로드하여 적용합니다. (반환값: 복원 성공 여부)
    bool applyStoredGyroCalibration();

    // 움직임 감지 시 WDT 해제 또는 부팅 트리거가 유발되도록 Any-Motion 인터럽트 설정을 활성화합니다. (p_threshG: 감지 가속도 임계치 G, p_duration: 최소 충족 틱 수, 반환값: 적용 성공 여부)
    bool enableWakeOnMotion(float p_threshG, uint16_t p_duration);

    // 현재 센서 수집 모듈들의 동작 상태 문자열을 반환합니다. (반환값: 상태 문자열 포인터)
    const char* getStatusText() const { return _statusText; }

    // 외부에서 SPI 뮤텍스를 참조할 수 있도록 게터 제공 (FsmMgr 제어용)
    SemaphoreHandle_t getSpiLock() const { return _spiLock; }

    // [신규] 하드웨어 FIFO 플러시
    void flushHardwareFifo();

    // [신규] 딥슬립 Wake-up 설정
    void prepareDeepSleepWakeup(float wake_g, uint16_t wake_dur);

    // [신규] 딥슬립 복귀 레지스터 초기화
    void restoreFromDeepSleepWakeup();

    // [신규] I2S DMA 제어
    void stopI2SDma();
    void startI2SDma();

    // [신규] 캘리브레이션 오프셋 동적 반영
    void updateCalibrationOffsets(const float* offsets);
};




