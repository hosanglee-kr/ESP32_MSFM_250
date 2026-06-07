/* ============================================================================
 * File: T230_Sensor_245.hpp
 * Summary: BMI270 (SPI) & ICS43434 (I2S) 멀티모달 센서 수집 엔진
 * ============================================================================ */

#pragma once

#include "T210_Def_245.hpp"
#include "T215_Type_245.hpp"
#include <SPI.h>
#include <driver/i2s.h>
#include "SparkFun_BMI270_Arduino_Library.h"

/**
 * @class CL_T2_SensorEngine
 * @brief SPI 기반 IMU(BMI270) 가속도/자이로 센서 및 I2S 기반 마이크(ICS43434) 데이터를 넌블로킹/블로킹 방식으로 수집하는 센서 인터페이스 엔진
 */
class CL_T2_SensorEngine {
private:
    SPIClass& _spi;            ///< SPI 통신 버스 인스턴스 참조
    BMI270    _bmi;            ///< BMI270 라이브러리 드라이버 객체

    bool      _isI2sInit;      ///< I2S 오디오 장치 초기화 완료 플래그
    bool      _isBmiInit;      ///< BMI270 센서 초기화 완료 플래그
    bool      _isPaused;       ///< 데이터 수집 임시 일시 정지 상태 플래그

    // 소프트웨어 캘리브레이션 (가속도 / 자이로 개별 Offset 및 Gain)
    float     _accOffsetX, _accOffsetY, _accOffsetZ;
    float     _accGainX,   _accGainY,   _accGainZ;
    float     _gyrOffsetX, _gyrOffsetY, _gyrOffsetZ;
    float     _gyrGainX,   _gyrGainY,   _gyrGainZ;

    float     _lsbToG;         ///< 가속도 LSB 값을 G 단위 변환하는 계수
    float     _lsbToDps;       ///< 자이로 LSB 값을 DPS 단위 변환하는 계수

    uint8_t   _accelRange;     ///< 현재 가속도 측정 범위 (2G, 4G, 8G 등)
    uint8_t   _gyroRange;      ///< 현재 자이로 측정 범위 (250Dps, 500Dps 등)
    uint8_t   _accelAxisMask;  ///< 가속도 수집 대상 축 활성화 마스크
    uint8_t   _gyroAxisMask;   ///< 자이로 수집 대상 축 활성화 마스크
    bool      _gyroEnabled;    ///< 자이로 센서 실동작 활성화 여부
    T2_Type::EM_ChannelMask_t _audioChannelMask; ///< 오디오 채널 설정 마스크 (L / R / Both)

    // 가속도 FIFO 누적용 링 버퍼
    float     _accumX[T2_Def::Accel::Sensor::FFT_SIZE_MAX];
    float     _accumY[T2_Def::Accel::Sensor::FFT_SIZE_MAX];
    float     _accumZ[T2_Def::Accel::Sensor::FFT_SIZE_MAX];
    uint16_t  _accumWriteIdx;  ///< 가속도 버퍼 쓰기 지점
    uint16_t  _accumReadIdx;   ///< 가속도 버퍼 읽기 지점
    uint16_t  _accumCount;     ///< 현재 버퍼에 누적 보관된 샘플 크기

    // 자이로 FIFO 누적용 링 버퍼
    float     _accumGX[T2_Def::Gyro::Sensor::FFT_SIZE_MAX];
    float     _accumGY[T2_Def::Gyro::Sensor::FFT_SIZE_MAX];
    float     _accumGZ[T2_Def::Gyro::Sensor::FFT_SIZE_MAX];
    uint16_t  _accumGWriteIdx; ///< 자이로 버퍼 쓰기 지점
    uint16_t  _accumGReadIdx;  ///< 자이로 버퍼 읽기 지점
    uint16_t  _accumGCount;    ///< 현재 버퍼에 누적 보관된 샘플 크기

    // 32bit 오디오(L/R 교차 배치) 수신용 DMA 스크래치 버퍼
    alignas(16) int32_t _dmaBuffer[T2_Def::Audio::Sensor::FFT_SIZE_MAX * 2];

    char _statusText[32];      ///< 엔진 내부 로그 텍스트용 버퍼

    // BMI270 고속 레지스터 접근 내부 헬퍼 함수
    bool _readRegs(uint8_t p_reg, uint8_t* p_data, uint16_t p_len);
    bool _writeRegs(uint8_t p_reg, const uint8_t* p_data, uint16_t p_len);
    uint8_t _mapAccelRange(uint8_t p_rangeG);
    uint8_t _mapGyroRange(uint16_t p_rangeDps);

public:
    CL_T2_SensorEngine(SPIClass& p_spiBus);
    ~CL_T2_SensorEngine() = default;

    /**
     * @brief 설정 스냅샷을 적용하여 BMI270, I2S 마이크 구동
     * @return 장치 초기화 올바름 여부
     */
    bool init(const T2_Type::ST_Global_System_t& p_sysCfg,
              const T2_Type::ST_Accel_Config_t& p_accCfg,
              const T2_Type::ST_Gyro_Config_t& p_gyrCfg,
              const T2_Type::ST_Audio_Config_t& p_audCfg);

    /**
     * @brief 센서 수집 일시 중지
     */
    void pause();

    /**
     * @brief 센서 수집 재개
     */
    void resume();

    /**
     * @brief 오디오 수집 DMA 및 임시 내부 상태 소거
     */
    void clearAudioBuffer();

    /**
     * @brief 센서 디바이스 완전 리셋 및 통신 버스 재부팅
     */
    void resetHardware();

    /**
     * @brief I2S DMA 버스로부터 마이크 오디오 청크를 읽고 float 값으로 변환
     * @param p_outL, p_outR 변환 결과를 받을 좌/우 오디오 버퍼
     * @param p_reqSamples 요청 샘플 개수
     * @return 실제 읽어들인 샘플 개수
     */
    uint32_t readAudioChunk(float* p_outL, float* p_outR, uint32_t p_reqSamples);

    /**
     * @brief BMI270 FIFO 버스트 데이터를 수동으로 1회 파싱하여 일괄 인출
     * @return 파싱된 센서 프레임의 전체 개수
     */
    uint16_t readVibFifoBatch(float* p_accX, float* p_accY, float* p_accZ,
                              float* p_gyrX, float* p_gyrY, float* p_gyrZ,
                              uint16_t p_maxFrames);

    /**
     * @brief 하드웨어 FIFO 버퍼 상태를 백그라운드 링 버퍼에 누적
     * @return 누적 진행 완료 샘플 개수
     */
    uint16_t accumulateFifo();

    /**
     * @brief 누적 보관된 가속도 3축 데이터를 요청 샘플 수만큼 파퓰레이션
     * @return 인출 완료 샘플 개수
     */
    uint16_t getAccumulatedAccel(float* p_outX, float* p_outY, float* p_outZ, uint16_t p_reqCount);

    /**
     * @brief 누적 보관된 자이로 3축 데이터를 요청 샘플 수만큼 파퓰레이션
     * @return 인출 완료 샘플 개수
     */
    uint16_t getAccumulatedGyro(float* p_outX, float* p_outY, float* p_outZ, uint16_t p_reqCount);

    /**
     * @brief 현재 누적되어 보관 중인 가속도 샘플 개수 획득
     */
    uint16_t getAccumulatedAccelCount() const { return _accumCount; }

    /**
     * @brief 현재 누적되어 보관 중인 자이로 샘플 개수 획득
     */
    uint16_t getAccumulatedGyroCount() const { return _accumGCount; }

    /**
     * @brief BMI270 내장 온도 센서 값 판독 (섭씨)
     */
    float readTemperatureSensor();

    /**
     * @brief 가속도계 자동 캘리브레이션 절차 기동
     */
    bool runAccelCalibration();

    /**
     * @brief 자이로계 자동 캘리브레이션 절차 기동
     */
    bool runGyroCalibration();

    /**
     * @brief 파일 시스템에 보관된 가속도 캘리브레이션 오프셋 적용
     */
    bool applyStoredAccelCalibration();

    /**
     * @brief 파일 시스템에 보관된 자이로 캘리브레이션 오프셋 적용
     */
    bool applyStoredGyroCalibration();

    /**
     * @brief 하드웨어 모션 감지 인터럽트(WOM) 활성화
     */
    bool enableWakeOnMotion(float p_threshG, uint16_t p_duration);

    /**
     * @brief 센서 엔진 상태 문자열 획득
     */
    const char* getStatusText() const { return _statusText; }
};
