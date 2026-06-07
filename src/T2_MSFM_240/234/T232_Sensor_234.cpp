
/* ============================================================================
 * File: T232_Sensor_234.cpp
 * Summary: BMI270 Driver & Data Acquisition Engine Implementation
 * ========================================================================== */
#include "T232_Sensor_234.h"
#include <ArduinoJson.h>
#include <LittleFS.h>

CL_T20_SensorEngine::CL_T20_SensorEngine(SPIClass& spi_bus) : _spi(spi_bus), _bmi() {
    memset(_status_text, 0, sizeof(_status_text));
    strlcpy(_status_text, "idle", sizeof(_status_text));
}

bool CL_T20_SensorEngine::begin(const ST_T20_ConfigSensor_t& s_cfg) {
    // [1] SPI 통신 시작
    int8_t rslt = _bmi.beginSPI(T20::C10_Pin::BMI_CS, T20::C10_BMI::SPI_FREQ_HZ, _spi);
    if (rslt != BMI2_OK) {
        strlcpy(_status_text, "spi_init_fail", sizeof(_status_text));
        return false;
    }

    // [2] 가속도계 설정 (1600Hz, Normal Avg4)
    bmi2_sens_config accelConfig;
    accelConfig.type                = BMI2_ACCEL;
    accelConfig.cfg.acc.odr         = BMI2_ACC_ODR_1600HZ;
    accelConfig.cfg.acc.bwp         = BMI2_ACC_NORMAL_AVG4;
    accelConfig.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    accelConfig.cfg.acc.range       = _mapAccelRange(s_cfg.accel_range);
    _bmi.setConfig(accelConfig);

    // [3] 자이로스코프 설정 (1600Hz, Normal)
    bmi2_sens_config gyroConfig;
    gyroConfig.type                 = BMI2_GYRO;
    gyroConfig.cfg.gyr.odr          = BMI2_GYR_ODR_1600HZ;
    gyroConfig.cfg.gyr.bwp          = BMI2_GYR_NORMAL_MODE;
    gyroConfig.cfg.gyr.filter_perf  = BMI2_PERF_OPT_MODE;
    gyroConfig.cfg.gyr.noise_perf   = BMI2_PERF_OPT_MODE;
    gyroConfig.cfg.gyr.range        = _mapGyroRange(s_cfg.gyro_range);
    _bmi.setConfig(gyroConfig);

    // [4] FIFO 설정 (Accel + Gyro 통합 보관)
    BMI270_FIFOConfig fifoConfig;
    fifoConfig.flags       = BMI2_FIFO_ACC_EN | BMI2_FIFO_GYR_EN;
    fifoConfig.watermark   = 16;  // 16샘플(약 10ms) 마다 인터럽트 발생 트리거
    fifoConfig.accelFilter = BMI2_ENABLE;
    fifoConfig.gyroFilter  = BMI2_ENABLE;
    _bmi.setFIFOConfig(fifoConfig);

    // [5] 인터럽트 핀 설정 (INT1 - Active High, Push-Pull)
    _bmi.mapInterruptToPin(BMI2_FWM_INT, BMI2_INT1);
    bmi2_int_pin_config intPinConfig;
    intPinConfig.pin_type             = BMI2_INT1;
    intPinConfig.pin_cfg[0].lvl       = BMI2_INT_ACTIVE_HIGH;
    intPinConfig.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
    _bmi.setInterruptPinConfig(intPinConfig);

    // [6] 기존에 백업된 보정값(Calibration)이 있다면 주입
    applyStoredCalibration();

    _initialized = true;
    strlcpy(_status_text, "1600Hz_active", sizeof(_status_text));
    return true;
}

/* ============================================================================
 *  FSM 최적화: 센서 절전 모드 제어
 * ========================================================================== */
 
 /* ============================================================================
 * [FSM 최적화] 센서 절전 모드 제어 (에러 수정됨)
 * ========================================================================== */
void CL_T20_SensorEngine::pause() {
    if (!_initialized || _is_paused) return;
    
    // SparkFun 라이브러리 지원 함수를 통한 전력 최적화 절전 모드 진입
    _bmi.enableAdvancedPowerSave(true);
    
    _is_paused = true;
    strlcpy(_status_text, "paused", sizeof(_status_text));
}

void CL_T20_SensorEngine::resume() {
    if (!_initialized || !_is_paused) return;
    
    // 절전 모드 해제 및 정상 가동 복귀
    _bmi.disableAdvancedPowerSave();
    
    _is_paused = false;
    strlcpy(_status_text, "1600Hz_active", sizeof(_status_text));
}


uint16_t CL_T20_SensorEngine::readFifoBatch(float* p_out_x, float* p_out_y, float* p_out_z, 
                                            uint16_t max_frames, EM_T20_AxisCount_t axis_count, 
                                            EM_T20_SensorAxis_t target_axis) {
                                                
    // [정합성 보완] 일시 정지(pause) 상태일 때는 SPI 버스를 낭비하지 않고 즉시 반환합니다.
    if (!_initialized || !p_out_x || _is_paused) return 0;


    uint16_t fifo_bytes = 0;
    if (_bmi.getFIFOLength(&fifo_bytes) != BMI2_OK || fifo_bytes == 0) return 0;

    // BMI270 FIFO 1프레임: Accel(6) + Gyro(6) = 12 bytes
    static BMI270_SensorData fifo_raw[32]; 
    uint16_t frames_to_read = (fifo_bytes / 12);
    if (frames_to_read > 32) frames_to_read = 32;
    if (frames_to_read > max_frames) frames_to_read = max_frames;

    if (_bmi.getFIFOData(fifo_raw, &frames_to_read) != BMI2_OK) return 0;

    for (uint16_t i = 0; i < frames_to_read; i++) {
        if (axis_count == EN_T20_AXIS_TRIPLE) {
            
            if (axis_count == EN_T20_AXIS_TRIPLE) {
                if (target_axis >= 3) { // 자이로 계열이 선택된 경우
                    p_out_x[i] = fifo_raw[i].gyroX;
                    p_out_y[i] = fifo_raw[i].gyroY;
                    p_out_z[i] = fifo_raw[i].gyroZ;
                } else { //  가속도 X, Y, Z를 각각의 포인터 버퍼에 분리 저장
                    p_out_x[i] = fifo_raw[i].accelX;
                    p_out_y[i] = fifo_raw[i].accelY;
                    p_out_z[i] = fifo_raw[i].accelZ;
                }
            }
        } else {
            // [1축 모드]: 타겟으로 지정된 단일 물리축만 p_out_x 버퍼에 저장
            switch (target_axis) {
                case EN_T20_AXIS_ACCEL_X: p_out_x[i] = fifo_raw[i].accelX; break;
                case EN_T20_AXIS_ACCEL_Y: p_out_x[i] = fifo_raw[i].accelY; break;
                case EN_T20_AXIS_ACCEL_Z: p_out_x[i] = fifo_raw[i].accelZ; break;
                case EN_T20_AXIS_GYRO_X:  p_out_x[i] = fifo_raw[i].gyroX;  break;
                case EN_T20_AXIS_GYRO_Y:  p_out_x[i] = fifo_raw[i].gyroY;  break;
                case EN_T20_AXIS_GYRO_Z:  p_out_x[i] = fifo_raw[i].gyroZ;  break;
                default:                  p_out_x[i] = fifo_raw[i].accelZ; break;
            }
        }
    }
    return frames_to_read;
}

bool CL_T20_SensorEngine::runCalibration() {
    // 1. 센서 내장 캘리브레이션 엔진 구동
    _bmi.performComponentRetrim();
    _bmi.performAccelOffsetCalibration(BMI2_GRAVITY_POS_Z);
    _bmi.performGyroOffsetCalibration();

    // 2. Direct SPI로 7바이트 오프셋 레지스터 추출 (0x71 ~ 0x77)
    uint8_t offsets[7] = {0};
    if (!_readRegs(T20::C10_BMI::REG_CALIB_OFFSET_START, offsets, 7)) return false;

    // 3. 결과를 LittleFS에 백업 저장 (ArduinoJson V7)
    JsonDocument doc;
    JsonArray arr = doc["offsets"].to<JsonArray>();
    for (int i = 0; i < 7; i++) arr.add(offsets[i]);

    File f = LittleFS.open(T20::C10_Path::FILE_BMI_CALIB, "w");
    if (f) {
        serializeJson(doc, f);
        f.close();
    }
    return true;
}

bool CL_T20_SensorEngine::applyStoredCalibration() {
    if (!LittleFS.exists(T20::C10_Path::FILE_BMI_CALIB)) return false;

    File f = LittleFS.open(T20::C10_Path::FILE_BMI_CALIB, "r");
    if (!f) return false;

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, f);
    f.close();

    if (error) return false;

    uint8_t offsets[7];
    // ArduinoJson V7: 읽기 전용은 JsonArrayConst 사용
    JsonArrayConst arr = doc["offsets"];

    if (!arr || arr.size() < 7) return false;

    for (int i = 0; i < 7; i++) {
        offsets[i] = arr[i];
    }

    return _writeRegs(T20::C10_BMI::REG_CALIB_OFFSET_START, offsets, 7);
}

bool CL_T20_SensorEngine::enableWakeOnMotion(float threshold_g, uint16_t duration) {
    if (!_initialized) return false;

    // 1. 가속도 센서를 전력 최적화(저전력) 모드로 전환
    _bmi.setAccelPowerMode(BMI2_POWER_OPT_MODE);

    // 2. Any-Motion 기능 활성화
    _bmi.enableFeature(BMI2_ANY_MOTION);

    bmi2_sens_config any_mo_cfg;
    any_mo_cfg.type = BMI2_ANY_MOTION;

    // 3. 지속 시간 (1 단위 = 20ms)
    any_mo_cfg.cfg.any_motion.duration = duration;

    // 4. 임계값 설정 (11비트 해상도: 0~2047로 0~1G 표현)
    uint16_t threshold_lsb = (uint16_t)(threshold_g * 2048.0f);
    if (threshold_lsb > 2047) {
        threshold_lsb = 2047;  // 11비트 최대값 초과(Overflow) 방지 클램핑
    }
    any_mo_cfg.cfg.any_motion.threshold = threshold_lsb;

    // 5. 감시할 축 활성화 (전체 축)
    any_mo_cfg.cfg.any_motion.select_x = BMI2_ENABLE;
    any_mo_cfg.cfg.any_motion.select_y = BMI2_ENABLE;
    any_mo_cfg.cfg.any_motion.select_z = BMI2_ENABLE;

    _bmi.setConfig(any_mo_cfg);

    // 6. Any-Motion 인터럽트를 INT1 핀으로 매핑
    _bmi.mapInterruptToPin(BMI2_ANY_MOTION_INT, BMI2_INT1);

    return true;
}

void CL_T20_SensorEngine::resetHardware() {
    _bmi.reset();
    _initialized = false;
    strlcpy(_status_text, "reset", sizeof(_status_text));
}

// --- Private Helpers ---

bool CL_T20_SensorEngine::_readRegs(uint8_t reg, uint8_t* data, uint16_t len) {
    _spi.beginTransaction(SPISettings(T20::C10_BMI::SPI_FREQ_HZ, MSBFIRST, SPI_MODE0));
    digitalWrite(T20::C10_Pin::BMI_CS, LOW);
    _spi.transfer(reg | 0x80);  // Read Flag 적용 (MSB 1)
    _spi.transfer(0x00);        // BMI270 Read Dummy Byte 필수 규격
    for (uint16_t i = 0; i < len; i++) data[i] = _spi.transfer(0x00);
    digitalWrite(T20::C10_Pin::BMI_CS, HIGH);
    _spi.endTransaction();
    return true;
}

bool CL_T20_SensorEngine::_writeRegs(uint8_t reg, const uint8_t* data, uint16_t len) {
    _spi.beginTransaction(SPISettings(T20::C10_BMI::SPI_FREQ_HZ, MSBFIRST, SPI_MODE0));
    digitalWrite(T20::C10_Pin::BMI_CS, LOW);
    _spi.transfer(reg & 0x7F);  // Write Flag 적용 (MSB 0)
    for (uint16_t i = 0; i < len; i++) _spi.transfer(data[i]);
    digitalWrite(T20::C10_Pin::BMI_CS, HIGH);
    _spi.endTransaction();
    return true;
}

uint8_t CL_T20_SensorEngine::_mapAccelRange(EM_T20_AccelRange_t r) {
    switch (r) {
        case EN_T20_ACCEL_2G:  return BMI2_ACC_RANGE_2G;
        case EN_T20_ACCEL_4G:  return BMI2_ACC_RANGE_4G;
        case EN_T20_ACCEL_16G: return BMI2_ACC_RANGE_16G;
        default:               return BMI2_ACC_RANGE_8G;
    }
}

uint8_t CL_T20_SensorEngine::_mapGyroRange(EM_T20_GyroRange_t r) {
    switch (r) {
        case EN_T20_GYRO_125:  return BMI2_GYR_RANGE_125;
        case EN_T20_GYRO_250:  return BMI2_GYR_RANGE_250;
        case EN_T20_GYRO_500:  return BMI2_GYR_RANGE_500;
        case EN_T20_GYRO_1000: return BMI2_GYR_RANGE_1000;
        default:               return BMI2_GYR_RANGE_2000;
    }
}





