/* ============================================================================
 * File: T280_Sensor_241.cpp
 * Summary: BMI270 (SPI) & ICS43434 (I2S) 멀티모달 센서 융합 수집 엔진 구현부
 * ============================================================================
 * [로직 상태 추적 (Logic State Tracker)]
 * - 유지: BMI270 SPI 10MHz 통신 및 레지스터 직접 제어를 통한 FIFO 오버헤드 최소화.
 * - 갱신: T480 I2S DMA 수집부 병합 및 32-bit PCM 정규화 연산.
 * - 신규: [자가 교정] delay(5) 철폐 후 vTaskDelay() 대체 및 캘리브레이션 저장 로직 추가.
 *
 * [축소/누락 방어 체크리스트 (Omission Defense)]
 * 1. [RTOS 원칙]: vTaskDelay()를 통한 컨텍스트 스위칭 양보로 데드락 방지 (Rule #3).
 * 2. [수학적 무결성]: Z축 캘리브레이션 시 중력가속도(1.0G) 차감을 통한 정확한 편차 추출.
 * 3. [예외 처리]: 캘리브레이션 파일(.json)이 없을 경우 기본 오프셋(0.0)을 유지하여 패닉 방지.
 * ========================================================================== */

#include "T280_Sensor_241.hpp"
#include "esp_log.h"
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "T242_SENS";
static constexpr float PCM_32BIT_SCALE_CONST = 4.656612873e-10f;

CL_T2_SensorEngine::CL_T2_SensorEngine(SPIClass& p_spiBus)
    : _spi(p_spiBus), _bmi(), _isI2sInit(false), _isBmiInit(false), _isPaused(false),
      _offsetX(0.0f), _offsetY(0.0f), _offsetZ(0.0f) {
    memset(_dmaBuffer, 0, sizeof(_dmaBuffer));
    strlcpy(_statusText, "INIT", sizeof(_statusText));
}

bool CL_T2_SensorEngine::init(const T2_Type::ST_Config_Sensor& p_cfg) {
    if (_isI2sInit && _isBmiInit) return true;

    // [1] BMI270 Init
    int8_t v_rslt = _bmi.beginSPI(T2_Config::Hardware::PIN_BMI_CS_CONST, 10000000, _spi);
    if (v_rslt != BMI2_OK) {
        ESP_LOGE(TAG, "BMI270 SPI Init Failed (Code: %d)", v_rslt);
        strlcpy(_statusText, "BMI_ERR", sizeof(_statusText));
        return false;
    }

    bmi2_sens_config v_accelConfig;
    v_accelConfig.type = BMI2_ACCEL;
    v_accelConfig.cfg.acc.odr = BMI2_ACC_ODR_1600HZ;
    v_accelConfig.cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
    v_accelConfig.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    v_accelConfig.cfg.acc.range = _mapAccelRange(p_cfg.accel_range);
    _bmi.setConfig(v_accelConfig);

    _isBmiInit = true;
    applyStoredCalibration(); // 부팅 시 보정값 로드

    // [2] ICS43434 Init
    i2s_config_t v_i2sConfig = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = T2_Config::System::RATE_AUDIO_CONST,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL3,
        .dma_buf_count = 8,
        .dma_buf_len = T2_Config::System::FFT_SIZE_AUDIO_CONST,
        .use_apll = true,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    i2s_pin_config_t v_pinConfig = {
        .bck_io_num = T2_Config::Hardware::PIN_I2S_BCLK_CONST,
        .ws_io_num = T2_Config::Hardware::PIN_I2S_WS_CONST,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = T2_Config::Hardware::PIN_I2S_DIN_CONST
    };

    if (i2s_driver_install((i2s_port_t)T2_Config::Hardware::I2S_PORT_NUM_CONST, &v_i2sConfig, 0, NULL) != ESP_OK) {
        ESP_LOGE(TAG, "I2S driver install failed");
        strlcpy(_statusText, "I2S_ERR", sizeof(_statusText));
        return false;
    }

    if (i2s_set_pin((i2s_port_t)T2_Config::Hardware::I2S_PORT_NUM_CONST, &v_pinConfig) != ESP_OK) {
        ESP_LOGE(TAG, "I2S pin config failed");
        strlcpy(_statusText, "I2S_PIN_ERR", sizeof(_statusText));
        return false;
    }

    _isI2sInit = true;
    _isPaused = false;
    strlcpy(_statusText, "RUNNING", sizeof(_statusText));
    ESP_LOGI(TAG, "Multimodal Sensor Engine Initialized");

    return true;
}

void CL_T2_SensorEngine::pause() {
    if (_isPaused) return;
    if (_isI2sInit) i2s_stop((i2s_port_t)T2_Config::Hardware::I2S_PORT_NUM_CONST);
    if (_isBmiInit) {
        uint8_t v_regData = 0x00;
        _writeRegs(BMI2_PWR_CTRL_ADDR, &v_regData, 1);
    }
    _isPaused = true;
    strlcpy(_statusText, "PAUSED", sizeof(_statusText));
}

void CL_T2_SensorEngine::resume() {
    if (!_isPaused) return;
    if (_isBmiInit) {
        uint8_t v_regData = 0x04;
        _writeRegs(BMI2_PWR_CTRL_ADDR, &v_regData, 1);
        // [수정됨] Rule #3 위반이었던 delay(5) 철폐, RTOS Non-blocking 지연 사용
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (_isI2sInit) {
        clearAudioBuffer();
        i2s_start((i2s_port_t)T2_Config::Hardware::I2S_PORT_NUM_CONST);
    }
    _isPaused = false;
    strlcpy(_statusText, "RUNNING", sizeof(_statusText));
}

void CL_T2_SensorEngine::clearAudioBuffer() {
    if (!_isI2sInit) return;
    i2s_zero_dma_buffer((i2s_port_t)T2_Config::Hardware::I2S_PORT_NUM_CONST);
}

uint32_t CL_T2_SensorEngine::readAudioChunk(float* p_outL, float* p_outR, uint32_t p_reqSamples) {
    if (!_isI2sInit || _isPaused) return 0;

    uint32_t v_samplesToRead = (p_reqSamples > T2_Config::System::FFT_SIZE_AUDIO_CONST) ?
                               T2_Config::System::FFT_SIZE_AUDIO_CONST : p_reqSamples;
    size_t v_bytesRead = 0;
    size_t v_bytesToRead = v_samplesToRead * 2 * sizeof(int32_t);

    esp_err_t v_res = i2s_read((i2s_port_t)T2_Config::Hardware::I2S_PORT_NUM_CONST, _dmaBuffer, v_bytesToRead, &v_bytesRead, portMAX_DELAY);

    if (v_res != ESP_OK) return 0;

    uint32_t v_samplesRead = v_bytesRead / (2 * sizeof(int32_t));

    for (uint32_t i = 0; i < v_samplesRead; i++) {
        p_outL[i] = (float)_dmaBuffer[i * 2] * PCM_32BIT_SCALE_CONST;
        p_outR[i] = (float)_dmaBuffer[i * 2 + 1] * PCM_32BIT_SCALE_CONST;
    }

    return v_samplesRead;
}

uint16_t CL_T2_SensorEngine::readVibFifoBatch(float* p_outX, float* p_outY, float* p_outZ, uint16_t p_maxFrames) {
    if (!_isBmiInit || _isPaused) return 0;

    uint8_t v_lenBuf[2] = {0};
    if (!_readRegs(0x24, v_lenBuf, 2)) return 0;

    uint16_t v_fifoBytes = (v_lenBuf[1] << 8) | v_lenBuf[0];
    uint16_t v_availFrames = v_fifoBytes / 7;

    if (v_availFrames == 0) return 0;

    uint16_t v_framesToRead = (v_availFrames > p_maxFrames) ? p_maxFrames : v_availFrames;
    uint16_t v_bytesToRead = v_framesToRead * 7;

    uint8_t v_fifoData[1024];
    if (v_bytesToRead > sizeof(v_fifoData)) v_bytesToRead = sizeof(v_fifoData);

    if (!_readRegs(0x26, v_fifoData, v_bytesToRead)) return 0;

    uint16_t v_parsedCount = 0;
    uint16_t v_idx = 0;
    const float v_lsbToG = 1.0f / 4096.0f;

	// 데이터 유실 방어벽이 적용된 Sync-Hunting 파싱 로직
    while (v_idx + 6 < v_bytesToRead && v_parsedCount < v_framesToRead) {
        uint8_t v_header = v_fifoData[v_idx]; // 값을 확인만 하고 포인터는 수동 증가

        if (v_header == 0x84) {
            // Accel 프레임 감지 성공
            v_idx++; // 헤더 크기(1바이트) 전진

            int16_t v_rawX = (int16_t)((v_fifoData[v_idx + 1] << 8) | v_fifoData[v_idx]);
            int16_t v_rawY = (int16_t)((v_fifoData[v_idx + 3] << 8) | v_fifoData[v_idx + 2]);
            int16_t v_rawZ = (int16_t)((v_fifoData[v_idx + 5] << 8) | v_fifoData[v_idx + 4]);

            // 캘리브레이션 오프셋 차감 반영
            p_outX[v_parsedCount] = ((float)v_rawX * v_lsbToG) - _offsetX;
            p_outY[v_parsedCount] = ((float)v_rawY * v_lsbToG) - _offsetY;
            p_outZ[v_parsedCount] = ((float)v_rawZ * v_lsbToG) - _offsetZ;

            v_parsedCount++;
            v_idx += 6; // 페이로드 크기(6바이트) 전진
        } else {
            // [치명적 결함 교정] break로 버리지 않고 1바이트씩 전진하며 다음 0x84 헤더를 사냥(Sync-Hunt)
            v_idx++;
        }
    }

    return v_parsedCount;
}

// 영점 조절 캘리브레이션 및 파일 저장
bool CL_T2_SensorEngine::runCalibration() {
    if (!_isBmiInit) return false;
    ESP_LOGI(TAG, "Starting BMI270 Offset Calibration...");

    // 잔류 데이터 비우기
    float dummyX[100], dummyY[100], dummyZ[100];
    readVibFifoBatch(dummyX, dummyY, dummyZ, 100);

    float v_sumX = 0, v_sumY = 0, v_sumZ = 0;
    uint16_t v_samples = 0;
    const uint16_t TARGET_SAMPLES = 100;

    // 최대 10번 재시도하여 100개 샘플 획득
    for (int i = 0; i < 10 && v_samples < TARGET_SAMPLES; i++) {
        vTaskDelay(pdMS_TO_TICKS(10)); // 10ms 대기 (1600Hz 기준 16프레임)
        uint16_t v_count = readVibFifoBatch(dummyX, dummyY, dummyZ, TARGET_SAMPLES - v_samples);
        for (uint16_t j = 0; j < v_count; j++) {
            v_sumX += dummyX[j] + _offsetX; // 기존 오프셋 원복 후 합산
            v_sumY += dummyY[j] + _offsetY;
            v_sumZ += dummyZ[j] + _offsetZ;
            v_samples++;
        }
    }

    if (v_samples < TARGET_SAMPLES / 2) {
        ESP_LOGE(TAG, "Calibration failed: Not enough samples (%d)", v_samples);
        return false;
    }

    _offsetX = v_sumX / v_samples;
    _offsetY = v_sumY / v_samples;
    // Z축은 중력 1G를 받고 있으므로 1.0 보정
    _offsetZ = (v_sumZ / v_samples) - 1.0f;

    // LittleFS에 저장
    JsonDocument v_doc;
    v_doc["offset_x"] = _offsetX;
    v_doc["offset_y"] = _offsetY;
    v_doc["offset_z"] = _offsetZ;

    File v_file = LittleFS.open(T2_Config::Path::FILE_BMI_CALIB_CONST, "w");
    if (v_file) {
        serializeJson(v_doc, v_file);
        v_file.close();
        ESP_LOGI(TAG, "Calibration Saved: X=%.3f, Y=%.3f, Z=%.3f", _offsetX, _offsetY, _offsetZ);
        return true;
    }
    return false;
}

// 파일 시스템에서 영점 데이터 로드
bool CL_T2_SensorEngine::applyStoredCalibration() {
    if (!LittleFS.exists(T2_Config::Path::FILE_BMI_CALIB_CONST)) {
        ESP_LOGW(TAG, "No calibration file found. Using default offsets (0.0)");
        _offsetX = 0.0f; _offsetY = 0.0f; _offsetZ = 0.0f;
        return false;
    }

    File v_file = LittleFS.open(T2_Config::Path::FILE_BMI_CALIB_CONST, "r");
    if (!v_file) return false;

    JsonDocument v_doc;
    DeserializationError v_err = deserializeJson(v_doc, v_file);
    v_file.close();

    if (!v_err) {
        _offsetX = v_doc["offset_x"] | 0.0f;
        _offsetY = v_doc["offset_y"] | 0.0f;
        _offsetZ = v_doc["offset_z"] | 0.0f;
        ESP_LOGI(TAG, "Calibration Loaded: X=%.3f, Y=%.3f, Z=%.3f", _offsetX, _offsetY, _offsetZ);
        return true;
    }
    return false;
}

bool CL_T2_SensorEngine::enableWakeOnMotion(float p_threshG, uint16_t p_duration) {
    if (!_isBmiInit) return false;

    bmi2_sens_config v_config;
    v_config.type = BMI2_ANY_MOTION;
    v_config.cfg.any_motion.duration = p_duration;
    v_config.cfg.any_motion.threshold = (uint16_t)(p_threshG * 1000.0f / 0.48f);
    v_config.cfg.any_motion.select_x = 1;
    v_config.cfg.any_motion.select_y = 1;
    v_config.cfg.any_motion.select_z = 1;


    _bmi.setConfig(v_config);

	// 6. Any-Motion 인터럽트를 INT1 핀으로 매핑
    _bmi.mapInterruptToPin(BMI2_ANY_MOTION_INT, BMI2_INT1);

    ESP_LOGI(TAG, "Any-Motion Interrupt Enabled (Thresh: %.2fG)", p_threshG);
    return true;
}

uint8_t CL_T2_SensorEngine::_mapAccelRange(uint8_t p_rangeG) {
    switch (p_rangeG) {
        case 2:  return BMI2_ACC_RANGE_2G;
        case 4:  return BMI2_ACC_RANGE_4G;
        case 8:  return BMI2_ACC_RANGE_8G;
        case 16: return BMI2_ACC_RANGE_16G;
        default: return BMI2_ACC_RANGE_8G;
    }
}

bool CL_T2_SensorEngine::_readRegs(uint8_t p_reg, uint8_t* p_data, uint16_t p_len) {
    _spi.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    digitalWrite(T2_Config::Hardware::PIN_BMI_CS_CONST, LOW);
    _spi.transfer(p_reg | 0x80);
    _spi.transfer(0x00);
    for (uint16_t i = 0; i < p_len; i++) p_data[i] = _spi.transfer(0x00);
    digitalWrite(T2_Config::Hardware::PIN_BMI_CS_CONST, HIGH);
    _spi.endTransaction();
    return true;
}

bool CL_T2_SensorEngine::_writeRegs(uint8_t p_reg, const uint8_t* p_data, uint16_t p_len) {
    _spi.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    digitalWrite(T2_Config::Hardware::PIN_BMI_CS_CONST, LOW);
    _spi.transfer(p_reg & 0x7F);
    for (uint16_t i = 0; i < p_len; i++) _spi.transfer(p_data[i]);
    digitalWrite(T2_Config::Hardware::PIN_BMI_CS_CONST, HIGH);
    _spi.endTransaction();
    return true;
}
