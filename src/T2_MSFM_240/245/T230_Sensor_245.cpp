#include "T230_Sensor_245.hpp"
#include "esp_log.h"
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

using namespace T2_Def;

static const char* TAG = "T245_SENS";

/**
 * @brief CL_T2_SensorEngine 생성자
 * @param p_spiBus 사용할 SPI 버스 참조
 */
CL_T2_SensorEngine::CL_T2_SensorEngine(SPIClass& p_spiBus)
    : _spi(p_spiBus), _bmi(), _isI2sInit(false), _isBmiInit(false), _isPaused(false),
      _accOffsetX(0.0f), _accOffsetY(0.0f), _accOffsetZ(0.0f),
      _accGainX(1.0f), _accGainY(1.0f), _accGainZ(1.0f),
      _gyrOffsetX(0.0f), _gyrOffsetY(0.0f), _gyrOffsetZ(0.0f),
      _gyrGainX(1.0f), _gyrGainY(1.0f), _gyrGainZ(1.0f),
      _lsbToG(1.0f / 4096.0f), _lsbToDps(0.0f),
      _accelRange(8), _gyroRange(250),
      _accelAxisMask(0b111), _gyroAxisMask(0b111),
      _gyroEnabled(false), _audioChannelMask(T2_Type::EM_ChannelMask_t::CH_STEREO),
      _accumWriteIdx(0), _accumReadIdx(0), _accumCount(0),
      _accumGWriteIdx(0), _accumGReadIdx(0), _accumGCount(0) {
    memset(_dmaBuffer, 0, sizeof(_dmaBuffer));
    memset(_accumX, 0, sizeof(_accumX));
    memset(_accumY, 0, sizeof(_accumY));
    memset(_accumZ, 0, sizeof(_accumZ));
    memset(_accumGX, 0, sizeof(_accumGX));
    memset(_accumGY, 0, sizeof(_accumGY));
    memset(_accumGZ, 0, sizeof(_accumGZ));
    strlcpy(_statusText, "INIT", sizeof(_statusText));
}

/**
 * @brief 설정 정보 스냅샷 기반으로 BMI270(가속도/자이로) 및 ICS43434(I2S 마이크) 초기화
 * @return 초기화 성공 여부
 */
bool CL_T2_SensorEngine::init(const T2_Type::ST_Global_System_t& p_sysCfg,
                              const T2_Type::ST_Accel_Config_t& p_accCfg,
                              const T2_Type::ST_Gyro_Config_t& p_gyrCfg,
                              const T2_Type::ST_Audio_Config_t& p_audCfg) {
    bool v_success = true;

    // [처리 단위 1] BMI270 하드웨어 및 기본 파라미터 초기화
    bool v_needsBmi = (p_accCfg.enable || p_gyrCfg.enable);
    if (v_needsBmi && !_isBmiInit) {
        int8_t v_rslt = _bmi.beginSPI(Imu::Hardware::PIN_CS_CONST, Imu::Hardware::SPI_FREQ_HZ_CONST, _spi);
        if (v_rslt != BMI2_OK) {
            ESP_LOGE(TAG, "BMI270 SPI Init Failed (Code: %d)", v_rslt);
            strlcpy(_statusText, "BMI_ERR", sizeof(_statusText));
            v_success = false;
        } else {
            _accelRange     = p_accCfg.range;
            _gyroRange      = p_gyrCfg.range;
            _accelAxisMask  = p_accCfg.axis_mask;
            _gyroAxisMask   = p_gyrCfg.axis_mask;
            _gyroEnabled    = p_gyrCfg.enable;

            _lsbToG         = (float)_accelRange / 32768.0f;
            _lsbToDps       = (float)_gyroRange / 32768.0f;

            pinMode(Imu::Hardware::PIN_INT1_CONST, INPUT);

            // [처리 단위 2] 가속도 센서 세부 ODR, 범위 설정 적용
            if (p_accCfg.enable) {
                bmi2_sens_config v_accelConfig;
                v_accelConfig.type                = BMI2_ACCEL;
                v_accelConfig.cfg.acc.odr         = p_accCfg.odr;
                v_accelConfig.cfg.acc.bwp         = p_accCfg.bwp;
                v_accelConfig.cfg.acc.filter_perf = p_accCfg.filter_perf;
                v_accelConfig.cfg.acc.range       = _mapAccelRange(p_accCfg.range);
                _bmi.setConfig(v_accelConfig);
            }

            // [처리 단위 3] 자이로 센서 세부 ODR, 범위 설정 적용
            if (_gyroEnabled) {
                bmi2_sens_config v_gyroConfig;
                v_gyroConfig.type               = BMI2_GYRO;
                v_gyroConfig.cfg.gyr.odr        = p_gyrCfg.odr;
                v_gyroConfig.cfg.gyr.bwp        = p_gyrCfg.bwp;
                v_gyroConfig.cfg.gyr.filter_perf= p_gyrCfg.filter_perf;
                v_gyroConfig.cfg.gyr.noise_perf = p_gyrCfg.noise_perf;
                v_gyroConfig.cfg.gyr.range      = _mapGyroRange(p_gyrCfg.range);
                _bmi.setConfig(v_gyroConfig);
            }

            // [처리 단위 4] 센서 내부 FIFO 데이터 형태 매핑 및 워터마크 설정
            BMI270_FIFOConfig v_fifoConfig;
            v_fifoConfig.flags = 0;
            if (p_accCfg.enable) v_fifoConfig.flags |= BMI2_FIFO_ACC_EN;
            if (_gyroEnabled)    v_fifoConfig.flags |= BMI2_FIFO_GYR_EN;

            v_fifoConfig.watermark   = p_accCfg.fifo_watermark;
            v_fifoConfig.accelFilter = BMI2_ENABLE;
            v_fifoConfig.gyroFilter  = BMI2_ENABLE;
            _bmi.setFIFOConfig(v_fifoConfig);

            _isBmiInit = true;
            applyStoredAccelCalibration();
            applyStoredGyroCalibration();
            ESP_LOGI(TAG, "IMU Domain Initialized (Acc:%d, Gyr:%d)", p_accCfg.enable, _gyroEnabled);
        }
    }

    // [처리 단위 5] I2S 마이크 드라이버 인스톨 및 핀 매핑
    if (p_audCfg.enable && !_isI2sInit) {
        i2s_config_t v_i2sConfig = {
            .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
            .sample_rate          = p_audCfg.sample_rate,
            .bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT,
            .channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT,
            .communication_format = I2S_COMM_FORMAT_STAND_I2S,
            .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL3,
            .dma_buf_count        = Audio::Hardware::DMA_BUF_COUNT_OPT_CONST,
            .dma_buf_len          = Audio::Hardware::DMA_BUF_LEN_OPT_CONST,
            .use_apll             = T2_Def::Audio::Hardware::USE_APLL_DEF,
            .tx_desc_auto_clear   = false,
            .fixed_mclk           = 0};

        i2s_pin_config_t v_pinConfig = {
            .bck_io_num = Audio::Hardware::PIN_I2S_BCLK_CONST,
            .ws_io_num = Audio::Hardware::PIN_I2S_WS_CONST,
            .data_out_num = I2S_PIN_NO_CHANGE,
            .data_in_num = Audio::Hardware::PIN_I2S_DIN_CONST
        };

        if (i2s_driver_install((i2s_port_t)Audio::Hardware::I2S_PORT_NUM_CONST, &v_i2sConfig, 0, NULL) != ESP_OK) {
            ESP_LOGE(TAG, "I2S driver install failed");
            strlcpy(_statusText, "I2S_ERR", sizeof(_statusText));
            v_success = false;
        } else {
            if (i2s_set_pin((i2s_port_t)Audio::Hardware::I2S_PORT_NUM_CONST, &v_pinConfig) != ESP_OK) {
                ESP_LOGE(TAG, "I2S pin config failed");
                strlcpy(_statusText, "I2S_PIN_ERR", sizeof(_statusText));
                v_success = false;
            } else {
                _isI2sInit = true;
                _audioChannelMask = static_cast<T2_Type::EM_ChannelMask_t>(p_audCfg.channel_mask);
                ESP_LOGI(TAG, "Audio Domain Initialized (Rate:%d, Mask:0x%X)", p_audCfg.sample_rate, (uint8_t)_audioChannelMask);
            }
        }
    }

    if (v_success) {
        _isPaused = false;
        strlcpy(_statusText, "RUNNING", sizeof(_statusText));
    }

    return v_success;
}

/**
 * @brief 센서 동작 일시 정지 (I2S 정지 및 IMU 전원 다운 제어)
 */
void CL_T2_SensorEngine::pause() {
    if (_isPaused) return;
    if (_isI2sInit) i2s_stop((i2s_port_t)Audio::Hardware::I2S_PORT_NUM_CONST);
    if (_isBmiInit) {
        uint8_t v_regData = 0x00;
        _writeRegs(BMI2_PWR_CTRL_ADDR, &v_regData, 1);
    }
    _isPaused = true;
    strlcpy(_statusText, "PAUSED", sizeof(_statusText));
}

/**
 * @brief 센서 동작 재개 (I2S 기동 및 IMU 전원 인가)
 */
void CL_T2_SensorEngine::resume() {
    if (!_isPaused) return;
    if (_isBmiInit) {
        uint8_t v_regData = 0x04;
        if (_gyroEnabled) v_regData |= 0x02;
        _writeRegs(BMI2_PWR_CTRL_ADDR, &v_regData, 1);
        vTaskDelay(pdMS_TO_TICKS(Imu::Hardware::STARTUP_DELAY_MS_CONST));
    }
    if (_isI2sInit) {
        clearAudioBuffer();
        i2s_start((i2s_port_t)Audio::Hardware::I2S_PORT_NUM_CONST);
    }
    _isPaused = false;
    strlcpy(_statusText, "RUNNING", sizeof(_statusText));
}

/**
 * @brief 센서 엔진 완전 리셋 (초기화 여부 파괴)
 */
void CL_T2_SensorEngine::resetHardware() {
    if (_isBmiInit) _bmi.reset();
    _isBmiInit = false;
    _isI2sInit = false;
    strlcpy(_statusText, "RESET", sizeof(_statusText));
}

/**
 * @brief 오디오 데이터 수집 I2S DMA 버퍼 플러시
 */
void CL_T2_SensorEngine::clearAudioBuffer() {
    if (!_isI2sInit) return;
    i2s_zero_dma_buffer((i2s_port_t)Audio::Hardware::I2S_PORT_NUM_CONST);
}

/**
 * @brief DMA 버퍼로부터 마이크 오디오 샘플 리드 및 float 정규화 변환
 * @param p_outL 좌측 데이터 버퍼
 * @param p_outR 우측 데이터 버퍼
 * @param p_reqSamples 취득할 샘플 수
 * @return 실제 읽어들인 샘플 크기
 */
uint32_t CL_T2_SensorEngine::readAudioChunk(float* p_outL, float* p_outR, uint32_t p_reqSamples) {
    if (!_isI2sInit || _isPaused) return 0;

    uint32_t v_samplesToRead = (p_reqSamples > Audio::Sensor::FFT_SIZE_MAX) ?
                               Audio::Sensor::FFT_SIZE_MAX : p_reqSamples;
    size_t v_bytesRead = 0;
    size_t v_bytesToRead = v_samplesToRead * 2 * sizeof(int32_t);

    esp_err_t v_res = i2s_read((i2s_port_t)Audio::Hardware::I2S_PORT_NUM_CONST, _dmaBuffer, v_bytesToRead, &v_bytesRead, portMAX_DELAY);
    if (v_res != ESP_OK) return 0;

    uint32_t v_samplesRead = v_bytesRead / (2 * sizeof(int32_t));

    bool hasLeft  = ((uint8_t)_audioChannelMask & (uint8_t)T2_Type::EM_ChannelMask_t::CH_LEFT);
    bool hasRight = ((uint8_t)_audioChannelMask & (uint8_t)T2_Type::EM_ChannelMask_t::CH_RIGHT);
    bool isStereo = hasLeft && hasRight;

    if (isStereo) {
        for (uint32_t i = 0; i < v_samplesRead; i++) {
            p_outL[i] = (float)_dmaBuffer[i * 2]     * Audio::Hardware::PCM_32BIT_SCALE_CONST;
            p_outR[i] = (float)_dmaBuffer[i * 2 + 1] * Audio::Hardware::PCM_32BIT_SCALE_CONST;
        }
    } else if (hasLeft) {
        for (uint32_t i = 0; i < v_samplesRead; i++) {
            p_outL[i] = (float)_dmaBuffer[i * 2] * Audio::Hardware::PCM_32BIT_SCALE_CONST;
            p_outR[i] = 0.0f;
        }
    } else if (hasRight) {
        for (uint32_t i = 0; i < v_samplesRead; i++) {
            p_outL[i] = (float)_dmaBuffer[i * 2 + 1] * Audio::Hardware::PCM_32BIT_SCALE_CONST;
            p_outR[i] = 0.0f;
        }
    } else {
        memset(p_outL, 0, v_samplesRead * sizeof(float));
        memset(p_outR, 0, v_samplesRead * sizeof(float));
    }

    return v_samplesRead;
}

/**
 * @brief 하드웨어 FIFO 버퍼 상태를 버스트로 대량 리드하여 가속도/자이로 파싱 처리
 */
uint16_t CL_T2_SensorEngine::readVibFifoBatch(float* p_accX, float* p_accY, float* p_accZ,
                                          float* p_gyrX, float* p_gyrY, float* p_gyrZ,
                                          uint16_t p_maxFrames) {
    if (!_isBmiInit || _isPaused) return 0;

    uint8_t v_lenBuf[2] = {0};
    if (!_readRegs(Imu::Hardware::REG_FIFO_LEN_ADDR_CONST, v_lenBuf, 2)) return 0;

    uint16_t v_fifoBytes = (v_lenBuf[1] << 8) | v_lenBuf[0];
    uint16_t v_availFrames = v_fifoBytes / Imu::Hardware::FIFO_FRAME_SIZE_CONST;

    if (v_availFrames == 0) return 0;

    uint16_t v_framesToRead = (v_availFrames > p_maxFrames) ? p_maxFrames : v_availFrames;
    uint16_t v_bytesToRead = v_framesToRead * Imu::Hardware::FIFO_FRAME_SIZE_CONST;

    uint8_t v_fifoData[Imu::Hardware::FIFO_BATCH_SIZE_MAX * Imu::Hardware::FIFO_FRAME_SIZE_CONST];
    if (v_bytesToRead > sizeof(v_fifoData)) v_bytesToRead = sizeof(v_fifoData);

    if (!_readRegs(Imu::Hardware::REG_FIFO_DATA_ADDR_CONST, v_fifoData, v_bytesToRead)) return 0;

    uint16_t v_accCount = 0;
    uint16_t v_gyrCount = 0;
    uint16_t v_idx = 0;

    while (v_idx + (Imu::Hardware::FIFO_FRAME_SIZE_CONST - 1) < v_bytesToRead &&
           (v_accCount < v_framesToRead || v_gyrCount < v_framesToRead)) {
        uint8_t v_header = v_fifoData[v_idx];

        if (v_header == Imu::Hardware::FIFO_HEADER_ACCEL_CONST) {
            v_idx++;
            int16_t v_rawX = (int16_t)((v_fifoData[v_idx + 1] << 8) | v_fifoData[v_idx]);
            int16_t v_rawY = (int16_t)((v_fifoData[v_idx + 3] << 8) | v_fifoData[v_idx + 2]);
            int16_t v_rawZ = (int16_t)((v_fifoData[v_idx + 5] << 8) | v_fifoData[v_idx + 4]);

            if (v_accCount < p_maxFrames) {
                p_accX[v_accCount] = (_accelAxisMask & (1 << 0)) ? (((float)v_rawX * _lsbToG - _accOffsetX) * _accGainX) : 0.0f;
                p_accY[v_accCount] = (_accelAxisMask & (1 << 1)) ? (((float)v_rawY * _lsbToG - _accOffsetY) * _accGainY) : 0.0f;
                p_accZ[v_accCount] = (_accelAxisMask & (1 << 2)) ? (((float)v_rawZ * _lsbToG - _accOffsetZ) * _accGainZ) : 0.0f;
                v_accCount++;
            }
            v_idx += 6;
        } else if (v_header == Imu::Hardware::FIFO_HEADER_GYRO_CONST) {
            v_idx++;
            int16_t v_rawGX = (int16_t)((v_fifoData[v_idx + 1] << 8) | v_fifoData[v_idx]);
            int16_t v_rawGY = (int16_t)((v_fifoData[v_idx + 3] << 8) | v_fifoData[v_idx + 2]);
            int16_t v_rawGZ = (int16_t)((v_fifoData[v_idx + 5] << 8) | v_fifoData[v_idx + 4]);

            if (_gyroEnabled && v_gyrCount < p_maxFrames) {
                p_gyrX[v_gyrCount] = (_gyroAxisMask & (1 << 0)) ? (((float)v_rawGX * _lsbToDps - _gyrOffsetX) * _gyrGainX) : 0.0f;
                p_gyrY[v_gyrCount] = (_gyroAxisMask & (1 << 1)) ? (((float)v_rawGY * _lsbToDps - _gyrOffsetY) * _gyrGainY) : 0.0f;
                p_gyrZ[v_gyrCount] = (_gyroAxisMask & (1 << 2)) ? (((float)v_rawGZ * _lsbToDps - _gyrOffsetZ) * _gyrGainZ) : 0.0f;
                v_gyrCount++;
            }
            v_idx += 6;
        } else {
            v_idx++;
        }
    }

    return (v_accCount > v_gyrCount) ? v_accCount : v_gyrCount;
}

/**
 * @brief 하드웨어 FIFO 버퍼 데이터를 내부 링 버퍼에 백그라운드 누적
 * @return 누적된 데이터 길이
 */
uint16_t CL_T2_SensorEngine::accumulateFifo() {
    if (!_isBmiInit || _isPaused) return 0;

    alignas(16) float v_tAccX[Imu::Hardware::FIFO_BATCH_SIZE_MAX];
    alignas(16) float v_tAccY[Imu::Hardware::FIFO_BATCH_SIZE_MAX];
    alignas(16) float v_tAccZ[Imu::Hardware::FIFO_BATCH_SIZE_MAX];
    alignas(16) float v_tGyrX[Imu::Hardware::FIFO_BATCH_SIZE_MAX];
    alignas(16) float v_tGyrY[Imu::Hardware::FIFO_BATCH_SIZE_MAX];
    alignas(16) float v_tGyrZ[Imu::Hardware::FIFO_BATCH_SIZE_MAX];

    uint16_t v_read = readVibFifoBatch(v_tAccX, v_tAccY, v_tAccZ, v_tGyrX, v_tGyrY, v_tGyrZ, Imu::Hardware::FIFO_BATCH_SIZE_MAX);
    if (v_read == 0) return 0;

    const uint16_t ACC_BUF_SIZE = Accel::Sensor::FFT_SIZE_MAX;
    for (uint16_t i = 0; i < v_read; i++) {
        _accumX[_accumWriteIdx] = v_tAccX[i];
        _accumY[_accumWriteIdx] = v_tAccY[i];
        _accumZ[_accumWriteIdx] = v_tAccZ[i];
        _accumWriteIdx = (_accumWriteIdx + 1) % ACC_BUF_SIZE;

        if (_accumCount < ACC_BUF_SIZE) {
            _accumCount++;
        } else {
            _accumReadIdx = (_accumReadIdx + 1) % ACC_BUF_SIZE;
        }
    }

    if (_gyroEnabled) {
        const uint16_t GYR_BUF_SIZE = Gyro::Sensor::FFT_SIZE_MAX;
        for (uint16_t i = 0; i < v_read; i++) {
            _accumGX[_accumGWriteIdx] = v_tGyrX[i];
            _accumGY[_accumGWriteIdx] = v_tGyrY[i];
            _accumGZ[_accumGWriteIdx] = v_tGyrZ[i];
            _accumGWriteIdx = (_accumGWriteIdx + 1) % GYR_BUF_SIZE;

            if (_accumGCount < GYR_BUF_SIZE) {
                _accumGCount++;
            } else {
                _accumGReadIdx = (_accumGReadIdx + 1) % GYR_BUF_SIZE;
            }
        }
    }

    return v_read;
}

/**
 * @brief 누적 보관 중인 가속도 센서 정보 획득
 */
uint16_t CL_T2_SensorEngine::getAccumulatedAccel(float* p_outX, float* p_outY, float* p_outZ, uint16_t p_reqCount) {
    if (_accumCount == 0 || p_reqCount == 0) return 0;

    uint16_t v_toCopy = (p_reqCount > _accumCount) ? _accumCount : p_reqCount;
    const uint16_t BUFFER_SIZE = Accel::Sensor::FFT_SIZE_MAX;

    for (uint16_t i = 0; i < v_toCopy; i++) {
        p_outX[i] = _accumX[_accumReadIdx];
        p_outY[i] = _accumY[_accumReadIdx];
        p_outZ[i] = _accumZ[_accumReadIdx];
        _accumReadIdx = (_accumReadIdx + 1) % BUFFER_SIZE;
        _accumCount--;
    }

    return v_toCopy;
}

/**
 * @brief 누적 보관 중인 자이로 센서 정보 획득
 */
uint16_t CL_T2_SensorEngine::getAccumulatedGyro(float* p_outX, float* p_outY, float* p_outZ, uint16_t p_reqCount) {
    if (_accumGCount == 0 || p_reqCount == 0) return 0;

    uint16_t v_toCopy = (p_reqCount > _accumGCount) ? _accumGCount : p_reqCount;
    const uint16_t BUFFER_SIZE = Gyro::Sensor::FFT_SIZE_MAX;

    for (uint16_t i = 0; i < v_toCopy; i++) {
        p_outX[i] = _accumGX[_accumGReadIdx];
        p_outY[i] = _accumGY[_accumGReadIdx];
        p_outZ[i] = _accumGZ[_accumGReadIdx];
        _accumGReadIdx = (_accumGReadIdx + 1) % BUFFER_SIZE;
        _accumGCount--;
    }

    return v_toCopy;
}

/**
 * @brief 가속도 센서 자동 하드웨어/소프트웨어 오프셋 캘리브레이션 튜닝 기동
 */
bool CL_T2_SensorEngine::runAccelCalibration() {
    if (!_isBmiInit) return false;
    ESP_LOGI(TAG, "Starting BMI270 Accel Calibration (HW Retrim + Soft Offset)...");

    _bmi.performComponentRetrim();
    _bmi.performAccelOffsetCalibration(BMI2_GRAVITY_POS_Z);

    float dAccX[100], dAccY[100], dAccZ[100];
    float dGyrX[100], dGyrY[100], dGyrZ[100];
    readVibFifoBatch(dAccX, dAccY, dAccZ, dGyrX, dGyrY, dGyrZ, 100);

    float v_sumX = 0, v_sumY = 0, v_sumZ = 0;
    uint16_t v_samples = 0;
    const uint16_t TARGET_SAMPLES = Accel::Calib::TARGET_SAMPLES_DEF;

    for (int i = 0; i < Accel::Calib::RETRY_MAX_CONST && v_samples < TARGET_SAMPLES; i++) {
        vTaskDelay(pdMS_TO_TICKS(Accel::Calib::LOOP_DELAY_MS_CONST));
        uint16_t v_count = readVibFifoBatch(dAccX, dAccY, dAccZ, dGyrX, dGyrY, dGyrZ, TARGET_SAMPLES - v_samples);
        for (uint16_t j = 0; j < v_count; j++) {
            v_sumX += (dAccX[j] / _accGainX) + _accOffsetX;
            v_sumY += (dAccY[j] / _accGainY) + _accOffsetY;
            v_sumZ += (dAccZ[j] / _accGainZ) + _accOffsetZ;
            v_samples++;
        }
    }

    if (v_samples < TARGET_SAMPLES / 2) return false;

    _accOffsetX = v_sumX / v_samples;
    _accOffsetY = v_sumY / v_samples;
    _accOffsetZ = (v_sumZ / v_samples) - 1.0f;
    _accGainX = 1.0f; _accGainY = 1.0f; _accGainZ = 1.0f;

    JsonDocument v_doc;
    JsonArray v_offArr = v_doc["offset"].to<JsonArray>();
    v_offArr.add(_accOffsetX); v_offArr.add(_accOffsetY); v_offArr.add(_accOffsetZ);
    JsonArray v_gainArr = v_doc["gain"].to<JsonArray>();
    v_gainArr.add(_accGainX); v_gainArr.add(_accGainY); v_gainArr.add(_accGainZ);

    File v_file = LittleFS.open(Accel::Calib::FILE_JSON_CONST, "w");
    if (v_file) {
        serializeJson(v_doc, v_file);
        v_file.close();
        ESP_LOGI(TAG, "Accel Calibration Saved: Off(%.3f, %.3f, %.3f)", _accOffsetX, _accOffsetY, _accOffsetZ);
        return true;
    }
    return false;
}

/**
 * @brief 자이로 센서 오프셋 캘리브레이션 튜닝 기동
 */
bool CL_T2_SensorEngine::runGyroCalibration() {
    if (!_isBmiInit || !_gyroEnabled) return false;
    ESP_LOGI(TAG, "Starting BMI270 Gyro Calibration...");

    _bmi.performGyroOffsetCalibration();

    float dAccX[200], dAccY[200], dAccZ[200];
    float dGyrX[200], dGyrY[200], dGyrZ[200];
    readVibFifoBatch(dAccX, dAccY, dAccZ, dGyrX, dGyrY, dGyrZ, 200);

    float v_sumX = 0, v_sumY = 0, v_sumZ = 0;
    uint16_t v_samples = 0;
    const uint16_t TARGET_SAMPLES = Gyro::Calib::TARGET_SAMPLES_DEF;

    for (int i = 0; i < Gyro::Calib::RETRY_MAX_CONST && v_samples < TARGET_SAMPLES; i++) {
        vTaskDelay(pdMS_TO_TICKS(Gyro::Calib::LOOP_DELAY_MS_CONST));
        uint16_t v_count = readVibFifoBatch(dAccX, dAccY, dAccZ, dGyrX, dGyrY, dGyrZ, TARGET_SAMPLES - v_samples);
        for (uint16_t j = 0; j < v_count; j++) {
            v_sumX += (dGyrX[j] / _gyrGainX) + _gyrOffsetX;
            v_sumY += (dGyrY[j] / _gyrGainY) + _gyrOffsetY;
            v_sumZ += (dGyrZ[j] / _gyrGainZ) + _gyrOffsetZ;
            v_samples++;
        }
    }

    if (v_samples < TARGET_SAMPLES / 2) return false;

    _gyrOffsetX = v_sumX / v_samples;
    _gyrOffsetY = v_sumY / v_samples;
    _gyrOffsetZ = v_sumZ / v_samples;
    _gyrGainX = 1.0f; _gyrGainY = 1.0f; _gyrGainZ = 1.0f;

    JsonDocument v_doc;
    JsonArray v_offArr = v_doc["offset"].to<JsonArray>();
    v_offArr.add(_gyrOffsetX); v_offArr.add(_gyrOffsetY); v_offArr.add(_gyrOffsetZ);
    JsonArray v_gainArr = v_doc["gain"].to<JsonArray>();
    v_gainArr.add(_gyrGainX); v_gainArr.add(_gyrGainY); v_gainArr.add(_gyrGainZ);

    File v_file = LittleFS.open(Gyro::Calib::FILE_JSON_CONST, "w");
    if (v_file) {
        serializeJson(v_doc, v_file);
        v_file.close();
        ESP_LOGI(TAG, "Gyro Calibration Saved: Off(%.3f, %.3f, %.3f)", _gyrOffsetX, _gyrOffsetY, _gyrOffsetZ);
        return true;
    }
    return false;
}

/**
 * @brief 저장된 가속도 캘리브레이션 튜닝 파일 파싱 적용
 */
bool CL_T2_SensorEngine::applyStoredAccelCalibration() {
    if (!LittleFS.exists(Accel::Calib::FILE_JSON_CONST)) {
        _accOffsetX = 0; _accOffsetY = 0; _accOffsetZ = 0;
        _accGainX = 1.0f; _accGainY = 1.0f; _accGainZ = 1.0f;
        return false;
    }
    File v_file = LittleFS.open(Accel::Calib::FILE_JSON_CONST, "r");
    if (!v_file) return false;

    JsonDocument v_doc;
    DeserializationError v_err = deserializeJson(v_doc, v_file);
    v_file.close();

    if (!v_err) {
        JsonArrayConst v_off = v_doc["offset"];
        if (v_off.size() >= 3) {
            _accOffsetX = v_off[0] | 0.0f;
            _accOffsetY = v_off[1] | 0.0f;
            _accOffsetZ = v_off[2] | 0.0f;
        }
        JsonArrayConst v_gain = v_doc["gain"];
        if (v_gain.size() >= 3) {
            _accGainX = v_gain[0] | 1.0f;
            _accGainY = v_gain[1] | 1.0f;
            _accGainZ = v_gain[2] | 1.0f;
        }
        return true;
    }
    return false;
}

/**
 * @brief 저장된 자이로 캘리브레이션 튜닝 파일 파싱 적용
 */
bool CL_T2_SensorEngine::applyStoredGyroCalibration() {
    if (!LittleFS.exists(Gyro::Calib::FILE_JSON_CONST)) {
        _gyrOffsetX = 0; _gyrOffsetY = 0; _gyrOffsetZ = 0;
        _gyrGainX = 1.0f; _gyrGainY = 1.0f; _gyrGainZ = 1.0f;
        return false;
    }
    File v_file = LittleFS.open(Gyro::Calib::FILE_JSON_CONST, "r");
    if (!v_file) return false;

    JsonDocument v_doc;
    DeserializationError v_err = deserializeJson(v_doc, v_file);
    v_file.close();

    if (!v_err) {
        JsonArrayConst v_off = v_doc["offset"];
        if (v_off.size() >= 3) {
            _gyrOffsetX = v_off[0] | 0.0f;
            _gyrOffsetY = v_off[1] | 0.0f;
            _gyrOffsetZ = v_off[2] | 0.0f;
        }
        JsonArrayConst v_gain = v_doc["gain"];
        if (v_gain.size() >= 3) {
            _gyrGainX = v_gain[0] | 1.0f;
            _gyrGainY = v_gain[1] | 1.0f;
            _gyrGainZ = v_gain[2] | 1.0f;
        }
        return true;
    }
    return false;
}

/**
 * @brief WOM(Wake On Motion) 하드웨어 인터럽트 활성화 및 GPIO 레벨 맵 설정
 */
bool CL_T2_SensorEngine::enableWakeOnMotion(float p_threshG, uint16_t p_duration) {
    if (!_isBmiInit) return false;

    bmi2_sens_config v_config;
    v_config.type = BMI2_ANY_MOTION;
    v_config.cfg.any_motion.duration = p_duration;

    float v_lsbMg = Imu::Hardware::ANY_MOTION_LSB_2G_MG * ((float)_accelRange / 2.0f);
    v_config.cfg.any_motion.threshold = (uint16_t)(p_threshG * 1000.0f / v_lsbMg);

    v_config.cfg.any_motion.select_x = 1;
    v_config.cfg.any_motion.select_y = 1;
    v_config.cfg.any_motion.select_z = 1;

    _bmi.setConfig(v_config);
    _bmi.mapInterruptToPin(BMI2_ANY_MOTION_INT, BMI2_INT1);
    return true;
}

/**
 * @brief 센서 가속도 범위 매핑 변환
 */
uint8_t CL_T2_SensorEngine::_mapAccelRange(uint8_t p_rangeG) {
    switch (p_rangeG) {
        case 2:  return BMI2_ACC_RANGE_2G;
        case 4:  return BMI2_ACC_RANGE_4G;
        case 8:  return BMI2_ACC_RANGE_8G;
        case 16: return BMI2_ACC_RANGE_16G;
        default: return BMI2_ACC_RANGE_8G;
    }
}

/**
 * @brief 센서 자이로 범위 매핑 변환
 */
uint8_t CL_T2_SensorEngine::_mapGyroRange(uint16_t p_rangeDps) {
    if (p_rangeDps <= 125) return BMI2_GYR_RANGE_125;
    if (p_rangeDps <= 250) return BMI2_GYR_RANGE_250;
    if (p_rangeDps <= 500) return BMI2_GYR_RANGE_500;
    if (p_rangeDps <= 1000) return BMI2_GYR_RANGE_1000;
    return BMI2_GYR_RANGE_2000;
}

/**
 * @brief SPI 통신을 통한 센서 레지스터 데이터 버스트 리드 (DMA 연계)
 */
bool CL_T2_SensorEngine::_readRegs(uint8_t p_reg, uint8_t* p_data, uint16_t p_len) {
    _spi.beginTransaction(SPISettings(Imu::Hardware::SPI_FREQ_HZ_CONST, MSBFIRST, SPI_MODE0));
    digitalWrite(Imu::Hardware::PIN_CS_CONST, LOW);
    _spi.transfer(p_reg | 0x80);
    _spi.transfer(0x00);
    _spi.transferBytes(nullptr, p_data, p_len);
    digitalWrite(Imu::Hardware::PIN_CS_CONST, HIGH);
    _spi.endTransaction();
    return true;
}

/**
 * @brief SPI 통신을 통한 센서 레지스터 데이터 라이트
 */
bool CL_T2_SensorEngine::_writeRegs(uint8_t p_reg, const uint8_t* p_data, uint16_t p_len) {
    _spi.beginTransaction(SPISettings(Imu::Hardware::SPI_FREQ_HZ_CONST, MSBFIRST, SPI_MODE0));
    digitalWrite(Imu::Hardware::PIN_CS_CONST, LOW);
    _spi.transfer(p_reg & 0x7F);
    for (uint16_t i = 0; i < p_len; i++) _spi.transfer(p_data[i]);
    digitalWrite(Imu::Hardware::PIN_CS_CONST, HIGH);
    _spi.endTransaction();
    return true;
}

/**
 * @brief 센서 다이 내부 온도 센서 판독
 */
float CL_T2_SensorEngine::readTemperatureSensor() {
    if (!_isBmiInit) return 0.0f;
    uint8_t v_tempBuf[2] = {0};
    if (!_readRegs(Imu::Hardware::REG_TEMP_MSB_CONST, v_tempBuf, 2)) return 0.0f;
    int16_t v_rawTemp = (int16_t)((v_tempBuf[1] << 8) | v_tempBuf[0]);
    if (v_rawTemp == Imu::Hardware::TEMP_INVALID_CONST) return 0.0f;
    float v_tempC = (float)v_rawTemp / Imu::Hardware::TEMP_SCALE_CONST + Imu::Hardware::TEMP_OFFSET_CONST;
    return v_tempC;
}
