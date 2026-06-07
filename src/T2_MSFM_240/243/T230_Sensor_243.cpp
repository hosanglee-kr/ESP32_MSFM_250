/* ============================================================================
 * File: T230_Sensor_243.cpp
 * Summary: BMI270 (SPI) & ICS43434 (I2S) 멀티모달 센서 융합 수집 엔진 구현부 (v243)
 * ============================================================================
 * [로직 상태 추적 (Logic State Tracker)]
 * - 유지: BMI270 SPI 10MHz 통신 및 레지스터 직접 제어를 통한 FIFO 오버헤드 최소화.
 * - 갱신: T240_v243 4-Tier 정의(Def) 및 타입(Type) 시스템 완벽 통합.
 * - 신규: [자가 교정] Z축 중력 보정 및 3축 Gain 팩터 적용 로직 추가.
 *
 * [축소/누락 방어 체크리스트 (Omission Defense)]
 * 1. [RTOS 원칙]: vTaskDelay()를 통한 컨텍스트 스위칭 양보로 데드락 방지 (Rule #3).
 * 2. [수학적 무결성]: Gain 보정 시 (Raw - Offset) * Gain 수식 적용으로 정밀도 확보.
 * 3. [예외 처리]: I2S 드라이버 설치 실패 시 상태 텍스트 업데이트 및 로그 출력.
 * ========================================================================== */

#include "T230_Sensor_243.hpp"
#include "esp_log.h"
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "T243_SENS";

CL_T2_SensorEngine::CL_T2_SensorEngine(SPIClass& p_spiBus)
    : _spi(p_spiBus), _bmi(), _isI2sInit(false), _isBmiInit(false), _isPaused(false),
      _offsetX(0.0f), _offsetY(0.0f), _offsetZ(0.0f),
      _gainX(1.0f), _gainY(1.0f), _gainZ(1.0f),
      _lsbToG(1.0f / 4096.0f), _accelRange(8), _accelAxisCount(3), _accelAxis(2), _gyroAxisCount(3), _gyroAxis(2),
      _gyroEnabled(false), _audioChannelMode(T2_Type::AudioChannelMode::STEREO) {
    memset(_dmaBuffer, 0, sizeof(_dmaBuffer));
    strlcpy(_statusText, "INIT", sizeof(_statusText));
}

bool CL_T2_SensorEngine::init(const T2_Type::ST_Global_System& p_sysCfg, const T2_Type::ST_Vib_Sensor& p_vibCfg, const T2_Type::ST_Audio_Sensor& p_audCfg) {
    bool v_success = true;

    // [1] BMI270 Init (Vib Domain)
    if (p_sysCfg.vib_enable && !_isBmiInit) {
        int8_t v_rslt = _bmi.beginSPI(T2_Def::Vib::Hardware::PIN_BMI_CS_CONST, T2_Def::Vib::Hardware::SPI_FREQ_HZ_CONST, _spi);
        if (v_rslt != BMI2_OK) {
            ESP_LOGE(TAG, "BMI270 SPI Init Failed (Code: %d)", v_rslt);
            strlcpy(_statusText, "BMI_ERR", sizeof(_statusText));
            v_success = false;
            } else {
            // [v243] Range, 축 개수 및 분석 타겟 설정
            _accelRange  = p_vibCfg.accel_range;
            _accelAxisCount = p_vibCfg.accel_axis_count;
            _accelAxis   = p_vibCfg.accel_axis;
            _gyroAxisCount = p_vibCfg.gyro_axis_count;
            _gyroAxis    = p_vibCfg.gyro_axis;
            _gyroEnabled = p_vibCfg.gyro_enable;
            _lsbToG      = (float)_accelRange / 32768.0f;

            // [1.1] Accel Config (p_vibCfg.accel_enable 반영)
            if (p_vibCfg.accel_enable) {
                bmi2_sens_config v_accelConfig;
                v_accelConfig.type                = BMI2_ACCEL;
                v_accelConfig.cfg.acc.odr         = p_vibCfg.accel_odr;
                v_accelConfig.cfg.acc.bwp         = BMI2_ACC_NORMAL_AVG4;
                v_accelConfig.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
                v_accelConfig.cfg.acc.range       = _mapAccelRange(p_vibCfg.accel_range);
                _bmi.setConfig(v_accelConfig);
            }

            // [1.2] Gyro Config (p_vibCfg.gyro_enable 반영)
            if (_gyroEnabled) {
                bmi2_sens_config v_gyroConfig;
                v_gyroConfig.type               = BMI2_GYRO;
                v_gyroConfig.cfg.gyr.odr        = BMI2_GYR_ODR_1600HZ;
                v_gyroConfig.cfg.gyr.bwp        = BMI2_GYR_NORMAL_MODE;
                v_gyroConfig.cfg.gyr.filter_perf= BMI2_PERF_OPT_MODE;
                v_gyroConfig.cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
                v_gyroConfig.cfg.gyr.range      = _mapGyroRange(p_vibCfg.gyro_range);
                _bmi.setConfig(v_gyroConfig);
            }

            // [1.3] FIFO 활성화 (Enable된 센서만 FIFO에 할당)
            BMI270_FIFOConfig v_fifoConfig;
            v_fifoConfig.flags = 0;
            if (p_vibCfg.accel_enable) v_fifoConfig.flags |= BMI2_FIFO_ACC_EN;
            if (_gyroEnabled)          v_fifoConfig.flags |= BMI2_FIFO_GYR_EN;

            v_fifoConfig.watermark   = T2_Def::Vib::Hardware::FIFO_BATCH_SIZE_DEF;
            v_fifoConfig.accelFilter = BMI2_ENABLE;
            v_fifoConfig.gyroFilter  = BMI2_ENABLE;
            _bmi.setFIFOConfig(v_fifoConfig);

            _isBmiInit = true;
            applyStoredCalibration();
            ESP_LOGI(TAG, "Vib Domain Initialized (Acc:%d, Gyr:%d)", p_vibCfg.accel_enable, _gyroEnabled);
        }
    }

    // [2] ICS43434 Init (Audio Domain)
    if (p_sysCfg.audio_enable && !_isI2sInit) {
        i2s_config_t v_i2sConfig = {
            .mode				  = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
            .sample_rate		  = p_audCfg.sample_rate,
            .bits_per_sample	  = I2S_BITS_PER_SAMPLE_32BIT,
            .channel_format		  = I2S_CHANNEL_FMT_RIGHT_LEFT,
            .communication_format = I2S_COMM_FORMAT_STAND_I2S,
            .intr_alloc_flags	  = ESP_INTR_FLAG_LEVEL3,
            .dma_buf_count		  = T2_Def::Audio::Hardware::DMA_BUF_COUNT_DEF,
            .dma_buf_len		  = (int)p_audCfg.fft_size,
            .use_apll			  = true,
            .tx_desc_auto_clear	  = false,
            .fixed_mclk			  = 0};

        i2s_pin_config_t v_pinConfig = {
            .bck_io_num = T2_Def::Audio::Hardware::PIN_I2S_BCLK_CONST,
            .ws_io_num = T2_Def::Audio::Hardware::PIN_I2S_WS_CONST,
            .data_out_num = I2S_PIN_NO_CHANGE,
            .data_in_num = T2_Def::Audio::Hardware::PIN_I2S_DIN_CONST
        };

        if (i2s_driver_install((i2s_port_t)T2_Def::Audio::Hardware::I2S_PORT_NUM_CONST, &v_i2sConfig, 0, NULL) != ESP_OK) {
            ESP_LOGE(TAG, "I2S driver install failed");
            strlcpy(_statusText, "I2S_ERR", sizeof(_statusText));
            v_success = false;
        } else {
            if (i2s_set_pin((i2s_port_t)T2_Def::Audio::Hardware::I2S_PORT_NUM_CONST, &v_pinConfig) != ESP_OK) {
                ESP_LOGE(TAG, "I2S pin config failed");
                strlcpy(_statusText, "I2S_PIN_ERR", sizeof(_statusText));
                v_success = false;
            } else {
                _isI2sInit = true;
                _audioChannelMode = p_audCfg.channel_mode;
                ESP_LOGI(TAG, "Audio Domain Initialized (Rate:%d, Mode:%d)", p_audCfg.sample_rate, (int)_audioChannelMode);
            }
        }
    }

    if (v_success) {
        _isPaused = false;
        strlcpy(_statusText, "RUNNING", sizeof(_statusText));
    }

    return v_success;
}

void CL_T2_SensorEngine::pause() {
    if (_isPaused) return;
    if (_isI2sInit) i2s_stop((i2s_port_t)T2_Def::Audio::Hardware::I2S_PORT_NUM_CONST);
    if (_isBmiInit) {
        uint8_t v_regData = 0x00; // Power Off
        _writeRegs(BMI2_PWR_CTRL_ADDR, &v_regData, 1);
    }
    _isPaused = true;
    strlcpy(_statusText, "PAUSED", sizeof(_statusText));
}

void CL_T2_SensorEngine::resume() {
    if (!_isPaused) return;
    if (_isBmiInit) {
        uint8_t v_regData = 0x04; // Accel Enable
        _writeRegs(BMI2_PWR_CTRL_ADDR, &v_regData, 1);
        vTaskDelay(pdMS_TO_TICKS(T2_Def::Vib::Hardware::STARTUP_DELAY_MS_CONST));
    }
    if (_isI2sInit) {
        clearAudioBuffer();
        i2s_start((i2s_port_t)T2_Def::Audio::Hardware::I2S_PORT_NUM_CONST);
    }
    _isPaused = false;
    strlcpy(_statusText, "RUNNING", sizeof(_statusText));
}

void CL_T2_SensorEngine::resetHardware() {
    if (_isBmiInit) _bmi.reset();
    _isBmiInit = false;
    _isI2sInit = false;
    strlcpy(_statusText, "RESET", sizeof(_statusText));
}

void CL_T2_SensorEngine::clearAudioBuffer() {
    if (!_isI2sInit) return;
    i2s_zero_dma_buffer((i2s_port_t)T2_Def::Audio::Hardware::I2S_PORT_NUM_CONST);
}

uint32_t CL_T2_SensorEngine::readAudioChunk(float* p_outL, float* p_outR, uint32_t p_reqSamples) {
    if (!_isI2sInit || _isPaused) return 0;

    // 버퍼 크기 클램핑
    uint32_t v_samplesToRead = (p_reqSamples > T2_Def::Audio::Sensor::FFT_SIZE_MAX) ?
                               T2_Def::Audio::Sensor::FFT_SIZE_MAX : p_reqSamples;
    size_t v_bytesRead = 0;
    size_t v_bytesToRead = v_samplesToRead * 2 * sizeof(int32_t);

    esp_err_t v_res = i2s_read((i2s_port_t)T2_Def::Audio::Hardware::I2S_PORT_NUM_CONST, _dmaBuffer, v_bytesToRead, &v_bytesRead, portMAX_DELAY);

    if (v_res != ESP_OK) return 0;

    uint32_t v_samplesRead = v_bytesRead / (2 * sizeof(int32_t));

    // [v243] 채널 모드에 따른 De-interleaving 최적화
    for (uint32_t i = 0; i < v_samplesRead; i++) {
        float v_valL = (float)_dmaBuffer[i * 2] * T2_Def::Audio::Hardware::PCM_32BIT_SCALE_CONST;
        float v_valR = (float)_dmaBuffer[i * 2 + 1] * T2_Def::Audio::Hardware::PCM_32BIT_SCALE_CONST;

        if (_audioChannelMode == T2_Type::AudioChannelMode::STEREO) {
            p_outL[i] = v_valL;
            p_outR[i] = v_valR;
        } else if (_audioChannelMode == T2_Type::AudioChannelMode::MONO_L) {
            p_outL[i] = v_valL;
            p_outR[i] = 0.0f; // Mono L 모드 시 R은 0 처리 (또는 p_outR 무시)
        } else if (_audioChannelMode == T2_Type::AudioChannelMode::MONO_R) {
            p_outL[i] = v_valR; // Mono R 모드 시 출력의 L에 R 데이터를 채움 (분석 엔진 호환성)
            p_outR[i] = 0.0f;
        }
    }

    return v_samplesRead;
}

uint16_t CL_T2_SensorEngine::readVibFifoBatch(float* p_outX, float* p_outY, float* p_outZ, uint16_t p_maxFrames) {
    if (!_isBmiInit || _isPaused) return 0;

    uint8_t v_lenBuf[2] = {0};
    if (!_readRegs(T2_Def::Vib::Hardware::REG_FIFO_LEN_ADDR_CONST, v_lenBuf, 2)) return 0; // FIFO Length Register

    uint16_t v_fifoBytes = (v_lenBuf[1] << 8) | v_lenBuf[0];
    uint16_t v_availFrames = v_fifoBytes / T2_Def::Vib::Hardware::FIFO_FRAME_SIZE_CONST; // Header(1) + Data(6) = 7 Bytes per frame

    if (v_availFrames == 0) return 0;

    uint16_t v_framesToRead = (v_availFrames > p_maxFrames) ? p_maxFrames : v_availFrames;
    uint16_t v_bytesToRead = v_framesToRead * T2_Def::Vib::Hardware::FIFO_FRAME_SIZE_CONST;

    uint8_t v_fifoData[T2_Def::Vib::Hardware::FIFO_BATCH_SIZE_MAX * T2_Def::Vib::Hardware::FIFO_FRAME_SIZE_CONST]; // Scratch buffer
    if (v_bytesToRead > sizeof(v_fifoData)) v_bytesToRead = sizeof(v_fifoData);

    if (!_readRegs(T2_Def::Vib::Hardware::REG_FIFO_DATA_ADDR_CONST, v_fifoData, v_bytesToRead)) return 0; // FIFO Data Register

    uint16_t v_parsedCount = 0;
    uint16_t v_idx = 0;
    // const float v_lsbToG = 1.0f / 4096.0f; // 8G range assumed for LSB calc in 16bit, but BMI library handles it usually.
                                           // Manual SPI access needs manual conversion. 4096 is for 8G in BMI270 (16-bit).

    while (v_idx + (T2_Def::Vib::Hardware::FIFO_FRAME_SIZE_CONST - 1) < v_bytesToRead && v_parsedCount < v_framesToRead) {
        uint8_t v_header = v_fifoData[v_idx];

        if (v_header == T2_Def::Vib::Hardware::FIFO_HEADER_ACCEL_CONST) { // Accel frame header
            v_idx++;
            int16_t v_rawX = (int16_t)((v_fifoData[v_idx + 1] << 8) | v_fifoData[v_idx]);
            int16_t v_rawY = (int16_t)((v_fifoData[v_idx + 3] << 8) | v_fifoData[v_idx + 2]);
            int16_t v_rawZ = (int16_t)((v_fifoData[v_idx + 5] << 8) | v_fifoData[v_idx + 4]);

            // [v243] 축 선택 및 캘리브레이션 반영 로직
            if (_accelAxisCount >= 3) {
                // 3축 모드: X, Y, Z 버퍼 모두 채움
                p_outX[v_parsedCount] = (((float)v_rawX * _lsbToG) - _offsetX) * _gainX;
                p_outY[v_parsedCount] = (((float)v_rawY * _lsbToG) - _offsetY) * _gainY;
                p_outZ[v_parsedCount] = (((float)v_rawZ * _lsbToG) - _offsetZ) * _gainZ;
            } else {
                // 1축 모드: _accelAxis에 해당하는 데이터만 p_outX에 채움
                float v_val = 0;
                switch (_accelAxis) {
                    case 0: v_val = ((float)v_rawX * _lsbToG - _offsetX) * _gainX; break;
                    case 1: v_val = ((float)v_rawY * _lsbToG - _offsetY) * _gainY; break;
                    case 2: v_val = ((float)v_rawZ * _lsbToG - _offsetZ) * _gainZ; break;
                    default: v_val = ((float)v_rawZ * _lsbToG - _offsetZ) * _gainZ; break;
                }
                p_outX[v_parsedCount] = v_val;
            }

            v_parsedCount++;
            v_idx += 6;
        } else if (v_header == T2_Def::Vib::Hardware::FIFO_HEADER_GYRO_CONST) { // Gyro frame header
            v_idx++;
            if (_gyroEnabled) {
                // 자이로 데이터 파싱
                int16_t v_rawGX = (int16_t)((v_fifoData[v_idx + 1] << 8) | v_fifoData[v_idx]);
                int16_t v_rawGY = (int16_t)((v_fifoData[v_idx + 3] << 8) | v_fifoData[v_idx + 2]);
                int16_t v_rawGZ = (int16_t)((v_fifoData[v_idx + 5] << 8) | v_fifoData[v_idx + 4]);

                if (_gyroAxisCount >= 3) {
                    p_outX[v_parsedCount] = (float)v_rawGX;
                    p_outY[v_parsedCount] = (float)v_rawGY;
                    p_outZ[v_parsedCount] = (float)v_rawGZ;
                } else {
                    float v_gVal = 0;
                    switch (_gyroAxis) {
                        case 0: case 3: v_gVal = (float)v_rawGX; break;
                        case 1: case 4: v_gVal = (float)v_rawGY; break;
                        case 2: case 5: v_gVal = (float)v_rawGZ; break;
                        default: v_gVal = (float)v_rawGZ; break;
                    }
                    p_outX[v_parsedCount] = v_gVal;
                }
                v_parsedCount++;
            }
            v_idx += 6;
        } else {
            v_idx++; // Sync-Hunting
        }
    }

    return v_parsedCount;
}

bool CL_T2_SensorEngine::runCalibration() {
    if (!_isBmiInit) return false;
    ESP_LOGI(TAG, "Starting BMI270 Calibration (HW Retrim + Soft Offset)...");

    // [v243] 하드웨어 레벨 보정 엔진 가동 (v234 기능 복원)
    _bmi.performComponentRetrim();
    _bmi.performAccelOffsetCalibration(BMI2_GRAVITY_POS_Z);
    if (_gyroEnabled) _bmi.performGyroOffsetCalibration();

    // 잔류 데이터 비우기
    float dummyX[T2_Def::Vib::Calib::TARGET_SAMPLES_DEF], dummyY[T2_Def::Vib::Calib::TARGET_SAMPLES_DEF], dummyZ[T2_Def::Vib::Calib::TARGET_SAMPLES_DEF];
    readVibFifoBatch(dummyX, dummyY, dummyZ, T2_Def::Vib::Calib::TARGET_SAMPLES_DEF);

    float v_sumX = 0, v_sumY = 0, v_sumZ = 0;
    uint16_t v_samples = 0;
    const uint16_t TARGET_SAMPLES = T2_Def::Vib::Calib::TARGET_SAMPLES_DEF;

    for (int i = 0; i < T2_Def::Vib::Calib::RETRY_MAX_CONST && v_samples < TARGET_SAMPLES; i++) {
        vTaskDelay(pdMS_TO_TICKS(T2_Def::Vib::Calib::LOOP_DELAY_MS_CONST));
        uint16_t v_count = readVibFifoBatch(dummyX, dummyY, dummyZ, TARGET_SAMPLES - v_samples);
        for (uint16_t j = 0; j < v_count; j++) {
            // 현재 적용된 오프셋/게인을 역산하여 원시(G) 데이터 합산
            v_sumX += (dummyX[j] / _gainX) + _offsetX;
            v_sumY += (dummyY[j] / _gainY) + _offsetY;
            v_sumZ += (dummyZ[j] / _gainZ) + _offsetZ;
            v_samples++;
        }
    }

    if (v_samples < TARGET_SAMPLES / 2) {
        ESP_LOGE(TAG, "Calibration failed: Not enough samples (%d)", v_samples);
        return false;
    }

    _offsetX = v_sumX / v_samples;
    _offsetY = v_sumY / v_samples;
    _offsetZ = (v_sumZ / v_samples) - 1.0f; // Z축 중력가속도(1G) 차감

    _gainX = 1.0f; _gainY = 1.0f; _gainZ = 1.0f; // 오프셋 교정 시 게인은 1.0으로 초기화 (또는 유지)

    // v243 전용 보정 파일 저장
    JsonDocument v_doc;
    JsonArray v_offArr = v_doc["offset"].to<JsonArray>();
    v_offArr.add(_offsetX); v_offArr.add(_offsetY); v_offArr.add(_offsetZ);

    JsonArray v_gainArr = v_doc["gain"].to<JsonArray>();
    v_gainArr.add(_gainX); v_gainArr.add(_gainY); v_gainArr.add(_gainZ);

    File v_file = LittleFS.open(T2_Def::Vib::Calib::FILE_JSON_CONST, "w");
    if (v_file) {
        serializeJson(v_doc, v_file);
        v_file.close();
        ESP_LOGI(TAG, "Calibration Saved: Off(%.3f, %.3f, %.3f), Gain(1.0, 1.0, 1.0)", _offsetX, _offsetY, _offsetZ);
        return true;
    }
    return false;
}

bool CL_T2_SensorEngine::applyStoredCalibration() {
    if (!LittleFS.exists(T2_Def::Vib::Calib::FILE_JSON_CONST)) {
        ESP_LOGW(TAG, "No calibration file found. Using defaults.");
        _offsetX = 0; _offsetY = 0; _offsetZ = 0;
        _gainX = 1.0f; _gainY = 1.0f; _gainZ = 1.0f;
        return false;
    }

    File v_file = LittleFS.open(T2_Def::Vib::Calib::FILE_JSON_CONST, "r");
    if (!v_file) return false;

    JsonDocument v_doc;
    DeserializationError v_err = deserializeJson(v_doc, v_file);
    v_file.close();

    if (!v_err) {
        JsonArrayConst v_off = v_doc["offset"];
        if (v_off.size() >= 3) {
            _offsetX = v_off[0] | 0.0f;
            _offsetY = v_off[1] | 0.0f;
            _offsetZ = v_off[2] | 0.0f;
        }
        JsonArrayConst v_gain = v_doc["gain"];
        if (v_gain.size() >= 3) {
            _gainX = v_gain[0] | 1.0f;
            _gainY = v_gain[1] | 1.0f;
            _gainZ = v_gain[2] | 1.0f;
        }
        ESP_LOGI(TAG, "Calibration Loaded: Off(%.3f, %.3f, %.3f), Gain(%.3f, %.3f, %.3f)",
                 _offsetX, _offsetY, _offsetZ, _gainX, _gainY, _gainZ);
        return true;
    }
    return false;
}

bool CL_T2_SensorEngine::enableWakeOnMotion(float p_threshG, uint16_t p_duration) {
    if (!_isBmiInit) return false;

    bmi2_sens_config v_config;
    v_config.type = BMI2_ANY_MOTION;
    v_config.cfg.any_motion.duration = p_duration;

    // [v243] Any-motion 임계치 LSB 스케일링 (2G 기준 0.48mg이며 Range에 비례함)
    float v_lsbMg = T2_Def::Vib::Hardware::ANY_MOTION_LSB_2G_MG * ((float)_accelRange / 2.0f);
    v_config.cfg.any_motion.threshold = (uint16_t)(p_threshG * 1000.0f / v_lsbMg);

    v_config.cfg.any_motion.select_x = 1;
    v_config.cfg.any_motion.select_y = 1;
    v_config.cfg.any_motion.select_z = 1;

    _bmi.setConfig(v_config);
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

uint8_t CL_T2_SensorEngine::_mapGyroRange(uint8_t p_rangeDps) {
    if (p_rangeDps <= 125) return BMI2_GYR_RANGE_125;
    if (p_rangeDps <= 250) return BMI2_GYR_RANGE_250;
    if (p_rangeDps <= 500) return BMI2_GYR_RANGE_500;
    if (p_rangeDps <= 1000) return BMI2_GYR_RANGE_1000;
    return BMI2_GYR_RANGE_2000;
}

bool CL_T2_SensorEngine::_readRegs(uint8_t p_reg, uint8_t* p_data, uint16_t p_len) {
    _spi.beginTransaction(SPISettings(T2_Def::Vib::Hardware::SPI_FREQ_HZ_CONST, MSBFIRST, SPI_MODE0));
    digitalWrite(T2_Def::Vib::Hardware::PIN_BMI_CS_CONST, LOW);
    _spi.transfer(p_reg | 0x80); // Read bit
    _spi.transfer(0x00); // Dummy byte for BMI270 SPI read
    for (uint16_t i = 0; i < p_len; i++) p_data[i] = _spi.transfer(0x00);
    digitalWrite(T2_Def::Vib::Hardware::PIN_BMI_CS_CONST, HIGH);
    _spi.endTransaction();
    return true;
}

bool CL_T2_SensorEngine::_writeRegs(uint8_t p_reg, const uint8_t* p_data, uint16_t p_len) {
    _spi.beginTransaction(SPISettings(T2_Def::Vib::Hardware::SPI_FREQ_HZ_CONST, MSBFIRST, SPI_MODE0));
    digitalWrite(T2_Def::Vib::Hardware::PIN_BMI_CS_CONST, LOW);
    _spi.transfer(p_reg & 0x7F); // Write bit
    for (uint16_t i = 0; i < p_len; i++) _spi.transfer(p_data[i]);
    digitalWrite(T2_Def::Vib::Hardware::PIN_BMI_CS_CONST, HIGH);
    _spi.endTransaction();
    return true;
}
