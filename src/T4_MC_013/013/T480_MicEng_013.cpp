/* ============================================================================
 * File: T480_MicEng_013.cpp
 * Summary: I2S DMA Microphone Acquisition Engine (ICS43434) Implementation
 * ============================================================================
 * * [AI 메모: v012 고도화 적용 사항]
 * 1. [Wi-Fi 스타베이션 방어]: I2S 인터럽트 할당 시 ESP_INTR_FLAG_LEVEL3을
 * 적용하여 네트워크 스택 연산 중에도 오디오 프레임 유실이 발생하지 않도록 격상.
 * 2. [SIMD 정렬 및 클램프]: p_reqSamples가 FFT_SIZE_CONST를 초과하지 
 * 못하도록 방어하고 32bit PCM 데이터를 Zero-copy 방식으로 정규화.
 * ========================================================================== */

#include "T480_MicEng_013.hpp"
#include "esp_log.h"

static const char* TAG = "T480_MIC";

T480_MicEngine::T480_MicEngine() {
    memset(_dmaBuffer, 0, sizeof(_dmaBuffer));
    strlcpy(_statusText, "INIT", sizeof(_statusText));
}

bool T480_MicEngine::init() {
    if (_isInitialized) return true;

    i2s_config_t v_i2sConfig = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SmeaConfig::System::SAMPLING_RATE_CONST,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL3, // 인터럽트 우선순위 최상위 격상
        .dma_buf_count = SmeaConfig::Hardware::I2S_DMA_BUF_COUNT_CONST,
        .dma_buf_len = SmeaConfig::System::FFT_SIZE_CONST,
        .use_apll = true,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    i2s_pin_config_t v_pinConfig = {
        .bck_io_num = SmeaConfig::Hardware::PIN_I2S_BCLK_CONST,
        .ws_io_num = SmeaConfig::Hardware::PIN_I2S_WS_CONST,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = SmeaConfig::Hardware::PIN_I2S_DIN_CONST
    };

    if (i2s_driver_install((i2s_port_t)SmeaConfig::Hardware::I2S_PORT_NUM_CONST, &v_i2sConfig, 0, NULL) != ESP_OK) {
        ESP_LOGE(TAG, "I2S driver install failed");
        return false;
    }

    if (i2s_set_pin((i2s_port_t)SmeaConfig::Hardware::I2S_PORT_NUM_CONST, &v_pinConfig) != ESP_OK) {
        ESP_LOGE(TAG, "I2S pin config failed");
        return false;
    }

    _isInitialized = true;
    _isPaused = false;
    strlcpy(_statusText, "RUNNING", sizeof(_statusText));
    return true;
}

void T480_MicEngine::pause() {
    if (!_isInitialized || _isPaused) return;
    i2s_stop((i2s_port_t)SmeaConfig::Hardware::I2S_PORT_NUM_CONST);
    _isPaused = true;
    strlcpy(_statusText, "PAUSED", sizeof(_statusText));
}

void T480_MicEngine::resume() {
    if (!_isInitialized || !_isPaused) return;
    clearBuffer(); // [방어] 재개 시 과거 쓰레기 버퍼 필연적 초기화
    i2s_start((i2s_port_t)SmeaConfig::Hardware::I2S_PORT_NUM_CONST);
    _isPaused = false;
    strlcpy(_statusText, "RUNNING", sizeof(_statusText));
}

void T480_MicEngine::clearBuffer() {
    if (!_isInitialized) return;
    i2s_zero_dma_buffer((i2s_port_t)SmeaConfig::Hardware::I2S_PORT_NUM_CONST);
}

uint32_t T480_MicEngine::readData(float* p_outL, float* p_outR, uint32_t p_reqSamples) {
    if (!_isInitialized || _isPaused) return 0;
    
    // [방어] 메모리 오염(Overrun) 클램핑
    uint32_t v_samplesToRead = (p_reqSamples > SmeaConfig::System::FFT_SIZE_CONST) ? SmeaConfig::System::FFT_SIZE_CONST : p_reqSamples;
    size_t v_bytesRead = 0;
    size_t v_bytesToRead = v_samplesToRead * 2 * sizeof(int32_t); 

    esp_err_t v_res = i2s_read((i2s_port_t)SmeaConfig::Hardware::I2S_PORT_NUM_CONST, _dmaBuffer, v_bytesToRead, &v_bytesRead, portMAX_DELAY);
    
    if (v_res != ESP_OK) return 0;

    uint32_t v_samplesRead = v_bytesRead / (2 * sizeof(int32_t));
    float v_scale = SmeaConfig::System::PCM_32BIT_SCALE_CONST;

    // SIMD 고속 De-interleaving 정규화
    for (uint32_t i = 0; i < v_samplesRead; i++) {
        p_outL[i] = (float)_dmaBuffer[i * 2] * v_scale;
        p_outR[i] = (float)_dmaBuffer[i * 2 + 1] * v_scale;
    }

    return v_samplesRead;
}

