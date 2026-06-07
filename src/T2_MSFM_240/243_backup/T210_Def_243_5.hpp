/* ============================================================================
 * File: T210_Def_243_5.hpp
 * Summary: T240 4-Tier 통합 상수/기본값 정의 SSOT (축소/누락 무결점 검증본)
 * ============================================================================
 * [시스템 구현 원칙 - Core Principles]
 * 1. [도메인 분리]: Global(인프라), Shared(공통로직), Vib(진동), Audio(소음).
 * 2. [네이밍 룰]: 고정상수는 _CONST, 동적 설정 초기값은 _DEF 접미사 강제.
 * 3. [매직넘버 철폐]: 문자열 크기, 태스크 딜레이 등 모든 숫자 하드코딩 제거.
 * 4. [수식 연동 방어]: 샘플링 레이트 등 의존성 있는 상수는 계산식으로 묶음 처리.
 * 5. [현장 방어벽]: FSM 큐 길이, FIFO 배치 크기 등 RTOS 제약 상수 복원.
 * ========================================================================== */

#pragma once

#include <cstdint>

namespace T2_Def {

    // ========================================================================
    // 1. Global: 디바이스 전역 인프라 (System, Task, Storage, Network)
    // ========================================================================
    namespace Global {
        namespace System {
            inline constexpr char const* VERSION_STR         = "T240_v243_5";
            inline constexpr char const* SITE_ID_DEF         = "FACTORY_A_LINE_1";
            inline constexpr uint8_t     TELEMETRY_HZ_DEF    = 10;
            inline constexpr uint8_t     WAVEFORM_HZ_DEF     = 0;
            
            // [복원] 시스템 전역 AI 텐서 시퀀스 제어
            inline constexpr uint16_t    SEQUENCE_FRAMES_MAX_CONST = 16;
            inline constexpr uint16_t    SEQUENCE_FRAMES_DEF       = 16;
            
            inline constexpr float       MATH_EPSILON_CONST  = 1e-6f;
            inline constexpr float       MATH_EPSILON_12_CONST= 1e-12f;
            inline constexpr float       MS_PER_SEC_CONST    = 1000.0f;
            inline constexpr uint32_t    BYTES_PER_MB_CONST  = 1024 * 1024;
            inline constexpr uint32_t    MS_PER_MIN_CONST    = 60 * 1000;
        }

        namespace Hardware {
            inline constexpr uint32_t    SERIAL_BAUD_CONST   = 115200;
            inline constexpr uint8_t     PIN_BTN_CONTROL_CONST = 0;
            inline constexpr uint8_t     PIN_RGB_LED_CONST   = 21;
            
            inline constexpr uint8_t     PIN_SD_CLK_CONST    = 39;
            inline constexpr uint8_t     PIN_SD_CMD_CONST    = 38;
            inline constexpr uint8_t     PIN_SD_D0_CONST     = 40;
            inline constexpr uint8_t     PIN_NOT_SET_CONST   = 0xFF; // [복원] 미할당 핀 상태
        }

        namespace Task {
            inline constexpr uint16_t    QUEUE_LEN_CONST      = 8; // [복원] FSM 통신 큐 길이
            inline constexpr uint32_t    WDG_TIMEOUT_MS_CONST = 2000;
            
            inline constexpr uint32_t    STORAGE_STACK_CONST  = 8192;
            inline constexpr uint8_t     STORAGE_PRIO_CONST   = 2;    
            inline constexpr uint8_t     CORE_PROCESS_CONST   = 1;
            
            inline constexpr uint32_t    ALIVE_CHECK_MS_CONST = 5000;
            inline constexpr uint32_t    QUEUE_BLOCK_MS_CONST = 100;
            inline constexpr uint32_t    REBOOT_DELAY_MS_CONST= 500;
            inline constexpr uint32_t    BOOT_DELAY_MS_CONST  = 1000;
            inline constexpr uint32_t    MAIN_LOOP_DELAY_MS_CONST = 10;
            inline constexpr uint32_t    LAZY_WRITE_MS_CONST  = 3000;
        }

        namespace Path {
            inline constexpr char const* MOUNT_SD_CONST      = "/sdcard";
            inline constexpr char const* FILE_CFG_JSON_CONST = "/sys/runtime_cfg_243.json";
            inline constexpr char const* DIR_FALLBACK_CONST  = "/fallback";
        }

        namespace NVS {
            inline constexpr char const* NAMESPACE_CONST     = "t240_sys";
            inline constexpr char const* KEY_FILE_SEQ_CONST  = "file_seq";
        }

        namespace NetLimit {
            inline constexpr uint16_t    MAX_SSID_LEN_CONST  = 32;                     
            inline constexpr uint16_t    MAX_PW_LEN_CONST    = 64;                     
            inline constexpr uint16_t    MAX_IP_LEN_CONST    = 16;                     
            inline constexpr uint8_t     MAX_MULTI_AP_CONST  = 3;                 
            inline constexpr uint16_t    MAX_BROKER_LEN_CONST= 64;                
            inline constexpr uint32_t    WIFI_DISCONNECT_DELAY_MS_CONST = 100;
        }

        namespace Net {
            inline constexpr uint16_t    MQTT_PORT_CONST     = 1883;
            inline constexpr char const* WS_URI_CONST        = "/api/t240/ws";
            inline constexpr char const* TZ_INFO_CONST       = "KST-9";
            inline constexpr char const* NTP_SERVER_1_CONST  = "pool.ntp.org";
            inline constexpr uint32_t    SYNC_TIMEOUT_MS_CONST= 5000;
            
            inline constexpr char const* WIFI_AP_SSID_DEF    = "SMEA_T240_AP";
            inline constexpr char const* WIFI_AP_PW_DEF      = "12345678";
            inline constexpr char const* WIFI_AP_IP_DEF      = "192.168.4.1";
            
            inline constexpr char const* MQTT_ID_DEF         = "T240_Edge";
            inline constexpr char const* MQTT_TOPIC_DEF      = "smea/t240";
            inline constexpr char const* MQTT_LWT_DEF        = "smea/t240/lwt";
            inline constexpr uint8_t     MQTT_PROTO_VER_DEF  = 4; // MQTT 3.1.1
            inline constexpr uint8_t     MQTT_QOS_DEF        = 1;
        }

        namespace StorageLimit {
            inline constexpr uint16_t    MAX_PATH_LEN_CONST  = 128;
            inline constexpr uint16_t    MAX_PREFIX_LEN_CONST= 16;
            inline constexpr uint16_t    MAX_ROTATE_LIST_CONST= 16;
            inline constexpr uint16_t    MAX_DIR_FILES_CONST = 100; 
            inline constexpr uint32_t    PREALLOC_BYTES_CONST= 10 * System::BYTES_PER_MB_CONST; 
            inline constexpr uint16_t    WATERMARK_HIGH_CONST= 8;
        }

        namespace Storage {
            inline constexpr uint8_t     PRE_TRIGGER_SEC_DEF = 3;
            inline constexpr uint32_t    ROTATE_MB_DEF       = 10;
            inline constexpr uint32_t    ROTATE_MIN_DEF      = 60;
            inline constexpr uint16_t    ROTATE_KEEP_MAX_DEF = 8;
            inline constexpr uint32_t    IDLE_FLUSH_MS_DEF   = 250;
            
            inline constexpr char const* SD_DIR_RAW_CONST    = "/t240_data/raw";
            inline constexpr char const* SD_DIR_BIN_CONST    = "/t240_data/bin";
        }
    }

    // ========================================================================
    // 2. Shared: 도메인 공유 신호처리 및 판정 로직
    // ========================================================================
    namespace Shared {
        namespace Trigger {
            inline constexpr uint32_t    HOLD_TIME_MS_DEF    = 5000;
            inline constexpr bool        USE_DEEP_SLEEP_DEF  = false;
            inline constexpr uint32_t    SLEEP_SEC_DEF       = 300;
            inline constexpr uint16_t    WAKE_DURATION_DEF   = 5;
        }
        namespace Dsp {
            inline constexpr int         MEDIAN_WINDOW_DEF   = 3;
            inline constexpr uint16_t    FIR_TAPS_CONST      = 63;
            inline constexpr float       NOTCH_FREQ_HZ_DEF   = 60.0f;
            inline constexpr float       NOTCH_Q_FACTOR_DEF  = 30.0f;
            inline constexpr float       FIR_HPF_CUTOFF_DEF  = 10.0f;
            inline constexpr float       PRE_EMPHASIS_ALPHA_DEF = 0.97f;
            inline constexpr float       WINDOW_AMPLITUDE_CORR_HANN = 2.0000f; 
            inline constexpr float       WINDOW_ENERGY_CORR_HANN    = 1.6330f;
        }
        namespace DecisionLimit {
            inline constexpr uint8_t     MAX_TRIAL_COUNT_CONST = 3; 
        }
        namespace Decision {
            inline constexpr float       STA_LTA_THRESHOLD_DEF = 3.0f;
            inline constexpr int         MIN_TRIGGER_COUNT_DEF = 1;
            inline constexpr float       VALID_START_SEC_DEF   = 0.30f; 
            inline constexpr float       VALID_END_SEC_DEF     = 0.50f;
        }
    }

    // ========================================================================
    // 3. Vib: 진동 특화 파라미터
    // ========================================================================
    namespace Vib {
        namespace Hardware {
            inline constexpr uint8_t     PIN_BMI_CS_CONST    = 10;
            inline constexpr uint8_t     PIN_BMI_INT1_CONST  = 11;
            inline constexpr uint32_t    SPI_FREQ_HZ_CONST   = 10000000; // [복원] 10MHz
            inline constexpr uint16_t    FIFO_BATCH_SIZE_CONST = 32;     // [복원] 인터럽트 캐시 제어
        }
        namespace Sensor {
            inline constexpr uint32_t    RATE_CONST          = 1600;
            inline constexpr uint32_t    FFT_SIZE_CONST      = 256;
            inline constexpr uint8_t     AXIS_MAX_CONST      = 3; // [복원]
            inline constexpr uint8_t     AXIS_DEF            = 2; // [복원] 기본 Z축(2) 선택
            inline constexpr uint8_t     ACCEL_RANGE_DEF     = 8;
            inline constexpr uint8_t     ACCEL_ODR_DEF       = 0x0C;
            inline constexpr uint8_t     GYRO_RANGE_DEF      = 2;
        }
        namespace Trigger {
            inline constexpr float       RMS_THRESH_DEF      = 0.5f;
            inline constexpr float       WAKE_THRESH_G_DEF   = 1.0f;
        }
        namespace FeatureLimit {
            inline constexpr uint32_t    STA_SAMPLES_CONST   = Sensor::RATE_CONST / 1000; 
            inline constexpr uint32_t    LTA_SAMPLES_CONST   = Sensor::RATE_CONST / 100;
        }
        namespace Calib {
            inline constexpr char const* FILE_JSON_CONST     = "/sys/bmi_calib_243.json"; 
        }
    }

    // ========================================================================
    // 4. Audio: 소음 특화 파라미터
    // ========================================================================
    namespace Audio {
        namespace Hardware {
            inline constexpr int         I2S_PORT_NUM_CONST  = 0;
            inline constexpr uint8_t     PIN_I2S_BCLK_CONST  = 41;
            inline constexpr uint8_t     PIN_I2S_MCLK_CONST  = 0; 
            inline constexpr uint8_t     PIN_I2S_WS_CONST    = 42;
            inline constexpr uint8_t     PIN_I2S_DIN_CONST   = 43;
            inline constexpr int         DMA_BUF_COUNT_CONST = 16;
            inline constexpr uint16_t    RAW_FRAME_BUFFERS_CONST = 4; // [복원] 오디오 핑퐁 버퍼 슬롯
        }
        namespace Sensor {
            inline constexpr uint32_t    RATE_CONST          = 42000;
            inline constexpr uint32_t    FFT_SIZE_CONST      = 1024;
            inline constexpr uint8_t     CH_CONST            = 2;
            inline constexpr uint8_t     BITS_PER_SAMPLE_CONST= 32;
        }
        namespace Trigger {
            inline constexpr float       RMS_THRESH_DEF      = 0.05f;
            inline constexpr uint8_t     MAX_BAND_RMS_CONST  = 8;
            
            inline constexpr float       BAND_RANGES_DEF[8][2] = {
                {10.0f, 150.0f}, {150.0f, 1000.0f}, {1000.0f, 5000.0f}, {5000.0f, 20000.0f},
                {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}
            };
        }
        namespace Dsp {
            inline constexpr float       BEAMFORMING_GAIN_DEF= 0.5f;
            inline constexpr float       WINDOW_MS_DEF       = 25.0f;              
            inline constexpr float       HOP_MS_DEF          = 10.0f;
            inline constexpr float       SPECTRAL_SUB_GAIN_DEF = 1.2f;
            inline constexpr float       NOISE_GATE_THRESH_DEF = 0.001f;
        }
        namespace FeatureLimit {
            inline constexpr uint32_t    MFCC_COEFFS_CONST   = 13;  
            inline constexpr uint32_t    MFCC_DIM_CONST      = 39;  
            inline constexpr uint32_t    MEL_BANDS_CONST     = 26;  
            inline constexpr uint16_t    DELTA_HISTORY_FRAMES_CONST = 5;
            
            inline constexpr uint32_t    STA_SAMPLES_CONST   = Sensor::RATE_CONST / 1000; 
            inline constexpr uint32_t    LTA_SAMPLES_CONST   = Sensor::RATE_CONST / 100;
            
            inline constexpr float       MEL_SCALE_2595_CONST= 2595.0f;                
            inline constexpr float       MEL_SCALE_700_CONST = 700.0f;                 
            inline constexpr uint16_t    MAX_PEAK_CANDIDATES_CONST = 128;                    
            inline constexpr uint8_t     TOP_PEAKS_COUNT_CONST = 5;                      
            inline constexpr uint8_t     CEPS_TARGET_COUNT_CONST = 3;    
            inline constexpr float       CEPS_TOLERANCE_CONST= 0.0003f;
        }
        namespace CalibLimit {
            inline constexpr float       GAIN_RATIO_MIN_CONST= 0.707f; 
            inline constexpr float       GAIN_RATIO_MAX_CONST= 1.414f; 
            inline constexpr uint32_t    WELCH_CHUNK_SAMPLES_CONST = Sensor::FFT_SIZE_CONST;
            inline constexpr uint8_t     AUTO_STABLE_SEC_CONST = 10;
        }
        namespace Calib {
            inline constexpr char const* FILE_JSON_CONST     = "/sys/aud_calib_243.json"; 
            inline constexpr uint32_t    AUTO_IDLE_MIN_DEF   = 60;
            inline constexpr float       REF_FREQ_HZ_DEF     = 1000.0f;
            inline constexpr float       FILTER_MIN_FREQ_HZ_DEF= 100.0f;
            inline constexpr float       FILTER_MAX_FREQ_HZ_DEF= 8000.0f;
            inline constexpr float       TARGET_GAIN_MAX_DEF = 3.0f;
            inline constexpr float       TARGET_GAIN_MIN_DEF = 0.3f;
            inline constexpr float       NORM_SAFE_THRESH_DEF= 1.5f;
        }
    }
}

