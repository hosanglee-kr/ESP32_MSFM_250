/* ============================================================================
 * File: T210_Def_243_8.hpp
 * Summary: T240 통합 SSOT (Vib-Audio 시너지 & 동적 리소스 제어 완전체)
 * ============================================================================
 * [구현 원칙 및 아키텍처 가이드라인 - v243_7 보완본]
 * * 1. [4-Tier 도메인 격리]:
 * - Global(인프라), Shared(공통로직), Vib(진동), Audio(소음) 계층 유지.
 * * 2. [리소스 제약의 이중화 - _MAX vs _DEF]:
 * - 모든 메모리/배열 크기는 정적 상한선인 **_MAX**를 따름 (OOM 방어).
 * - 실제 연산 및 초기화 루프는 동적 설정값인 **_DEF**를 따름 (런타임 최적화).
 * * 3. [센서 모듈 및 채널 스위치]:
 * - 모듈(Vib/Audio), 하드웨어(Accel/Gyro), 소프트웨어(Mono/Stereo, 1/3축)별
 * 개별 ENABLE_DEF 스위치를 배치하여 전력 및 연산 자원 관리 최적화.
 * * 4. [계산식 연동 방어 (Derived SSOT)]:
 * - 타 상수에 의존하는 값(ex: MFCC 총 차원, STA 샘플수 등)은 하드코딩 금지.
 * - 반드시 컴파일 타임 계산식으로 정의하여 파라미터 간 정합성 자동 보장.
 * * 5. [매직넘버 및 익명구조체 금지]:
 * - 모든 숫자는 의미 있는 상수 네임스페이스 내에 존재해야 함.
 * - 모든 데이터는 명시적 타입(ST_...)을 가져야 함 (Type 파일 분리 예정).
 * * 6. [SIMD 및 네트워크 정렬]:
 * - 연산 슬롯은 alignas(16) 정렬, 네트워크 패킷은 SoA 평탄화 및 #pragma pack(1).
 * ========================================================================== */

#pragma once

#include <cstdint>

namespace T2_Def {

    // ========================================================================
    // 1. Global: 디바이스 인프라 (모듈 스위치 및 시스템 한계)
    // ========================================================================
    namespace Global {
        namespace System {
            inline constexpr char const* VERSION_STR           = "T240_v243_7";
            inline constexpr char const* SITE_ID_DEF           = "FACTORY_A_LINE_1";

            inline constexpr bool        VIB_ENABLE_DEF        = true;
            inline constexpr bool        AUDIO_ENABLE_DEF      = true;

            inline constexpr uint8_t     TELEMETRY_HZ_DEF      = 10;
            inline constexpr uint8_t     WAVEFORM_HZ_DEF       = 0;

            //
            inline constexpr uint16_t    SEQUENCE_FRAMES_MAX   = 32;
            inline constexpr uint16_t    SEQUENCE_FRAMES_DEF   = 16;

            inline constexpr float       MATH_EPSILON_CONST    = 1e-6f;
            inline constexpr float       MATH_EPSILON_12_CONST = 1e-12f;
            inline constexpr float       MS_PER_SEC_CONST      = 1000.0f;
            inline constexpr uint32_t    BYTES_PER_MB_CONST    = 1024 * 1024;
            inline constexpr uint32_t    MS_PER_MIN_CONST      = 60 * 1000;
        }

        namespace Hardware {
            inline constexpr uint32_t    SERIAL_BAUD_CONST     = 115200;
            inline constexpr uint8_t     PIN_BTN_CONTROL_CONST = 0;
            inline constexpr uint8_t     PIN_RGB_LED_CONST     = 21;
            inline constexpr uint8_t     PIN_SD_CLK_CONST      = 39;
            inline constexpr uint8_t     PIN_SD_CMD_CONST      = 38;
            inline constexpr uint8_t     PIN_SD_D0_CONST       = 40;
            inline constexpr uint8_t     PIN_NOT_SET_CONST     = 0xFF;
        }

        namespace Task {
            inline constexpr uint16_t    QUEUE_LEN_MAX         = 16;
            inline constexpr uint16_t    QUEUE_LEN_DEF         = 8;
            inline constexpr uint32_t    WDG_TIMEOUT_MS_DEF    = 2000;
            inline constexpr uint32_t    STORAGE_STACK_MAX     = 16384;
            inline constexpr uint32_t    STORAGE_STACK_DEF     = 8192;
            inline constexpr uint8_t     STORAGE_PRIO_DEF      = 2;
            inline constexpr uint8_t     CORE_PROCESS_DEF      = 1;

            inline constexpr uint32_t    ALIVE_CHECK_MS_DEF     = 5000;
            inline constexpr uint32_t    QUEUE_BLOCK_MS_DEF     = 100;
            inline constexpr uint32_t    REBOOT_DELAY_MS_DEF    = 500;
            inline constexpr uint32_t    BOOT_DELAY_MS_DEF      = 1000;
            inline constexpr uint32_t    MAIN_LOOP_DELAY_MS_DEF = 10;
            inline constexpr uint32_t    LAZY_WRITE_MS_DEF      = 3000;
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
            inline constexpr uint16_t    MAX_SSID_LEN_CONST           = 32;
            inline constexpr uint16_t    MAX_PW_LEN_CONST             = 64;
            inline constexpr uint16_t    MAX_IP_LEN_CONST             = 16;
            inline constexpr uint8_t     MAX_MULTI_AP_CONST           = 3;
            inline constexpr uint16_t    MAX_BROKER_LEN_CONST         = 64;
            inline constexpr uint32_t    WIFI_DISCONNECT_DELAY_MS_DEF = 100;
        }

        namespace Net {
            inline constexpr uint16_t    MQTT_PORT_DEF       = 1883;
            inline constexpr char const* WS_URI_CONST        = "/api/t240/ws";
            inline constexpr char const* TZ_INFO_CONST       = "KST-9";
            inline constexpr char const* NTP_SERVER_1_CONST  = "pool.ntp.org";
            inline constexpr uint32_t    SYNC_TIMEOUT_MS_DEF = 5000;

            inline constexpr char const* WIFI_AP_SSID_DEF    = "SMEA_T240_AP";
            inline constexpr char const* WIFI_AP_PW_DEF      = "12345678";
            inline constexpr char const* WIFI_AP_IP_DEF      = "192.168.4.1";
            inline constexpr char const* MQTT_ID_DEF         = "T240_Edge";
            inline constexpr char const* MQTT_TOPIC_DEF      = "smea/t240";
            inline constexpr char const* MQTT_LWT_DEF        = "smea/t240/lwt";
            inline constexpr uint8_t     MQTT_PROTO_VER_DEF  = 4;
            inline constexpr uint8_t     MQTT_QOS_DEF        = 1;
        }

        namespace StorageLimit {
            inline constexpr uint16_t    MAX_PATH_LEN_CONST  = 128;
            inline constexpr uint16_t    MAX_PREFIX_LEN_CONST= 16;
            inline constexpr uint16_t    MAX_ROTATE_LIST_MAX = 32;
            inline constexpr uint16_t    MAX_DIR_FILES_MAX   = 500;
            inline constexpr uint32_t    PREALLOC_BYTES_DEF  = 10 * System::BYTES_PER_MB_CONST;
            inline constexpr uint16_t    WATERMARK_HIGH_DEF  = 8;
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
    // 2. Shared: 통합 신호처리 및 한계치(Limit)
    // ========================================================================
    namespace Shared {
        namespace FeatureLimit {
            inline constexpr uint8_t     BAND_RMS_MAX        = 16;
            inline constexpr uint8_t     TOP_PEAKS_MAX       = 10;
            inline constexpr uint8_t     CEPS_TARGET_MAX     = 5;
            inline constexpr uint16_t    FIR_TAPS_MAX        = 127;
            inline constexpr uint16_t    DELTA_HISTORY_MAX   = 10;
        }

        namespace Feature {
            inline constexpr uint8_t     BAND_RMS_DEF        = 8;
            inline constexpr uint8_t     TOP_PEAKS_DEF       = 5;
            inline constexpr uint8_t     CEPS_TARGET_DEF     = 3;
            inline constexpr uint16_t    FIR_TAPS_DEF        = 63;
            inline constexpr uint16_t    DELTA_HISTORY_DEF   = 5;
            inline constexpr float       CEPS_TOLERANCE_DEF  = 0.0003f;
        }

        namespace Trigger {
            inline constexpr uint32_t    HOLD_TIME_MS_DEF    = 5000;
            inline constexpr bool        USE_DEEP_SLEEP_DEF  = false;
            inline constexpr uint32_t    SLEEP_SEC_DEF       = 300;
            inline constexpr uint16_t    WAKE_DURATION_DEF   = 5;
        }

        namespace Dsp {
            inline constexpr int         MEDIAN_WINDOW_MAX          = 15;
            inline constexpr int         MEDIAN_WINDOW_DEF          = 3;
            inline constexpr float       NOTCH_FREQ_HZ_DEF          = 60.0f;
            inline constexpr float       NOTCH_Q_FACTOR_DEF         = 30.0f;
            inline constexpr float       FIR_HPF_CUTOFF_DEF         = 10.0f;
            inline constexpr float       PRE_EMPHASIS_ALPHA_DEF     = 0.97f;
            inline constexpr float       WINDOW_AMPLITUDE_CORR_HANN = 2.0000f;
            inline constexpr float       WINDOW_ENERGY_CORR_HANN    = 1.6330f;
        }

        namespace Decision {
            inline constexpr uint8_t     MAX_TRIAL_COUNT_MAX        = 5;
            inline constexpr uint8_t     MAX_TRIAL_COUNT_DEF        = 3;
            inline constexpr float       STA_LTA_THRESHOLD_DEF      = 3.0f;
            inline constexpr int         MIN_TRIGGER_COUNT_DEF      = 1;
            inline constexpr float       VALID_START_SEC_DEF        = 0.30f;
            inline constexpr float       VALID_END_SEC_DEF          = 0.50f;
        }
    }

    // ========================================================================
    // 3. Vib: 진동 특화 파라미터
    // ========================================================================
    namespace Vib {
        namespace Hardware {
            inline constexpr uint8_t     PIN_BMI_CS_CONST    = 10;
            inline constexpr uint8_t     PIN_BMI_INT1_CONST  = 11;
            inline constexpr uint32_t    SPI_FREQ_HZ_CONST   = 10000000;
            inline constexpr uint16_t    FIFO_BATCH_SIZE_MAX = 64;
            inline constexpr uint16_t    FIFO_BATCH_SIZE_DEF = 32;
        }
        namespace Sensor {
            inline constexpr bool        ACCEL_ENABLE_DEF    = true;
            inline constexpr bool        GYRO_ENABLE_DEF     = false;
            inline constexpr uint8_t     AXIS_MAX            = 3;
            inline constexpr uint8_t     AXIS_COUNT_DEF      = 3; // 1축 vs 3축 연산
            inline constexpr uint8_t     TARGET_AXIS_DEF     = 2; // Z축 (단일축일 경우)

            inline constexpr uint32_t    RATE_MAX            = 3200;
            inline constexpr uint32_t    RATE_DEF            = 1600;
            inline constexpr uint32_t    FFT_SIZE_MAX        = 1024;
            inline constexpr uint32_t    FFT_SIZE_DEF        = 256;

            inline constexpr uint8_t     ACCEL_RANGE_DEF     = 8;
            inline constexpr uint8_t     ACCEL_ODR_DEF       = 0x0C;
            inline constexpr uint8_t     GYRO_RANGE_DEF      = 2;
        }
        namespace Trigger {
            inline constexpr float       RMS_THRESH_DEF      = 0.5f;
            inline constexpr float       WAKE_THRESH_G_DEF   = 1.0f;

            inline constexpr float       BAND_RANGES_DEF[Shared::FeatureLimit::BAND_RMS_MAX][2] = {
                {10.0f, 30.0f}, {30.0f, 60.0f}, {60.0f, 100.0f}, {100.0f, 250.0f},
                {250.0f, 400.0f}, {400.0f, 600.0f}, {600.0f, 800.0f}, {0.0f, 0.0f},
                {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f},
                {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}
            };
        }
        namespace FeatureLimit {
            inline constexpr uint32_t    STA_SAMPLES_DEF     = Sensor::RATE_DEF / 1000;
            inline constexpr uint32_t    LTA_SAMPLES_DEF     = Sensor::RATE_DEF / 100;
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
            inline constexpr int         I2S_PORT_NUM_CONST     = 0;
            inline constexpr uint8_t     PIN_I2S_BCLK_CONST     = 41;
            inline constexpr uint8_t     PIN_I2S_MCLK_CONST     = 0;
            inline constexpr uint8_t     PIN_I2S_WS_CONST       = 42;
            inline constexpr uint8_t     PIN_I2S_DIN_CONST      = 43;
            inline constexpr int         DMA_BUF_COUNT_MAX      = 32;
            inline constexpr int         DMA_BUF_COUNT_DEF      = 16;
            inline constexpr uint16_t    RAW_FRAME_BUFFERS_MAX  = 8;
            inline constexpr uint16_t    RAW_FRAME_BUFFERS_DEF  = 4;
        }
        namespace Sensor {
            inline constexpr uint8_t     CH_MODE_MONO_L_CONST   = 1;
            inline constexpr uint8_t     CH_MODE_MONO_R_CONST   = 2;
            inline constexpr uint8_t     CH_MODE_STEREO_CONST   = 3;

            inline constexpr uint8_t     CHANNELS_MAX           = 2;
            inline constexpr uint8_t     CHANNEL_MODE_DEF       = CH_MODE_STEREO_CONST;
            inline constexpr uint8_t     BITS_PER_SAMPLE_CONST  = 32;

            inline constexpr uint32_t    RATE_MAX               = 48000;
            inline constexpr uint32_t    RATE_DEF               = 42000;
            inline constexpr uint32_t    FFT_SIZE_MAX           = 4096;
            inline constexpr uint32_t    FFT_SIZE_DEF           = 1024;
        }
        namespace Trigger {
            inline constexpr float       RMS_THRESH_DEF         = 0.05f;
            inline constexpr float       CREST_FACTOR_NG_DEF    = 4.0f;
            inline constexpr float       SKEWNESS_NG_DEF        = 2.0f;

            inline constexpr float       BAND_RANGES_DEF[Shared::FeatureLimit::BAND_RMS_MAX][2] = {
                {100.0f, 500.0f}   , {500.0f, 1500.0f}   , {1500.0f, 4000.0f}  , {4000.0f, 8000.0f},
                {8000.0f, 12000.0f}, {12000.0f, 16000.0f}, {16000.0f, 20000.0f}, {0.0f, 0.0f},
                {0.0f, 0.0f}       , {0.0f, 0.0f}        , {0.0f, 0.0f}        , {0.0f, 0.0f},
                {0.0f, 0.0f}       , {0.0f, 0.0f}        , {0.0f, 0.0f}        , {0.0f, 0.0f}
            };
        }
        namespace Dsp {
            inline constexpr float       BEAMFORMING_GAIN_DEF    = 0.5f;
            inline constexpr float       WINDOW_MS_DEF           = 25.0f;
            inline constexpr float       HOP_MS_DEF              = 10.0f;
            inline constexpr float       SPECTRAL_SUB_GAIN_DEF   = 1.2f;
            inline constexpr float       NOISE_GATE_THRESH_DEF   = 0.001f;
        }
        namespace FeatureLimit {
            inline constexpr uint32_t    MFCC_COEFFS_MAX         = 32;
            inline constexpr uint32_t    MFCC_COEFFS_DEF         = 13;
            inline constexpr uint32_t    MFCC_COMPONENTS_MAX     = 3; // (Static, Delta, D-Delta)
            inline constexpr uint32_t    MFCC_COMPONENTS_DEF     = 3;

            inline constexpr uint32_t    MFCC_DIM_MAX            = MFCC_COEFFS_MAX * MFCC_COMPONENTS_MAX; // 96
            inline constexpr uint32_t    MFCC_DIM_DEF            = MFCC_COEFFS_DEF * MFCC_COMPONENTS_DEF; // 39

            inline constexpr uint32_t    MEL_BANDS_MAX           = 40;
            inline constexpr uint32_t    MEL_BANDS_DEF           = 26;

            inline constexpr float       MEL_SCALE_2595_CONST    = 2595.0f;
            inline constexpr float       MEL_SCALE_700_CONST     = 700.0f;

            inline constexpr uint32_t    STA_SAMPLES_DEF         = Sensor::RATE_DEF / 1000;
            inline constexpr uint32_t    LTA_SAMPLES_DEF         = Sensor::RATE_DEF / 100;
        }
        namespace CalibLimit {
            inline constexpr float       GAIN_RATIO_MIN_CONST    = 0.707f;
            inline constexpr float       GAIN_RATIO_MAX_CONST    = 1.414f;
            inline constexpr uint32_t    WELCH_CHUNK_SAMPLES_DEF = Sensor::FFT_SIZE_DEF;
            inline constexpr uint8_t     AUTO_STABLE_SEC_DEF     = 10;
        }
        namespace Calib {
            inline constexpr char const* FILE_JSON_CONST         = "/sys/aud_calib_243.json";
            inline constexpr uint32_t    AUTO_IDLE_MIN_DEF       = 60;
            inline constexpr float       REF_FREQ_HZ_DEF         = 1000.0f;
            inline constexpr float       FILTER_MIN_FREQ_HZ_DEF  = 100.0f;
            inline constexpr float       FILTER_MAX_FREQ_HZ_DEF  = 8000.0f;
            inline constexpr float       TARGET_GAIN_MAX_DEF     = 3.0f;
            inline constexpr float       TARGET_GAIN_MIN_DEF     = 0.3f;
            inline constexpr float       NORM_SAFE_THRESH_DEF    = 1.5f;
        }
    }
}
