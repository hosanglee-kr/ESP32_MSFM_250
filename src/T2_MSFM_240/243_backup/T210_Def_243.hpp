/* ============================================================================
 * File: T210_Def_243.hpp
 * Summary: T240 멀티모달 시스템 전역 SSOT (MLOps & PdM 기능 100% 완전체 복원)
 * ============================================================================
 * [구현 원칙]
 * 1. [네임스페이스]: T2_Def(상수), T2_Type(구조체/Enum)으로 엄격 분리.
 * 2. 배열크기 등의 고정상수명은 _CONST 접미사, 기본값 등은 _DEF 접미사 적용.
 * 3. [SIMD 정렬]: 연산용 텐서 및 청크는 반드시 alignas(16) 적용 (총 384 Bytes).
 * 4. [바이너리 무결성]: 네트워크 패킷은 #pragma pack(1)로 패딩 제거 (Rule #37).
 * 5. [복원 완료]: OTA/Tuning 커맨드, Cepstrum/Top Peaks 지표, config_dump 복원.
 * [TODO] 
 * 1. 진동,소음 변수 명확히 구분
 * 2. 진동, 소음 샘플링 때문에 경합하는지 점검 필요
 * ========================================================================== */

#pragma once

#include <cstdint>

namespace T2_Def {

    namespace Sensor {
        inline constexpr uint8_t ACCEL_RANGE_DEF        = 8;    // 8G
        inline constexpr uint8_t ACCEL_ODR_DEF          = 0x0C; // 1600Hz (BMI270 규격)
        inline constexpr uint8_t GYRO_RANGE_DEF         = 2;    // 2000dps
    }

    // [1] 시스템 연산 상수 (_CONST)
    namespace System {
        inline constexpr uint32_t RATE_VIB_CONST            = 1600;   // 진동 샘플링 레이트
        inline constexpr uint32_t RATE_AUDIO_CONST          = 42000;  // 오디오 샘플링 레이트

        inline constexpr uint32_t FFT_SIZE_VIB_CONST        = 256;    // 진동 FFT 크기
        inline constexpr uint32_t FFT_SIZE_AUDIO_CONST      = 1024;   // 오디오 FFT 크기

        inline constexpr uint32_t VIB_AXIS_CONST            = 3;  // X, Y, Z
        inline constexpr uint32_t AUDIO_CH_CONST            = 2;  // L, R (Beamforming 전)

        inline constexpr uint32_t MFCC_COEFFS_CONST         = 13;  // 기본 MFCC 차원
        inline constexpr uint32_t MFCC_DIM_CONST            = 39;  // 13 + delta + delta-delta
        inline constexpr uint32_t MEL_BANDS_CONST           = 26;  // 필터뱅크 개수

        inline constexpr uint32_t SEQUENCE_FRAMES_MAX_CONST = 16;      // AI 시퀀스 길이
        inline constexpr float    MATH_EPSILON_CONST        = 1e-6f;   // [복원] 일반 방어용
        inline constexpr float    MATH_EPSILON_12_CONST     = 1e-12f;  // NaN 방어용 고정밀
        inline constexpr float    MS_PER_SEC_CONST          = 1000.0f; // [복원] 시간 환산 상수
        
        inline constexpr uint8_t TELEMETRY_HZ_DEF       = 10;
        inline constexpr uint8_t WAVEFORM_HZ_DEF        = 0;    // 기본 오프 (대역폭 방어)
        inline constexpr char const* SITE_ID_DEF        = "FACTORY_A_LINE_1";
    }

    namespace Hardware {
        inline constexpr uint32_t SERIAL_BAUD_CONST         = 115200;
        inline constexpr uint8_t  PIN_BTN_CONTROL_CONST     = 0;  // 메인 트리거 핀
        inline constexpr uint8_t  PIN_RGB_LED_CONST         = 21; // [복원] 상태 LED 핀

        // BMI270 (SPI)
        inline constexpr uint8_t  PIN_BMI_CS_CONST          = 10;
        inline constexpr uint8_t  PIN_BMI_INT1_CONST        = 11; // Any-motion용 인터럽트

        // ICS43434 (I2S)
        inline constexpr int      I2S_PORT_NUM_CONST        = 0;
        inline constexpr uint8_t  PIN_I2S_BCLK_CONST        = 41;
        inline constexpr uint8_t  PIN_I2S_MCLK_CONST        = 0; 
        inline constexpr uint8_t  PIN_I2S_WS_CONST          = 42;
        inline constexpr uint8_t  PIN_I2S_DIN_CONST         = 43;
        
        inline constexpr int      I2S_DMA_BUF_COUNT_CONST   = 16; // [복원] Wi-Fi 블로킹 방어

        // SDMMC 멀티핀 (1-bit / 4-bit) [복원]
        inline constexpr uint8_t  PIN_SD_CLK_CONST          = 39;
        inline constexpr uint8_t  PIN_SD_CMD_CONST          = 38;
        inline constexpr uint8_t  PIN_SD_D0_CONST           = 40;
    } 
    
    namespace Path {
        inline constexpr char const* MOUNT_SD_CONST         = "/sdcard";
        inline constexpr char const* FILE_CFG_JSON_CONST    = "/sys/runtime_cfg_243.json";
        inline constexpr char const* SD_DIR_RAW_CONST       = "/t240_data/raw";
        inline constexpr char const* FILE_BMI_CALIB_CONST   = "/sys/bmi_calib_243.json"; 
        inline constexpr char const* FILE_AUD_CALIB_CONST   = "/sys/aud_calib_243.json"; 
        inline constexpr char const* SD_DIR_BIN_CONST       = "/t240_data/bin";
        inline constexpr char const* DIR_FALLBACK_CONST     = "/fallback"; // [복원] SD카드 오류 시 LittleFS 우회 경로
    }

    // [복원] 비휘발성 저장소 (NVS) 파라미터
    namespace NVS {
        inline constexpr char const* NAMESPACE_CONST        = "t240_sys";
        inline constexpr char const* KEY_FILE_SEQ_CONST     = "file_seq";
    }

    namespace Dsp {
        inline constexpr int   MEDIAN_WINDOW_DEF        = 3;
        inline constexpr float NOTCH_FREQ_HZ_DEF        = 60.0f;
        inline constexpr float NOTCH_Q_FACTOR_DEF       = 30.0f;
        inline constexpr float FIR_HPF_CUTOFF_DEF       = 10.0f;
        inline constexpr float PRE_EMPHASIS_ALPHA_DEF   = 0.97f;
        inline constexpr float BEAMFORMING_GAIN_DEF     = 0.5f;
        
        // 윈도우 함수 적용에 따른 에너지 손실 보정 계수
        inline constexpr float WINDOW_AMPLITUDE_CORR_HANN   = 2.0000f; 
        inline constexpr float WINDOW_ENERGY_CORR_HANN      = 1.6330f;
    } 

    namespace Trigger {
        inline constexpr uint32_t HOLD_TIME_MS_DEF      = 5000;
        inline constexpr bool     USE_DEEP_SLEEP_DEF    = false;
        inline constexpr uint32_t SLEEP_TIMEOUT_SEC_DEF = 300;
        inline constexpr float    WAKE_THRESH_G_DEF     = 1.0f;
        inline constexpr uint16_t WAKE_DURATION_DEF     = 5;
    }

    // [4] 특징량 및 알고리즘 제한 (_CONST)
    namespace FeatureLimit {
        inline constexpr uint8_t  MAX_BAND_RMS_CONST    = 8;
        inline constexpr uint16_t FIR_TAPS_CONST        = 63;
        
        // [복원] 심화 진단 지표용 수학 상수
        inline constexpr float    MEL_SCALE_2595_CONST  = 2595.0f;                
        inline constexpr float    MEL_SCALE_700_CONST   = 700.0f;                 
        inline constexpr uint16_t MAX_PEAK_CANDIDATES_CONST = 128;                    
        inline constexpr uint8_t  TOP_PEAKS_COUNT_CONST = 5;                      
        inline constexpr uint8_t  CEPS_TARGET_COUNT_CONST = 3;                      
    }

    namespace Task {
        inline constexpr uint32_t WDG_TIMEOUT_MS_CONST  = 2000;

        // 스토리지 백그라운드 태스크 제어 상수
        inline constexpr uint32_t STORAGE_STACK_CONST   = 8192; 
        inline constexpr uint8_t  STORAGE_PRIO_CONST    = 2;    
        inline constexpr uint8_t  CORE_PROCESS_CONST    = 1;    

        // [복원] RTOS 매직넘버 철폐용 스케줄링 상수
        inline constexpr uint32_t ALIVE_CHECK_MS_CONST  = 5000;
        inline constexpr uint32_t QUEUE_BLOCK_MS_CONST  = 100;
        inline constexpr uint32_t REBOOT_DELAY_MS_CONST = 500;
        inline constexpr uint32_t BOOT_DELAY_MS_CONST   = 1000;
        inline constexpr uint32_t MAIN_LOOP_DELAY_MS_CONST = 10;
        inline constexpr uint32_t LAZY_WRITE_DELAY_MS_CONST = 3000;
    }

    namespace Web {
        inline constexpr uint16_t JSON_BUF_SIZE_CONST       = 2048; // [복원] 기본 API JSON
        inline constexpr uint16_t LARGE_JSON_BUF_SIZE_CONST = 8192; // [복원] Config 덤프용
        inline constexpr uint32_t BTN_DEBOUNCE_MS_CONST     = 500;  // [복원] 하드웨어 버튼 디바운스
    }

    namespace Storage {
        inline constexpr uint8_t  PRE_TRIGGER_SEC_DEF   = 3;
        inline constexpr uint32_t ROTATE_MB_DEF         = 10;
        inline constexpr uint32_t ROTATE_MIN_DEF        = 60;
        inline constexpr uint16_t ROTATE_KEEP_MAX_DEF   = 8;
        inline constexpr uint32_t IDLE_FLUSH_MS_DEF     = 250;
    } 

    // [5] 저장소 제약 (_CONST)
    namespace StorageLimit {
        inline constexpr uint16_t MAX_PATH_LEN_CONST    = 128;
        inline constexpr uint16_t MAX_ROTATE_LIST_CONST = 16;
        inline constexpr uint32_t PREALLOC_BYTES_CONST  = 10 * 1024 * 1024; // 10MB 선할당
    }

    // [6] 동적 설정 초기 기본값 (_DEF)
    namespace Decision {
        inline constexpr float RULE_VIB_RMS_THRESH_DEF   = 0.5f;
        inline constexpr float RULE_AUDIO_RMS_THRESH_DEF = 0.05f;
    } 

    // [3] 네트워크 및 통신 상수 (_CONST)
    namespace Net {
        inline constexpr uint8_t     WIFI_MULTI_MAX_CONST  = 3;
        inline constexpr uint16_t    MQTT_PORT_CONST       = 1883;
        inline constexpr char const* WS_URI_CONST          = "/api/t240/ws";
        inline constexpr char const* TZ_INFO_CONST         = "KST-9";
        inline constexpr char const* NTP_SERVER_1_CONST    = "pool.ntp.org";
        inline constexpr uint32_t    SYNC_TIMEOUT_MS_CONST = 5000;
        
        inline constexpr char const* WIFI_AP_SSID_DEF   = "SMEA_T240_AP";
        inline constexpr char const* WIFI_AP_PW_DEF     = "12345678";
        inline constexpr char const* WIFI_AP_IP_DEF     = "192.168.4.1";
        inline constexpr char const* MQTT_ID_DEF        = "T240_Edge";
        inline constexpr char const* MQTT_TOPIC_DEF     = "smea/t240";
    }
}

namespace T2_Type {

    // --- 시스템 열거형 ---
    enum class SystemState : uint8_t {
        INIT = 0,
        READY,
        MONITORING,
        RECORDING,
        NOISE_LEARNING,
        MAINTENANCE,
        ERROR,
        CALIBRATING
    };
    
    enum class StatusBit : uint8_t {
        NTP_SYNCED    = 0,
        SD_MOUNTED    = 1,
        RECORDING_NOW = 2,
        SENSOR_FAULT  = 3 
    };

    enum class DetectionResult : uint8_t {
        PASS = 0,
        RULE_VIB_NG,
        RULE_AUDIO_NG,
        TEST_NG,  // [복원]
        ML_NG     // [복원]
    };

    enum class SystemCommand : uint8_t {
        CMD_START = 0,
        CMD_STOP,
        CMD_LEARN_NOISE,
        CMD_CALIBRATE,
        CMD_REBOOT,
        CMD_OTA_START,       // [복원] OTA 진입
        CMD_OTA_END,         // [복원] OTA 종료
        CMD_TUNING_PREVIEW,  // [복원] Web UI 핫스왑
        CMD_TUNING_SAVE,     // [복원] 설정 영구 저장
        CMD_TUNING_CANCEL    // [복원] 설정 롤백
    };

    // [복원] 레코딩 트리거 발생 소스 (Explainable AI)
    enum class TriggerSource : uint8_t {
        NONE = 0,
        HW_WAKE,
        SW_RMS,
        SW_BAND,
        MANUAL
    };

    // [복원] 웹소켓 스트림 다중화 타입
    enum class StreamType : uint8_t {
        TELEMETRY   = 0x01,
        SPECTRUM    = 0x02,
        WAVEFORM    = 0x03,
        CALIBRATION = 0x04 
    };

    enum class NoiseMode : uint8_t { OFF = 0, FIXED, ADAPTIVE };
    enum class WiFiMode : uint8_t { STA_ONLY = 0, AP_ONLY, AP_STA, AUTO_FALLBACK };
    enum class WindowType : uint8_t { HANN = 0, HAMMING, BLACKMAN };

    // --- 설정 하위 구조체 ---
    struct ST_Config_Sensor {
        uint8_t accel_range; 
        uint8_t accel_odr;   
        uint8_t gyro_range;
    };

    struct ST_Config_Trigger {
        uint32_t hold_time_ms;
        bool     use_deep_sleep;
        uint32_t sleep_timeout_sec;
        bool     motion_en;     
        float    wake_thresh_g;
        uint16_t wake_duration;
        float    vib_rms_thresh;
        float    audio_rms_thresh;
        bool     band_enable[8];
        float    band_start_hz[8];
        float    band_end_hz[8];
        float    band_thresh[8];
    };

    struct ST_Config_FilterFIR { bool enabled; float cutoff_hz; uint16_t num_taps; };
    struct ST_Config_FilterIIR { bool enabled; float cutoff_hz; float q_factor; };
    struct ST_Config_FilterNotch { bool enabled; float target_freq_hz; float gain; float q_factor; };

    struct ST_Config_Noise {
        bool      enable_gate;
        float     gate_threshold_abs;
        NoiseMode mode;
        float     spectral_subtract_strength;
        float     adaptive_alpha;
        uint16_t  noise_learn_frames;
    };

    struct ST_Config_Dsp {
        bool                  remove_dc;
        bool                  median_enabled;
        uint8_t               median_window;
        ST_Config_FilterFIR   fir_hpf;
        ST_Config_FilterFIR   fir_lpf;
        ST_Config_FilterIIR   iir_hpf;
        ST_Config_FilterIIR   iir_lpf;
        ST_Config_FilterNotch notch;
        bool                  preemphasis_enable;
        float                 preemphasis_alpha;
        ST_Config_Noise       noise;
        WindowType            window_type;
        float                 beamforming_gain;
        float                 calib_gain_L;
        float                 calib_gain_R;
        alignas(16) float     calib_eq_coeffs[T2_Def::FeatureLimit::FIR_TAPS_CONST];
    };

    struct ST_Config_Calib {
        uint32_t auto_idle_min;
        float    ref_freq_hz;
        float    filter_min_freq_hz;
        float    filter_max_freq_hz;
        float    target_gain_max;
        float    target_gain_min;
        float    norm_safe_thresh;
    };

    struct ST_Config_Feature {
        float    ceps_targets[3];
        float    spatial_freq_min_hz;
        float    spatial_freq_max_hz;
        float    peak_amplitude_limit_min;
        float    peak_freq_gap_limit_hz_min;
    };

    struct ST_Config_Storage {
        uint32_t rotation_mb;
        uint32_t rotation_min;
        bool     save_raw;
        uint16_t rotate_keep_max;
        uint32_t idle_flush_ms;
        uint8_t  pre_trigger_sec;
    };

    struct ST_Config_WiFi {
        WiFiMode mode;
        char     ap_ssid[32];
        char     ap_password[64];
        char     ap_ip[16];
        char     multi_ssid[3][32];
        char     multi_pw[3][64];
    };
    
    struct ST_Config_Mqtt {
        bool     enable;
        char     broker[64];
        uint16_t port;
        char     id[32];      
        char     password[64];
        char     topic_root[64];
        uint8_t  qos;
    };
    
    struct ST_Config_System {
        char    site_id[32];
        uint8_t telemetry_hz;
        uint8_t waveform_hz;
    };

    // [복원] 외부 출력 제어 파이프라인 (대역폭 방어)
    struct ST_Config_Output {
        bool     enabled;
        bool     output_sequence; 
        uint16_t sequence_frames;
    };

    struct DynamicConfig {
        ST_Config_System  system; 
        ST_Config_Sensor  sensor;
        ST_Config_Trigger trigger;
        ST_Config_Dsp     dsp;
        ST_Config_Calib   calib;
        ST_Config_Feature feature;
        ST_Config_Storage storage;
        ST_Config_WiFi    wifi;
        ST_Config_Mqtt    mqtt;
        ST_Config_Output  output; // [복원]
        uint8_t           op_mode;
        uint32_t          watchdog_ms;
    };


    // --- 고장 진단 통합 데이터 규격 ---
    
    // [복원] 피크 배열 구조체
    struct SpectralPeak {
        float frequency;
        float amplitude;
    };

    // 16B Aligned 정밀 패딩 계산 구조체 (총 384 Bytes 완벽 보장)
    struct alignas(16) UnifiedFeatureSlot {
        // [Block 1] 16 bytes
        uint64_t timestamp;         // 8
        uint32_t frame_id;          // 4
        uint32_t uptime_ms;         // 4
        
        // [Block 2] 16 bytes
        uint8_t  status_flags;      // 1
        int8_t   internal_temp;     // 1
        uint8_t  trial_no;          // 1 (복원)
        uint8_t  trigger_source;    // 1 (복원, TriggerSource)
        uint8_t  _reserved[12];     // 12

        // [Block 3] 16 bytes
        float    vib_rms[3];        // 12
        float    audio_rms;         // 4
        
        // [Block 4] 16 bytes
        float    vib_peak_freq[3];  // 12
        float    spectral_centroid; // 4
        
        // [Block 5] 16 bytes
        float    vib_centroid[3];   // 12
        float    kurtosis;          // 4

        // [Block 6] 16 bytes
        float    crest_factor;      // 4
        float    sta_lta_ratio;     // 4
        float    vib_skewness;      // 4
        float    pooling_stddev_min;// 4 (복원)

        // [Block 7] 16 bytes (시계열 델타 복원)
        float    phase_coherence;   // 4 (복원)
        float    mean_ipd;          // 4 (복원)
        float    delta_rms;         // 4 (복원)
        float    delta_delta_rms;   // 4 (복원)

        // [Block 8, 9] 32 bytes
        float    band_energy[8];    // 32

        // [Block 10] 16 bytes (켑스트럼 복원)
        float    cpsr_max[3];       // 12 (복원)
        float    _pad_cpsr1;        // 4

        // [Block 11] 16 bytes
        float    cpsr_mxrms[3];     // 12 (복원)
        float    _pad_cpsr2;        // 4

        // [Block 12, 13, 14] 48 bytes (피크 어레이 복원)
        SpectralPeak top_peaks[5];  // 40 (복원)
        uint8_t  _pad_peaks[8];     // 8
        
        // [Block 15~24] 160 bytes
        alignas(16) float mfcc[39]; // 156
        float    _pad_mfcc;         // 4
    };

    struct alignas(16) UnifiedRawChunk {
        float vib[3][256];          
        float audio[1024];          
    };

    // --- 통신 패킷 송출 전용 규격 (Byte Packing 강제) ---
    #pragma pack(push, 1)

    // [복원] 데이터 MLOps 추적성 확보용 설정 덤프 헤더 
    struct FileHeader {
        char     magic[4];         // "SMEA"
        uint16_t version;          // 243
        uint16_t struct_size;      
        uint32_t sample_rate_hz;   // [복원]
        uint16_t fft_size;         // [복원]
        uint16_t mfcc_dim;         // [복원]
        uint8_t  active_axes;      // [복원]
        uint8_t  _reserved;        
        uint32_t total_records;    
        char     config_dump[T2_Def::Web::LARGE_JSON_BUF_SIZE_CONST]; // [복원] 8KB JSON 덤프
    };

    struct WsHeader {
        uint8_t  magic;   // 0x54 ('T')
        uint8_t  type;    // StreamType 매핑
        uint16_t length;  
        uint8_t  stage;   
        uint8_t  _pad[3]; 
    };

    struct PktTelemetry {
        WsHeader header;
        uint8_t  sys_state;
        uint8_t  detect_result;
        uint8_t  trial_no;       // [복원]
        uint8_t  trigger_source; // [복원]
        
        float    vib_rms[3];
        float    audio_rms;
        float    kurtosis;
        float    spectral_centroid;

        float    crest_factor;   
        float    sta_lta_ratio;  
        float    vib_centroid[3]; 

        float    band_energy[8]; 
        
        float    cpsr_max[3];    // [복원]
        float    cpsr_mxrms[3];  // [복원]
        SpectralPeak top_peaks[5]; // [복원]

        float    mfcc[39];       
    };

    struct PktSpectrum {
        WsHeader header;
        float    frequencies[513];  // (1024/2) + 1
    };

    struct PktWaveform {
        WsHeader header;
        float    samples[1024];
    };

    // [복원] Web UI 필터 튜닝 시각화 전용 패킷
    struct PktCalibration {
        WsHeader header;
        float    target_gain_curve[(T2_Def::System::FFT_SIZE_AUDIO_CONST / 2) + 1];
        float    actual_fir_coeffs[T2_Def::FeatureLimit::FIR_TAPS_CONST];
    };

    #pragma pack(pop)
}
