/* ============================================================================
 * File: T210_Def_243_3.hpp
 * Summary: T240 4-Tier(Global/Shared/Vib/Audio) 통합 SSOT (상수/핀맵 100% 복원판)
 * ============================================================================
 * [시스템 구현 원칙 - Core Principles]
 * 1. [SSOT 무결성]: T2_Def(상수), T2_Type(구조체/Enum)으로 도메인과 용도를 엄격 분리.
 * 2. [4-Tier 아키텍처]: Global(인프라), Shared(공통로직), Vib(진동), Audio(소음) 계층화.
 * 3. [익명구조체 금지]: 모든 데이터 모델은 명시적 타입명(ST_...)을 가져야 함.
 * 4. [SIMD 가속 최적화]: 연산용 슬롯(FeatureSlot)은 alignas(16)으로 정렬 (416 Bytes).
 * 5. [패킷 무결성 방어]: PktTelemetry는 SIMD 구조체를 쓰지 않고 평탄화하여 pack(1) 보장.
 * ========================================================================== */

#pragma once

#include <cstdint>

// ============================================================================
// [PART 1] T2_Def: 시스템 전역 상수 (Domain -> Purpose 계층화)
// ============================================================================

namespace T2_Def {

    // ------------------------------------------------------------------------
    // 1. Global: 디바이스 전역 인프라 (System, WiFi, MQTT, Storage)
    // ------------------------------------------------------------------------
    namespace Global {
        namespace System {
            inline constexpr char const* VERSION_STR         = "T240_v243_3";
            inline constexpr char const* SITE_ID_DEF         = "FACTORY_A_LINE_1";
            inline constexpr uint8_t     TELEMETRY_HZ_DEF    = 10;
            inline constexpr uint8_t     WAVEFORM_HZ_DEF     = 0;
            inline constexpr float       MATH_EPSILON_CONST  = 1e-6f;
            inline constexpr float       MATH_EPSILON_12_CONST= 1e-12f;
            inline constexpr float       MS_PER_SEC_CONST    = 1000.0f;
        }
        namespace Hardware {
            inline constexpr uint32_t    SERIAL_BAUD_CONST   = 115200;
            inline constexpr uint8_t     PIN_BTN_CONTROL_CONST = 0;
            inline constexpr uint8_t     PIN_RGB_LED_CONST   = 21;
            inline constexpr uint8_t     PIN_SD_CLK_CONST    = 39;
            inline constexpr uint8_t     PIN_SD_CMD_CONST    = 38;
            inline constexpr uint8_t     PIN_SD_D0_CONST     = 40;
        }
        namespace Task {
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
        namespace Net {
            inline constexpr uint8_t     WIFI_MULTI_MAX_CONST= 3;
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
        }
        namespace Storage {
            inline constexpr uint8_t     PRE_TRIGGER_SEC_DEF = 3;
            inline constexpr uint32_t    ROTATE_MB_DEF       = 10;
            inline constexpr uint32_t    ROTATE_MIN_DEF      = 60;
            inline constexpr uint16_t    ROTATE_KEEP_MAX_DEF = 8;
            inline constexpr uint32_t    IDLE_FLUSH_MS_DEF   = 250;
            
            inline constexpr uint16_t    MAX_PATH_LEN_CONST  = 128;
            inline constexpr uint16_t    MAX_ROTATE_LIST_CONST= 16;
            inline constexpr uint32_t    PREALLOC_BYTES_CONST= 10 * 1024 * 1024; // 10MB
            
            inline constexpr char const* SD_DIR_RAW_CONST    = "/t240_data/raw";
            inline constexpr char const* SD_DIR_BIN_CONST    = "/t240_data/bin";
        }
    }

    // ------------------------------------------------------------------------
    // 2. Shared: 도메인 공유 신호처리 로직 (Trigger, Dsp)
    // ------------------------------------------------------------------------
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
    }

    // ------------------------------------------------------------------------
    // 3. Vib: 진동 특화 파라미터
    // ------------------------------------------------------------------------
    namespace Vib {
        namespace Hardware {
            inline constexpr uint8_t     PIN_BMI_CS_CONST    = 10;
            inline constexpr uint8_t     PIN_BMI_INT1_CONST  = 11;
        }
        namespace Sensor {
            inline constexpr uint32_t    RATE_CONST          = 1600;
            inline constexpr uint32_t    FFT_SIZE_CONST      = 256;
            inline constexpr uint8_t     AXIS_CONST          = 3;
            inline constexpr uint8_t     ACCEL_RANGE_DEF     = 8;
            inline constexpr uint8_t     ACCEL_ODR_DEF       = 0x0C;
            inline constexpr uint8_t     GYRO_RANGE_DEF      = 2;
        }
        namespace Trigger {
            inline constexpr float       RMS_THRESH_DEF      = 0.5f;
            inline constexpr float       WAKE_THRESH_G_DEF   = 1.0f;
        }
        namespace Calib {
            inline constexpr char const* FILE_JSON_CONST     = "/sys/bmi_calib_243.json"; 
        }
    }

    // ------------------------------------------------------------------------
    // 4. Audio: 소음 특화 파라미터
    // ------------------------------------------------------------------------
    namespace Audio {
        namespace Hardware {
            inline constexpr int         I2S_PORT_NUM_CONST  = 0;
            inline constexpr uint8_t     PIN_I2S_BCLK_CONST  = 41;
            inline constexpr uint8_t     PIN_I2S_MCLK_CONST  = 0; 
            inline constexpr uint8_t     PIN_I2S_WS_CONST    = 42;
            inline constexpr uint8_t     PIN_I2S_DIN_CONST   = 43;
            inline constexpr int         DMA_BUF_COUNT_CONST = 16;
        }
        namespace Sensor {
            inline constexpr uint32_t    RATE_CONST          = 42000;
            inline constexpr uint32_t    FFT_SIZE_CONST      = 1024;
            inline constexpr uint8_t     CH_CONST            = 2;
        }
        namespace Trigger {
            inline constexpr float       RMS_THRESH_DEF      = 0.05f;
            inline constexpr uint8_t     MAX_BAND_RMS_CONST  = 8;
        }
        namespace Dsp {
            inline constexpr float       BEAMFORMING_GAIN_DEF= 0.5f;
        }
        namespace Feature {
            inline constexpr uint32_t    MFCC_COEFFS_CONST   = 13;  
            inline constexpr uint32_t    MFCC_DIM_CONST      = 39;  
            inline constexpr uint32_t    MEL_BANDS_CONST     = 26;  
            inline constexpr uint32_t    SEQUENCE_FRAMES_MAX_CONST = 16;
            
            inline constexpr float       MEL_SCALE_2595_CONST= 2595.0f;                
            inline constexpr float       MEL_SCALE_700_CONST = 700.0f;                 
            inline constexpr uint16_t    MAX_PEAK_CANDIDATES_CONST = 128;                    
            inline constexpr uint8_t     TOP_PEAKS_COUNT_CONST = 5;                      
            inline constexpr uint8_t     CEPS_TARGET_COUNT_CONST = 3;    
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

// ============================================================================
// [PART 2] T2_Type: 시스템 구조체 및 열거형 (익명구조체 전면 금지)
// ============================================================================

namespace T2_Type {

    // --- [2.1] 열거형 정의 ---
    enum class SystemState : uint8_t { INIT=0, READY, MONITORING, RECORDING, NOISE_LEARNING, MAINTENANCE, ERROR, CALIBRATING };
    enum class StatusBit : uint8_t { NTP_SYNCED=0, SD_MOUNTED=1, RECORDING_NOW=2, SENSOR_FAULT=3 };
    enum class DetectionResult : uint8_t { PASS=0, RULE_VIB_NG, RULE_AUDIO_NG, TEST_NG, ML_NG };
    enum class SystemCommand : uint8_t { CMD_START=0, CMD_STOP, CMD_LEARN_NOISE, CMD_CALIBRATE, CMD_REBOOT, CMD_OTA_START, CMD_OTA_END, CMD_TUNING_PREVIEW, CMD_TUNING_SAVE, CMD_TUNING_CANCEL };
    enum class TriggerSource : uint8_t { NONE=0, HW_WAKE, SW_RMS, SW_BAND, MANUAL };
    enum class StreamType : uint8_t { TELEMETRY=0x01, SPECTRUM=0x02, WAVEFORM=0x03, CALIBRATION=0x04 };
    enum class NoiseMode : uint8_t { OFF=0, FIXED, ADAPTIVE };
    enum class WiFiMode : uint8_t { STA_ONLY=0, AP_ONLY, AP_STA, AUTO_FALLBACK };
    enum class WindowType : uint8_t { HANN=0, HAMMING, BLACKMAN };

    // --- [2.2] 설정 하위 구조체 (도메인별 분리) ---

    // Global
    struct ST_Global_WiFi { WiFiMode mode; char ap_ssid[32]; char ap_pw[64]; char ap_ip[16]; char multi_ssid[3][32]; char multi_pw[3][64]; };
    struct ST_Global_Mqtt { bool enable; char broker[64]; uint16_t port; char id[32]; char pw[64]; char topic_root[64]; uint8_t qos; };
    struct ST_Global_Storage { uint32_t rot_mb; uint32_t rot_min; bool save_raw; uint16_t keep_max; uint32_t idle_flush_ms; uint8_t pre_trig_sec; };
    struct ST_Global_System { char site_id[32]; uint8_t tele_hz; uint8_t wave_hz; uint8_t op_mode; uint32_t watchdog_ms; };
    struct ST_Global_Output { bool enabled; bool output_sequence; uint16_t sequence_frames; };

    // Shared
    struct ST_FilterFIR { bool en; float cutoff; uint16_t taps; };
    struct ST_FilterIIR { bool en; float cutoff; float q; };
    struct ST_FilterNotch { bool en; float freq; float gain; float q; };
    struct ST_Shared_Trigger { uint32_t hold_ms; bool use_sleep; uint32_t sleep_sec; };
    struct ST_Shared_Dsp { bool rem_dc; bool med_en; uint8_t med_win; ST_FilterFIR hpf; ST_FilterFIR lpf; ST_FilterIIR iir_hpf; ST_FilterIIR iir_lpf; WindowType win_type; ST_FilterNotch notch; };

    // Vib
    struct ST_Vib_Sensor { uint8_t accel_range; uint8_t accel_odr; uint8_t gyro_range; };
    struct ST_Vib_Trigger { bool motion_en; float wake_g; uint16_t wake_dur; float rms_thresh; };
    struct ST_Vib_Calib { float offset[3]; float gain[3]; };
    struct ST_Vib_Feature { float peak_amp_min; float peak_freq_gap_min; };

    // Audio
    struct ST_Audio_Trigger { float rms_thresh; bool band_en[8]; float band_start[8]; float band_end[8]; float band_thresh[8]; };
    struct ST_Audio_Noise { bool gate_en; float gate_thresh; NoiseMode mode; float sub_str; float adp_alpha; uint16_t learn_frames; };
    struct ST_Audio_Dsp { bool pre_en; float pre_alpha; ST_Audio_Noise noise; float beam_gain; };
    struct ST_Audio_Calib { uint32_t auto_idle_min; float ref_freq; float filt_min; float filt_max; float gain_max; float gain_min; float norm_safe; float gain_L; float gain_R; alignas(16) float eq_coeffs[63]; };
    struct ST_Audio_Feature { float ceps_targets[3]; float f_min; float f_max; };

    // --- [2.3] 통합 마스터 설정 (Master Config) ---
    struct DynamicConfig {
        ST_Global_System  system;
        ST_Global_WiFi    wifi;
        ST_Global_Mqtt    mqtt;
        ST_Global_Storage storage;
        ST_Global_Output  output;
        ST_Shared_Trigger shared_trig;
        ST_Shared_Dsp     shared_dsp;
        ST_Vib_Sensor     vib_sensor;
        ST_Vib_Trigger    vib_trig;
        ST_Vib_Calib      vib_calib;
        ST_Vib_Feature    vib_feat;
        ST_Audio_Trigger  aud_trig;
        ST_Audio_Dsp      aud_dsp;
        ST_Audio_Calib    aud_calib;
        ST_Audio_Feature  aud_feat;
    };

    // ============================================================================
    // [PART 3] 고장 진단 통합 데이터 모델 (Memory Chunk 정밀 정렬)
    // ============================================================================
    
    struct SpectralPeak { float freq; float amp; };

    // [Block 1] 공통 헤더 청크 (32 Bytes)
    struct ST_Slot_Header {
        uint64_t ts; uint32_t fid; uint32_t uptime;
        uint8_t flags; int8_t temp; uint8_t trial; uint8_t src; uint8_t _res[12];
    };
    // [Block 2] 진동 특화 지표 청크 (80 Bytes)
    struct ST_Slot_Vib {
        float rms[3]; float peak_f[3]; float centroid[3]; float kurt; float crest; float sta_lta; float skew; float std; 
        float cal_off[3]; float _pad; 
    };
    // [Block 3] 소음 특화 지표 청크 (64 Bytes)
    struct ST_Slot_Audio {
        float rms; float centroid; float coh; float ipd; float d_rms; float dd_rms; float cpsr_max[3]; float cpsr_mxr[3];
        float _pad[3]; 
    };
    // [Block 4] 소음 대역 및 피크 청크 (80 Bytes)
    struct ST_Slot_Bands { float energy[8]; SpectralPeak peaks[5]; float _pad[2]; };
    
    // [Block 5] AI 텐서 청크 (160 Bytes)
    struct ST_Slot_Tensor { alignas(16) float mfcc[39]; float _pad; };

    // 전체 조립: 32 + 80 + 64 + 80 + 160 = 총 416 Bytes (16의 배수 완벽 호환)
    struct alignas(16) UnifiedFeatureSlot {
        ST_Slot_Header header; ST_Slot_Vib vib; ST_Slot_Audio audio; ST_Slot_Bands bands; ST_Slot_Tensor tensor;
    };

    struct alignas(16) UnifiedRawChunk { float vib[3][256]; float audio[1024]; };

    // ============================================================================
    // [PART 4] 통신 및 바이너리 패킷 규격 (Byte Packing)
    // ============================================================================
    #pragma pack(push, 1)

    struct FileHeader {
        char magic[4]; uint16_t ver; uint16_t struct_size;
        uint32_t s_rate; uint16_t fft; uint16_t mfcc_d; uint8_t axes; uint8_t _res;
        uint32_t total; char config_dump[8192]; 
    };

    struct WsHeader { uint8_t magic; uint8_t type; uint16_t len; uint8_t stage; uint8_t _pad[3]; };

    // [중요 교정]: 네트워크 패킷은 SIMD Aligned 구조체를 포함하면 패딩이 깨지므로 
    // 전송 효율을 위해 멤버 변수를 평탄화(Flattening)하여 직접 정의합니다.
    struct PktTelemetry {
        WsHeader header; 
        uint8_t  sys_state; 
        uint8_t  detect_result; 
        uint8_t  trial_no;
        uint8_t  trigger_source;
        
        float    vib_rms[3];
        float    audio_rms;
        float    kurtosis;
        float    spectral_centroid;

        float    crest_factor;   
        float    sta_lta_ratio;  
        float    vib_centroid[3]; 

        float    band_energy[8]; 
        
        float    cpsr_max[3];    
        float    cpsr_mxrms[3];  
        SpectralPeak top_peaks[5]; 

        float    mfcc[39];       
    };

    struct PktSpectrum {
        WsHeader header;
        float    frequencies[513];  
    };

    struct PktWaveform {
        WsHeader header;
        float    samples[1024];
    };

    struct PktCalibration {
        WsHeader header;
        float    target_gain_curve[513];
        float    actual_fir_coeffs[63];
    };

    #pragma pack(pop)
}
