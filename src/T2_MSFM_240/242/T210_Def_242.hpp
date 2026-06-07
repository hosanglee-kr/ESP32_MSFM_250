/* ============================================================================
 * File: T210_Def_242.hpp
 * Summary: T240 멀티모달(진동+소음) 시스템 전역 SSOT(Single Source of Truth)
 * ============================================================================
 * [구현 원칙]
 * 1. [네임스페이스]: T240_Config(상수), T240_Type(구조체/Enum)으로 엄격 분리.
 * 2. 배열크기등의 고정상수명은 _CONST 접미사, 기본깂 등은 _DEF 접미사 적용
 * 2. [SIMD 정렬]: 연산용 텐서 및 청크는 반드시 alignas(16) 적용 (Rule #23).
 * 3. [바이너리 무결성]: 네트워크 패킷은 #pragma pack(1)로 패딩 제거 (Rule #37).
 * 4. [교정 완료]: 누락되었던 FIR 계수 배열 및 이종 센서 통합 청크 규격 복원.
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
		inline constexpr uint32_t RATE_VIB_CONST			= 1600;	  // 진동 샘플링 레이트
		inline constexpr uint32_t RATE_AUDIO_CONST			= 42000;  // 오디오 샘플링 레이트

		inline constexpr uint32_t FFT_SIZE_VIB_CONST		= 256;	 // 진동 FFT 크기
		inline constexpr uint32_t FFT_SIZE_AUDIO_CONST		= 1024;	 // 오디오 FFT 크기

		inline constexpr uint32_t VIB_AXIS_CONST			= 3;  // X, Y, Z
		inline constexpr uint32_t AUDIO_CH_CONST			= 2;  // L, R (Beamforming 전)

		inline constexpr uint32_t MFCC_COEFFS_CONST			= 13;  // 기본 MFCC 차원
		inline constexpr uint32_t MFCC_DIM_CONST			= 39;  // 13 + delta + delta-delta
		inline constexpr uint32_t MEL_BANDS_CONST			= 26;  // 필터뱅크 개수

		inline constexpr uint32_t SEQUENCE_FRAMES_MAX_CONST = 16;	   // AI 시퀀스 길이
		inline constexpr float	  MATH_EPSILON_12_CONST		= 1e-12f;  // NaN 방어용
		
		inline constexpr uint8_t TELEMETRY_HZ_DEF       = 10;
        inline constexpr uint8_t WAVEFORM_HZ_DEF        = 0;    // 기본 오프 (대역폭 방어)
        inline constexpr char const* SITE_ID_DEF        = "FACTORY_A_LINE_1";

	}

	namespace Hardware {
		inline constexpr uint32_t SERIAL_BAUD_CONST		    = 115200;
		inline constexpr uint8_t  PIN_BTN_CONTROL_CONST     = 0;  // 메인 트리거 핀

		// BMI270 (SPI)
		inline constexpr uint8_t  PIN_BMI_CS_CONST		    = 10;
		inline constexpr uint8_t  PIN_BMI_INT1_CONST        = 11; // [복원] Any-motion용
        

		// ICS43434 (I2S)
		inline constexpr int	  I2S_PORT_NUM_CONST	    = 0;

		// ICS43434 I2S 핀맵 추가
        inline constexpr uint8_t  PIN_I2S_BCLK_CONST        = 41;
        inline constexpr uint8_t  PIN_I2S_MCLK_CONST        = 0; 
        inline constexpr uint8_t  PIN_I2S_WS_CONST          = 42;
        inline constexpr uint8_t  PIN_I2S_DIN_CONST         = 43;

	} 
	
	namespace Path {
		inline constexpr char const* MOUNT_SD_CONST		    = "/sdcard";
		inline constexpr char const* FILE_CFG_JSON_CONST    = "/sys/runtime_cfg_240.json";
		inline constexpr char const* SD_DIR_RAW_CONST	    = "/t240_data/raw";
        inline constexpr char const* FILE_BMI_CALIB_CONST   = "/sys/bmi_calib_242.json";	 //진동 캘리브레이션 저장 파일 경로 추가
        inline constexpr char const* FILE_AUD_CALIB_CONST   = "/sys/aud_calib_242.json"; // 마이크 캘리브레이션 저장
        inline constexpr char const* SD_DIR_BIN_CONST       = "/t240_data/bin";			// 스토리지 엔진이 바이너리를 저장할 기본 폴더 경로
    }

	namespace Dsp {
		inline constexpr int   MEDIAN_WINDOW_DEF	    = 3;
		inline constexpr float NOTCH_FREQ_HZ_DEF	    = 60.0f;
		inline constexpr float NOTCH_Q_FACTOR_DEF	    = 30.0f;
		inline constexpr float FIR_HPF_CUTOFF_DEF	    = 10.0f;
		inline constexpr float PRE_EMPHASIS_ALPHA_DEF   = 0.97f;
		inline constexpr float BEAMFORMING_GAIN_DEF	    = 0.5f;
		
        // 윈도우 함수 적용에 따른 에너지 손실 보정 계수 (Hann = 2.0, Hamming = 1.85)
        inline constexpr float WINDOW_AMPLITUDE_CORR_HANN   = 2.0000f; 
        inline constexpr float WINDOW_ENERGY_CORR_HANN      = 1.6330f;
    }

	namespace Trigger {
		inline constexpr uint32_t HOLD_TIME_MS_DEF		= 5000;
		inline constexpr bool	  USE_DEEP_SLEEP_DEF	= false;
		inline constexpr uint32_t SLEEP_TIMEOUT_SEC_DEF = 300;
		inline constexpr float	  WAKE_THRESH_G_DEF		= 1.0f;
		inline constexpr uint16_t WAKE_DURATION_DEF		= 5;
	}

	// [4] 특징량 및 알고리즘 제한 (_CONST)
	namespace FeatureLimit {
		inline constexpr uint8_t  MAX_BAND_RMS_CONST    = 8;	// 밴드 에너지 분할 개수
		inline constexpr uint16_t FIR_TAPS_CONST	    = 63;	// FIR 필터 탭 수
	}  // namespace FeatureLimit

	namespace Task {
		inline constexpr uint32_t WDG_TIMEOUT_MS_CONST  = 2000;

		// 스토리지 백그라운드 태스크 제어 상수
        inline constexpr uint32_t STORAGE_STACK_CONST   = 8192; // 8KB 스택 여유
        inline constexpr uint8_t  STORAGE_PRIO_CONST    = 2;    // 연산(Process)보다 낮게 배정
        inline constexpr uint8_t  CORE_PROCESS_CONST    = 1;    // Core 1 (APP_CPU)
	}

	namespace Storage {
		inline constexpr uint8_t  PRE_TRIGGER_SEC_DEF   = 3;
		inline constexpr uint32_t ROTATE_MB_DEF		    = 10;
		inline constexpr uint32_t ROTATE_MIN_DEF	    = 60;
		inline constexpr uint16_t ROTATE_KEEP_MAX_DEF   = 8;
		inline constexpr uint32_t IDLE_FLUSH_MS_DEF	    = 250;
	}  // namespace Storage

	// [5] 저장소 제약 (_CONST)
	namespace StorageLimit {
		inline constexpr uint16_t MAX_PATH_LEN_CONST	= 128;
		inline constexpr uint16_t MAX_ROTATE_LIST_CONST = 16;
		inline constexpr uint32_t PREALLOC_BYTES_CONST	= 10 * 1024 * 1024;	 // 10MB 선할당
	}

	// [6] 동적 설정 초기 기본값 (_DEF)
	namespace Decision {
		inline constexpr float RULE_VIB_RMS_THRESH_DEF	 = 0.5f;
		inline constexpr float RULE_AUDIO_RMS_THRESH_DEF = 0.05f;
	} 

	// [3] 네트워크 및 통신 상수 (_CONST)
	namespace Net {
		inline constexpr uint8_t	 WIFI_MULTI_MAX_CONST  = 3;
		inline constexpr uint16_t	 MQTT_PORT_CONST	   = 1883;
		inline constexpr char const* WS_URI_CONST		   = "/api/t240/ws";
		inline constexpr char const* TZ_INFO_CONST		   = "KST-9";
		inline constexpr char const* NTP_SERVER_1_CONST	   = "pool.ntp.org";
		inline constexpr uint32_t	 SYNC_TIMEOUT_MS_CONST = 5000;
		
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
        NTP_SYNCED    = 0, // 비트 0: 시간 동기화 완료 여부
        SD_MOUNTED    = 1, // 비트 1: 저장소 정상 인식 여부
        RECORDING_NOW = 2, // 비트 2: 세션 기록 중 여부
        SENSOR_FAULT  = 3  // 비트 3: 센서 통신 에러 발생 여부
    };


	enum class DetectionResult : uint8_t {
		PASS,
		RULE_VIB_NG,
		RULE_AUDIO_NG
	};


	enum class SystemCommand : uint8_t {
		CMD_START = 0,
		CMD_STOP,
		CMD_LEARN_NOISE,
		CMD_CALIBRATE,
		CMD_REBOOT
	};

	enum class NoiseMode : uint8_t {
		OFF = 0,
		FIXED,
		ADAPTIVE
	};
	enum class WiFiMode : uint8_t {
		STA_ONLY = 0,
		AP_ONLY,
		AP_STA,
		AUTO_FALLBACK
	};
	enum class WindowType : uint8_t {
		HANN = 0,
		HAMMING,
		BLACKMAN
	};

	// --- 설정 하위 구조체 ---
	struct ST_Config_Sensor {
		uint8_t accel_range;  // BMI270 G-Range (2, 4, 8, 16)
		uint8_t accel_odr;    // ODR 설정 (0x08: 100Hz ~ 0x0C: 1600Hz)
		uint8_t gyro_range;
	};

	struct ST_Config_Trigger {
		uint32_t hold_time_ms;
		bool	 use_deep_sleep;
		uint32_t sleep_timeout_sec;
		bool     motion_en;     // 인터럽트 활성화 스위치
		float	 wake_thresh_g;
		uint16_t wake_duration;
		float	 vib_rms_thresh;
		float	 audio_rms_thresh;
		bool	 band_enable[8];
		float	 band_start_hz[8];
		float	 band_end_hz[8];
		float	 band_thresh[8];
	};

	struct ST_Config_FilterFIR {
		bool	 enabled;
		float	 cutoff_hz;
		uint16_t num_taps;
	};

	struct ST_Config_FilterIIR {
		bool  enabled;
		float cutoff_hz;
		float q_factor;
	};

	struct ST_Config_FilterNotch {
		bool  enabled;
		float target_freq_hz;
		float gain;
		float q_factor;
	};

	struct ST_Config_Noise {
		bool	  enable_gate;
		float	  gate_threshold_abs;
		NoiseMode mode;
		float	  spectral_subtract_strength;
		float	  adaptive_alpha;
		uint16_t  noise_learn_frames;
	};

	struct ST_Config_Dsp {
		bool				  remove_dc;
		bool				  median_enabled;
		uint8_t				  median_window;
		ST_Config_FilterFIR	  fir_hpf;
		ST_Config_FilterFIR	  fir_lpf;
		ST_Config_FilterIIR	  iir_hpf;
		ST_Config_FilterIIR	  iir_lpf;
		ST_Config_FilterNotch notch;
		bool				  preemphasis_enable;
		float				  preemphasis_alpha;
		ST_Config_Noise		  noise;
		WindowType			  window_type;
		float				  beamforming_gain;
		float				  calib_gain_L;
		float				  calib_gain_R;
		alignas(16) float     calib_eq_coeffs[T2_Def::FeatureLimit::FIR_TAPS_CONST];
	};


	// 캘리브레이터 제어 파라미터 (T415 원본 복원)
	struct ST_Config_Calib {
		uint32_t auto_idle_min;
		float	 ref_freq_hz;
		float	 filter_min_freq_hz;
		float	 filter_max_freq_hz;
		float	 target_gain_max;
		float	 target_gain_min;
		float	 norm_safe_thresh;
	};

	// 특징량 추출기 튜닝 파라미터 (T415 원본 복원)
	struct ST_Config_Feature {
		float	 ceps_targets[3];
		float	 spatial_freq_min_hz;
		float	 spatial_freq_max_hz;
		float	 peak_amplitude_limit_min;
		float	 peak_freq_gap_limit_hz_min;
	};

	struct ST_Config_Storage {
		uint32_t rotation_mb;
		uint32_t rotation_min;
		bool	 save_raw;
		uint16_t rotate_keep_max;
		uint32_t idle_flush_ms;
		uint8_t	 pre_trigger_sec;
	};

	struct ST_Config_WiFi {
		WiFiMode mode;
		char	 ap_ssid[32];
		char	 ap_password[64];
		char	 ap_ip[16];
		char	 multi_ssid[3][32];
		char	 multi_pw[3][64];
	};
	
	struct ST_Config_Mqtt {
		bool	 enable;
		char	 broker[64];
		uint16_t port;
		char	 id[32];      
		char	 password[64];
		char	 topic_root[64];
		uint8_t	 qos;
	};
	
	// 공정 메타데이터 및 스트리밍 제어 구조체
    struct ST_Config_System {
        char    site_id[32];
        uint8_t telemetry_hz;
        uint8_t waveform_hz;
    };

	struct DynamicConfig {
		ST_Config_System  system; 
		ST_Config_Sensor  sensor;
		ST_Config_Trigger trigger;
		ST_Config_Dsp	  dsp;
		ST_Config_Calib   calib;
		ST_Config_Feature feature;
		ST_Config_Storage storage;
		ST_Config_WiFi	  wifi;
		ST_Config_Mqtt	  mqtt;
		uint8_t			  op_mode;
		uint32_t		  watchdog_ms;
	};


	// --- 고장 진단 통합 데이터 규격 ---
	// 가시성 에러 해결을 위해 통합 헤더로 전진 배치 (SIMD 16B Aligned)
	struct alignas(16) UnifiedFeatureSlot {
		uint64_t timestamp;			// Epoch Time
		uint32_t frame_id;			// Sequence ID
		uint32_t uptime_ms;         // 시스템 가동 시간 (로그 추적용)
        
		uint8_t	 status_flags;		// NTP sync, Trigger source 등
		int8_t   internal_temp;     // 센서 내부 온도 (바이어스 보정용)
		uint8_t	 _reserved[2];

		float	 vib_rms[3];		// 진동 RMS (X, Y, Z)
		float	 vib_peak_freq[3];	// 진동 최대 피크 주파수 (X, Y, Z)
		float    vib_centroid[3];   // 진동 주파수 에너지 중심
		
		float	 audio_rms;			// Beamformed Audio RMS
		float	 spectral_centroid; // 주파수 중심(고음역대인지 저음역대인지 판단)
		float	 kurtosis;		  	// 첨도 (충격성 이상 탐지)

		// 베어링 초기 결함 감지용 핵심 지표
        float    crest_factor;		// 파고율
        float    sta_lta_ratio;		// 급격한 변화(단기평균 / 장기 평균)
        float    vib_skewness;      // 비대칭 충격성 지표 (기어 결함용)

		float	 band_energy[8];  	// 주파수 밴드별 에너지 합산

		alignas(16) float mfcc[39];	 // TinyML 모델 입력 텐서
	};

	// 스토리지 기록용 하이브리드 Raw 데이터 규격 (SIMD 16B Aligned)
	struct alignas(16) UnifiedRawChunk {
		float vib[3][256];			// 진동 1.6kHz Raw
		float audio[1024];			// 오디오 42kHz Raw (Mono/Beamformed)
	};

	// --- 통신 패킷 송출 전용 규격 (Byte Packing 강제) ---
	#pragma pack(push, 1)

	struct FileHeader {
        char     magic[4];         // "SMEA"
        uint16_t version;          // 240
        uint16_t struct_size;      // 데이터 오염 검증용
        uint64_t total_records;    // [교정] _reserved를 레코드 수 기록용으로 변경
    };

	struct WsHeader {
		uint8_t	 magic;	  // 0x54 ('T')
		uint8_t	 type;	  // 0: Telemetry, 1: Spectrum, 2: Waveform
		uint16_t length;  // 가변 페이로드 대응
		uint8_t	 stage;	  // 처리 단계 표식
		uint8_t	 _pad[3];
	};

	struct PktTelemetry {
        WsHeader header;
        uint8_t  sys_state;
        uint8_t  detect_result;
        uint8_t  _pad[2];
        
        float    vib_rms[3];
        float    audio_rms;
        float    kurtosis;
        float    spectral_centroid;

        float    crest_factor;   // 파고율
        float    sta_lta_ratio;  // 급격한 변화(단기평균 / 장기 평균)
        float    vib_centroid[3]; // 패킷 송출 지표 추가


        float    band_energy[8]; // 주파수 밴드별 에너지 합산
        float    mfcc[39];		 // TinyML 모델 입력 텐서
    };

	struct PktSpectrum {
		WsHeader header;
		float	 frequencies[513];	// (1024/2) + 1
	};

	struct PktWaveform {
		WsHeader header;
		float	 samples[1024];
	};

	#pragma pack(pop)
}




