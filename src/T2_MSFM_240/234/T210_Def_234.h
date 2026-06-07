/* ============================================================================
 * File: T210_Def_234.h
 * Summary: T20 MFCC 시스템 통합 정의 (FSM 및 Freq/Time 필터 고도화)
 * Description: 시스템 전역 상수, 데이터 구조체, FSM 상태, 기본 설정 생성 로직 통합
 * ========================================================================== */

#pragma once
#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ----------------------------------------------------------------------------
 * [PART 1] 시스템 전역 상수 (Namespace)
 * ------------------------------------------------------------------------- */
namespace T20 {
    // --- [1.1] 시스템 기본 제어 상수 ---
    namespace C10_Sys {
        inline constexpr char const* VERSION_STR         = "T20_Mfcc_v231";     // 시스템 펌웨어 식별 버전
        inline constexpr uint16_t    QUEUE_LEN           = 8U;                  // RTOS Task 간 메시지 통신 큐 최대 길이
        inline constexpr uint16_t    CFG_PROFILE_COUNT   = 4U;                  // SDMMC 장치 연결 설정 프로필 보존 개수
        inline constexpr uint16_t    RAW_FRAME_BUFFERS   = 4U;                  // DMA 및 센서 수집용 핑퐁 링버퍼(슬롯) 개수
        inline constexpr uint16_t    SEQUENCE_FRAMES_MAX = 16U;                 // TinyML 2D 텐서 조립을 위한 시퀀스 최대 프레임 크기 (버퍼 한계값)
        inline constexpr uint16_t    SEQUENCE_FRAMES_DEF = 16U;                 // 기본 구동 시 시퀀스 프레임 설정값
        inline constexpr uint8_t     PIN_NOT_SET         = 0xFFU;               // 하드웨어 핀 미할당 상태를 나타내는 특수값
        inline constexpr uint32_t    WATCHDOG_MS_DEF     = 2000UL;              // 소프트웨어 워치독 타임아웃 기본값 (2초)
    }

    // --- [1.2] 하드웨어 핀 맵 ---
    namespace C10_Pin {
        inline constexpr uint8_t BTN_CONTROL = 0U;                              // 제어용 사용자 버튼 (일반적으로 Boot 핀 활용)
        inline constexpr uint8_t RGB_LED     = 21U;                             // 상태 인디케이터용 RGB LED 핀

        inline constexpr uint8_t BMI_SCK     = 12U;                             // BMI270 센서 SPI 클럭 핀
        inline constexpr uint8_t BMI_MISO    = 13U;                             // BMI270 센서 SPI MISO 핀
        inline constexpr uint8_t BMI_MOSI    = 11U;                             // BMI270 센서 SPI MOSI 핀
        inline constexpr uint8_t BMI_CS      = 10U;                             // BMI270 센서 SPI Chip Select 핀
        inline constexpr uint8_t BMI_INT1    = 14U;                             // BMI270 센서 하드웨어 인터럽트(FIFO/Any-Motion) 수신 핀

        inline constexpr uint8_t SDMMC_CLK   = 39U;                             // SDMMC 호스트 클럭 핀
        inline constexpr uint8_t SDMMC_CMD   = 38U;                             // SDMMC 호스트 커맨드 핀
        inline constexpr uint8_t SDMMC_D0    = 40U;                             // SDMMC 데이터 0번 핀
        inline constexpr uint8_t SDMMC_D1    = C10_Sys::PIN_NOT_SET;            // SDMMC 데이터 1번 핀 (1-bit 모드 사용 시 미할당)
        inline constexpr uint8_t SDMMC_D2    = C10_Sys::PIN_NOT_SET;            // SDMMC 데이터 2번 핀
        inline constexpr uint8_t SDMMC_D3    = C10_Sys::PIN_NOT_SET;            // SDMMC 데이터 3번 핀
    }

    // --- [1.3] RTOS 태스크 자원 ---
    namespace C10_Task {
        inline constexpr uint32_t SENSOR_STACK   = 6144U;                       // 센서 데이터 수집 태스크 스택 크기
        inline constexpr uint32_t PROCESS_STACK  = 12288U;                      // DSP 및 MFCC 연산 전담 태스크 스택 크기 (메모리 집약)
        inline constexpr uint32_t RECORDER_STACK = 8192U;                       // SD/LittleFS 스토리지 기록 전담 태스크 스택 크기

        inline constexpr uint8_t  SENSOR_PRIO    = 4U;                          // 센서 수집 우선순위 (실시간성 보장을 위해 가장 높음)
        inline constexpr uint8_t  PROCESS_PRIO   = 3U;                          // DSP 연산 우선순위
        inline constexpr uint8_t  RECORDER_PRIO  = 2U;                          // 파일 기록 우선순위 (가장 낮음, 큐 버퍼링으로 커버)
    }

    // --- [1.4] 디지털 신호 처리 (DSP) 및 특징량 설계 ---
    namespace C10_DSP {
        inline constexpr uint16_t FFT_SIZE         = 256U;                      // 기본 고속 푸리에 변환(FFT) 윈도우 크기
        inline constexpr uint16_t FFT_BINS         = (FFT_SIZE / 2U) + 1U;      // Nyquist 정리에 따른 유효 주파수 Bin 개수 (129)
        inline constexpr float    SAMPLE_RATE_HZ   = 1600.0f;                   // 센서 샘플링 및 DSP 연산 기준 주파수 (1.6kHz)
        inline constexpr float    FREQ_RES_HZ      = SAMPLE_RATE_HZ / (float)FFT_SIZE; // 단일 FFT Bin이 나타내는 주파수 대역폭(해상도)

        inline constexpr uint16_t MEL_FILTERS      = 26U;                       // MFCC 추출을 위한 Mel-Filter Bank 개수
        inline constexpr float    MEL_SCALE_CONST  = 2595.0f;                   // Mel 스케일 변환 기본 상수
        inline constexpr float    MEL_FREQ_CONST   = 700.0f;                    // Mel 스케일 기준 주파수 편향값

        inline constexpr uint16_t MFCC_COEFFS_MAX  = 32U;                       // 할당 가능한 최대 MFCC 계수 차원
        inline constexpr uint16_t MFCC_COEFFS_DEF  = 13U;                       // 기본 설정 시 사용되는 MFCC 계수 (일반적 음성/진동 표준)
        inline constexpr uint16_t MFCC_HISTORY_LEN = 5U;                        // Delta/Delta-Delta 계산을 위한 과거 프레임 보존 길이
        inline constexpr uint16_t MFCC_COMPONENTS  = 3U;                        // 특징량 구성 요소 수 (1:Static, 2:Delta, 3:Delta-Delta)
        inline constexpr uint16_t MFCC_TOTAL_DIM   = MFCC_COEFFS_MAX * MFCC_COMPONENTS; // 단일 축 당 최대 특징량 차원 수 (32 * 3 = 96)

        inline constexpr uint8_t  AXIS_COUNT_MAX   = 3U;                        // 동시 연산 가능한 최대 물리 축 개수 (X, Y, Z)
        inline constexpr uint8_t  AXIS_COUNT_DEF   = 3U;                        // 기본 설정 시 사용하는 물리 축 개수
        inline constexpr uint16_t MAX_FEATURE_DIM  = AXIS_COUNT_MAX * MFCC_TOTAL_DIM; // 모든 축을 합산한 최대 1D 특징량 벡터 차원 (3 * 96 = 288)

        inline constexpr uint8_t  TRIGGER_BANDS_MAX= 3U;                        // 주파수 도메인 기반 스마트 트리거 동시 감시 최대 대역 수
        inline constexpr uint16_t SIMD_ALIGN       = 16U;                       // ESP32-S3 벡터 연산(SIMD) 최적화를 위한 16바이트 메모리 정렬 기준
    }

    // --- [1.5] 센서(BMI270) 하드웨어 제어 ---
    namespace C10_BMI {
        inline constexpr uint32_t SPI_FREQ_HZ            = 10000000UL;          // 센서 통신용 SPI 속도 (10MHz)
        inline constexpr uint8_t  REG_CALIB_OFFSET_START = 0x71U;               // 내부 보정값(캘리브레이션) 저장 레지스터 시작 주소
        inline constexpr float    LSB_PER_G              = 2048.0f;             // 16G 측정 범위 설정 시 1G가 나타내는 LSB 단위값
        inline constexpr uint16_t FIFO_FRAME_SIZE        = 12U;                 // 단일 센서 프레임의 바이트 크기 (Accel 6B + Gyro 6B)
        inline constexpr uint16_t FIFO_BATCH_SIZE        = 32U;                 // 인터럽트 발생 시 한 번에 퍼올릴 최대 프레임 개수
        inline constexpr uint32_t ANY_MOTION_STEP_MS     = 20UL;                // 하드웨어 모션 감지(Any-Motion) 내부 타이머의 1틱 기준 시간 (20ms)
    }

    // --- [1.6] 레코더 및 스토리지 관리 ---
    namespace C10_Rec {
        inline constexpr uint32_t BINARY_MAGIC        = 0x54323042UL;           // 고유 바이너리 파일 식별자 ("T20B")
        inline constexpr uint16_t BINARY_VERSION_NUM  = 230U;                   // 바이너리 파일 포맷 버전 (하위 호환성 체크용)
        inline constexpr uint16_t BATCH_WMARK_HIGH    = 8U;                     // 버퍼에 이 개수만큼 프레임이 쌓이면 물리적 디스크 쓰기 트리거
        inline constexpr uint32_t BATCH_IDLE_FLUSH_MS = 250U;                   // 데이터가 워터마크에 도달하지 않아도 강제로 디스크를 비우는 유휴 대기 시간

        inline constexpr uint8_t  FLAG_NTP_SYNCED     = 0x01U;                  // 타임스탬프가 NTP 서버와 정밀 동기화되었음을 나타내는 비트 플래그
        inline constexpr uint8_t  FLAG_TRIGGERED      = 0x02U;                  // 스마트 트리거 이벤트에 의해 기록된 프레임임을 나타내는 비트 플래그

        inline constexpr uint32_t ROTATION_MB_DEF     = 10U;                    // 파일 자동 분할(로테이션) 기준 용량 기본값 (10MB)
        inline constexpr uint32_t ROTATION_MIN_DEF    = 60U;                    // 파일 자동 분할 기준 시간 기본값 (60분)
        inline constexpr uint16_t ROTATE_KEEP_MAX     = 8U;                     // 저장소에 보존할 최근 로테이션 파일 최대 개수
    }

    // --- [1.7] 스마트 트리거 제어 ---
    namespace C10_Trigger {
        inline constexpr float    THRES_RMS_DEF       = 0.5f;                   // 진동/충격 감지를 위한 RMS 임계값 기본값
        inline constexpr uint32_t HOLD_MS             = 5000UL;                 // 이벤트 소멸 후 레코딩 세션을 닫기 전 지연(유지) 시간 (5초)
        inline constexpr uint32_t DURATION_MS_DEF     = 100UL;                  // 센서 칩셋 레벨 모션 감지를 인가하기 위한 최소 진동 지속 시간 (100ms)
        inline constexpr uint32_t SLEEP_SEC_DEF       = 300U;                   // 지정된 시간 동안 이벤트가 없으면 딥슬립으로 진입 (5분)
    }

    // --- [1.8] 웹서버 및 통신 ---
    namespace C10_Web {
        inline constexpr char const* WS_URI              = "/api/t20/ws";       // 실시간 데이터 스트리밍용 WebSocket 엔드포인트
        inline constexpr uint16_t    JSON_BUF_SIZE       = 2048U;               // 일반적인 API 응답 JSON 버퍼 할당 크기
        inline constexpr uint16_t    LARGE_JSON_BUF_SIZE = 8192U;               // 복잡한 시스템 설정 덤프를 위한 확장 JSON 버퍼 크기
        inline constexpr uint32_t    BTN_DEBOUNCE_MS     = 500U;                // 하드웨어 버튼의 물리적 바운싱 현상 무시를 위한 지연 시간
    }

    // --- [1.9] 네트워크 및 MQTT ---
    namespace C10_Net {
        inline constexpr uint8_t  WIFI_MULTI_MAX      = 3U;                     // Auto-Fallback 모드에서 순회할 수 있는 최대 AP 등록 개수
        inline constexpr uint16_t MQTT_PORT_DEF       = 1883U;                  // 표준 MQTT 통신 포트
    }

    // --- [1.10] 시간 정보 시스템 ---
    namespace C10_Time {
        inline constexpr char const* NTP_SERVER_1     = "pool.ntp.org";         // 주 NTP 서버 주소
        inline constexpr char const* NTP_SERVER_2     = "time.nist.gov";        // 보조 NTP 서버 주소
        inline constexpr char const* TZ_INFO          = "KST-9";                // 한국 표준시 타임존 설정 문자열
        inline constexpr uint32_t    SYNC_TIMEOUT_MS  = 5000U;                  // 부팅 시 NTP 동기화 대기 시간 상한 (5초)
    }

    // --- [1.11] 비휘발성 저장소 (NVS) ---
    namespace C10_NVS {
        inline constexpr char const* NAMESPACE        = "t20_sys";              // 시스템 설정 관리를 위한 NVS 전용 네임스페이스
        inline constexpr char const* KEY_FILE_SEQ     = "file_seq";             // 파일명 생성을 위한 시퀀스 번호 키
    }

    // --- [1.12] 내부 스토리지 디렉토리 및 파일 경로 매핑 ---
    namespace C10_Path {
        inline constexpr char const* MOUNT_SD         = "/sdcard";              // SD 카드 마운트 포인트
        inline constexpr char const* DIR_SYS          = "/sys";                 // 시스템 설정 및 내부 인덱스 파일 디렉토리
        inline constexpr char const* DIR_WEB          = "/www";                 // 프론트엔드 정적 웹 리소스 디렉토리
        inline constexpr char const* FILE_CFG_JSON    = "/sys/runtime_cfg_231_012.json"; // 런타임 환경 설정 파일 저장 경로
        inline constexpr char const* FILE_REC_IDX     = "/sys/recorder_index.json";      // 로테이션 파일 관리용 인덱스 맵
        inline constexpr char const* FILE_BMI_CALIB   = "/sys/bmi_calib.json";           // 센서 캘리브레이션 오프셋 백업 파일
        inline constexpr char const* WEB_INDEX        = "index_231_012.html";            // 웹서버 기본 접속 화면 파일명
        inline constexpr char const* SD_DIR_BIN       = "/t20_data/bin";                 // 바이너리 데이터 파일 저장 경로
        inline constexpr char const* SD_PREFIX_BIN    = "/t20_data/bin/rec_";            // 데이터 파일명 생성용 접두어
        inline constexpr char const* DIR_FALLBACK     = "/fallback";                     // SD카드 오류 시 LittleFS 내장 메모리 기록 우회 경로
        inline constexpr char const* SD_DIR_RAW       = "/t20_data/raw";                 // 가공되지 않은 순수 파형 데이터(Raw) 저장 경로
    }
}

/* ----------------------------------------------------------------------------
 * [PART 2] 데이터 구조체 및 열거형 정의
 * ------------------------------------------------------------------------- */

// --- [2.1] 센서 측정 및 물리량 정의 ---
typedef enum {
    EN_T20_AXIS_ACCEL_X = 0,
    EN_T20_AXIS_ACCEL_Y,
    EN_T20_AXIS_ACCEL_Z,
    EN_T20_AXIS_GYRO_X,
    EN_T20_AXIS_GYRO_Y,
    EN_T20_AXIS_GYRO_Z
} EM_T20_SensorAxis_t;

typedef enum {
    EN_T20_ACCEL_2G = 0,
    EN_T20_ACCEL_4G,
    EN_T20_ACCEL_8G,
    EN_T20_ACCEL_16G
} EM_T20_AccelRange_t;

typedef enum {
    EN_T20_GYRO_125 = 0,
    EN_T20_GYRO_250,
    EN_T20_GYRO_500,
    EN_T20_GYRO_1000,
    EN_T20_GYRO_2000
} EM_T20_GyroRange_t;

typedef enum {
    EN_T20_FFT_256  = 256,
    EN_T20_FFT_512  = 512,
    EN_T20_FFT_1024 = 1024,
    EN_T20_FFT_2048 = 2048,
    EN_T20_FFT_4096 = 4096
} EM_T20_FftSize_t;

typedef enum {
    EN_T20_AXIS_SINGLE = 1,
    EN_T20_AXIS_TRIPLE = 3
} EM_T20_AxisCount_t;

// --- [2.2] DSP 전처리 및 동작 모드 열거형 ---
typedef enum {
    EN_T20_WINDOW_HANN = 0,
    EN_T20_WINDOW_HAMMING,
    EN_T20_WINDOW_BLACKMAN,
    EN_T20_WINDOW_FLATTOP,
    EN_T20_WINDOW_RECTANGULAR
} EM_T20_WindowType_t;

typedef enum {
    EN_T20_NOISE_OFF = 0,
    EN_T20_NOISE_FIXED,
    EN_T20_NOISE_ADAPTIVE
} EM_T20_NoiseMode_t;

typedef enum {
    EN_T20_WIFI_STA_ONLY = 0,
    EN_T20_WIFI_AP_ONLY,
    EN_T20_WIFI_AP_STA,
    EN_T20_WIFI_AUTO_FALLBACK
} EM_T20_WiFiMode_t;

typedef enum {
    EN_T20_STORAGE_LITTLEFS = 0,
    EN_T20_STORAGE_SDMMC
} EM_T20_StorageBackend_t;

// --- [2.3] 시스템 상태 및 이벤트 (FSM) 신규 정의 ---

// 1. 시스템 운영 모드 (자동 시작, 수동 시작, 스케줄)
typedef enum {
    EN_T20_OP_MANUAL = 0,
    EN_T20_OP_AUTO,
    EN_T20_OP_SCHEDULE
} EM_T20_OpMode_t;

// 2. 시스템 전역 상태 (FSM States)
typedef enum {
    EN_T20_STATE_INIT = 0,
    EN_T20_STATE_READY,
    EN_T20_STATE_MONITORING,
    EN_T20_STATE_RECORDING,
    EN_T20_STATE_NOISE_LEARNING,
    EN_T20_STATE_ERROR
} EM_T20_SysState_t;

// 3. 메시지 큐 제어 커맨드 (Events)
typedef enum {
    EN_T20_CMD_NONE = 0,
    EN_T20_CMD_START,
    EN_T20_CMD_STOP,
    EN_T20_CMD_LEARN_NOISE,
    EN_T20_CMD_CALIBRATE,
    EN_T20_CMD_REBOOT  // 안전한 비동기 재부팅 명령
} EM_T20_Command_t;

// 4. 레코딩 트리거 발생 소스 (어떤 이유로 기록이 켜졌는지)
typedef enum {
    EN_T20_TRIG_SRC_NONE = 0,
    EN_T20_TRIG_SRC_HW_WAKE,
    EN_T20_TRIG_SRC_SW_RMS,
    EN_T20_TRIG_SRC_SW_BAND_0,
    EN_T20_TRIG_SRC_SW_BAND_1,
    EN_T20_TRIG_SRC_SW_BAND_2,
    EN_T20_TRIG_SRC_MANUAL
} EM_T20_TriggerSource_t;

// 5. 트리거 런타임 제어 컨텍스트 (메모리에만 존재, 설정 파일에 저장되지 않음)
typedef struct {
    bool                   is_triggered;
    EM_T20_TriggerSource_t active_source;
    uint32_t               hold_end_tick;
} ST_T20_TriggerCtx_t;

// --- [2.4] 하위 설정 관리 구조체 ---
typedef struct {
    EM_T20_SensorAxis_t axis;           // 1축 분석 모드 시 메인 타겟이 되는 물리 축
    EM_T20_AccelRange_t accel_range;    // 가속도 센서 측정 한계 범위
    EM_T20_GyroRange_t  gyro_range;     // 자이로 센서 측정 한계 범위
} ST_T20_ConfigSensor_t;

typedef struct {
    bool  enable;                       // 해당 주파수 감시 밴드 활성화 여부
    float start_hz;                     // 감시 시작 주파수
    float end_hz;                       // 감시 종료 주파수
    float threshold;                    // 트리거 유발을 위한 누적 에너지 임계값
} ST_T20_TriggerBand_t;

typedef struct {
    // 1. 하드웨어 전원/모션 제어 (BMI270 칩셋 레벨)
    struct {
        bool     use_deep_sleep;										// 유휴 시 절전 모드 진입 여부
        uint32_t sleep_timeout_sec;										// 딥슬립 진입 대기 시간(초)
        float    wake_threshold_g;     									// 하드웨어 Any-Motion 임계값 (G 단위)
        uint16_t duration_x20ms;       									// 센서 내부 하드웨어 모션 감지용 시간값
    } hw_power;

    // 2. 소프트웨어 이벤트 감시 (DSP 레벨)
    struct {
        bool     use_rms;              									// 진동 기반 스마트 트리거(RMS) 활성화 여부
        uint32_t hold_time_ms;         									// 트리거 소멸 후 레코딩 유지 시간
        float    rms_threshold_power;  									// 전체 진동 파워 임계값
        ST_T20_TriggerBand_t bands[T20::C10_DSP::TRIGGER_BANDS_MAX]; 	// 다중 밴드 감시
    } sw_event;
} ST_T20_ConfigTrigger_t;

// --- [2.5] 시스템 코어 특징량(Feature) 텐서 구조체 ---
// 주의: 고속 벡터 연산(SIMD) 효율을 위해 전체 및 내부 배열을 16바이트 정렬(Alignment) 처리함
typedef struct alignas(T20::C10_DSP::SIMD_ALIGN) {
    uint64_t timestamp_ms;                                              // 정밀 타임스탬프 (8바이트)
    uint32_t frame_id;                                                  // 누적 시퀀스 번호 (4바이트)
    uint8_t  active_axes;                                               // 연산에 포함된 유효 축 수 (1바이트)
    uint8_t  status_flags;                                              // 시스템 상태(NTP 동기화, 트리거 등) 플래그 (1바이트)
    uint8_t  reserved_1[2];                                             // 헤더 구역 16바이트 정렬을 위한 2바이트 패딩

    float    rms[T20::C10_DSP::AXIS_COUNT_MAX];                         // 각 물리축별 실시간 실효치(RMS) 진동값 (12바이트)
    float    band_energy[T20::C10_DSP::TRIGGER_BANDS_MAX];              // 감시 대역 내 파워 스펙트럼 총합 에너지 (12바이트)
    uint8_t  reserved_2[8];                                             // 데이터 구역 시작점(48바이트 오프셋)을 16의 배수로 맞추기 위한 패딩

    // 핵심 특징량 행렬: [물리축][39차원 x 3 (Static, Delta, D-Delta)]
    // float=4바이트이므로 총 3 * 96 * 4 = 1152바이트 (16의 배수로 SIMD 완벽 호환)
    alignas(T20::C10_DSP::SIMD_ALIGN) float features[T20::C10_DSP::AXIS_COUNT_MAX][T20::C10_DSP::MFCC_TOTAL_DIM];
} ST_T20_FeatureVector_t;

typedef struct {
    uint16_t           hop_size;        // FFT 윈도우 슬라이딩 이동 크기 (Overlap 결정)
    uint16_t           mfcc_coeffs;     // 추출할 MFCC 계수 차원 (보통 13)
    EM_T20_FftSize_t   fft_size;        // FFT 연산 프레임 크기
    EM_T20_AxisCount_t axis_count;      // 연산 수행 물리 축 수 (1 또는 3)
} ST_T20_ConfigFeature_t;

// --- [2.6] DSP 전처리(Pre-process) 하위 구조체 ---
typedef struct {
    bool  enable;                       // Pre-emphasis(고역 강조) 적용 여부
    float alpha;                        // 강조 필터 가중치 (일반적으로 0.97 사용)
} ST_T20_PreproEmphasisConfig_t;

// 1. FIR 필터 설정 (신규)
typedef struct {
    bool     enabled;
    float    cutoff_hz;
    uint16_t num_taps;                  // FIR 차수 (예: 64, 128)
} ST_T20_FilterFIRConfig_t;

// 2. IIR 단일 필터 설정 (LPF, HPF 공용)
typedef struct {
    bool  enabled;
    float cutoff_hz;
    float q_factor;
} ST_T20_FilterIIRConfig_t;

// 3. Median Filter (충격성 튀는 노이즈 제거)
typedef struct {
    bool    enabled;
    uint8_t window_size;                // 3, 5, 7 등 (홀수 권장)
} ST_T20_FilterMedianConfig_t;

// 4. Adaptive Notch Filter (특정 주파수 성분 추적 제거)
typedef struct {
    bool  enabled;
    float target_freq_hz;               // 제거 대상 주파수 (예: 60Hz 전원 노이즈)
    float gain;                         // Notch Gain (API 요구 파라미터, 보통 0.0)
    float q_factor;                     // 컷팅 대역폭을 결정하는 Q-Factor (클수록 좁게 파임)
} ST_T20_FilterNotchConfig_t;

typedef struct {
    bool               enable_gate;                 // 특정 진폭 이하 파형을 0으로 묵음처리하는 노이즈 게이트 적용 여부
    float              gate_threshold_abs;          // 노이즈 게이트 절대값 임계치
    EM_T20_NoiseMode_t mode;                        // 적응형 또는 고정형 스펙트럼 감산 노이즈 제거 모드
    float              spectral_subtract_strength;  // 노이즈 감산 시 반영 강도
    float              adaptive_alpha;              // 적응형 노이즈 프로필 학습 속도 (Alpha)
    uint16_t           noise_learn_frames;          // 고정형 모드 시 노이즈 프로필 구성을 위해 취득할 프레임 수
} ST_T20_PreproNoiseConfig_t;

// 전처리 통합 설정 구조체 (신호처리 순서대로 재배치)
typedef struct {
    bool                          remove_dc;
    ST_T20_FilterMedianConfig_t   median;      // [1] 스파이크 제거
    ST_T20_FilterFIRConfig_t      fir_hpf;     // [2-1] FIR 기반 고역 통과 (신규)
    ST_T20_FilterFIRConfig_t      fir_lpf;     // [2-2] FIR 기반 저역 통과 (신규)
    ST_T20_FilterIIRConfig_t      iir_hpf;     // [3-1] IIR 저주파 차단
    ST_T20_FilterIIRConfig_t      iir_lpf;     // [3-2] IIR 고주파 차단
    ST_T20_FilterNotchConfig_t    notch;       // [4] 특정 주파수 제거
    ST_T20_PreproEmphasisConfig_t preemphasis; // [5] 고역 강조
    ST_T20_PreproNoiseConfig_t    noise;       // [6] 스펙트럼 감산
    EM_T20_WindowType_t           window_type; // [7] FFT 윈도우
} ST_T20_PreprocessConfig_t;

// --- [2.7] 네트워크, 통신, 저장 하위 구조체 ---
typedef struct {
    char ssid[32];
    char password[64];
    bool use_static_ip;                 // DHCP 대신 고정 IP 사용 여부
    char local_ip[16], gateway[16], subnet[16], dns1[16], dns2[16];
} ST_T20_WiFiCredential_t;

typedef struct {
    EM_T20_WiFiMode_t       mode;                                       // 무선 통신 모드
    ST_T20_WiFiCredential_t multi_ap[T20::C10_Net::WIFI_MULTI_MAX];     // AP 연결 우선순위 뱅크
    char                    ap_ssid[32];                                // 장비 자체 AP 모드 시 SSID
    char                    ap_password[64];                            // 장비 자체 AP 모드 시 패스워드
    char                    ap_ip[16];                                  // 장비 자체 AP 구동 시 부여할 IP
} ST_T20_ConfigWiFi_t;

typedef struct {
    bool     enable;                    // MQTT 통신 활성화 여부
    char     broker[64];                // 브로커 도메인/IP
    uint16_t port;                      // 브로커 접속 포트
    char     id[16];                    // 디바이스 Client ID
    char     password[16];              // 인증 패스워드
    char     topic_root[64];            // 발행/구독 최상위 루트 토픽 문자열
} ST_T20_ConfigMqtt_t;

typedef struct {
    char    profile_name[32];           // SDMMC 하드웨어 설정 프로필명
    bool    use_1bit_mode;              // D1~D3 미사용(1-bit) 데이터 전송 모드 사용 여부
    uint8_t clk_pin, cmd_pin, d0_pin, d1_pin, d2_pin, d3_pin;
} ST_T20_SdmmcProfile_t;

typedef struct {
    uint32_t magic;                                             // 포맷 인식자 ("T20B")
    uint16_t version;                                           // 구조체 호환성 버전 번호
    uint16_t header_size;                                       // 자기참조를 통한 헤더 크기
    uint32_t sample_rate_hz;                                    // 원본 센서 샘플링 레이트
    uint16_t fft_size;                                          // 기록된 시점의 FFT 크기
    uint16_t mfcc_dim;                                          // 단일 차원의 MFCC 계수 크기
    uint8_t  active_axes;                                       // 데이터 배열의 축 개수
    uint8_t  reserved;                                          // 구조체 패딩
    uint32_t record_count;                                      // 파일에 포함된 데이터 프레임 수
    char     config_dump[T20::C10_Web::JSON_BUF_SIZE];          // 생성 시점의 JSON 설정 전체 덤프 데이터
} ST_T20_RecorderBinaryHeader_t;

typedef struct {
    uint32_t rotation_mb;               // 이 용량을 초과하면 파일을 분할함
    uint32_t rotation_min;              // 이 시간을 초과하면 파일을 분할함
    bool     save_raw;                  // 특징량과 별도로 가공 전 파형 데이터(Raw) 보존 여부
    uint16_t rotate_keep_max;           // 오래된 파일 자동 삭제를 위한 최대 보존 횟수
    uint32_t idle_flush_ms;             // 버퍼가 다 차지 않아도 디스크를 비우는 강제 타임아웃
    uint8_t  pre_trigger_sec;           // 트리거 발생 전 데이터 보존 시간 (초)
} ST_T20_ConfigStorage_t;

// [개편됨] 시스템 제어 구조체 (auto_start 삭제, op_mode 도입)
typedef struct {
    EM_T20_OpMode_t op_mode;            // 부팅 시 가동 방식 (수동, 자동, 스케줄)
    uint8_t         button_pin;         // 시스템 제어 입력 핀
    uint32_t        watchdog_ms;        // 센서 먹통 감지 등을 위한 시스템 감시 타이머
} ST_T20_ConfigSystem_t;

typedef struct {
    bool     enabled;                   // 외부 출력 기능 자체 활성 여부
    bool     output_sequence;           // 참이면 16프레임을 텐서로 묶어 전송, 거짓이면 1프레임 즉시 전송
    uint16_t sequence_frames;           // 시퀀스 묶음 프레임 수치
} ST_T20_ConfigOutput_t;

// --- [2.8] 시스템 통합 관리 (Master Config) 구조체 ---
typedef struct {
    ST_T20_PreprocessConfig_t preprocess;
    ST_T20_ConfigSensor_t     sensor;
    ST_T20_ConfigWiFi_t       wifi;
    ST_T20_ConfigMqtt_t       mqtt;
    ST_T20_ConfigFeature_t    feature;
    ST_T20_ConfigOutput_t     output;
    ST_T20_ConfigStorage_t    storage;
    ST_T20_ConfigTrigger_t    trigger;
    ST_T20_ConfigSystem_t     system;
} ST_T20_Config_t;

/* ----------------------------------------------------------------------------
 * [PART 3] 기본 설정 빌더 팩토리
 * ------------------------------------------------------------------------- */

/**
 * @brief 런타임 설정 파일(JSON)이 없거나 초기화될 때 호출되어 논리적 기본값을 할당합니다.
 * @return ST_T20_Config_t 정합성이 보장된 초기화 설정 구조체
 */
static inline ST_T20_Config_t T20_makeDefaultConfig() {
    ST_T20_Config_t cfg;
    memset(&cfg, 0, sizeof(cfg));

     // [1] 전처리(DSP) 파이프라인 신규 설정 적용
    cfg.preprocess.remove_dc              = true;
    cfg.preprocess.window_type            = EN_T20_WINDOW_HANN;

    // Median Filter 기본값
    cfg.preprocess.median.enabled         = false;
    cfg.preprocess.median.window_size     = 3;
    
    // FIR 필터 기본값 세팅 (기본 비활성)
    cfg.preprocess.fir_hpf.enabled        = false;
    cfg.preprocess.fir_hpf.cutoff_hz      = 15.0f;
    cfg.preprocess.fir_hpf.num_taps       = 64;

    cfg.preprocess.fir_lpf.enabled        = false;
    cfg.preprocess.fir_lpf.cutoff_hz      = 750.0f;
    cfg.preprocess.fir_lpf.num_taps       = 64;

    // IIR HPF (15Hz 이하 제거)
    cfg.preprocess.iir_hpf.enabled        = true;
    cfg.preprocess.iir_hpf.cutoff_hz      = 15.0f;
    cfg.preprocess.iir_hpf.q_factor       = 0.707f;

    // IIR LPF (800Hz 이상 차단 - Nyquist)
    cfg.preprocess.iir_lpf.enabled        = false;
    cfg.preprocess.iir_lpf.cutoff_hz      = 750.0f;
    cfg.preprocess.iir_lpf.q_factor       = 0.707f;

    // Adaptive Notch (60Hz 전원 노이즈 대비) 기본값 세팅 수정
    cfg.preprocess.notch.enabled          = false;
    cfg.preprocess.notch.target_freq_hz   = 60.0f;
    cfg.preprocess.notch.gain             = 0.0f;  
    cfg.preprocess.notch.q_factor         = 10.0f; 

    cfg.preprocess.preemphasis.enable     = true;
    cfg.preprocess.preemphasis.alpha      = 0.97f;

    cfg.preprocess.noise.enable_gate      = true;
    cfg.preprocess.noise.gate_threshold_abs = 0.002f;
    cfg.preprocess.noise.mode             = EN_T20_NOISE_ADAPTIVE;
    cfg.preprocess.noise.spectral_subtract_strength = 1.0f;
    cfg.preprocess.noise.adaptive_alpha   = 0.05f;
    cfg.preprocess.noise.noise_learn_frames = 8U;

    // [2] 센서 물리 한계치 및 메인 축 설정
    cfg.sensor.axis                                 = EN_T20_AXIS_ACCEL_Z;
    cfg.sensor.accel_range                          = EN_T20_ACCEL_8G;
    cfg.sensor.gyro_range                           = EN_T20_GYRO_2000;

    // [3] 무선 통신망 폴백 모드 및 AP 인프라 기본 세팅
    cfg.wifi.mode                                   = EN_T20_WIFI_AUTO_FALLBACK;
    strlcpy(cfg.wifi.ap_ssid						, "T20_MFCC_AP", 32);
    strlcpy(cfg.wifi.ap_password					, "12345678", 64);
    strlcpy(cfg.wifi.ap_ip							, "192.168.4.1", 16);

    // [4] MQTT 통신 (기본 비활성)
    cfg.mqtt.enable                                 = false;
    strlcpy(cfg.mqtt.broker							, "broker.hivemq.com", 64);
    cfg.mqtt.port                                   = T20::C10_Net::MQTT_PORT_DEF;
    strlcpy(cfg.mqtt.id								, "T20_DEVICE", 16);
    strlcpy(cfg.mqtt.topic_root						, "t20/sensor", 64);

    // [5] 특징량(Feature) 차원 및 해상도 매핑
    cfg.feature.fft_size                            = (EM_T20_FftSize_t)T20::C10_DSP::FFT_SIZE;
    cfg.feature.hop_size                            = T20::C10_DSP::FFT_SIZE; 
    cfg.feature.mfcc_coeffs                         = T20::C10_DSP::MFCC_COEFFS_DEF;
    cfg.feature.axis_count                          = (EM_T20_AxisCount_t)T20::C10_DSP::AXIS_COUNT_DEF;

    // [6] 외부 출력 파이프라인 제어
    cfg.output.sequence_frames                      = T20::C10_Sys::SEQUENCE_FRAMES_DEF;
    cfg.output.enabled                              = true;
    cfg.output.output_sequence                      = false; 

    // [7] 영구 스토리지 보존 및 파일 로테이션 정책
    cfg.storage.save_raw                            = false;
    cfg.storage.rotation_mb                         = T20::C10_Rec::ROTATION_MB_DEF;
    cfg.storage.rotation_min                        = T20::C10_Rec::ROTATION_MIN_DEF;
    cfg.storage.rotate_keep_max                     = T20::C10_Rec::ROTATE_KEEP_MAX;
    cfg.storage.idle_flush_ms                       = T20::C10_Rec::BATCH_IDLE_FLUSH_MS;
    cfg.storage.pre_trigger_sec                     = 4;

    // [8] 스마트 트리거 제어 기반 절전 및 이벤트 레코딩 정책
    cfg.trigger.hw_power.use_deep_sleep      		= false;
    cfg.trigger.hw_power.sleep_timeout_sec   		= T20::C10_Trigger::SLEEP_SEC_DEF;
    cfg.trigger.hw_power.wake_threshold_g    		= 1.0f; 
    cfg.trigger.hw_power.duration_x20ms      		= 5;    

    cfg.trigger.sw_event.hold_time_ms        		= 5000UL; 
    cfg.trigger.sw_event.use_rms             		= false;
    cfg.trigger.sw_event.rms_threshold_power 		= T20::C10_Trigger::THRES_RMS_DEF; 

    for (int i = 0; i < T20::C10_DSP::TRIGGER_BANDS_MAX; i++) {
        cfg.trigger.sw_event.bands[i].enable    	= false;
        cfg.trigger.sw_event.bands[i].start_hz  	= 0.0f;
        cfg.trigger.sw_event.bands[i].end_hz    	= 0.0f;
        cfg.trigger.sw_event.bands[i].threshold 	= 0.0f;
    }

    // [9] 시스템 구동 특성 (FSM Operation Mode 변경 반영)
    cfg.system.op_mode                              = EN_T20_OP_AUTO;
    cfg.system.button_pin                           = T20::C10_Pin::BTN_CONTROL;
    cfg.system.watchdog_ms                          = T20::C10_Sys::WATCHDOG_MS_DEF;

    return cfg;
}

