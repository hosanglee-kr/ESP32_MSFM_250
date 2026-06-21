

/* ============================================================================
 * File: T210_Def_250.hpp
 * Summary: 4-Tier 시스템 상수 정의 및 동적 파라미터 SSOT
 * ============================================================================
 * [구현 원칙 및 아키텍처 가이드라인]
 * 1. [도메인 완벽 격리]: Accel, Gyro, Audio의 주율(Hz) 및 윈도우 크기를 독립 상수로 분리.
 * 2. [IMU 공통 인프라]: BMI270 통신 및 FIFO 등 하드웨어 의존성은 Imu 네임스페이스로 통합.
 * 3. [공통 Shared 제거]: 각 센서별 특화된 FIR 차수, 대역 수, 필터 설정을 개별 할당.
 * 4. [AI 텐서 동적화]: 전체 채널 수(3+3+2=8)를 반영한 텐서 차원 컴파일 타임 계산식 적용.
 * 5. [매직넘버 제로]: 하드코딩 금지. 모든 임계치와 기본값은 이 파일(SSOT)을 통제소로 함.
 * ========================================================================== */

#pragma once

#include <cstdint>
#include <cmath>
#include <algorithm>
#include <atomic>
#include "esp_attr.h"
#include "esp_timer.h"
#include "esp_rom_crc.h"


// 실행 코드를 플래시 ROM에서 직접 페치 (필터 계수 테이블용)
#define SMEA_FLASH_RODATA __attribute__((section(".rodata")))
// 핫 버퍼 강제 내부 SRAM 상주 (사전 매핑 테이블 및 핵심 상태 버퍼용)
#define SMEA_SRAM_ATTR    DRAM_ATTR
// 16바이트 벡터 정렬 강제
#define SMEA_ALIGN_16     __attribute__((aligned(16)))

// esp-dsp 버전 체크용 매크로 (v1.8.2)
#define ESP_DSP_VERSION_CHECK_VAL 10802

// 192바이트 규모의 정적 인덱스 룩업 테이블 (내부 SRAM 상주)
extern const uint16_t g_BandBinMap[192] DRAM_ATTR;


// FPU NaN/Inf 및 안전 연산 세이프티 매크로
#define SMEA_IS_NAN(v) (__builtin_isnan(v))
#define SMEA_IS_INF(v) (__builtin_isinf(v))
#define SMEA_SAN_FLOAT(v) ((SMEA_IS_NAN(v) || SMEA_IS_INF(v)) ? 0.0f : (v))
#define SMEA_CLAMP_FLOAT(v, min_v, max_v) (std::clamp(SMEA_SAN_FLOAT(v), (min_v), (max_v)))
#define SMEA_SAFE_DIV(num, denom, eps) ((fabsf(denom) < (eps)) ? ((num) / (eps)) : ((num) / (denom)))

namespace T2_Def {

// ========================================================================
// 1. Global: 전역 디바이스 인프라 및 통신/저장/공통 규칙
// ========================================================================
namespace Global {
    namespace System {
        inline constexpr char const* VERSION_STR           = "MSFM_T2_250";       // 시스템 펌웨어 식별 버전
        inline constexpr char const* SITE_ID_DEF           = "FACTORY_A_LINE_1";  // 기본 설치 현장 및 설비 식별자
        inline constexpr uint16_t    SITE_ID_LEN_MAX       = 32;                  // [이슈 2] 현장 식별자 최대 길이 (매직넘버 제거)

        inline constexpr uint8_t     TELEMETRY_HZ_DEF      = 10;                  // 상태 및 특징량 데이터 전송 주기 (Hz)
        inline constexpr uint8_t     WAVEFORM_HZ_DEF       = 0;                   // 원시 파형 데이터 전송 주기 (Hz)

        inline constexpr uint16_t    SEQUENCE_FRAMES_MAX   = 32;                  // AI 텐서 시퀀스 최대 프레임 수
        inline constexpr uint16_t    SEQUENCE_FRAMES_DEF   = 16;                  // AI 텐서 시퀀스 기본 프레임 수

        inline constexpr float       MATH_EPSILON_CONST    = 1e-6f;               // 부동소수점 0 나누기 방어용 최소값
        inline constexpr float       MATH_EPSILON_12_CONST = 1e-12f;              // 로그/제곱근 0 진입 방어용 고정밀 최소값
        inline constexpr float       MS_PER_SEC_CONST      = 1000.0f;             // 초 단위 밀리초 역산 계수
        inline constexpr uint32_t    BYTES_PER_MB_CONST    = 1024 * 1024;         // 메가바이트 역산 계수
        inline constexpr uint32_t    MS_PER_MIN_CONST      = 60 * 1000;           // 분 단위 밀리초 역산 계수
    }  // namespace System

    namespace Hardware {
        inline constexpr uint32_t    SERIAL_BAUD_CONST     = 115200;              // 디버깅용 시리얼 보드레이트
        inline constexpr uint8_t     PIN_BTN_CONTROL_CONST = 0;                   // 제어용 사용자 물리 버튼 핀
        inline constexpr uint8_t     PIN_RGB_LED_CONST     = 21;                  // 상태 표시용 RGB LED 핀

        inline constexpr uint8_t     PIN_SD_CLK_CONST      = 39;                  // SD카드 SD_MMC 클럭 핀
        inline constexpr uint8_t     PIN_SD_CMD_CONST      = 38;                  // SD카드 SD_MMC 커맨드 핀
        inline constexpr uint8_t     PIN_SD_D0_CONST       = 40;                  // SD카드 SD_MMC 데이터 0번 핀
        inline constexpr uint8_t     PIN_NOT_SET_CONST     = 0xFF;                // 핀 미할당 방어값
        inline constexpr uint8_t     PIN_SAFETY_RELAY_CONST = 25;                  // 프리엠프티브 릴레이 제어 핀
    }  // namespace Hardware

    namespace Task {
        inline constexpr uint16_t QUEUE_LEN_MAX            = 16;                  // FSM 메시지 큐 최대 길이
        inline constexpr uint16_t QUEUE_LEN_DEF            = 8;                   // FSM 메시지 큐 기본 길이
        inline constexpr uint32_t WDG_TIMEOUT_MS_DEF       = 2000;                // 워치독 타임아웃 기본값
        inline constexpr uint32_t STORAGE_STACK_MAX        = 16384;               // 스토리지 태스크 스택
        inline constexpr uint32_t STORAGE_STACK_DEF        = 8192;                // 스토리지 태스크 기본 할당량
        inline constexpr uint8_t  STORAGE_PRIO_DEF         = 2;                   // 스토리지 우선순위
        inline constexpr uint8_t  CORE_CAPTURE_DEF         = 0;                   // 센서 데이터 수집 태스크 할당 코어
        inline constexpr uint8_t  CORE_PROCESS_DEF         = 1;                   // 메인 신호처리 태스크 할당 코어

        inline constexpr uint32_t IMU_ACQ_STACK_SIZE       = 4096;                // IMU 수집 태스크 스택 크기
        inline constexpr uint32_t AUD_PROC_STACK_SIZE      = 8192;                // 오디오 처리 태스크 스택 크기
        inline constexpr uint32_t VIB_PROC_STACK_SIZE      = 8192;                // 진동 처리 태스크 스택 크기
        inline constexpr uint8_t  IMU_ACQ_PRIORITY         = 12;                  // IMU 수집 태스크 우선순위
        inline constexpr uint8_t  AUD_PROC_PRIORITY        = 6;                   // 오디오 처리 태스크 우선순위
        inline constexpr uint8_t  VIB_PROC_PRIORITY        = 5;                   // 진동 처리 태스크 우선순위

        inline constexpr uint32_t ALIVE_CHECK_MS_DEF       = 5000;                // 헬스체크 보고 주기
        inline constexpr uint32_t QUEUE_BLOCK_MS_DEF       = 100;                 // 큐 대기 타임아웃
        inline constexpr uint32_t REBOOT_DELAY_MS_DEF      = 500;                 // 재부팅 전 안전 대기
        inline constexpr uint32_t BOOT_DELAY_MS_DEF        = 1000;                // 부팅 직후 센서 안정화 대기
        inline constexpr uint32_t MAIN_LOOP_DELAY_MS_DEF   = 10;                  // 메인 루프 양보 대기
        inline constexpr uint32_t LAZY_WRITE_MS_DEF        = 3000;                // 지연 쓰기(Lazy Write) 주기
    }  // namespace Task

    namespace Path {
        inline constexpr char const* MOUNT_SD_CONST        = "/sdcard";                 // VFS 마운트 포인트
        inline constexpr char const* FILE_CFG_JSON_CONST   = "/sys/cfg_250.json";       // 런타임 동적 설정값
        inline constexpr char const* FILE_CFG_TMP_CONST    = "/sys/cfg_250.json.tmp";   // 설정 임시 파일 경로
        inline constexpr char const* FILE_IDX_JSON_CONST   = "/sys/idx_250.json";       // 인덱스 파일 경로
        inline constexpr char const* FILE_IDX_TMP_CONST    = "/sys/idx_250.json.tmp";   // 인덱스 임시 파일 경로
        inline constexpr char const* DIR_FALLBACK_CONST    = "/fallback";               // 내부 플래시 우회 경로
    }  // namespace Path

    namespace NVS {
        inline constexpr char const*  NAMESPACE_CONST    = "T2_250_sys";                // NVS 파티션 네임스페이스
        inline constexpr char const*  KEY_FILE_SEQ_CONST = "file_seq";                  // 시퀀스 번호 저장 키
    }  // namespace NVS

    namespace NetLimit {
        inline constexpr uint16_t     NET_SSID_LEN_MAX                 = 32;            // Wi-Fi SSID 최대 길이
        inline constexpr uint16_t     NET_PW_LEN_MAX                   = 64;            // Wi-Fi 비밀번호 최대 길이
        inline constexpr uint16_t     NET_IP_LEN_MAX                   = 16;            // IP 주소 최대 길이
        inline constexpr uint8_t      NET_MULTI_AP_MAX                 = 3;             // 오토폴백 예비 AP 최대 개수
        inline constexpr uint16_t     NET_BROKER_LEN_MAX               = 64;            // MQTT 브로커 주소 길이
        inline constexpr uint16_t     NET_ID_LEN_MAX                   = 32;            // MQTT Client ID 최대 길이
        inline constexpr uint16_t     NET_TOPIC_LEN_MAX                = 64;            // MQTT 토픽 최대 길이
        inline constexpr uint32_t     NET_WIFI_DISCONNECT_DELAY_MS_DEF = 100;           // Wi-Fi 재연결 대기 시간
    }  // namespace NetLimit

    namespace Net {
        inline constexpr char const*  WIFI_AP_SSID_DEF           = "MSFM_T2_AP";        // 자체 AP SSID
        inline constexpr char const*  WIFI_AP_PW_DEF             = "12345678";          // 자체 AP PW
        inline constexpr char const*  WIFI_AP_IP_DEF             = "192.168.4.1";       // 자체 AP IP

        inline constexpr char const*  NTP_TZ_INFO_CONST          = "KST-9";             // 한국 표준시
        inline constexpr char const*  NTP_SERVER_1_CONST         = "pool.ntp.org";      // 주 NTP 서버
        inline constexpr char const*  NTP_SERVER_2_CONST         = "time.google.com";   // 보조 NTP 서버
        inline constexpr uint32_t     NTP_SYNC_TIMEOUT_MS_DEF    = 5000;                // NTP 타임아웃

        inline constexpr uint16_t     HTTP_PORT_DEF              = 80;                  // HTTP 포트
        inline constexpr char const*  WS_URI_CONST               = "/ws";               // 웹소켓 URI

        inline constexpr uint16_t     MQTT_PORT_DEF              = 1883;                // MQTT 포트
        inline constexpr char const*  MQTT_ID_DEF                = "MSFM_T2_001";       // MQTT 클라이언트 ID
        inline constexpr char const*  MQTT_TOPIC_DEF             = "FACTORY_A_LINE_1/MSFM_T2_001";      // MQTT 최상위 토픽
        inline constexpr char const*  MQTT_LWT_DEF               = "FACTORY_A_LINE_1/MSFM_T2_001/lwt";  // 유언(LWT) 토픽
        inline constexpr uint8_t      MQTT_PROTO_VER_DEF         = 4;                   // MQTT 버전 (v3.1.1)
        inline constexpr uint8_t      MQTT_QOS_DEF               = 1;                   // 기본 QoS
    }  // namespace Net

    namespace StorageLimit {
        inline constexpr uint16_t     PATH_LEN_MAX       = 128;                               // 파일 절대 경로 상한
        inline constexpr uint16_t     PREFIX_LEN_MAX     = 16;                                // 데이터 파일 접두어 상한
        inline constexpr uint16_t     ROTATE_LIST_MAX    = 32;                                // 로테이션 목록 상한
        inline constexpr uint16_t     DIR_FILES_MAX      = 500;                               // O(N) 지연 방지 파일 상한
        inline constexpr uint32_t     PREALLOC_BYTES_MAX = 10 * System::BYTES_PER_MB_CONST;   // 플래시 선할당 크기
        inline constexpr uint16_t     WATERMARK_HIGH_DEF = 8;                                 // 디스크 플러시 버퍼 임계치
    }  // namespace StorageLimit

    namespace Storage {
        inline constexpr uint8_t      PRE_TRIGGER_SEC_DEF   = 3;                                 // 프리트리거 과거 보존 시간
        inline constexpr uint32_t     ROTATE_MB_DEF         = 10;                                // 로테이션 기준 용량
        inline constexpr uint32_t     ROTATE_MIN_DEF        = 60;                                // 로테이션 기준 시간
        inline constexpr uint16_t     ROTATE_KEEP_MAX_DEF   = 8;                                 // 파일 최대 보존 개수
        inline constexpr uint32_t     IDLE_FLUSH_MS_DEF     = 250;                               // 유휴 강제 플러시 시간

        inline constexpr char const*  SD_DIR_RAW_CONST      = "/MSFM_T2_250_data/raw";           // 원시 파형 저장 경로
        inline constexpr char const*  SD_DIR_BIN_CONST      = "/MSFM_T2_250_data/bin";           // 특징량 바이너리 저장 경로
    }  // namespace Storage

    // 공유 공통 설정 (트리거 등 전역 공통 사항)
    namespace Trigger {
        inline constexpr uint32_t HOLD_TIME_MS_DEF      = 5000;                                  // 이벤트 소멸 후 유지 지연 시간
        inline constexpr bool     USE_DEEP_SLEEP_DEF    = false;                                 // 유휴 절전 진입 여부
        inline constexpr uint32_t SLEEP_SEC_DEF         = 300;                                   // 절전 진입 유휴 대기 시간
        inline constexpr uint16_t WAKE_DURATION_DEF     = 5;                                     // 하드웨어 모션 감지 인정 프레임 수
    }  // namespace Trigger

    namespace Dsp {
        inline constexpr int     MEDIAN_WINDOW_MAX          = 15;                                // 메디안 필터 윈도우 정적 최대치
        inline constexpr float   WINDOW_AMPLITUDE_CORR_HANN = 2.0000f;                           // Hann 윈도우 진폭 보상 계수
        inline constexpr float   WINDOW_ENERGY_CORR_HANN    = 1.6330f;                           // Hann 윈도우 에너지 보상 계수
    }  // namespace Dsp

    namespace Decision {
        inline constexpr uint8_t MAX_TRIAL_COUNT_MAX   = 5;                                     // 노이즈 오작동 검증 로직 최대 횟수
        inline constexpr uint8_t MAX_TRIAL_COUNT_DEF   = 3;                                     // 이벤트 검증 기본 횟수
        inline constexpr float   STA_LTA_THRESHOLD_DEF = 3.0f;                                  // 단기/장기 급변 감지 비율 임계치
        inline constexpr int     MIN_TRIGGER_COUNT_DEF = 1;                                     // 유효 판정 최소 연속 트리거 발생 횟수
        inline constexpr float   VALID_START_SEC_DEF   = 0.30f;                                 // 유효 판정 시작 시간 (초기 과도 응답 배제)
        inline constexpr float   VALID_END_SEC_DEF     = 0.50f;                                 // 판정 종료 유효 시간
    }  // namespace Decision
}  // namespace Global

// ========================================================================
// 2. Imu: 진동 센서 하드웨어 공통 (BMI270)
// ========================================================================
namespace Imu {
    namespace Hardware {
        inline constexpr uint8_t  PIN_CS_CONST              = 10;        // BMI270 SPI CS 핀
        inline constexpr uint8_t  PIN_INT1_WATERMARK_CONST  = 11;        // FIFO Watermark 인터럽트 전용 수신 핀 (Core 0 바인딩)
        inline constexpr uint8_t  PIN_INT2_MOTION_CONST     = 12;        // Any-Motion 전용 수신 핀 (Core 1 / 외부 Ext0 딥슬립 웨이크업 바인딩)
        inline constexpr uint32_t SPI_FREQ_HZ_CONST         = 10000000;  // SPI 버스 속도 (10MHz)

        // ---BMI270 내부 인터럽트 및 상태 레지스터 주소 (매직넘버 제로화) ---
        inline constexpr uint8_t  REG_INT_STATUS_0_ADDR     = 0x1C;      // 인터럽트 상태 레지스터 0 (Motion 감지 확인용)
        inline constexpr uint8_t  REG_INT_STATUS_1_ADDR     = 0x1D;      // 인터럽트 상태 레지스터 1 (FIFO Watermark 확인용)
        inline constexpr uint8_t  REG_FIFO_WTM_STATUS_BIT   = 0x02;      // Status 1 내 FIFO Watermark 비트 마스크 (0x02)
        inline constexpr uint8_t  REG_ANY_MOTION_STATUS_BIT = 0x20;      // Status 0 내 Any-Motion 감지 비트 마스크 (0x20)


        inline constexpr uint16_t FIFO_WATERMARK_LIMIT     = 40;         // 40세트 누적 시 워터마크 인터럽트 발생 기본값
        inline constexpr uint16_t FIFO_BATCH_SIZE_MAX      = 128;        // FIFO 버스트 리드 버퍼 크기 정적 상한선
        inline constexpr uint16_t FIFO_BATCH_SIZE_DEF      = 40;         // 예제 watermark 매핑: 약 25ms 주기로 고속 인출 수행

        inline constexpr uint8_t  FIFO_FRAME_SIZE_CONST    = 7;          // FIFO 프레임 크기 (Header+Data)
        inline constexpr uint8_t  FIFO_HEADER_ACCEL_CONST  = 0x84;       // Accel 식별 헤더
        inline constexpr uint8_t  FIFO_HEADER_GYRO_CONST   = 0x88;       // Gyro 식별 헤더
        inline constexpr uint8_t  REG_FIFO_LEN_ADDR_CONST  = 0x24;       // FIFO 길이 레지스터
        inline constexpr uint8_t  REG_FIFO_DATA_ADDR_CONST = 0x26;       // FIFO 데이터 레지스터

        // --- Any-Motion 제어 레지스터 상세 파라미터 ---
        inline constexpr float    ANY_MOTION_LSB_2G_MG     = 0.48f;      // Any-motion 임계치 LSB (2G)
        inline constexpr uint32_t STARTUP_DELAY_MS_CONST   = 5;          // 활성화 안정화 대기 시간
        inline constexpr uint8_t  REG_TEMP_MSB_CONST       = 0x22;       // BMI270 칩 온도 레지스터 및 변환 상수
        inline constexpr int16_t  TEMP_INVALID_CONST       = -32768;     // 온도 레지스터 무효값 판정
        inline constexpr float    TEMP_SCALE_CONST         = 512.0f;     // 온도 LSB → ℃ 변환 계수
        inline constexpr float    TEMP_OFFSET_CONST        = 23.0f;      // 온도 보정 오프셋 (℃)
    }  // namespace Hardware
}  // namespace Imu

// ========================================================================
// 3. Accel: 가속도 (진동) 파이프라인
// ========================================================================
namespace Accel {
    namespace Sensor {
        inline constexpr bool     ENABLE_DEF          = true;      // 가속도 기본 활성화 스위치
        inline constexpr uint8_t  AXIS_MAX            = 3;         // 물리 축 최대 개수 (X, Y, Z)
        inline constexpr uint8_t  AXIS_MASK_DEF       = 0b111;     // 기본 활성 축 마스크 (3축 모두)

        inline constexpr uint32_t RATE_MAX            = 1600;      // 가속도 최대 샘플링 레이트
        inline constexpr uint32_t RATE_DEF            = 1600;      // 기본 샘플링 레이트 (ODR = BMI2_ACC_ODR_1600HZ)
        inline constexpr uint8_t  RANGE_DEF           = 8;         // 하드웨어 측정 범위 기본값 (±8G 셋팅: BMI2_ACC_RANGE_8G)
        inline constexpr uint8_t  ODR_REG_DEF         = 0x0C;      // 레지스터 1600Hz 인덱스값 매핑
        inline constexpr uint8_t  BWP_REG_DEF         = 0x02;      // Normal 모드 대역폭 매핑 (BMI2_ACC_NORMAL_AVG4)
        inline constexpr uint8_t  PERF_MODE_DEF       = 0x01;      // 성능 최적화 모드 (BMI2_PERF_OPT_MODE)
        inline constexpr uint8_t  NOISE_PERF_MODE_DEF = 0x01;      // [이슈 18] 가속도 저소음 최적화 모드 바인딩

        inline constexpr uint32_t FFT_SIZE_MAX        = 1024;      // 주파수 분석 최대 해상도
        inline constexpr uint32_t FFT_SIZE_DEF        = 1024;      // [물리 논리 동기화] 시간축 정합을 위한 가속도 분석 윈도우 1024 통일

    }  // namespace Sensor
    namespace FeatureLimit {
        inline constexpr uint8_t  BAND_MAX            = 16;        // 가속도 주파수 대역 감시 최대 개수
        inline constexpr uint8_t  BAND_DEF            = 8;         // 실제 런타임 연산 기본 개수
        inline constexpr uint16_t FIR_TAPS_MAX        = 127;       // 가속도 FIR 필터 최대 차수
        inline constexpr uint16_t FIR_TAPS_DEF        = 63;        // FIR 필터 기본 차수

        // --- 힐버트 필터 및 군지연 보정 상수 (31차 FIR) ---
        inline constexpr uint16_t HILBERT_FIR_TAPS    = 31;        // 힐버트 필터 탭 수 (31차 정수형 FIR 구현)
        inline constexpr uint16_t HILBERT_GROUP_DELAY = (HILBERT_FIR_TAPS - 1) / 2; // 힐버트 군지연(Group Delay) 15
        
        static_assert(HILBERT_GROUP_DELAY == 15, "Hilbert Filter Group Delay configuration mismatch!");
        static_assert(ESP_DSP_VERSION_CHECK_VAL == 10802, "esp-dsp version assertion failed!");

        // 현상: STA_SAMPLES_DEF = 1600/1000 = 1, LTA_SAMPLES_DEF = 1600/100 = 16로 STA가 1샘플에 불과하여 STA/LTA가 무력화됨.
        // 해결방안: 해당 상수를 (샘플레이트 * 시구간_ms) / 1000 형태로 재정의합니다. 동시에 uint32_t에 맞게 반올림 처리합니다.
        inline constexpr uint32_t STA_DURATION_MS     = 1;          // 1ms                                            
        inline constexpr uint32_t LTA_DURATION_MS     = 10;         // 10ms
        inline constexpr uint32_t STA_SAMPLES_DEF     = (Sensor::RATE_DEF * STA_DURATION_MS + 500) / 1000;   // (1600*1)/1000 = 1.6 → 2
        inline constexpr uint32_t LTA_SAMPLES_DEF = (Sensor::RATE_DEF * LTA_DURATION_MS + 500) / 1000;   // (1600*10)/1000 = 16 → 16

    }  // namespace FeatureLimit
    namespace Dsp {
        inline constexpr float        HPF_CUTOFF_DEF      = 10.0f;     // 가속도 HPF 기본 차단 주파수
        inline constexpr float        LPF_CUTOFF_DEF      = 800.0f;    // 가속도 LPF 기본 차단 주파수
        inline constexpr float        NOTCH_FREQ_DEF      = 60.0f;     // 전원 노이즈 노치 제거 주파수
        inline constexpr float        NOTCH2_FREQ_DEF     = 120.0f;    // 제2 고조파 노이즈 제거 주파수
        inline constexpr float        NOTCH_Q_DEF         = 30.0f;     // 노치 Q-Factor
    }  // namespace Dsp
    namespace Trigger {
        inline constexpr float        RMS_THRESH_DEF       = 0.5f;      // 광대역 RMS 결함 임계치
        inline constexpr float        WAKE_THRESH_G_DEF    = 1.0f;      // Any-Motion 인터럽트 임계치
        inline constexpr float        KURT_NG_THRESH_DEF   = 5.0f;      // 첨도 결함 임계치
        inline constexpr float        CREST_NG_THRESH_DEF   = 6.0f;     // 파고율 결함 임계치
        inline constexpr float        SKEW_NG_THRESH_DEF    = 1.5f;     // 왜도 결함 임계치
        inline constexpr float        BAND_RANGES_DEF[FeatureLimit::BAND_MAX][2] = {
            {10.0f, 30.0f}, {30.0f, 60.0f}, {60.0f, 100.0f}, {100.0f, 250.0f}, {250.0f, 400.0f}, {400.0f, 600.0f}, {600.0f, 800.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}};
    }  // namespace Trigger
    
    namespace Calib {
        inline constexpr char const*  FILE_JSON_CONST     = "/sys/acc_calib_250.json";     // 가속도 영점 보정 파일
        inline constexpr uint16_t     TARGET_SAMPLES_DEF  = 100;                           // 캘리브레이션 목표 샘플
        inline constexpr uint16_t     RETRY_MAX_CONST     = 15;                            // 캘리브레이션 재시도
        inline constexpr uint32_t     LOOP_DELAY_MS_CONST = 20;                            // 캘리브레이션 루프 대기
    }  // namespace Calib
}  // namespace Accel

// ========================================================================
// 4. Gyro: 자이로 (각속도) 파이프라인
// ========================================================================
namespace Gyro {
    namespace Sensor {
        inline constexpr bool     ENABLE_DEF          = false;  // 자이로 기본 비활성화 스위치 (최적화)
        inline constexpr uint8_t  AXIS_MAX            = 3;      // 물리 축 최대 개수 (X, Y, Z)
        inline constexpr uint8_t  AXIS_MASK_DEF       = 0b111;  // 기본 활성 축 마스크

        // 가속도와 타임라인 1:1 결합을 위해 ODR 1600Hz 완벽 동기화 및 버퍼 개정
        inline constexpr uint32_t RATE_MAX            = 1600;   // 가속도와 동기화를 위해 정적 속도를 1600Hz로 잠금
        inline constexpr uint32_t RATE_DEF            = 1600;   // 자이로 기본 샘플링 레이트 (ODR = BMI2_GYR_ODR_1600HZ)
        inline constexpr uint8_t  RANGE_DEF           = 250;    // 측정 정밀도 범위 셋팅 (±250 dps: BMI2_GYR_RANGE_250)
        inline constexpr uint8_t  ODR_REG_DEF         = 0x0C;   // BMI270 Gyro 1600Hz 레지스터 매핑값
        inline constexpr uint8_t  BWP_REG_DEF         = 0x00;   // 자이로 로우패스 필터 Normal 모드 매핑 (BMI2_GYR_NORMAL_MODE)
        inline constexpr uint8_t  PERF_MODE_DEF       = 0x01;   // 성능 최적화 모드 진입 지시어
        inline constexpr uint8_t  NOISE_PERF_MODE_DEF = 0x01;   // [이슈 2] 자이로 저소음 전용 모드 (noise_perf 레지스터 독립 통제)

        inline constexpr uint32_t FFT_SIZE_MAX        = 1024;    // [물리적 비대칭성 모순 제거] 가속도와 프레임 주기를 동일하게 1024로 고정
        inline constexpr uint32_t FFT_SIZE_DEF        = 1024;    // 자이로 기본 분석 해상도
    }  // namespace Sensor
    namespace FeatureLimit {
        inline constexpr uint8_t  BAND_MAX          = 8;         // 자이로 주파수 대역 감시 최대 개수
        inline constexpr uint8_t  BAND_DEF          = 4;         // 자이로 실제 런타임 대역 감시 수
        inline constexpr uint16_t FIR_TAPS_MAX      = 63;        // 자이로 FIR 최대 차수 (낮게 유지)
        inline constexpr uint16_t FIR_TAPS_DEF      = 31;        // 자이로 FIR 기본 차수

        // 현상: STA_SAMPLES_DEF = 1600/1000 = 1, LTA_SAMPLES_DEF = 1600/100 = 16로 STA가 1샘플에 불과하여 STA/LTA가 무력화됨.
        // 해결방안: 해당 상수를 (샘플레이트 * 시구간_ms) / 1000 형태로 재정의합니다. 동시에 uint32_t에 맞게 반올림 처리합니다.
        inline constexpr uint32_t STA_DURATION_MS = 1;           // 1ms
        inline constexpr uint32_t LTA_DURATION_MS = 10;          // 10ms
        inline constexpr uint32_t STA_SAMPLES_DEF =
            (Sensor::RATE_DEF * STA_DURATION_MS + 500) / 1000;   // (1600*1)/1000 = 1.6 → 2
        inline constexpr uint32_t LTA_SAMPLES_DEF =
            (Sensor::RATE_DEF * LTA_DURATION_MS + 500) / 1000;   // (1600*10)/1000 = 16 → 16


    }  // namespace FeatureLimit
    namespace Dsp {
        inline constexpr float       HPF_CUTOFF_DEF          = 1.0f;      // 자이로 DC 드리프트 제거용 HPF
        inline constexpr float       LPF_CUTOFF_DEF          = 500.0f;    // 자이로 고주파 노이즈 제거용 LPF
        inline constexpr float       NOTCH_FREQ_DEF          = 60.0f;     // 전원 노이즈 노치
        inline constexpr float       NOTCH2_FREQ_DEF         = 120.0f;    // 제2 고조파 노치
        inline constexpr float       NOTCH_Q_DEF             = 30.0f;     // 노치 Q-Factor
    }  // namespace Dsp
    namespace Trigger {
        inline constexpr float       RMS_THRESH_DEF          = 10.0f;      // 각속도 RMS 결함 임계치
        inline constexpr float       KURT_NG_THRESH_DEF      = 5.0f;       // 첨도 결함 임계치
        inline constexpr float       CREST_NG_THRESH_DEF     = 6.0f;       // 파고율 결함 임계치
        inline constexpr float       SKEW_NG_THRESH_DEF      = 1.5f;       // 왜도 결함 임계치
        inline constexpr float       BAND_RANGES_DEF[FeatureLimit::BAND_MAX][2] = {
            {1.0f, 10.0f}, {10.0f, 50.0f}, {50.0f, 100.0f}, {100.0f, 250.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}};
    }  // namespace Trigger
    namespace Calib {
        inline constexpr char const* FILE_JSON_CONST         = "/sys/gyr_calib_250.json";           // 자이로 영점 보정 파일
        inline constexpr uint16_t    TARGET_SAMPLES_DEF      = 200;                                 // 자이로 노이즈 평균화를 위해 더 많은 샘플 권장
        inline constexpr uint16_t    RETRY_MAX_CONST         = 15;                                  // 캘리브레이션 재시도
        inline constexpr uint32_t    LOOP_DELAY_MS_CONST     = 20;                                  // 캘리브레이션 루프 대기
    }
}  // namespace Gyro

// ========================================================================
// 5. Audio: 소음 파이프라인
// ========================================================================
namespace Audio {
    namespace Hardware {
        inline constexpr int      I2S_PORT_NUM_CONST       = 0;                 // ESP32 I2S 할당 포트
        inline constexpr uint8_t  PIN_I2S_BCLK_CONST       = 41;                // I2S BCLK 핀
        inline constexpr uint8_t  PIN_I2S_MCLK_CONST       = 0;                 // I2S MCLK 핀
        inline constexpr uint8_t  PIN_I2S_WS_CONST         = 42;                // I2S WS(LRCLK) 핀
        inline constexpr uint8_t  PIN_I2S_DIN_CONST        = 43;                // I2S DATA 핀
        inline constexpr int      DMA_BUF_COUNT_MAX        = 32;                // I2S DMA 버퍼 상한
        inline constexpr int      DMA_BUF_COUNT_DEF        = 16;                // I2S DMA 버퍼 기본 개수 (원본유지)
        // [이슈 22] 중간안 B 확정: 128KB → 32KB (SRAM 96KB 절감, 언더런 마진 ~97ms)
        inline constexpr int      DMA_BUF_LEN_OPT_CONST    = 512;               // 청크 당 샘플 수 (512/42000 ≈ 12.2ms 주기)
        inline constexpr int      DMA_BUF_COUNT_OPT_CONST  = 8;                 // 핑폰 버퍼 개수 (8 * 12.2ms ≈ 97ms 총 마진)
        inline constexpr uint16_t RAW_FRAME_BUFFERS_MAX    = 8;                 // 핑폰 버퍼 최대치
        inline constexpr uint16_t RAW_FRAME_BUFFERS_DEF    = 4;                 // 핑퐁 버퍼 기본치
        inline constexpr float    PCM_32BIT_SCALE_CONST    = 4.656612873e-10f;  // 32bit PCM 정규화 계수
        inline constexpr bool     USE_APLL_DEF             = true;              // [이슈 24] I2S APLL 항상 활성 (42kHz 정밀 클럭 필수)
    }  // namespace Hardware
    namespace Sensor {
        inline constexpr bool     ENABLE_DEF              = true;      // 소음 마스터 활성화 스위치
        inline constexpr uint8_t  CHANNELS_MAX            = 2;         // 오디오 물리 채널 정적 할당
        inline constexpr uint8_t  CHANNEL_MASK_DEF        = 0b11;      // 기본 스테레오 활성 (CH_STEREO)
        inline constexpr uint8_t  BITS_PER_SAMPLE_CONST   = 32;        // I2S 비트 심도
        inline constexpr uint32_t RATE_MAX                = 48000;     // 마이크 최대 샘플링 레이트
        inline constexpr uint32_t RATE_DEF                = 42000;     // 마이크 기본 샘플링 레이트
        inline constexpr uint32_t FFT_SIZE_MAX            = 4096;      // 오디오 최대 FFT 해상도
        inline constexpr uint32_t FFT_SIZE_DEF            = 1024;      // 오디오 기본 FFT 해상도
    }  // namespace Sensor
    namespace FeatureLimit {
        inline constexpr uint8_t  BAND_MAX                = 16;       // 오디오 주파수 대역 감시 최대치
        inline constexpr uint8_t  BAND_DEF                = 8;        // 실제 런타임 밴드 개수
        inline constexpr uint16_t FIR_TAPS_MAX            = 255;      // 오디오 FIR 최대 차수 (정밀 EQ)
        inline constexpr uint16_t FIR_TAPS_DEF            = 63;       // 오디오 FIR 기본 차수
        inline constexpr uint8_t  TOP_PEAKS_MAX           = 10;       // 스펙트럼 피크 추출 한계
        inline constexpr uint8_t  TOP_PEAKS_DEF           = 5;        // 상위 피크 추출 기본 개수
        inline constexpr uint8_t  CEPS_TARGET_MAX         = 5;        // 켑스트럼 타겟 최대치
        inline constexpr uint8_t  CEPS_TARGET_DEF         = 3;        // 켑스트럼 타겟 기본 개수
        inline constexpr float    CEPS_TOLERANCE_DEF      = 0.0003f;  // 켑스트럼 오차 허용치
        inline constexpr uint8_t  CEPS_NOISE_MARGIN_CONST = 5;        // [이슈 40] 켑스트럼 피크 노이즈 마진 빈 수 (±5)
        inline constexpr uint32_t MEL_BANDS_MAX           = 40;       // Mel 밴드 상한
        inline constexpr uint32_t MEL_BANDS_DEF           = 26;       // 런타임 Mel 밴드
        inline constexpr float    MEL_SCALE_2595_CONST    = 2595.0f;  // Mel 스케일 변환 계수
        inline constexpr float    MEL_SCALE_700_CONST     = 700.0f;   // Mel 스케일 분모 상수
        inline constexpr uint16_t MEL_CHUNK_ROWS_CONST    = 64;       // 행렬 연산 캐시 타일링 청크

        // Audio ST/LT duration: 1ms, 10ms (샘플레이트 42kHz 기준)
        inline constexpr uint32_t STA_DURATION_MS = 1;
        inline constexpr uint32_t LTA_DURATION_MS = 10;
        inline constexpr uint32_t STA_SAMPLES_DEF =
            (Sensor::RATE_DEF * STA_DURATION_MS + 500) / 1000;   // (42000*1+500)/1000 = 42.5 → 42
        inline constexpr uint32_t LTA_SAMPLES_DEF =
            (Sensor::RATE_DEF * LTA_DURATION_MS + 500) / 1000;   // (42000*10+500)/1000 = 420.5 → 420


    }  // namespace FeatureLimit
    namespace Dsp {
        inline constexpr float        HPF_CUTOFF_DEF          = 20.0f;       // 오디오 고역 통과(DC 차단)
        inline constexpr float        LPF_CUTOFF_DEF          = 16000.0f;    // 오디오 저역 통과 (안티에일리어싱)
        inline constexpr float        NOTCH_FREQ_DEF          = 60.0f;       // 오디오 전원 험 노이즈
        inline constexpr float        NOTCH2_FREQ_DEF         = 120.0f;      // 제2 고조파 험 노이즈
        inline constexpr float        NOTCH_Q_DEF             = 30.0f;       // 오디오 노치 Q
        inline constexpr float        BEAMFORMING_GAIN_DEF    = 0.5f;        // 스테레오 빔포밍 합성 게인
        inline constexpr float        WINDOW_MS_DEF           = 25.0f;       // STFT 슬라이딩 윈도우 ms
        inline constexpr float        HOP_MS_DEF              = 10.0f;       // STFT 이동 간격 ms
        inline constexpr float        SPECTRAL_SUB_GAIN_DEF   = 1.2f;        // 스펙트럼 감산 노이즈 제거 강도
        inline constexpr float        NOISE_LEARN_ALPHA_DEF   = 0.01f;       // 적응형 노이즈 학습 계수
        inline constexpr float        NOISE_GATE_THRESH_DEF   = 0.001f;      // 무음 묵음 처리 임계치
        inline constexpr float        PRE_EMPHASIS_ALPHA_DEF  = 0.97f;       // 고주파 강조 프리엠파시스
    }  // namespace Dsp
    namespace Trigger {
        inline constexpr float        RMS_THRESH_DEF          = 0.05f;        // 소음 광대역 트리거 임계치
        inline constexpr float        KURT_NG_THRESH_DEF      = 5.0f;         // 소음 첨도 불량 임계치
        inline constexpr float        CREST_NG_THRESH_DEF     = 4.0f;         // 소음 파고율 불량 임계치
        inline constexpr float        SKEW_NG_THRESH_DEF      = 2.0f;         // 소음 왜도 불량 임계치
        inline constexpr float        BAND_RANGES_DEF[FeatureLimit::BAND_MAX][2] = {
            {100.0f, 500.0f}, {500.0f, 1500.0f}, {1500.0f, 4000.0f}, {4000.0f, 8000.0f}, {8000.0f, 12000.0f}, {12000.0f, 16000.0f}, {16000.0f, 20000.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}};
    }  // namespace Trigger
    namespace CalibLimit {
        inline constexpr float        GAIN_RATIO_MIN_CONST      = 0.707f;                   // EQ 튜닝 보수적 하한
        inline constexpr float        GAIN_RATIO_MAX_CONST      = 1.414f;                   // EQ 튜닝 보수적 상한
        inline constexpr uint32_t     WELCH_CHUNK_SAMPLES_DEF   = Sensor::FFT_SIZE_DEF;     // Welch 평균화 청크
        inline constexpr uint8_t      AUTO_STABLE_SEC_DEF       = 10;                       // 자동 캘리 안정화 판정
    }  // namespace CalibLimit
    namespace Calib {
        inline constexpr char const*  FILE_JSON_CONST        = "/sys/aud_calib_250.json";   // 마이크 EQ 보정 파일
        inline constexpr uint32_t     AUTO_IDLE_MIN_DEF      = 60;                          // 유휴 시간 후 자동 캘리 진입
        inline constexpr float        REF_FREQ_HZ_DEF        = 1000.0f;                     // 타겟 기준 주파수
        inline constexpr float        FILTER_MIN_FREQ_HZ_DEF = 100.0f;                      // 유효 EQ 필터 하한
        inline constexpr float        FILTER_MAX_FREQ_HZ_DEF = 8000.0f;                     // 유효 EQ 필터 상한
        inline constexpr float        TARGET_GAIN_MAX_DEF    = 3.0f;                        // FIR EQ 증폭 상한
        inline constexpr float        TARGET_GAIN_MIN_DEF    = 0.3f;                        // FIR EQ 감쇠 하한
        inline constexpr float        NORM_SAFE_THRESH_DEF   = 1.5f;                        // 필터 정규화 안전 마진
    }  // namespace Calib
}  // namespace Audio

inline constexpr uint32_t I2S_DMA_CHUNK_SIZE = Audio::Sensor::CHANNELS_MAX * (Audio::Sensor::BITS_PER_SAMPLE_CONST / 8) * Audio::Hardware::DMA_BUF_LEN_OPT_CONST; // I2S DMA 수집용 버퍼 청크 바이트 크기 계산식

// ========================================================================
// 6. AI: 시계열 텐서 조합 및 추론 하이퍼파라미터 (SSOT 파생 계산식 적용)
// ========================================================================
namespace AI {
    namespace Tensor {
        // 모든 도메인의 다중 채널을 1D Vector로 합치기 위한 통합 채널 수 계산
        inline constexpr uint8_t  CHANNELS_MAX          = Accel::Sensor::AXIS_MAX + Gyro::Sensor::AXIS_MAX + Audio::Sensor::CHANNELS_MAX;     // 3 + 3 + 2 = 8

        inline constexpr uint32_t MFCC_COEFFS_MAX      = 32;                     // 정적 단일 프레임 계수 할당 상한
        inline constexpr uint32_t MFCC_COEFFS_DEF      = 13;                     // 런타임 추출 기본 계수 차수
        inline constexpr uint32_t MFCC_COMPONENTS_MAX  = 3;                      // 3-Dimension 조합 (Static, Delta, D-Delta)
        inline constexpr uint32_t MFCC_COMPONENTS_DEF  = 3;                      // 런타임 차원 조합
        inline constexpr uint16_t DELTA_HISTORY_MAX    = 10;                     // Delta 연산 기록 버퍼 최대치
        inline constexpr uint16_t DELTA_HISTORY_DEF    = 5;                      // Delta 연산 프레임 간격
        inline constexpr uint16_t DELTA_GAP_CONST      = DELTA_HISTORY_DEF - 1;  // [이슈 49] Delta 인덱스 타임 갭 (=4, 하드코딩 매직넘버 제거)

        // MFCC 차원 = 추출 차수 * 변위 컴포넌트(3) * 활성 채널(8) : 32 * 3 * 8 = 768 차원 할당
        inline constexpr uint32_t MFCC_DIM_MAX          = MFCC_COEFFS_MAX * MFCC_COMPONENTS_MAX * CHANNELS_MAX;
        // 13 * 3 * 8 = 312 차원 연산
        inline constexpr uint32_t MFCC_DIM_DEF          = MFCC_COEFFS_DEF * MFCC_COMPONENTS_DEF * CHANNELS_MAX;
        }  // namespace Tensor
    }  // namespace AI
}  // namespace T2_Def

inline std::atomic<int64_t> g_MonotonicTimeOffsetUs{0}; // NTP 시간 동기화 시 시스템 단조 시계(esp_timer) 보정을 위한 원자적 오프셋 누적 메모리 (마이크로초)

// 오프셋 보정이 반영된 물리 및 소프트웨어 통합 단조 증가 타임스탬프 반환 함수 (마이크로초)
inline uint64_t get_monotonic_timestamp_us() {
    return static_cast<uint64_t>(esp_timer_get_time()) + g_MonotonicTimeOffsetUs.load(std::memory_order_relaxed);
}

// 데이터 영역에 대해 하드웨어 가속기(ROM)를 사용하여 32비트 고속 CRC32 체크섬을 계산하는 함수
inline uint32_t calculate_crc32_le(const uint8_t* p_data, uint32_t p_len) {
    return esp_rom_crc32_le(0, p_data, p_len);
}

namespace T2_General {
    struct AccelPolicy {
        static constexpr bool enable_mfcc = false;
        static constexpr bool enable_fft = true;
        static constexpr bool enable_timbre = false;
        static constexpr bool enable_band_energy = true;
        static constexpr int  band_count = 8;
    };
    struct GyroPolicy {
        static constexpr bool enable_mfcc = false;
        static constexpr bool enable_fft = true;
        static constexpr bool enable_timbre = false;
        static constexpr bool enable_band_energy = true;
        static constexpr int  band_count = 4; // 저주파 대역 제한
    };
    struct AudioPolicy {
        static constexpr bool enable_mfcc = true;
        static constexpr bool enable_fft = true;
        static constexpr bool enable_timbre = true;
        static constexpr bool enable_band_energy = true;
        static constexpr int  band_count = 16;
    };
}



