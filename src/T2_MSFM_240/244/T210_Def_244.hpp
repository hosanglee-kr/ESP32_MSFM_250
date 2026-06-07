/* ============================================================================
 * File: T210_Def_244.hpp
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
            inline constexpr char const* VERSION_STR           = "T240_v243_7";         // 시스템 펌웨어 식별 버전
            inline constexpr char const* SITE_ID_DEF           = "FACTORY_A_LINE_1";    // 기본 설치 현장 및 설비 식별자

            inline constexpr bool        VIB_ENABLE_DEF        = true;                  // 진동 센서 모듈 마스터 활성화 스위치
            inline constexpr bool        AUDIO_ENABLE_DEF      = true;                  // 소음(마이크) 센서 모듈 마스터 활성화 스위치

            inline constexpr uint8_t     TELEMETRY_HZ_DEF      = 10;                    // 상태 및 특징량 데이터 전송 주기 (Hz)
            inline constexpr uint8_t     WAVEFORM_HZ_DEF       = 0;                     // 원시 파형 데이터 전송 주기 (Hz, 0=기본 비활성)

            inline constexpr uint16_t    SEQUENCE_FRAMES_MAX   = 32;                    // AI 텐서 시퀀스 최대 프레임 수 (정적 메모리 할당 한계치)
            inline constexpr uint16_t    SEQUENCE_FRAMES_DEF   = 16;                    // AI 텐서 시퀀스 기본 프레임 수 (런타임 제어용)

            inline constexpr float       MATH_EPSILON_CONST    = 1e-6f;                 // 부동소수점 0 나누기 방어용 최소값
            inline constexpr float       MATH_EPSILON_12_CONST = 1e-12f;                // 로그(Log)/제곱근 연산 0 진입 방어용 고정밀 최소값
            inline constexpr float       MS_PER_SEC_CONST      = 1000.0f;               // 초(Sec) 단위를 밀리초(ms)로 역산하기 위한 계수
            inline constexpr uint32_t    BYTES_PER_MB_CONST    = 1024 * 1024;           // 메가바이트(MB)를 바이트(Bytes)로 역산하기 위한 계수
            inline constexpr uint32_t    MS_PER_MIN_CONST      = 60 * 1000;             // 분(Min) 단위를 밀리초(ms)로 역산하기 위한 계수
        }

        namespace Hardware {
            inline constexpr uint32_t    SERIAL_BAUD_CONST     = 115200;                // 디버깅용 PC 시리얼 통신 보드레이트
            inline constexpr uint8_t     PIN_BTN_CONTROL_CONST = 0;                     // 제어용 사용자 물리 버튼 핀 (보통 Boot 핀)
            inline constexpr uint8_t     PIN_RGB_LED_CONST     = 21;                    // 상태 표시용 RGB LED 핀
            inline constexpr uint8_t     PIN_SD_CLK_CONST      = 39;                    // SD카드 SPI 클럭 핀
            inline constexpr uint8_t     PIN_SD_CMD_CONST      = 38;                    // SD카드 SPI 커맨드(MOSI) 핀
            inline constexpr uint8_t     PIN_SD_D0_CONST       = 40;                    // SD카드 SPI 데이터(MISO) 핀
            inline constexpr uint8_t     PIN_NOT_SET_CONST     = 0xFF;                  // 하드웨어 핀 미할당 상태를 나타내는 특수 방어값
        }

        namespace Task {
            inline constexpr uint16_t    QUEUE_LEN_MAX         = 16;                    // FSM 메시지 큐 최대 길이 (메모리 할당용)
            inline constexpr uint16_t    QUEUE_LEN_DEF         = 8;                     // FSM 메시지 큐 기본 사용 길이
            inline constexpr uint32_t    WDG_TIMEOUT_MS_DEF    = 2000;                  // 소프트웨어 워치독 타임아웃 기본값 (2초)
            inline constexpr uint32_t    STORAGE_STACK_MAX     = 16384;                 // 스토리지 태스크 스택 최대 크기 (16KB)
            inline constexpr uint32_t    STORAGE_STACK_DEF     = 8192;                  // 스토리지 태스크 스택 기본 할당량 (8KB)
            inline constexpr uint8_t     STORAGE_PRIO_DEF      = 2;                     // 스토리지 태스크 우선순위 (보통 낮게 설정)
            inline constexpr uint8_t     CORE_PROCESS_DEF      = 1;                     // 메인 신호처리 태스크를 할당할 CPU 코어 번호

            inline constexpr uint32_t    ALIVE_CHECK_MS_DEF     = 5000;                 // 메인 루프 시스템 헬스체크 보고 주기 (5초)
            inline constexpr uint32_t    QUEUE_BLOCK_MS_DEF     = 100;                  // 큐 대기 타임아웃 블로킹 시간 (100ms)
            inline constexpr uint32_t    REBOOT_DELAY_MS_DEF    = 500;                  // 소프트웨어 재부팅 전 안전 대기 시간 (500ms)
            inline constexpr uint32_t    BOOT_DELAY_MS_DEF      = 1000;                 // 부팅 직후 센서 안정화 대기 시간 (1초)
            inline constexpr uint32_t    MAIN_LOOP_DELAY_MS_DEF = 10;                   // 메인 루프 와치독 양보를 위한 대기 시간 (10ms)
            inline constexpr uint32_t    LAZY_WRITE_MS_DEF      = 3000;                 // 파일 시스템 지연 쓰기(Lazy Write) 주기 (3초)
        }

        namespace Path {
            inline constexpr char const* MOUNT_SD_CONST      = "/sdcard";               // SD 카드 VFS 마운트 포인트
            inline constexpr char const* FILE_CFG_JSON_CONST = "/sys/runtime_cfg_243.json"; // 런타임 동적 설정값 저장 파일 경로
            inline constexpr char const* DIR_FALLBACK_CONST  = "/fallback";             // SD카드 오류 시 내부 플래시메모리 우회 경로
        }

        namespace NVS {
            inline constexpr char const* NAMESPACE_CONST     = "t240_sys";              // ESP32 NVS(비휘발성 메모리) 파티션 네임스페이스
            inline constexpr char const* KEY_FILE_SEQ_CONST  = "file_seq";              // 파일명 로테이션 생성을 위한 시퀀스 번호 저장 키
        }

        namespace NetLimit {
            inline constexpr uint16_t    MAX_SSID_LEN_CONST           = 32;             // Wi-Fi SSID 최대 길이 한계
            inline constexpr uint16_t    MAX_PW_LEN_CONST             = 64;             // Wi-Fi 비밀번호 최대 길이 한계
            inline constexpr uint16_t    MAX_IP_LEN_CONST             = 16;             // IP 주소 문자열 최대 길이 한계 (xxx.xxx.xxx.xxx\0)
            inline constexpr uint8_t     MAX_MULTI_AP_CONST           = 3;              // 자동 폴백용 예비 AP 등록 최대 개수
            inline constexpr uint16_t    MAX_BROKER_LEN_CONST         = 64;             // MQTT 브로커 주소 문자열 최대 길이 한계
            inline constexpr uint32_t    WIFI_DISCONNECT_DELAY_MS_DEF = 100;            // Wi-Fi 재연결 전 네트워크 스택 안정화 대기 시간
        }

        namespace Net {
            inline constexpr uint16_t    MQTT_PORT_DEF       = 1883;                    // 표준 MQTT 접속 포트
            inline constexpr char const* WS_URI_CONST        = "/api/t240/ws";          // 실시간 웹소켓 통신 엔드포인트 URI
            inline constexpr char const* TZ_INFO_CONST       = "KST-9";                 // 한국 표준시 타임존 설정값
            inline constexpr char const* NTP_SERVER_1_CONST  = "pool.ntp.org";          // 시간 동기화용 주 NTP 서버
			inline constexpr char const* NTP_SERVER_2_CONST  = "time.google.com";       // 시간 동기화용 주 NTP 서버
            inline constexpr uint32_t    SYNC_TIMEOUT_MS_DEF = 5000;                    // NTP 동기화 타임아웃 기본값 (5초)

            inline constexpr char const* WIFI_AP_SSID_DEF    = "SMEA_T240_AP";          // 디바이스 자체 AP 모드 구동 시 기본 SSID
            inline constexpr char const* WIFI_AP_PW_DEF      = "12345678";              // 디바이스 자체 AP 모드 구동 시 기본 비밀번호
            inline constexpr char const* WIFI_AP_IP_DEF      = "192.168.4.1";           // 디바이스 자체 AP 모드 구동 시 부여할 IP
            inline constexpr char const* MQTT_ID_DEF         = "T240_Edge";             // MQTT 통신 클라이언트 기본 ID
            inline constexpr char const* MQTT_TOPIC_DEF      = "smea/t240";             // MQTT 발행/구독 최상위 루트 토픽
            inline constexpr char const* MQTT_LWT_DEF        = "smea/t240/lwt";         // 비정상 종료 감지용 유언(Last Will) 토픽
            inline constexpr uint8_t     MQTT_PROTO_VER_DEF  = 4;                       // MQTT 프로토콜 버전 (4 = v3.1.1, 5 = v5.0)
            inline constexpr uint8_t     MQTT_QOS_DEF        = 1;                       // 기본 QoS 레벨 (1 = At least once)
        }

        namespace StorageLimit {
            inline constexpr uint16_t    MAX_PATH_LEN_CONST  = 128;                     // 파일 시스템 절대 경로 최대 길이 한계
            inline constexpr uint16_t    MAX_PREFIX_LEN_CONST= 16;                      // 데이터 파일 접두어 최대 길이 한계
            inline constexpr uint16_t    MAX_ROTATE_LIST_MAX = 32;                      // 링버퍼 관리를 위한 로테이션 목록 최대 보존 수
            inline constexpr uint16_t    MAX_DIR_FILES_MAX   = 500;                     // 단일 디렉토리 내 O(N) 지연 방지용 파일 개수 상한
            inline constexpr uint32_t    PREALLOC_BYTES_DEF  = 10 * System::BYTES_PER_MB_CONST; // 플래시 수명 보호를 위한 파일 선할당 크기 (10MB)
            inline constexpr uint16_t    WATERMARK_HIGH_DEF  = 8;                       // 디스크 플러시를 유발하는 버퍼 적재 프레임 임계치
        }

        namespace Storage {
            inline constexpr uint8_t     PRE_TRIGGER_SEC_DEF = 3;                       // 트리거 발생 시 소급하여 저장할 과거 데이터 시간 (3초)
            inline constexpr uint32_t    ROTATE_MB_DEF       = 10;                      // 이 용량을 초과하면 새 파일로 로테이션 (10MB)
            inline constexpr uint32_t    ROTATE_MIN_DEF      = 60;                      // 이 시간을 초과하면 새 파일로 로테이션 (60분)
            inline constexpr uint16_t    ROTATE_KEEP_MAX_DEF = 8;                       // 용량 초과 시 과거 파일을 삭제하며 유지할 파일 개수
            inline constexpr uint32_t    IDLE_FLUSH_MS_DEF   = 250;                     // 버퍼가 다 차지 않아도 강제로 기록을 밀어내는 유휴 시간 (250ms)

            inline constexpr char const* SD_DIR_RAW_CONST    = "/t240_data/raw";        // 가공되지 않은 파형(Raw) 데이터 저장 디렉토리
            inline constexpr char const* SD_DIR_BIN_CONST    = "/t240_data/bin";        // 분석 완료된 특징량(Feature) 바이너리 저장 디렉토리
        }
    }


    // ========================================================================
    // 2. Shared: 통합 신호처리 및 한계치(Limit)
    // ========================================================================
        namespace Shared {
        namespace FeatureLimit {
            inline constexpr uint8_t     BAND_RMS_MAX               = 16;       // 주파수 대역(Band) 에너지 감시 배열의 정적 메모리 할당 한계 (OOM 방어)
            inline constexpr uint8_t     TOP_PEAKS_MAX              = 10;       // 스펙트럼 피크(Peak) 추출 배열의 정적 메모리 할당 한계
            inline constexpr uint8_t     CEPS_TARGET_MAX            = 5;        // 켑스트럼(Cepstrum) 타겟 주파수 배열의 정적 메모리 할당 한계
            inline constexpr uint16_t    FIR_TAPS_MAX               = 127;      // FIR 필터 계수(Taps)의 정적 메모리 할당 한계 차수
            inline constexpr uint16_t    DELTA_HISTORY_MAX          = 10;       // 시계열 변화량(Delta) 연산을 위한 과거 프레임 보존 링버퍼 최대 크기
        }

        namespace Feature {
            inline constexpr uint8_t     BAND_RMS_DEF               = 8;        // 런타임에 실제 연산할 주파수 대역 기본 개수
            inline constexpr uint8_t     TOP_PEAKS_DEF              = 5;        // 런타임에 실제 추출할 상위 피크 기본 개수
            inline constexpr uint8_t     CEPS_TARGET_DEF            = 3;        // 런타임에 실제 감시할 켑스트럼 타겟 기본 개수
            inline constexpr uint16_t    FIR_TAPS_DEF               = 63;       // 런타임에 적용할 FIR 필터 기본 차수 (주파수 해상도 결정)
            inline constexpr uint16_t    DELTA_HISTORY_DEF          = 5;        // 런타임에 적용할 변화량(Delta) 과거 프레임 기본 보존 개수
            inline constexpr float       CEPS_TOLERANCE_DEF         = 0.0003f;  // 켑스트럼 피크 검출 시 허용되는 수학적 오차율(Tolerance) 기본값
        }

        namespace Trigger {
            inline constexpr uint32_t    HOLD_TIME_MS_DEF           = 5000;     // 트리거(이벤트) 조건 소멸 후 레코딩 세션을 유지할 지연 시간 (5초)
            inline constexpr bool        USE_DEEP_SLEEP_DEF         = false;    // 지정된 유휴 시간 경과 시 시스템 절전 모드 진입 여부
            inline constexpr uint32_t    SLEEP_SEC_DEF              = 300;      // 절전 모드 진입을 위한 이벤트 부재 대기 시간 (300초 = 5분)
            inline constexpr uint16_t    WAKE_DURATION_DEF          = 5;        // 하드웨어 모션 감지(Wake) 인터럽트를 유효로 인정할 최소 지속 시간 프레임
        }

        namespace Dsp {
            inline constexpr int         MEDIAN_WINDOW_MAX          = 15;       // 메디안(중간값) 필터 윈도우 배열의 정적 최대 크기 (홀수)
            inline constexpr int         MEDIAN_WINDOW_DEF          = 3;        // 스파이크 노이즈 제거용 메디안 필터 윈도우 기본 크기
            inline constexpr float       NOTCH_FREQ_HZ_DEF          = 60.0f;    // 노치 필터 기본 제거 대상 주파수 (주로 60Hz 교류 전원 노이즈 타겟팅)
            inline constexpr float       NOTCH_Q_FACTOR_DEF         = 30.0f;    // 노치 필터의 컷팅 대역폭을 결정하는 Q-Factor 기본값 (클수록 좁게 파임)
            inline constexpr float       NOTCH_2_FREQ_HZ_DEF        = 120.0f;   // 제2 노치 필터 기본 제거 대상 주파수 (주로 2차 고조파 노이즈 타겟팅)
            inline constexpr float       FIR_HPF_CUTOFF_DEF         = 10.0f;    // 고역 통과 필터(HPF)의 기본 컷오프 주파수 (DC 오프셋 및 초저주파 진동 차단용)
            inline constexpr float       PRE_EMPHASIS_ALPHA_DEF     = 0.97f;    // 프리엠파시스(고주파 강조) 필터의 가중치 계수
            inline constexpr float       WINDOW_AMPLITUDE_CORR_HANN = 2.0000f;  // Hann 윈도우 적용 시 손실된 진폭(Amplitude)을 보상하기 위한 수학적 교정 계수
            inline constexpr float       WINDOW_ENERGY_CORR_HANN    = 1.6330f;  // Hann 윈도우 적용 시 손실된 에너지(Energy)를 보상하기 위한 수학적 교정 계수
        }

        namespace Decision {
            inline constexpr uint8_t     MAX_TRIAL_COUNT_MAX        = 5;        // 일시적 노이즈로 인한 오작동 방지용 검증 로직의 최대 반복 횟수 한계
            inline constexpr uint8_t     MAX_TRIAL_COUNT_DEF        = 3;        // 이벤트 판정을 위해 상태를 검사할 기본 반복 횟수 (3회)
            inline constexpr float       STA_LTA_THRESHOLD_DEF      = 3.0f;     // 단기/장기 평균(STA/LTA) 기반 급격한 신호 변화 감지 임계치 (비율)
            inline constexpr int         MIN_TRIGGER_COUNT_DEF      = 1;        // 유효한 결함으로 인정하기 위한 최소 연속 트리거 발생 횟수
            inline constexpr float       VALID_START_SEC_DEF        = 0.30f;    // 설비 가동 초기 과도 응답(불안정 구간)을 배제하기 위한 유효 판정 시작 시간 (0.3초)
            inline constexpr float       VALID_END_SEC_DEF          = 0.50f;    // 판정을 종료하고 결론을 도출할 유효 판정 종료 시간 (0.5초)
        }
    }


    // ========================================================================
    // 3. Vib: 진동 특화 파라미터
    // ========================================================================
        namespace Vib {
        namespace Hardware {
            inline constexpr uint8_t     PIN_BMI_CS_CONST       = 10;           // BMI270 진동 센서 SPI Chip Select 핀
            inline constexpr uint8_t     PIN_BMI_INT1_CONST     = 11;           // BMI270 하드웨어 인터럽트(FIFO/모션 감지) 수신 핀
            inline constexpr uint32_t    SPI_FREQ_HZ_CONST      = 10000000;     // 진동 데이터 고속 전송을 위한 SPI 버스 속도 (10MHz)
            inline constexpr uint16_t    FIFO_BATCH_SIZE_MAX    = 64;           // 인터럽트 발생 시 읽어올 FIFO 버퍼 최대 상한 (OOM 방어)
            inline constexpr uint16_t    FIFO_BATCH_SIZE_DEF    = 32;           // 런타임에 실제로 한 번에 퍼올릴 FIFO 프레임 기본 개수
            inline constexpr uint8_t     FIFO_FRAME_SIZE_CONST  = 7;            // BMI270 FIFO 프레임 크기 (Header 1 + Accel 6)
            inline constexpr uint8_t     FIFO_HEADER_ACCEL_CONST= 0x84;         // BMI270 FIFO Accel 프레임 식별 헤더
            inline constexpr uint8_t     FIFO_HEADER_GYRO_CONST = 0x88;         // BMI270 FIFO Gyro 프레임 식별 헤더
            inline constexpr uint8_t     REG_FIFO_LEN_ADDR_CONST = 0x24;        // BMI270 FIFO 길이 레지스터 주소
            inline constexpr uint8_t     REG_FIFO_DATA_ADDR_CONST= 0x26;        // BMI270 FIFO 데이터 레지스터 주소
            inline constexpr float       ANY_MOTION_LSB_2G_MG   = 0.48f;        // BMI270 Any-motion 임계치 LSB (2G 기준, mg)
            inline constexpr uint32_t    STARTUP_DELAY_MS_CONST = 5;            // BMI270 가속도계 활성화 후 안정화 대기 시간 (ms)
        }
        namespace Sensor {
            inline constexpr bool        ACCEL_ENABLE_DEF       = true;         // 가속도 센서 연산 활성화 스위치 (기본 켜짐)
            inline constexpr bool        GYRO_ENABLE_DEF        = false;        // 자이로 센서 연산 활성화 스위치 (연산량 절감을 위해 기본 꺼짐)
            inline constexpr uint8_t     AXIS_MAX               = 3;            // 정적 배열 할당용 진동 센서 물리 축 최대 개수 (X, Y, Z)
            inline constexpr uint8_t     ACCEL_AXIS_COUNT_DEF   = 3;            // 런타임에 실제 연산할 축의 개수 (1축 단일 연산 vs 3축 병렬 연산)
            inline constexpr uint8_t     ACCEL_TARGET_AXIS_DEF  = 2;            // 1축 연산 모드일 경우 타겟이 되는 기본 물리 축 (0:X, 1:Y, 2:Z)
            inline constexpr uint8_t     GYRO_AXIS_COUNT_DEF    = 3;            // 런타임에 실제 연산할 축의 개수 (1축 단일 연산 vs 3축 병렬 연산)
            inline constexpr uint8_t     GYRO_TARGET_AXIS_DEF   = 2;            // 1축 연산 모드일 경우 타겟이 되는 기본 물리 축 (0:X, 1:Y, 2:Z)

            inline constexpr uint32_t    RATE_MAX               = 3200;         // 진동 센서 샘플링 레이트 정적 할당 최대치 (Hz)
            inline constexpr uint32_t    RATE_DEF               = 1600;         // 런타임 진동 샘플링 레이트 기본값 (1.6kHz)
            inline constexpr uint32_t    FFT_SIZE_MAX           = 1024;         // 진동 도메인 FFT 연산 버퍼의 정적 할당 최대치
            inline constexpr uint32_t    FFT_SIZE_DEF           = 256;          // 런타임 진동 FFT 윈도우 기본 사이즈 (주파수 해상도 결정)

            inline constexpr uint8_t     ACCEL_RANGE_DEF        = 8;            // 가속도 센서 측정 한계 범위 기본값 (±8G)
            inline constexpr uint8_t     ACCEL_ODR_DEF          = 0x0C;         // 가속도 센서 출력 데이터 레이트(ODR) 레지스터 설정값 (1.6kHz 매핑)
            inline constexpr uint8_t     GYRO_RANGE_DEF         = 2;            // 자이로 센서 측정 한계 범위 기본값 (±2000 dps)
        }
        namespace Trigger {
            inline constexpr float       RMS_THRESH_DEF         = 0.5f;         // 진동 기반 스마트 트리거 발동을 위한 전체 RMS 임계값
            inline constexpr float       WAKE_THRESH_G_DEF      = 1.0f;         // 하드웨어 칩셋 레벨 Any-Motion 감지 임계값 (1.0G)
            inline constexpr float       KURT_NG_THRESH_DEF     = 5.0f;         // 진동 파형 첨도 불량 임계치
            inline constexpr float       CREST_NG_THRESH_DEF    = 6.0f;         // 진동 파형 파고율 불량 임계치
            inline constexpr float       SKEW_NG_THRESH_DEF     = 1.5f;         // 진동 파형 비대칭성 불량 임계치

            // [시너지] Audio의 8-Band 에너지를 진동에 적용하여 저주파 회전체(베어링/기어) 결함 주파수를 대역별로 추적
            inline constexpr float       BAND_RANGES_DEF[Shared::FeatureLimit::BAND_RMS_MAX][2] = {
                {10.0f, 30.0f}, {30.0f, 60.0f}, {60.0f, 100.0f}, {100.0f, 250.0f},
                {250.0f, 400.0f}, {400.0f, 600.0f}, {600.0f, 800.0f}, {0.0f, 0.0f},
                {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f},
                {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}
            };
        }
        namespace FeatureLimit {
            // [계산식] 런타임 샘플링 레이트(RATE_DEF)에 종속시켜 매직넘버를 철폐한 STA/LTA 샘플 윈도우 크기
            inline constexpr uint32_t    STA_SAMPLES_DEF        = Sensor::RATE_DEF / 1000;
            inline constexpr uint32_t    LTA_SAMPLES_DEF        = Sensor::RATE_DEF / 100;
        }
        namespace Calib {
            inline constexpr char const* FILE_JSON_CONST        = "/sys/bmi_calib_243.json";    // 진동 센서 영점(Offset) 보정값 저장 경로
            inline constexpr uint16_t    TARGET_SAMPLES_DEF     = 100;          // 캘리브레이션 시 수집할 목표 샘플 개수
            inline constexpr uint16_t    RETRY_MAX_CONST        = 15;           // 캘리브레이션 샘플 수집 최대 재시도 횟수
            inline constexpr uint32_t    LOOP_DELAY_MS_CONST    = 20;           // 캘리브레이션 루프 내 대기 시간 (ms)
        }
    }

    // ========================================================================
    // 4. Audio: 소음 특화 파라미터
    // ========================================================================
    namespace Audio {
        namespace Hardware {
            inline constexpr int         I2S_PORT_NUM_CONST     = 0;            // ESP32 하드웨어 I2S 할당 포트 번호
            inline constexpr uint8_t     PIN_I2S_BCLK_CONST     = 41;           // I2S Bit Clock (BCLK) 핀
            inline constexpr uint8_t     PIN_I2S_MCLK_CONST     = 0;            // I2S Master Clock (MCLK) 핀 (일반적으로 미사용시 0)
            inline constexpr uint8_t     PIN_I2S_WS_CONST       = 42;           // I2S Word Select (LRCLK) 핀
            inline constexpr uint8_t     PIN_I2S_DIN_CONST      = 43;           // I2S Data In (마이크 수신) 핀
            inline constexpr int         DMA_BUF_COUNT_MAX      = 32;           // I2S DMA 수신 링버퍼 개수 상한 (정적 할당)
            inline constexpr int         DMA_BUF_COUNT_DEF      = 16;           // 런타임 I2S DMA 수신 링버퍼 기본 개수
            inline constexpr uint16_t    RAW_FRAME_BUFFERS_MAX  = 8;            // 오디오 원시 파형 핑퐁 버퍼 최대 슬롯 수
            inline constexpr uint16_t    RAW_FRAME_BUFFERS_DEF  = 4;            // 오디오 원시 파형 핑퐁 버퍼 기본 사용 개수
            inline constexpr float       PCM_32BIT_SCALE_CONST  = 4.656612873e-10f; // 32bit PCM 정규화를 위한 계수 (1 / 2^31)
        }
        namespace Sensor {
            // 오디오 채널 모드 타입 안정성 제어 상수
            inline constexpr uint8_t     CH_MODE_MONO_L_CONST   = 1;            // 좌측 마이크 단일 채널 모드
            inline constexpr uint8_t     CH_MODE_MONO_R_CONST   = 2;            // 우측 마이크 단일 채널 모드
            inline constexpr uint8_t     CH_MODE_STEREO_CONST   = 3;            // 좌/우 스테레오 채널 모드

            inline constexpr uint8_t     CHANNELS_MAX           = 2;            // 오디오 물리 채널 정적 할당 최대 개수
            inline constexpr uint8_t     CHANNEL_MODE_DEF       = CH_MODE_STEREO_CONST; // 런타임 마이크 채널 구동 기본 모드
            inline constexpr uint8_t     BITS_PER_SAMPLE_CONST  = 32;           // I2S 버스 샘플당 비트 심도 (ICS43434 32bit PCM)

            inline constexpr uint32_t    RATE_MAX               = 48000;        // 마이크 샘플링 레이트 정적 할당 최대치 (Hz)
            inline constexpr uint32_t    RATE_DEF               = 42000;        // 런타임 마이크 샘플링 레이트 기본값 (42kHz)
            inline constexpr uint32_t    FFT_SIZE_MAX           = 4096;         // 소음 도메인 FFT 연산 버퍼 정적 할당 최대치
            inline constexpr uint32_t    FFT_SIZE_DEF           = 1024;         // 런타임 소음 FFT 윈도우 기본 사이즈
        }
        namespace Trigger {
            inline constexpr float       RMS_THRESH_DEF         = 0.05f;        // 소음 기반 스마트 트리거 발동을 위한 RMS 임계값
            inline constexpr float       KURT_NG_THRESH_DEF     = 5.0f;         // 소음 파형 첨도 불량 임계치
            inline constexpr float       CREST_FACTOR_NG_DEF    = 4.0f;         // 소음 파형의 충격성(Crest Factor) 불량 임계치
            inline constexpr float       SKEWNESS_NG_DEF        = 2.0f;         // 소음 파형의 비대칭성(Skewness) 불량 임계치

            // 소음 도메인용 고주파 대역 다중 감시 (기계 마찰/리크 소음 타겟팅)
            inline constexpr float       BAND_RANGES_DEF[Shared::FeatureLimit::BAND_RMS_MAX][2] = {
                {100.0f, 500.0f}   , {500.0f, 1500.0f}   , {1500.0f, 4000.0f}  , {4000.0f, 8000.0f},
                {8000.0f, 12000.0f}, {12000.0f, 16000.0f}, {16000.0f, 20000.0f}, {0.0f, 0.0f},
                {0.0f, 0.0f}       , {0.0f, 0.0f}        , {0.0f, 0.0f}        , {0.0f, 0.0f},
                {0.0f, 0.0f}       , {0.0f, 0.0f}        , {0.0f, 0.0f}        , {0.0f, 0.0f}
            };
        }
        namespace Dsp {
            inline constexpr float       BEAMFORMING_GAIN_DEF    = 0.5f;        // 스테레오 마이크 빔포밍 연산 시 채널 합성 가중치
            inline constexpr float       WINDOW_MS_DEF           = 25.0f;       // STFT 연산을 위한 슬라이딩 윈도우 크기 (ms)
            inline constexpr float       HOP_MS_DEF              = 10.0f;       // STFT 연산을 위한 윈도우 이동 간격 (Overlap 결정)
            inline constexpr float       SPECTRAL_SUB_GAIN_DEF   = 1.2f;        // 스펙트럼 감산법 기반 노이즈 제거 강도 계수
            inline constexpr float       NOISE_LEARN_ALPHA_DEF   = 0.01f;       // 적응형 노이즈 프로파일 학습 계수 (Alpha)
            inline constexpr float       NOISE_GATE_THRESH_DEF   = 0.001f;      // 무음 구간 묵음 처리를 위한 노이즈 게이트 절대값 임계치
        }
        namespace FeatureLimit {
            inline constexpr uint32_t    MFCC_COEFFS_MAX         = 32;          // 단일 프레임 MFCC 계수 정적 할당 최대 차수
            inline constexpr uint32_t    MFCC_COEFFS_DEF         = 13;          // 런타임 MFCC 계수 추출 기본 차수
            inline constexpr uint32_t    MFCC_COMPONENTS_MAX     = 3;           // MFCC 시계열 변화량 조합 최대 차수 (Static, Delta, D-Delta)
            inline constexpr uint32_t    MFCC_COMPONENTS_DEF     = 3;           // 런타임 변화량 조합 기본값

            // [계산식] MFCC 차원 상수를 기반으로 전체 텐서 크기를 수식화 (SIMD 메모리 정렬 기준)
            // [정밀 교정] 진동 3축 + 오디오 2채널(L/R) = 총 5채널 기반 차원 재산출
            inline constexpr uint32_t    MFCC_DIM_MAX            = MFCC_COEFFS_MAX * MFCC_COMPONENTS_MAX * 5; // 32 * 3 * 5 = 480
            inline constexpr uint32_t    MFCC_DIM_DEF            = MFCC_COEFFS_DEF * MFCC_COMPONENTS_DEF * 5; // 13 * 3 * 5 = 195

            inline constexpr uint32_t    MEL_BANDS_MAX           = 40;          // Mel-Filterbank 정적 할당 최대 밴드 수
            inline constexpr uint32_t    MEL_BANDS_DEF           = 26;          // 런타임 Mel-Filterbank 기본 밴드 수

            inline constexpr float       MEL_SCALE_2595_CONST    = 2595.0f;     // 주파수를 Mel 스케일로 변환하기 위한 로그 계수
            inline constexpr float       MEL_SCALE_700_CONST     = 700.0f;      // 주파수를 Mel 스케일로 변환하기 위한 분모 상수

            inline constexpr uint16_t    MEL_CHUNK_ROWS_CONST    = 64;          // MFCC 행렬 연산 시 캐시 효율을 위한 SRAM 타일링 청크 크기

            // [계산식] 런타임 샘플링 레이트에 종속시켜 동적으로 계산된 소음 STA/LTA 샘플 수
            inline constexpr uint32_t    STA_SAMPLES_DEF         = Audio::Sensor::RATE_DEF / 1000;
            inline constexpr uint32_t    LTA_SAMPLES_DEF         = Audio::Sensor::RATE_DEF / 100;
        }
        namespace CalibLimit {
            inline constexpr float       GAIN_RATIO_MIN_CONST    = 0.707f;      // 자동 캘리브레이션 튜닝 게인 하한 방어선 (-3dB 보수적 접근)
            inline constexpr float       GAIN_RATIO_MAX_CONST    = 1.414f;      // 자동 캘리브레이션 튜닝 게인 상한 방어선 (+3dB 과보상 방어)
            inline constexpr uint32_t    WELCH_CHUNK_SAMPLES_DEF = Audio::Sensor::FFT_SIZE_DEF; // 스펙트럼 밀도 평균화를 위한 Welch 청크 기본 크기
            inline constexpr uint8_t     AUTO_STABLE_SEC_DEF     = 10;          // 자동 캘리브레이션 시 백그라운드 노이즈 안정화 판정 시간 (초)
        }
        namespace Calib {
            inline constexpr char const* FILE_JSON_CONST         = "/sys/aud_calib_243.json"; // 마이크 EQ 보정 파일 저장 경로
            inline constexpr uint32_t    AUTO_IDLE_MIN_DEF       = 60;          // 설정 시간 동안 트리거가 없으면 자동 캘리브레이션 진입 (분)
            inline constexpr float       REF_FREQ_HZ_DEF         = 1000.0f;     // 마이크 보정 기준 타겟 주파수 (1kHz)
            inline constexpr float       FILTER_MIN_FREQ_HZ_DEF  = 100.0f;      // 평탄화(EQ) 보정 대상 최소 주파수 하한
            inline constexpr float       FILTER_MAX_FREQ_HZ_DEF  = 8000.0f;     // 평탄화(EQ) 보정 대상 최대 주파수 상한
            inline constexpr float       TARGET_GAIN_MAX_DEF     = 3.0f;        // FIR EQ 필터의 증폭(Gain) 물리적 상한치 제한
            inline constexpr float       TARGET_GAIN_MIN_DEF     = 0.3f;        // FIR EQ 필터의 감쇠(Gain) 물리적 하한치 제한
            inline constexpr float       NORM_SAFE_THRESH_DEF    = 1.5f;        // 필터 계수 정규화 시 무한 증폭을 막는 안전 마진 임계값
        }
    }

}
