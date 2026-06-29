

/* ============================================================================
 * File: T215_Type_250.hpp
 * Summary: 4-Tier 데이터 타입 및 패킷
 * ============================================================================
 * [시스템 구현 원칙 - Core Principles]
 * 1. [도메인 격리]: UnifiedRawChunk를 해체하고 Accel, Gyro, Audio 고유 주율(Hz)에 맞춘 독립 버퍼.
 * 2. [종속성 분리]: 공통 Shared DSP를 제거하고, 각 도메인 및 물리 축/채널별 독립 필터 할당.
 * 3. [메모리 최적화]: 비트 마스크(AxisMask, ChannelMask)를 도입하여 메모리 동적 최적화.
 * 4. [SIMD 및 네트워크 정렬]: 연산 슬롯 alignas(16) 완벽 보장 및 패킷 1바이트 팩킹 유지.
 * 5. [매직넘버 제로]: 모든 배열 크기는 T210_Def_250.hpp의 SSOT 상수를 상속.
 * ========================================================================== */

#pragma once

#include <atomic>
#include <cstdint>
#include <bitset>
#include "T210_Def_250.hpp"


//// using namespace T2_Def;

namespace T2_Type {

// ========================================================================
// [PART 1] 시스템 열거형 (Enum Classes)
// ========================================================================

// 시스템 상태 열거형
enum class EM_SystemState_t : uint8_t {
    INIT           = 0,         // 초기화
    READY,                      // 준비
    MONITORING,                 // 모니터링
    RECORDING,                  // 녹음
    NOISE_LEARNING,             // 노이즈 학습
    MAINTENANCE,                // 유지보수
    ERROR,                      // 에러
    CALIBRATING,                // 교정
	COUNT						// 시스템 상태 개수
};

// 운용 모드 열거형
enum class EM_OpMode_t : uint8_t {
    MANUAL         = 0,         // 수동
    AUTO,                       // 자동
    SCHEDULE,                   // 예약
	COUNT						// 오퍼레이션 모드 개수
};

// 축 비트마스크 열거형
enum class EM_AxisMask_t : uint8_t {
    NONE           = 0,         // 없음
    AXIS_X         = 1 << 0,    // 0b001
    AXIS_Y         = 1 << 1,    // 0b010
    AXIS_Z         = 1 << 2,    // 0b100
    AXIS_ALL       = 0b111,     // 전체
	COUNT						// 축 마스크 개수
};

// 채널 마스크 열거형
enum class EM_ChannelMask_t : uint8_t {
    NONE           = 0,         // 없음
    CH_LEFT        = 1 << 0,    // 0b01
    CH_RIGHT       = 1 << 1,    // 0b10
    CH_STEREO      = 0b11,      // 전체 스테레오
	COUNT						// 채널 마스크 개수
};

// 데이터 페이로드 타입 열거형
enum class EM_DataPayloadType_t : uint8_t {
    VIB_ONLY       = 1,         // 진동 데이터만
    AUDIO_ONLY     = 2,         // 오디오 데이터만
    VIB_AUDIO_BOTH = 3,         // 진동 + 오디오 데이터 모두
	COUNT						// 데이터 페이로드 타입 개수
};

// 상태 비트 열거형
enum class EM_StatusBit_t : uint8_t {
    NTP_SYNCED     = 0,         // NTP 시간 동기화 여부
    SD_MOUNTED     = 1,         // SD 카드 마운트 여부
    RECORDING_NOW  = 2,         // 녹음 중 여부
    SENSOR_FAULT   = 3,         // 센서 오류 여부
	COUNT						// 상태 비트 개수
};

// 판정 결과 열거형
enum class EM_DetectionResult_t : uint8_t {
    PASS           = 0,         // 합격
    RULE_VIB_NG,                // 진동 규칙 위반
    RULE_AUDIO_NG,              // 오디오 규칙 위반
    TEST_NG,                    // 테스트 위반
    ML_NG,                      // ML 규칙 위반
	COUNT						// 판정 결과 개수
};

// 시스템 커맨드 열거형
enum class EM_SystemCommand_t : uint8_t {
    CMD_START          = 0,     // 시작
    CMD_STOP,                   // 중지
    CMD_LEARN_NOISE,            // 노이즈 학습
    CMD_CALIBRATE,              // 교정
    CMD_REBOOT,                 // 재부팅
    CMD_OTA_START,              // OTA 시작
    CMD_OTA_END,                // OTA 종료
    CMD_MANUAL_REC_START,       // 수동 녹음 시작
    CMD_MANUAL_REC_STOP,        // 수동 녹음 중지
    CMD_TUNING_PREVIEW,         // 튜닝 미리보기
    CMD_TUNING_SAVE,            // 튜닝 저장
    CMD_TUNING_CANCEL,          // 튜닝 취소
	COUNT						// 시스템 커맨드 개수
};

// 비동기 세션 커맨드 열거형
enum class EM_AsyncSessionCmd_t : uint8_t {
    NONE = 0,                   // 없음
    OPEN_AUTO,                  // 자동 개방
    OPEN_MANUAL,                // 수동 개방
    OPEN_CALIB_MAN,             // 교정 수동 개방
    CLOSE_NORMAL,               // 정상 닫힘
    CLOSE_MANUAL,               // 수동 닫힘
    CLOSE_CALIB_DONE,           // 교정 완료 닫힘
	COUNT						// 비동기 세션 개수
};

// 트리거 소스 열거형
enum class EM_TriggerSource_t : uint8_t {
    NONE           = 0,         // 없음
    HW_WAKE,                    // 하드웨어 웨이크업
    SW_RMS,                     // 소프트웨어 RMS
    SW_BAND,                    // 소프트웨어 주파수 대역
    MANUAL,                     // 수동
	COUNT						// 트리거 소스 개수
};

// 스트림 타입 열거형
enum class EM_StreamType_t : uint8_t {
    TELEMETRY      = 0x01,      // 텔레메트리
    SPECTRUM       = 0x02,      // 스펙트럼
    WAVEFORM       = 0x03,      // 파형
    CALIBRATION    = 0x04,      // 교정
    SEQUENCE       = 0x05,      // 시퀀스
	COUNT						// 스트림 타입 개수
};

// 노이즈 모드 열거형
enum class EM_NoiseMode_t : uint8_t {
    OFF            = 0,         // 끔
    FIXED,                      // 고정
    ADAPTIVE,                   // 적응형
	COUNT						// 노이즈 모드 개수
};

// 와이파이 모드 열거형
enum class EM_WiFiMode_t : uint8_t {
    STA_ONLY       = 0,         // STA 모드만
    AP_ONLY,                    // AP 모드만
    AP_STA,                     // AP + STA 모드
    AUTO_FALLBACK,              // 자동 폴백
	COUNT						// 와이파이 모드 개수
};

// 윈도우 타입 열거형
enum class EM_WindowType_t : uint8_t {
    HANN           = 0,         // 해닝 윈도우
    HAMMING,                    // 해밍 윈도우
    BLACKMAN,                   // 블랙맨 윈도우
	COUNT						// 윈도우 타입 개수
};


// ========================================================================
// [PART 2] 동적 설정 구조체 (런타임 제어 및 스위치 기능 통합)
// ========================================================================

// ------------------------------------------------------------------------
// Tier 1. Global (디바이스 전역 인프라)
// ------------------------------------------------------------------------

// 와이파이 설정 구조체
struct ST_Global_WiFi_t {
    EM_WiFiMode_t mode;                                               // Wi-Fi 동작 모드 (STA, AP, AP_STA, AUTO_FALLBACK 등)
    char          ap_ssid[T2_Def::Global::NetLimit::NET_SSID_LEN_MAX]; // 자체 AP 모드 구동 시 사용할 SSID
    char          ap_pw[T2_Def::Global::NetLimit::NET_PW_LEN_MAX];     // 자체 AP 모드 구동 시 사용할 비밀번호
    char          ap_ip[T2_Def::Global::NetLimit::NET_IP_LEN_MAX];     // 자체 AP 모드의 기본 게이트웨이 IP 주소
    char          multi_ssid[T2_Def::Global::NetLimit::NET_MULTI_AP_MAX][T2_Def::Global::NetLimit::NET_SSID_LEN_MAX]; // 예비 접속용 외부 AP SSID 리스트
    char          multi_pw[T2_Def::Global::NetLimit::NET_MULTI_AP_MAX][T2_Def::Global::NetLimit::NET_PW_LEN_MAX];     // 예비 접속용 외부 AP 비밀번호 리스트
    uint32_t      disconnect_delay_ms;                                // [이슈 9] Wi-Fi 연결 해제 후 재연결 시도 전 대기 시간 (ms)
};

// MQTT 설정 구조체
struct ST_Global_Mqtt_t {
    bool     enable;                                                  // MQTT 통신 기능 활성화 여부
    char     broker[T2_Def::Global::NetLimit::NET_BROKER_LEN_MAX];    // MQTT 브로커 서버 도메인 또는 IP 주소
    uint16_t port;                                                    // MQTT 브로커 서비스 포트 번호 (기본 1883)
    char     id[T2_Def::Global::NetLimit::NET_ID_LEN_MAX];            // MQTT 클라이언트 식별자 ID
    char     pw[T2_Def::Global::NetLimit::NET_PW_LEN_MAX];            // MQTT 클라이언트 접속 인증 비밀번호
    char     topic_root[T2_Def::Global::NetLimit::NET_TOPIC_LEN_MAX]; // MQTT 데이터 발행/구독 최상위 루트 토픽명
    char     lwt_topic[T2_Def::Global::NetLimit::NET_TOPIC_LEN_MAX];  // LWT(Last Will and Testament) 비정상 연결 종료 유언 등록 토픽명
    uint8_t  qos;                                                     // MQTT 메시지 발행 품질 등급 (QoS 0, 1, 2)
    uint8_t  proto_ver;                                               // MQTT 프로토콜 버전 (예: 4 = v3.1.1)
};

// NTP 설정 구조체
struct ST_Global_NTP_t {
    char     ntp_server1[T2_Def::Global::NetLimit::NET_BROKER_LEN_MAX];   // 주 시간 동기화 NTP 서버 주소
    char     ntp_server2[T2_Def::Global::NetLimit::NET_BROKER_LEN_MAX];   // 보조 시간 동기화 NTP 서버 주소
    char     ntp_tz[32];                                                  // 타임존 환경 변수 설정 문자열 (예: KST-9)
};

// 저장장치 설정 구조체
struct ST_Global_Storage_t {
    uint32_t  rot_mb;                  // 파일 순환 저장 임계 크기 (단위: MB)
    uint32_t  rot_min;                 // 파일 순환 저장 임계 시간 (단위: 분)
    bool      save_raw;                // 진동/각속도 원시 파형 데이터의 영속 파일 저장 활성화 여부
    uint16_t  keep_max;                // 유지할 수 있는 최대 보존 파일 개수 (초과 시 오래된 파일 순환 삭제)
    uint32_t  idle_flush_ms;           // 디스크 지연 쓰기 시 강제 플러시 수행 임계 대기 시간 (ms)
    uint8_t   pre_trig_sec;            // 트리거 감지 시점 기준 이전으로 역산해 저장할 Pre-Trigger 보존 기간 (초)
    bool      enable_auto_format;       // 저장 매체 파티션 에러 또는 손상 시 자동 포맷 실행 허용 여부
    bool      enable_raw_audio_saving;  // 오디오 원시 파형(.wav)의 로컬 디스크 파일 영속 저장 스위치
};

// 시스템 설정 구조체
struct ST_Global_System_t {
    char          site_id[T2_Def::Global::System::SITE_ID_LEN_MAX];  // 센서 노드가 물리적으로 설치된 공정/라인 등 현장 식별자
    uint8_t       tele_hz;                                           // 상태 보고 및 특징량 데이터의 텔레메트리 송출 속도 주기 (Hz)
    uint8_t       wave_hz;                                           // 웹소켓을 통한 원시 파형 실시간 스트리밍 송출 주기 (Hz)
    EM_OpMode_t   op_mode;                                           // 장비 운용 모드 (수동 제어, 자동 감시, 스케줄 운용)
    uint32_t      watchdog_ms;                                       // 내부 스레드들의 주기적 상태 검사용 소프트웨어 워치독 감시 주기 (ms)
};

// MLOps 의사결정/트리거 파라미터 동적 제어 구조체
struct ST_Global_Decision_t {
    uint8_t max_trial_count;     // 노이즈 오작동 검증 최대 횟수 (기본: MAX_TRIAL_COUNT_DEF)
    float   sta_lta_threshold;   // 단기/장기 급변 감지 비율 임계치 (기본: STA_LTA_THRESHOLD_DEF)
    int     min_trigger_count;   // 유효 판정 최소 연속 트리거 (기본: MIN_TRIGGER_COUNT_DEF)
    float   valid_start_sec;     // 유효 판정 시작 시간 초 (기본: VALID_START_SEC_DEF)
    float   valid_end_sec;       // 판정 종료 유효 시간 초 (기본: VALID_END_SEC_DEF)
};

// 출력 설정 구조체
struct ST_Global_Output_t {
    bool     enabled;            // 이벤트 발생 시 외부 출력(LED 점등, GPIO 토글 등) 활성화 여부
    bool     output_sequence;    // 유효한 이벤트 판정 시 진동 파형 시퀀스 데이터를 외부로 송출(Push/Stream)할지 여부
    uint16_t sequence_frames;    // 출력할 파형 시퀀스 데이터의 프레임 수
};

// ------------------------------------------------------------------------
// 필터 구조체 원형 (각 도메인 내부에 종속되어 선언됨)
// ------------------------------------------------------------------------

struct ST_FilterFIR_t {
    bool     en;                    // FIR 필터 활성화 여부
    float    cutoff;                // FIR 필터 컷오프 주파수
    uint16_t taps;                  // FIR 필터 탭 수
};

struct ST_FilterIIR_t {
    bool  en;                       // IIR 필터 활성화 여부
    float cutoff;                   // IIR 필터 컷오프 주파수
    float q;                        // IIR 필터 Q값
};

struct ST_FilterNotch_t {
    bool  en;					    // 노이즈 필터 활성화 여부
    float freq;						// 노이즈 필터 주파수
    float gain;						// 노이즈 필터 게인
    float q;						// 노이즈 필터 Q값
};

struct ST_Dsp_Config_t {
    bool             rem_dc;        // [개선 14] DC 제거(High Pass)
    bool             med_en;        // 중앙값 필터 사용
    uint8_t          med_win;       // 중앙값 필터 윈도우 크기
    ST_FilterFIR_t   hpf;           // FIR 하이패스 필터
    ST_FilterFIR_t   lpf;           // FIR 로우패스 필터
    ST_FilterIIR_t   iir_hpf;       // IIR 하이패스 필터
    ST_FilterIIR_t   iir_lpf;       // IIR 로우패스 필터
    ST_FilterNotch_t notch;         // 노치 필터
    ST_FilterNotch_t notch2;        // 노치 필터2
    EM_WindowType_t  win_type;      // 윈도우 타입
};


// ------------------------------------------------------------------------
// Tier 2. Accel (가속도 전용 파이프라인)
// ------------------------------------------------------------------------

struct ST_Accel_Config_t {
    bool     enable;             // 가속도 센서 사용 활성화
    uint8_t  axis_mask;          // 가속도 축 비트마스크
    uint8_t  range;              // 가속도 센서 Full Scale Range
    uint8_t  odr;                // 가속도 센서 출력 데이터 속도(ODR)

    bool     enable_mfcc;        // MFCC 계산 사용 여부
    bool     enable_fft;         // FFT 계산 사용 여부
    bool     enable_timbre;      // 고유한 기계 진동 특성(음색) 계산 사용 여부
    bool     enable_band_energy; // 주파수 대역별 에너지 분석 사용 여부

    // BMI270 드라이버 하드웨어 필터 세부 속성 직렬화 멤버 확장
    uint8_t  bwp;                // BMI2_ACC_NORMAL_AVG4 바인딩
    uint8_t  filter_perf;        // BMI2_PERF_OPT_MODE 바인딩
    uint16_t fifo_watermark;     // watermark = 40 바인딩

    uint32_t sample_rate;        // 가속도 데이터 샘플링 속도 (Hz)
    uint32_t fft_size;           // FFT 연산에 사용할 데이터 샘플 수

    bool     motion_en;          // 동적 활동 감지(Motion Detection) 기능 사용 여부
    float    wake_g;             // 활동 감지 트리거 임계값 (g 단위)
    uint16_t wake_dur;           // 활동 지속 시간 체크 윈도우 (ms)

    uint8_t  noise_perf;  				// 가속도 저소음 모드 제어 (BMI270 bwp 파라미터 연동)

    float    rms_thresh[T2_Def::Accel::Sensor::AXIS_MAX];    // RMS 진폭 임계값 (g 단위)
    float    kurt_ng_thresh[T2_Def::Accel::Sensor::AXIS_MAX];  // 첨도(Kurtosis) 비정상 진단 임계값
    float    crest_ng_thresh[T2_Def::Accel::Sensor::AXIS_MAX]; // Crest Factor 비정상 진단 임계값
    float    skew_ng_thresh[T2_Def::Accel::Sensor::AXIS_MAX];  // 비대칭도(Skewness) 비정상 진단 임계값

    uint8_t  active_band_count;    // 유효한 주파수 대역 개수
    std::bitset<16> band_en;       // 주파수 대역 활성화 비트맵
    float    band_start[T2_Def::Accel::FeatureLimit::BAND_MAX];  // 주파수 대역 시작주파수 (Hz)
    float    band_end[T2_Def::Accel::FeatureLimit::BAND_MAX];    // 주파수 대역 종료주파수 (Hz)
    float    band_thresh[T2_Def::Accel::Sensor::AXIS_MAX][T2_Def::Accel::FeatureLimit::BAND_MAX]; // 주파수 대역별 에너지 임계값 (g 단위)

    float    offset[T2_Def::Accel::Sensor::AXIS_MAX];          // 축별 오프셋 보정값 (g 단위)
    float    gain[T2_Def::Accel::Sensor::AXIS_MAX];            // 축별 게인 보정값

    float    peak_amp_min;                                    // 피크 진폭 최소 임계값 (g 단위)
    float    peak_freq_gap_min;                               // 피크 주파수 간격 최소 임계값 (Hz)

    ST_Dsp_Config_t dsp;                                      // DSP 처리 공통 설정
};


// ------------------------------------------------------------------------
// Tier 3. Gyro (자이로 전용 파이프라인)
// ------------------------------------------------------------------------

struct ST_Gyro_Config_t {
    bool     enable;            // 자이로 센서 사용 활성화
    uint8_t  axis_mask;         // 자이로 축 비트마스크
    uint8_t  range;             // 자이로 센서 Full Scale Range
    uint8_t  odr;               // 자이로 센서 출력 데이터 속도(ODR)

    bool     enable_mfcc;       // MFCC 계산 사용 여부
    bool     enable_fft;        // FFT 계산 사용 여부
    bool     enable_timbre;     // 고유한 기계 진동 특성(음색) 계산 사용 여부
    bool     enable_band_energy; // 주파수 대역별 에너지 분석 사용 여부

    // BMI270 드라이버 하드웨어 필터 세부 속성 직렬화 멤버 확장
    uint8_t  bwp;               // BMI2_GYR_NORMAL_MODE 바인딩
    uint8_t  filter_perf;       // 성능 최적화 모드 바인딩
    uint8_t  noise_perf;        // 노이즈 최적화 모드 바인딩

    uint32_t sample_rate;       // 자이로 데이터 샘플링 속도 (Hz)
    uint32_t fft_size;          // FFT 연산에 사용할 데이터 샘플 수

    float    rms_thresh[T2_Def::Gyro::Sensor::AXIS_MAX];    // RMS 진폭 임계값 (DPS 단위)
    float    kurt_ng_thresh[T2_Def::Gyro::Sensor::AXIS_MAX];  // 첨도(Kurtosis) 비정상 진단 임계값
    float    crest_ng_thresh[T2_Def::Gyro::Sensor::AXIS_MAX]; // Crest Factor 비정상 진단 임계값
    float    skew_ng_thresh[T2_Def::Gyro::Sensor::AXIS_MAX];  // 비대칭도(Skewness) 비정상 진단 임계값

    uint8_t  active_band_count;    // 유효한 주파수 대역 개수
    std::bitset<16> band_en;       // 주파수 대역 활성화 비트맵
    float    band_start[T2_Def::Gyro::FeatureLimit::BAND_MAX];  // 주파수 대역 시작주파수 (Hz)
    float    band_end[T2_Def::Gyro::FeatureLimit::BAND_MAX];    // 주파수 대역 종료주파수 (Hz)
    float    band_thresh[T2_Def::Gyro::Sensor::AXIS_MAX][T2_Def::Gyro::FeatureLimit::BAND_MAX]; // 주파수 대역별 에너지 임계값 (DPS 단위)

    float    offset[T2_Def::Gyro::Sensor::AXIS_MAX];          // 축별 오프셋 보정값 (DPS 단위)
    float    gain[T2_Def::Gyro::Sensor::AXIS_MAX];            // 축별 게인 보정값

    float    peak_amp_min;                                    // 피크 진폭 최소 임계값 (DPS 단위)
    float    peak_freq_gap_min;                               // 피크 주파수 간격 최소 임계값 (Hz)

    ST_Dsp_Config_t dsp;                                      // DSP 처리 공통 설정
};


// ------------------------------------------------------------------------
// Tier 4. Audio (소음 전용 파이프라인)
// ------------------------------------------------------------------------

struct ST_Audio_Noise_t {
    bool           gate_en;        // 노이즈 게이트 활성화 여부
    float          gate_thresh;    // 노이즈 게이트 임계값 (dB SPL)
    EM_NoiseMode_t mode;           // 노이즈 감지 모드 (Sub Band / Adaptive)
    float          sub_str;        // Sub Band Mode 기준 세기 (dB SPL)
    float          adp_alpha;      // Adaptive Mode 학습 계수 (0.0 ~ 1.0)
    uint16_t       learn_frames;   // 초기 학습 프레임 수 (노이즈 레벨 학습용)
};

struct ST_Audio_Config_t {
    bool     enable;              // 소음 분석 모듈 사용 활성화
    uint8_t  channel_mask;        // 사용할 마이크 채널 비트마스크 (1: Mic0, 2: Mic1, 3: Both)
    uint32_t sample_rate;         // ADC 샘플링 속도 (Hz)
    uint32_t fft_size;            // FFT 연산에 사용할 데이터 샘플 수

    bool     enable_mfcc;         // MFCC 계산 사용 여부
    bool     enable_fft;          // FFT 계산 사용 여부
    bool     enable_timbre;       // 고유한 기계 진동 특성(음색) 계산 사용 여부
    bool     enable_band_energy;  // 주파수 대역별 에너지 분석 사용 여부

    float    rms_thresh[T2_Def::Audio::Sensor::CHANNELS_MAX];      // RMS 진폭 임계값 (dB SPL)
    float    kurt_ng_thresh[T2_Def::Audio::Sensor::CHANNELS_MAX];  // 첨도(Kurtosis) 비정상 진단 임계값
    float    crest_ng_thresh[T2_Def::Audio::Sensor::CHANNELS_MAX]; // Crest Factor 비정상 진단 임계값
    float    skew_ng_thresh[T2_Def::Audio::Sensor::CHANNELS_MAX];  // 비대칭도(Skewness) 비정상 진단 임계값

    uint8_t  active_band_count;    // 유효한 주파수 대역 개수
    std::bitset<16> band_en;        // 주파수 대역 활성화 비트맵
    float    band_start[T2_Def::Audio::FeatureLimit::BAND_MAX];  // 주파수 대역 시작주파수 (Hz)
    float    band_end[T2_Def::Audio::FeatureLimit::BAND_MAX];    // 주파수 대역 종료주파수 (Hz)
    float    band_thresh[T2_Def::Audio::Sensor::CHANNELS_MAX][T2_Def::Audio::FeatureLimit::BAND_MAX]; // 주파수 대역별 에너지 임계값 (dB SPL)

    ST_Audio_Noise_t noise;        // 노이즈 분석 설정
    bool     pre_en;               // 사전 감쇠(Preprocessing) 사용 여부
    float    pre_alpha;            // 사전 감쇠 계수
    float    beam_gain;            // 빔포밍 이득
    float    window_ms;            // 윈도우 크기 (ms)
    float    hop_ms;               // 홉 크기 (ms)

    uint32_t auto_idle_min;         // 오디오 자동 정지 시간 (ms)
    float    ref_freq;             // 레퍼런스 주파수 (Hz)
    float    filt_min;             // 필터 최소 주파수 (Hz)
    float    filt_max;             // 필터 최대 주파수 (Hz)
    float    gain_max;             // 게인 최대치 (dB)
    float    gain_min;             // 게인 최소치 (dB)
    float    norm_safe;            // 정규화 안전 범위
    float    gain_ch[T2_Def::Audio::Sensor::CHANNELS_MAX]; // 채널별 게인
    alignas(16) float eq_coeffs[T2_Def::Audio::Sensor::CHANNELS_MAX][T2_Def::Audio::FeatureLimit::FIR_TAPS_MAX]; // EQ 계수

	uint8_t  melband_size;         // 멜 필터 뱅크 개수

    uint8_t  active_ceps_count;    // 유효한 MFCC 개수
    uint8_t  active_peak_count;    // 유효한 피크 개수
    float    ceps_targets[T2_Def::Audio::FeatureLimit::CEPS_TARGET_MAX]; // MFCC 타겟
    float    f_min;                // 최소 주파수 (Hz)
    float    f_max;                // 최대 주파수 (Hz)
    float    peak_amp_min;         // 피크 진폭 최소 임계값 (dB)
    float    peak_freq_gap_min;    // 피크 주파수 간격 최소 임계값 (Hz)

    ST_Dsp_Config_t dsp; // [수정] dsp[2] 채널별 배열을 해체하고 도메인 공통 단일 구조체로 축소하여 메모리 절약
};


// ------------------------------------------------------------------------
// 마스터 동적 설정 구조체 (SSOT)
// ------------------------------------------------------------------------
struct ST_DynamicConfig_t {
    ST_Global_System_t   system;     // 시스템 공통 설정
    ST_Global_WiFi_t     wifi;       // Wi-Fi 설정
    ST_Global_Mqtt_t     mqtt;       // MQTT 설정
	ST_Global_NTP_t		 ntp;        // NTP 설정
    ST_Global_Storage_t  storage;    // 저장 설정
    ST_Global_Output_t   output;     // 출력 설정
    ST_Global_Decision_t decision;   // MLOps 의사결정 파라미터 동적 제어

    ST_Accel_Config_t    accel;      // 가속도 센서 설정
    ST_Gyro_Config_t     gyro;       // 자이로 센서 설정
    ST_Audio_Config_t    audio;      // 오디오 센서 설정

    uint32_t             trig_hold_ms;         // 트리거 홀드 시간 (ms)
    bool                 trig_use_sleep;       // 트리거 슬립 사용 여부
    uint32_t             trig_sleep_sec;       // 트리거 슬립 시간 (초)
    bool                 enable_advanced_physics_compensation; // 고급 물리 보정(온도, 속도 적분, 레버암 등) 일괄 제어
    uint32_t             revision;             // 설정 데이터 Revision
    uint32_t             config_version;       // 설정 데이터 버전
};

// ------------------------------------------------------------------------
// 런타임 제어 컨텍스트
// ------------------------------------------------------------------------
struct ST_Runtime_TriggerCtx_t {
    bool               is_triggered;        // 트리거 발생 여부
    EM_TriggerSource_t active_source;       // 트리거 소스
    uint32_t           hold_end_tick;       // 트리거 홀드 종료 시간
};


// ============================================================================
// [PART 3] 원시 데이터(Raw) 및 진단 특징량 데이터 모델 (16-Byte Aligned)
// ============================================================================

// 가속도 원시 데이터 구조체
struct alignas(16) ST_Raw_Accel_t {
    uint64_t ts;                            // 타임스탬프
    uint32_t sample_rate;                   // 샘플링 속도
    uint8_t  active_mask;                   // 활성화 마스크
    float    data[T2_Def::Accel::Sensor::AXIS_MAX][T2_Def::Accel::Sensor::FFT_SIZE_MAX];
};

// 가속도 원시 데이터 구조체
struct alignas(16) ST_Raw_Gyro_t {
    uint64_t ts;                            // 타임스탬프
    uint32_t sample_rate;                   // 샘플링 속도
    uint8_t  active_mask;                   // 활성화 마스크
    float    data[T2_Def::Gyro::Sensor::AXIS_MAX][T2_Def::Gyro::Sensor::FFT_SIZE_MAX]; // 자이로 데이터
};

// 오디오 원시 데이터 구조체
struct alignas(16) ST_Raw_Audio_t {
    uint64_t ts;                            // 타임스탬프
    uint32_t sample_rate;                   // 샘플링 속도
    uint8_t  active_mask;                   // 활성화 마스크
    float    data[T2_Def::Audio::Sensor::CHANNELS_MAX][T2_Def::Audio::Sensor::FFT_SIZE_MAX]; // 오디오 데이터
};

// 스펙트럼 피크 데이터 구조체
struct ST_SpectralPeak_t {
    float freq;                 // 피크 주파수
    float amp;                  // 피크 진폭
};

// [Block 1] 공통 헤더 (32 Bytes 완벽 정렬)
struct ST_Slot_Header_t {
    uint64_t ts;                // 타임스탬프
    uint32_t fid;               // 파일 ID
    uint32_t uptime;            // 부팅 시간
    uint8_t  flags;             // 플래그
    uint8_t  trial;             // 트라이얼
    uint8_t  src;               // 소스
    uint8_t  accel_mask;        // 가속도 센서 마스크
    uint8_t  gyro_mask;         // 자이로 센서 마스크
    uint8_t  audio_mask;        // 오디오 센서 마스크
    uint8_t  payload_type;      // 페이로드 타입
    uint8_t  _pad;              // 패딩
    float    temp;              // 온도
    uint8_t  _res[4];           // 예약
};

// [Block 2] 가속도 지표 (304 Bytes 완벽 정렬)
struct ST_Slot_Accel_t {
    float rms[T2_Def::Accel::Sensor::AXIS_MAX];             // 평균 제곱근
    float peak_f[T2_Def::Accel::Sensor::AXIS_MAX];          // 피크 주파수
    float centroid[T2_Def::Accel::Sensor::AXIS_MAX];        // 주파수 중심
    float kurt[T2_Def::Accel::Sensor::AXIS_MAX];            // 첨도
    float crest[T2_Def::Accel::Sensor::AXIS_MAX];           // 파고도
    float sta_lta[T2_Def::Accel::Sensor::AXIS_MAX];         // 단기-장기 평균 비율
    float skew[T2_Def::Accel::Sensor::AXIS_MAX];            // 왜도
    float std[T2_Def::Accel::Sensor::AXIS_MAX];             // 표준 편차
    float cal_off[T2_Def::Accel::Sensor::AXIS_MAX];         // 캘리브레이션 오프셋
    float band_energy[T2_Def::Accel::Sensor::AXIS_MAX][T2_Def::Accel::FeatureLimit::BAND_MAX];  // 주파수 대역별 에너지
    float _pad[1];                                          // 패딩 4 Bytes 추가 (300 + 4 = 304 Bytes, 16배수 정렬)
};

// [Block 3] 자이로 지표 (224 Bytes 완벽 정렬)
struct ST_Slot_Gyro_t {
    float rms[T2_Def::Gyro::Sensor::AXIS_MAX];              // 평균 제곱근
    float peak_f[T2_Def::Gyro::Sensor::AXIS_MAX];           // 피크 주파수
    float centroid[T2_Def::Gyro::Sensor::AXIS_MAX];         // 주파수 중심
    float kurt[T2_Def::Gyro::Sensor::AXIS_MAX];             // 첨도
    float crest[T2_Def::Gyro::Sensor::AXIS_MAX];            // 파고도
    float sta_lta[T2_Def::Gyro::Sensor::AXIS_MAX];          // 단기-장기 평균 비율
    float skew[T2_Def::Gyro::Sensor::AXIS_MAX];             // 왜도
    float std[T2_Def::Gyro::Sensor::AXIS_MAX];              // 표준 편차
    float cal_off[T2_Def::Gyro::Sensor::AXIS_MAX];          // 캘리브레이션 오프셋
    float drift_est[T2_Def::Gyro::Sensor::AXIS_MAX];        // 자이로 특화 드리프트 지표
    float band_energy[T2_Def::Gyro::Sensor::AXIS_MAX][T2_Def::Gyro::FeatureLimit::BAND_MAX]; // 주파수 대역별 에너지
    float _pad[2];                                          // 패딩 8 Bytes 추가 (216 + 8 = 224 Bytes, 16배수 정렬)
};

// [Block 4] 오디오 단일 채널 전용 (220 Bytes)
struct ST_Audio_Channel_Slot_t {
    float        rms;                                       // 평균 제곱근
    float        energy;                                      // 에너지
    float        centroid;                                    // 주파수 중심
    float        kurt;                                        // 첨도
    float        crest;                                       // 파고도
    float        skew;                                        // 왜도
    float        sta_lta;                                     // 단기-장기 평균 비율
    float        d_rms;                                       // rms 1차 미분값 (진동 변화량)
    float        dd_rms;                                      // rms 2차 미분값 (진동 가속 변화량)
    float        cpsr_max[T2_Def::Audio::FeatureLimit::CEPS_TARGET_MAX]; // 주파수 대역별 에너지
    float        cpsr_mxr[T2_Def::Audio::FeatureLimit::CEPS_TARGET_MAX];    // 주파수 대역별 에너지
    float        band_energy[T2_Def::Audio::FeatureLimit::BAND_MAX];      // 주파수 대역별 에너지
    ST_SpectralPeak_t top_peaks[T2_Def::Audio::FeatureLimit::TOP_PEAKS_MAX]; // 주파수 대역별 에너지
};

// [Block 4-2] 오디오 스테레오 결합 지표 (576 Bytes 완벽 정렬)
struct ST_Slot_Audio_t {
    ST_Audio_Channel_Slot_t ch[T2_Def::Audio::Sensor::CHANNELS_MAX]; // 220 * 2 = 440 Bytes
    float        timbre_bands[32];                                   // 128 Bytes (1/3 옥타브 대역 상대 비율)
    uint8_t      _pad[8];                                            // 8 Bytes 패딩으로 총 576 Bytes (16배수 정렬)
};

// [Block 5] AI 텐서 (3072 Bytes 완벽 정렬)
struct ST_Slot_Tensor_t {
    alignas(16) float mfcc[T2_Def::AI::Tensor::MFCC_DIM_MAX];   // 768 * 4 = 3072 Bytes
};

// 1. 진동 도메인 슬롯 (Accel + Gyro 통합)
struct alignas(16) ST_FeatureSlot_Vib_t {
    ST_Slot_Header_t header;         // 페이로드 타입: VIB_ONLY
    ST_Slot_Accel_t  accel;          // 가속도 지표 (RMS, Kurtosis 등)
    ST_Slot_Gyro_t   gyro;           // 자이로 지표 (RMS, Kurtosis 등)
    float            mfcc[T2_Def::AI::Tensor::MFCC_DIM_DEF]; // 3축(acc) + 3축(gyr) 융합 MFCC
};

// 2. 오디오 도메인 슬롯 (Audio 전용)
struct alignas(16) ST_FeatureSlot_Aud_t {
    ST_Slot_Header_t header;         // 페이로드 타입: AUDIO_ONLY
    ST_Slot_Audio_t  audio;          // 오디오 스테레오 지표 (Coh, IPD 포함)
    float            mfcc[T2_Def::AI::Tensor::MFCC_COEFFS_DEF * T2_Def::AI::Tensor::MFCC_COMPONENTS_DEF * T2_Def::Audio::Sensor::CHANNELS_MAX]; // 56*4 = 224 Bytes
};

// 3. AI 추론용 융합 텐서 구조 (최종 동기화 시점)
struct alignas(16) ST_UnifiedTensor_t {
    float data[T2_Def::AI::Tensor::MFCC_DIM_DEF]; // 전체 8채널 MFCC 융합 데이터
};


// 공유 컨텍스트 (더블 버퍼링 구조)
struct alignas(32) ST_SharedContext_t {
    alignas(32) ST_FeatureSlot_Vib_t vib_slots[2];      // 진동 슬롯 더블 버퍼
    alignas(32) std::atomic<uint8_t> vib_idx;          // 진동 슬롯 인덱스

    alignas(32) ST_FeatureSlot_Aud_t aud_slots[2];      // 오디오 슬롯 더블 버퍼
    alignas(32) std::atomic<uint8_t> aud_idx;          // 오디오 슬롯 인덱스
};


// ============================================================================
// [PART 4] 통신 및 바이너리 패킷 규격 (Byte Packing & _MAX 기반 고정 평탄화)
// ============================================================================
// 트리거 원인 (고정 크기, 고정 순서)
struct __attribute__((packed)) ST_TriggerReason_t {
    uint8_t  trigger_axis;        // 0: X, 1: Y, 2: Z, 3: Audio
    char     metric_name[16];     // 예: "RMS", "BAND_ENERGY"
    float    measured_value;      // 실제 측정값
    float    threshold_value;     // 임계값
    float    excess_ratio;        // 초과율 (measured_value / threshold_value)
};

// 파일 헤더 (고정 크기, 고정 순서)
struct __attribute__((packed, aligned(16))) ST_FileHeader_t {
    char     magic[4];          // 매직 넘버 (T215_Type_250용)
    uint16_t ver;               // 버전
    uint16_t struct_size;       // 구조체 크기
    uint32_t s_rate;            // 샘플링 레이트
    uint16_t fft;               // FFT 포인트
    uint16_t mfcc_d;            // MFCC 차원수
    uint8_t  accel_mask;        // 구버전 axes 삭제 후 가속도 축 마스크 배치
    uint8_t  gyro_mask;         // 자이로 축 마스크 배치
    uint8_t  audio_mask;        // 오디오 채널 마스크 배치
    uint8_t  _res;              // 예약
    uint32_t total;             // 전체 데이터 크기
    uint64_t trigger_t0;          // T0 절대 타임스탬프 (Monotonic)
    ST_TriggerReason_t reason;    // 트리거 상세 원인 메타데이터
    uint8_t  ml_fault_class;      // AI 예측값
    float    ml_confidence;       // AI 분류 정확도
    uint8_t  _pad_header[11];     // 16의 배수(80바이트) 정렬용 패딩
    char     config_dump[8192];   // 설정 덤프
};

// WebSocket 패킷 헤더 (고정 크기, 고정 순서)
struct __attribute__((packed)) ST_WsHeader_t {
    uint8_t  magic;             // 매직 넘버
    uint8_t  type;              // 패킷 타입
    uint16_t len;               // 페이로드 길이
    uint8_t  stage;             // 처리 단계
    uint8_t  source;            // 데이터 소스
    uint8_t  _pad[2];           // 패딩
};

// 텔레메트리 패킷 (고정 크기, 고정 순서)
struct __attribute__((packed, aligned(16))) ST_PktTelemetry_t {
    ST_WsHeader_t header;          // 헤더 (8 Bytes)
    uint8_t  sys_state;            // 시스템 상태 (1 Bytes)
    uint8_t  detect_result;        // 검출 결과 (1 Bytes)
    uint8_t  trial_no;             // 시험 번호 (1 Bytes)
    uint8_t  trigger_source;       // 트리거 소스 (1 Bytes)
    uint8_t  accel_mask;           // 가속도 마스크 (1 Bytes)
    uint8_t  gyro_mask;            // 자이로 마스크 (1 Bytes)
    uint8_t  audio_mask;           // 오디오 마스크 (1 Bytes)
    uint8_t  payload_type;         // 페이로드 타입 (1 Bytes)

    uint64_t accel_ts;             // 가속도 타임스탬프 (8 Bytes)
    uint64_t gyro_ts;              // 자이로 타임스탬프 (8 Bytes)
    uint64_t audio_ts;             // 오디오 타임스탬프 (8 Bytes)
    float    temp;                 // 온도 (4 Bytes)
    uint8_t  _pad_header[4];       // 패딩 (4 Bytes)

    // [제로 카피 연산 영역] 구조체 최후미 배치 및 정렬 오버라이드
    G_T2_10_Def_ALIGN_16 float accel_band_energy[16];   // 가속도 1/3 옥타브 대역 에너지 (16 * 4 = 64 Bytes)
    G_T2_10_Def_ALIGN_16 float gyro_rms_energy[2];      // 자이로 RMS 에너지 (2 * 4 = 8 Bytes)
    uint8_t             _pad_gyro[8];            // 패딩 (8 Bytes)
    G_T2_10_Def_ALIGN_16 float audio_timbre_bands[32];  // 오디오 1/3 옥타브 대역 에너지 (32 * 4 = 128 Bytes)
    G_T2_10_Def_ALIGN_16 float audio_mfcc[13];          // 오디오 MFCC 계수 (13 * 4 = 52 Bytes)
    uint8_t             _pad_end[12];            // 패딩 (12 Bytes)
};

// [확인] 텔레메트리 패킷 정렬 검증 (빌드 시 자동 체크)
static_assert(offsetof(ST_PktTelemetry_t, accel_band_energy) % 16 == 0, "Unaligned tensor offset!");
static_assert(offsetof(ST_PktTelemetry_t, gyro_rms_energy) % 16 == 0, "Unaligned tensor offset!");
static_assert(offsetof(ST_PktTelemetry_t, audio_timbre_bands) % 16 == 0, "Unaligned tensor offset!");
static_assert(offsetof(ST_PktTelemetry_t, audio_mfcc) % 16 == 0, "Unaligned tensor offset!");
static_assert(sizeof(ST_PktTelemetry_t) % 16 == 0, "Unaligned structure size!");

// 스펙트럼 패킷 (고정 크기, 고정 순서)
struct __attribute__((packed)) ST_PktSpectrum_t {
    ST_WsHeader_t header;
    float    frequencies[(T2_Def::Audio::Sensor::FFT_SIZE_MAX / 2) + 1];
};

// 도메인별 독립 파형 전송을 위한 패킷 세분화
struct __attribute__((packed)) ST_PktWaveformAudio_t {
    ST_WsHeader_t header;
    float    samples[T2_Def::Audio::Sensor::FFT_SIZE_MAX];
};

struct __attribute__((packed)) ST_PktWaveformAccel_t {
    ST_WsHeader_t header;
    float    samples[T2_Def::Accel::Sensor::FFT_SIZE_MAX];
};

struct __attribute__((packed)) ST_PktWaveformGyro_t {
    ST_WsHeader_t header;
    float    samples[T2_Def::Gyro::Sensor::FFT_SIZE_MAX];
};

struct __attribute__((packed)) ST_PktCalibration_t {
    ST_WsHeader_t header;
    float    target_gain_curve[(T2_Def::Audio::Sensor::FFT_SIZE_MAX / 2) + 1];
    float    actual_fir_coeffs[T2_Def::Audio::FeatureLimit::FIR_TAPS_MAX];
};

struct __attribute__((packed)) ST_PktSequence_t {
    ST_WsHeader_t header;
    float    data[T2_Def::Global::System::SEQUENCE_FRAMES_MAX * T2_Def::AI::Tensor::MFCC_DIM_MAX];
};

}  // namespace T2_Type

