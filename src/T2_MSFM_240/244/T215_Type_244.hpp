/* ============================================================================
 * File: T215_Type_244.hpp
 * Summary: T240 4-Tier 데이터 타입 및 패킷 (_MAX 기반 정적할당 및 동적제어 완전체)
 * ============================================================================
 * [시스템 구현 원칙 - Core Principles]
 * 1. [동적 할당의 안전성]: 배열 크기는 컴파일 타임의 _MAX로 고정하여 OOM 방어.
 * 2. [런타임 제어]: On/Off 스위치 및 active_count 변수로 실제 연산 루프 최적화.
 * 3. [SIMD 가속 최적화]: 연산 슬롯 재조립. Header(32)+Vib(144)+Audio(240)+Tensor(384).
 * -> 총 800 Bytes. 16 Bytes 배수(alignas) 완벽 보장.
 * 4. [웹 패킷 불변성]: PktTelemetry는 실제 사용량과 무관하게 _MAX 크기로 평탄화 전송.
 * JS Float32Array 파싱 완벽 일치를 위해 Header 구간 16바이트 정렬 보장.
 * ========================================================================== */

#pragma once

#include <cstdint>

#include "T210_Def_244.hpp"


namespace T2_Type {

// ========================================================================
// [PART 1] 시스템 열거형 (Enum Classes)
// ========================================================================

enum class SystemState : uint8_t {
    INIT           = 0,     // 전원 인가 후 센서, 파일 시스템, 네트워크 등 하드웨어 초기화 상태
    READY,                  // 모든 초기화 완료 후 제어 명령을 기다리는 대기 상태
    MONITORING,             // 센서 데이터를 실시간으로 수집하며 결함(트리거) 조건을 감시하는 상태
    RECORDING,              // 이상 감지로 인해 SD카드/플래시에 데이터를 고속 저장 중인 상태
    NOISE_LEARNING,         // 스펙트럼 감산법을 위한 주변 배경 소음(Background Noise) 프로필 수집/학습 상태
    MAINTENANCE,            // OTA(무선 업데이트) 및 시스템 버스 락(Lock) 해제를 위한 안전 격리 상태
    ERROR,                  // 치명적인 하드웨어 결함 또는 워치독 타임아웃 오류 발생 상태
    CALIBRATING             // 마이크 주파수 평탄화(EQ) 및 진동 센서 영점 보정 상태
};

enum class OpMode : uint8_t {
    MANUAL         = 0,     // 사용자의 명시적인 시작(Start) 명령에 의해서만 구동되는 수동 제어 모드
    AUTO,                   // 전원 인가 직후 자동으로 감시(Monitoring) 상태로 진입하는 무인 구동 모드
    SCHEDULE                // 지정된 시간표(Cron)에 따라 감시와 절전(Sleep)을 반복하는 스케줄링 모드
};

// Audio 채널 모드 타입 안정성 보강
enum class AudioChannelMode : uint8_t {
    MONO_L         = 1,     // 좌측(Left) 마이크 데이터만 수집 및 연산하는 단일 채널 최적화 모드
    MONO_R         = 2,     // 우측(Right) 마이크 데이터만 수집 및 연산하는 단일 채널 최적화 모드
    STEREO         = 3      // 좌/우 양쪽 마이크 데이터를 수집하여 빔포밍 및 위상차 연산에 활용하는 듀얼 모드
};

// 패킷 및 슬롯 페이로드 식별자
enum class DataPayloadType : uint8_t {
    VIB_ONLY       = 1,     // 진동 데이터 슬롯만 유효한 페이로드 (소음 모듈 Off 시 대역폭 절약)
    AUDIO_ONLY     = 2,     // 소음 데이터 슬롯만 유효한 페이로드 (진동 모듈 Off 시 대역폭 절약)
    VIB_AUDIO_BOTH = 3      // 진동과 소음 데이터가 모두 포함된 융합(Sensor Fusion) 페이로드
};

enum class StatusBit : uint8_t {
    NTP_SYNCED     = 0,     // [1 << 0] 타임스탬프가 NTP 서버와 정밀 동기화되었음을 나타내는 비트 인덱스
    SD_MOUNTED     = 1,     // [1 << 1] SD 카드가 정상적으로 인식되어 마운트되었음을 나타내는 비트 인덱스
    RECORDING_NOW  = 2,     // [1 << 2] 현재 시스템이 스토리지에 데이터를 기록 중임을 나타내는 비트 인덱스
    SENSOR_FAULT   = 3      // [1 << 3] 하나 이상의 센서에 통신/물리적 장애가 발생했음을 나타내는 비트 인덱스
};

enum class DetectionResult : uint8_t {
    PASS           = 0,     // 모든 진단 룰(Rule) 및 AI 판정 결과 정상 상태 (Pass)
    RULE_VIB_NG,            // 진동 룰베이스(RMS, Band, 통계 지표) 판정 결과 결함(NG) 감지됨
    RULE_AUDIO_NG,          // 소음 룰베이스(RMS, Band, 통계 지표) 판정 결과 결함(NG) 감지됨
    TEST_NG,                // 생산 라인 자가 진단 및 시스템 테스트 결과 결함(NG) 발생
    ML_NG                   // 엣지 머신러닝(AI) 추론 모델 판정 결과 결함(NG) 감지됨
};

enum class SystemCommand : uint8_t {
    CMD_START          = 0, // 감시(Monitoring) 상태로 진입하라는 외부/내부 제어 명령
    CMD_STOP,               // 감시를 중단하고 대기(Ready) 상태로 복귀하라는 제어 명령
    CMD_LEARN_NOISE,        // 능동 소음 제거를 위한 배경 소음 학습 파이프라인 가동 명령
    CMD_CALIBRATE,          // 센서의 하드웨어/소프트웨어 보정 로직 가동 명령
    CMD_REBOOT,             // 시스템 메모리 덤프 후 안전한 소프트웨어 재부팅 수행 명령
    CMD_OTA_START,          // 펌웨어 원격 무선 업데이트(OTA) 세션 개방 명령 (유지보수 상태 진입)
    CMD_OTA_END,            // OTA 세션 종료 및 시스템 정상화 명령
    CMD_MANUAL_REC_START,   // [v015 이식] 사용자 수동 기록 시작 명령
    CMD_MANUAL_REC_STOP,    // [v015 이식] 사용자 수동 기록 중지 명령
    CMD_TUNING_PREVIEW,     // 변경된 설정을 플래시에 저장하지 않고 램(RAM)에만 임시 적용(핫스왑)하는 명령
    CMD_TUNING_SAVE,        // 현재 RAM에 적용된 설정을 비휘발성 메모리(NVS/JSON)에 영구 저장하는 명령
    CMD_TUNING_CANCEL       // 임시 적용된 설정을 폐기하고, 플래시에 저장된 마지막 상태로 롤백하는 명령
};

// [v015 이식] 비동기 세션 제어 명령 (Storage Manager 연동용)
enum class AsyncSessionCmd : uint8_t {
    NONE = 0,
    OPEN_AUTO,              // 자동 트리거 기반 세션 개방
    OPEN_MANUAL,            // 수동 버튼/API 기반 세션 개방
    OPEN_CALIB_MAN,         // 수동 교정용 데이터 수집 세션 개방
    CLOSE_NORMAL,           // 정상 종료 (트리거 시간 만료)
    CLOSE_MANUAL,           // 수동 종료 명령에 의한 세션 폐쇄
    CLOSE_CALIB_DONE        // 교정 데이터 수집 완료 및 교정기 가동 트리거
};

enum class TriggerSource : uint8_t {
    NONE           = 0,     // 트리거가 발생하지 않은 평시 상태 (주로 텔레메트리 전송 시)
    HW_WAKE,                // 센서 칩셋(BMI270 등) 자체 Any-Motion 하드웨어 인터럽트에 의한 트리거
    SW_RMS,                 // 소프트웨어로 계산된 전체 대역 에너지(RMS) 임계치 초과에 의한 트리거
    SW_BAND,                // 지정된 특정 주파수 대역(Band)의 에너지 임계치 초과에 의한 정밀 트리거
    MANUAL                  // 사용자 또는 관리자 API 강제 호출에 의한 수동 트리거
};

enum class StreamType : uint8_t {
    TELEMETRY      = 0x01,  // 특징량(Feature), 통계, 시스템 상태 등을 포함한 저용량 고효율 메타데이터 스트림
    SPECTRUM       = 0x02,  // FFT 변환이 완료된 주파수 도메인 스펙트럼(Bins) 데이터 스트림
    WAVEFORM       = 0x03,  // 가공되지 않은 시간 도메인 원시 파형 (Raw PCM) 데이터 스트림
    CALIBRATION    = 0x04,  // 튜닝 시 타겟 게인(Target Gain)과 실제 FIR 계수 적용 상태를 보여주는 디버깅 스트림
    SEQUENCE       = 0x05   // AI 추론용 시퀀스 텐서 (Multi-frame) 데이터 스트림
};

enum class NoiseMode : uint8_t {
    OFF            = 0,     // 스펙트럼 감산 등 노이즈 제거 파이프라인 전면 비활성화
    FIXED,                 // 사전에 학습된 고정 소음 프로필(Fixed Profile)을 뺀 나머지 신호만 처리하는 모드
    ADAPTIVE               // 주변 소음 변화에 맞춰 프로필을 실시간(Alpha rate)으로 갱신하는 적응형 노이즈 감산 모드
};

enum class WiFiMode : uint8_t {
    STA_ONLY       = 0,     // 외부 공유기(AP)에 접속만 수행하는 클라이언트 최적화 모드
    AP_ONLY,                // 디바이스 자체가 공유기 역할을 하여 설정망을 구축하는 호스트 전용 모드
    AP_STA,                 // 호스트 역할과 클라이언트 역할을 동시에 수행하는 듀얼 브릿지 모드
    AUTO_FALLBACK           // STA 접속 실패 시 통신 고립 방어를 위해 자동으로 AP 모드로 전환하는 페일세이프 모드
};

enum class WindowType : uint8_t {
    HANN           = 0,     // 범용 주파수 분석에 적합하며, 손실 진폭 보상이 가능한 Hann 윈도우
    HAMMING,                // Hann과 유사하나 인접 주파수 간섭(Leakage) 억제에 약간 더 유리한 Hamming 윈도우
    BLACKMAN                // 메인 로브는 넓으나 사이드 로브 억제력이 가장 뛰어난 Blackman 윈도우
};


// ========================================================================
// [PART 2] 동적 설정 구조체 (런타임 제어 및 스위치 기능 통합)
// ========================================================================

// ------------------------------------------------------------------------
// Tier 1. Global (디바이스 전역 인프라)
// ------------------------------------------------------------------------

struct ST_Global_WiFi {
	WiFiMode mode;                                                                              // Wi-Fi 구동 모드 (STA, AP, 듀얼, 자동 폴백 등 네트워크 전략 결정)
	char	 ap_ssid[T2_Def::Global::NetLimit::MAX_SSID_LEN_CONST];                             // 디바이스 자체 AP 모드 구동 시 송출할 SSID
	char	 ap_pw[T2_Def::Global::NetLimit::MAX_PW_LEN_CONST];                                 // 디바이스 자체 AP 모드 구동 시 사용할 비밀번호
	char	 ap_ip[T2_Def::Global::NetLimit::MAX_IP_LEN_CONST];                                 // 디바이스 자체 AP 모드 구동 시 할당할 고정 IP 주소
	char	 multi_ssid[T2_Def::Global::NetLimit::MAX_MULTI_AP_CONST][T2_Def::Global::NetLimit::MAX_SSID_LEN_CONST]; // 클라이언트(STA) 모드 시 접속을 시도할 예비 공유기 SSID 배열 (오토 폴백용)
	char	 multi_pw[T2_Def::Global::NetLimit::MAX_MULTI_AP_CONST][T2_Def::Global::NetLimit::MAX_PW_LEN_CONST];     // 예비 공유기 접속을 위한 비밀번호 배열
};

struct ST_Global_Mqtt {
	bool	 enable;                                                                            // MQTT 클라우드 통신 파이프라인 전면 활성화 스위치
	char	 broker[T2_Def::Global::NetLimit::MAX_BROKER_LEN_CONST];                            // MQTT 브로커(서버)의 IP 주소 또는 도메인 네임
	uint16_t port;                                                                              // MQTT 브로커 접속 포트 (보통 1883 또는 MQTTS 8883)
	char	 id[32];                                                                            // 브로커에서 디바이스를 식별하기 위한 고유 Client ID
	char	 pw[64];                                                                            // 브로커 접속을 위한 인증 패스워드 (또는 토큰)
	char	 topic_root[64];                                                                    // 시스템 상태 및 데이터를 발행(Publish)할 최상위 기준 토픽
	char	 lwt_topic[64];                                                                     // 비정상적인 연결 끊김(정전/네트워크 장애) 시 브로커가 대신 발행할 유언(Last Will) 토픽
	uint8_t	 qos;                                                                               // MQTT 메시지 전송 품질 보장 레벨 (0: 최대 1회, 1: 최소 1회, 2: 정확히 1회)
	uint8_t	 proto_ver;                                                                         // 연결에 사용할 MQTT 프로토콜 버전 (4 = v3.1.1, 5 = v5.0)
};

struct ST_Global_Storage {
	uint32_t rot_mb;                                                                            // 단일 파일 크기가 이 용량(MB)을 초과하면 새 파일로 로테이션(분할) 처리
	uint32_t rot_min;                                                                           // 파일 생성 후 이 시간(분)이 경과하면 용량과 무관하게 새 파일로 로테이션 처리
	bool	 save_raw;                                                                          // 특징량(Feature) 외에 가공되지 않은 시간 도메인 원시 파형(Raw)까지 동시 저장할지 여부
	uint16_t keep_max;                                                                          // 디스크 용량 관리를 위해 보존할 최근 로테이션 파일의 최대 개수 (오래된 것부터 삭제)
	uint32_t idle_flush_ms;                                                                     // 버퍼가 다 차지 않아도, 이 시간(ms) 동안 데이터 유입이 없으면 강제로 디스크에 기록(Flush)
	uint8_t	 pre_trig_sec;                                                                      // 이벤트(트리거) 발생 시점 기준, 소급하여 파일에 함께 기록할 과거 데이터의 시간(초)
};

struct ST_Global_System {
	char	 site_id[32];                                                                       // 데이터가 수집되는 물리적 설비명 또는 설치 현장 식별자 (Tagging용)
	uint8_t	 tele_hz;                                                                           // 클라우드/웹으로 상태 및 특징량(Telemetry) 데이터를 송신하는 초당 횟수 (Hz)
	uint8_t	 wave_hz;                                                                           // 원시 파형(Waveform) 데이터를 실시간 스트리밍하는 초당 횟수 (부하가 커서 보통 0으로 비활성)
	OpMode	 op_mode;                                                                           // 부팅 직후 시스템 구동 방식 (수동 대기, 자동 감시, 스케줄링)
	uint32_t watchdog_ms;                                                                       // 소프트웨어 오작동(무한 루프 등)을 감시하여 시스템을 강제 리셋할 워치독 타이머 (ms)
	bool	 vib_enable;                                                                        // 진동 센서(BMI270 등) 하드웨어 전원 및 신호처리 파이프라인 마스터 활성화 스위치
	bool	 audio_enable;                                                                      // 소음 센서(I2S 마이크 등) 하드웨어 전원 및 신호처리 파이프라인 마스터 활성화 스위치
};

struct ST_Global_Output {
	bool	 enabled;                                                                           // 외부(클라우드/웹)로 데이터를 푸시(Push)하는 기능 자체의 활성화 스위치
	bool	 output_sequence;                                                                   // 데이터를 1프레임씩 보내지 않고, AI 모델 입력 규격에 맞춰 시퀀스(텐서) 단위로 묶어서 전송할지 여부
	uint16_t sequence_frames;                                                                   // 시퀀스 묶음 전송 시, 하나의 텐서를 구성할 프레임의 개수 (예: 16프레임 = 1시퀀스)
};

// ------------------------------------------------------------------------
// Tier 2. Shared (진동/소음 공통 신호처리 논리)
// ------------------------------------------------------------------------

struct ST_FilterFIR {
	bool	 en;                                                                                // FIR(유한 임펄스 응답) 필터 연산 파이프라인 활성화 스위치
	float	 cutoff;                                                                            // 필터 차단 주파수 (Hz) - 이 기준을 넘거나 못 미치는 대역의 신호를 감쇠시킴
	uint16_t taps;                                                                              // 필터 계수의 개수(차수). 높을수록 차단 대역이 예리해지나 CPU 연산량이 급증함
};

struct ST_FilterIIR {
	bool  en;                                                                                   // IIR(무한 임펄스 응답) 필터 활성화 스위치 (FIR 대비 연산이 가벼우나 위상 지연 발생 가능)
	float cutoff;                                                                               // 필터 차단 주파수 (Hz)
	float q;                                                                                    // Q-Factor (품질 계수). 컷오프 주파수 부근의 공진(Peak) 특성 및 경사도를 결정
};

struct ST_FilterNotch {
	bool  en;                                                                                   // 노치(대역 제거) 필터 활성화 스위치 (특정 주파수 성분만 핀셋으로 제거할 때 사용)
	float freq;                                                                                 // 제거 타겟 중심 주파수 (Hz) (예: 공장 교류 전원에 의한 60Hz/120Hz 험 노이즈)
	float gain;                                                                                 // 노치 필터의 뎁스/게인 보상값 (일반적으로 타겟 대역을 완전히 파내기 위해 0.0 적용)
	float q;                                                                                    // Q-Factor. 파내는 대역폭의 예리함을 결정 (값이 클수록 타겟 주파수만 매우 좁게 제거함)
};

struct ST_Shared_Trigger {
	uint32_t hold_ms;                                                                           // 트리거(이벤트) 소멸 후, 일시적인 센서 튐으로 인한 세션 끊김을 방지하고 레코딩을 유지할 시간 (ms)
	bool	 use_sleep;                                                                         // 지정된 유휴 시간 동안 트리거가 발생하지 않으면 시스템을 딥슬립(절전) 모드로 전환할지 여부
	uint32_t sleep_sec;                                                                         // 절전 모드 진입을 위한 이벤트 부재 대기 시간 (초 단위)
};

struct ST_Shared_Dsp {
	bool		   rem_dc;                                                                      // 신호의 직류 편향(DC Offset) 성분을 연산 초기 단계에서 평균값을 빼는 방식으로 영점 정렬할지 여부
	bool		   med_en;                                                                      // 순간적인 스파이크성 튀는 값(Impulse Noise)을 부드럽게 깎아내는 메디안(중간값) 필터 활성화
	uint8_t		   med_win;                                                                     // 메디안 필터 적용 시 훑어볼 윈도우 프레임 크기 (중앙값을 구하기 위해 항상 3, 5 같은 홀수 적용)
	ST_FilterFIR   hpf;                                                                         // 초저주파 진동 및 중력 가속도(1G) 성분을 차단하기 위한 FIR 기반 고역 통과 필터 (High-Pass)
	ST_FilterFIR   lpf;                                                                         // 나이퀴스트(Nyquist) 주파수 이상의 에일리어싱 노이즈 유입을 막는 FIR 기반 저역 통과 필터 (Low-Pass)
	ST_FilterIIR   iir_hpf;                                                                     // 연산 리소스를 아끼면서 초저주파 성분을 차단하기 위한 IIR 기반 고역 통과 필터
	ST_FilterIIR   iir_lpf;                                                                     // 연산 리소스를 아끼면서 고주파 성분을 깎아내기 위한 IIR 기반 저역 통과 필터
	WindowType	   win_type;                                                                    // FFT 변환 시 주파수 누설(Leakage)을 막고 양 끝단의 불연속성을 줄이는 윈도우 함수 종류 (Hann 등)
	ST_FilterNotch notch;                                                                       // 기계적 공진이나 전원 노이즈 등 특정 고정 주파수 성분을 억제하는 공통 노치 필터
	ST_FilterNotch notch2;                                                                      // 고조파 또는 추가 노이즈 성분 제거를 위한 제2 노치 필터
};

// ------------------------------------------------------------------------
// Tier 3. Vib (진동 특화 파이프라인)
// ------------------------------------------------------------------------

struct ST_Vib_Sensor {
	bool	 accel_enable;                                                                      // 가속도 센서 연산 파이프라인 전면 활성화 스위치 (연산량/전력 제어)
	bool	 gyro_enable;                                                                       // 자이로 센서 연산 활성화 스위치 (특정 회전 진단 시에만 사용)

	uint8_t	 accel_axis_count;                                                                  // 가속도 런타임 연산에 투입할 축의 개수 (1축 단일 연산 vs 3축 병렬 연산)
	uint8_t	 accel_axis;                                                                        // 가속도 단일 축 분석 모드일 때 메인 타겟이 되는 센서 물리 축 (0:X, 1:Y, 2:Z)
	uint8_t	 gyro_axis_count;                                                                   // 자이로 런타임 연산에 투입할 축의 개수
	uint8_t	 gyro_axis;                                                                         // 자이로 단일 축 분석 모드일 때 메인 타겟이 되는 센서 물리 축 (0:X, 1:Y, 2:Z)

	uint8_t	 accel_range;                                                                       // 가속도 센서 측정 한계 범위 하드웨어 설정치 (±2G, 4G, 8G, 16G)
	uint8_t	 accel_odr;                                                                         // 가속도 센서 출력 데이터 레이트(ODR) 레지스터 매핑값
	uint8_t	 gyro_range;                                                                        // 자이로 센서 측정 한계 범위 하드웨어 설정치 (dps)
	uint32_t sample_rate;                                                                       // 런타임에 동적으로 적용되는 진동 센서 샘플링 레이트 (Hz)
	uint32_t fft_size;                                                                          // 런타임에 동적으로 적용되는 주파수 분석(FFT) 윈도우 크기 (주파수 해상도 결정)
};

struct ST_Vib_Trigger {
	bool	 motion_en;                                                                         // 센서 하드웨어 레벨의 Any-Motion(모션 감지) 인터럽트 핀 활성화 스위치
	float	 wake_g;                                                                            // Any-Motion 인터럽트를 유발할 충격 가속도 임계치 (G 단위)
	uint16_t wake_dur;                                                                          // 노이즈성 충격을 무시하기 위해, 지정된 임계치를 넘어야 하는 최소 유지 시간
	
	// [3축 독립 개정] 물리 강성에 따른 Radial(X/Y) vs Axial(Z) 임계치 격리
	float	 rms_thresh[3];                                                                     // X, Y, Z 각 축별 광대역 RMS 트리거 임계치
	float	 kurt_ng_thresh[3];                                                                 // X, Y, Z 각 축별 첨도 불량 판정 임계치
	float	 crest_ng_thresh[3];                                                                // X, Y, Z 각 축별 파고율 불량 판정 임계치
	float	 skew_ng_thresh[3];                                                                 // X, Y, Z 각 축별 왜도 불량 판정 임계치

	uint8_t	 active_band_count;                                                                 // 런타임에 실제 사용할 주파수 감시 밴드 개수 (MAX 한계 내에서 동적 할당)
	bool	 band_en[T2_Def::Shared::FeatureLimit::BAND_RMS_MAX];                               // 개별 감시 밴드의 활성화/비활성화 스위치 배열
	float	 band_start[T2_Def::Shared::FeatureLimit::BAND_RMS_MAX];                            // [시너지] 특정 베어링/기어 결함 주파수를 추적하기 위한 밴드 시작점 (Hz)
	float	 band_end[T2_Def::Shared::FeatureLimit::BAND_RMS_MAX];                              // 특정 결함 주파수 추적 밴드 종료점 (Hz)
	float	 band_thresh[3][T2_Def::Shared::FeatureLimit::BAND_RMS_MAX];                        // [3축 개정] 각 축별 대역 에너지 임계치
};

struct ST_Vib_Calib {
	float offset[3];                                                                            // 센서 부착 틸트 및 중력 가속도로 인한 영점(DC Offset) 틀어짐을 보정하는 X,Y,Z 오프셋 값
	float gain[3];                                                                              // 센서 칩셋 제조 편차로 인한 감도 보정을 수행하는 축별 스케일 팩터(Multiplier)
};

struct ST_Vib_Feature {
	float peak_amp_min;                                                                         // 노이즈를 배제하고 유의미한 진동 결함 피크로 인정할 최소 진폭(Amplitude) 임계치
	float peak_freq_gap_min;                                                                    // 피크 추출 시, 동일 결함으로 간주하여 병합(Merge)하거나 필터링할 최소 주파수 이격 거리 (Hz)
};

// ------------------------------------------------------------------------
// Tier 4. Audio (소음 특화 파이프라인)
// ------------------------------------------------------------------------

struct ST_Audio_Sensor {
	AudioChannelMode channel_mode;                                                              // 런타임 마이크 채널 구동 모드 (단일 L/R 채널 최적화 또는 스테레오 위상 분석 모드)
	uint32_t		 sample_rate;                                                               // 런타임 음향 데이터 수집 샘플링 주파수 (Hz)
	uint32_t		 fft_size;                                                                  // 런타임 음향 주파수 분석(FFT) 해상도를 결정하는 윈도우 크기 (메모리 제약 하에 동적 할당)
};

struct ST_Audio_Trigger {
	// [듀얼 채널 개정] 마이크 장착부의 Noise Floor 물리적 편차 대응을 위한 L/R 임계치 격리
	float	rms_thresh[2];                                                                      // L/R 개별 광대역 RMS 트리거 임계치
	float	kurt_ng_thresh[2];                                                                  // L/R 개별 첨도 불량 판정 임계치
	float	crest_ng_thresh[2];                                                                 // L/R 개별 파고율 불량 판정 임계치
	float	skew_ng_thresh[2];                                                                  // L/R 개별 왜도 불량 판정 임계치

	uint8_t active_band_count;                                                                  // 런타임에 실제 감시할 소음 주파수 대역 개수
	bool	band_en[T2_Def::Shared::FeatureLimit::BAND_RMS_MAX];                                // 개별 감시 밴드의 활성화 스위치 배열
	float	band_start[T2_Def::Shared::FeatureLimit::BAND_RMS_MAX];                             // 에어 리크(Air Leak)나 고주파 마찰 소음 등을 핀셋 타겟팅하기 위한 밴드 시작점 (Hz)
	float	band_end[T2_Def::Shared::FeatureLimit::BAND_RMS_MAX];                               // 핀셋 타겟팅 밴드 종료점 (Hz)
	float	band_thresh[2][T2_Def::Shared::FeatureLimit::BAND_RMS_MAX];                         // [듀얼 개정] L/R 대역별 에너지 임계치
};

struct ST_Audio_Noise {
	bool	  gate_en;                                                                          // 일정 진폭 이하의 미세한 배경 소음(White Noise)을 0으로 묵음 처리할지 여부
	float	  gate_thresh;                                                                      // 노이즈 게이트를 개방하여 유의미한 소음으로 받아들일 절대 진폭 임계치
	NoiseMode mode;                                                                             // 스펙트럼 감산법 구동 모드 지정 (정적 프로필 고정 차감 vs 환경 적응형 실시간 차감)
	float	  sub_str;                                                                          // 스펙트럼 감산법 적용 시 제거할 노이즈 프로필의 강도 비율 (오버 서브트랙션 억제용)
	float	  adp_alpha;                                                                        // 적응형(Adaptive) 모드 구동 시, 최신 배경 소음을 노이즈 프로필에 반영하는 갱신 속도(이동 평균 알파값)
	uint16_t  learn_frames;                                                                     // 시스템 초기화 시 고정 배경 소음 프로필을 학습(Learning)하기 위해 취득할 윈도우 프레임 수
};

struct ST_Audio_Dsp {
	bool		   pre_en;                                                                      // 고주파수 성분의 감쇠 현상을 보상하여 평탄한 스펙트럼을 얻기 위한 프리엠파시스 활성화
	float		   pre_alpha;                                                                   // 프리엠파시스 필터 가중치 계수 (통상 0.97 부근 사용)
	ST_Audio_Noise noise;                                                                       // 능동 소음 억제 및 노이즈 프로필 관리 하위 구조체
	float		   beam_gain;                                                                   // 스테레오 채널 위상차 빔포밍 합성 시 주음원 방향 신호에 적용할 가중치 게인
	float		   window_ms;                                                                   // STFT(단시간 푸리에 변환) 분석을 위해 음향 데이터를 분할하는 실제 시간 창의 길이 (ms)
	float		   hop_ms;                                                                      // 슬라이딩 윈도우 이동 간격(ms). window_ms 대비 hop_ms의 비율이 오버랩(Overlap) 정도를 결정
};

struct ST_Audio_Calib {
	uint32_t auto_idle_min;                                                                     // 시스템이 지정된 시간(분) 동안 트리거 없이 완전히 정숙하면, 스스로 자동 캘리브레이션(EQ) 진입
	float	 ref_freq;                                                                          // 마이크 주파수 응답 평탄화(EQ) 보정 연산의 기준 타겟이 되는 절대 주파수 (보통 1kHz)
	float	 filt_min;                                                                          // 캘리브레이션 EQ 필터가 적용될 유효 주파수 대역의 하한치 (이 이하의 저역은 왜곡 방지를 위해 보정 생략)
	float	 filt_max;                                                                          // 캘리브레이션 EQ 필터가 적용될 유효 주파수 대역의 상한치 (이 이상의 고역은 보정 생략)
	float	 gain_max;                                                                          // FIR EQ 필터 튜닝 시 특정 주파수 대역의 증폭(Gain) 물리적 상한치 제한 (과증폭 억제)
	float	 gain_min;                                                                          // FIR EQ 필터 튜닝 시 특정 주파수 대역의 감쇠(Gain) 물리적 하한치 제한 (과감쇠 억제)
	float	 norm_safe;                                                                         // 필터 계수 정규화 시 무한 증폭 루프를 방어하기 위한 수학적 안전 마진 임계값
	float	 gain_L;                                                                            // 좌측(Left) 마이크 하드웨어 감도 편차 수동 보정 게인
	float	 gain_R;                                                                            // 우측(Right) 마이크 하드웨어 감도 편차 수동 보정 게인
	alignas(16) float eq_coeffs[2][T2_Def::Shared::FeatureLimit::FIR_TAPS_MAX];                 // [듀얼 개정] L/R 개별 캘리브레이션 FIR 필터 계수 배열 (SIMD 최적화 강제 정렬)
};

struct ST_Audio_Feature {
	uint8_t active_ceps_count;                                                                  // 런타임에 실제 추적/감시할 켑스트럼(Cepstrum) 피크 타겟 개수
	uint8_t active_peak_count;                                                                  // 런타임에 실제 스펙트럼에서 추출할 주요 하모닉 피크의 개수
	float	ceps_targets[T2_Def::Shared::FeatureLimit::CEPS_TARGET_MAX];                        // 엔진, 펌프 등 주요 기계적 회전체 고유 소음의 켑스트럼 타겟 큐퍼런시(Quefrency) 배열
	float	f_min;                                                                              // 피크 추출, 대역 분석 및 MFCC 특징량 산출 파이프라인에 입력으로 허용할 유효 대역폭 범위의 하한 (Hz)
	float	f_max;                                                                              // 분석 유효 대역폭 범위의 상한 (Hz)
	float	peak_amp_min;                                                                       // 유의미한 소음 피크로 검출하기 위한 최소 진폭 (바닥 배경 소음 피크를 무시하기 위함)
	float	peak_freq_gap_min;                                                                  // 넓게 퍼진 단일 피크를 여러 개로 오인하지 않도록 인접 피크를 병합할 최소 주파수 이격 거리 (Hz)
};


// ------------------------------------------------------------------------
// 마스터 동적 설정 구조체 (SSOT)
// ------------------------------------------------------------------------
struct DynamicConfig {
	ST_Global_System  system;
	ST_Global_WiFi	  wifi;
	ST_Global_Mqtt	  mqtt;
	ST_Global_Storage storage;
	ST_Global_Output  output;
	ST_Shared_Trigger shared_trig;
	ST_Shared_Dsp	  shared_dsp;
	ST_Vib_Sensor	  vib_sensor;
	ST_Vib_Trigger	  vib_trig;
	ST_Vib_Calib	  vib_calib;
	ST_Vib_Feature	  vib_feat;
	ST_Audio_Sensor	  aud_sensor;
	ST_Audio_Trigger  aud_trig;
	ST_Audio_Dsp	  aud_dsp;
	ST_Audio_Calib	  aud_calib;
	ST_Audio_Feature  aud_feat;
};

// ------------------------------------------------------------------------
// 런타임 제어 컨텍스트 (비휘발성 설정이 아닌 메모리 런타임 전용)
// ------------------------------------------------------------------------
struct ST_Runtime_TriggerCtx {
	bool		  is_triggered;
	TriggerSource active_source;
	uint32_t	  hold_end_tick;
};

// ============================================================================
// [PART 3] 고장 진단 데이터 모델 (800 Bytes SIMD Perfect Alignment)
// ============================================================================

struct SpectralPeak {
	float freq;                                                                                 // 검출된 주파수 피크의 위치 (Hz)
	float amp;                                                                                  // 검출된 주파수 피크의 진폭 크기 (Magnitude)
};

// [Block 1] 공통 헤더 (32 Bytes)
struct ST_Slot_Header {
	uint64_t ts;                                                                                // 데이터 수집 완료 시점의 절대 타임스탬프 (NTP 동기화 기준 밀리초)
	uint32_t fid;                                                                               // 부팅 후 순차 증가하는 프레임 고유 식별 번호 (데이터 누락 및 순서 역전 감지용)
	uint32_t uptime;                                                                            // 시스템 부팅 후 경과 시간 (밀리초)
	uint8_t	 flags;                                                                             // 시스템 상태 비트 플래그 묶음 (StatusBit 열거형 매핑: NTP, SD 마운트 등)
	int8_t	 temp;                                                                              // 센서 칩셋 내부 온도 (온도 변화에 따른 센서 드리프트 오차 보정 참고용)
	uint8_t	 trial;                                                                             // 노이즈성 오작동 방지를 위한 현재 이벤트 판정/검증 반복 횟수
	uint8_t	 src;                                                                               // 현재 슬롯 생성을 유발한 트리거 소스 (TriggerSource 열거형 매핑)
	uint8_t	 active_axes;                                                                       // 현재 슬롯에 유효한 데이터가 채워진 진동 물리 축의 개수 (1축 또는 3축)
	uint8_t	 payload_type;                                                                      // 슬롯의 유효 데이터 포함 범위 (DataPayloadType 매핑: 진동 단독, 소음 단독, 융합)
	uint8_t	 _res[10];                                                                          // (22B + 10B = 32B) 헤더를 32바이트 16의 배수 선상에 완벽히 정렬시키기 위한 예약 패딩
};

// [Block 2] 진동 특화 지표 (304 Bytes)
struct ST_Slot_Vib {
	float rms[3];                                                                               // X, Y, Z 각 축별 전체 주파수 대역의 평균 진동 에너지 (Root Mean Square)
	float peak_f[3];                                                                            // 각 축별 스펙트럼에서 가장 에너지가 강한 지배(Dominant) 주파수 성분
	float centroid[3];                                                                          // 각 축별 주파수 무게 중심 (기어 마모 등 고주파 진동 발생 시 값이 커짐)
	float kurt[3];                                                                              // [3축 개정] 각 축별 첨도 (Kurtosis)
	float crest[3];                                                                             // [3축 개정] 각 축별 파고율 (Crest Factor)
	float sta_lta[3];                                                                           // [3축 개정] 각 축별 단기/장기 평균 비율 (급변 지표)
	float skew[3];                                                                              // [3축 개정] 각 축별 왜도 (Skewness)
	float std[3];                                                                               // [3축 개정] 각 축별 표준편차 (Standard Deviation)
	float cal_off[3];                                                                           // 데이터 수집 시점에 적용되어 있던 센서의 영점 보정(Calibration) 오프셋 값
	float band_energy[3][T2_Def::Shared::FeatureLimit::BAND_RMS_MAX];                           // [3축 개정] 3축별 개별 주파수 밴드 에너지 감시 (48 Floats = 192 Bytes)
	float _pad[1];                                                                              // [정밀 얼라인먼트] 300B + 4B = 304 Bytes (304 / 16 = 19, 완벽 정렬!)
};

// [Block 3] 단일 채널 전용 소음 특징량 구조체 (코드 중복 제거 및 구조화)
struct ST_Audio_Channel_Slot {
	float        rms;                                                                           // RMS
	float        energy;                                                                        // 절대 에너지
	float        centroid;                                                                      // 주파수 무게 중심
	float        kurt;                                                                          // 첨도
	float        crest;                                                                         // 파고율
	float        skew;                                                                          // 왜도
	float        sta_lta;                                                                       // STA/LTA 비율
	float        d_rms;                                                                         // RMS 변화율
	float        dd_rms;                                                                        // RMS 변화 가속도
	
	// [듀얼 채널 개정] 주파수/스펙트럼 특징량 채널별 완전 격리
	float        cpsr_max[T2_Def::Shared::FeatureLimit::CEPS_TARGET_MAX];                       // 켑스트럼 피크 최대 강도
	float        cpsr_mxr[T2_Def::Shared::FeatureLimit::CEPS_TARGET_MAX];                       // 켑스트럼 대비 피크 비율
	float        band_energy[T2_Def::Shared::FeatureLimit::BAND_RMS_MAX];                       // 대역별 에너지 밀도
	SpectralPeak top_peaks[T2_Def::Shared::FeatureLimit::TOP_PEAKS_MAX];                        // 상위 피크 정보
}; // ch_slot 크기: float 9개(36B) + cpsr_max(20B) + cpsr_mxr(20B) + band_energy(64B) + top_peaks(80B) = 220 Bytes

// [Block 3-2] 통합 소음 특징량 구조체 듀얼 채널(L/R) 확장 (464 Bytes)
struct ST_Slot_Audio {
	ST_Audio_Channel_Slot ch[2];                                                                // ch[0] = Left, ch[1] = Right 독립 분석
	
	// 공간 결함성 지표 (L/R 교차 상관 및 위상 분석)
	float        coh;                                                                           // 공간 결함성 (단일 유지)
	float        ipd;                                                                           // 채널 간 위상차 (단일 유지)
	
	float        _pad[4];                                                                       // [정밀 얼라인먼트] 440B + 8B + 16B = 464 Bytes (464 / 16 = 29, 완벽 정렬!)
};

// [Block 4] AI 텐서 (1920 Bytes)
struct ST_Slot_Tensor {
	alignas(16) float mfcc[T2_Def::Audio::FeatureLimit::MFCC_DIM_MAX];                          // 5채널(Vib 3축 + Audio L/R 2채널) 통합 MFCC 1D 평탄화 배열 (480 Floats = 1920 Bytes)
};

// 전체 조립: 32 + 304 + 464 + 1920 = 총 2720 Bytes (SIMD 16B 완벽 정합성: 2720 / 16 = 170)
struct alignas(16) UnifiedFeatureSlot {
	ST_Slot_Header header;                                                                      // 메타데이터 및 추적 정보
	ST_Slot_Vib	   vib;                                                                         // 진동 룰베이스 통계 및 주파수 지표
	ST_Slot_Audio  audio;                                                                       // 소음 룰베이스 통계, 켑스트럼 및 위상 지표
	ST_Slot_Tensor tensor;                                                                      // 머신러닝 AI 엣지 추론을 위한 텐서 입력 벡터
};

// 스토리지 기록용 하이브리드 Raw 데이터
struct alignas(16) UnifiedRawChunk {
	float vib[3][T2_Def::Vib::Sensor::FFT_SIZE_MAX];                                            // 가속도 센서 원시 파형 (X, Y, Z)
	float audio_l[T2_Def::Audio::Sensor::FFT_SIZE_MAX];                                         // 마이크로폰 좌측(Left) 원시 파형
	float audio_r[T2_Def::Audio::Sensor::FFT_SIZE_MAX];                                         // 마이크로폰 우측(Right) 원시 파형
};

// ============================================================================
// [PART 4] 통신 및 바이너리 패킷 규격 (Byte Packing & _MAX 기반 고정 평탄화)
// ============================================================================
// 컴파일러의 임의적인 메모리 패딩을 금지하고 구조체를 1바이트 단위로 꽉 채워(Packing)
// 네트워크 전송 및 파일 입출력 시 시스템 간 바이트 오프셋 어긋남을 원천 차단함
#pragma pack(push, 1)

// 스토리지에 저장되는 바이너리 파일(.bin)의 무결성 검증 및 MLOps 추적성을 위한 매직 헤더
struct FileHeader {
	char	 magic[4];                                                                          // 고유 파일 포맷 식별자 (예: "T240")
	uint16_t ver;                                                                               // 바이너리 구조체 버저닝 (파서(Parser) 하위 호환성 유지용)
	uint16_t struct_size;                                                                       // 단일 데이터 프레임(UnifiedFeatureSlot)의 바이트 크기 (크로스 체킹용)
	uint32_t s_rate;                                                                            // 데이터 기록 당시 설정되어 있던 메인 센서의 샘플링 레이트
	uint16_t fft;                                                                               // 데이터 기록 당시 설정되어 있던 메인 주파수 분석(FFT) 크기
	uint16_t mfcc_d;                                                                            // 데이터 기록 당시 추출된 MFCC 텐서의 총 차원 수
	uint8_t	 axes;                                                                              // 데이터 기록 당시 연산에 동원된 물리 축의 개수
	uint8_t	 _res;                                                                              // 헤더 구조체 4바이트 정렬을 위한 1바이트 예약 패딩
	uint32_t total;                                                                             // 해당 바이너리 파일 내에 기록된 총 프레임(Slot)의 누적 개수
	char	 config_dump[8192];                                                                 // [MLOps 핵심] 기록 당시의 전체 시스템 JSON 설정값을 문자열로 통째로 덤프 (완벽한 재현성 보장)
};

// 웹소켓(WebSocket) 실시간 스트리밍 패킷의 공통 프롤로그 헤더 (8 Bytes)
struct WsHeader {
	uint8_t	 magic;                                                                             // 웹소켓 패킷 동기화 매직 바이트 (0xAA)
	uint8_t	 type;                                                                              // 페이로드 형태 (StreamType: 1:Tele, 2:Spec, 3:Wave, 4:Cal, 5:Seq)
	uint16_t len;                                                                               // 페이로드 바이트 길이
	uint8_t	 stage;                                                                             // FSM 연산 스테이지
	uint8_t	 source;                                                                            // 데이터 근원지 (0:None, 1:Audio, 2:VibX, 3:VibY, 4:VibZ)
	uint8_t	 _pad[2];                                                                           // 8바이트 정렬을 위한 패딩
};

// [정렬 방어 기술]: WsHeader(8B) + 상태변수 6개(6B) + _pad[2](2B) = 16B 선행 확보 완료.
// 이를 통해 이어지는 수백 개의 float(4B) 배열들이 브라우저의 Float32Array 메모리 뷰에 매핑될 때,
// "offset is not a multiple of 4" 파싱 크래시(Alignment Fault)가 발생하는 것을 수학적으로 완벽히 차단함.
struct PktTelemetry {
	WsHeader header;                                                                            // [8B] 공통 웹소켓 헤더
	uint8_t	 sys_state;                                                                         // [1B] 디바이스 상태 (SystemState 매핑)
	uint8_t	 detect_result;                                                                     // [1B] 종합 불량 판정 결과 (DetectionResult 매핑)
	uint8_t	 trial_no;                                                                          // [1B] 현재 판정 로직의 검증 반복 회차
	uint8_t	 trigger_source;                                                                    // [1B] 현재 패킷 송신을 유발한 트리거 근원지 (TriggerSource 매핑)
	uint8_t	 active_axes;                                                                       // [1B] 현재 패킷에 담긴 유효 진동 축 개수
	uint8_t	 payload_type;                                                                      // [1B] 현재 패킷 내 유효 데이터 종류 (DataPayloadType 매핑: Vib, Audio, Both)
	uint8_t	 _pad[2];                                                                           // [2B] 자바스크립트 메모리 정렬 방어선 (16 Bytes 오프셋 강제 달성)

	// [신규] 두 센서의 대기 주기 격차 해소를 위한 개별 물리 타임스탬프
	uint64_t audio_ts;                                                                          // [8B] 오디오 특징량의 절대 수집 타임스탬프
	uint64_t vib_ts;                                                                            // [8B] 진동 특징량의 절대 수집 타임스탬프

	// --- Vib Flattening (3축 완전 격리 개정) ---
	float	 vib_rms[3];                                                                        // X, Y, Z 축별 진동 RMS 평탄화 배열
	float	 vib_centroid[3];                                                                   // X, Y, Z 축별 진동 주파수 무게중심 평탄화 배열
	float	 vib_kurtosis[3];                                                                   // [3축 개정] X, Y, Z 각 축별 첨도 지표
	float	 vib_crest_factor[3];                                                               // [3축 개정] X, Y, Z 각 축별 파고율 지표
	float	 vib_sta_lta_ratio[3];                                                              // [3축 개정] X, Y, Z 각 축별 STA/LTA 비율 지표
	float	 vib_skewness[3];                                                                   // [3축 개정] X, Y, Z 각 축별 왜도 지표
	float	 vib_std[3];                                                                        // [3축 개정] X, Y, Z 각 축별 표준편차 지표
	float	 vib_band_energy[3][T2_Def::Shared::FeatureLimit::BAND_RMS_MAX];                    // [3축 개정] 3축별 개별 주파수 밴드 에너지 감시 (48 Floats)

	// --- Audio Flattening (L/R 듀얼 채널 완전 격리 개정) ---
	float	 audio_rms[2];                                                                      // [듀얼 개정] L/R 마이크 각각의 RMS 레벨
	float	 audio_energy[2];                                                                   // [듀얼 개정] L/R 마이크 각각의 절대 에너지 총합
	float	 audio_kurtosis[2];                                                                 // [듀얼 개정] L/R 마이크 각각의 파형 첨도
	float	 audio_crest_factor[2];                                                             // [듀얼 개정] L/R 마이크 각각의 파형 파고율
	float	 audio_sta_lta_ratio[2];                                                            // [듀얼 개정] L/R 마이크 각각의 급변 감지 지표 (STA/LTA)
	float	 audio_skewness[2];                                                                 // [듀얼 개정] L/R 마이크 각각의 비대칭성 지표 (왜도)
	float	 audio_spectral_centroid[2];                                                        // [듀얼 개정] L/R 마이크 각각의 주파수 무게중심
	
	// 공간 결함성 지표 (L/R 교차 상관 분석 결과 - 단일 유지)
	float	 audio_coh;                                                                         // 공간 결함성
	float	 audio_ipd;                                                                         // 채널 간 위상차
	
	float	 audio_band_energy[2][T2_Def::Shared::FeatureLimit::BAND_RMS_MAX];                  // [듀얼 개정] L/R 개별 밴드별 에너지 (32 Floats)
	float	 audio_peak_freqs[2][T2_Def::Shared::FeatureLimit::TOP_PEAKS_MAX];                  // [듀얼 개정] L/R 개별 추출된 상위 피크들의 주파수 (20 Floats)
	float	 audio_peak_amps[2][T2_Def::Shared::FeatureLimit::TOP_PEAKS_MAX];                   // [듀얼 개정] L/R 개별 추출된 상위 피크들의 진폭 (20 Floats)
	float	 audio_cpsr_max[2][T2_Def::Shared::FeatureLimit::CEPS_TARGET_MAX];                  // [듀얼 개정] L/R 개별 켑스트럼 피크 최대 진폭 (10 Floats)
	float	 audio_cpsr_mxrms[2][T2_Def::Shared::FeatureLimit::CEPS_TARGET_MAX];                // [듀얼 개정] L/R 개별 켑스트럼 피크 비율 (10 Floats)

	// --- AI Tensor ---
	float	 mfcc[T2_Def::Audio::FeatureLimit::MFCC_DIM_MAX];                                   // [5채널 개정] 5채널(Vib 3축 + Audio L/R 2채널) 통합 MFCC 1D 평탄화 텐서 배열
};

// 실시간 주파수 도메인 시각화(Chart.js 등)를 위한 스펙트럼 전송 패킷
struct PktSpectrum {
	WsHeader header;                                                                            // 공통 웹소켓 헤더
	float	 frequencies[(T2_Def::Audio::Sensor::FFT_SIZE_MAX / 2) + 1];                        // 나이퀴스트(Nyquist) 정리에 따라 절반으로 압축된 주파수 Bin 데이터 배열
};

// 실시간 시간 도메인 파형 시각화를 위한 원시(Raw) PCM 전송 패킷
struct PktWaveform {
	WsHeader header;                                                                            // 공통 웹소켓 헤더
	float	 samples[T2_Def::Audio::Sensor::FFT_SIZE_MAX];                                      // FFT 처리 전의 가공되지 않은 시간 도메인 센서 데이터 배열
};

// 캘리브레이션 모드 구동 시 튜닝 결과 시각화를 위한 필터 응답 전송 패킷
struct PktCalibration {
	WsHeader header;                                                                            // 공통 웹소켓 헤더
	float	 target_gain_curve[(T2_Def::Audio::Sensor::FFT_SIZE_MAX / 2) + 1];                  // 이상적인 주파수 평탄화 목표 게인 곡선
	float	 actual_fir_coeffs[T2_Def::Shared::FeatureLimit::FIR_TAPS_MAX];                     // 실제 FIR 필터 계수 배열
};

// [신규] 실시간 AI 시퀀스 텐서 전송 패킷
struct PktSequence {
    WsHeader header;
    float    data[T2_Def::Global::System::SEQUENCE_FRAMES_MAX * T2_Def::Audio::FeatureLimit::MFCC_DIM_MAX]; // 전 채널 통합 시퀀스 데이터
};

// [신규] 진동 특화 파형 전송 패킷 (오디오와 크기가 다를 수 있음)
struct PktWaveformVib {
    WsHeader header;
    float    samples[T2_Def::Vib::Sensor::FFT_SIZE_MAX];
};

// 1바이트 강제 팩킹 해제 및 컴파일러 기본 메모리 정렬(보통 4B 또는 8B) 정책으로 원상 복구
#pragma pack(pop)


}  // namespace T2_Type
