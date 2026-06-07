/* ============================================================================
 * File: T215_Type_245.hpp
 * Summary: T240 4-Tier 데이터 타입 및 패킷
 * ============================================================================ */

#pragma once

#include <cstdint>
#include "T210_Def_245.hpp"

namespace T2_Type {

// ========================================================================
// [PART 1] 시스템 열거형 (Enum Classes)
// ========================================================================

/**
 * @enum EM_SystemState_t
 * @brief 시스템 전역 동작 FSM 상태 열거형
 */
enum class EM_SystemState_t : uint8_t {
    INIT           = 0, ///< 시스템 부팅 및 하위 드라이버 초기화 단계
    READY,              ///< 대기 상태 (수집 대기)
    MONITORING,         ///< 실시간 신호 감지 및 이상 분석 동작 중
    RECORDING,          ///< 트리거 감지 또는 수동 정지 요청에 따른 SD카드 바이너리 기록 중
    NOISE_LEARNING,     ///< 오디오 도메인 소음 차감 프로파일 생성용 학습 모드 구동 중
    MAINTENANCE,        ///< OTA FOTA 무선 업데이트 또는 정기 유지관리 상태 (태스크 일시정지)
    ERROR,              ///< 센서, 통신 또는 파일시스템의 중대 장애 상태
    CALIBRATING         ///< 센서 오프셋 및 보정 파라미터 계측/연산 작업 중
};

/**
 * @enum EM_OpMode_t
 * @brief 시스템 동작 제어 모드
 */
enum class EM_OpMode_t : uint8_t {
    MANUAL         = 0, ///< 사용자 수동 입력을 통한 세션 관리
    AUTO,               ///< 이벤트를 자동 감지하여 처리하는 모드
    SCHEDULE            ///< 특정 주기나 시점 제어 모드
};

/**
 * @enum EM_AxisMask_t
 * @brief IMU 3축 활성화 여부 지정을 위한 비트 마스크
 */
enum class EM_AxisMask_t : uint8_t {
    NONE           = 0,
    AXIS_X         = 1 << 0, ///< X축 동작 활성화
    AXIS_Y         = 1 << 1, ///< Y축 동작 활성화
    AXIS_Z         = 1 << 2, ///< Z축 동작 활성화
    AXIS_ALL       = 0b111   ///< 전축(3축) 활성화
};

/**
 * @enum EM_ChannelMask_t
 * @brief 오디오 마이크 2채널 활성화 여부 지정을 위한 비트 마스크
 */
enum class EM_ChannelMask_t : uint8_t {
    NONE           = 0,
    CH_LEFT        = 1 << 0, ///< 좌측(L) 단독 활성화
    CH_RIGHT       = 1 << 1, ///< 우측(R) 단독 활성화
    CH_STEREO      = 0b11    ///< 좌우(L/R) 둘 다 활성화
};

/**
 * @enum EM_DataPayloadType_t
 * @brief 취득 데이터의 탑재(Payload) 형태 지정
 */
enum class EM_DataPayloadType_t : uint8_t {
    VIB_ONLY       = 1,      ///< 진동(가속도/자이로) 특징량만 탑재
    AUDIO_ONLY     = 2,      ///< 마이크 소음 특징량만 탑재
    VIB_AUDIO_BOTH = 3       ///< 진동 및 소음 특징량 모두 탑재
};

/**
 * @enum EM_StatusBit_t
 * @brief 시스템 하드웨어 및 동작 플래그를 비트 단위로 관리하기 위한 인덱스
 */
enum class EM_StatusBit_t : uint8_t {
    NTP_SYNCED     = 0,      ///< NTP 서버로부터 시간 동기화 완료 여부
    SD_MOUNTED     = 1,      ///< SD카드 물리적 마운트 상태 여부
    RECORDING_NOW  = 2,      ///< 현재 세션 기록 활성화 작동 상태 여부
    SENSOR_FAULT   = 3       ///< 센서 계통 통신 이상 여부
};

/**
 * @enum EM_DetectionResult_t
 * @brief 트리거 감지 판정 결과 코드
 */
enum class EM_DetectionResult_t : uint8_t {
    PASS           = 0,      ///< 정상 (이상 무)
    RULE_VIB_NG,            ///< 룰베이스 가속도/자이로 기준 이상 판단
    RULE_AUDIO_NG,          ///< 룰베이스 오디오 기준 이상 판단
    TEST_NG,                ///< 강제 진단 모드 판단
    ML_NG                   ///< 머신러닝/AI 모델 기준 이상 판단
};

/**
 * @enum EM_SystemCommand_t
 * @brief 디바이스 외부에서 수신하는 제어 명령 열거형
 */
enum class EM_SystemCommand_t : uint8_t {
    CMD_START          = 0,  ///< 감시(Monitoring) 개시
    CMD_STOP,                ///< 감시/세션 중지
    CMD_LEARN_NOISE,         ///< 소음 백그라운드 학습 기동
    CMD_CALIBRATE,           ///< 캘리브레이션 개시
    CMD_REBOOT,              ///< 칩 소프트웨어 리셋 및 부팅
    CMD_OTA_START,           ///< FOTA 진입 준비
    CMD_OTA_END,             ///< FOTA 마무리
    CMD_MANUAL_REC_START,    ///< 수동 임시 기록 세션 개시
    CMD_MANUAL_REC_STOP,     ///< 수동 임시 기록 세션 중지
    CMD_TUNING_PREVIEW,      ///< 필터 튜닝 정보 프리뷰 적용
    CMD_TUNING_SAVE,         ///< 필터 튜닝 정보 플래시 저장
    CMD_TUNING_CANCEL        ///< 필터 튜닝 취소 및 롤백
};

/**
 * @enum EM_AsyncSessionCmd_t
 * @brief 스토리지 세션 오프닝/클로징 통신 제어 신호
 */
enum class EM_AsyncSessionCmd_t : uint8_t {
    NONE = 0,
    OPEN_AUTO,
    OPEN_MANUAL,
    OPEN_CALIB_MAN,
    CLOSE_NORMAL,
    CLOSE_MANUAL,
    CLOSE_CALIB_DONE
};

/**
 * @enum EM_TriggerSource_t
 * @brief 감지 이벤트 판정을 발생시킨 소스 정보
 */
enum class EM_TriggerSource_t : uint8_t {
    NONE           = 0,
    HW_WAKE,                 ///< 가속도계 WOM 하드웨어 인터럽트
    SW_RMS,                  ///< 가속도/자이로/소음 소프트웨어 RMS 임계초과
    SW_BAND,                 ///< 개별 주파수 대역 에너지 임계초과
    MANUAL                   ///< 사용자 직접 이벤트 강제 생성
};

/**
 * @enum EM_StreamType_t
 * @brief 웹소켓 통신 시 패킷 바이너리의 특성을 전달하는 스트림 분류 마커
 */
enum class EM_StreamType_t : uint8_t {
    TELEMETRY      = 0x01,   ///< 텔레메트리 연산값 패킷
    SPECTRUM       = 0x02,   ///< 실시간 FFT 파워 스펙트럼 패킷
    WAVEFORM       = 0x03,   ///< 실시간 타임 도메인 파형 신호 패킷
    CALIBRATION    = 0x04,   ///< 캘리브레이션 튜닝 파라미터 패킷
    SEQUENCE       = 0x05    ///< 머신러닝 추론용 데이터 시퀀스 플랫 패킷
};

/**
 * @enum EM_NoiseMode_t
 * @brief 소음 차감 프로파일 관리 모드
 */
enum class EM_NoiseMode_t : uint8_t {
    OFF            = 0,
    FIXED,
    ADAPTIVE
};

/**
 * @enum EM_WiFiMode_t
 * @brief 디바이스 WiFi 통신 무선 모드
 */
enum class EM_WiFiMode_t : uint8_t {
    STA_ONLY       = 0,
    AP_ONLY,
    AP_STA,
    AUTO_FALLBACK
};

/**
 * @enum EM_WindowType_t
 * @brief FFT 파워 스펙트럼 연산 시 적용할 창함수 종류
 */
enum class EM_WindowType_t : uint8_t {
    HANN           = 0,
    HAMMING,
    BLACKMAN
};


// ========================================================================
// [PART 2] 동적 설정 구조체 (런타임 제어 및 스위치 기능 통합)
// ========================================================================

/**
 * @struct ST_Global_WiFi_t
 * @brief 디바이스 무선 와이파이 다중 접속 및 AP 설정 정보
 */
struct ST_Global_WiFi_t {
    EM_WiFiMode_t mode;
    char          ap_ssid[T2_Def::Global::NetLimit::NET_SSID_LEN_MAX];
    char          ap_pw[T2_Def::Global::NetLimit::NET_PW_LEN_MAX];
    char          ap_ip[T2_Def::Global::NetLimit::NET_IP_LEN_MAX];
    char          multi_ssid[T2_Def::Global::NetLimit::NET_MULTI_AP_MAX][T2_Def::Global::NetLimit::NET_SSID_LEN_MAX];
    char          multi_pw[T2_Def::Global::NetLimit::NET_MULTI_AP_MAX][T2_Def::Global::NetLimit::NET_PW_LEN_MAX];
    uint32_t      disconnect_delay_ms;
};

/**
 * @struct ST_Global_Mqtt_t
 * @brief MQTT 브로커 통신 접속 및 기본 토픽 설정 정보
 */
struct ST_Global_Mqtt_t {
    bool     enable;
    char     broker[T2_Def::Global::NetLimit::NET_BROKER_LEN_MAX];
    uint16_t port;
    char     id[T2_Def::Global::NetLimit::NET_ID_LEN_MAX];
    char     pw[T2_Def::Global::NetLimit::NET_PW_LEN_MAX];
    char     topic_root[T2_Def::Global::NetLimit::NET_TOPIC_LEN_MAX];
    char     lwt_topic[T2_Def::Global::NetLimit::NET_TOPIC_LEN_MAX];
    uint8_t  qos;
    uint8_t  proto_ver;
};

/**
 * @struct ST_Global_NTP_t
 * @brief 타임 동기화를 위한 NTP 서버 IP 설정 정보
 */
struct ST_Global_NTP_t {
    char     ntp_server1[T2_Def::Global::NetLimit::NET_BROKER_LEN_MAX];
    char     ntp_server2[T2_Def::Global::NetLimit::NET_BROKER_LEN_MAX];
    char     ntp_tz[32];
};

/**
 * @struct ST_Global_Storage_t
 * @brief 스토리지(SD카드 등) 파일 세션 관리 및 회전 한계값 설정 정보
 */
struct ST_Global_Storage_t {
    uint32_t  rot_mb;
    uint32_t  rot_min;
    bool      save_raw;
    uint16_t  keep_max;
    uint32_t  idle_flush_ms;
    uint8_t   pre_trig_sec;
};

/**
 * @struct ST_Global_System_t
 * @brief 디바이스 아이디 및 송출 주기 등 전역 시스템 파라미터
 */
struct ST_Global_System_t {
    char          site_id[T2_Def::Global::System::SITE_ID_LEN_MAX];
    uint8_t       tele_hz;
    uint8_t       wave_hz;
    EM_OpMode_t   op_mode;
    uint32_t      watchdog_ms;
};

/**
 * @struct ST_Global_Decision_t
 * @brief MLOps 레벨의 트리거 동작 의사결정 인계 구역 설정 구조체
 */
struct ST_Global_Decision_t {
    uint8_t max_trial_count;     ///< 연속 감지 실패 제한 횟수
    float   sta_lta_threshold;   ///< 감지비 임계치
    int     min_trigger_count;   ///< 유효 판단 감지 카운트 최솟값
    float   valid_start_sec;     ///< 세션 유효 판정 적용 개시 초
    float   valid_end_sec;       ///< 세션 유효 판정 중단 종료 초
};

/**
 * @struct ST_Global_Output_t
 * @brief 특징 텐서 시퀀스의 출력 형태 관련 스펙트럼
 */
struct ST_Global_Output_t {
    bool     enabled;
    bool     output_sequence;
    uint16_t sequence_frames;
};

/**
 * @struct ST_FilterFIR_t
 * @brief FIR 계수 연산 정보 (활성화, 컷오프 주파수, Taps 계수 크기)
 */
struct ST_FilterFIR_t {
    bool     en;
    float    cutoff;
    uint16_t taps;
};

/**
 * @struct ST_FilterIIR_t
 * @brief Biquad IIR 계수 연산 정보
 */
struct ST_FilterIIR_t {
    bool  en;
    float cutoff;
    float q;
};

/**
 * @struct ST_FilterNotch_t
 * @brief 특정 단 대역 제거용 IIR Notch 파라미터
 */
struct ST_FilterNotch_t {
    bool  en;
    float freq;
    float gain;
    float q;
};

/**
 * @struct ST_Dsp_Config_t
 * @brief DC 리무버, 메디안 윈도우 크기, 각종 필터 조합 구성을 위한 DSP 설정 구조체
 */
struct ST_Dsp_Config_t {
    bool             rem_dc;
    bool             med_en;
    uint8_t          med_win;
    ST_FilterFIR_t   hpf;
    ST_FilterFIR_t   lpf;
    ST_FilterIIR_t   iir_hpf;
    ST_FilterIIR_t   iir_lpf;
    ST_FilterNotch_t notch;
    ST_FilterNotch_t notch2;
    EM_WindowType_t  win_type;
};

/**
 * @struct ST_Accel_Config_t
 * @brief 가속도 센서(진동) 취득 축 활성화 마스크 및 계수 설정 정보
 */
struct ST_Accel_Config_t {
    bool     enable;
    uint8_t  axis_mask;
    uint8_t  range;
    uint8_t  odr;
    uint8_t  bwp;
    uint8_t  filter_perf;
    uint16_t fifo_watermark;
    uint32_t sample_rate;
    uint32_t fft_size;
    bool     motion_en;
    float    wake_g;
    uint16_t wake_dur;
    uint8_t  noise_perf;

    float    rms_thresh[T2_Def::Accel::Sensor::AXIS_MAX];
    float    kurt_ng_thresh[T2_Def::Accel::Sensor::AXIS_MAX];
    float    crest_ng_thresh[T2_Def::Accel::Sensor::AXIS_MAX];
    float    skew_ng_thresh[T2_Def::Accel::Sensor::AXIS_MAX];

    uint8_t  active_band_count;
    bool     band_en[T2_Def::Accel::FeatureLimit::BAND_MAX];
    float    band_start[T2_Def::Accel::FeatureLimit::BAND_MAX];
    float    band_end[T2_Def::Accel::FeatureLimit::BAND_MAX];
    float    band_thresh[T2_Def::Accel::Sensor::AXIS_MAX][T2_Def::Accel::FeatureLimit::BAND_MAX];

    float    offset[T2_Def::Accel::Sensor::AXIS_MAX];
    float    gain[T2_Def::Accel::Sensor::AXIS_MAX];

    float    peak_amp_min;
    float    peak_freq_gap_min;

    ST_Dsp_Config_t dsp;
};

/**
 * @struct ST_Gyro_Config_t
 * @brief 자이로 센서 취득 축 활성화 마스크 및 계수 설정 정보
 */
struct ST_Gyro_Config_t {
    bool     enable;
    uint8_t  axis_mask;
    uint8_t  range;
    uint8_t  odr;
    uint8_t  bwp;
    uint8_t  filter_perf;
    uint8_t  noise_perf;
    uint32_t sample_rate;
    uint32_t fft_size;

    float    rms_thresh[T2_Def::Gyro::Sensor::AXIS_MAX];
    float    kurt_ng_thresh[T2_Def::Gyro::Sensor::AXIS_MAX];
    float    crest_ng_thresh[T2_Def::Gyro::Sensor::AXIS_MAX];
    float    skew_ng_thresh[T2_Def::Gyro::Sensor::AXIS_MAX];

    uint8_t  active_band_count;
    bool     band_en[T2_Def::Gyro::FeatureLimit::BAND_MAX];
    float    band_start[T2_Def::Gyro::FeatureLimit::BAND_MAX];
    float    band_end[T2_Def::Gyro::FeatureLimit::BAND_MAX];
    float    band_thresh[T2_Def::Gyro::Sensor::AXIS_MAX][T2_Def::Gyro::FeatureLimit::BAND_MAX];

    float    offset[T2_Def::Gyro::Sensor::AXIS_MAX];
    float    gain[T2_Def::Gyro::Sensor::AXIS_MAX];

    float    peak_amp_min;
    float    peak_freq_gap_min;

    ST_Dsp_Config_t dsp;
};

/**
 * @struct ST_Audio_Noise_t
 * @brief 마이크 오디오 소음 차감 및 게이트 설정 파라미터
 */
struct ST_Audio_Noise_t {
    bool      gate_en;
    float     gate_thresh;
    EM_NoiseMode_t mode;
    float     sub_str;
    float     adp_alpha;
    uint16_t  learn_frames;
};

/**
 * @struct ST_Audio_Config_t
 * @brief I2S 마이크 오디오 취득 및 특징량 추출용 설정 정보
 */
struct ST_Audio_Config_t {
    bool     enable;
    uint8_t  channel_mask;
    uint32_t sample_rate;
    uint32_t fft_size;

    float    rms_thresh[T2_Def::Audio::Sensor::CHANNELS_MAX];
    float    kurt_ng_thresh[T2_Def::Audio::Sensor::CHANNELS_MAX];
    float    crest_ng_thresh[T2_Def::Audio::Sensor::CHANNELS_MAX];
    float    skew_ng_thresh[T2_Def::Audio::Sensor::CHANNELS_MAX];

    uint8_t  active_band_count;
    bool     band_en[T2_Def::Audio::FeatureLimit::BAND_MAX];
    float    band_start[T2_Def::Audio::FeatureLimit::BAND_MAX];
    float    band_end[T2_Def::Audio::FeatureLimit::BAND_MAX];
    float    band_thresh[T2_Def::Audio::Sensor::CHANNELS_MAX][T2_Def::Audio::FeatureLimit::BAND_MAX];

    ST_Audio_Noise_t noise;
    bool     pre_en;
    float    pre_alpha;
    float    beam_gain;
    float    window_ms;
    float    hop_ms;

    uint32_t auto_idle_min;
    float    ref_freq;
    float    filt_min;
    float    filt_max;
    float    gain_max;
    float    gain_min;
    float    norm_safe;
    float    gain_ch[T2_Def::Audio::Sensor::CHANNELS_MAX];
    alignas(16) float eq_coeffs[T2_Def::Audio::Sensor::CHANNELS_MAX][T2_Def::Audio::FeatureLimit::FIR_TAPS_MAX];

    uint8_t mel_bands;

    uint8_t  active_ceps_count;
    uint8_t  active_peak_count;
    float    ceps_targets[T2_Def::Audio::FeatureLimit::CEPS_TARGET_MAX];
    float    f_min;
    float    f_max;
    float    peak_amp_min;
    float    peak_freq_gap_min;

    ST_Dsp_Config_t dsp;
};

/**
 * @struct ST_DynamicConfig_t
 * @brief 디바이스의 마스터 제어용 동적 설정 구조체 (SSOT)
 */
struct ST_DynamicConfig_t {
    ST_Global_System_t   system;
    ST_Global_WiFi_t     wifi;
    ST_Global_Mqtt_t     mqtt;
    ST_Global_NTP_t      ntp;
    ST_Global_Storage_t  storage;
    ST_Global_Output_t   output;
    ST_Global_Decision_t decision;

    ST_Accel_Config_t    accel;
    ST_Gyro_Config_t     gyro;
    ST_Audio_Config_t    audio;

    uint32_t             trig_hold_ms;
    bool                 trig_use_sleep;
    uint32_t             trig_sleep_sec;
};

/**
 * @struct ST_Runtime_TriggerCtx_t
 * @brief 런타임 이벤트 홀드 및 스케줄 상태 관리 구조체
 */
struct ST_Runtime_TriggerCtx_t {
    bool               is_triggered;
    EM_TriggerSource_t active_source;
    uint32_t           hold_end_tick;
};


// ============================================================================
// [PART 3] 원시 데이터(Raw) 및 진단 특징량 데이터 모델 (16-Byte Aligned)
// ============================================================================

/**
 * @struct ST_Raw_Accel_t
 * @brief 16바이트 정렬된 가속도 원시(Raw) 데이터 버퍼 구조체
 */
struct alignas(16) ST_Raw_Accel_t {
    uint64_t ts;
    uint32_t sample_rate;
    uint8_t  active_mask;
    float    data[T2_Def::Accel::Sensor::AXIS_MAX][T2_Def::Accel::Sensor::FFT_SIZE_MAX];
};

/**
 * @struct ST_Raw_Gyro_t
 * @brief 16바이트 정렬된 자이로 원시 데이터 버퍼 구조체
 */
struct alignas(16) ST_Raw_Gyro_t {
    uint64_t ts;
    uint32_t sample_rate;
    uint8_t  active_mask;
    float    data[T2_Def::Gyro::Sensor::AXIS_MAX][T2_Def::Gyro::Sensor::FFT_SIZE_MAX];
};

/**
 * @struct ST_Raw_Audio_t
 * @brief 16바이트 정렬된 마이크 오디오 원시 데이터 버퍼 구조체
 */
struct alignas(16) ST_Raw_Audio_t {
    uint64_t ts;
    uint32_t sample_rate;
    uint8_t  active_mask;
    float    data[T2_Def::Audio::Sensor::CHANNELS_MAX][T2_Def::Audio::Sensor::FFT_SIZE_MAX];
};

/**
 * @struct ST_SpectralPeak_t
 * @brief 주파수 피크 위치(Hz) 및 진폭 세기 저장체
 */
struct ST_SpectralPeak_t {
    float freq;
    float amp;
};

/**
 * @struct ST_Slot_Header_t
 * @brief 통합 추출 데이터 슬롯용 메타정보 헤더
 */
struct ST_Slot_Header_t {
    uint64_t ts;
    uint32_t fid;
    uint32_t uptime;
    uint8_t  flags;
    uint8_t  trial;
    uint8_t  src;
    uint8_t  accel_mask;
    uint8_t  gyro_mask;
    uint8_t  audio_mask;
    uint8_t  payload_type;
    uint8_t  _pad;
    float    temp;
    uint8_t  _res[4];
};

/**
 * @struct ST_Slot_Accel_t
 * @brief 가속도 3축의 시간/주파수 특징량 지표 보관용 (16바이트 정렬)
 */
struct ST_Slot_Accel_t {
    float rms[T2_Def::Accel::Sensor::AXIS_MAX];
    float peak_f[T2_Def::Accel::Sensor::AXIS_MAX];
    float centroid[T2_Def::Accel::Sensor::AXIS_MAX];
    float kurt[T2_Def::Accel::Sensor::AXIS_MAX];
    float crest[T2_Def::Accel::Sensor::AXIS_MAX];
    float sta_lta[T2_Def::Accel::Sensor::AXIS_MAX];
    float skew[T2_Def::Accel::Sensor::AXIS_MAX];
    float std[T2_Def::Accel::Sensor::AXIS_MAX];
    float cal_off[T2_Def::Accel::Sensor::AXIS_MAX];
    float band_energy[T2_Def::Accel::Sensor::AXIS_MAX][T2_Def::Accel::FeatureLimit::BAND_MAX];
    float _pad[1];
};

/**
 * @struct ST_Slot_Gyro_t
 * @brief 자이로 3축의 시간/주파수 특징량 지표 보관용 (16바이트 정렬)
 */
struct ST_Slot_Gyro_t {
    float rms[T2_Def::Gyro::Sensor::AXIS_MAX];
    float peak_f[T2_Def::Gyro::Sensor::AXIS_MAX];
    float centroid[T2_Def::Gyro::Sensor::AXIS_MAX];
    float kurt[T2_Def::Gyro::Sensor::AXIS_MAX];
    float crest[T2_Def::Gyro::Sensor::AXIS_MAX];
    float sta_lta[T2_Def::Gyro::Sensor::AXIS_MAX];
    float skew[T2_Def::Gyro::Sensor::AXIS_MAX];
    float std[T2_Def::Gyro::Sensor::AXIS_MAX];
    float cal_off[T2_Def::Gyro::Sensor::AXIS_MAX];
    float drift_est[T2_Def::Gyro::Sensor::AXIS_MAX];
    float band_energy[T2_Def::Gyro::Sensor::AXIS_MAX][T2_Def::Gyro::FeatureLimit::BAND_MAX];
    float _pad[2];
};

/**
 * @struct ST_Audio_Channel_Slot_t
 * @brief 오디오 단일 채널 전용 특징량 지표 (RMS, 센트로이드, 켑스트럼)
 */
struct ST_Audio_Channel_Slot_t {
    float        rms;
    float        energy;
    float        centroid;
    float        kurt;
    float        crest;
    float        skew;
    float        sta_lta;
    float        d_rms;
    float        dd_rms;
    float        cpsr_max[T2_Def::Audio::FeatureLimit::CEPS_TARGET_MAX];
    float        cpsr_mxr[T2_Def::Audio::FeatureLimit::CEPS_TARGET_MAX];
    float        band_energy[T2_Def::Audio::FeatureLimit::BAND_MAX];
    ST_SpectralPeak_t top_peaks[T2_Def::Audio::FeatureLimit::TOP_PEAKS_MAX];
};

/**
 * @struct ST_Slot_Audio_t
 * @brief 오디오 스테레오 채널의 결합 지표 보관용 (16바이트 정렬)
 */
struct ST_Slot_Audio_t {
    ST_Audio_Channel_Slot_t ch[T2_Def::Audio::Sensor::CHANNELS_MAX];
    float        coh;
    float        ipd;
};

/**
 * @struct ST_Slot_Tensor_t
 * @brief 머신러닝 추론용 MFCC 특징 벡터 데이터 저장 슬롯 (16바이트 정렬)
 */
struct ST_Slot_Tensor_t {
    alignas(16) float mfcc[T2_Def::AI::Tensor::MFCC_DIM_MAX];
};

/**
 * @struct ST_UnifiedFeatureSlot_t
 * @brief 헤더, 가속도, 자이로, 오디오, 텐서가 집약된 통합 연산 슬롯 (16바이트 정렬)
 */
struct alignas(16) ST_UnifiedFeatureSlot_t {
    ST_Slot_Header_t header;
    ST_Slot_Accel_t  accel;
    ST_Slot_Gyro_t   gyro;
    ST_Slot_Audio_t  audio;
    ST_Slot_Tensor_t tensor;
};


// ============================================================================
// [PART 4] 통신 및 바이너리 패킷 규격 (Byte Packing & _MAX 기반 고정 평탄화)
// ============================================================================
#pragma pack(push, 1)

/**
 * @struct ST_FileHeader_t
 * @brief SD카드 저장 바이너리 파일 메타데이터 기록을 위한 고정 규격 파일 헤더
 */
struct ST_FileHeader_t {
    char     magic[4];               ///< 매직넘버 마커
    uint16_t ver;                    ///< 저장 파일 포맷 버전
    uint16_t struct_size;            ///< 레코드 바이트 사이즈
    uint32_t s_rate;                 ///< 취득 샘플 레이트
    uint16_t fft;                    ///< FFT 분석 빈 크기
    uint16_t mfcc_d;                 ///< MFCC 차원 수
    uint8_t  accel_mask;
    uint8_t  gyro_mask;
    uint8_t  audio_mask;
    uint8_t  _res;
    uint32_t total;                  ///< 총 파일 내 레코드 수
    char     config_dump[8192];      ///< 당시 활성화되었던 JSON 설정 문자열 백업 영역
};

/**
 * @struct ST_WsHeader_t
 * @brief 실시간 웹소켓 방송 시 선행되는 팩키드 헤더 규격
 */
struct ST_WsHeader_t {
    uint8_t  magic;                  ///< 웹소켓 식별용 마커 (0xAA)
    uint8_t  type;                   ///< 패킷 타입 (EM_StreamType_t)
    uint16_t len;                    ///< 실제 탑재 페이로드의 바이트 크기
    uint8_t  stage;                  ///< 현재 FSM 상태 코드 값
    uint8_t  source;                 ///< 발생 채널/축 정보 인덱스
    uint8_t  _pad[2];
};

/**
 * @struct ST_PktTelemetry_t
 * @brief 웹소켓 브로드캐스트용 평탄화(Flattened) 통합 특징량 결과 전송 패킷
 */
struct ST_PktTelemetry_t {
    ST_WsHeader_t header;
    uint8_t  sys_state;
    uint8_t  detect_result;
    uint8_t  trial_no;
    uint8_t  trigger_source;
    uint8_t  accel_mask;
    uint8_t  gyro_mask;
    uint8_t  audio_mask;
    uint8_t  payload_type;

    uint64_t accel_ts;
    uint64_t gyro_ts;
    uint64_t audio_ts;

    float    accel_rms[T2_Def::Accel::Sensor::AXIS_MAX];
    float    accel_centroid[T2_Def::Accel::Sensor::AXIS_MAX];
    float    accel_kurtosis[T2_Def::Accel::Sensor::AXIS_MAX];
    float    accel_crest_factor[T2_Def::Accel::Sensor::AXIS_MAX];
    float    accel_sta_lta_ratio[T2_Def::Accel::Sensor::AXIS_MAX];
    float    accel_skewness[T2_Def::Accel::Sensor::AXIS_MAX];
    float    accel_std[T2_Def::Accel::Sensor::AXIS_MAX];
    float    accel_band_energy[T2_Def::Accel::Sensor::AXIS_MAX][T2_Def::Accel::FeatureLimit::BAND_MAX];

    float    gyro_rms[T2_Def::Gyro::Sensor::AXIS_MAX];
    float    gyro_centroid[T2_Def::Gyro::Sensor::AXIS_MAX];
    float    gyro_kurtosis[T2_Def::Gyro::Sensor::AXIS_MAX];
    float    gyro_crest_factor[T2_Def::Gyro::Sensor::AXIS_MAX];
    float    gyro_sta_lta_ratio[T2_Def::Gyro::Sensor::AXIS_MAX];
    float    gyro_skewness[T2_Def::Gyro::Sensor::AXIS_MAX];
    float    gyro_std[T2_Def::Gyro::Sensor::AXIS_MAX];
    float    gyro_drift_est[T2_Def::Gyro::Sensor::AXIS_MAX];
    float    gyro_band_energy[T2_Def::Gyro::Sensor::AXIS_MAX][T2_Def::Gyro::FeatureLimit::BAND_MAX];

    float    audio_rms[T2_Def::Audio::Sensor::CHANNELS_MAX];
    float    audio_energy[T2_Def::Audio::Sensor::CHANNELS_MAX];
    float    audio_kurtosis[T2_Def::Audio::Sensor::CHANNELS_MAX];
    float    audio_crest_factor[T2_Def::Audio::Sensor::CHANNELS_MAX];
    float    audio_sta_lta_ratio[T2_Def::Audio::Sensor::CHANNELS_MAX];
    float    audio_skewness[T2_Def::Audio::Sensor::CHANNELS_MAX];
    float    audio_spectral_centroid[T2_Def::Audio::Sensor::CHANNELS_MAX];
    float    audio_coh;
    float    audio_ipd;
    float    audio_band_energy[T2_Def::Audio::Sensor::CHANNELS_MAX][T2_Def::Audio::FeatureLimit::BAND_MAX];
    float    audio_peak_freqs[T2_Def::Audio::Sensor::CHANNELS_MAX][T2_Def::Audio::FeatureLimit::TOP_PEAKS_MAX];
    float    audio_peak_amps[T2_Def::Audio::Sensor::CHANNELS_MAX][T2_Def::Audio::FeatureLimit::TOP_PEAKS_MAX];
    float    audio_cpsr_max[T2_Def::Audio::Sensor::CHANNELS_MAX][T2_Def::Audio::FeatureLimit::CEPS_TARGET_MAX];
    float    audio_cpsr_mxrms[T2_Def::Audio::Sensor::CHANNELS_MAX][T2_Def::Audio::FeatureLimit::CEPS_TARGET_MAX];

    float    mfcc[T2_Def::AI::Tensor::MFCC_DIM_MAX];
};

/**
 * @struct ST_PktSpectrum_t
 * @brief 웹소켓 브로드캐스트용 오디오 주파수 스펙트럼 전송 패킷
 */
struct ST_PktSpectrum_t {
    ST_WsHeader_t header;
    float    frequencies[(T2_Def::Audio::Sensor::FFT_SIZE_MAX / 2) + 1];
};

/**
 * @struct ST_PktWaveformAudio_t
 * @brief 웹소켓 브로드캐스트용 타임도메인 오디오 원시 파형 패킷
 */
struct ST_PktWaveformAudio_t {
    ST_WsHeader_t header;
    float    samples[T2_Def::Audio::Sensor::FFT_SIZE_MAX];
};

/**
 * @struct ST_PktWaveformAccel_t
 * @brief 웹소켓 브로드캐스트용 타임도메인 가속도 원시 파형 패킷
 */
struct ST_PktWaveformAccel_t {
    ST_WsHeader_t header;
    float    samples[T2_Def::Accel::Sensor::FFT_SIZE_MAX];
};

/**
 * @struct ST_PktWaveformGyro_t
 * @brief 웹소켓 브로드캐스트용 타임도메인 자이로 원시 파형 패킷
 */
struct ST_PktWaveformGyro_t {
    ST_WsHeader_t header;
    float    samples[T2_Def::Gyro::Sensor::FFT_SIZE_MAX];
};

/**
 * @struct ST_PktCalibration_t
 * @brief 웹소켓 브로드캐스트용 캘리브레이션 매트릭스 패킷
 */
struct ST_PktCalibration_t {
    ST_WsHeader_t header;
    float    target_gain_curve[(T2_Def::Audio::Sensor::FFT_SIZE_MAX / 2) + 1];
    float    actual_fir_coeffs[T2_Def::Audio::FeatureLimit::FIR_TAPS_MAX];
};

/**
 * @struct ST_PktSequence_t
 * @brief 웹소켓 브로드캐스트용 AI 시퀀스 데이터 플랫 패킷
 */
struct ST_PktSequence_t {
    ST_WsHeader_t header;
    float    data[T2_Def::Global::System::SEQUENCE_FRAMES_MAX * T2_Def::AI::Tensor::MFCC_DIM_MAX];
};

#pragma pack(pop)

}  // namespace T2_Type
