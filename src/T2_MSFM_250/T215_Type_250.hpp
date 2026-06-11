

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
#include "T210_Def_250.hpp"

namespace T2_Type {

// ========================================================================
// [PART 1] 시스템 열거형 (Enum Classes)
// ========================================================================

enum class EM_SystemState_t : uint8_t {
    INIT           = 0,
    READY,
    MONITORING,
    RECORDING,
    NOISE_LEARNING,
    MAINTENANCE,
    ERROR,
    CALIBRATING
};

enum class EM_OpMode_t : uint8_t {
    MANUAL         = 0,
    AUTO,
    SCHEDULE
};

enum class EM_AxisMask_t : uint8_t {
    NONE           = 0,
    AXIS_X         = 1 << 0, // 0b001
    AXIS_Y         = 1 << 1, // 0b010
    AXIS_Z         = 1 << 2, // 0b100
    AXIS_ALL       = 0b111
};

enum class EM_ChannelMask_t : uint8_t {
    NONE           = 0,
    CH_LEFT        = 1 << 0, // 0b01
    CH_RIGHT       = 1 << 1, // 0b10
    CH_STEREO      = 0b11
};

enum class EM_DataPayloadType_t : uint8_t {
    VIB_ONLY       = 1,
    AUDIO_ONLY     = 2,
    VIB_AUDIO_BOTH = 3
};

enum class EM_StatusBit_t : uint8_t {
    NTP_SYNCED     = 0,
    SD_MOUNTED     = 1,
    RECORDING_NOW  = 2,
    SENSOR_FAULT   = 3
};

enum class EM_DetectionResult_t : uint8_t {
    PASS           = 0,
    RULE_VIB_NG,
    RULE_AUDIO_NG,
    TEST_NG,
    ML_NG
};

enum class EM_SystemCommand_t : uint8_t {
    CMD_START          = 0,
    CMD_STOP,
    CMD_LEARN_NOISE,
    CMD_CALIBRATE,
    CMD_REBOOT,
    CMD_OTA_START,
    CMD_OTA_END,
    CMD_MANUAL_REC_START,
    CMD_MANUAL_REC_STOP,
    CMD_TUNING_PREVIEW,
    CMD_TUNING_SAVE,
    CMD_TUNING_CANCEL
};

enum class EM_AsyncSessionCmd_t : uint8_t {
    NONE = 0,
    OPEN_AUTO,
    OPEN_MANUAL,
    OPEN_CALIB_MAN,
    CLOSE_NORMAL,
    CLOSE_MANUAL,
    CLOSE_CALIB_DONE
};

enum class EM_TriggerSource_t : uint8_t {
    NONE           = 0,
    HW_WAKE,
    SW_RMS,
    SW_BAND,
    MANUAL
};

enum class EM_StreamType_t : uint8_t {
    TELEMETRY      = 0x01,
    SPECTRUM       = 0x02,
    WAVEFORM       = 0x03,
    CALIBRATION    = 0x04,
    SEQUENCE       = 0x05
};

enum class EM_NoiseMode_t : uint8_t {
    OFF            = 0,
    FIXED,
    ADAPTIVE
};

enum class EM_WiFiMode_t : uint8_t {
    STA_ONLY       = 0,
    AP_ONLY,
    AP_STA,
    AUTO_FALLBACK
};

enum class EM_WindowType_t : uint8_t {
    HANN           = 0,
    HAMMING,
    BLACKMAN
};


// ========================================================================
// [PART 2] 동적 설정 구조체 (런타임 제어 및 스위치 기능 통합)
// ========================================================================

// ------------------------------------------------------------------------
// Tier 1. Global (디바이스 전역 인프라)
// ------------------------------------------------------------------------

struct ST_Global_WiFi_t {
    EM_WiFiMode_t mode;
    char          ap_ssid[T2_Def::Global::NetLimit::NET_SSID_LEN_MAX];
    char          ap_pw[T2_Def::Global::NetLimit::NET_PW_LEN_MAX];
    char          ap_ip[T2_Def::Global::NetLimit::NET_IP_LEN_MAX];
    char          multi_ssid[T2_Def::Global::NetLimit::NET_MULTI_AP_MAX][T2_Def::Global::NetLimit::NET_SSID_LEN_MAX];
    char          multi_pw[T2_Def::Global::NetLimit::NET_MULTI_AP_MAX][T2_Def::Global::NetLimit::NET_PW_LEN_MAX];
    uint32_t      disconnect_delay_ms;  // [이슈 9] Wi-Fi 재연결 대기 시간 (ms)
};

struct ST_Global_Mqtt_t {
    bool     enable;
    char     broker[T2_Def::Global::NetLimit::NET_BROKER_LEN_MAX];
    uint16_t port;
    char     id[T2_Def::Global::NetLimit::NET_ID_LEN_MAX];
    char     pw[T2_Def::Global::NetLimit::NET_PW_LEN_MAX];
    char     topic_root[T2_Def::Global::NetLimit::NET_TOPIC_LEN_MAX];
    char     lwt_topic[T2_Def::Global::NetLimit::NET_TOPIC_LEN_MAX];
    uint8_t  qos;
    uint8_t  proto_ver;                                              // 타임존 문자열 (예: KST-9)
};

struct ST_Global_NTP_t {
    char     ntp_server1[T2_Def::Global::NetLimit::NET_BROKER_LEN_MAX];   // 주 NTP 서버 주소
    char     ntp_server2[T2_Def::Global::NetLimit::NET_BROKER_LEN_MAX];   // 보조 NTP 서버 주소
    char     ntp_tz[32];                                                  // 타임존 문자열 (예: KST-9)
};

struct ST_Global_Storage_t {
    uint32_t  rot_mb;
    uint32_t  rot_min;
    bool      save_raw;
    uint16_t  keep_max;
    uint32_t  idle_flush_ms;
    uint8_t   pre_trig_sec;
};

struct ST_Global_System_t {
    char          site_id[T2_Def::Global::System::SITE_ID_LEN_MAX];  // [이슈 2] SITE_ID_LEN_MAX 상수로 매직넘버 제거
    uint8_t       tele_hz;
    uint8_t       wave_hz;
    EM_OpMode_t   op_mode;
    uint32_t      watchdog_ms;
};

// MLOps 의사결정/트리거 파라미터 동적 제어 구조체
struct ST_Global_Decision_t {
    uint8_t max_trial_count;     // 노이즈 오작동 검증 최대 횟수 (기본: MAX_TRIAL_COUNT_DEF)
    float   sta_lta_threshold;   // 단기/장기 급변 감지 비율 임계치 (기본: STA_LTA_THRESHOLD_DEF)
    int     min_trigger_count;   // 유효 판정 최소 연속 트리거 (기본: MIN_TRIGGER_COUNT_DEF)
    float   valid_start_sec;     // 유효 판정 시작 시간 초 (기본: VALID_START_SEC_DEF)
    float   valid_end_sec;       // 판정 종료 유효 시간 초 (기본: VALID_END_SEC_DEF)
};

struct ST_Global_Output_t {
    bool     enabled;
    bool     output_sequence;
    uint16_t sequence_frames;
};

// ------------------------------------------------------------------------
// 필터 구조체 원형 (각 도메인 내부에 종속되어 선언됨)
// ------------------------------------------------------------------------

struct ST_FilterFIR_t {
    bool     en;
    float    cutoff;
    uint16_t taps;
};

struct ST_FilterIIR_t {
    bool  en;
    float cutoff;
    float q;
};

struct ST_FilterNotch_t {
    bool  en;
    float freq;
    float gain;
    float q;
};

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


// ------------------------------------------------------------------------
// Tier 2. Accel (가속도 전용 파이프라인)
// ------------------------------------------------------------------------

struct ST_Accel_Config_t {
    bool     enable;
    uint8_t  axis_mask;
    uint8_t  range;
    uint8_t  odr;

    // BMI270 드라이버 하드웨어 필터 세부 속성 직렬화 멤버 확장
    uint8_t  bwp;               // BMI2_ACC_NORMAL_AVG4 바인딩
    uint8_t  filter_perf;       // BMI2_PERF_OPT_MODE 바인딩
    uint16_t fifo_watermark;    // watermark = 40 바인딩

    uint32_t sample_rate;
    uint32_t fft_size;

    bool     motion_en;
    float    wake_g;
    uint16_t wake_dur;

    uint8_t  noise_perf;  				// 가속도 저소음 모드 제어 (BMI270 bwp 파라미터 연동)

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

    ST_Dsp_Config_t dsp; // 축별 배열을 해체하고 도메인 공통 단일 구조체로 축소하여 메모리 절약
};


// ------------------------------------------------------------------------
// Tier 3. Gyro (자이로 전용 파이프라인)
// ------------------------------------------------------------------------

struct ST_Gyro_Config_t {
    bool     enable;
    uint8_t  axis_mask;
    uint8_t  range;
    uint8_t  odr;                // 가속도와 동일한 동기화 속도 추적 제어용 레지스터 속성

    // BMI270 드라이버 하드웨어 필터 세부 속성 직렬화 멤버 확장
    uint8_t  bwp;               // BMI2_GYR_NORMAL_MODE 바인딩
    uint8_t  filter_perf;       // 성능 최적화 모드 바인딩
    uint8_t  noise_perf;        // 노이즈 최적화 모드 바인딩


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

    ST_Dsp_Config_t dsp; // [수정] dsp[3] 축별 배열을 해체하고 도메인 공통 단일 구조체로 축소하여 메모리 절약
};


// ------------------------------------------------------------------------
// Tier 4. Audio (소음 전용 파이프라인)
// ------------------------------------------------------------------------

struct ST_Audio_Noise_t {
    bool           gate_en;
    float          gate_thresh;
    EM_NoiseMode_t mode;
    float          sub_str;
    float          adp_alpha;
    uint16_t       learn_frames;
};

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

	uint8_t mel_bands;	// 기본값: MEL_BANDS_DEF(26) (reloadAudioMelFilter 및 _computeMfcc에서 동적 반영 완료)


    uint8_t  active_ceps_count;
    uint8_t  active_peak_count;
    float    ceps_targets[T2_Def::Audio::FeatureLimit::CEPS_TARGET_MAX];
    float    f_min;
    float    f_max;
    float    peak_amp_min;
    float    peak_freq_gap_min;

    ST_Dsp_Config_t dsp; // [수정] dsp[2] 채널별 배열을 해체하고 도메인 공통 단일 구조체로 축소하여 메모리 절약
};


// ------------------------------------------------------------------------
// 마스터 동적 설정 구조체 (SSOT)
// ------------------------------------------------------------------------
struct ST_DynamicConfig_t {
    ST_Global_System_t   system;
    ST_Global_WiFi_t     wifi;
    ST_Global_Mqtt_t     mqtt;
	ST_Global_NTP_t		 ntp;
    ST_Global_Storage_t  storage;
    ST_Global_Output_t   output;
    ST_Global_Decision_t decision;  // [이슈 8] MLOps 의사결정 파라미터 동적 제어

    ST_Accel_Config_t    accel;
    ST_Gyro_Config_t     gyro;
    ST_Audio_Config_t    audio;

    uint32_t             trig_hold_ms;
    bool                 trig_use_sleep;
    uint32_t             trig_sleep_sec;
};

// ------------------------------------------------------------------------
// 런타임 제어 컨텍스트
// ------------------------------------------------------------------------
struct ST_Runtime_TriggerCtx_t {
    bool               is_triggered;
    EM_TriggerSource_t active_source;
    uint32_t           hold_end_tick;
};


// ============================================================================
// [PART 3] 원시 데이터(Raw) 및 진단 특징량 데이터 모델 (16-Byte Aligned)
// ============================================================================

struct alignas(16) ST_Raw_Accel_t {
    uint64_t ts;
    uint32_t sample_rate;
    uint8_t  active_mask;
    float    data[T2_Def::Accel::Sensor::AXIS_MAX][T2_Def::Accel::Sensor::FFT_SIZE_MAX];
};

struct alignas(16) ST_Raw_Gyro_t {
    uint64_t ts;
    uint32_t sample_rate;
    uint8_t  active_mask;
    float    data[T2_Def::Gyro::Sensor::AXIS_MAX][T2_Def::Gyro::Sensor::FFT_SIZE_MAX];
};

struct alignas(16) ST_Raw_Audio_t {
    uint64_t ts;
    uint32_t sample_rate;
    uint8_t  active_mask;
    float    data[T2_Def::Audio::Sensor::CHANNELS_MAX][T2_Def::Audio::Sensor::FFT_SIZE_MAX];
};

struct ST_SpectralPeak_t {
    float freq;
    float amp;
};

// [Block 1] 공통 헤더 (32 Bytes 완벽 정렬)
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

// [Block 2] 가속도 지표 (304 Bytes 완벽 정렬)
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
    float band_energy[T2_Def::Accel::Sensor::AXIS_MAX][T2_Def::Accel::FeatureLimit::BAND_MAX]; // 3*16*4 = 192 Bytes
    float _pad[1]; // 패딩 4 Bytes 추가 (300 + 4 = 304 Bytes, 16배수 정렬)
};

// [Block 3] 자이로 지표 (224 Bytes 완벽 정렬)
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
    float drift_est[T2_Def::Gyro::Sensor::AXIS_MAX]; // 자이로 특화 드리프트 지표
    float band_energy[T2_Def::Gyro::Sensor::AXIS_MAX][T2_Def::Gyro::FeatureLimit::BAND_MAX]; // 3*8*4 = 96 Bytes
    float _pad[2]; // 패딩 8 Bytes 추가 (216 + 8 = 224 Bytes, 16배수 정렬)
};

// [Block 4] 오디오 단일 채널 전용 (220 Bytes)
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

// [Block 4-2] 오디오 스테레오 결합 지표 (576 Bytes 완벽 정렬)
struct ST_Slot_Audio_t {
    ST_Audio_Channel_Slot_t ch[T2_Def::Audio::Sensor::CHANNELS_MAX]; // 220 * 2 = 440 Bytes
    float        timbre_bands[32];                                   // 128 Bytes (1/3 옥타브 대역 상대 비율)
    uint8_t      _pad[8];                                            // 8 Bytes 패딩으로 총 576 Bytes (16배수 정렬)
};

// [Block 5] AI 텐서 (3072 Bytes 완벽 정렬)
struct ST_Slot_Tensor_t {
    alignas(16) float mfcc[T2_Def::AI::Tensor::MFCC_DIM_MAX]; // 768 * 4 = 3072 Bytes
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
    float            mfcc[T2_Def::AI::Tensor::MFCC_COEFFS_DEF * T2_Def::AI::Tensor::MFCC_COMPONENTS_DEF * T2_Def::Audio::Sensor::CHANNELS_MAX];
};

// 3. AI 추론용 융합 텐서 구조 (최종 동기화 시점)
struct alignas(16) ST_UnifiedTensor_t {
    float data[T2_Def::AI::Tensor::MFCC_DIM_DEF]; // 전체 8채널 MFCC 융합 데이터
};


// 공유 컨텍스트 (더블 버퍼링 구조)
struct alignas(16) ST_SharedContext_t {
    ST_FeatureSlot_Vib_t vib_slots[2];
    std::atomic<uint8_t> vib_idx;

    ST_FeatureSlot_Aud_t aud_slots[2];
    std::atomic<uint8_t> aud_idx;
};


// ============================================================================
// [PART 4] 통신 및 바이너리 패킷 규격 (Byte Packing & _MAX 기반 고정 평탄화)
// ============================================================================
#pragma pack(push, 1)

struct ST_TriggerReason_t {
    uint8_t  trigger_axis;        // 0: X, 1: Y, 2: Z, 3: Audio
    char     metric_name[16];     // 예: "RMS", "BAND_ENERGY"
    float    measured_value;      // 실제 측정값
    float    threshold_value;     // 임계값
    float    excess_ratio;        // 초과율 (measured_value / threshold_value)
};

struct ST_FileHeader_t {
    char     magic[4];
    uint16_t ver;
    uint16_t struct_size;
    uint32_t s_rate;
    uint16_t fft;
    uint16_t mfcc_d;
    uint8_t  accel_mask; // [수정] 구버전 axes 삭제 후 가속도 축 마스크 배치
    uint8_t  gyro_mask;  // [수정] 자이로 축 마스크 배치
    uint8_t  audio_mask; // [수정] 오디오 채널 마스크 배치
    uint8_t  _res;
    uint32_t total;
    uint64_t trigger_t0;          // T0 절대 타임스탬프 (Monotonic)
    ST_TriggerReason_t reason;    // 트리거 상세 원인 메타데이터
    char     config_dump[8192];
};


struct ST_WsHeader_t {
    uint8_t  magic;
    uint8_t  type;
    uint16_t len;
    uint8_t  stage;
    uint8_t  source;
    uint8_t  _pad[2];
};

// JS Float32Array 파싱 최적화 Flat 구조
struct ST_PktTelemetry_t {
    ST_WsHeader_t header;           // 8 Bytes
    uint8_t  sys_state;             // 1
    uint8_t  detect_result;         // 1
    uint8_t  trial_no;              // 1
    uint8_t  trigger_source;        // 1
    uint8_t  accel_mask;            // 1
    uint8_t  gyro_mask;             // 1
    uint8_t  audio_mask;            // 1
    uint8_t  payload_type;          // 1 (여기까지 16 Bytes)

    uint64_t accel_ts;              // 8
    uint64_t gyro_ts;               // 8
    uint64_t audio_ts;              // 8
    float    temp;                  // 4
    uint8_t  _pad_header[4];        // 4 (여기까지 48 Bytes)

    // --- [제로 카피 연산 영역] 구조체 최후미 배치 및 정렬 오버라이드 ---
    SMEA_ALIGN_16 float accel_band_energy[16];   // 16 * 4 = 64 Bytes
    SMEA_ALIGN_16 float gyro_rms_energy[2];      // 2 * 4 = 8 Bytes
    uint8_t             _pad_gyro[8];            // 8 Bytes 패딩 (120 -> 128)
    SMEA_ALIGN_16 float audio_timbre_bands[32];  // 32 * 4 = 128 Bytes
    SMEA_ALIGN_16 float audio_mfcc[13];          // 13 * 4 = 52 Bytes
    uint8_t             _pad_end[12];            // 12 Bytes 패딩 (308 -> 320)
};

// 컴파일 타임 오프셋 정렬 무결성 검증 (빌드 시 덤프 방지)
static_assert(offsetof(ST_PktTelemetry_t, accel_band_energy) % 16 == 0, "Unaligned tensor offset!");
static_assert(offsetof(ST_PktTelemetry_t, gyro_rms_energy) % 16 == 0, "Unaligned tensor offset!");
static_assert(offsetof(ST_PktTelemetry_t, audio_timbre_bands) % 16 == 0, "Unaligned tensor offset!");
static_assert(offsetof(ST_PktTelemetry_t, audio_mfcc) % 16 == 0, "Unaligned tensor offset!");
static_assert(sizeof(ST_PktTelemetry_t) % 16 == 0, "Unaligned structure size!");

struct ST_PktSpectrum_t {
    ST_WsHeader_t header;
    float    frequencies[(T2_Def::Audio::Sensor::FFT_SIZE_MAX / 2) + 1];
};

// 도메인별 독립 파형 전송을 위한 패킷 세분화
struct ST_PktWaveformAudio_t {
    ST_WsHeader_t header;
    float    samples[T2_Def::Audio::Sensor::FFT_SIZE_MAX];
};

struct ST_PktWaveformAccel_t {
    ST_WsHeader_t header;
    float    samples[T2_Def::Accel::Sensor::FFT_SIZE_MAX];
};

struct ST_PktWaveformGyro_t {
    ST_WsHeader_t header;
    float    samples[T2_Def::Gyro::Sensor::FFT_SIZE_MAX];
};

struct ST_PktCalibration_t {
    ST_WsHeader_t header;
    float    target_gain_curve[(T2_Def::Audio::Sensor::FFT_SIZE_MAX / 2) + 1];
    float    actual_fir_coeffs[T2_Def::Audio::FeatureLimit::FIR_TAPS_MAX];
};

struct ST_PktSequence_t {
    ST_WsHeader_t header;
    float    data[T2_Def::Global::System::SEQUENCE_FRAMES_MAX * T2_Def::AI::Tensor::MFCC_DIM_MAX];
};

#pragma pack(pop)

}  // namespace T2_Type

