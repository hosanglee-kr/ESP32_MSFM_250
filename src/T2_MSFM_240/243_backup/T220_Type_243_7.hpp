/* ============================================================================
 * File: T220_Type_243_7.hpp
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

#include "T210_Def_243_8.hpp"


namespace T2_Type {

// ========================================================================
// [PART 1] 시스템 열거형 (Enum Classes)
// ========================================================================

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

enum class OpMode : uint8_t {
	MANUAL = 0,
	AUTO,
	SCHEDULE
};

// Audio 채널 모드 타입 안정성 보강
enum class AudioChannelMode : uint8_t {
	MONO_L = 1,
	MONO_R = 2,
	STEREO = 3
};

// 패킷 및 슬롯 페이로드 식별자
enum class DataPayloadType : uint8_t {
	VIB_ONLY	   = 1,
	AUDIO_ONLY	   = 2,
	VIB_AUDIO_BOTH = 3
};

enum class StatusBit : uint8_t {
	NTP_SYNCED	  = 0,
	SD_MOUNTED	  = 1,
	RECORDING_NOW = 2,
	SENSOR_FAULT  = 3
};

enum class DetectionResult : uint8_t {
	PASS = 0,
	RULE_VIB_NG,
	RULE_AUDIO_NG,
	TEST_NG,
	ML_NG
};

enum class SystemCommand : uint8_t {
	CMD_START = 0,
	CMD_STOP,
	CMD_LEARN_NOISE,
	CMD_CALIBRATE,
	CMD_REBOOT,
	CMD_OTA_START,
	CMD_OTA_END,
	CMD_TUNING_PREVIEW,
	CMD_TUNING_SAVE,
	CMD_TUNING_CANCEL
};

enum class TriggerSource : uint8_t {
	NONE = 0,
	HW_WAKE,
	SW_RMS,
	SW_BAND,
	MANUAL
};

enum class StreamType : uint8_t {
	TELEMETRY	= 0x01,
	SPECTRUM	= 0x02,
	WAVEFORM	= 0x03,
	CALIBRATION = 0x04
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

// ========================================================================
// [PART 2] 동적 설정 구조체 (런타임 제어 및 스위치 기능 통합)
// ========================================================================

// ------------------------------------------------------------------------
// Tier 1. Global
// ------------------------------------------------------------------------
struct ST_Global_WiFi {
	WiFiMode mode;
	char	 ap_ssid[T2_Def::Global::NetLimit::MAX_SSID_LEN_CONST];
	char	 ap_pw[T2_Def::Global::NetLimit::MAX_PW_LEN_CONST];
	char	 ap_ip[T2_Def::Global::NetLimit::MAX_IP_LEN_CONST];
	char	 multi_ssid[T2_Def::Global::NetLimit::MAX_MULTI_AP_CONST][T2_Def::Global::NetLimit::MAX_SSID_LEN_CONST];
	char	 multi_pw[T2_Def::Global::NetLimit::MAX_MULTI_AP_CONST][T2_Def::Global::NetLimit::MAX_PW_LEN_CONST];
};

struct ST_Global_Mqtt {
	bool	 enable;
	char	 broker[T2_Def::Global::NetLimit::MAX_BROKER_LEN_CONST];
	uint16_t port;
	char	 id[32];
	char	 pw[64];
	char	 topic_root[64];
	char	 lwt_topic[64];
	uint8_t	 qos;
	uint8_t	 proto_ver;
};

struct ST_Global_Storage {
	uint32_t rot_mb;
	uint32_t rot_min;
	bool	 save_raw;
	uint16_t keep_max;
	uint32_t idle_flush_ms;
	uint8_t	 pre_trig_sec;
};

struct ST_Global_System {
	char	 site_id[32];
	uint8_t	 tele_hz;
	uint8_t	 wave_hz;
	OpMode	 op_mode;
	uint32_t watchdog_ms;
	bool	 vib_enable;
	bool	 audio_enable;
};

struct ST_Global_Output {
	bool	 enabled;
	bool	 output_sequence;
	uint16_t sequence_frames;
};

// ------------------------------------------------------------------------
// Tier 2. Shared (진동/소음 공통 신호처리 논리)
// ------------------------------------------------------------------------
struct ST_FilterFIR {
	bool	 en;
	float	 cutoff;
	uint16_t taps;
};

struct ST_FilterIIR {
	bool  en;
	float cutoff;
	float q;
};

struct ST_FilterNotch {
	bool  en;
	float freq;
	float gain;
	float q;
};

struct ST_Shared_Trigger {
	uint32_t hold_ms;
	bool	 use_sleep;
	uint32_t sleep_sec;
};

struct ST_Shared_Dsp {
	bool		   rem_dc;
	bool		   med_en;
	uint8_t		   med_win;
	ST_FilterFIR   hpf;
	ST_FilterFIR   lpf;
	ST_FilterIIR   iir_hpf;
	ST_FilterIIR   iir_lpf;
	WindowType	   win_type;
	ST_FilterNotch notch;
};

// ------------------------------------------------------------------------
// Tier 3. Vib
// ------------------------------------------------------------------------
struct ST_Vib_Sensor {
	bool	 accel_enable;
	bool	 gyro_enable;
	uint8_t	 axis;
	uint8_t	 axis_count;
	uint8_t	 accel_range;
	uint8_t	 accel_odr;
	uint8_t	 gyro_range;
	uint32_t sample_rate;  // 동적 레이트
	uint32_t fft_size;	   // 동적 FFT 크기
};

struct ST_Vib_Trigger {
	bool	 motion_en;
	float	 wake_g;
	uint16_t wake_dur;
	float	 rms_thresh;

	uint8_t	 active_band_count;	 // 런타임 실제 사용 밴드 수
	bool	 band_en[T2_Def::Shared::FeatureLimit::BAND_RMS_MAX];
	float	 band_start[T2_Def::Shared::FeatureLimit::BAND_RMS_MAX];
	float	 band_end[T2_Def::Shared::FeatureLimit::BAND_RMS_MAX];
	float	 band_thresh[T2_Def::Shared::FeatureLimit::BAND_RMS_MAX];
};

struct ST_Vib_Calib {
	float offset[3];
	float gain[3];
};

struct ST_Vib_Feature {
	float peak_amp_min;
	float peak_freq_gap_min;
};

// ------------------------------------------------------------------------
// Tier 4. Audio
// ------------------------------------------------------------------------
// [신규] Audio 센서 동적 설정용 구조체 신설
struct ST_Audio_Sensor {
	AudioChannelMode channel_mode;
	uint32_t		 sample_rate;
	uint32_t		 fft_size;
};

struct ST_Audio_Trigger {
	float	rms_thresh;
	float	crest_ng_thresh;
	float	skew_ng_thresh;

	uint8_t active_band_count;
	bool	band_en[T2_Def::Shared::FeatureLimit::BAND_RMS_MAX];
	float	band_start[T2_Def::Shared::FeatureLimit::BAND_RMS_MAX];
	float	band_end[T2_Def::Shared::FeatureLimit::BAND_RMS_MAX];
	float	band_thresh[T2_Def::Shared::FeatureLimit::BAND_RMS_MAX];
};

struct ST_Audio_Noise {
	bool	  gate_en;
	float	  gate_thresh;
	NoiseMode mode;
	float	  sub_str;
	float	  adp_alpha;
	uint16_t  learn_frames;
};

struct ST_Audio_Dsp {
	bool		   pre_en;
	float		   pre_alpha;
	ST_Audio_Noise noise;
	float		   beam_gain;
	float		   window_ms;
	float		   hop_ms;
};

struct ST_Audio_Calib {
	uint32_t auto_idle_min;
	float	 ref_freq;
	float	 filt_min;
	float	 filt_max;
	float	 gain_max;
	float	 gain_min;
	float	 norm_safe;
	float	 gain_L;
	float	 gain_R;
	alignas(16) float eq_coeffs[T2_Def::Shared::FeatureLimit::FIR_TAPS_MAX];
};

struct ST_Audio_Feature {
	uint8_t active_ceps_count;
	uint8_t active_peak_count;
	float	ceps_targets[T2_Def::Shared::FeatureLimit::CEPS_TARGET_MAX];
	float	f_min;
	float	f_max;
	float	peak_amp_min;
	float	peak_freq_gap_min;
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
	float freq;
	float amp;
};

// [Block 1] 공통 헤더 (32 Bytes)
struct ST_Slot_Header {
	uint64_t ts;
	uint32_t fid;
	uint32_t uptime;
	uint8_t	 flags;
	int8_t	 temp;
	uint8_t	 trial;
	uint8_t	 src;
	uint8_t	 active_axes;
	uint8_t	 payload_type;	// DataPayloadType 캐스팅용
	uint8_t	 _res[10];		// 10바이트 패딩으로 총 32 Bytes 완벽 유지
};

// [Block 2] 진동 특화 지표 (144 Bytes)
struct ST_Slot_Vib {
	float rms[3];
	float peak_f[3];
	float centroid[3];
	float kurt;
	float crest;
	float sta_lta;
	float skew;
	float std;
	float cal_off[3];												// (17 * 4) = 68B
	float band_energy[T2_Def::Shared::FeatureLimit::BAND_RMS_MAX];	// 16 * 4 = 64B
	float _pad[3];													// (68+64) = 132B -> 16배수 맞춤용 12B 패딩 -> 총 144B
};

// [Block 3] 소음 특화 지표 (240 Bytes)
struct ST_Slot_Audio {
	float rms;
	float energy;
	float centroid;
	float kurt;
	float
		crest;
	float
				 sta_lta;
	float		 skew;
	float		 coh;
	float		 ipd;
	float		 d_rms;
	float		 dd_rms;												   // 11*4 = 44B
	float		 cpsr_max[T2_Def::Shared::FeatureLimit::CEPS_TARGET_MAX];  // 5*4 = 20B
	float		 cpsr_mxr[T2_Def::Shared::FeatureLimit::CEPS_TARGET_MAX];  // 5*4 = 20B
	float		 band_energy[T2_Def::Shared::FeatureLimit::BAND_RMS_MAX];  // 16*4 = 64B
	SpectralPeak top_peaks[T2_Def::Shared::FeatureLimit::TOP_PEAKS_MAX];   // 10*8 = 80B
	float		 _pad[3];												   // 44+20+20+64+80 = 228B -> 16배수 맞춤용 12B 패딩 -> 총 240B
};

// [Block 4] AI 텐서 (384 Bytes)
struct ST_Slot_Tensor {
	alignas(16) float mfcc[T2_Def::Audio::FeatureLimit::MFCC_DIM_MAX];	// 96 * 4 = 384B.
};

// 전체 조립: 32 + 144 + 240 + 384 = 총 800 Bytes (SIMD 16B 완벽 호환 보장)
struct alignas(16) UnifiedFeatureSlot {
	ST_Slot_Header header;
	ST_Slot_Vib	   vib;
	ST_Slot_Audio  audio;
	ST_Slot_Tensor tensor;
};

// 스토리지 기록용 하이브리드 Raw 데이터
struct alignas(16) UnifiedRawChunk {
	float vib[3][T2_Def::Vib::Sensor::FFT_SIZE_MAX];
	float audio[T2_Def::Audio::Sensor::FFT_SIZE_MAX];
};

// ============================================================================
// [PART 4] 통신 및 바이너리 패킷 규격 (Byte Packing & _MAX 기반 고정 평탄화)
// ============================================================================
#pragma pack(push, 1)

// 파일 로깅용 매직 헤더
struct FileHeader {
	char	 magic[4];
	uint16_t ver;
	uint16_t struct_size;
	uint32_t s_rate;
	uint16_t fft;
	uint16_t mfcc_d;
	uint8_t	 axes;
	uint8_t	 _res;
	uint32_t total;
	char	 config_dump[8192];
};

struct WsHeader {
	uint8_t	 magic;
	uint8_t	 type;
	uint16_t len;
	uint8_t	 stage;
	uint8_t	 _pad[3];
};

// [정렬 방어 기술]: WsHeader(8B) + payload_type까지 6B + _pad[2](2B) = 16B 선행 확보.
struct PktTelemetry {
	WsHeader header;
	uint8_t	 sys_state;
	uint8_t	 detect_result;
	uint8_t	 trial_no;
	uint8_t	 trigger_source;
	uint8_t	 active_axes;
	uint8_t	 payload_type;
	uint8_t	 _pad[2];  // 16 Bytes 오프셋 달성 (JS Float32Array 파싱 크래시 완벽 차단)

	// Vib Flattening
	float	 vib_rms[3];
	float	 vib_centroid[3];
	float	 vib_kurtosis;
	float	 vib_crest_factor;
	float	 vib_sta_lta_ratio;
	float	 vib_band_energy[T2_Def::Shared::FeatureLimit::BAND_RMS_MAX];

	// Audio Flattening
	float	 audio_rms;
	float	 audio_energy;
	float	 audio_kurtosis;
	float	 audio_crest_factor;
	float	 audio_sta_lta_ratio;
	float	 audio_skewness;
	float	 audio_spectral_centroid;

	float	 audio_band_energy[T2_Def::Shared::FeatureLimit::BAND_RMS_MAX];

	float	 audio_peak_freqs[T2_Def::Shared::FeatureLimit::TOP_PEAKS_MAX];
	float	 audio_peak_amps[T2_Def::Shared::FeatureLimit::TOP_PEAKS_MAX];
	float	 audio_cpsr_max[T2_Def::Shared::FeatureLimit::CEPS_TARGET_MAX];
	float	 audio_cpsr_mxrms[T2_Def::Shared::FeatureLimit::CEPS_TARGET_MAX];

	// AI Tensor
	float	 mfcc[T2_Def::Audio::FeatureLimit::MFCC_DIM_MAX];
};

struct PktSpectrum {
	WsHeader header;
	float	 frequencies[(T2_Def::Audio::Sensor::FFT_SIZE_MAX / 2) + 1];
};

struct PktWaveform {
	WsHeader header;
	float	 samples[T2_Def::Audio::Sensor::FFT_SIZE_MAX];
};

struct PktCalibration {
	WsHeader header;
	float	 target_gain_curve[(T2_Def::Audio::Sensor::FFT_SIZE_MAX / 2) + 1];
	float	 actual_fir_coeffs[T2_Def::Shared::FeatureLimit::FIR_TAPS_MAX];
};

#pragma pack(pop)
}  // namespace T2_Type
