/* ============================================================================
 * File: T220_Type_243_6.hpp
 * Summary: T240 4-Tier 데이터 타입 및 패킷 (Vib-Audio 시너지 통합본)
 * ============================================================================
 * [시스템 구현 원칙 - Synergy & Safety Principles]
 * 1. [도메인 캡슐화]: Vib와 Audio가 각각 고유의 Band와 통계 지표를 품도록 캡슐화.
 * 2. [SIMD 가속 최적화]: 연산 슬롯 재조립. Header(32)+Vib(112)+Audio(144)+Tensor(160).
 * -> 총 448 Bytes. 16 Bytes 배수(alignas) 완벽 보장 (OOM 및 정렬 크래시 원천 차단).
 * 3. [웹 패킷 평탄화]: PktTelemetry는 AoS(구조체배열)를 엄격히 금지하고 SoA로 평탄화.
 * 4. [MLOps 추적성]: 8KB Config Dump, 런타임 FSM 컨텍스트, 축 정보(active_axes) 100% 보존.
 * ========================================================================== */

#pragma once

#include <cstdint>

#include "T210_Def_243_6.hpp"

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
// [PART 2] 동적 설정 구조체 (익명구조체 전면 금지 및 시너지 파라미터 적용)
// ========================================================================

// ------------------------------------------------------------------------
// Tier 1. Global (디바이스 전역 인프라)
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
// Tier 3. Vib (진동 특화 - Audio 다중 대역 이식)
// ------------------------------------------------------------------------
struct ST_Vib_Sensor {
	uint8_t axis;
	uint8_t axis_count;
	uint8_t accel_range;
	uint8_t accel_odr;
	uint8_t gyro_range;
};

struct ST_Vib_Trigger {
	bool	 motion_en;
	float	 wake_g;
	uint16_t wake_dur;
	float	 rms_thresh;
	// [시너지] 진동 다중 밴드 추적
	bool	 band_en[T2_Def::Vib::Trigger::MAX_BAND_RMS_CONST];
	float	 band_start[T2_Def::Vib::Trigger::MAX_BAND_RMS_CONST];
	float	 band_end[T2_Def::Vib::Trigger::MAX_BAND_RMS_CONST];
	float	 band_thresh[T2_Def::Vib::Trigger::MAX_BAND_RMS_CONST];
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
// Tier 4. Audio (소음 특화 - Vib 물리 통계 이식)
// ------------------------------------------------------------------------
struct ST_Audio_Trigger {
	float rms_thresh;
	// [시너지] 소음의 충격/비대칭 룰베이스 한계치
	float crest_ng_thresh;
	float skew_ng_thresh;

	bool  band_en[T2_Def::Audio::Trigger::MAX_BAND_RMS_CONST];
	float band_start[T2_Def::Audio::Trigger::MAX_BAND_RMS_CONST];
	float band_end[T2_Def::Audio::Trigger::MAX_BAND_RMS_CONST];
	float band_thresh[T2_Def::Audio::Trigger::MAX_BAND_RMS_CONST];
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
	alignas(16) float eq_coeffs[T2_Def::Shared::Dsp::FIR_TAPS_CONST];
};

struct ST_Audio_Feature {
	float ceps_targets[T2_Def::Shared::FeatureLimit::CEPS_TARGET_COUNT_CONST];
	float f_min;
	float f_max;
	float peak_amp_min;
	float peak_freq_gap_min;
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
// [PART 3] 고장 진단 데이터 모델 (도메인 캡슐화 & SIMD 정밀 패딩)
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
	uint8_t	 _res[11];
};

// [Block 2] 진동 특화 지표 (112 Bytes) - [시너지] 밴드 에너지 병합
struct ST_Slot_Vib {
	float rms[3];
	float peak_f[3];
	float centroid[3];
	float kurt;
	float crest;
	float sta_lta;
	float skew;
	float std;
	float cal_off[3];											  // 여기까지 17 floats (68B)
	float band_energy[T2_Def::Vib::Trigger::MAX_BAND_RMS_CONST];  // 32B
	float _pad[3];												  // (68+32) = 100B. 16의 배수 맞춤용 12B 패딩 -> 총 112B 완벽 정렬.
};

// [Block 3] 소음 특화 지표 (144 Bytes) - [시너지] 물리 통계 및 밴드 병합
struct ST_Slot_Audio {
	float		 rms;
	float		 energy;
	float		 centroid;
	float		 kurt;
	float		 crest;
	float		 sta_lta;
	float		 skew;	// [시너지] 진동 지표 추가
	float		 coh;
	float		 ipd;
	float		 d_rms;
	float		 dd_rms;  // 여기까지 11 floats (44B)

	float		 cpsr_max[T2_Def::Shared::FeatureLimit::CEPS_TARGET_COUNT_CONST];  // 12B
	float		 cpsr_mxr[T2_Def::Shared::FeatureLimit::CEPS_TARGET_COUNT_CONST];  // 12B

	float		 band_energy[T2_Def::Audio::Trigger::MAX_BAND_RMS_CONST];		  // 32B
	SpectralPeak top_peaks[T2_Def::Shared::FeatureLimit::TOP_PEAKS_COUNT_CONST];  // 40B

	float		 _pad;	// 44+12+12+32+40 = 140B. 16의 배수 맞춤용 4B 패딩 -> 총 144B 완벽 정렬.
};

// [Block 4] AI 텐서 (160 Bytes)
struct ST_Slot_Tensor {
	alignas(16) float mfcc[T2_Def::Audio::FeatureLimit::MFCC_DIM_CONST];  // 156B
	float _pad;															  // 160B
};

// 전체 조립: 32 + 112 + 144 + 160 = 총 448 Bytes (SIMD 16B 완벽 호환 보장)
struct alignas(16) UnifiedFeatureSlot {
	ST_Slot_Header header;
	ST_Slot_Vib	   vib;
	ST_Slot_Audio  audio;
	ST_Slot_Tensor tensor;
};

// 스토리지 기록용 하이브리드 Raw 데이터
struct alignas(16) UnifiedRawChunk {
	float vib[3][T2_Def::Vib::Sensor::FFT_SIZE_CONST];
	float audio[T2_Def::Audio::Sensor::FFT_SIZE_CONST];
};

// ============================================================================
// [PART 4] 통신 및 바이너리 패킷 규격 (Byte Packing & SoA 평탄화)
// ============================================================================
#pragma pack(push, 1)

// 파일 로깅용 매직 헤더 (MLOps 추적성 보장)
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

struct PktTelemetry {
	WsHeader header;
	uint8_t	 sys_state;
	uint8_t	 detect_result;
	uint8_t	 trial_no;
	uint8_t	 trigger_source;
	uint8_t	 active_axes;
	uint8_t	 _pad[3];

	// Vib Flattening
	float	 vib_rms[3];
	float	 vib_centroid[3];
	float	 vib_kurtosis;
	float	 vib_crest_factor;
	float	 vib_sta_lta_ratio;
	float	 vib_band_energy[T2_Def::Vib::Trigger::MAX_BAND_RMS_CONST];

	// Audio Flattening
	float	 audio_rms;
	float	 audio_energy;
	float	 audio_kurtosis;
	float	 audio_crest_factor;
	float	 audio_sta_lta_ratio;
	float	 audio_skewness;
	float	 audio_spectral_centroid;

	float	 audio_band_energy[T2_Def::Audio::Trigger::MAX_BAND_RMS_CONST];

	float	 audio_peak_freqs[T2_Def::Shared::FeatureLimit::TOP_PEAKS_COUNT_CONST];
	float	 audio_peak_amps[T2_Def::Shared::FeatureLimit::TOP_PEAKS_COUNT_CONST];
	float	 audio_cpsr_max[T2_Def::Shared::FeatureLimit::CEPS_TARGET_COUNT_CONST];
	float	 audio_cpsr_mxrms[T2_Def::Shared::FeatureLimit::CEPS_TARGET_COUNT_CONST];

	float	 mfcc[T2_Def::Audio::FeatureLimit::MFCC_DIM_CONST];  // AI Tensor
};

struct PktSpectrum {
	WsHeader header;
	float	 frequencies[513];
};

struct PktWaveform {
	WsHeader header;
	float	 samples[1024];
};

struct PktCalibration {
	WsHeader header;
	float	 target_gain_curve[513];
	float	 actual_fir_coeffs[63];
};

#pragma pack(pop)
}  // namespace T2_Type
