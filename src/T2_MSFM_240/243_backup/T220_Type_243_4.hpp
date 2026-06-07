/* ============================================================================
 * File: T220_Type_243_4.hpp
 * Summary: T240 4-Tier 데이터 타입, 설정 구조체, 패킷 포맷 통합 정의
 * ============================================================================
 * [시스템 구현 원칙 - Core Principles]
 * 1. [도메인 분리]: Config 구조체를 Global, Shared, Vib, Audio 4-Tier로 계층화.
 * 2. [익명구조체 금지]: 모든 구조체는 명시적 타입명(ST_...)을 가져 유지보수성 극대화.
 * 3. [SIMD 가속 최적화]: 연산용 슬롯(FeatureSlot)은 내부 청크별 alignas(16) 강제 적용.
 * - Header(32B)+Vib(80B)+Audio(64B)+Bands(80B)+Tensor(160B) = 총 416B 완벽 정렬.
 * 4. [네트워크 대역폭 방어]: PktTelemetry는 JS Float32Array 파싱 완벽 일치를 위해
 * 구조체 배열(AoS)을 피하고 단일 타입 배열(SoA)로 평탄화(Flattening)하여 전송.
 * 5. [MLOps 추적성]: FileHeader에 설정 덤프(config_dump) 삽입으로 데이터 맥락 보존.
 * ========================================================================== */

#pragma once

#include "T210_Def_243_4.hpp" // (또는 상수를 분리한 최신 Def 헤더 파일명 참조)
#include <cstdint>

namespace T2_Type {

    // ========================================================================
    // [PART 1] 시스템 열거형 (Enum Classes)
    // ========================================================================

    enum class SystemState : uint8_t {
        INIT = 0, READY, MONITORING, RECORDING, NOISE_LEARNING, MAINTENANCE, ERROR, CALIBRATING
    };

    enum class StatusBit : uint8_t {
        NTP_SYNCED = 0, SD_MOUNTED = 1, RECORDING_NOW = 2, SENSOR_FAULT = 3 
    };

    enum class DetectionResult : uint8_t {
        PASS = 0, RULE_VIB_NG, RULE_AUDIO_NG, TEST_NG, ML_NG 
    };

    enum class SystemCommand : uint8_t {
        CMD_START = 0, CMD_STOP, CMD_LEARN_NOISE, CMD_CALIBRATE, CMD_REBOOT, 
        CMD_OTA_START, CMD_OTA_END, CMD_TUNING_PREVIEW, CMD_TUNING_SAVE, CMD_TUNING_CANCEL 
    };

    enum class TriggerSource : uint8_t { NONE = 0, HW_WAKE, SW_RMS, SW_BAND, MANUAL };
    enum class StreamType : uint8_t { TELEMETRY = 0x01, SPECTRUM = 0x02, WAVEFORM = 0x03, CALIBRATION = 0x04 };
    enum class NoiseMode : uint8_t { OFF = 0, FIXED, ADAPTIVE };
    enum class WiFiMode : uint8_t { STA_ONLY = 0, AP_ONLY, AP_STA, AUTO_FALLBACK };
    enum class WindowType : uint8_t { HANN = 0, HAMMING, BLACKMAN };

    // ========================================================================
    // [PART 2] 동적 설정 구조체 (4-Tier 도메인 완벽 분리 및 익명구조체 금지)
    // ========================================================================

    // ------------------------------------------------------------------------
    // Tier 1. Global (디바이스 전역 인프라)
    // ------------------------------------------------------------------------
    struct ST_Global_WiFi { 
        WiFiMode mode; 
        char ap_ssid[T2_Def::Global::NetLimit::MAX_SSID_LEN_CONST]; 
        char ap_pw[T2_Def::Global::NetLimit::MAX_PW_LEN_CONST]; 
        char ap_ip[T2_Def::Global::NetLimit::MAX_IP_LEN_CONST]; 
        char multi_ssid[T2_Def::Global::NetLimit::MAX_MULTI_AP_CONST][T2_Def::Global::NetLimit::MAX_SSID_LEN_CONST]; 
        char multi_pw[T2_Def::Global::NetLimit::MAX_MULTI_AP_CONST][T2_Def::Global::NetLimit::MAX_PW_LEN_CONST]; 
    };

    struct ST_Global_Mqtt { 
        bool enable; 
        char broker[T2_Def::Global::NetLimit::MAX_BROKER_LEN_CONST]; 
        uint16_t port; 
        char id[32]; 
        char pw[64]; 
        char topic_root[64]; 
        uint8_t qos; 
        uint8_t proto_ver; // [복원] MQTT 프로토콜 버전 (3.1.1 vs 5.0)
    };

    struct ST_Global_Storage { 
        uint32_t rot_mb; 
        uint32_t rot_min; 
        bool save_raw; 
        uint16_t keep_max; 
        uint32_t idle_flush_ms; 
        uint8_t pre_trig_sec; 
    };

    struct ST_Global_System { 
        char site_id[32]; 
        uint8_t tele_hz; 
        uint8_t wave_hz; 
        uint8_t op_mode; 
        uint32_t watchdog_ms; 
    };

    struct ST_Global_Output { 
        bool enabled; 
        bool output_sequence; 
        uint16_t sequence_frames; 
    };

    // ------------------------------------------------------------------------
    // Tier 2. Shared (진동/소음 공통 신호처리 논리)
    // ------------------------------------------------------------------------
    struct ST_FilterFIR { bool en; float cutoff; uint16_t taps; };
    struct ST_FilterIIR { bool en; float cutoff; float q; };
    struct ST_FilterNotch { bool en; float freq; float gain; float q; };
    struct ST_Shared_Trigger { uint32_t hold_ms; bool use_sleep; uint32_t sleep_sec; };
    
    struct ST_Shared_Dsp { 
        bool rem_dc; 
        bool med_en; 
        uint8_t med_win; 
        ST_FilterFIR hpf; 
        ST_FilterFIR lpf; 
        ST_FilterIIR iir_hpf; 
        ST_FilterIIR iir_lpf; 
        WindowType win_type; 
        ST_FilterNotch notch; 
    };

    // ------------------------------------------------------------------------
    // Tier 3. Vib (진동 특화 파이프라인)
    // ------------------------------------------------------------------------
    struct ST_Vib_Sensor { uint8_t accel_range; uint8_t accel_odr; uint8_t gyro_range; };
    struct ST_Vib_Trigger { bool motion_en; float wake_g; uint16_t wake_dur; float rms_thresh; };
    struct ST_Vib_Calib { float offset[3]; float gain[3]; };
    struct ST_Vib_Feature { float peak_amp_min; float peak_freq_gap_min; };

    // ------------------------------------------------------------------------
    // Tier 4. Audio (소음 특화 파이프라인)
    // ------------------------------------------------------------------------
    struct ST_Audio_Trigger { 
        float rms_thresh; 
        bool band_en[8]; 
        float band_start[8]; 
        float band_end[8]; 
        float band_thresh[8]; 
    };

    struct ST_Audio_Noise { 
        bool gate_en; 
        float gate_thresh; 
        NoiseMode mode; 
        float sub_str; 
        float adp_alpha; 
        uint16_t learn_frames; 
    };

    struct ST_Audio_Dsp { 
        bool pre_en; 
        float pre_alpha; 
        ST_Audio_Noise noise; 
        float beam_gain; 
        float window_ms; // [복원] 오버랩 튜닝 제어용
        float hop_ms;    // [복원]
    };

    struct ST_Audio_Calib { 
        uint32_t auto_idle_min; 
        float ref_freq; 
        float filt_min; 
        float filt_max; 
        float gain_max; 
        float gain_min; 
        float norm_safe; 
        float gain_L; 
        float gain_R; 
        alignas(16) float eq_coeffs[T2_Def::Shared::Dsp::FIR_TAPS_CONST]; 
    };

    struct ST_Audio_Feature { 
        float ceps_targets[3]; 
        float f_min; 
        float f_max; 
        float peak_amp_min;      // [복원] 동적 피크 튜닝용
        float peak_freq_gap_min; // [복원]
    };

    // ------------------------------------------------------------------------
    // 마스터 동적 설정 구조체 (SSOT)
    // ------------------------------------------------------------------------
    struct DynamicConfig {
        ST_Global_System  system;
        ST_Global_WiFi    wifi;
        ST_Global_Mqtt    mqtt;
        ST_Global_Storage storage;
        ST_Global_Output  output;
        ST_Shared_Trigger shared_trig;
        ST_Shared_Dsp     shared_dsp;
        ST_Vib_Sensor     vib_sensor;
        ST_Vib_Trigger    vib_trig;
        ST_Vib_Calib      vib_calib;
        ST_Vib_Feature    vib_feat;
        ST_Audio_Trigger  aud_trig;
        ST_Audio_Dsp      aud_dsp;
        ST_Audio_Calib    aud_calib;
        ST_Audio_Feature  aud_feat;
    };

    // ============================================================================
    // [PART 3] 고장 진단 데이터 모델 (Memory Chunk 정밀 정렬)
    // ============================================================================
    
    struct SpectralPeak { float freq; float amp; };

    // [Block 1] 공통 헤더 (32 Bytes)
    struct ST_Slot_Header {
        uint64_t ts; uint32_t fid; uint32_t uptime;
        uint8_t flags; int8_t temp; uint8_t trial; uint8_t src; uint8_t _res[12];
    };
    
    // [Block 2] 진동 특화 지표 (80 Bytes) - 정밀 12Bytes 패딩 적용 완료
    struct ST_Slot_Vib {
        float rms[3]; float peak_f[3]; float centroid[3]; float kurt; float crest; float sta_lta; float skew; float std; 
        float cal_off[3]; float _pad[3]; // (17 * 4) + (3 * 4) = 80B
    };
    
    // [Block 3] 소음 특화 지표 (64 Bytes)
    struct ST_Slot_Audio {
        float rms; float energy; float centroid; float kurt; float coh; float ipd; float d_rms; float dd_rms; 
        float cpsr_max[3]; float cpsr_mxr[3]; float _pad[2]; // (14 * 4) + (2 * 4) = 64B
    };
    
    // [Block 4] 소음 대역 및 피크 (80 Bytes)
    struct ST_Slot_Bands { 
        float energy[8]; 
        SpectralPeak peaks[5]; 
        float _pad[2]; // (8*4) + (5*8) + (2*4) = 32 + 40 + 8 = 80B
    };
    
    // [Block 5] AI 텐서 (160 Bytes)
    struct ST_Slot_Tensor { 
        alignas(16) float mfcc[39]; 
        float _pad; // 156 + 4 = 160B
    };

    // 전체 조립: 32 + 80 + 64 + 80 + 160 = 총 416 Bytes (SIMD 16B 완벽 호환)
    struct alignas(16) UnifiedFeatureSlot {
        ST_Slot_Header header; 
        ST_Slot_Vib    vib; 
        ST_Slot_Audio  audio; 
        ST_Slot_Bands  bands; 
        ST_Slot_Tensor tensor;
    };

    // 스토리지 기록용 하이브리드 Raw 데이터
    struct alignas(16) UnifiedRawChunk { 
        float vib[3][256]; 
        float audio[1024]; 
    };

    // ============================================================================
    // [PART 4] 통신 및 바이너리 패킷 규격 (Byte Packing & 평탄화)
    // ============================================================================
    #pragma pack(push, 1)

    // 파일 로깅용 매직 헤더 (MLOps 추적성)
    struct FileHeader {
        char     magic[4]; 
        uint16_t ver; 
        uint16_t struct_size;
        uint32_t s_rate; 
        uint16_t fft; 
        uint16_t mfcc_d; 
        uint8_t  axes; 
        uint8_t  _res;
        uint32_t total; 
        char     config_dump[8192]; 
    };

    struct WsHeader { 
        uint8_t magic; 
        uint8_t type; 
        uint16_t len; 
        uint8_t stage; 
        uint8_t _pad[3]; 
    };

    // [중요 교정]: JS Float32Array 파싱 완벽 일치를 위해 AoS 배제 및 SoA 평탄화 적용
    struct PktTelemetry {
        WsHeader header; 
        uint8_t  sys_state; 
        uint8_t  detect_result; 
        uint8_t  trial_no;
        uint8_t  trigger_source;
        
        // Vib Flattening
        float    vib_rms[3];
        float    vib_centroid[3];
        float    vib_kurtosis;
        float    vib_crest_factor;
        float    vib_sta_lta_ratio;

        // Audio Flattening
        float    audio_rms;
        float    audio_energy;
        float    audio_kurtosis;
        float    audio_spectral_centroid;
        
        float    band_energy[8]; 
        
        float    peak_freqs[5];  
        float    peak_amps[5];   
        float    cpsr_max[3];    
        float    cpsr_mxrms[3];  

        // AI Tensor
        float    mfcc[39];       
    };

    struct PktSpectrum {
        WsHeader header;
        float    frequencies[513];  
    };

    struct PktWaveform {
        WsHeader header;
        float    samples[1024];
    };

    struct PktCalibration {
        WsHeader header;
        float    target_gain_curve[513];
        float    actual_fir_coeffs[63];
    };

    #pragma pack(pop)
}
