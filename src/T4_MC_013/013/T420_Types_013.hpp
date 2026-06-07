/* ============================================================================
 * [SMEA-100 핵심 구현 원칙 및 AI 셀프 회고 바이블]
 * 1. 매직넘버 철폐: 모든 하드웨어 및 버퍼 크기는 SmeaConfig 의존.
 * 2. 16-Byte 정렬(SIMD 최적화): esp-dsp 가속기의 128-bit 처리 특성을 극대화하기 위해
 * 모든 연산 버퍼 및 필터 계수는 반드시 alignas(16)으로 메모리를 정렬한다.
 * 3. [명시적 패딩 방어]: 컴파일러 암묵적 패딩으로 인한 SD카드 바이너리 로깅
 * 정합성 파괴를 막기 위해 구조체 끝에 _reserved 바이트를 강제 할당한다.
 * 4. [네트워크 대역폭 방어]: 6Mbps에 달하는 무거운 데이터를 고속(100Hz) 일괄
 * 전송 시 발생하는 OOM을 막기 위해 WsHeader 기반 다중화(Multiplexing) 규격을
 * 도입하고, packed 속성으로 브라우저의 Float32Array 파싱 완벽 일치를 보장한다.
 * 5. [v013 추가]: Web UI/UX 오버레이 차트를 위한 패킷 식별자(stage) 및 
 * 캘리브레이션 전용 패킷(PktCalibration) 규격 신설.
 * ============================================================================
 * File: T420_Types_013.hpp
 * Summary: Core Data Structures, Feature Slots & Multiplexed Web Packets
 * ========================================================================== */
#pragma once

#include "T410_Def_013.hpp"
#include <cstdint>

namespace SmeaType {

    // [v013 신설] SD 카드 바이너리 포맷 파편화 방어용 매직 헤더
    struct FileHeader {
        char     magic[4];   // "SMEA"
        uint16_t version;    // 13 (v013)
        uint16_t struct_size;// sizeof(FeatureSlot)
        uint64_t _reserved;  // 16바이트 정렬 맞춤용 패딩
    };

    struct SpectralPeak {
        float frequency;
        float amplitude;
    };

    struct alignas(16) RawDataSlot {
        alignas(16) float raw_L[SmeaConfig::System::FFT_SIZE_CONST];
        alignas(16) float raw_R[SmeaConfig::System::FFT_SIZE_CONST];
    };

    struct alignas(16) FeatureSlot {
        alignas(16) float mfcc[SmeaConfig::System::MFCC_TOTAL_DIM_CONST];

        float             band_rms[SmeaConfig::FeatureLimit::MAX_BAND_RMS_COUNT_CONST];
        float             cpsr_max[SmeaConfig::FeatureLimit::CEPS_TARGET_COUNT_CONST];
        float             cpsr_mxrms[SmeaConfig::FeatureLimit::CEPS_TARGET_COUNT_CONST];

        float             rms;
        float             energy;
        float             kurtosis;
        float             crest_factor;
        float             pooling_stddev_min;
        float             spectral_centroid;
        float             sta_lta_ratio;

        float             phase_coherence;
        float             mean_ipd;
        float             delta_rms;
        float             delta_delta_rms;

		SpectralPeak      top_peaks[SmeaConfig::FeatureLimit::TOP_PEAKS_COUNT_CONST];

        // 64비트 Epoch Time 대응 구조체 유지
        uint64_t          timestamp;
        uint8_t           trial_no;

        uint8_t           _reserved[7];
    };

    #pragma pack(push, 1)

    enum class StreamType : uint8_t {
        TELEMETRY   = 0x01,
        SPECTRUM    = 0x02,
        WAVEFORM    = 0x03,
        CALIBRATION = 0x04  // [v013 추가] 캘리브레이션 전용
    };

    struct WsHeader {
        uint8_t  magic;    
        uint8_t  type;     
        uint16_t length;   
        uint8_t  stage;    // [v013 추가] 다중 채널/처리 단계 식별용 (0:RawL, 1:RawR, 2:PostDSP 등)
        uint8_t  _pad[3];  // 8바이트 정렬 맞춤
    };

    struct PktTelemetry {
        WsHeader header;

        uint8_t  sys_state;
        uint8_t  trial_no;
        uint8_t  _pad[2]; 

        float    rms;
        float    sta_lta_ratio;
        float    kurtosis;
        float    spectral_centroid;

        float    band_rms[SmeaConfig::FeatureLimit::MAX_BAND_RMS_COUNT_CONST];

		float    peak_freqs[SmeaConfig::FeatureLimit::TOP_PEAKS_COUNT_CONST];
		float    peak_amps[SmeaConfig::FeatureLimit::TOP_PEAKS_COUNT_CONST];

        float    mfcc[SmeaConfig::System::MFCC_TOTAL_DIM_CONST];
    };

    struct PktSpectrum {
        WsHeader header;
        float    bins[(SmeaConfig::System::FFT_SIZE_CONST / 2) + 1];
    };

    struct PktWaveform {
        WsHeader header;
        float    samples[SmeaConfig::System::FFT_SIZE_CONST];
    };
    
    // [v013 신설] 캘리브레이션 타겟 게인과 실제 계수 곡선을 웹에 뿌려주기 위함
    struct PktCalibration {
        WsHeader header;
        float    target_gain_curve[(SmeaConfig::System::FFT_SIZE_CONST / 2) + 1];
        float    actual_fir_coeffs[SmeaConfig::CalibLimit::EQ_FIR_TAPS_CONST];
    };

    #pragma pack(pop)
}
