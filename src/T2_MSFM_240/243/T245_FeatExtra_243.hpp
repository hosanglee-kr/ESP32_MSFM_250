/* ============================================================================
 * File: T245_FeatExtra_243.hpp
 * Summary: 멀티모달(진동/소음) 융합 특징량 추출 엔진 (v243 고도화)
 * ============================================================================
 * [로직 상태 추적 (Logic State Tracker)]
 * - 유지: PSRAM 기반 거대 행렬(MFCC Mel/DCT) 연산 및 39D 특징량 추출.
 * - 갱신: v243 UnifiedFeatureSlot(800 Bytes) 구조체에 데이터 평탄화 저장.
 * - 신규: [정밀도] MATH_EPSILON_12_CONST를 이용한 NaN 방어 및 NMS 기반 피크 정교화.
 *
 * [축소/누락 방어 체크리스트 (Omission Defense)]
 * 1. [진단 무결성]: 베어링 결함 진단 핵심인 Kurtosis(첨도) 연산 100% 구현.
 * 2. [SIMD 가속]: esp-dsp 및 dspm(Matrix) 라이브러리 연동 시 16바이트 정렬 보장.
 * 3. [메모리 관리]: 거대 FFT/MFCC 버퍼는 PSRAM 할당, 고속 진동 버퍼는 SRAM 유지.
 * 4. [SSOT]: 모든 연산 루프는 설정된 _DEF 상수 및 동적 p_len을 따름.
 * ========================================================================== */
#pragma once

#include "T210_Def_243_9.hpp"
#include "T215_Type_243_8.hpp"
#include "esp_dsp.h"
#include <cstdint>

class CL_T2_FeatureExtractor {
private:
    // 행렬 연산용 패딩 상수 (SIMD 최적화)
    static constexpr uint16_t MEL_BANDS_MAX = T2_Def::Audio::FeatureLimit::MEL_BANDS_MAX;
    static constexpr uint16_t MFCC_DIM_MAX  = T2_Def::Audio::FeatureLimit::MFCC_DIM_MAX;
    static constexpr uint16_t BINS_MAX      = (T2_Def::Audio::Sensor::FFT_SIZE_MAX / 2) + 1;

    static constexpr uint16_t MEL_PADDED    = (MEL_BANDS_MAX + 3) & ~3;
    static constexpr uint16_t BINS_PADDED   = (BINS_MAX + 3) & ~3;
    static constexpr uint16_t MFCC_PADDED   = (MFCC_DIM_MAX + 3) & ~3;

    // --- Audio용 동적 할당 버퍼 (PSRAM) ---
    float* _melBankFlat;    // Mel Filterbank Matrix (BINS x MEL)
    float* _dctMatrixFlat;  // DCT Matrix (MEL x MFCC)
    float* _fftWorkAudio;   // FFT Complex Work Buffer (SIZE x 2)
    float* _powerAudio;     // Power Spectrum Buffer
    float* _windowAudio;    // Windowing Buffer
    float* _noiseProfile;   // Learned Background Noise Profile
    uint32_t _learnedFrames;// Count of frames used for noise learning
    bool     _isLearning;   // Flag to indicate active noise learning
    bool     _isInitialized;// Flag to indicate if the engine is initialized

    // --- Vibration용 버퍼 (Internal SRAM 또는 PSRAM 선택적 할당) ---
    float* _fftWorkVib;     // FFT Complex Work Buffer
    float* _powerVib;       // Power Spectrum Buffer
    float* _melScratchBuf;  // Mel Scratch (CHUNK x MEL)
    float* _dctScratchBuf;  // DCT Scratch (MEL x MFCC)
    
    // v243 보완: 4채널(Vib 3 + Audio 1) 히스토리 독립 관리 (PSRAM 할당 권장)
    float (*_mfccHistory)[T2_Def::Shared::FeatureLimit::DELTA_HISTORY_MAX][MFCC_PADDED];
    float (*_deltaHistory)[T2_Def::Shared::FeatureLimit::DELTA_HISTORY_MAX][MFCC_PADDED];
    uint16_t _historyCount[4];

    // 내부 연산 헬퍼
    void _computeVibStats(const float* p_data, uint32_t p_len, float& p_rms, float& p_kurt, float& p_crest, float& p_skew, float& p_std);
    void _extractTopPeaks(const float* p_power, uint32_t p_bins, float p_sampleRate, T2_Type::SpectralPeak* p_outPeaks, uint8_t p_maxCount, float p_ampMin, float p_freqGap);
    void _computeMfcc(const float* p_power, float* p_outMfcc, uint8_t p_chIdx);
    float _computeSpectralCentroid(const float* p_power, uint32_t p_bins, float p_sampleRate);
    void  _computeBandEnergies(const float* p_power, uint32_t p_bins, float p_sampleRate, uint8_t p_count, const bool* p_en, const float* p_start, const float* p_end, float* p_outEnergies);

public:
    CL_T2_FeatureExtractor();
    ~CL_T2_FeatureExtractor();

    /**
     * @brief 엔진 초기화 및 PSRAM 버퍼 할당
     */
    bool init();

    /**
     * @brief 진동 특징량 추출 (X, Y, Z 독립 처리)
     */
    void extractVibration(const float* p_vibX, const float* p_vibY, const float* p_vibZ, uint32_t p_len, uint32_t p_sampleRate, T2_Type::UnifiedFeatureSlot& p_slot, const T2_Type::ST_Vib_Feature& p_featCfg, const T2_Type::ST_Vib_Trigger& p_trigCfg, uint8_t p_axisCount = 3);

    /**
     * @brief 소음 특징량 추출 (RMS, Spectral, MFCC39)
     */
    void extractAudio(const float* p_aud, uint32_t p_len, uint32_t p_sampleRate, T2_Type::UnifiedFeatureSlot& p_slot, const T2_Type::ST_Audio_Feature& p_featCfg, const T2_Type::ST_Audio_Trigger& p_trigCfg);

    /**
     * @brief 노이즈 프로필 학습 시작/중단
     */
    void setNoiseLearning(bool p_enable);

    /**
     * @brief 노이즈 프로필 초기화
     */
    void resetNoiseProfile();

    /**
     * @brief MFCC 델타 연산을 위한 히스토리 초기화
     */
    void resetHistory();

    /**
     * @brief [v015 이식] 오디오 파워 스펙트럼 버퍼 포인터 반환 (스트리밍용)
     */
    const float* getAudioPowerSpectrum() const { return _powerAudio; }
};
