/* ============================================================================
 * [SMEA-100 핵심 구현 원칙 및 AI 셀프 회고 바이블]
 * 1. [방어] 거대 버퍼 스택 할당 금지: RTOS Stack Overflow 방지를 위해 수 KB에 달하는
 * 텐서 버퍼는 init() 시 MALLOC_CAP_SPIRAM으로 힙에 할당한다.
 * 2. [방어] SIMD 정렬 위반 차단: ESP-DSP 행렬 곱셈 전 16-Byte 강제 정렬.
 * 3. [방어] 수학적 결함 교정: Parseval 정리, NMS, EMA 적용 유지.
 * 4. [물리 병목 타파]: PSRAM과 SRAM 간 직접적인 행렬 곱셈 시 발생하는 캐시 미스를
 * 막기 위해, Internal SRAM 스크래치 버퍼를 이용한 '메모리 타일링' 기법 도입.
 * 5. [v013 핫스왑 방어]: extract() 핫패스 내에서 Mutex 호출 완전 제거.
 * T415의 isTuningActive()를 감지하여 락(Lock) 없이 파라미터를 실시간 리로드함.
 * 어설픈 노이즈 스케일링을 폐기하고 하드 리셋(resetNoiseProfile)으로 수학적 무결성 확보.
 * ============================================================================
 * File: T440_FeatExtra_013.hpp
 * Summary: 39D MFCC, Cepstrum & Spatial Feature Extraction (Tiling Optimized)
 * ========================================================================== */
#pragma once

#include "T410_Def_013.hpp"
#include "T420_Types_013.hpp"
#include "T415_ConfigMgr_013.hpp"
#include <cstdint>

class T440_FeatureExtractor {
private:
    static constexpr uint16_t BINS_PADDED = ((SmeaConfig::System::FFT_SIZE_CONST / 2) + 1 + 3) & ~3;
    static constexpr uint16_t MEL_PADDED  = (SmeaConfig::System::MEL_BANDS_CONST + 3) & ~3;
    static constexpr uint16_t MFCC_PADDED = (SmeaConfig::System::MFCC_COEFFS_CONST + 3) & ~3;

    alignas(16) float         _mfccHistory[SmeaConfig::FeatureLimit::DELTA_HISTORY_FRAMES_CONST][SmeaConfig::System::MFCC_COEFFS_CONST];
    alignas(16) float         _rmsHistory[SmeaConfig::FeatureLimit::DELTA_HISTORY_FRAMES_CONST];
    alignas(16) float         _deltaHistory[SmeaConfig::FeatureLimit::DELTA_HISTORY_FRAMES_CONST][SmeaConfig::System::MFCC_COEFFS_CONST];
    uint8_t                   _historyCount;

    float* _melBankFlat;
    float* _dctMatrixFlat;
    float* _fftSpatialL;
    float* _fftSpatialR;

    alignas(16) float         _fftWorkBuf[SmeaConfig::System::FFT_SIZE_CONST * 2];
    alignas(16) float         _powerSpectrum[BINS_PADDED];
    alignas(16) float         _window[SmeaConfig::System::FFT_SIZE_CONST];
    
    static constexpr uint16_t MEL_CHUNK_ROWS = 64; 
    alignas(16) float         _melScratchBuf[MEL_CHUNK_ROWS * MEL_PADDED];
    alignas(16) float         _dctScratchBuf[MEL_PADDED * MFCC_PADDED];

    alignas(16) float         _noiseProfile[BINS_PADDED];
    uint32_t                  _noiseLearnedFrames = 0;      
    bool                      _isNoiseLearning = false;
    
    DynamicConfig             _cachedCfg;  // 캐싱용 변수

public:
    T440_FeatureExtractor();
    ~T440_FeatureExtractor();

    bool init();

    void extract(const float* p_cleanSignal, const float* p_rawL, const float* p_rawR,
                 uint32_t p_len, SmeaType::FeatureSlot& p_outSlot);

    void setNoiseLearning(bool p_active) {
        _isNoiseLearning = p_active;
        if(p_active) _noiseLearnedFrames = 0;
    }
    
    void resetNoiseProfile();
    // void scaleNoiseProfile(float p_gainRatio);  <-- [v013 수학적 무결성 원칙에 의해 영구 삭제]
    
    void reloadConfigCache();
    
    // [항목 29 방어] 스펙트럼 패킷 전송을 위한 FFT 버퍼 Getter
    const float* getPowerSpectrumBuf() const { return _powerSpectrum; }


private:
    void _computeBasicFeatures(const float* p_signal, uint32_t p_len, SmeaType::FeatureSlot& p_slot);
    void _computePowerSpectrum(const float* p_signal, uint32_t p_len);
    void _computeSpectralCentroid(SmeaType::FeatureSlot& p_slot);
    void _computeBandRMS(SmeaType::FeatureSlot& p_slot);
    void _computeNtopPeaks(SmeaType::FeatureSlot& p_slot);
    void _computeMfcc39(SmeaType::FeatureSlot& p_slot);
    void _computeCepstrum(SmeaType::FeatureSlot& p_slot);
    void _computeSpatialFeatures(const float* p_L, const float* p_R, uint32_t p_len, SmeaType::FeatureSlot& p_slot);
    void _applyTemporalDerivatives(SmeaType::FeatureSlot& p_slot);
};

