/* ============================================================================
 * File: T240_FeatExtra_241.hpp
 * Summary: 멀티모달(진동/소음) 융합 특징량 추출 엔진 (고장 탐지 최적화)
 * ============================================================================
 * [로직 상태 추적 (Logic State Tracker)]
 * - 유지: T440의 PSRAM 메모리 타일링 기반 39D MFCC 및 오디오 통계 추출.
 * - 갱신: [교정] 진동 파이프라인에 첨도(Kurtosis) 및 N-Top Peaks 탐지 로직 전면 복원.
 * - 신규: [원칙 준수] 모든 지역 변수 v_ 접두어 적용 및 64비트 시계열 동기화 바인딩.
 *
 * [축소/누락 방어 체크리스트 (Omission Defense)]
 * 1. [고장 탐지 무결성]: 베어링 고장 진단 필수 지표인 Kurtosis 연산 누락 방지 (Rule #42).
 * 2. [수학적 결함 방어]: NMS(Non-Maximum Suppression)를 통한 가짜 피크 제거 로직 복원 (Rule #9).
 * 3. [메모리 안전성]: 1024-FFT급 거대 버퍼는 반드시 PSRAM 힙 할당 유지 (Rule #21).
 * 4. [SSOT]: 모든 런타임 임계값은 CL_T2_ConfigManager의 캐시를 통해서만 접근.
 * ========================================================================== */
#pragma once

#include "T210_Def_241.hpp"
#include <cstdint>

class CL_T2_FeatureExtractor {
private:
	// 행렬 연산 상수 및 스크래치
    static constexpr uint16_t MEL_PADDED 		= (T2_Config::System::MEL_BANDS_CONST + 3) & ~3;
    static constexpr uint16_t BINS_PADDED 		= ((T2_Config::System::FFT_SIZE_AUDIO_CONST / 2 + 1) + 3) & ~3;
    static constexpr uint16_t MFCC_PADDED 		= (T2_Config::System::MFCC_COEFFS_CONST + 3) & ~3;
    static constexpr uint16_t MEL_CHUNK_ROWS 	= 16;

	T2_Type::DynamicConfig _cachedCfg;

    // --- Audio용 동적 할당 버퍼 (PSRAM) ---
    float* _melBankFlat;
    float* _dctMatrixFlat;
    float* _fftWorkAudio;
    float* _powerAudio;
    float* _windowAudio;



    // 환경 노이즈 학습 및 차감을 위한 동적 버퍼 및 상태 변수
    alignas(16) float _noiseProfile[BINS_PADDED];
    uint32_t _noiseLearnedFrames;
    bool     _isNoiseLearning;

    // --- Vibration용 정적 버퍼 (Internal SRAM) ---
    alignas(16) float _fftWorkVib[T2_Config::System::VIB_AXIS_CONST][T2_Config::System::FFT_SIZE_VIB_CONST * 2];
    alignas(16) float _powerVib[T2_Config::System::VIB_AXIS_CONST][T2_Config::System::FFT_SIZE_VIB_CONST];
    alignas(16) float _windowVib[T2_Config::System::FFT_SIZE_VIB_CONST];

    // MFCC 히스토리
    uint16_t _historyCount;
    alignas(16) float _mfccHistory[5][T2_Config::System::MFCC_COEFFS_CONST];
    alignas(16) float _deltaHistory[5][T2_Config::System::MFCC_COEFFS_CONST];


    alignas(16) float _melScratchBuf[MEL_CHUNK_ROWS * MEL_PADDED];
    alignas(16) float _dctScratchBuf[MEL_PADDED * MFCC_PADDED];

    // 내부 연산 엔진
    void _computeVibPeaks(uint8_t p_axis, T2_Type::UnifiedFeatureSlot& p_slot);
    void _computeAudioMfcc39(T2_Type::UnifiedFeatureSlot& p_slot);
    void _applyTemporalDerivatives(T2_Type::UnifiedFeatureSlot& p_slot);

    // 스펙트럴 센트로이드 헬퍼 선언
    void _computeSpectralCentroid(T2_Type::UnifiedFeatureSlot& p_slot);


public:
    CL_T2_FeatureExtractor();
    ~CL_T2_FeatureExtractor();

    bool init();
    void reloadConfigCache();
	void resetNoiseProfile();

	void setNoiseLearning(bool p_active) { _isNoiseLearning = p_active; } // [추가]


    // [복원] 진동 특징량 (Kurtosis 포함)
    void extractVibration(const float* p_vibX, const float* p_vibY, const float* p_vibZ,
                          uint32_t p_len, T2_Type::UnifiedFeatureSlot& p_slot);

    // [보완] 오디오 특징량
    void extractAudio(const float* p_audioBeam, uint32_t p_len, T2_Type::UnifiedFeatureSlot& p_slot);
};
