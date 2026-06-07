/* ============================================================================
 * File: T245_FeatExtra_245.hpp
 * Summary: 멀티모달(진동/소음) 융합 특징량 추출 엔진
 * ============================================================================ */

#pragma once

#include "T210_Def_245.hpp"
#include "T215_Type_245.hpp"
#include "esp_dsp.h"
#include <cstdint>

/**
 * @class CL_T2_FeatureExtractor
 * @brief 수집/필터링된 오디오 및 IMU 원시 데이터를 사용하여 RMS, Centroid, MFCC 등의 특징 벡터를 연산하는 클래스
 */
class CL_T2_FeatureExtractor {
private:
    // 행렬 연산용 패딩 상수 (SIMD 최적화)
    static constexpr uint16_t MEL_BANDS_MAX = T2_Def::Audio::FeatureLimit::MEL_BANDS_MAX;
    static constexpr uint16_t MFCC_DIM_MAX  = T2_Def::AI::Tensor::MFCC_DIM_MAX;
    static constexpr uint16_t BINS_MAX      = (T2_Def::Audio::Sensor::FFT_SIZE_MAX / 2) + 1;

    static constexpr uint16_t MEL_PADDED    = (MEL_BANDS_MAX + 3) & ~3;
    static constexpr uint16_t BINS_PADDED   = (BINS_MAX + 3) & ~3;
    static constexpr uint16_t MFCC_COEFFS_MAX_VAL = T2_Def::AI::Tensor::MFCC_COEFFS_MAX;
    static constexpr uint16_t MFCC_PADDED   = (MFCC_COEFFS_MAX_VAL + 3) & ~3;

    static constexpr uint16_t BINS_IMU_MAX   = (T2_Def::Accel::Sensor::FFT_SIZE_DEF / 2) + 1;   // 513
    static constexpr uint16_t BINS_IMU_PADDED = (BINS_IMU_MAX + 3) & ~3;                       // 516

    // --- Audio용 동적 할당 버퍼 (PSRAM) ---
    float* _melBankFlat;    ///< Mel Filterbank 행렬 버퍼 (BINS x MEL)
    float* _dctMatrixFlat;  ///< DCT 변환 행렬 버퍼 (MEL x MFCC)
    float* _fftWorkAudio;   ///< 오디오 FFT 연산용 복소수 작업 버퍼
    float* _powerAudio;     ///< 오디오 파워 스펙트럼 결과 버퍼
    float* _windowAudio;    ///< 오디오 윈도잉 데이터 임시 버퍼
    float* _noiseProfile;   ///< 학습된 소음 프로파일 버퍼 (차감용)
    uint32_t _learnedFrames; ///< 소음 프로파일 생성 시 누적 프레임 수
    bool     _isLearning;    ///< 현재 소음 프로파일 학습 모드 작동 여부
    bool     _isInitialized; ///< 특징 추출 엔진 초기화 완료 플래그
    uint8_t  _activeMelBands; ///< 실시간 적용 활성화 멜 밴드 개수

    float* _melBankIMU;    ///< IMU용 Mel 필터뱅크 행렬 버퍼 (BINS x MEL)
    float* _dctMatrixIMU;  ///< DCT 변환 행렬 버퍼 (MEL x MFCC)

    // --- IMU용 버퍼 ---
    float* _fftWorkIMU;     ///< IMU FFT 복소수 작업 버퍼 (가속도/자이로 공용)
    float* _powerIMU;       ///< IMU 파워 스펙트럼 결과 버퍼
    float* _cepsIfftWork;   ///< Cepstrum 켑스트럼 IFFT 복소수 작업 버퍼

    // 8채널 (Accel 3축 + Gyro 3축 + Audio 2채널) 히스토리 독립 관리 (델타/델타-델타 특징 연산용)
    float (*_mfccHistory)[T2_Def::AI::Tensor::DELTA_HISTORY_MAX][MFCC_PADDED];
    float (*_deltaHistory)[T2_Def::AI::Tensor::DELTA_HISTORY_MAX][MFCC_PADDED];
    uint16_t _historyCount[8]; ///< 각 채널별 히스토리 누적 수
    uint8_t  _historyHead[8];  ///< 각 채널별 링 버퍼 헤드 인덱스

    float _prevRms[2];         ///< 이전 프레임 오디오 RMS (STA/LTA 연산용)
    float _prevDRms[2];        ///< 이전 프레임의 Delta RMS

    // 내부 연산 헬퍼 함수
    void _computeStats(const float* p_data, uint32_t p_len, float& p_rms, float& p_kurt, float& p_crest, float& p_skew, float& p_std);
    void _extractTopPeaks(const float* p_power, uint32_t p_bins, float p_sampleRate, T2_Type::ST_SpectralPeak_t* p_outPeaks, uint8_t p_maxCount, float p_ampMin, float p_freqGap);
    void _computeMfcc(const float* p_power, uint32_t p_bins, float* p_outMfcc, uint8_t p_chIdx, bool isAudio);
    float _computeSpectralCentroid(const float* p_power, uint32_t p_bins, float p_sampleRate);
    void  _computeBandEnergies(const float* p_power, uint32_t p_bins, float p_sampleRate, uint8_t p_count, const bool* p_en, const float* p_start, const float* p_end, float* p_outEnergies);

public:
    CL_T2_FeatureExtractor();
    ~CL_T2_FeatureExtractor();

    /**
     * @brief 오디오 구성 및 IMU 필터뱅크 등 특징 추출 엔진 초기화
     * @param audioCfg 오디오 채널 및 샘플 설정 정보
     * @return 초기화 성공 여부
     */
    bool init(const T2_Type::ST_Audio_Config_t& audioCfg);

    /**
     * @brief 3축 가속도 raw 신호 특징 추출 (RMS, kurtosis, crest factor 등)
     * @param p_inX, p_inY, p_inZ 3축 raw 센서 입력 신호 버퍼
     * @param p_len 샘플 길이
     * @param p_sampleRate 샘플링 주파수 (Hz)
     * @param p_slot 결과 기록용 통합 특징 구조체 슬롯
     * @param p_accCfg 가속도계 세부 채널 설정
     */
    void extractAccel(const float* p_inX, const float* p_inY, const float* p_inZ, uint32_t p_len, uint32_t p_sampleRate,
                      T2_Type::ST_UnifiedFeatureSlot_t& p_slot, const T2_Type::ST_Accel_Config_t& p_accCfg);

    /**
     * @brief 3축 자이로 raw 신호 특징 추출
     * @param p_inX, p_inY, p_inZ 3축 raw 센서 입력 신호 버퍼
     * @param p_len 샘플 길이
     * @param p_sampleRate 샘플링 주파수 (Hz)
     * @param p_slot 결과 기록용 통합 특징 구조체 슬롯
     * @param p_gyrCfg 자이로계 세부 채널 설정
     */
    void extractGyro(const float* p_inX, const float* p_inY, const float* p_inZ, uint32_t p_len, uint32_t p_sampleRate,
                     T2_Type::ST_UnifiedFeatureSlot_t& p_slot, const T2_Type::ST_Gyro_Config_t& p_gyrCfg);

    /**
     * @brief 2채널 오디오 raw 신호 특징 추출 (RMS, Spectral Centroid, MFCC 등)
     * @param p_audL, p_audR 좌/우 채널 raw 마이크 입력 버퍼
     * @param p_len 샘플 길이
     * @param p_sampleRate 샘플링 주파수 (Hz)
     * @param p_slot 결과 기록용 통합 특징 구조체 슬롯
     * @param p_audCfg 마이크/오디오 세부 채널 설정
     */
    void extractAudio(const float* p_audL, const float* p_audR, uint32_t p_len, uint32_t p_sampleRate,
                      T2_Type::ST_UnifiedFeatureSlot_t& p_slot, const T2_Type::ST_Audio_Config_t& p_audCfg);

    /**
     * @brief 소음 학습 프로파일링 모드 설정
     * @param p_enable true 설정 시 다음 오디오 프레임들을 학습에 반영
     */
    void setNoiseLearning(bool p_enable);

    /**
     * @brief 샘플 주파수 변경에 대응하여 오디오 Mel 필터뱅크 재구성
     * @param sampleRate 변경된 샘플 주파수
     */
    void reloadAudioMelFilter(float sampleRate);

    /**
     * @brief 학습된 소음 차감 프로파일 초기화
     */
    void resetNoiseProfile();

    /**
     * @brief 연산용 프레임 특징 히스토리 클리어
     */
    void resetHistory();

    /**
     * @brief 현재 계산된 오디오 파워 스펙트럼 포인터 반환
     */
    const float* getAudioPowerSpectrum() const { return _powerAudio; }
};
