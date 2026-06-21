/* 
============================================================================
 * File: T245_FeatExtra_250.hpp
 * Summary: 멀티모달(진동/소음) 융합 특징량 추출 엔진 헤더
 * 주요기능 :
 *  - xxx
 ============================================================================ 
*/

#pragma once

#include "T210_Def_250.hpp"
#include "T215_Type_250.hpp"
#include "esp_dsp.h"
#include <cstdint>
#include <bitset>

class CL_T2_FeatureExtractor {
private:
    // 행렬 연산용 패딩 상수 (SIMD 최적화)
    static constexpr uint16_t MEL_BANDS_MAX       = T2_Def::Audio::FeatureLimit::MEL_BANDS_MAX;     // 멜 밴드 최대 개수
    static constexpr uint16_t MFCC_DIM_MAX        = T2_Def::AI::Tensor::MFCC_DIM_MAX;               // MFCC 차원 최대 개수
    static constexpr uint16_t BINS_MAX            = (T2_Def::Audio::Sensor::FFT_SIZE_MAX / 2) + 1;  // FFT 빈 최대 개수

    static constexpr uint16_t MEL_PADDED          = (MEL_BANDS_MAX + 3) & ~3;                       // 멜 패딩 개수
    static constexpr uint16_t BINS_PADDED         = (BINS_MAX + 3) & ~3;                            // 빈 패딩 개수
    static constexpr uint16_t MFCC_COEFFS_MAX_VAL = T2_Def::AI::Tensor::MFCC_COEFFS_MAX;            // MFCC 계수 최대 개수
    static constexpr uint16_t MFCC_PADDED         = (MFCC_COEFFS_MAX_VAL + 3) & ~3;                 // MFCC 패딩 개수

	static constexpr uint16_t BINS_IMU_MAX        = (T2_Def::Accel::Sensor::FFT_SIZE_DEF / 2) + 1;  // IMU FFT 빈 최대 개수
	static constexpr uint16_t BINS_IMU_PADDED     = (BINS_IMU_MAX + 3) & ~3;                        // IMU 빈 패딩 개수

    // --- Audio용 동적 할당 버퍼 (PSRAM) ---
    float*   _melBankFlat;                          // Mel 필터 뱅크 행렬 (BINS x MEL)
    float*   _dctMatrixFlat;                        // DCT 행렬 (MEL x MFCC)
    float*   _fftWorkAudio;                         // 복소수 FFT 작업 버퍼
    float*   _powerAudio;                           // 파워 스펙트럼 버퍼
    float*   _windowAudio;                          // 윈도우잉 버퍼
    float*   _noiseProfile;                         // 학습된 노이즈 프로파일
    uint32_t _learnedFrames;                        // 학습된 프레임 수
    bool     _isLearning;                           // 학습 플래그
    bool     _isInitialized;                        // 초기화 플래그
    uint8_t  _activeMelBands;                       // 활성 멜 밴드 수

	float*   _melBankIMU;                           // IMU용 Mel (BINS x MEL)
    float*   _dctMatrixIMU;                         // IMU용 DCT (MEL x MFCC)


    // --- IMU용 버퍼 ---
    float*   _fftWorkIMU;                           // FFT 복소수 작업 버퍼 (가속도/자이로 공용)
    float*   _powerIMU;                             // 파워 스펙트럼 버퍼
    float*   _cepsIfftWork;                         // Cepstrum IFFT 복소수 버퍼

    // 8채널 (Accel 3 + Gyro 3 + Audio 2) 히스토리 독립 관리
    float (*_mfccHistory)[T2_Def::AI::Tensor::DELTA_HISTORY_MAX][MFCC_PADDED];   // 8채널 MFCC 이력 버퍼
    float (*_deltaHistory)[T2_Def::AI::Tensor::DELTA_HISTORY_MAX][MFCC_PADDED];  // 8채널 델타 이력 버퍼
    uint16_t _historyCount[8];                      // 채널별 히스토리 카운트
    uint8_t  _historyHead[8];                       // 채널별 히스토리 헤드

    float    _prevRms[2];                           // 이전 RMS 값
    float    _prevDRms[2];                          // 이전 RMS 변화량

    // 내부 연산 헬퍼
    // @brief: 데이터의 RMS, Kurt, Crest, Skew, Std 계산
    void _computeStats(const float* p_data, uint32_t p_len, float& p_rms, float& p_kurt, float& p_crest, float& p_skew, float& p_std);
    // @brief: 스펙트럼 피크 추출
    void _extractTopPeaks(const float* p_power, uint32_t p_bins, float p_sampleRate, T2_Type::ST_SpectralPeak_t* p_outPeaks, uint8_t p_maxCount, float p_ampMin, float p_freqGap);
	// @brief: MFCC 계산
    void _computeMfcc(const float* p_power, uint32_t p_bins, float* p_outMfcc, uint8_t p_chIdx, bool isAudio);
    // @brief: 스펙트럼 중심(Centroid) 계산
    float _computeSpectralCentroid(const float* p_power, uint32_t p_bins, float p_sampleRate);
    // @brief: 밴드 에너지 계산
    void  _computeBandEnergies(const float* p_power, uint32_t p_bins, float p_sampleRate, uint8_t p_count, const std::bitset<16>& p_en, const float* p_start, const float* p_end, float* p_outEnergies);

public:
    CL_T2_FeatureExtractor();
    ~CL_T2_FeatureExtractor();

    // 오디오 설정 스펙 정보에 따라 FFT, Mel Filterbank 및 DCT 변환용 메모리 버퍼들을 힙에 할당 초기화합니다. (audioCfg: 오디오 설정, 반환값: 성공 여부)
    bool init(const T2_Type::ST_Audio_Config_t& audioCfg);

    // 가속도 센서 3축 신호로부터 RMS, Crest, Skew, 밴드 에너지 및 푸리에 파워 스펙트럼 등의 특징량을 연산합니다. (p_inX/Y/Z: 입력 진동, p_len: 크기, p_sampleRate: 샘플율, p_slot: 출력 데이터 슬롯 참조, p_accCfg: 센서 사양)
    void extractAccel(const float* p_inX, const float* p_inY, const float* p_inZ, uint32_t p_len, uint32_t p_sampleRate,
                      T2_Type::ST_FeatureSlot_Vib_t& p_vibSlot, const T2_Type::ST_Accel_Config_t& p_accCfg);

    // 자이로 센서 3축 신호로부터 진폭 통계치 및 밴드 에너지, 푸리에 스펙트럼 피크 등의 관성 특징량을 연산합니다. (p_inX/Y/Z: 입력 데이터, p_len: 크기, p_sampleRate: 샘플율, p_slot: 출력 데이터 슬롯 참조, p_gyrCfg: 센서 사양)
    void extractGyro(const float* p_inX, const float* p_inY, const float* p_inZ, uint32_t p_len, uint32_t p_sampleRate,
                     T2_Type::ST_FeatureSlot_Vib_t& p_vibSlot, const T2_Type::ST_Gyro_Config_t& p_gyrCfg);

    // 좌우 사운드 파형으로부터 밴드 에너지, MFCC 계수, 델타 특징량 및 통계 특징값을 수집합니다. (p_audL/R: 입력 사운드, p_len: 크기, p_sampleRate: 샘플율, p_slot: 출력 데이터 슬롯 참조, p_audCfg: 센서 사양)
    void extractAudio(const float* p_audL, const float* p_audR, uint32_t p_len, uint32_t p_sampleRate,
                      T2_Type::ST_FeatureSlot_Aud_t& p_audSlot, const T2_Type::ST_Audio_Config_t& p_audCfg);

    // 노이즈 프로파일 학습 연산 수행 여부를 켜거나 끕니다. (p_enable: 학습 활성화 여부)
    void setNoiseLearning(bool p_enable);

    // 변경된 주파수 샘플율에 맞춰 내부 오디오 Mel 필터 계수 평탄화 행렬을 재구성합니다. (sampleRate: 샘플율)
    void reloadAudioMelFilter(float sampleRate);

    // 기학습 누적된 배경 소음 노이즈 스펙트럼 통계치를 리셋합니다.
    void resetNoiseProfile();

    // MFCC 델타/더블델타 가속도 추출에 쓰이는 시간차 이력 링버퍼의 데이터를 전부 클리어합니다.
    void resetHistory();

    // 현재 추출 연산이 적용된 최종 사운드 파워 스펙트럼 배열의 주소 포인터를 반환합니다. (반환값: 파워 스펙트럼 버퍼 포인터)
    const float* getAudioPowerSpectrum() const { return _powerAudio; }
};
