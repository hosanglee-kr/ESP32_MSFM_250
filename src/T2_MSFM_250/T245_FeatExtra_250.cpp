/* ============================================================================
 * File: T245_FeatExtra_250.cpp
 * Summary: 멀티모달(진동/소음) 융합 특징량 추출 엔진 구현부
 * ============================================================================ */

#include "T245_FeatExtra_250.hpp"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "dspm_mult.h"
#include <cmath>
#include <algorithm>
#include <cstring>

#include "T246_MelGen_250.hpp"
#include "esp32s3/rom/cache.h"

static const char* TAG = "T245_FEAT";

CL_T2_FeatureExtractor::CL_T2_FeatureExtractor()
    : _melBankFlat(nullptr), _dctMatrixFlat(nullptr), _fftWorkAudio(nullptr),
      _powerAudio(nullptr), _windowAudio(nullptr), _noiseProfile(nullptr),
      _fftWorkIMU(nullptr), _powerIMU(nullptr),
      _cepsIfftWork(nullptr), _mfccHistory(nullptr), _deltaHistory(nullptr),
      _learnedFrames(0), _isLearning(false), _isInitialized(false),
      _activeMelBands(T2_Def::Audio::FeatureLimit::MEL_BANDS_DEF) {
    memset(_historyCount, 0, sizeof(_historyCount));
    memset(_historyHead, 0, sizeof(_historyHead));
    _prevRms[0] = 0.0f; _prevRms[1] = 0.0f;
    _prevDRms[0] = 0.0f; _prevDRms[1] = 0.0f;
}

CL_T2_FeatureExtractor::~CL_T2_FeatureExtractor() {
    if (_melBankFlat) heap_caps_free(_melBankFlat);
    if (_dctMatrixFlat) heap_caps_free(_dctMatrixFlat);
    if (_fftWorkAudio) heap_caps_free(_fftWorkAudio);
    if (_powerAudio) heap_caps_free(_powerAudio);
    if (_windowAudio) heap_caps_free(_windowAudio);
    if (_noiseProfile) heap_caps_free(_noiseProfile);
    if (_fftWorkIMU) heap_caps_free(_fftWorkIMU);
    if (_powerIMU) heap_caps_free(_powerIMU);
    if (_cepsIfftWork) heap_caps_free(_cepsIfftWork);
    if (_mfccHistory) heap_caps_free(_mfccHistory);
    if (_deltaHistory) heap_caps_free(_deltaHistory);
}

bool CL_T2_FeatureExtractor::init(const T2_Type::ST_Audio_Config_t& audioCfg) {

    // 런타임 활성 Mel 대역 수 설정 (경계값 검증)
    _activeMelBands = audioCfg.mel_bands;
    // 설정값이 유효한 범위 내에 있는지 확인하고, 범위를 벗어나는 경우 기본값 사용
    if (_activeMelBands == 0 || _activeMelBands > MEL_BANDS_MAX) {
        _activeMelBands = T2_Def::Audio::FeatureLimit::MEL_BANDS_DEF;
    }

    // 오디오용 버퍼 할당
    _melBankFlat   = (float*)heap_caps_aligned_alloc(16, BINS_PADDED * MEL_PADDED * sizeof(float), MALLOC_CAP_SPIRAM);
    _dctMatrixFlat = (float*)heap_caps_aligned_alloc(16, MEL_PADDED * MFCC_PADDED * sizeof(float), MALLOC_CAP_SPIRAM);
    _fftWorkAudio  = (float*)heap_caps_aligned_alloc(16, T2_Def::Audio::Sensor::FFT_SIZE_MAX * 2 * sizeof(float), MALLOC_CAP_SPIRAM);
    _powerAudio    = (float*)heap_caps_aligned_alloc(16, BINS_PADDED * sizeof(float), MALLOC_CAP_SPIRAM);
    _windowAudio   = (float*)heap_caps_aligned_alloc(16, T2_Def::Audio::Sensor::FFT_SIZE_MAX * sizeof(float), MALLOC_CAP_SPIRAM);
    _noiseProfile  = (float*)heap_caps_aligned_alloc(16, 2 * BINS_PADDED * sizeof(float), MALLOC_CAP_SPIRAM);

    // 가속도 및 자이로 센서 공용 FFT, 파워 및 Cepstrum 복소수 버퍼 할당
    _fftWorkIMU    = (float*)heap_caps_aligned_alloc(16, T2_Def::Accel::Sensor::FFT_SIZE_MAX * 2 * sizeof(float), MALLOC_CAP_SPIRAM);
    _powerIMU      = (float*)heap_caps_aligned_alloc(16, T2_Def::Accel::Sensor::FFT_SIZE_MAX * sizeof(float), MALLOC_CAP_SPIRAM);
    _cepsIfftWork  = (float*)heap_caps_aligned_alloc(16, T2_Def::Audio::Sensor::FFT_SIZE_MAX * 2 * sizeof(float), MALLOC_CAP_SPIRAM);


    // IMU 전용 Mel / DCT 할당
    _melBankIMU    = (float*)heap_caps_aligned_alloc(16, BINS_IMU_PADDED * MEL_PADDED * sizeof(float), MALLOC_CAP_SPIRAM);
    _dctMatrixIMU  = (float*)heap_caps_aligned_alloc(16, MEL_PADDED * MFCC_PADDED * sizeof(float), MALLOC_CAP_SPIRAM);

    // MFCC 계산을 위한 이력 배열
    _mfccHistory   = (float (*)[T2_Def::AI::Tensor::DELTA_HISTORY_MAX][MFCC_PADDED])heap_caps_aligned_alloc(
                         16, sizeof(float) * 8 * T2_Def::AI::Tensor::DELTA_HISTORY_MAX * MFCC_PADDED, MALLOC_CAP_SPIRAM);
    // 델타 계산을 위한 이력 배열
    _deltaHistory  = (float (*)[T2_Def::AI::Tensor::DELTA_HISTORY_MAX][MFCC_PADDED])heap_caps_aligned_alloc(
                         16, sizeof(float) * 8 * T2_Def::AI::Tensor::DELTA_HISTORY_MAX * MFCC_PADDED, MALLOC_CAP_SPIRAM);

    // 메모리 할당 실패 검사
    if (!_melBankFlat || !_dctMatrixFlat || !_fftWorkAudio || !_powerAudio || !_windowAudio || !_noiseProfile ||
        !_fftWorkIMU || !_powerIMU || !_cepsIfftWork ||
        !_melBankIMU || !_dctMatrixIMU || !_mfccHistory || !_deltaHistory) {
        ESP_LOGE(TAG, "Memory Allocation Failed");
        return false;
    }

    // 제로 초기화
    memset(_mfccHistory,   0, sizeof(float) * 8 * T2_Def::AI::Tensor::DELTA_HISTORY_MAX * MFCC_PADDED);
    memset(_deltaHistory,  0, sizeof(float) * 8 * T2_Def::AI::Tensor::DELTA_HISTORY_MAX * MFCC_PADDED);
    memset(_historyCount,  0, sizeof(_historyCount));
    memset(_historyHead,   0, sizeof(_historyHead));
    memset(_melBankFlat,   0, BINS_PADDED * MEL_PADDED * sizeof(float));
    memset(_melBankIMU,    0, BINS_IMU_PADDED * MEL_PADDED * sizeof(float));
    memset(_dctMatrixFlat, 0, MEL_PADDED * MFCC_PADDED * sizeof(float));
    memset(_dctMatrixIMU,  0, MEL_PADDED * MFCC_PADDED * sizeof(float));
    memset(_noiseProfile,  0, 2 * BINS_PADDED * sizeof(float));

    // 오디오 윈도우 1회 사전 생성
    dsps_wind_hann_f32(_windowAudio, T2_Def::Audio::Sensor::FFT_SIZE_MAX);

    // IMU 전용 Mel Filterbank 생성 (나이퀴스트 800Hz 고정)
    MelFilterbankGenerator::generateImuMelFilterbank(_melBankIMU, BINS_IMU_MAX, MEL_PADDED, MEL_BANDS_MAX);

    // DCT Matrix 생성 (Audio / IMU 공통)
    for (int i = 0; i < MFCC_COEFFS_MAX_VAL; i++) {
        for (int j = 0; j < MEL_BANDS_MAX; j++) {
            _dctMatrixFlat[j * MFCC_PADDED + i] = cos(M_PI * i * (j + 0.5f) / MEL_BANDS_MAX);
        }
    }
    memcpy(_dctMatrixIMU, _dctMatrixFlat, MEL_PADDED * MFCC_PADDED * sizeof(float));

    // 오디오 멜 필터뱅크 (실제 샘플레이트 기반 동적 생성)
    reloadAudioMelFilter((float)audioCfg.sample_rate);

    // 완료
    _isInitialized = true;
    ESP_LOGI(TAG, "Feature Engine Initialized (Audio SR: %d Hz)", audioCfg.sample_rate);
    return true;
}

// Old init block removed.

void CL_T2_FeatureExtractor::extractAccel(const float* p_inX, const float* p_inY, const float* p_inZ, uint32_t p_len, uint32_t p_sampleRate,
                                          T2_Type::ST_FeatureSlot_Vib_t& p_vibSlot, const T2_Type::ST_Accel_Config_t& p_accCfg) {
    if (!_isInitialized) return;

    // 입력 데이터 포인터 배열 준비
    const float* v_in[3] = {p_inX, p_inY, p_inZ};

    // 축별 처리 루프
    for (int i = 0; i < T2_Def::Accel::Sensor::AXIS_MAX; i++) {
        if (!v_in[i]) continue;

        // 마스크 체크 (채널 활성화/비활성화)
        if (!(p_accCfg.axis_mask & (1 << i))) {
            // 비활성 축: 해당 축의 MFCC 및 히스토리 영역 제로 클리어
            const uint32_t singleChDim = T2_Def::AI::Tensor::MFCC_COEFFS_DEF * T2_Def::AI::Tensor::MFCC_COMPONENTS_DEF;

            // 출력 슬롯의 해당 채널 MFCC 데이터 영역을 0으로 초기화
            float* v_finalOut = &p_vibSlot.mfcc[i * singleChDim];
            memset(v_finalOut, 0, singleChDim * sizeof(float));

            // MFCC 히스토리 초기화
            memset(_mfccHistory[i], 0, sizeof(_mfccHistory[i]));
            // 델타 히스토리 초기화
            memset(_deltaHistory[i], 0, sizeof(_deltaHistory[i]));
            // 히스토리 카운트 초기화
            _historyCount[i] = 0;
            continue;
        }

        // 통계 추출 및 오프셋 직렬화
        _computeStats(v_in[i], p_len, p_vibSlot.accel.rms[i], p_vibSlot.accel.kurt[i], p_vibSlot.accel.crest[i], p_vibSlot.accel.skew[i], p_vibSlot.accel.std[i]);

        // 보정 오프셋 직렬화
        p_vibSlot.accel.cal_off[i] = p_accCfg.offset[i];

        // FFT 연산 버퍼 클리어
        memset(_fftWorkIMU, 0, T2_Def::Accel::Sensor::FFT_SIZE_MAX * 2 * sizeof(float));
        // 입력 데이터를 FFT 버퍼에 복사 (실수부만)
        for (uint32_t j = 0; j < p_len; j++) {
            _fftWorkIMU[j * 2] = v_in[i][j];
            _fftWorkIMU[j * 2 + 1] = 0.0f;
        }

        // L1 캐시 일관성 보장: CPU 쓰기 후 플러시
        Cache_WriteBack_Addr((uint32_t)_fftWorkIMU, T2_Def::Accel::Sensor::FFT_SIZE_MAX * 2 * sizeof(float));

        // FFT 실행
        dsps_fft2r_fc32(_fftWorkIMU, p_len);
        // 비트 역순 정렬
        dsps_bit_rev_fc32(_fftWorkIMU, p_len);

        // L1 캐시 일관성 보장: FPU/DMA 연산 후 캐시 무효화
        Cache_Invalidate_Addr((uint32_t)_fftWorkIMU, T2_Def::Accel::Sensor::FFT_SIZE_MAX * 2 * sizeof(float));

        // 주파수 빈 개수 계산
        uint32_t v_bins = (p_len / 2) + 1;
        float v_invLen = 1.0f / (float)p_len;
        // 전력 스펙트럼 계산
        for (uint32_t j = 0; j < v_bins; j++) {
            float re = _fftWorkIMU[j * 2];
            float im = _fftWorkIMU[j * 2 + 1];
            _powerIMU[j] = (re * re + im * im) * v_invLen;
        }

        // 스펙트럼 중심 주파수 계산
        p_vibSlot.accel.centroid[i] = _computeSpectralCentroid(_powerIMU, v_bins, (float)p_sampleRate);
        T2_Type::ST_SpectralPeak_t v_peaks[T2_Def::Accel::FeatureLimit::BAND_MAX] = {0};
        // 상위 1개 주파수 피크 추출
        _extractTopPeaks(_powerIMU, v_bins, (float)p_sampleRate, v_peaks, 1, p_accCfg.peak_amp_min, p_accCfg.peak_freq_gap_min);
        // 주 파수 피크 주파수 저장
        p_vibSlot.accel.peak_f[i] = v_peaks[0].freq;

        // 가변 밴드 에너지 계산 및 복사
        float v_currentBands[T2_Def::Accel::FeatureLimit::BAND_MAX] = {0};
        _computeBandEnergies(_powerIMU, v_bins, (float)p_sampleRate, p_accCfg.active_band_count, p_accCfg.band_en, p_accCfg.band_start, p_accCfg.band_end, v_currentBands);
        memcpy(p_vibSlot.accel.band_energy[i], v_currentBands, sizeof(v_currentBands));

        // MFCC 계산 및 히스토리 업데이트 (정책 기반)
        if constexpr (T2_General::AccelPolicy::enable_mfcc) {
            _computeMfcc(_powerIMU, v_bins, p_vibSlot.mfcc, i, false);
        } else {
            // MFCC 비활성화 시: 출력 버퍼 및 히스토리 초기화
            const uint32_t singleChDim = T2_Def::AI::Tensor::MFCC_COEFFS_DEF * T2_Def::AI::Tensor::MFCC_COMPONENTS_DEF;
            memset(&p_vibSlot.mfcc[i * singleChDim], 0, singleChDim * sizeof(float));
            memset(_mfccHistory[i], 0, sizeof(_mfccHistory[i]));
            memset(_deltaHistory[i], 0, sizeof(_deltaHistory[i]));
            _historyCount[i] = 0;
        }
    }
}

void CL_T2_FeatureExtractor::extractGyro(const float* p_inX, const float* p_inY, const float* p_inZ, uint32_t p_len, uint32_t p_sampleRate,
                                         T2_Type::ST_FeatureSlot_Vib_t& p_vibSlot, const T2_Type::ST_Gyro_Config_t& p_gyrCfg) {
    if (!_isInitialized) return;

    // 입력 데이터 포인터 배열 준비
    const float* v_in[3] = {p_inX, p_inY, p_inZ};

    // 자이로 축별 처리 루프
    for (int i = 0; i < T2_Def::Gyro::Sensor::AXIS_MAX; i++) {
        // 마스크 체크 (채널 활성화/비활성화)
        if (!(p_gyrCfg.axis_mask & (1 << i))) {
            // 비활성 축: 자이로 오프셋인 채널 3, 4, 5 영역 클리어 및 이력 초기화
            const uint32_t singleChDim = T2_Def::AI::Tensor::MFCC_COEFFS_DEF * T2_Def::AI::Tensor::MFCC_COMPONENTS_DEF;
            float* v_finalOut = &p_vibSlot.mfcc[(3 + i) * singleChDim];
            memset(v_finalOut, 0, singleChDim * sizeof(float));
            memset(_mfccHistory[3 + i], 0, sizeof(_mfccHistory[3 + i]));
            memset(_deltaHistory[3 + i], 0, sizeof(_deltaHistory[3 + i]));
            _historyCount[3 + i] = 0;
            continue;
        }

        // 1차 차분(각가속도) 계산
        alignas(16) float v_diff[T2_Def::Gyro::Sensor::FFT_SIZE_MAX] = {0};
        v_diff[0] = 0.0f;
        for (uint32_t j = 1; j < p_len; j++) {
            v_diff[j] = v_in[i][j] - v_in[i][j-1];
        }

        // 관성 통계 지표 추출 및 오프셋 정렬 (차분 신호 기준)
        _computeStats(v_diff, p_len, p_vibSlot.gyro.rms[i], p_vibSlot.gyro.kurt[i], p_vibSlot.gyro.crest[i], p_vibSlot.gyro.skew[i], p_vibSlot.gyro.std[i]);
        p_vibSlot.gyro.cal_off[i] = p_gyrCfg.offset[i];
        p_vibSlot.gyro.drift_est[i] = 0.0f;

        // FFT 수행 및 파워 변환 (차분 신호 기준)
        memset(_fftWorkIMU, 0, T2_Def::Gyro::Sensor::FFT_SIZE_MAX * 2 * sizeof(float));
        for (uint32_t j = 0; j < p_len; j++) {
            _fftWorkIMU[j * 2] = v_diff[j];
            _fftWorkIMU[j * 2 + 1] = 0.0f;
        }

        // L1 캐시 일관성 보장: CPU 쓰기 후 플러시
        Cache_WriteBack_Addr((uint32_t)_fftWorkIMU, T2_Def::Gyro::Sensor::FFT_SIZE_MAX * 2 * sizeof(float));

        // FFT 연산 수행
        dsps_fft2r_fc32(_fftWorkIMU, p_len);
        // 비트 역순 정렬
        dsps_bit_rev_fc32(_fftWorkIMU, p_len);

        // L1 캐시 일관성 보장: FPU/DMA 연산 후 캐시 무효화
        Cache_Invalidate_Addr((uint32_t)_fftWorkIMU, T2_Def::Gyro::Sensor::FFT_SIZE_MAX * 2 * sizeof(float));

        // 전력 스펙트럼 계산
        uint32_t v_bins = (p_len / 2) + 1;
        float v_invLen = 1.0f / (float)p_len;
        for (uint32_t j = 0; j < v_bins; j++) {
            float re = _fftWorkIMU[j * 2];
            float im = _fftWorkIMU[j * 2 + 1];
            _powerIMU[j] = (re * re + im * im) * v_invLen;
        }

        // 스펙트럼 센트로이드 계산
        p_vibSlot.gyro.centroid[i] = _computeSpectralCentroid(_powerIMU, v_bins, (float)p_sampleRate);

        // 상위 1개 주파수 피크 추출
        T2_Type::ST_SpectralPeak_t v_gyrPeak[1] = {0};
        _extractTopPeaks(_powerIMU, v_bins, (float)p_sampleRate, v_gyrPeak, 1, p_gyrCfg.peak_amp_min, p_gyrCfg.peak_freq_gap_min);
        p_vibSlot.gyro.peak_f[i] = v_gyrPeak[0].freq;

        // DC 성분을 이용한 저주파 영역 드리프트 추정
        float v_dcBins = fmaxf(1.0f, (float)p_sampleRate / 2.0f / (float)v_bins);
        float v_driftSum = 0.0f;
        for (uint32_t j = 0; j < (uint32_t)v_dcBins && j < v_bins; j++) v_driftSum += _powerIMU[j];
        p_vibSlot.gyro.drift_est[i] = v_driftSum / fmaxf(1.0f, v_dcBins);

        // 대역 에너지 계산
        float v_currentBands[T2_Def::Gyro::FeatureLimit::BAND_MAX] = {0};
        _computeBandEnergies(_powerIMU, v_bins, (float)p_sampleRate, p_gyrCfg.active_band_count, p_gyrCfg.band_en, p_gyrCfg.band_start, p_gyrCfg.band_end, v_currentBands);
        memcpy(p_vibSlot.gyro.band_energy[i], v_currentBands, sizeof(v_currentBands));

        // MFCC 계산 및 이력 업데이트
        if constexpr (T2_General::GyroPolicy::enable_mfcc) {
            // MFCC 계산 및 이력 업데이트
            _computeMfcc(_powerIMU, v_bins, p_vibSlot.mfcc, 3 + i, false);
        } else {
            // MFCC 비활성화 시: 자이로 전용 MFCC 영역 초기화
            const uint32_t singleChDim = T2_Def::AI::Tensor::MFCC_COEFFS_DEF * T2_Def::AI::Tensor::MFCC_COMPONENTS_DEF;
            memset(&p_vibSlot.mfcc[(3 + i) * singleChDim], 0, singleChDim * sizeof(float));
            memset(_mfccHistory[3 + i], 0, sizeof(_mfccHistory[3 + i]));
            memset(_deltaHistory[3 + i], 0, sizeof(_deltaHistory[3 + i]));
            _historyCount[3 + i] = 0;
        }
    }
}

/* ============================================================================
 * File: T245_FeatExtra_250.cpp (extractAudio 교정 완료 완전체 블록)
 * ============================================================================ */

void CL_T2_FeatureExtractor::extractAudio(const float* p_audL, const float* p_audR, uint32_t p_len, uint32_t p_sampleRate,
                                          T2_Type::ST_FeatureSlot_Aud_t& p_audSlot, const T2_Type::ST_Audio_Config_t& p_audCfg) {
    if (!_isInitialized) return;

    // 오디오 전용 로컬 버퍼 v_fftLeft, v_fftRight 제거
    uint32_t v_len = (p_len > T2_Def::Audio::Sensor::FFT_SIZE_MAX) ? T2_Def::Audio::Sensor::FFT_SIZE_MAX : p_len;
    uint32_t v_bins = v_len / 2 + 1;

    // FFT 크기 관련 변수
    const float v_invLen = 1.0f / (float)v_len;
    int v_tolSamples = (int)(T2_Def::Audio::FeatureLimit::CEPS_TOLERANCE_DEF * p_sampleRate);
    if (v_tolSamples < 1) v_tolSamples = 1;

    // 1/3 옥타브 밴드 누적용 파워 스펙트럼 배열
    float v_powerAccum[BINS_MAX] = {0};

    // 채널 활성화/비활성화
    bool hasLeft  = ((uint8_t)p_audCfg.channel_mask & (uint8_t)T2_Type::EM_ChannelMask_t::CH_LEFT);
    bool hasRight = ((uint8_t)p_audCfg.channel_mask & (uint8_t)T2_Type::EM_ChannelMask_t::CH_RIGHT);

    // LEFT Channel 연산
    if (p_audL && hasLeft) {
        // LEFT RMS, Kurt, Crest, Skew, Std 계산
        float v_unusedStdL = 0.0f;
        _computeStats(p_audL, v_len, p_audSlot.audio.ch[0].rms, p_audSlot.audio.ch[0].kurt, p_audSlot.audio.ch[0].crest, p_audSlot.audio.ch[0].skew, v_unusedStdL);
        // LEFT 파워 계산
        p_audSlot.audio.ch[0].energy = p_audSlot.audio.ch[0].rms * p_audSlot.audio.ch[0].rms;

        // LEFT RMS 미분 가속도 연산
        float currentRms = p_audSlot.audio.ch[0].rms;
        if (_prevRms[0] > 0.00001f) {
            p_audSlot.audio.ch[0].d_rms = currentRms - _prevRms[0];
            p_audSlot.audio.ch[0].dd_rms = p_audSlot.audio.ch[0].d_rms - _prevDRms[0];
            _prevDRms[0] = p_audSlot.audio.ch[0].d_rms;
        } else {
            p_audSlot.audio.ch[0].d_rms = 0.0f; p_audSlot.audio.ch[0].dd_rms = 0.0f; _prevDRms[0] = 0.0f;
        }
        _prevRms[0] = currentRms;

        // L1 캐시 일관성 보장: 무효화
        Cache_Invalidate_Addr((uint32_t)_fftWorkAudio, T2_Def::Audio::Sensor::FFT_SIZE_MAX * 2 * sizeof(float));

        // 샘플을 FFT 입력 버퍼로 복사
        for (uint32_t i = 0; i < v_len; i++) {
            _fftWorkAudio[i * 2] = p_audL[i] * _windowAudio[i];
            _fftWorkAudio[i * 2 + 1] = 0.0f;
        }

        // L1 캐시 일관성 보장: 플러시
        Cache_WriteBack_Addr((uint32_t)_fftWorkAudio, T2_Def::Audio::Sensor::FFT_SIZE_MAX * 2 * sizeof(float));

        // FFT 연산 수행
        dsps_fft2r_fc32(_fftWorkAudio, v_len);
        // 비트 역순 정렬
        dsps_bit_rev_fc32(_fftWorkAudio, v_len);

        // L1 캐시 일관성 보장: 무효화
        Cache_Invalidate_Addr((uint32_t)_fftWorkAudio, T2_Def::Audio::Sensor::FFT_SIZE_MAX * 2 * sizeof(float));

        // 전력 스펙트럼 계산
        for (uint32_t i = 0; i < v_bins; i++) {
            float re = _fftWorkAudio[i * 2]; float im = _fftWorkAudio[i * 2 + 1];
            _powerAudio[i] = (re * re + im * im) / v_len;
        }

        // 배경 소음 노이즈 학습 필터링 수식 연동
        if (_isLearning) {
            // 학습 파라미터
            float v_alpha = T2_Def::Audio::Dsp::NOISE_LEARN_ALPHA_DEF;
            // 노이즈 프로파일 갱신
            for (uint32_t i = 0; i < v_bins; i++) {
                _noiseProfile[i] = (v_alpha * _powerAudio[i]) + ((1.0f - v_alpha) * _noiseProfile[i]);
            }
            // 학습 프레임 카운트 증가
            if (_learnedFrames < UINT32_MAX) _learnedFrames++;
        } else if (_learnedFrames > 0) { // 노이즈 학습이 되어 있을 경우
            // 노이즈 제거 강도
            float v_subStr = p_audCfg.noise.sub_str;
            // 노이즈 제거 수행
            for (uint32_t i = 0; i < v_bins; i++) {
                _powerAudio[i] = fmaxf(_powerAudio[i] - (_noiseProfile[i] * v_subStr), T2_Def::Global::System::MATH_EPSILON_12_CONST);
            }
        }

        // 주파수 분석을 위해 전력 스펙트럼을 누적
        for (uint32_t i = 0; i < v_bins; i++) {
            v_powerAccum[i] += _powerAudio[i];
        }

        // 스펙트럼 중심 계산
        p_audSlot.audio.ch[0].centroid = _computeSpectralCentroid(_powerAudio, v_bins, (float)p_sampleRate);

        // 피크 개수 결정
        uint8_t v_peakCnt = p_audCfg.active_peak_count;
        if (v_peakCnt == 0 || v_peakCnt > T2_Def::Audio::FeatureLimit::TOP_PEAKS_MAX) v_peakCnt = T2_Def::Audio::FeatureLimit::TOP_PEAKS_DEF;
        // 피크 추출
        _extractTopPeaks(_powerAudio, v_bins, (float)p_sampleRate, p_audSlot.audio.ch[0].top_peaks, v_peakCnt, p_audCfg.peak_amp_min, p_audCfg.peak_freq_gap_min);

        // 밴드 에너지 계산
        _computeBandEnergies(_powerAudio, v_bins, (float)p_sampleRate, p_audCfg.active_band_count, p_audCfg.band_en, p_audCfg.band_start, p_audCfg.band_end, p_audSlot.audio.ch[0].band_energy);

        // 켑스트럼 변환
        Cache_Invalidate_Addr((uint32_t)_cepsIfftWork, T2_Def::Audio::Sensor::FFT_SIZE_MAX * 2 * sizeof(float));
        // 1. 시간축 데이터 초기화 및 로그 스케일링/대칭 복사
        for (uint32_t i = 0; i < v_len; i++) { _cepsIfftWork[i * 2] = 0.0f; _cepsIfftWork[i * 2 + 1] = 0.0f; }

        // 로그 스케일링 및 대칭 복사
        for (uint32_t i = 0; i < v_bins; i++) {
            float v_val = log10f(fmaxf(_powerAudio[i], T2_Def::Global::System::MATH_EPSILON_12_CONST));
            _cepsIfftWork[i * 2] = v_val;
            if (i > 0 && i < v_bins - 1) _cepsIfftWork[(v_len - i) * 2] = v_val;
        }
        // 허미션 정렬(대칭 데이터에 음수 부호 부여)
        for (uint32_t i = 0; i < v_len; i++) _cepsIfftWork[i * 2 + 1] = -_cepsIfftWork[i * 2 + 1];

        // L1 캐시 플러시
        Cache_WriteBack_Addr((uint32_t)_cepsIfftWork, T2_Def::Audio::Sensor::FFT_SIZE_MAX * 2 * sizeof(float));
        // IFFT 변환 수행
        dsps_fft2r_fc32(_cepsIfftWork, v_len);
        // 비트 역순 정렬
        dsps_bit_rev_fc32(_cepsIfftWork, v_len);
        // L1 캐시 무효화
        Cache_Invalidate_Addr((uint32_t)_cepsIfftWork, T2_Def::Audio::Sensor::FFT_SIZE_MAX * 2 * sizeof(float));

        // 스케일링 및 허미션 복원
        for (uint32_t i = 0; i < v_len; i++) { _cepsIfftWork[i * 2] *= v_invLen; _cepsIfftWork[i * 2 + 1] *= -v_invLen; }

        // 켑스트럼 계수 추출
        for (uint8_t c = 0; c < p_audCfg.active_ceps_count; c++) {
            float v_targetSec = p_audCfg.ceps_targets[c];
            // 유효한 목표치인지 확인
            if (v_targetSec < T2_Def::Global::System::MATH_EPSILON_CONST) {
                p_audSlot.audio.ch[0].cpsr_max[c] = 0.0f; p_audSlot.audio.ch[0].cpsr_mxr[c] = 1.0f; continue;
            }
            // 목표치를 인덱스로 변환
            int v_targetIdx = (int)(v_targetSec * p_sampleRate);
            // 유효 범위 확인
            if (v_targetIdx >= (int)v_len) v_targetIdx = v_len - 1;

            // 탐색 범위 계산
            int v_startIdx = v_targetIdx - v_tolSamples; int v_endIdx = v_targetIdx + v_tolSamples;
            if (v_startIdx < 0) v_startIdx = 0; if (v_endIdx >= (int)v_len) v_endIdx = v_len - 1;

            // 주변 영역의 최대값(Peak) 탐색
            float v_maxVal = -1e9f;
            for (int k = v_startIdx; k <= v_endIdx; k++) {
                float val = fabsf(_cepsIfftWork[k * 2]);
                if (val > v_maxVal) v_maxVal = val;
            }
            // 스펙트럼 최대값 저장
            p_audSlot.audio.ch[0].cpsr_max[c] = v_maxVal;

            // 배경 소음 수준 계산
            float v_noiseSum = 0.0f; int v_noiseCnt = 0;
            const int v_margin = T2_Def::Audio::FeatureLimit::CEPS_NOISE_MARGIN_CONST;
            for (int k = v_startIdx - v_margin; k <= v_endIdx + v_margin; k++) {
                if (k >= 0 && k < (int)v_len && (k < v_startIdx || k > v_endIdx)) {
                    v_noiseSum += fabsf(_cepsIfftWork[k * 2]); v_noiseCnt++;
                }
            }
            // 배경 노이즈 평균 계산
            float v_noiseMean = (v_noiseCnt > 0) ? (v_noiseSum / v_noiseCnt) : 1e-6f;
            if (v_noiseMean < 1e-6f) v_noiseMean = 1e-6f;
            // 노이즈 대비 최대값 비율 저장
            p_audSlot.audio.ch[0].cpsr_mxr[c] = v_maxVal / v_noiseMean;
        }

        // MFCC 계산 (8채널 중 7번 슬롯)
        _computeMfcc(_powerAudio, v_bins, p_audSlot.mfcc, 7, true);
    }

    // ============================================================
    // RIGHT Channel 연산 (히스토리 채널 ID = 7)
    // ============================================================
    if (p_audR && hasRight) {
        float v_unusedStdR = 0.0f;
        _computeStats(p_audR, v_len, p_audSlot.audio.ch[1].rms, p_audSlot.audio.ch[1].kurt, p_audSlot.audio.ch[1].crest, p_audSlot.audio.ch[1].skew, v_unusedStdR);
        p_audSlot.audio.ch[1].energy = p_audSlot.audio.ch[1].rms * p_audSlot.audio.ch[1].rms;

        // RMS 변위 (미분) 연산
        float currentRms = p_audSlot.audio.ch[1].rms;
        if (_prevRms[1] > 0.00001f) {
            p_audSlot.audio.ch[1].d_rms = currentRms - _prevRms[1];
            p_audSlot.audio.ch[1].dd_rms = p_audSlot.audio.ch[1].d_rms - _prevDRms[1];
            _prevDRms[1] = p_audSlot.audio.ch[1].d_rms;
        } else {
            p_audSlot.audio.ch[1].d_rms = 0.0f; p_audSlot.audio.ch[1].dd_rms = 0.0f; _prevDRms[1] = 0.0f;
        }
        _prevRms[1] = currentRms;

        // L1 캐시 일관성 보장: 무효화
        Cache_Invalidate_Addr((uint32_t)_fftWorkAudio, T2_Def::Audio::Sensor::FFT_SIZE_MAX * 2 * sizeof(float));

        for (uint32_t i = 0; i < v_len; i++) {
            _fftWorkAudio[i * 2] = p_audR[i] * _windowAudio[i];
            _fftWorkAudio[i * 2 + 1] = 0.0f;
        }

        // L1 캐시 일관성 보장: 플러시
        Cache_WriteBack_Addr((uint32_t)_fftWorkAudio, T2_Def::Audio::Sensor::FFT_SIZE_MAX * 2 * sizeof(float));

        dsps_fft2r_fc32(_fftWorkAudio, v_len);
        dsps_bit_rev_fc32(_fftWorkAudio, v_len);

        // L1 캐시 일관성 보장: 무효화
        Cache_Invalidate_Addr((uint32_t)_fftWorkAudio, T2_Def::Audio::Sensor::FFT_SIZE_MAX * 2 * sizeof(float));

        for (uint32_t i = 0; i < v_bins; i++) {
            float re = _fftWorkAudio[i * 2]; float im = _fftWorkAudio[i * 2 + 1];
            _powerAudio[i] = (re * re + im * im) / v_len;
        }

        if (_isLearning) {
            float v_alpha = T2_Def::Audio::Dsp::NOISE_LEARN_ALPHA_DEF;
            for (uint32_t i = 0; i < v_bins; i++) {
                _noiseProfile[BINS_PADDED + i] = (v_alpha * _powerAudio[i]) + ((1.0f - v_alpha) * _noiseProfile[BINS_PADDED + i]);
            }
        } else if (_learnedFrames > 0) {
            float v_subStr = p_audCfg.noise.sub_str;
            for (uint32_t i = 0; i < v_bins; i++) {
                _powerAudio[i] = fmaxf(_powerAudio[i] - (_noiseProfile[BINS_PADDED + i] * v_subStr), T2_Def::Global::System::MATH_EPSILON_12_CONST);
            }
        }

        // 파워 스펙트럼 누적
        for (uint32_t i = 0; i < v_bins; i++) {
            v_powerAccum[i] += _powerAudio[i];
        }

        p_audSlot.audio.ch[1].centroid = _computeSpectralCentroid(_powerAudio, v_bins, (float)p_sampleRate);
        uint8_t v_peakCnt = p_audCfg.active_peak_count;
        if (v_peakCnt == 0 || v_peakCnt > T2_Def::Audio::FeatureLimit::TOP_PEAKS_MAX) v_peakCnt = T2_Def::Audio::FeatureLimit::TOP_PEAKS_DEF;
        _extractTopPeaks(_powerAudio, v_bins, (float)p_sampleRate, p_audSlot.audio.ch[1].top_peaks, v_peakCnt, p_audCfg.peak_amp_min, p_audCfg.peak_freq_gap_min);
        _computeBandEnergies(_powerAudio, v_bins, (float)p_sampleRate, p_audCfg.active_band_count, p_audCfg.band_en, p_audCfg.band_start, p_audCfg.band_end, p_audSlot.audio.ch[1].band_energy);

        // ============================================================
        // [우채널 켑스트럼 연산]
        // ============================================================
        Cache_Invalidate_Addr((uint32_t)_cepsIfftWork, T2_Def::Audio::Sensor::FFT_SIZE_MAX * 2 * sizeof(float));
        for (uint32_t i = 0; i < v_len; i++) { _cepsIfftWork[i * 2] = 0.0f; _cepsIfftWork[i * 2 + 1] = 0.0f; }
        for (uint32_t i = 0; i < v_bins; i++) {
            float v_val = log10f(fmaxf(_powerAudio[i], T2_Def::Global::System::MATH_EPSILON_12_CONST));
            _cepsIfftWork[i * 2] = v_val;
            if (i > 0 && i < v_bins - 1) _cepsIfftWork[(v_len - i) * 2] = v_val;
        }
        for (uint32_t i = 0; i < v_len; i++) _cepsIfftWork[i * 2 + 1] = -_cepsIfftWork[i * 2 + 1];

        Cache_WriteBack_Addr((uint32_t)_cepsIfftWork, T2_Def::Audio::Sensor::FFT_SIZE_MAX * 2 * sizeof(float));
        dsps_fft2r_fc32(_cepsIfftWork, v_len);
        dsps_bit_rev_fc32(_cepsIfftWork, v_len);
        Cache_Invalidate_Addr((uint32_t)_cepsIfftWork, T2_Def::Audio::Sensor::FFT_SIZE_MAX * 2 * sizeof(float));

        for (uint32_t i = 0; i < v_len; i++) { _cepsIfftWork[i * 2] *= v_invLen; _cepsIfftWork[i * 2 + 1] *= -v_invLen; }

        for (uint8_t c = 0; c < p_audCfg.active_ceps_count; c++) {
            float v_targetSec = p_audCfg.ceps_targets[c];
            if (v_targetSec < T2_Def::Global::System::MATH_EPSILON_CONST) {
                p_audSlot.audio.ch[1].cpsr_max[c] = 0.0f; p_audSlot.audio.ch[1].cpsr_mxr[c] = 1.0f; continue;
            }
            int v_targetIdx = (int)(v_targetSec * p_sampleRate);
            if (v_targetIdx >= (int)v_len) v_targetIdx = v_len - 1;

            int v_startIdx = v_targetIdx - v_tolSamples; int v_endIdx = v_targetIdx + v_tolSamples;
            if (v_startIdx < 0) v_startIdx = 0; if (v_endIdx >= (int)v_len) v_endIdx = v_len - 1;

            float v_maxVal = -1e9f;
            for (int k = v_startIdx; k <= v_endIdx; k++) {
                float val = fabsf(_cepsIfftWork[k * 2]);
                if (val > v_maxVal) v_maxVal = val;
            }
            p_audSlot.audio.ch[1].cpsr_max[c] = v_maxVal;

            float v_noiseSum = 0.0f; int v_noiseCnt = 0;
            const int v_margin = T2_Def::Audio::FeatureLimit::CEPS_NOISE_MARGIN_CONST;
            for (int k = v_startIdx - v_margin; k <= v_endIdx + v_margin; k++) {
                if (k >= 0 && k < (int)v_len && (k < v_startIdx || k > v_endIdx)) {
                    v_noiseSum += fabsf(_cepsIfftWork[k * 2]); v_noiseCnt++;
                }
            }
            float v_noiseMean = (v_noiseCnt > 0) ? (v_noiseSum / v_noiseCnt) : 1e-6f;
            if (v_noiseMean < 1e-6f) v_noiseMean = 1e-6f;
            p_audSlot.audio.ch[1].cpsr_mxr[c] = v_maxVal / v_noiseMean;
        }

        _computeMfcc(_powerAudio, v_bins, p_audSlot.mfcc, 7, true);
    }

    // 1/3 옥타브 밴드 상대 에너지(Timbre 지표) 계산 & Coherence/IPD 계산 삭제
    float v_timbreBands[32] = {0};
    float v_timbreSum = 0.0f;
    uint32_t mapOffset = 0;
    if (v_len == 2048) mapOffset = 64;
    else if (v_len == 4096) mapOffset = 128;

    for (int b = 0; b < 32; b++) {
        uint8_t startBin = g_T2_40_Dsp_BandBinMap_arr[mapOffset + b * 2];
        uint8_t endBin = g_T2_40_Dsp_BandBinMap_arr[mapOffset + b * 2 + 1];

        float bandSum = 0.0f;
        for (uint32_t j = startBin; j <= endBin && j < v_bins; j++) {
            bandSum += v_powerAccum[j];
        }
        v_timbreBands[b] = bandSum;
        v_timbreSum += bandSum;
    }

    float invTimbreSum = 1.0f / fmaxf(v_timbreSum, T2_Def::Global::System::MATH_EPSILON_CONST);
    for (int b = 0; b < 32; b++) {
        p_audSlot.audio.timbre_bands[b] = v_timbreBands[b] * invTimbreSum;
    }
}


void CL_T2_FeatureExtractor::_computeStats(const float* p_data, uint32_t p_len, float& p_rms, float& p_kurt, float& p_crest, float& p_skew, float& p_std) {
    if (p_len == 0) {
        p_rms = 0.0f; p_kurt = 0.0f; p_crest = 0.0f; p_skew = 0.0f; p_std = 0.0f;
        return;
    }

    // 온라인 알고리즘을 사용한 통계량 계산
    double mean = 0.0;
    double M2 = 0.0, M3 = 0.0, M4 = 0.0;
    float v_sqSum = 0.0f;
    float v_maxAbs = 0.0f;

    // Welford's online algorithm을 이용한 2, 3, 4차 중심모멘트 계산
    for (uint32_t i = 0; i < p_len; i++) {
        float val = p_data[i];
        v_sqSum += val * val;
        float absVal = fabsf(val);
        if (absVal > v_maxAbs) v_maxAbs = absVal;

        double delta = val - mean;
        double delta_n = delta / (i + 1);
        double delta_n2 = delta_n * delta_n;
        double term1 = delta * delta_n * i;

        mean += delta_n;
        M4 += term1 * delta_n2 * (i * i - i + 1) + 6.0 * delta_n2 * M2 - 4.0 * delta_n * M3;
        M3 += term1 * delta_n * (i - 2.0) - 3.0 * delta_n * M2;
        M2 += term1;
    }

    // RMS 계산
    p_rms = SMEA_SAN_FLOAT(sqrtf(v_sqSum / p_len));
    // 분산 계산
    float v_var = (float)(M2 / p_len);
    // 표준편차 계산
    p_std = SMEA_SAN_FLOAT(sqrtf(v_var));

    // 편차 제곱근의 평균을 위한 분모 계산
    float v_denom = fmaxf(p_std * p_std, T2_Def::Global::System::MATH_EPSILON_12_CONST);
    // 왜도 계산
    p_skew = (p_std > T2_Def::Global::System::MATH_EPSILON_12_CONST) ? (float)(M3 / p_len) / (v_denom * p_std) : 0.0f;
    // 왜도 포화 처리
    p_skew = SMEA_SAN_FLOAT(p_skew);

    // 첨도(Kurtosis) 계산
    p_kurt = SMEA_SAN_FLOAT((float)(M4 / p_len) / (v_denom * v_denom));
    // 첨도 포화 처리
    p_kurt = SMEA_SAN_FLOAT(p_kurt);

    // Crest Factor 계산
    p_crest = SMEA_SAN_FLOAT(v_maxAbs / fmaxf(p_rms, T2_Def::Global::System::MATH_EPSILON_12_CONST));
    // Crest Factor 포화 처리
    p_crest = SMEA_SAN_FLOAT(p_crest);
}

// 스펙트럼 중심(Centroid) 계산
float CL_T2_FeatureExtractor::_computeSpectralCentroid(const float* p_power, uint32_t p_bins, float p_sampleRate) {
    float v_num = 0, v_den = 0;
    for (uint32_t i = 0; i < p_bins; i++) {
        float freq = i * (p_sampleRate / 2.0f) / (p_bins - 1);
        v_num += freq * p_power[i];
        v_den += p_power[i];
    }
    return SMEA_SAN_FLOAT(v_num / fmaxf(v_den, T2_Def::Global::System::MATH_EPSILON_CONST));
}

// 스펙트럼 피크 추출 (Peak Picking)
void CL_T2_FeatureExtractor::_extractTopPeaks(const float* p_power, uint32_t p_bins, float p_sampleRate, T2_Type::ST_SpectralPeak_t* p_outPeaks, uint8_t p_maxCount, float p_ampMin, float p_freqGap) {
    struct Peak { float f; float a; };
    constexpr uint16_t MAX_CANDIDATES = 128;
    Peak v_cands[MAX_CANDIDATES];
    uint16_t v_candCount = 0;

    // 이웃보다 크고(Local Maxima) 최소 진폭 이상인 주파수 성분 후보 탐색
    for (uint32_t i = 1; i < p_bins - 1; i++) {

        if (p_power[i] > p_power[i-1] && p_power[i] > p_power[i+1] && p_power[i] > p_ampMin) {
            // 주파수 계산
            float freq = i * (p_sampleRate / 2.0f) / (p_bins - 1);

            // 정렬 상태 유지하며 삽입
            uint16_t insertIdx = v_candCount;
            while (insertIdx > 0 && v_cands[insertIdx - 1].a < p_power[i]) {
                if (insertIdx < MAX_CANDIDATES) {
                    v_cands[insertIdx] = v_cands[insertIdx - 1];
                }
                insertIdx--;
            }
            // 유효 범위 내 삽입
            if (insertIdx < MAX_CANDIDATES) {
                v_cands[insertIdx] = {freq, p_power[i]};
                if (v_candCount < MAX_CANDIDATES) v_candCount++;
            }
        }
    }

    // 주파수 간격(Gap) 필터링
    uint8_t count = 0;
    for (uint16_t i = 0; i < v_candCount; i++) {
        // 최대 개수 확인
        if (count >= p_maxCount) break;
        bool tooClose = false;
        // 이미 선택된 피크들과의 간격 확인
        for (int j = 0; j < count; j++) {
            if (std::abs(v_cands[i].f - p_outPeaks[j].freq) < p_freqGap) {
                tooClose = true;
                break;
            }
        }
        // 유효 간격 확인 시 피크 선택
        if (!tooClose) {
            p_outPeaks[count].freq = v_cands[i].f;
            p_outPeaks[count].amp = v_cands[i].a;
            count++;
        }
    }
    // 나머지 공간 초기화
    for (; count < p_maxCount; count++) {
        p_outPeaks[count].freq = 0.0f;
        p_outPeaks[count].amp = 0.0f;
    }
}

// 밴드 에너지 계산
void CL_T2_FeatureExtractor::_computeBandEnergies(const float* p_power, uint32_t p_bins, float p_sampleRate, uint8_t p_count, const std::bitset<16>& p_en, const float* p_start, const float* p_end, float* p_outEnergies) {
    // 주파수 해상도(Hz/Bin)
    float v_binHz = (p_sampleRate / 2.0f) / (p_bins - 1);

    uint8_t v_limit = std::min((int)p_count, (int)T2_Def::Accel::FeatureLimit::BAND_MAX);
    for (uint8_t i = 0; i < v_limit; i++) {
        // 밴드 활성화 체크
        if (!p_en[i]) {
            p_outEnergies[i] = 0.0f;
            continue;
        }
        // 주파수 구간의 시작/끝 인덱스 계산
        uint32_t v_startIdx = (uint32_t)(p_start[i] / v_binHz);
        uint32_t v_endIdx   = (uint32_t)(p_end[i] / v_binHz);
        // 경계 조건 처리
        if (v_startIdx >= p_bins) v_startIdx = p_bins - 1;
        if (v_endIdx >= p_bins)   v_endIdx = p_bins - 1;

        // 밴드 에너지 합산
        float v_sum = 0;
        for (uint32_t j = v_startIdx; j <= v_endIdx; j++) {
            v_sum += p_power[j];
        }
        // 밴드 에너지 저장
        p_outEnergies[i] = v_sum;
    }
}

// MFCC 계산
void CL_T2_FeatureExtractor::_computeMfcc(const float* p_power, uint32_t p_bins, float* p_outMfcc, uint8_t p_chIdx, bool isAudio) {
    if (p_chIdx >= 8) return;

    // 4배수 패딩
    const uint16_t binPadded = (p_bins + 3) & ~3;

    // Mel 필터뱅크 결과
    alignas(16) float v_melEnergies[MEL_PADDED] = {0};
    // Mel 필터뱅크 행렬 및 DCT 행렬 포인터
    float* melBank = isAudio ? _melBankFlat : _melBankIMU;
    float* dctMat  = isAudio ? _dctMatrixFlat : _dctMatrixIMU;

    // SIMD 가속화 행렬 곱셈 1회 수행
    dspm_mult_f32(p_power, melBank, v_melEnergies, 1, binPadded, MEL_PADDED);

    // 오디오/IMU 공용 활성 멜 밴드 수
    const uint8_t activeBands = isAudio ? _activeMelBands : MEL_BANDS_MAX;
    // 로그 에너지 계산
    for (int i = 0; i < activeBands; i++) {
        float v_energySanitized = SMEA_SAN_FLOAT(v_melEnergies[i]);
        v_melEnergies[i] = log10f(fmaxf(v_energySanitized, T2_Def::Global::System::MATH_EPSILON_12_CONST));
    }
    // 나머지 공간 초기화
    for (int i = activeBands; i < MEL_PADDED; i++) {
        v_melEnergies[i] = 0.0f;
    }

    // DCT 변환 행렬 곱셈
    alignas(16) float v_dctOut[MFCC_PADDED] = {0};
    dspm_mult_f32(v_melEnergies, dctMat, v_dctOut, 1, MEL_PADDED, MFCC_PADDED);

    // MFCC 차원
    uint16_t v_dim = T2_Def::AI::Tensor::MFCC_COEFFS_DEF;
    // 채널별 히스토리 카운트
    uint16_t& v_chCnt = _historyCount[p_chIdx];

    // 순환 버퍼 인덱스 갱신
    uint8_t v_currHead = _historyHead[p_chIdx];
    uint8_t v_nextHead = (v_currHead + 1) % T2_Def::AI::Tensor::DELTA_HISTORY_MAX;
    _historyHead[p_chIdx] = v_nextHead;

    // MFCC 이력 저장
    memcpy(_mfccHistory[p_chIdx][v_nextHead], v_dctOut, v_dim * sizeof(float));
    if (v_chCnt < T2_Def::AI::Tensor::DELTA_HISTORY_MAX) {
        v_chCnt++;
    }

    // 단일 채널 차원
    const uint32_t singleChDim = T2_Def::AI::Tensor::MFCC_COEFFS_DEF * T2_Def::AI::Tensor::MFCC_COMPONENTS_DEF;
    // 출력 포인터 설정
    float* v_finalOut = isAudio ? &p_outMfcc[(p_chIdx - 6) * singleChDim] : &p_outMfcc[p_chIdx * singleChDim];

    // 정적 컴포넌트 복사
    memcpy(v_finalOut, v_dctOut, v_dim * sizeof(float));

    // 델타 및 델타-델타 계산
    if (v_chCnt >= T2_Def::AI::Tensor::DELTA_HISTORY_MAX) {
        // 가장 오래된 원소 인덱스
        uint8_t v_oldestHead = (v_nextHead + 1) % T2_Def::AI::Tensor::DELTA_HISTORY_MAX;
        // 시간 간격 역수
        float v_invDeltaGap = 1.0f / (float)T2_Def::AI::Tensor::DELTA_GAP_CONST;

        // Delta 계수 계산
        for (int i = 0; i < v_dim; i++) {
            v_finalOut[v_dim + i] = (_mfccHistory[p_chIdx][v_nextHead][i] - _mfccHistory[p_chIdx][v_oldestHead][i]) * v_invDeltaGap;
        }

        // Delta 계수 이력 저장
        memcpy(_deltaHistory[p_chIdx][v_nextHead], &v_finalOut[v_dim], v_dim * sizeof(float));

        // Delta-Delta 계수 계산
        for (int i = 0; i < v_dim; i++) {
            v_finalOut[v_dim * 2 + i] = (_deltaHistory[p_chIdx][v_nextHead][i] - _deltaHistory[p_chIdx][v_oldestHead][i]) * v_invDeltaGap;
        }
    } else {
        // 델타 및 델타-델타 초기화
        memset(&v_finalOut[v_dim], 0, v_dim * 2 * sizeof(float));
    }
}

void CL_T2_FeatureExtractor::setNoiseLearning(bool p_enable) {
    _isLearning = p_enable;
    if (_isLearning) _learnedFrames = 0;
}

void CL_T2_FeatureExtractor::reloadAudioMelFilter(float sampleRate) {
    if (!_melBankFlat) {
        ESP_LOGE(TAG, "reloadAudioMelFilter: _melBankFlat is null");
        return;
    }

    const float nyquist    = sampleRate / 2.0f;

    // Mel 뱅크 초기화
    memset(_melBankFlat, 0, BINS_PADDED * MEL_PADDED * sizeof(float));

    MelFilterbankGenerator::generateAudioMelFilterbank(_melBankFlat, sampleRate, _activeMelBands, BINS_MAX, MEL_PADDED, MEL_BANDS_MAX);

    // DCT Matrix는 주파수 축과 무관하므로 재생성 불필요
    ESP_LOGI(TAG, "Audio Mel Filterbank reloaded (SR: %.0f Hz, Nyquist: %.0f Hz, Bins: %d)",
             sampleRate, nyquist, BINS_MAX);
}



void CL_T2_FeatureExtractor::resetNoiseProfile() {
    if (_noiseProfile) memset(_noiseProfile, 0, 2 * BINS_PADDED * sizeof(float));
    _learnedFrames = 0;
}

void CL_T2_FeatureExtractor::resetHistory() {
    memset(_historyCount, 0, sizeof(_historyCount));
    memset(_historyHead, 0, sizeof(_historyHead));
    memset(_mfccHistory, 0, sizeof(float) * 8 * T2_Def::AI::Tensor::DELTA_HISTORY_MAX * MFCC_PADDED);
    memset(_deltaHistory, 0, sizeof(float) * 8 * T2_Def::AI::Tensor::DELTA_HISTORY_MAX * MFCC_PADDED);
}
