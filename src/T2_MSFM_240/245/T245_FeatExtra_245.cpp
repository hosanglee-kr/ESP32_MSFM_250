/* ============================================================================
 * File: T245_FeatExtra_245.cpp
 * Summary: 멀티모달(진동/소음) 융합 특징량 추출 엔진 구현부
 * ============================================================================ */

#include "T245_FeatExtra_245.hpp"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "dspm_mult.h"
#include <cmath>
#include <algorithm>
#include <cstring>

static const char* TAG = "T245_FEAT";

/**
 * @brief CL_T2_FeatureExtractor 생성자
 */
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

/**
 * @brief CL_T2_FeatureExtractor 소멸자
 */
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

/**
 * @brief 특징량 추출을 위한 메모리 할당 및 변환 행렬 초기 구성
 * @param audioCfg 오디오 채널 설정 정보
 * @return 초기화 성공 여부
 */
bool CL_T2_FeatureExtractor::init(const T2_Type::ST_Audio_Config_t& audioCfg) {
    _activeMelBands = audioCfg.mel_bands;
    if (_activeMelBands == 0 || _activeMelBands > MEL_BANDS_MAX) {
        _activeMelBands = T2_Def::Audio::FeatureLimit::MEL_BANDS_DEF;
    }

    // [처리 단위 1] 오디오용 스펙트럼 및 DCT 버퍼 PSRAM 할당
    _melBankFlat   = (float*)heap_caps_aligned_alloc(16, BINS_PADDED * MEL_PADDED * sizeof(float), MALLOC_CAP_SPIRAM);
    _dctMatrixFlat = (float*)heap_caps_aligned_alloc(16, MEL_PADDED * MFCC_PADDED * sizeof(float), MALLOC_CAP_SPIRAM);
    _fftWorkAudio  = (float*)heap_caps_aligned_alloc(16, T2_Def::Audio::Sensor::FFT_SIZE_MAX * 2 * sizeof(float), MALLOC_CAP_SPIRAM);
    _powerAudio    = (float*)heap_caps_aligned_alloc(16, BINS_PADDED * sizeof(float), MALLOC_CAP_SPIRAM);
    _windowAudio   = (float*)heap_caps_aligned_alloc(16, T2_Def::Audio::Sensor::FFT_SIZE_MAX * sizeof(float), MALLOC_CAP_SPIRAM);
    _noiseProfile  = (float*)heap_caps_aligned_alloc(16, 2 * BINS_PADDED * sizeof(float), MALLOC_CAP_SPIRAM);

    // [처리 단위 2] 가속도/자이로(IMU)용 고속 연산 버퍼 할당 (SRAM 및 PSRAM 구분)
    _fftWorkIMU    = (float*)heap_caps_aligned_alloc(16, T2_Def::Accel::Sensor::FFT_SIZE_MAX * 2 * sizeof(float), MALLOC_CAP_INTERNAL);
    _powerIMU      = (float*)heap_caps_aligned_alloc(16, T2_Def::Accel::Sensor::FFT_SIZE_MAX * sizeof(float), MALLOC_CAP_INTERNAL);
    _cepsIfftWork  = (float*)heap_caps_aligned_alloc(16, T2_Def::Audio::Sensor::FFT_SIZE_MAX * 2 * sizeof(float), MALLOC_CAP_SPIRAM);

    _melBankIMU    = (float*)heap_caps_aligned_alloc(16, BINS_IMU_PADDED * MEL_PADDED * sizeof(float), MALLOC_CAP_SPIRAM);
    _dctMatrixIMU  = (float*)heap_caps_aligned_alloc(16, MEL_PADDED * MFCC_PADDED * sizeof(float), MALLOC_CAP_SPIRAM);

    // [처리 단위 3] Delta/Delta-Delta 특징 누적을 위한 채널 히스토리 버퍼 할당
    _mfccHistory   = (float (*)[T2_Def::AI::Tensor::DELTA_HISTORY_MAX][MFCC_PADDED])heap_caps_aligned_alloc(
                         16, sizeof(float) * 8 * T2_Def::AI::Tensor::DELTA_HISTORY_MAX * MFCC_PADDED, MALLOC_CAP_SPIRAM);
    _deltaHistory  = (float (*)[T2_Def::AI::Tensor::DELTA_HISTORY_MAX][MFCC_PADDED])heap_caps_aligned_alloc(
                         16, sizeof(float) * 8 * T2_Def::AI::Tensor::DELTA_HISTORY_MAX * MFCC_PADDED, MALLOC_CAP_SPIRAM);

    if (!_melBankFlat || !_dctMatrixFlat || !_fftWorkAudio || !_powerAudio || !_windowAudio || !_noiseProfile ||
        !_fftWorkIMU || !_powerIMU || !_cepsIfftWork ||
        !_melBankIMU || !_dctMatrixIMU || !_mfccHistory || !_deltaHistory) {
        ESP_LOGE(TAG, "Memory Allocation Failed");
        return false;
    }

    // [처리 단위 4] 임시 버퍼들 0으로 초기화
    memset(_mfccHistory,   0, sizeof(float) * 8 * T2_Def::AI::Tensor::DELTA_HISTORY_MAX * MFCC_PADDED);
    memset(_deltaHistory,  0, sizeof(float) * 8 * T2_Def::AI::Tensor::DELTA_HISTORY_MAX * MFCC_PADDED);
    memset(_historyCount,  0, sizeof(_historyCount));
    memset(_historyHead,   0, sizeof(_historyHead));
    memset(_melBankFlat,   0, BINS_PADDED * MEL_PADDED * sizeof(float));
    memset(_melBankIMU,    0, BINS_IMU_PADDED * MEL_PADDED * sizeof(float));
    memset(_dctMatrixFlat, 0, MEL_PADDED * MFCC_PADDED * sizeof(float));
    memset(_dctMatrixIMU,  0, MEL_PADDED * MFCC_PADDED * sizeof(float));
    memset(_noiseProfile,  0, 2 * BINS_PADDED * sizeof(float));
    
    // 오디오 Hann 윈도우 사전 계산 적용
    dsps_wind_hann_f32(_windowAudio, T2_Def::Audio::Sensor::FFT_SIZE_MAX);

    // [처리 단위 5] IMU 전용 Mel Filterbank 생성 (Nyquist 800Hz 고정 대역)
    {
        const float v_lowFreq_imu  = 10.0f;
        const float v_highFreq_imu = 800.0f;
        const float v_lowMel_imu   = 1127.0f * log1p(v_lowFreq_imu / 700.0f);
        const float v_highMel_imu  = 1127.0f * log1p(v_highFreq_imu / 700.0f);

        for (int m = 0; m < MEL_BANDS_MAX; m++) {
            const float v_mCenter = v_lowMel_imu + (v_highMel_imu - v_lowMel_imu) * (m + 1) / (MEL_BANDS_MAX + 1);
            const float v_mLeft   = v_lowMel_imu + (v_highMel_imu - v_lowMel_imu) * m       / (MEL_BANDS_MAX + 1);
            const float v_mRight  = v_lowMel_imu + (v_highMel_imu - v_lowMel_imu) * (m + 2) / (MEL_BANDS_MAX + 1);

            const float v_hzC = 700.0f * (expm1(v_mCenter / 1127.0f));
            const float v_hzL = 700.0f * (expm1(v_mLeft   / 1127.0f));
            const float v_hzR = 700.0f * (expm1(v_mRight  / 1127.0f));

            for (int b = 0; b < BINS_IMU_MAX; b++) {
                const float v_hzB = (float)b * (800.0f / (float)BINS_IMU_MAX);
                float v_weight = 0.0f;
                if (v_hzB >= v_hzL && v_hzB <= v_hzC)      v_weight = (v_hzB - v_hzL) / (v_hzC - v_hzL);
                else if (v_hzB > v_hzC && v_hzB <= v_hzR)  v_weight = (v_hzR - v_hzB) / (v_hzR - v_hzC);
                _melBankIMU[b * MEL_PADDED + m] = v_weight;
            }
        }
    }

    // [처리 단위 6] DCT 변환 계수 행렬 구성
    for (int i = 0; i < MFCC_COEFFS_MAX_VAL; i++) {
        for (int j = 0; j < MEL_BANDS_MAX; j++) {
            _dctMatrixFlat[j * MFCC_PADDED + i] = cos(M_PI * i * (j + 0.5f) / MEL_BANDS_MAX);
        }
    }
    memcpy(_dctMatrixIMU, _dctMatrixFlat, MEL_PADDED * MFCC_PADDED * sizeof(float));

    reloadAudioMelFilter((float)audioCfg.sample_rate);

    _isInitialized = true;
    ESP_LOGI(TAG, "Feature Engine Initialized (Audio SR: %d Hz)", audioCfg.sample_rate);
    return true;
}

/**
 * @brief 가속도 3축 센서 원시 데이터 특징 추출 실행
 * @param p_inX, p_inY, p_inZ 입력 신호
 * @param p_len 샘플 길이
 * @param p_sampleRate 샘플율
 * @param p_slot 출력용 데이터 슬롯
 * @param p_accCfg 가속도 세부 설정
 */
void CL_T2_FeatureExtractor::extractAccel(const float* p_inX, const float* p_inY, const float* p_inZ, uint32_t p_len, uint32_t p_sampleRate,
                                          T2_Type::ST_UnifiedFeatureSlot_t& p_slot, const T2_Type::ST_Accel_Config_t& p_accCfg) {
    if (!_isInitialized) return;

    const float* v_in[3] = {p_inX, p_inY, p_inZ};

    for (int i = 0; i < T2_Def::Accel::Sensor::AXIS_MAX; i++) {
        if (!v_in[i]) continue;

        // 비활성화된 축에 대해서는 특징량 영역을 0으로 소거 처리
        if (!(p_accCfg.axis_mask & (1 << i))) {
            const uint32_t singleChDim = T2_Def::AI::Tensor::MFCC_COEFFS_DEF * T2_Def::AI::Tensor::MFCC_COMPONENTS_DEF;
            float* v_finalOut = &p_slot.tensor.mfcc[i * singleChDim];
            memset(v_finalOut, 0, singleChDim * sizeof(float));
            memset(_mfccHistory[i], 0, sizeof(_mfccHistory[i]));
            memset(_deltaHistory[i], 0, sizeof(_deltaHistory[i]));
            _historyCount[i] = 0;
            continue;
        }

        // [처리 단위 1] RMS 및 첨도(Kurtosis) 등 표준 통계값 추출
        _computeStats(v_in[i], p_len, p_slot.accel.rms[i], p_slot.accel.kurt[i], p_slot.accel.crest[i], p_slot.accel.skew[i], p_slot.accel.std[i]);
        p_slot.accel.cal_off[i] = p_accCfg.offset[i];

        // [처리 단위 2] 가속도 주파수 분석을 위한 고속 FFT 수행
        memset(_fftWorkIMU, 0, T2_Def::Accel::Sensor::FFT_SIZE_MAX * 2 * sizeof(float));
        for (uint32_t j = 0; j < p_len; j++) {
            _fftWorkIMU[j * 2] = v_in[i][j];
            _fftWorkIMU[j * 2 + 1] = 0.0f;
        }
        dsps_fft2r_fc32(_fftWorkIMU, p_len);
        dsps_bit_rev_fc32(_fftWorkIMU, p_len);

        uint32_t v_bins = (p_len / 2) + 1;
        float v_invLen = 1.0f / (float)p_len;
        for (uint32_t j = 0; j < v_bins; j++) {
            float re = _fftWorkIMU[j * 2];
            float im = _fftWorkIMU[j * 2 + 1];
            _powerIMU[j] = (re * re + im * im) * v_invLen;
        }

        // 센트로이드 및 피크 주파수 파악
        p_slot.accel.centroid[i] = _computeSpectralCentroid(_powerIMU, v_bins, (float)p_sampleRate);
        T2_Type::ST_SpectralPeak_t v_peaks[T2_Def::Accel::FeatureLimit::BAND_MAX] = {0};
        _extractTopPeaks(_powerIMU, v_bins, (float)p_sampleRate, v_peaks, 1, p_accCfg.peak_amp_min, p_accCfg.peak_freq_gap_min);
        p_slot.accel.peak_f[i] = v_peaks[0].freq;

        // [처리 단위 3] 각 주파수 밴드 대역의 통계 에너지 추출
        float v_currentBands[T2_Def::Accel::FeatureLimit::BAND_MAX] = {0};
        _computeBandEnergies(_powerIMU, v_bins, (float)p_sampleRate, p_accCfg.active_band_count, p_accCfg.band_en, p_accCfg.band_start, p_accCfg.band_end, v_currentBands);
        memcpy(p_slot.accel.band_energy[i], v_currentBands, sizeof(v_currentBands));

        // [처리 단위 4] 가속도 특성값 벡터화 (MFCC 연산)
        _computeMfcc(_powerIMU, v_bins, p_slot.tensor.mfcc, i, false);
    }
}

/**
 * @brief 자이로 3축 센서 원시 데이터 특징 추출 실행
 * @param p_inX, p_inY, p_inZ 입력 신호
 * @param p_len 샘플 길이
 * @param p_sampleRate 샘플율
 * @param p_slot 출력용 데이터 슬롯
 * @param p_gyrCfg 자이로 세부 설정
 */
void CL_T2_FeatureExtractor::extractGyro(const float* p_inX, const float* p_inY, const float* p_inZ, uint32_t p_len, uint32_t p_sampleRate,
                                         T2_Type::ST_UnifiedFeatureSlot_t& p_slot, const T2_Type::ST_Gyro_Config_t& p_gyrCfg) {
    if (!_isInitialized) return;

    const float* v_in[3] = {p_inX, p_inY, p_inZ};

    for (int i = 0; i < T2_Def::Gyro::Sensor::AXIS_MAX; i++) {
        if (!(p_gyrCfg.axis_mask & (1 << i))) {
            const uint32_t singleChDim = T2_Def::AI::Tensor::MFCC_COEFFS_DEF * T2_Def::AI::Tensor::MFCC_COMPONENTS_DEF;
            float* v_finalOut = &p_slot.tensor.mfcc[(3 + i) * singleChDim];
            memset(v_finalOut, 0, singleChDim * sizeof(float));
            memset(_mfccHistory[3 + i], 0, sizeof(_mfccHistory[3 + i]));
            memset(_deltaHistory[3 + i], 0, sizeof(_deltaHistory[3 + i]));
            _historyCount[3 + i] = 0;
            continue;
        }

        // [처리 단위 1] RMS 및 표준편차 등의 통계 변수 추출
        _computeStats(v_in[i], p_len, p_slot.gyro.rms[i], p_slot.gyro.kurt[i], p_slot.gyro.crest[i], p_slot.gyro.skew[i], p_slot.gyro.std[i]);
        p_slot.gyro.cal_off[i] = p_gyrCfg.offset[i];
        p_slot.gyro.drift_est[i] = 0.0f;

        // [처리 단위 2] 자이로 주파수 분석을 위한 고속 FFT 수행
        memset(_fftWorkIMU, 0, T2_Def::Gyro::Sensor::FFT_SIZE_MAX * 2 * sizeof(float));
        for (uint32_t j = 0; j < p_len; j++) {
            _fftWorkIMU[j * 2] = v_in[i][j];
            _fftWorkIMU[j * 2 + 1] = 0.0f;
        }
        dsps_fft2r_fc32(_fftWorkIMU, p_len);
        dsps_bit_rev_fc32(_fftWorkIMU, p_len);

        uint32_t v_bins = (p_len / 2) + 1;
        float v_invLen = 1.0f / (float)p_len;
        for (uint32_t j = 0; j < v_bins; j++) {
            float re = _fftWorkIMU[j * 2];
            float im = _fftWorkIMU[j * 2 + 1];
            _powerIMU[j] = (re * re + im * im) * v_invLen;
        }

        p_slot.gyro.centroid[i] = _computeSpectralCentroid(_powerIMU, v_bins, (float)p_sampleRate);

        T2_Type::ST_SpectralPeak_t v_gyrPeak[1] = {0};
        _extractTopPeaks(_powerIMU, v_bins, (float)p_sampleRate, v_gyrPeak, 1,
                         p_gyrCfg.peak_amp_min, p_gyrCfg.peak_freq_gap_min);
        p_slot.gyro.peak_f[i] = v_gyrPeak[0].freq;

        // [처리 단위 3] 저주파 영역 에너지를 이용한 드리프트 추정값 연산
        float v_dcBins = fmaxf(1.0f, (float)p_sampleRate / 2.0f / (float)v_bins);
        float v_driftSum = 0.0f;
        for (uint32_t j = 0; j < (uint32_t)v_dcBins && j < v_bins; j++) v_driftSum += _powerIMU[j];
        p_slot.gyro.drift_est[i] = v_driftSum / fmaxf(1.0f, v_dcBins);

        // 밴드별 주파수 에너지 분석
        float v_currentBands[T2_Def::Gyro::FeatureLimit::BAND_MAX] = {0};
        _computeBandEnergies(_powerIMU, v_bins, (float)p_sampleRate, p_gyrCfg.active_band_count, p_gyrCfg.band_en, p_gyrCfg.band_start, p_gyrCfg.band_end, v_currentBands);
        memcpy(p_slot.gyro.band_energy[i], v_currentBands, sizeof(v_currentBands));

        // 자이로 특징값 벡터 구성 (MFCC 연산)
        _computeMfcc(_powerIMU, v_bins, p_slot.tensor.mfcc, 3 + i, false);
    }
}

/**
 * @brief 오디오 채널 raw 데이터 특징 추출 실행 (좌/우 마이크 분리 및 특징 결합)
 * @param p_audL, p_audR 입력 마이크 채널 신호
 * @param p_len 샘플 길이
 * @param p_sampleRate 샘플율
 * @param p_slot 출력용 데이터 슬롯
 * @param p_audCfg 오디오 세부 설정
 */
void CL_T2_FeatureExtractor::extractAudio(const float* p_audL, const float* p_audR, uint32_t p_len, uint32_t p_sampleRate,
                                          T2_Type::ST_UnifiedFeatureSlot_t& p_slot, const T2_Type::ST_Audio_Config_t& p_audCfg) {
    if (!_isInitialized) return;

    uint32_t v_len = (p_len > T2_Def::Audio::Sensor::FFT_SIZE_MAX) ? T2_Def::Audio::Sensor::FFT_SIZE_MAX : p_len;
    uint32_t v_bins = v_len / 2 + 1;

    alignas(16) float v_fftLeft[T2_Def::Audio::Sensor::FFT_SIZE_MAX * 2] = {0};
    alignas(16) float v_fftRight[T2_Def::Audio::Sensor::FFT_SIZE_MAX * 2] = {0};

    bool hasLeft  = ((uint8_t)p_audCfg.channel_mask & (uint8_t)T2_Type::EM_ChannelMask_t::CH_LEFT);
    bool hasRight = ((uint8_t)p_audCfg.channel_mask & (uint8_t)T2_Type::EM_ChannelMask_t::CH_RIGHT);

    // [처리 단위 1] LEFT 오디오 채널 연산 파이프라인
    if (p_audL && hasLeft) {
        float v_unusedStdL = 0.0f;
        _computeStats(p_audL, v_len, p_slot.audio.ch[0].rms, p_slot.audio.ch[0].kurt, p_slot.audio.ch[0].crest, p_slot.audio.ch[0].skew, v_unusedStdL);
        p_slot.audio.ch[0].energy = p_slot.audio.ch[0].rms * p_slot.audio.ch[0].rms;

        // RMS 1차/2차 시간 차분값(미분) 연산
        float currentRms = p_slot.audio.ch[0].rms;
        if (_prevRms[0] > 0.00001f) {
            p_slot.audio.ch[0].d_rms = currentRms - _prevRms[0];
            p_slot.audio.ch[0].dd_rms = p_slot.audio.ch[0].d_rms - _prevDRms[0];
            _prevDRms[0] = p_slot.audio.ch[0].d_rms;
        } else {
            p_slot.audio.ch[0].d_rms = 0.0f;
            p_slot.audio.ch[0].dd_rms = 0.0f;
            _prevDRms[0] = 0.0f;
        }
        _prevRms[0] = currentRms;

        // 윈도잉 후 고속 FFT 연산
        for (uint32_t i = 0; i < v_len; i++) {
            v_fftLeft[i * 2] = p_audL[i] * _windowAudio[i];
            v_fftLeft[i * 2 + 1] = 0.0f;
        }
        dsps_fft2r_fc32(v_fftLeft, v_len);
        dsps_bit_rev_fc32(v_fftLeft, v_len);

        for (uint32_t i = 0; i < v_bins; i++) {
            float re = v_fftLeft[i * 2];
            float im = v_fftLeft[i * 2 + 1];
            _powerAudio[i] = (re * re + im * im) / v_len;
        }

        // 소음 차감 프로파일 업데이트 또는 적용
        if (_isLearning) {
            float v_alpha = T2_Def::Audio::Dsp::NOISE_LEARN_ALPHA_DEF;
            for (uint32_t i = 0; i < v_bins; i++) {
                _noiseProfile[i] = (v_alpha * _powerAudio[i]) + ((1.0f - v_alpha) * _noiseProfile[i]);
            }
            if (_learnedFrames < UINT32_MAX) _learnedFrames++;
        } else if (_learnedFrames > 0) {
            float v_subStr = p_audCfg.noise.sub_str;
            for (uint32_t i = 0; i < v_bins; i++) {
                _powerAudio[i] = fmaxf(_powerAudio[i] - (_noiseProfile[i] * v_subStr), T2_Def::Global::System::MATH_EPSILON_12_CONST);
            }
        }

        p_slot.audio.ch[0].centroid = _computeSpectralCentroid(_powerAudio, v_bins, (float)p_sampleRate);
        uint8_t v_peakCnt = p_audCfg.active_peak_count;
        if (v_peakCnt == 0 || v_peakCnt > T2_Def::Audio::FeatureLimit::TOP_PEAKS_MAX) {
            v_peakCnt = T2_Def::Audio::FeatureLimit::TOP_PEAKS_DEF;
        }
        _extractTopPeaks(_powerAudio, v_bins, (float)p_sampleRate, p_slot.audio.ch[0].top_peaks, v_peakCnt, p_audCfg.peak_amp_min, p_audCfg.peak_freq_gap_min);
        _computeBandEnergies(_powerAudio, v_bins, (float)p_sampleRate, p_audCfg.active_band_count, p_audCfg.band_en, p_audCfg.band_start, p_audCfg.band_end, p_slot.audio.ch[0].band_energy);

        // 켑스트럼(Cepstrum) IFFT 역변환 및 타깃 피크 검출
        for (uint32_t i = 0; i < v_len; i++) {
            _cepsIfftWork[i * 2]     = 0.0f;
            _cepsIfftWork[i * 2 + 1] = 0.0f;
        }
        for (uint32_t i = 0; i < v_bins; i++) {
            float v_val = log10f(fmaxf(_powerAudio[i], T2_Def::Global::System::MATH_EPSILON_12_CONST));
            _cepsIfftWork[i * 2] = v_val;
            if (i > 0 && i < v_bins - 1) {
                _cepsIfftWork[(v_len - i) * 2] = v_val;
            }
        }
        for (uint32_t i = 0; i < v_len; i++) {
            _cepsIfftWork[i * 2 + 1] = -_cepsIfftWork[i * 2 + 1];
        }
        dsps_fft2r_fc32(_cepsIfftWork, v_len);
        dsps_bit_rev_fc32(_cepsIfftWork, v_len);

        const float v_invLen = 1.0f / (float)v_len;
        for (uint32_t i = 0; i < v_len; i++) {
            _cepsIfftWork[i * 2]     *=  v_invLen;
            _cepsIfftWork[i * 2 + 1] *= -v_invLen;
        }

        int v_tolSamples = (int)(T2_Def::Audio::FeatureLimit::CEPS_TOLERANCE_DEF * p_sampleRate);
        if (v_tolSamples < 1) v_tolSamples = 1;

        for (uint8_t c = 0; c < p_audCfg.active_ceps_count; c++) {
            float v_targetSec = p_audCfg.ceps_targets[c];
            if (v_targetSec < T2_Def::Global::System::MATH_EPSILON_CONST) {
                p_slot.audio.ch[0].cpsr_max[c] = 0.0f;
                p_slot.audio.ch[0].cpsr_mxr[c] = 1.0f;
                continue;
            }
            int v_targetIdx = (int)(v_targetSec * p_sampleRate);
            if (v_targetIdx >= (int)v_len) v_targetIdx = v_len - 1;

            int v_startIdx = v_targetIdx - v_tolSamples;
            int v_endIdx = v_targetIdx + v_tolSamples;
            if (v_startIdx < 0) v_startIdx = 0;
            if (v_endIdx >= (int)v_len) v_endIdx = v_len - 1;

            float v_maxVal = -1e9f;
            for (int k = v_startIdx; k <= v_endIdx; k++) {
                float val = fabsf(_cepsIfftWork[k * 2]);
                if (val > v_maxVal) v_maxVal = val;
            }
            p_slot.audio.ch[0].cpsr_max[c] = v_maxVal;

            float v_noiseSum = 0.0f;
            int v_noiseCnt = 0;
            const int v_margin = T2_Def::Audio::FeatureLimit::CEPS_NOISE_MARGIN_CONST;
            for (int k = v_startIdx - v_margin; k <= v_endIdx + v_margin; k++) {
                if (k >= 0 && k < (int)v_len && (k < v_startIdx || k > v_endIdx)) {
                    v_noiseSum += fabsf(_cepsIfftWork[k * 2]);
                    v_noiseCnt++;
                }
            }
            float v_noiseMean = (v_noiseCnt > 0) ? (v_noiseSum / v_noiseCnt) : 1e-6f;
            if (v_noiseMean < 1e-6f) v_noiseMean = 1e-6f;
            p_slot.audio.ch[0].cpsr_mxr[c] = v_maxVal / v_noiseMean;
        }

        _computeMfcc(_powerAudio, v_bins, p_slot.tensor.mfcc, 6, true);
    }

    // [처리 단위 2] RIGHT 오디오 채널 연산 파이프라인
    if (p_audR && hasRight) {
        float v_unusedStdR = 0.0f;
        _computeStats(p_audR, v_len, p_slot.audio.ch[1].rms, p_slot.audio.ch[1].kurt, p_slot.audio.ch[1].crest, p_slot.audio.ch[1].skew, v_unusedStdR);
        p_slot.audio.ch[1].energy = p_slot.audio.ch[1].rms * p_slot.audio.ch[1].rms;

        float currentRms = p_slot.audio.ch[1].rms;
        if (_prevRms[1] > 0.00001f) {
            p_slot.audio.ch[1].d_rms = currentRms - _prevRms[1];
            p_slot.audio.ch[1].dd_rms = p_slot.audio.ch[1].d_rms - _prevDRms[1];
            _prevDRms[1] = p_slot.audio.ch[1].d_rms;
        } else {
            p_slot.audio.ch[1].d_rms = 0.0f;
            p_slot.audio.ch[1].dd_rms = 0.0f;
            _prevDRms[1] = 0.0f;
        }
        _prevRms[1] = currentRms;

        for (uint32_t i = 0; i < v_len; i++) {
            v_fftRight[i * 2] = p_audR[i] * _windowAudio[i];
            v_fftRight[i * 2 + 1] = 0.0f;
        }
        dsps_fft2r_fc32(v_fftRight, v_len);
        dsps_bit_rev_fc32(v_fftRight, v_len);

        for (uint32_t i = 0; i < v_bins; i++) {
            float re = v_fftRight[i * 2];
            float im = v_fftRight[i * 2 + 1];
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

        p_slot.audio.ch[1].centroid = _computeSpectralCentroid(_powerAudio, v_bins, (float)p_sampleRate);
        uint8_t v_peakCnt = p_audCfg.active_peak_count;
        if (v_peakCnt == 0 || v_peakCnt > T2_Def::Audio::FeatureLimit::TOP_PEAKS_MAX) {
            v_peakCnt = T2_Def::Audio::FeatureLimit::TOP_PEAKS_DEF;
        }
        _extractTopPeaks(_powerAudio, v_bins, (float)p_sampleRate, p_slot.audio.ch[1].top_peaks, v_peakCnt, p_audCfg.peak_amp_min, p_audCfg.peak_freq_gap_min);
        _computeBandEnergies(_powerAudio, v_bins, (float)p_sampleRate, p_audCfg.active_band_count, p_audCfg.band_en, p_audCfg.band_start, p_audCfg.band_end, p_slot.audio.ch[1].band_energy);

        // Right 채널 켑스트럼 IFFT
        for (uint32_t i = 0; i < v_len; i++) {
            _cepsIfftWork[i * 2]     = 0.0f;
            _cepsIfftWork[i * 2 + 1] = 0.0f;
        }
        for (uint32_t i = 0; i < v_bins; i++) {
            float v_val = log10f(fmaxf(_powerAudio[i], T2_Def::Global::System::MATH_EPSILON_12_CONST));
            _cepsIfftWork[i * 2] = v_val;
            if (i > 0 && i < v_bins - 1) {
                _cepsIfftWork[(v_len - i) * 2] = v_val;
            }
        }
        for (uint32_t i = 0; i < v_len; i++) {
            _cepsIfftWork[i * 2 + 1] = -_cepsIfftWork[i * 2 + 1];
        }
        dsps_fft2r_fc32(_cepsIfftWork, v_len);
        dsps_bit_rev_fc32(_cepsIfftWork, v_len);

        const float v_invLen = 1.0f / (float)v_len;
        for (uint32_t i = 0; i < v_len; i++) {
            _cepsIfftWork[i * 2]     *=  v_invLen;
            _cepsIfftWork[i * 2 + 1] *= -v_invLen;
        }

        int v_tolSamples = (int)(T2_Def::Audio::FeatureLimit::CEPS_TOLERANCE_DEF * p_sampleRate);
        if (v_tolSamples < 1) v_tolSamples = 1;

        for (uint8_t c = 0; c < p_audCfg.active_ceps_count; c++) {
            float v_targetSec = p_audCfg.ceps_targets[c];
            if (v_targetSec < T2_Def::Global::System::MATH_EPSILON_CONST) {
                p_slot.audio.ch[1].cpsr_max[c] = 0.0f;
                p_slot.audio.ch[1].cpsr_mxr[c] = 1.0f;
                continue;
            }
            int v_targetIdx = (int)(v_targetSec * p_sampleRate);
            if (v_targetIdx >= (int)v_len) v_targetIdx = v_len - 1;

            int v_startIdx = v_targetIdx - v_tolSamples;
            int v_endIdx = v_targetIdx + v_tolSamples;
            if (v_startIdx < 0) v_startIdx = 0;
            if (v_endIdx >= (int)v_len) v_endIdx = v_len - 1;

            float v_maxVal = -1e9f;
            for (int k = v_startIdx; k <= v_endIdx; k++) {
                float val = fabsf(_cepsIfftWork[k * 2]);
                if (val > v_maxVal) v_maxVal = val;
            }
            p_slot.audio.ch[1].cpsr_max[c] = v_maxVal;

            float v_noiseSum = 0.0f;
            int v_noiseCnt = 0;
            const int v_margin = T2_Def::Audio::FeatureLimit::CEPS_NOISE_MARGIN_CONST;
            for (int k = v_startIdx - v_margin; k <= v_endIdx + v_margin; k++) {
                if (k >= 0 && k < (int)v_len && (k < v_startIdx || k > v_endIdx)) {
                    v_noiseSum += fabsf(_cepsIfftWork[k * 2]);
                    v_noiseCnt++;
                }
            }
            float v_noiseMean = (v_noiseCnt > 0) ? (v_noiseSum / v_noiseCnt) : 1e-6f;
            if (v_noiseMean < 1e-6f) v_noiseMean = 1e-6f;
            p_slot.audio.ch[1].cpsr_mxr[c] = v_maxVal / v_noiseMean;
        }

        _computeMfcc(_powerAudio, v_bins, p_slot.tensor.mfcc, 7, true);
    }

    // [처리 단위 3] 상호 연관도(Coherence) 및 위상차(IPD) 연산
    if (p_audL && p_audR && hasLeft && hasRight) {
        float v_cohSum = 0;
        float v_ipdSum = 0;
        uint32_t v_cohCount = 0;

        for (uint32_t i = 1; i < v_bins - 1; i++) {
            float L_re = v_fftLeft[i * 2], L_im = v_fftLeft[i * 2 + 1];
            float R_re = v_fftRight[i * 2], R_im = v_fftRight[i * 2 + 1];

            float C_re = L_re * R_re + L_im * R_im;
            float C_im = L_im * R_re - L_re * R_im;

            float L_pow = L_re * L_re + L_im * L_im;
            float R_pow = R_re * R_re + R_im * R_im;
            float denom_sq = L_pow * R_pow;

            if (denom_sq > T2_Def::Global::System::MATH_EPSILON_12_CONST * T2_Def::Global::System::MATH_EPSILON_12_CONST) {
                float num_sq = C_re * C_re + C_im * C_im;
                float coh_sq = num_sq / denom_sq;
                if (coh_sq > 1.0f) coh_sq = 1.0f;
                v_cohSum += sqrtf(coh_sq);
                v_ipdSum += atan2f(C_im, C_re);
                v_cohCount++;
            }
        }
        p_slot.audio.coh = (v_cohCount > 0) ? (v_cohSum / v_cohCount) : 0.0f;
        p_slot.audio.ipd = (v_cohCount > 0) ? (v_ipdSum / v_cohCount) : 0.0f;
    } else {
        p_slot.audio.coh = 0.0f;
        p_slot.audio.ipd = 0.0f;
    }
}

/**
 * @brief 수치 데이터 시퀀스의 기초 통계 연산 (평균, RMS, 첨도 등)
 */
void CL_T2_FeatureExtractor::_computeStats(const float* p_data, uint32_t p_len, float& p_rms, float& p_kurt, float& p_crest, float& p_skew, float& p_std) {
    if (p_len == 0) {
        p_rms = 0.0f; p_kurt = 0.0f; p_crest = 0.0f; p_skew = 0.0f; p_std = 0.0f;
        return;
    }

    double mean = 0.0;
    double M2 = 0.0, M3 = 0.0, M4 = 0.0;
    float v_sqSum = 0.0f;
    float v_maxAbs = 0.0f;

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

    p_rms = sqrtf(v_sqSum / p_len);
    float v_var = (float)(M2 / p_len);
    p_std = sqrtf(v_var);

    float v_denom = fmaxf(p_std * p_std, T2_Def::Global::System::MATH_EPSILON_12_CONST);
    p_skew = (p_std > T2_Def::Global::System::MATH_EPSILON_12_CONST) ? (float)(M3 / p_len) / (v_denom * p_std) : 0.0f;
    p_kurt = (float)(M4 / p_len) / (v_denom * v_denom);
    p_crest = v_maxAbs / fmaxf(p_rms, T2_Def::Global::System::MATH_EPSILON_12_CONST);
}

/**
 * @brief 스펙트럼 센트로이드(주파수 무게중심) 연산
 */
float CL_T2_FeatureExtractor::_computeSpectralCentroid(const float* p_power, uint32_t p_bins, float p_sampleRate) {
    float v_num = 0, v_den = 0;
    for (uint32_t i = 0; i < p_bins; i++) {
        float freq = i * (p_sampleRate / 2.0f) / (p_bins - 1);
        v_num += freq * p_power[i];
        v_den += p_power[i];
    }
    return v_num / fmaxf(v_den, T2_Def::Global::System::MATH_EPSILON_CONST);
}

/**
 * @brief 파워 스펙트럼상 주요 피크 주파수 검출
 */
void CL_T2_FeatureExtractor::_extractTopPeaks(const float* p_power, uint32_t p_bins, float p_sampleRate, T2_Type::ST_SpectralPeak_t* p_outPeaks, uint8_t p_maxCount, float p_ampMin, float p_freqGap) {
    struct Peak { float f; float a; };
    constexpr uint16_t MAX_CANDIDATES = 128;
    Peak v_cands[MAX_CANDIDATES];
    uint16_t v_candCount = 0;

    for (uint32_t i = 1; i < p_bins - 1; i++) {
        if (p_power[i] > p_power[i-1] && p_power[i] > p_power[i+1] && p_power[i] > p_ampMin) {
            float freq = i * (p_sampleRate / 2.0f) / (p_bins - 1);
            
            uint16_t insertIdx = v_candCount;
            while (insertIdx > 0 && v_cands[insertIdx - 1].a < p_power[i]) {
                if (insertIdx < MAX_CANDIDATES) {
                    v_cands[insertIdx] = v_cands[insertIdx - 1];
                }
                insertIdx--;
            }
            if (insertIdx < MAX_CANDIDATES) {
                v_cands[insertIdx] = {freq, p_power[i]};
                if (v_candCount < MAX_CANDIDATES) v_candCount++;
            }
        }
    }

    uint8_t count = 0;
    for (uint16_t i = 0; i < v_candCount; i++) {
        if (count >= p_maxCount) break;
        bool tooClose = false;
        for (int j = 0; j < count; j++) {
            if (std::abs(v_cands[i].f - p_outPeaks[j].freq) < p_freqGap) { 
                tooClose = true; 
                break; 
            }
        }
        if (!tooClose) {
            p_outPeaks[count].freq = v_cands[i].f;
            p_outPeaks[count].amp = v_cands[i].a;
            count++;
        }
    }
    for (; count < p_maxCount; count++) { 
        p_outPeaks[count].freq = 0.0f; 
        p_outPeaks[count].amp = 0.0f; 
    }
}

/**
 * @brief 설정된 세그먼트 주파수 밴드별 에너지 총합 연산
 */
void CL_T2_FeatureExtractor::_computeBandEnergies(const float* p_power, uint32_t p_bins, float p_sampleRate, uint8_t p_count, const bool* p_en, const float* p_start, const float* p_end, float* p_outEnergies) {
    float v_binHz = (p_sampleRate / 2.0f) / (p_bins - 1);

    uint8_t v_limit = std::min((int)p_count, (int)T2_Def::Accel::FeatureLimit::BAND_MAX);
    for (uint8_t i = 0; i < v_limit; i++) {
        if (!p_en[i]) {
            p_outEnergies[i] = 0.0f;
            continue;
        }

        uint32_t v_startIdx = (uint32_t)(p_start[i] / v_binHz);
        uint32_t v_endIdx   = (uint32_t)(p_end[i] / v_binHz);

        if (v_startIdx >= p_bins) v_startIdx = p_bins - 1;
        if (v_endIdx >= p_bins)   v_endIdx = p_bins - 1;

        float v_sum = 0;
        for (uint32_t j = v_startIdx; j <= v_endIdx; j++) {
            v_sum += p_power[j];
        }
        p_outEnergies[i] = v_sum;
    }
}

/**
 * @brief Mel 필터뱅크 및 DCT 행렬 연산을 활용한 MFCC 및 델타 특징 추출
 */
void CL_T2_FeatureExtractor::_computeMfcc(const float* p_power, uint32_t p_bins, float* p_outMfcc, uint8_t p_chIdx, bool isAudio) {
    if (p_chIdx >= 8) return;

    const uint16_t binPadded = (p_bins + 3) & ~3;

    alignas(16) float v_melEnergies[MEL_PADDED] = {0};
    float* melBank = isAudio ? _melBankFlat : _melBankIMU;
    float* dctMat  = isAudio ? _dctMatrixFlat : _dctMatrixIMU;

    // Mel 필터뱅크 가중 행렬 곱셈
    dspm_mult_f32(p_power, melBank, v_melEnergies, 1, binPadded, MEL_PADDED);

    const uint8_t activeBands = isAudio ? _activeMelBands : MEL_BANDS_MAX;
    for (int i = 0; i < activeBands; i++) {
        v_melEnergies[i] = log10f(fmaxf(v_melEnergies[i], T2_Def::Global::System::MATH_EPSILON_12_CONST));
    }
    for (int i = activeBands; i < MEL_PADDED; i++) {
        v_melEnergies[i] = 0.0f;
    }

    alignas(16) float v_dctOut[MFCC_PADDED] = {0};
    // DCT 변환 행렬 곱셈
    dspm_mult_f32(v_melEnergies, dctMat, v_dctOut, 1, MEL_PADDED, MFCC_PADDED);

    uint16_t v_dim = T2_Def::AI::Tensor::MFCC_COEFFS_DEF;
    uint16_t& v_chCnt = _historyCount[p_chIdx];

    // 링 버퍼 헤드 순환
    uint8_t v_currHead = _historyHead[p_chIdx];
    uint8_t v_nextHead = (v_currHead + 1) % T2_Def::AI::Tensor::DELTA_HISTORY_MAX;
    _historyHead[p_chIdx] = v_nextHead;

    memcpy(_mfccHistory[p_chIdx][v_nextHead], v_dctOut, v_dim * sizeof(float));
    if (v_chCnt < T2_Def::AI::Tensor::DELTA_HISTORY_MAX) {
        v_chCnt++;
    }

    const uint32_t singleChDim = T2_Def::AI::Tensor::MFCC_COEFFS_DEF * T2_Def::AI::Tensor::MFCC_COMPONENTS_DEF;
    float* v_finalOut = &p_outMfcc[p_chIdx * singleChDim];

    // 정적(Static) MFCC 복사
    memcpy(v_finalOut, v_dctOut, v_dim * sizeof(float));

    // 동적(Delta / Delta-Delta) MFCC 연산 (과거 히스토리가 확보되었을 경우)
    if (v_chCnt >= T2_Def::AI::Tensor::DELTA_HISTORY_MAX) {
        uint8_t v_oldestHead = (v_nextHead + 1) % T2_Def::AI::Tensor::DELTA_HISTORY_MAX;
        float v_invDeltaGap = 1.0f / (float)T2_Def::AI::Tensor::DELTA_GAP_CONST;

        for (int i = 0; i < v_dim; i++) {
            v_finalOut[v_dim + i] = (_mfccHistory[p_chIdx][v_nextHead][i] - _mfccHistory[p_chIdx][v_oldestHead][i]) * v_invDeltaGap;
        }

        memcpy(_deltaHistory[p_chIdx][v_nextHead], &v_finalOut[v_dim], v_dim * sizeof(float));

        for (int i = 0; i < v_dim; i++) {
            v_finalOut[v_dim * 2 + i] = (_deltaHistory[p_chIdx][v_nextHead][i] - _deltaHistory[p_chIdx][v_oldestHead][i]) * v_invDeltaGap;
        }
    } else {
        memset(&v_finalOut[v_dim], 0, v_dim * 2 * sizeof(float));
    }
}

/**
 * @brief 오디오 샘플링 속도 변화에 대응한 Mel 가중치 매트릭스 재구성
 */
void CL_T2_FeatureExtractor::reloadAudioMelFilter(float sampleRate) {
    if (!_melBankFlat) {
        ESP_LOGE(TAG, "reloadAudioMelFilter: _melBankFlat is null");
        return;
    }

    const float nyquist    = sampleRate / 2.0f;
    const float v_lowFreq  = 100.0f;
    const float v_highFreq = fminf(8000.0f, nyquist * 0.95f);
    const float v_lowMel   = 1127.0f * log1p(v_lowFreq / 700.0f);
    const float v_highMel  = 1127.0f * log1p(v_highFreq / 700.0f);

    memset(_melBankFlat, 0, BINS_PADDED * MEL_PADDED * sizeof(float));

    for (int m = 0; m < _activeMelBands; m++) {
        const float v_mCenter = v_lowMel + (v_highMel - v_lowMel) * (m + 1) / (MEL_BANDS_MAX + 1);
        const float v_mLeft   = v_lowMel + (v_highMel - v_lowMel) * m       / (MEL_BANDS_MAX + 1);
        const float v_mRight  = v_lowMel + (v_highMel - v_lowMel) * (m + 2) / (MEL_BANDS_MAX + 1);

        const float v_hzC = 700.0f * (expm1(v_mCenter / 1127.0f));
        const float v_hzL = 700.0f * (expm1(v_mLeft   / 1127.0f));
        const float v_hzR = 700.0f * (expm1(v_mRight  / 1127.0f));

        for (int b = 0; b < BINS_MAX; b++) {
            const float v_hzB = (float)b * (nyquist / (float)BINS_MAX);
            float v_weight = 0.0f;
            if (v_hzB >= v_hzL && v_hzB <= v_hzC)      v_weight = (v_hzB - v_hzL) / (v_hzC - v_hzL);
            else if (v_hzB > v_hzC && v_hzB <= v_hzR)  v_weight = (v_hzR - v_hzB) / (v_hzR - v_hzC);
            _melBankFlat[b * MEL_PADDED + m] = v_weight;
        }
    }

    ESP_LOGI(TAG, "Audio Mel Filterbank reloaded (SR: %.0f Hz, Nyquist: %.0f Hz, Bins: %d)",
             sampleRate, nyquist, BINS_MAX);
}

/**
 * @brief 학습용 소음 백그라운드 스펙트럼 초기화
 */
void CL_T2_FeatureExtractor::resetNoiseProfile() {
    if (_noiseProfile) memset(_noiseProfile, 0, 2 * BINS_PADDED * sizeof(float));
    _learnedFrames = 0;
}

/**
 * @brief MFCC 히스토리 저장 링 버퍼 완전 초기화
 */
void CL_T2_FeatureExtractor::resetHistory() {
    memset(_historyCount, 0, sizeof(_historyCount));
    memset(_historyHead, 0, sizeof(_historyHead));
    memset(_mfccHistory, 0, sizeof(float) * 8 * T2_Def::AI::Tensor::DELTA_HISTORY_MAX * MFCC_PADDED);
    memset(_deltaHistory, 0, sizeof(float) * 8 * T2_Def::AI::Tensor::DELTA_HISTORY_MAX * MFCC_PADDED);
}
