/* 
============================================================================
 * File: T440_FeatExtra_013.cpp
 * Summary: 39D MFCC, Cepstrum & Spatial Feature Extraction (Tiling Optimized)
 * ============================================================================
 * * [AI 메모: 마스터플랜 및 v013 핵심 적용 사항]
 * 1. [교정] O(N) 슬라이딩 STA/LTA 탐색, Parseval 정리 보정, NMS 로직 유지.
 * 2. [타일링 최적화]: _computeMfcc39 내부 캐시 미스 방어 루틴 유지.
 * 3. [v013 핫스왑]: extract() 맨 앞줄에서 isTuningActive() 감지 시 캐시 리로드 수행.
 * 노이즈 스케일링 함수 삭제 및 하드 리셋 일원화.
 * ========================================================================== 
 */

#include "T440_FeatExtra_013.hpp"
#include "esp_log.h"
#include "dsps_fft2r.h"
#include "dsps_wind.h"
#include "dspm_mult.h"
#include "dsps_math.h"
#include <cmath>
#include <cstring>

static const char* TAG = "T440_FEAT";

T440_FeatureExtractor::T440_FeatureExtractor() {
    _historyCount = 0;
    _melBankFlat = nullptr;
    _dctMatrixFlat = nullptr;
    _fftSpatialL = nullptr;
    _fftSpatialR = nullptr;
    memset(_mfccHistory, 0, sizeof(_mfccHistory));
    memset(_deltaHistory, 0, sizeof(_deltaHistory));
}

T440_FeatureExtractor::~T440_FeatureExtractor() {
    if (_melBankFlat) heap_caps_free(_melBankFlat);
    if (_dctMatrixFlat) heap_caps_free(_dctMatrixFlat);
    if (_fftSpatialL) heap_caps_free(_fftSpatialL);
    if (_fftSpatialR) heap_caps_free(_fftSpatialR);
}

bool T440_FeatureExtractor::init() {
    _melBankFlat = (float*)heap_caps_aligned_alloc(16, BINS_PADDED * MEL_PADDED * sizeof(float), MALLOC_CAP_SPIRAM);
    _dctMatrixFlat = (float*)heap_caps_aligned_alloc(16, MEL_PADDED * MFCC_PADDED * sizeof(float), MALLOC_CAP_SPIRAM);
    _fftSpatialL = (float*)heap_caps_aligned_alloc(16, SmeaConfig::System::FFT_SIZE_CONST * 2 * sizeof(float), MALLOC_CAP_SPIRAM);
    _fftSpatialR = (float*)heap_caps_aligned_alloc(16, SmeaConfig::System::FFT_SIZE_CONST * 2 * sizeof(float), MALLOC_CAP_SPIRAM);

    if (!_melBankFlat || !_dctMatrixFlat || !_fftSpatialL || !_fftSpatialR) {
        ESP_LOGE(TAG, "Matrix allocation failed!"); return false;
    }

    memset(_melBankFlat, 0, BINS_PADDED * MEL_PADDED * sizeof(float));
    memset(_dctMatrixFlat, 0, MEL_PADDED * MFCC_PADDED * sizeof(float));
    memset(_noiseProfile, 0, sizeof(_noiseProfile));

    dsps_wind_hann_f32(_window, SmeaConfig::System::FFT_SIZE_CONST);

    float v_fs = (float)SmeaConfig::System::SAMPLING_RATE_CONST;
    float v_minMel = 0.0f;
    float v_maxMel = SmeaConfig::FeatureLimit::MEL_SCALE_2595_CONST * log10f(1.0f + ((v_fs / 2.0f) / SmeaConfig::FeatureLimit::MEL_SCALE_700_CONST));
    float v_melStep = (v_maxMel - v_minMel) / (float)(SmeaConfig::System::MEL_BANDS_CONST + 1);

    uint16_t v_binPoints[SmeaConfig::System::MEL_BANDS_CONST + 2];
    for (uint16_t i = 0; i < SmeaConfig::System::MEL_BANDS_CONST + 2; i++) {
        float v_hz = SmeaConfig::FeatureLimit::MEL_SCALE_700_CONST * (powf(10.0f, (v_minMel + i * v_melStep) / SmeaConfig::FeatureLimit::MEL_SCALE_2595_CONST) - 1.0f);
        v_binPoints[i] = (uint16_t)roundf(SmeaConfig::System::FFT_SIZE_CONST * v_hz / v_fs);
    }

    uint16_t v_bins = (SmeaConfig::System::FFT_SIZE_CONST / 2) + 1;
    for (uint16_t m = 0; m < SmeaConfig::System::MEL_BANDS_CONST; m++) {
        uint16_t v_left = v_binPoints[m], v_center = v_binPoints[m + 1], v_right = v_binPoints[m + 2];
        for (uint16_t k = v_left; k < v_center && k < v_bins; k++)
            _melBankFlat[k * MEL_PADDED + m] = (float)(k - v_left) / (float)(v_center - v_left + SmeaConfig::System::MATH_EPSILON_12_CONST);
        for (uint16_t k = v_center; k < v_right && k < v_bins; k++)
            _melBankFlat[k * MEL_PADDED + m] = (float)(v_right - k) / (float)(v_right - v_center + SmeaConfig::System::MATH_EPSILON_12_CONST);
    }

    for (uint16_t k = 0; k < SmeaConfig::System::MEL_BANDS_CONST; k++) {
        for (uint16_t n = 0; n < SmeaConfig::System::MFCC_COEFFS_CONST; n++) {
            _dctMatrixFlat[k * MFCC_PADDED + n] = cosf(((float)M_PI / (float)SmeaConfig::System::MEL_BANDS_CONST) * (k + 0.5f) * n);
        }
    }
    
    reloadConfigCache(); // [v013] 부팅 시 초기 캐시 로드
    
    _historyCount = 0; _noiseLearnedFrames = 0; _isNoiseLearning = false;
    return true;
}

void T440_FeatureExtractor::extract(const float* p_cleanSignal, const float* p_rawL, const float* p_rawR, uint32_t p_len, SmeaType::FeatureSlot& p_outSlot) {
    if (p_len > SmeaConfig::System::FFT_SIZE_CONST) return;
    
    // [v013 핫스왑] Web UI/UX 튜닝 발생 시 락프리 캐시 갱신
    if (T415_ConfigManager::getInstance().isTuningActive()) {
        reloadConfigCache();
    }
    
    _computeBasicFeatures(p_cleanSignal, p_len, p_outSlot);
    _computePowerSpectrum(p_cleanSignal, p_len);
    _computeBandRMS(p_outSlot);
    _computeSpectralCentroid(p_outSlot);
    _computeNtopPeaks(p_outSlot);
    _computeMfcc39(p_outSlot);
    _computeCepstrum(p_outSlot);
    _computeSpatialFeatures(p_rawL, p_rawR, p_len, p_outSlot);
}

void T440_FeatureExtractor::_computeBasicFeatures(const float* p_signal, uint32_t p_len, SmeaType::FeatureSlot& p_slot) {
    float v_sumSq = 0.0f;
    float v_peak = 0.0f;
    float v_sum = 0.0f;
    for (uint32_t i = 0; i < p_len; i++) v_sum += p_signal[i];
    float v_mean = v_sum / p_len;

    float v_varianceSum = 0.0f;
    float v_moment4Sum = 0.0f;

    for (uint32_t i = 0; i < p_len; i++) {
        float v_val = fabsf(p_signal[i]);
        v_sumSq += v_val * v_val;
        if (v_val > v_peak) v_peak = v_val;
        float v_diff = p_signal[i] - v_mean;
        float v_diff2 = v_diff * v_diff;
        v_varianceSum += v_diff2;
        v_moment4Sum += v_diff2 * v_diff2;
    }

    p_slot.rms = sqrtf(fmaxf(v_sumSq / p_len, SmeaConfig::System::MATH_EPSILON_12_CONST));
    p_slot.energy = v_sumSq / (float)SmeaConfig::System::SAMPLING_RATE_CONST;
    p_slot.crest_factor = p_slot.rms > 0 ? (v_peak / p_slot.rms) : 0.0f;

    float v_staSum = 0.0f, v_ltaSum = 0.0f;
    float v_maxRatio = 1.0f;
    uint32_t v_staLen = SmeaConfig::FeatureLimit::STA_SAMPLES_CONST;
    uint32_t v_ltaLen = SmeaConfig::FeatureLimit::LTA_SAMPLES_CONST;

    for(uint32_t i = 0; i < v_ltaLen; i++) {
        float sq = p_signal[i] * p_signal[i];
        v_ltaSum += sq;
        if(i >= v_ltaLen - v_staLen) v_staSum += sq;
    }

    for(uint32_t i = v_ltaLen; i < p_len; i++) {
        float v_newSq = p_signal[i] * p_signal[i];
        float v_oldLtaSq = p_signal[i - v_ltaLen] * p_signal[i - v_ltaLen];
        float v_oldStaSq = p_signal[i - v_staLen] * p_signal[i - v_staLen];

        v_ltaSum = v_ltaSum + v_newSq - v_oldLtaSq;
        v_staSum = v_staSum + v_newSq - v_oldStaSq;

        float v_ltaAvg = v_ltaSum / v_ltaLen;
        float v_staAvg = v_staSum / v_staLen;
        if (v_ltaAvg > SmeaConfig::System::MATH_EPSILON_12_CONST) {
            float v_ratio = v_staAvg / v_ltaAvg;
            if (v_ratio > v_maxRatio) v_maxRatio = v_ratio;
        }
    }
    p_slot.sta_lta_ratio = v_maxRatio;

    float v_variance = v_varianceSum / p_len;
    float v_moment4 = v_moment4Sum / p_len;
    if (v_variance > 1e-6f) {
        p_slot.kurtosis = (v_moment4 / (v_variance * v_variance)) - 3.0f;
    } else {
        p_slot.kurtosis = 0.0f;
    }
    p_slot.pooling_stddev_min = sqrtf(fmaxf(v_variance, SmeaConfig::System::MATH_EPSILON_12_CONST));
}

void T440_FeatureExtractor::_computePowerSpectrum(const float* p_signal, uint32_t p_len) {
    uint16_t v_bins = (p_len / 2) + 1;

    for (uint32_t i = 0; i < p_len; i++) {
        _fftWorkBuf[i * 2]     = p_signal[i] * _window[i];
        _fftWorkBuf[i * 2 + 1] = 0.0f;
    }

    dsps_fft2r_fc32(_fftWorkBuf, p_len);
    dsps_bit_rev2r_fc32(_fftWorkBuf, p_len);

    float v_scale = (1.0f / (float)p_len) * 2.0f * 2.666f;
    for (uint16_t i = 0; i < v_bins; i++) {
        float v_re = _fftWorkBuf[i * 2] * v_scale;
        float v_im = _fftWorkBuf[i * 2 + 1] * v_scale;
        _powerSpectrum[i] = fmaxf((v_re * v_re + v_im * v_im), SmeaConfig::System::MATH_EPSILON_12_CONST);
    }

    if (BINS_PADDED > v_bins) {
        memset(&_powerSpectrum[v_bins], 0, (BINS_PADDED - v_bins) * sizeof(float));
    }

    DynamicConfig v_cfg = _cachedCfg;

    if (_isNoiseLearning) {
        float v_alpha = 0.01f;
        for (uint16_t i = 0; i < v_bins; i++) {
            _noiseProfile[i] = (v_alpha * _powerSpectrum[i]) + ((1.0f - v_alpha) * _noiseProfile[i]);
        }
        if (_noiseLearnedFrames < UINT32_MAX) _noiseLearnedFrames++;
    }

    if (!_isNoiseLearning && _noiseLearnedFrames > 0) {
        for (uint16_t i = 0; i < v_bins; i++) {
            _powerSpectrum[i] = fmaxf(_powerSpectrum[i] - (_noiseProfile[i] * v_cfg.dsp.spectral_sub_gain), SmeaConfig::System::MATH_EPSILON_12_CONST);
        }
    }
}

void T440_FeatureExtractor::_computeSpectralCentroid(SmeaType::FeatureSlot& p_slot) {
    uint16_t v_bins = (SmeaConfig::System::FFT_SIZE_CONST / 2) + 1;
    float v_binRes = (float)SmeaConfig::System::SAMPLING_RATE_CONST / SmeaConfig::System::FFT_SIZE_CONST;
    float v_weightedSum = 0.0f;
    float v_energySum = 0.0f;

    for (uint16_t i = 1; i < v_bins; i++) {
        float v_freq = i * v_binRes;
        v_weightedSum += v_freq * _powerSpectrum[i];
        v_energySum += _powerSpectrum[i];
    }

    if (v_energySum > SmeaConfig::System::MATH_EPSILON_12_CONST) {
        p_slot.spectral_centroid = v_weightedSum / v_energySum;
    } else {
        p_slot.spectral_centroid = 0.0f;
    }
}

void T440_FeatureExtractor::_computeNtopPeaks(SmeaType::FeatureSlot& p_slot) {
    uint16_t v_bins = (SmeaConfig::System::FFT_SIZE_CONST / 2) + 1;
    float v_binRes = (float)SmeaConfig::System::SAMPLING_RATE_CONST / SmeaConfig::System::FFT_SIZE_CONST;
    DynamicConfig v_cfg = _cachedCfg;

    float v_amplitude_limitMin = v_cfg.feature.peak_amplitude_limit_min;
    float v_freqGap_limitMin   = v_cfg.feature.peak_freq_gap_limit_hz_min;

    SmeaType::SpectralPeak v_freqComp_candis[SmeaConfig::FeatureLimit::MAX_PEAK_CANDIDATES_CONST];
    uint16_t v_peakCand_Count = 0;

    for (uint16_t i = 1; i < v_bins - 1 && v_peakCand_Count < SmeaConfig::FeatureLimit::MAX_PEAK_CANDIDATES_CONST; i++) {
        if (_powerSpectrum[i] > v_amplitude_limitMin &&
            _powerSpectrum[i] > _powerSpectrum[i - 1] &&
            _powerSpectrum[i] > _powerSpectrum[i + 1]) {
            v_freqComp_candis[v_peakCand_Count].frequency = i * v_binRes;
            v_freqComp_candis[v_peakCand_Count].amplitude = _powerSpectrum[i];
            v_peakCand_Count++;
        }
    }

    for (uint16_t i = 0; i < v_peakCand_Count - 1; i++) {
        for (uint16_t j = 0; j < v_peakCand_Count - i - 1; j++) {
            if (v_freqComp_candis[j].amplitude < v_freqComp_candis[j + 1].amplitude) {
                SmeaType::SpectralPeak temp = v_freqComp_candis[j];
                v_freqComp_candis[j]        = v_freqComp_candis[j + 1];
                v_freqComp_candis[j + 1]    = temp;
            }
        }
    }

    SmeaType::SpectralPeak v_finalPeaks[SmeaConfig::FeatureLimit::TOP_PEAKS_COUNT_CONST];
    uint8_t v_peakFinal_Count = 0;

    for (uint16_t i = 0; i < v_peakCand_Count && v_peakFinal_Count < SmeaConfig::FeatureLimit::TOP_PEAKS_COUNT_CONST; i++) {
        bool v_isValid = true;
        for (uint8_t j = 0; j < v_peakFinal_Count; j++) {
            if (fabsf(v_freqComp_candis[i].frequency - v_finalPeaks[j].frequency) < v_freqGap_limitMin) {
                v_isValid = false; break;
            }
        }
        if (v_isValid) {
            v_finalPeaks[v_peakFinal_Count] = v_freqComp_candis[i];
            v_peakFinal_Count++;
        }
    }

    for (uint8_t i = 0; i < SmeaConfig::FeatureLimit::TOP_PEAKS_COUNT_CONST; i++) {
        if (i < v_peakFinal_Count) p_slot.top_peaks[i] = v_finalPeaks[i];
        else p_slot.top_peaks[i] = {0.0f, 0.0f};
    }
}

void T440_FeatureExtractor::_computeMfcc39(SmeaType::FeatureSlot& p_slot) {
    alignas(16) float v_melEnergy[MEL_PADDED] = {0};

    // (PSRAM Cache Miss Stall) 해결을 위한 메모리 타일링 연산
    // 거대한 PSRAM 행렬을 Internal SRAM 크기의 Chunk 단위로 잘라 복사한 뒤 SIMD 연산을 수행합니다.
    for (uint16_t k = 0; k < BINS_PADDED; k += MEL_CHUNK_ROWS) {
        uint16_t v_currentRows = (k + MEL_CHUNK_ROWS > BINS_PADDED) ? (BINS_PADDED - k) : MEL_CHUNK_ROWS;
        
        // 1. PSRAM -> Internal SRAM 고속 블록 복사
        memcpy(_melScratchBuf, &_melBankFlat[k * MEL_PADDED], v_currentRows * MEL_PADDED * sizeof(float));
        
        alignas(16) float v_tempOut[MEL_PADDED] = {0};
        
        // 2. SRAM 간의 다이렉트 행렬 곱 (캐시 미스 원천 차단)
        dspm_mult_f32(&_powerSpectrum[k], _melScratchBuf, v_tempOut, 1, v_currentRows, MEL_PADDED);
        
        // 3. 누적 합산
        dsps_add_f32(v_melEnergy, v_tempOut, v_melEnergy, MEL_PADDED, 1, 1, 1);
    }

    for (uint16_t m = 0; m < SmeaConfig::System::MEL_BANDS_CONST; m++) {
        v_melEnergy[m] = logf(fmaxf(v_melEnergy[m], SmeaConfig::System::MATH_EPSILON_12_CONST));
    }

    alignas(16) float v_dctOut[MFCC_PADDED] = {0};
    
    // DCT 행렬(1.7KB)은 크기가 작으므로 한 번에 SRAM 복사 후 연산
    memcpy(_dctScratchBuf, _dctMatrixFlat, MEL_PADDED * MFCC_PADDED * sizeof(float));
    dspm_mult_f32(v_melEnergy, _dctScratchBuf, v_dctOut, 1, MEL_PADDED, MFCC_PADDED);

    memcpy(p_slot.mfcc, v_dctOut, SmeaConfig::System::MFCC_COEFFS_CONST * sizeof(float));
    _applyTemporalDerivatives(p_slot);
}

void T440_FeatureExtractor::_applyTemporalDerivatives(SmeaType::FeatureSlot& p_slot) {
    float* p_mfcc39 = p_slot.mfcc;
    uint16_t v_dim = SmeaConfig::System::MFCC_COEFFS_CONST;

    if (_historyCount < SmeaConfig::FeatureLimit::DELTA_HISTORY_FRAMES_CONST) {
        memcpy(_mfccHistory[_historyCount], p_mfcc39, sizeof(float) * v_dim);
        _rmsHistory[_historyCount] = p_slot.rms;
        _historyCount++;
    } else {
        for (uint8_t i = 0; i < SmeaConfig::FeatureLimit::DELTA_HISTORY_FRAMES_CONST - 1; i++) {
            memcpy(_mfccHistory[i], _mfccHistory[i + 1], sizeof(float) * v_dim);
        }
        memcpy(_mfccHistory[SmeaConfig::FeatureLimit::DELTA_HISTORY_FRAMES_CONST - 1], p_mfcc39, sizeof(float) * v_dim);
        _rmsHistory[SmeaConfig::FeatureLimit::DELTA_HISTORY_FRAMES_CONST - 1] = p_slot.rms;
    }

    if (_historyCount >= SmeaConfig::FeatureLimit::DELTA_HISTORY_FRAMES_CONST) {
        for (uint16_t i = 0; i < v_dim; i++) {
            p_mfcc39[v_dim + i] = ((_mfccHistory[3][i] - _mfccHistory[1][i]) + 2.0f * (_mfccHistory[4][i] - _mfccHistory[0][i])) / 10.0f;
        }

        for (uint8_t i = 0; i < SmeaConfig::FeatureLimit::DELTA_HISTORY_FRAMES_CONST - 1; i++) {
            memcpy(_deltaHistory[i], _deltaHistory[i + 1], sizeof(float) * v_dim);
        }
        memcpy(_deltaHistory[SmeaConfig::FeatureLimit::DELTA_HISTORY_FRAMES_CONST - 1], p_mfcc39 + v_dim, sizeof(float) * v_dim);

        for (uint16_t i = 0; i < v_dim; i++) {
            p_mfcc39[v_dim * 2 + i] = ((_deltaHistory[3][i] - _deltaHistory[1][i]) + 2.0f * (_deltaHistory[4][i] - _deltaHistory[0][i])) / 10.0f;
        }

        p_slot.delta_rms = (_rmsHistory[4] - _rmsHistory[2]) / 2.0f;
        p_slot.delta_delta_rms = _rmsHistory[4] - (2.0f * _rmsHistory[3]) + _rmsHistory[2];

    } else {
        memset(p_mfcc39 + v_dim, 0, v_dim * 2 * sizeof(float));
    }
}

void T440_FeatureExtractor::_computeCepstrum(SmeaType::FeatureSlot& p_slot) {
    uint16_t v_bins = (SmeaConfig::System::FFT_SIZE_CONST / 2) + 1;
    DynamicConfig v_cfg = _cachedCfg;
    float* v_cepsWorkBuf = _fftSpatialL;

    for (uint16_t i = 0; i < SmeaConfig::System::FFT_SIZE_CONST; i++) {
        if (i < v_bins) {
            v_cepsWorkBuf[i * 2] = logf(fmaxf(_powerSpectrum[i], SmeaConfig::System::MATH_EPSILON_12_CONST));
        } else {
            v_cepsWorkBuf[i * 2] = v_cepsWorkBuf[(SmeaConfig::System::FFT_SIZE_CONST - i) * 2];
        }
        v_cepsWorkBuf[i * 2 + 1] = 0.0f;
    }

    dsps_fft2r_fc32(v_cepsWorkBuf, SmeaConfig::System::FFT_SIZE_CONST);
    dsps_bit_rev2r_fc32(v_cepsWorkBuf, SmeaConfig::System::FFT_SIZE_CONST);

    for (uint8_t n = 0; n < SmeaConfig::FeatureLimit::CEPS_TARGET_COUNT_CONST; n++) {
        float v_targetHz = v_cfg.feature.ceps_targets[n];
        if (v_targetHz <= 0.0f) {
            p_slot.cpsr_max[n] = 0.0f; p_slot.cpsr_mxrms[n] = 0.0f; continue;
        }

        float v_targetQuefrencySec = 1.0f / v_targetHz;
        uint16_t v_targetIdx = (uint16_t)roundf(v_targetQuefrencySec * SmeaConfig::System::SAMPLING_RATE_CONST);
        uint16_t v_toleranceIdx = 3;

        uint16_t v_startIdx = (v_targetIdx > v_toleranceIdx) ? (v_targetIdx - v_toleranceIdx) : 1;
        uint16_t v_endIdx = v_targetIdx + v_toleranceIdx;

        if (v_endIdx >= SmeaConfig::System::FFT_SIZE_CONST / 2) {
            v_endIdx = (SmeaConfig::System::FFT_SIZE_CONST / 2) - 1;
        }

        float v_maxVal = 0.0f;
        float v_sumSq = 0.0f;
        uint16_t v_count = 0;

        for (uint16_t i = v_startIdx; i <= v_endIdx; i++) {
            float v_real = v_cepsWorkBuf[i * 2];
            float v_imag = v_cepsWorkBuf[i * 2 + 1];
            float v_cepsVal = sqrtf(v_real * v_real + v_imag * v_imag) / (float)SmeaConfig::System::FFT_SIZE_CONST;

            if (v_cepsVal > v_maxVal) v_maxVal = v_cepsVal;
            v_sumSq += v_cepsVal * v_cepsVal;
            v_count++;
        }

        p_slot.cpsr_max[n] = v_maxVal;
        float v_rms = v_count > 0 ? sqrtf(v_sumSq / v_count) : SmeaConfig::System::MATH_EPSILON_12_CONST;
        p_slot.cpsr_mxrms[n] = v_maxVal / fmaxf(v_rms, SmeaConfig::System::MATH_EPSILON_12_CONST);
    }
}

void T440_FeatureExtractor::_computeSpatialFeatures(const float* p_L, const float* p_R, uint32_t p_len, SmeaType::FeatureSlot& p_slot) {
    if (p_len > SmeaConfig::System::FFT_SIZE_CONST) return;
    DynamicConfig v_cfg = _cachedCfg;

    for (uint32_t i = 0; i < p_len; i++) {
        _fftSpatialL[i * 2] = p_L[i] * _window[i];
        _fftSpatialL[i * 2 + 1] = 0.0f;
        _fftSpatialR[i * 2] = p_R[i] * _window[i];
        _fftSpatialR[i * 2 + 1] = 0.0f;
    }

    dsps_fft2r_fc32(_fftSpatialL, SmeaConfig::System::FFT_SIZE_CONST);
    dsps_bit_rev2r_fc32(_fftSpatialL, SmeaConfig::System::FFT_SIZE_CONST);
    dsps_fft2r_fc32(_fftSpatialR, SmeaConfig::System::FFT_SIZE_CONST);
    dsps_bit_rev2r_fc32(_fftSpatialR, SmeaConfig::System::FFT_SIZE_CONST);

    uint16_t v_bins = (SmeaConfig::System::FFT_SIZE_CONST / 2) + 1;
    float v_sumIpd = 0.0f, v_coherenceSum = 0.0f, v_energySum = 0.0f;

    uint16_t v_startBin = (uint16_t)(v_cfg.feature.spatial_freq_min_hz * SmeaConfig::System::FFT_SIZE_CONST / SmeaConfig::System::SAMPLING_RATE_CONST);
    uint16_t v_endBin = (uint16_t)(v_cfg.feature.spatial_freq_max_hz * SmeaConfig::System::FFT_SIZE_CONST / SmeaConfig::System::SAMPLING_RATE_CONST);
    if (v_endBin > v_bins) v_endBin = v_bins;

    for (uint16_t i = v_startBin; i < v_endBin; i++) {
        float v_reL = _fftSpatialL[i * 2], v_imL = _fftSpatialL[i * 2 + 1];
        float v_reR = _fftSpatialR[i * 2], v_imR = -_fftSpatialR[i * 2 + 1];

        float v_crossRe = v_reL * v_reR - v_imL * v_imR;
        float v_crossIm = v_reL * v_imR + v_imL * v_reR;

        float v_phaseDiff = atan2f(v_crossIm, v_crossRe);
        float v_binEnergy = (v_reL*v_reL + v_imL*v_imL) + (v_reR*v_reR + v_imR*v_imR);

        v_sumIpd += fabsf(v_phaseDiff) * v_binEnergy;
        v_coherenceSum += cosf(v_phaseDiff) * v_binEnergy;
        v_energySum += v_binEnergy;
    }

    if (v_energySum > SmeaConfig::System::MATH_EPSILON_12_CONST) {
        p_slot.mean_ipd = v_sumIpd / v_energySum;
        p_slot.phase_coherence = (v_coherenceSum / v_energySum + 1.0f) / 2.0f;
    } else {
        p_slot.mean_ipd = 0.0f; p_slot.phase_coherence = 1.0f;
    }
}

void T440_FeatureExtractor::_computeBandRMS(SmeaType::FeatureSlot& p_slot) {
    float v_binRes = (float)SmeaConfig::System::SAMPLING_RATE_CONST / SmeaConfig::System::FFT_SIZE_CONST;
    uint16_t v_maxBin = (SmeaConfig::System::FFT_SIZE_CONST / 2) + 1;
    DynamicConfig v_cfg = _cachedCfg;

    for (uint8_t b = 0; b < v_cfg.feature.band_rms_count; b++) {
        uint16_t v_startBin = (uint16_t)(v_cfg.feature.band_ranges[b][0] / v_binRes);
        uint16_t v_endBin = (uint16_t)(v_cfg.feature.band_ranges[b][1] / v_binRes);

        float v_sumSq = 0.0f;
        for (uint16_t i = v_startBin; i <= v_endBin && i < v_maxBin; i++) {
            v_sumSq += _powerSpectrum[i];
        }
        p_slot.band_rms[b] = sqrtf(v_sumSq); 
    }
}

void T440_FeatureExtractor::resetNoiseProfile() {
    memset(_noiseProfile, 0, sizeof(_noiseProfile));
    _noiseLearnedFrames = 0;
    ESP_LOGI(TAG, "Noise Profile Hard-Reset (Mathematical Integrity Ensured)");
}

void T440_FeatureExtractor::reloadConfigCache() {
    _cachedCfg = T415_ConfigManager::getInstance().getConfig();
}

