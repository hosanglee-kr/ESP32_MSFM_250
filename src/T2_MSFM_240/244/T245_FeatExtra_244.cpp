/* ============================================================================
 * File: T245_FeatExtra_244.hpp
 * Summary: 멀티모달(진동/소음) 융합 특징량 추출 엔진 구현부 (v243)
 * ========================================================================== */
#include "T245_FeatExtra_244.hpp"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "dspm_mult.h"
#include <cmath>
#include <algorithm>
#include <cstring>
#include <vector>

static const char* TAG = "T243_FEAT";

CL_T2_FeatureExtractor::CL_T2_FeatureExtractor()
    : _melBankFlat(nullptr), _dctMatrixFlat(nullptr), _fftWorkAudio(nullptr),
      _powerAudio(nullptr), _windowAudio(nullptr), _noiseProfile(nullptr),
      _fftWorkVib(nullptr), _powerVib(nullptr), _melScratchBuf(nullptr), _dctScratchBuf(nullptr),
      _mfccHistory(nullptr), _deltaHistory(nullptr),
      _learnedFrames(0), _isLearning(false), _isInitialized(false) {
    memset(_historyCount, 0, sizeof(_historyCount));
}

CL_T2_FeatureExtractor::~CL_T2_FeatureExtractor() {
    if (_melBankFlat) heap_caps_free(_melBankFlat);
    if (_dctMatrixFlat) heap_caps_free(_dctMatrixFlat);
    if (_fftWorkAudio) heap_caps_free(_fftWorkAudio);
    if (_powerAudio) heap_caps_free(_powerAudio);
    if (_windowAudio) heap_caps_free(_windowAudio);
    if (_noiseProfile) heap_caps_free(_noiseProfile);
    if (_fftWorkVib) heap_caps_free(_fftWorkVib);
    if (_powerVib) heap_caps_free(_powerVib);
    if (_melScratchBuf) heap_caps_free(_melScratchBuf);
    if (_dctScratchBuf) heap_caps_free(_dctScratchBuf);
    if (_mfccHistory) heap_caps_free(_mfccHistory);
    if (_deltaHistory) heap_caps_free(_deltaHistory);
}

bool CL_T2_FeatureExtractor::init() {
    // [1] PSRAM 동적 할당 (SIMD 정렬 보장, _noiseProfile은 L/R 듀얼 마이크 관리를 위해 2배 크기로 확장)
    _melBankFlat = (float*)heap_caps_aligned_alloc(16, BINS_PADDED * MEL_PADDED * sizeof(float), MALLOC_CAP_SPIRAM);
    _dctMatrixFlat = (float*)heap_caps_aligned_alloc(16, MEL_PADDED * MFCC_PADDED * sizeof(float), MALLOC_CAP_SPIRAM);
    _fftWorkAudio = (float*)heap_caps_aligned_alloc(16, T2_Def::Audio::Sensor::FFT_SIZE_MAX * 2 * sizeof(float), MALLOC_CAP_SPIRAM);
    _powerAudio = (float*)heap_caps_aligned_alloc(16, BINS_PADDED * sizeof(float), MALLOC_CAP_SPIRAM);
    _windowAudio = (float*)heap_caps_aligned_alloc(16, T2_Def::Audio::Sensor::FFT_SIZE_MAX * sizeof(float), MALLOC_CAP_SPIRAM);
    _noiseProfile = (float*)heap_caps_aligned_alloc(16, 2 * BINS_PADDED * sizeof(float), MALLOC_CAP_SPIRAM);

    // [2] SRAM/PSRAM 혼합 할당 (DRAM 오버플로우 방지, MFCC 히스토리는 5채널 격상)
    _fftWorkVib    = (float*)heap_caps_aligned_alloc(16, T2_Def::Vib::Sensor::FFT_SIZE_MAX * 2 * sizeof(float), MALLOC_CAP_INTERNAL);
    _powerVib      = (float*)heap_caps_aligned_alloc(16, T2_Def::Vib::Sensor::FFT_SIZE_MAX * sizeof(float), MALLOC_CAP_INTERNAL);
    _melScratchBuf = (float*)heap_caps_aligned_alloc(16, T2_Def::Audio::FeatureLimit::MEL_CHUNK_ROWS_CONST * MEL_PADDED * sizeof(float), MALLOC_CAP_SPIRAM);
    _dctScratchBuf = (float*)heap_caps_aligned_alloc(16, MEL_PADDED * MFCC_PADDED * sizeof(float), MALLOC_CAP_SPIRAM);
    _mfccHistory   = (float (*)[T2_Def::Shared::FeatureLimit::DELTA_HISTORY_MAX][MFCC_PADDED])heap_caps_aligned_alloc(16, sizeof(float) * 5 * T2_Def::Shared::FeatureLimit::DELTA_HISTORY_MAX * MFCC_PADDED, MALLOC_CAP_SPIRAM);
    _deltaHistory  = (float (*)[T2_Def::Shared::FeatureLimit::DELTA_HISTORY_MAX][MFCC_PADDED])heap_caps_aligned_alloc(16, sizeof(float) * 5 * T2_Def::Shared::FeatureLimit::DELTA_HISTORY_MAX * MFCC_PADDED, MALLOC_CAP_SPIRAM);

    if (!_melBankFlat || !_dctMatrixFlat || !_fftWorkAudio || !_powerAudio || !_windowAudio || !_noiseProfile ||
        !_fftWorkVib || !_powerVib || !_melScratchBuf || !_dctScratchBuf || !_mfccHistory || !_deltaHistory) {
        ESP_LOGE(TAG, "Memory Allocation Failed (Hybrid RAM Strategy)");
        return false;
    }

    memset(_mfccHistory, 0, sizeof(float) * 5 * T2_Def::Shared::FeatureLimit::DELTA_HISTORY_MAX * MFCC_PADDED);
    memset(_deltaHistory, 0, sizeof(float) * 5 * T2_Def::Shared::FeatureLimit::DELTA_HISTORY_MAX * MFCC_PADDED);

    // [3] 초기화 및 필터뱅크 생성
    memset(_melBankFlat, 0, BINS_PADDED * MEL_PADDED * sizeof(float));
    memset(_dctMatrixFlat, 0, MEL_PADDED * MFCC_PADDED * sizeof(float));
    memset(_noiseProfile, 0, 2 * BINS_PADDED * sizeof(float));

    // Mel Bank 초기화 (삼각형 필터 - 간략화된 선형 생성)
    float v_lowFreq = 100.0f;
    float v_highFreq = 8000.0f;
    float v_lowMel = 1127.0f * log1p(v_lowFreq / 700.0f);
    float v_highMel = 1127.0f * log1p(v_highFreq / 700.0f);

    for (int m = 0; m < MEL_BANDS_MAX; m++) {
        float v_mCenter = v_lowMel + (v_highMel - v_lowMel) * (m + 1) / (MEL_BANDS_MAX + 1);
        float v_mLeft   = v_lowMel + (v_highMel - v_lowMel) * m / (MEL_BANDS_MAX + 1);
        float v_mRight  = v_lowMel + (v_highMel - v_lowMel) * (m + 2) / (MEL_BANDS_MAX + 1);

        // Mel to Hz 역산
        float v_hzC = 700.0f * (expm1(v_mCenter / 1127.0f));
        float v_hzL = 700.0f * (expm1(v_mLeft / 1127.0f));
        float v_hzR = 700.0f * (expm1(v_mRight / 1127.0f));

        for (int b = 0; b < BINS_MAX; b++) {
            float v_hzB = (float)b * (22050.0f / BINS_MAX); // 44.1kHz 기준 가정 (런타임에 보정 가능)
            float v_weight = 0.0f;
            if (v_hzB >= v_hzL && v_hzB <= v_hzC) v_weight = (v_hzB - v_hzL) / (v_hzC - v_hzL);
            else if (v_hzB > v_hzC && v_hzB <= v_hzR) v_weight = (v_hzR - v_hzB) / (v_hzR - v_hzC);
            _melBankFlat[b * MEL_PADDED + m] = v_weight;
        }
    }

    // DCT Matrix 초기화
    for (int i = 0; i < MFCC_DIM_MAX; i++) {
        for (int j = 0; j < MEL_BANDS_MAX; j++) {
            _dctMatrixFlat[j * MFCC_PADDED + i] = cos(M_PI * i * (j + 0.5f) / MEL_BANDS_MAX);
        }
    }

    _isInitialized = true;
    ESP_LOGI(TAG, "Feature Engine v243 Initialized (PSRAM Allocated)");
    return true;
}

void CL_T2_FeatureExtractor::extractVibration(const float* p_vibX, const float* p_vibY, const float* p_vibZ, uint32_t p_len, uint32_t p_sampleRate, T2_Type::UnifiedFeatureSlot& p_slot, const T2_Type::ST_Vib_Feature& p_featCfg, const T2_Type::ST_Vib_Trigger& p_trigCfg, uint8_t p_axisCount) {
    if (!_isInitialized) return;

    const float* v_in[3] = {p_vibX, p_vibY, p_vibZ};

    int v_loops = (p_axisCount > 3) ? 3 : p_axisCount;
    for (int i = 0; i < v_loops; i++) {
        if (!v_in[i]) continue;

        // [Step 1] [3축 개정] 각 축별 통계 지표 추출 (RMS, Kurtosis, Crest Factor, Skewness, STD)
        _computeVibStats(v_in[i], p_len, p_slot.vib.rms[i], p_slot.vib.kurt[i], p_slot.vib.crest[i], p_slot.vib.skew[i], p_slot.vib.std[i]);

        // [Step 2] 주파수 도메인 변환 (FFT)
        memset(_fftWorkVib, 0, T2_Def::Vib::Sensor::FFT_SIZE_MAX * 2 * sizeof(float));
        for (uint32_t j = 0; j < p_len; j++) {
            _fftWorkVib[j * 2] = v_in[i][j];
            _fftWorkVib[j * 2 + 1] = 0;
        }
        dsps_fft2r_fc32(_fftWorkVib, p_len);
        dsps_bit_rev_fc32(_fftWorkVib, p_len);

        uint32_t v_bins = (p_len / 2) + 1;
        for (uint32_t j = 0; j < v_bins; j++) {
            float re = _fftWorkVib[j * 2];
            float im = _fftWorkVib[j * 2 + 1];
            _powerVib[j] = (re * re + im * im) / p_len;
        }

        // [Step 3] [3축 개정] 축별 밴드 에너지 추출 및 다차원 배열 저장
        float v_currentBands[T2_Def::Shared::FeatureLimit::BAND_RMS_MAX] = {0};
        _computeBandEnergies(_powerVib, v_bins, (float)p_sampleRate, p_trigCfg.active_band_count, p_trigCfg.band_en, p_trigCfg.band_start, p_trigCfg.band_end, v_currentBands);

        // 각 축 i에 대해 추출된 밴드 에너지를 직접 복사!
        memcpy(p_slot.vib.band_energy[i], v_currentBands, sizeof(v_currentBands));

        // [Step 4] 진동 MFCC 추출 (채널 0, 1, 2)
        _computeMfcc(_powerVib, p_slot.tensor.mfcc, i);
    }
}

void CL_T2_FeatureExtractor::extractAudio(const float* p_audL, const float* p_audR, uint32_t p_len, uint32_t p_sampleRate,
                                          T2_Type::UnifiedFeatureSlot& p_slot, const T2_Type::ST_Audio_Feature& p_featCfg, const T2_Type::ST_Audio_Trigger& p_trigCfg) {
    if (!_isInitialized) return;

    uint32_t v_len = (p_len > T2_Def::Audio::Sensor::FFT_SIZE_MAX) ? T2_Def::Audio::Sensor::FFT_SIZE_MAX : p_len;
    uint32_t v_bins = v_len / 2 + 1;

    // Left/Right FFT 결과 저장을 위한 로컬 스택 버퍼 (교차 상관성 및 위상차 계산용)
    alignas(16) float v_fftLeft[T2_Def::Audio::Sensor::FFT_SIZE_MAX * 2] = {0};
    alignas(16) float v_fftRight[T2_Def::Audio::Sensor::FFT_SIZE_MAX * 2] = {0};

    // --- LEFT 채널 특징 추출 (chIdx = 0) ---
    if (p_audL) {
        // [Step 1] 통계 특징량
        float v_unusedStdL = 0.0f;
        _computeVibStats(p_audL, v_len, p_slot.audio.ch[0].rms, p_slot.audio.ch[0].kurt, p_slot.audio.ch[0].crest, p_slot.audio.ch[0].skew, v_unusedStdL);

        // [Step 2] FFT 및 Power Spectrum
        dsps_wind_hann_f32(_windowAudio, v_len);
        for (uint32_t i = 0; i < v_len; i++) {
            v_fftLeft[i * 2] = p_audL[i] * _windowAudio[i];
            v_fftLeft[i * 2 + 1] = 0;
        }
        dsps_fft2r_fc32(v_fftLeft, v_len);
        dsps_bit_rev2r_fc32(v_fftLeft, v_len);

        for (uint32_t i = 0; i < v_bins; i++) {
            float re = v_fftLeft[i * 2];
            float im = v_fftLeft[i * 2 + 1];
            _powerAudio[i] = (re * re + im * im) / v_len;
        }

        // [Step 3] 적응형 노이즈 프로필 학습 및 감산 (Left: offset = 0)
        if (_isLearning) {
            float v_alpha = T2_Def::Audio::Dsp::NOISE_LEARN_ALPHA_DEF;
            for (uint32_t i = 0; i < v_bins; i++) {
                _noiseProfile[i] = (v_alpha * _powerAudio[i]) + ((1.0f - v_alpha) * _noiseProfile[i]);
            }
            if (_learnedFrames < UINT32_MAX) _learnedFrames++;
        } else if (_learnedFrames > 0) {
            float v_subStr = T2_Def::Audio::Dsp::SPECTRAL_SUB_GAIN_DEF;
            for (uint32_t i = 0; i < v_bins; i++) {
                _powerAudio[i] = fmaxf(_powerAudio[i] - (_noiseProfile[i] * v_subStr), T2_Def::Global::System::MATH_EPSILON_12_CONST);
            }
        }

        // Spectral Centroid & Peaks & Band Energies & MFCC
        p_slot.audio.ch[0].centroid = _computeSpectralCentroid(_powerAudio, v_bins, (float)p_sampleRate);
        _extractTopPeaks(_powerAudio, v_bins, (float)p_sampleRate, p_slot.audio.ch[0].top_peaks, T2_Def::Shared::FeatureLimit::TOP_PEAKS_MAX, p_featCfg.peak_amp_min, p_featCfg.peak_freq_gap_min);
        _computeBandEnergies(_powerAudio, v_bins, (float)p_sampleRate, p_trigCfg.active_band_count, p_trigCfg.band_en, p_trigCfg.band_start, p_trigCfg.band_end, p_slot.audio.ch[0].band_energy);
        _computeMfcc(_powerAudio, p_slot.tensor.mfcc, 3); // Left MFCC -> Channel index 3
    }

    // --- RIGHT 채널 특징 추출 (chIdx = 1) ---
    if (p_audR) {
        // [Step 1] 통계 특징량
        float v_unusedStdR = 0.0f;
        _computeVibStats(p_audR, v_len, p_slot.audio.ch[1].rms, p_slot.audio.ch[1].kurt, p_slot.audio.ch[1].crest, p_slot.audio.ch[1].skew, v_unusedStdR);

        // [Step 2] FFT 및 Power Spectrum
        dsps_wind_hann_f32(_windowAudio, v_len);
        for (uint32_t i = 0; i < v_len; i++) {
            v_fftRight[i * 2] = p_audR[i] * _windowAudio[i];
            v_fftRight[i * 2 + 1] = 0;
        }
        dsps_fft2r_fc32(v_fftRight, v_len);
        dsps_bit_rev2r_fc32(v_fftRight, v_len);

        for (uint32_t i = 0; i < v_bins; i++) {
            float re = v_fftRight[i * 2];
            float im = v_fftRight[i * 2 + 1];
            _powerAudio[i] = (re * re + im * im) / v_len; // _powerAudio 공유 재사용
        }

        // [Step 3] 적응형 노이즈 프로필 학습 및 감산 (Right: offset = BINS_PADDED)
        if (_isLearning) {
            float v_alpha = T2_Def::Audio::Dsp::NOISE_LEARN_ALPHA_DEF;
            for (uint32_t i = 0; i < v_bins; i++) {
                _noiseProfile[BINS_PADDED + i] = (v_alpha * _powerAudio[i]) + ((1.0f - v_alpha) * _noiseProfile[BINS_PADDED + i]);
            }
        } else if (_learnedFrames > 0) {
            float v_subStr = T2_Def::Audio::Dsp::SPECTRAL_SUB_GAIN_DEF;
            for (uint32_t i = 0; i < v_bins; i++) {
                _powerAudio[i] = fmaxf(_powerAudio[i] - (_noiseProfile[BINS_PADDED + i] * v_subStr), T2_Def::Global::System::MATH_EPSILON_12_CONST);
            }
        }

        // Spectral Centroid & Peaks & Band Energies & MFCC
        p_slot.audio.ch[1].centroid = _computeSpectralCentroid(_powerAudio, v_bins, (float)p_sampleRate);
        _extractTopPeaks(_powerAudio, v_bins, (float)p_sampleRate, p_slot.audio.ch[1].top_peaks, T2_Def::Shared::FeatureLimit::TOP_PEAKS_MAX, p_featCfg.peak_amp_min, p_featCfg.peak_freq_gap_min);
        _computeBandEnergies(_powerAudio, v_bins, (float)p_sampleRate, p_trigCfg.active_band_count, p_trigCfg.band_en, p_trigCfg.band_start, p_trigCfg.band_end, p_slot.audio.ch[1].band_energy);
        _computeMfcc(_powerAudio, p_slot.tensor.mfcc, 4); // Right MFCC -> Channel index 4
    }

    // --- 공간 결함성 지표 (L/R 교차 상관 분석) ---
    if (p_audL && p_audR) {
        float v_cohSum = 0;
        float v_ipdSum = 0;
        uint32_t v_cohCount = 0;

        for (uint32_t i = 1; i < v_bins - 1; i++) {
            float L_re = v_fftLeft[i * 2], L_im = v_fftLeft[i * 2 + 1];
            float R_re = v_fftRight[i * 2], R_im = v_fftRight[i * 2 + 1];

            // Cross power spectral density (L * conj(R))
            float C_re = L_re * R_re + L_im * R_im;
            float C_im = L_im * R_re - L_re * R_im;

            float L_pow = L_re * L_re + L_im * L_im;
            float R_pow = R_re * R_re + R_im * R_im;
            float denom = sqrtf(L_pow * R_pow);

            if (denom > T2_Def::Global::System::MATH_EPSILON_12_CONST) {
                float coh = sqrtf((C_re * C_re + C_im * C_im)) / denom;
                v_cohSum += coh;
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

void CL_T2_FeatureExtractor::_computeVibStats(const float* p_data, uint32_t p_len, float& p_rms, float& p_kurt, float& p_crest, float& p_skew, float& p_std) {
    float v_sum = 0, v_sqSum = 0;
    float v_maxAbs = 0;

    for (uint32_t i = 0; i < p_len; i++) {
        float val = p_data[i];
        v_sum += val;
        v_sqSum += val * val;
        if (std::abs(val) > v_maxAbs) v_maxAbs = std::abs(val);
    }

    float v_mean = v_sum / p_len;
    p_rms = sqrt(v_sqSum / p_len);

    float v_varSum = 0, v_kurtSum = 0, v_skewSum = 0;
    for (uint32_t i = 0; i < p_len; i++) {
        float diff = p_data[i] - v_mean;
        float d2 = diff * diff;
        v_varSum += d2;
        v_skewSum += d2 * diff;
        v_kurtSum += d2 * d2;
    }

    float v_var = v_varSum / p_len;
    p_std = sqrt(v_var);

    float v_denom = fmaxf(p_std * p_std, T2_Def::Global::System::MATH_EPSILON_12_CONST);
    p_skew = (v_skewSum / p_len) / (v_denom * p_std);
    p_kurt = (v_kurtSum / p_len) / (v_denom * v_denom);
    p_crest = v_maxAbs / fmaxf(p_rms, T2_Def::Global::System::MATH_EPSILON_CONST);
}

float CL_T2_FeatureExtractor::_computeSpectralCentroid(const float* p_power, uint32_t p_bins, float p_sampleRate) {
    float v_num = 0, v_den = 0;
    for (uint32_t i = 0; i < p_bins; i++) {
        float freq = i * (p_sampleRate / 2.0f) / (p_bins - 1);
        v_num += freq * p_power[i];
        v_den += p_power[i];
    }
    return v_num / fmaxf(v_den, T2_Def::Global::System::MATH_EPSILON_CONST);
}

void CL_T2_FeatureExtractor::_extractTopPeaks(const float* p_power, uint32_t p_bins, float p_sampleRate, T2_Type::SpectralPeak* p_outPeaks, uint8_t p_maxCount, float p_ampMin, float p_freqGap) {
    struct Peak { float f; float a; };
    std::vector<Peak> v_candidates;

    for (uint32_t i = 1; i < p_bins - 1; i++) {
        if (p_power[i] > p_power[i-1] && p_power[i] > p_power[i+1] && p_power[i] > p_ampMin) {
            float freq = i * (p_sampleRate / 2.0f) / (p_bins - 1);
            v_candidates.push_back({freq, p_power[i]});
        }
    }

    std::sort(v_candidates.begin(), v_candidates.end(), [](const Peak& a, const Peak& b){ return a.a > b.a; });

    uint8_t count = 0;
    for (const auto& p : v_candidates) {
        if (count >= p_maxCount) break;
        bool tooClose = false;
        for (int j = 0; j < count; j++) {
            if (std::abs(p.f - p_outPeaks[j].freq) < p_freqGap) { tooClose = true; break; }
        }
        if (!tooClose) {
            p_outPeaks[count].freq = p.f;
            p_outPeaks[count].amp = p.a;
            count++;
        }
    }
    for (; count < p_maxCount; count++) { p_outPeaks[count].freq = 0; p_outPeaks[count].amp = 0; }
}

void CL_T2_FeatureExtractor::_computeBandEnergies(const float* p_power, uint32_t p_bins, float p_sampleRate, uint8_t p_count, const bool* p_en, const float* p_start, const float* p_end, float* p_outEnergies) {
    float v_binHz = (p_sampleRate / 2.0f) / (p_bins - 1);

    for (uint8_t i = 0; i < p_count; i++) {
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

void CL_T2_FeatureExtractor::_computeMfcc(const float* p_power, float* p_outMfcc, uint8_t p_chIdx) {
    if (p_chIdx >= 5) return;

    float v_melEnergies[MEL_BANDS_MAX] = {0};

    // Matrix Mult (Power * MelFilterbank)
    for (int k = 0; k < BINS_PADDED; k += T2_Def::Audio::FeatureLimit::MEL_CHUNK_ROWS_CONST) {
        int v_currRows = fminf(T2_Def::Audio::FeatureLimit::MEL_CHUNK_ROWS_CONST, BINS_PADDED - k);
        memcpy(_melScratchBuf, &_melBankFlat[k * MEL_PADDED], v_currRows * MEL_PADDED * sizeof(float));
        alignas(16) float v_tempOut[MEL_PADDED] = {0};
        dspm_mult_f32(&p_power[k], _melScratchBuf, v_tempOut, 1, v_currRows, MEL_PADDED);
        for (int m = 0; m < MEL_BANDS_MAX; m++) v_melEnergies[m] += v_tempOut[m];
    }

    // Log Energy
    for (int i = 0; i < MEL_BANDS_MAX; i++) {
        v_melEnergies[i] = log10f(fmaxf(v_melEnergies[i], T2_Def::Global::System::MATH_EPSILON_12_CONST));
    }

    // DCT Matrix 연산
    alignas(16) float v_dctOut[MFCC_PADDED] = {0};
    memcpy(_dctScratchBuf, _dctMatrixFlat, MEL_PADDED * MFCC_PADDED * sizeof(float));
    dspm_mult_f32(v_melEnergies, _dctScratchBuf, v_dctOut, 1, MEL_PADDED, MFCC_PADDED);

    uint16_t v_dim = T2_Def::Audio::FeatureLimit::MFCC_COEFFS_DEF;
    uint16_t& v_chCnt = _historyCount[p_chIdx];

    // History Update (Rolling)
    if (v_chCnt < T2_Def::Shared::FeatureLimit::DELTA_HISTORY_MAX) {
        memcpy(_mfccHistory[p_chIdx][v_chCnt], v_dctOut, v_dim * sizeof(float));
        v_chCnt++;
    } else {
        for (int i = 0; i < T2_Def::Shared::FeatureLimit::DELTA_HISTORY_MAX - 1; i++) {
            memcpy(_mfccHistory[p_chIdx][i], _mfccHistory[p_chIdx][i+1], v_dim * sizeof(float));
        }
        memcpy(_mfccHistory[p_chIdx][T2_Def::Shared::FeatureLimit::DELTA_HISTORY_MAX - 1], v_dctOut, v_dim * sizeof(float));
    }

    // 채널별 출력 위치 오프셋 계산 (96차원 단위)
    float* v_finalOut = &p_outMfcc[p_chIdx * T2_Def::Audio::FeatureLimit::MFCC_DIM_MAX];

    // Static
    memcpy(v_finalOut, v_dctOut, v_dim * sizeof(float));

    if (v_chCnt >= T2_Def::Shared::FeatureLimit::DELTA_HISTORY_MAX) {
        // Delta
        for (int i = 0; i < v_dim; i++) {
            v_finalOut[v_dim + i] = (_mfccHistory[p_chIdx][4][i] - _mfccHistory[p_chIdx][0][i]) / 4.0f;
        }
        // History Delta Update
        for (int i = 0; i < T2_Def::Shared::FeatureLimit::DELTA_HISTORY_MAX - 1; i++) {
            memcpy(_deltaHistory[p_chIdx][i], _deltaHistory[p_chIdx][i+1], v_dim * sizeof(float));
        }
        memcpy(_deltaHistory[p_chIdx][T2_Def::Shared::FeatureLimit::DELTA_HISTORY_MAX - 1], &v_finalOut[v_dim], v_dim * sizeof(float));

        // Delta-Delta
        for (int i = 0; i < v_dim; i++) {
            v_finalOut[v_dim * 2 + i] = (_deltaHistory[p_chIdx][4][i] - _deltaHistory[p_chIdx][0][i]) / 4.0f;
        }
    } else {
        memset(&v_finalOut[v_dim], 0, v_dim * 2 * sizeof(float));
    }
}

void CL_T2_FeatureExtractor::setNoiseLearning(bool p_enable) {
    _isLearning = p_enable;
    if (_isLearning) _learnedFrames = 0;
}

void CL_T2_FeatureExtractor::resetNoiseProfile() {
    if (_noiseProfile) memset(_noiseProfile, 0, 2 * BINS_PADDED * sizeof(float));
    _learnedFrames = 0;
}

void CL_T2_FeatureExtractor::resetHistory() {
    memset(_historyCount, 0, sizeof(_historyCount));
    memset(_mfccHistory, 0, sizeof(float) * 5 * T2_Def::Shared::FeatureLimit::DELTA_HISTORY_MAX * MFCC_PADDED);
    memset(_deltaHistory, 0, sizeof(float) * 5 * T2_Def::Shared::FeatureLimit::DELTA_HISTORY_MAX * MFCC_PADDED);
}
