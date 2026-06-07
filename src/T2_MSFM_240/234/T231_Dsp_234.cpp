/*
============================================================================
 * File: T231_Dsp_234.cpp
 * Summary: MFCC & DSP Pipeline Engine Implementation (SIMD Optimized)
 * * * * [AI 셀프 회고 및 구현 원칙 - 실수 방지 유형화 목록] * * *
 * * [유형 1: 하드웨어(SIMD) 가속기 및 아키텍처 몰이해]
 * - 실수: 128-bit 행렬 가속기(dspm)가 4개의 Float를 한 번에 읽는 특성(Loop Unrolling)을 간과하여 Over-read 패닉 유발.
 * - 원칙: 가속기에 들어가는 모든 버퍼(Power, Mel, DCT)의 2D 차원(Row/Col)은 반드시 16바이트(4의 배수)로 패딩(Padding)하여 물리적으로 할당할 것. (bins_padded, mel_padded, mfcc_padded)
 * - 실수: ESP-DSP 필터 함수의 제자리(In-place) 연산 오염(UB)을 간과함.
 * - 원칙: FIR/IIR 필터 적용 시 입출력을 동일 버퍼로 주지 말고, 반드시 임시 버퍼(_fft_io_buf)를 활용해 핑퐁(Ping-pong) 교차 연산할 것.
 * * [유형 2: C++ 컴파일러 최적화 및 메모리 생명주기 맹점]
 * - 실수: -ffast-math 최적화 시 isnan(), isinf()가 강제 삭제되는 컴파일러 함정을 간과함.
 * - 원칙: NaN/Inf 유입 방어는 반드시 IEEE-754 지수부 비트 마스킹(0x7F800000)으로 하드웨어 레벨에서 직접 검사할 것.
 * - 실수: 동적 할당 재진입 시 변수 갱신만 발생하고 메모리는 재할당되지 않는 맹점 발생.
 * - 원칙: fft_size가 변하지 않더라도 mfcc_coeffs 등 차원 변수가 변경될 때 이전 데이터의 쓰레기값(Garbage)이 간섭하지 못하도록, 가변 행렬은 begin() 시점마다 최대 크기 기준으로 일괄 memset 초기화를 수행할 것.
 * * [유형 3: DSP 수학적 정합성 및 시계열(Time-series) 훼손]
 * - 실수: 5/7 Tap Median 필터를 In-place로 덮어써, 미래의 연산이 덮어써진 과거를 참조하는 재귀적(IIR) 위상 왜곡 유발.
 * - 원칙: 과거 참조 윈도우는 반드시 원본 링버퍼 캐시(orig_history)를 도입하여 격리할 것.
 * - 실수: 오버랩(Overlap) 환경의 Pre-emphasis 연산에서 이전 프레임의 마지막 샘플을 무리하게 이월(Carry-over)하여 경계면 파열음(Pop/Click) 유발.
 * - 원칙: 오버랩 프레임은 시계열이 연속적이지 않으므로, 버퍼 할당 없이 역순(Reverse) 루프를 돌며 프레임 내 독립 연산을 수행할 것.
 * - 실수: FFT 결과물 스케일링 누락 및 Sinc 함수 분모 0 나누기 예외 처리 미흡.
 * - 원칙: FFT 결과는 반드시 1/N 스케일링을 적용해 에너지 임계값 왜곡을 막고, 0.0f 검사는 Epsilon(fabsf(x) < 1e-6f)을 사용할 것.
 * * [유형 4: 부주의한 복붙(Copy-Paste) 및 데이터 꼬임]
 * - 실수: 다차원 배열(MFCC History)을 1D 배열 밀어내듯 memmove하여 행/열 스트라이드(Stride) 꼬임 파괴 유발.
 * - 원칙: 2D 배열 복사는 반드시 행 단위(Row-by-row) 개별 memcpy를 수행하여 시계열 데이터 파괴를 막을 것.
 * - 실수: FIR HPF 초기화 블록이 LPF의 설정 변수를 참조.
 * - 원칙: 코어 파이프라인 블록은 작성 후 변수 스코프와 참조 대상을 3회 이상 교차 검증할 것.
 * ========================================================================== 
 */

#include "T231_Dsp_234.h"

#include <math.h>
#include <string.h>

CL_T20_DspPipeline::CL_T20_DspPipeline() {
    memset(_hpf_state, 0, sizeof(_hpf_state));
    memset(_lpf_state, 0, sizeof(_lpf_state));
    memset(_notch_state, 0, sizeof(_notch_state));
    memset(_history_count, 0, sizeof(_history_count));
    memset(_noise_learned_frames, 0, sizeof(_noise_learned_frames));
    _current_fft_size = 0;
}

CL_T20_DspPipeline::~CL_T20_DspPipeline() {
    _freeBuffers();
}

void CL_T20_DspPipeline::_freeBuffers() {
    auto safe_free = [](void* ptr) { if (ptr) heap_caps_free(ptr); };

    safe_free(_work_frame);     _work_frame     = nullptr;
    safe_free(_window);         _window         = nullptr;
    safe_free(_power);          _power          = nullptr;
    safe_free(_noise_spectrum); _noise_spectrum = nullptr;
    safe_free(_fft_io_buf);     _fft_io_buf     = nullptr;
    safe_free(_mel_bank_flat);  _mel_bank_flat  = nullptr;
    safe_free(_dct_matrix_flat); _dct_matrix_flat = nullptr;
}

bool CL_T20_DspPipeline::begin(const ST_T20_Config_t& cfg) {
    _cfg = cfg;

    const uint16_t N = (uint16_t)_cfg.feature.fft_size;
    // [수학적 한계 방어] FFT 길이는 반드시 2의 제곱수여야 함 (ESP-DSP 요구사항)
    if ((N & (N - 1)) != 0 || N == 0) return false;

    const uint16_t bins = (N / 2) + 1;
    const float fs = T20::C10_DSP::SAMPLE_RATE_HZ;
    
    // [하드웨어 패닉 방어] MFCC 차원 최소/최대 한계 클램핑
    uint16_t mfcc_dim = _cfg.feature.mfcc_coeffs;
    if (mfcc_dim < 1) mfcc_dim = 1; 
    if (mfcc_dim > T20::C10_DSP::MFCC_COEFFS_MAX) {
        mfcc_dim = T20::C10_DSP::MFCC_COEFFS_MAX;
    }
    _cfg.feature.mfcc_coeffs = mfcc_dim; 
    
    // [SIMD Over-read 패닉 방어] 128비트 단위 초과 읽기를 막기 위한 패딩(Padding) 상수
    const uint16_t bins_padded  = (bins + 3) & ~3;
    const uint16_t mel_padded   = (T20::C10_DSP::MEL_FILTERS + 3) & ~3;
    const uint16_t mfcc_padded  = (mfcc_dim + 3) & ~3;
    const uint16_t mfcc_max_padded = (T20::C10_DSP::MFCC_COEFFS_MAX + 3) & ~3;

    // [1] 동적 버퍼 재할당 및 OOM 롤백 방어
    if (_current_fft_size != N) {
        Serial.printf("[DSP] Re-allocating internal buffers for FFT Size: %d\n", N);
        _freeBuffers();
        _current_fft_size = N;

        _work_frame      = (float*)heap_caps_aligned_alloc(16, N * sizeof(float), MALLOC_CAP_INTERNAL);
        _window          = (float*)heap_caps_aligned_alloc(16, N * sizeof(float), MALLOC_CAP_INTERNAL);
        _power           = (float*)heap_caps_aligned_alloc(16, bins_padded * sizeof(float), MALLOC_CAP_INTERNAL);
        _fft_io_buf      = (float*)heap_caps_aligned_alloc(16, N * 2 * sizeof(float), MALLOC_CAP_INTERNAL);
        _noise_spectrum  = (float*)heap_caps_aligned_alloc(16, 3 * bins_padded * sizeof(float), MALLOC_CAP_INTERNAL);
        
        _mel_bank_flat   = (float*)heap_caps_aligned_alloc(16, bins_padded * mel_padded * sizeof(float), MALLOC_CAP_INTERNAL);
        _dct_matrix_flat = (float*)heap_caps_aligned_alloc(16, mel_padded * mfcc_max_padded * sizeof(float), MALLOC_CAP_INTERNAL);

        if (!_work_frame || !_window || !_power || !_fft_io_buf || !_mel_bank_flat || !_noise_spectrum || !_dct_matrix_flat) {
            Serial.println(F("[DSP] Critical: Internal SRAM OOM!"));
            _freeBuffers(); // 일부 할당 실패 시 영구 누수를 막기 위한 즉각 롤백
            return false;
        }
        if (dsps_fft2r_init_fc32(NULL, N) != ESP_OK) return false;
    }

    // [메모리 파편화 방어] fft_size가 변하지 않고 차원(mfcc_coeffs)만 런타임에 변경될 경우, 
    // 이전 행렬의 쓰레기값이 연산에 간섭하는 것을 원천 차단하기 위해 매번 최대 할당 크기만큼 초기화
    memset(_power, 0, bins_padded * sizeof(float));
    memset(_mel_bank_flat, 0, bins_padded * mel_padded * sizeof(float));
    memset(_dct_matrix_flat, 0, mel_padded * mfcc_max_padded * sizeof(float));

    _generateWindow(_window, N, _cfg.preprocess.window_type);

    float nyquist = fs / 2.0f * 0.95f; // Nyquist 한계 폭주를 막기 위한 5% 안전 마진

    // [2] FIR 계수 생성 (복붙 오타 교정 및 주파수 폭주 클램핑)
    if (_cfg.preprocess.fir_hpf.enabled) {
        uint16_t taps = _cfg.preprocess.fir_hpf.num_taps;
        if (taps < 3) taps = 3;      
        if (taps % 2 == 0) taps--;   
        if (taps > 127) taps = 127;  
        _cfg.preprocess.fir_hpf.num_taps = taps; 

        float cutoff = _cfg.preprocess.fir_hpf.cutoff_hz;
        if (cutoff > nyquist) cutoff = nyquist;

        _generateFirHpfWindowedSinc(_fir_hpf_coeffs, taps, cutoff);
        for(uint8_t a = 0; a < (uint8_t)_cfg.feature.axis_count; a++) {
            dsps_fir_init_f32(&_fir_hpf_inst[a], _fir_hpf_coeffs, _fir_hpf_state[a], taps);
        }
    }
    
    if (_cfg.preprocess.fir_lpf.enabled) {
        uint16_t taps = _cfg.preprocess.fir_lpf.num_taps;
        if (taps < 3) taps = 3;
        if (taps % 2 == 0) taps--;
        if (taps > 127) taps = 127;
        _cfg.preprocess.fir_lpf.num_taps = taps;

        float cutoff = _cfg.preprocess.fir_lpf.cutoff_hz;
        if (cutoff > nyquist) cutoff = nyquist;

        _generateFirLpfWindowedSinc(_fir_lpf_coeffs, taps, cutoff);
        for(uint8_t a = 0; a < (uint8_t)_cfg.feature.axis_count; a++) {
            dsps_fir_init_f32(&_fir_lpf_inst[a], _fir_lpf_coeffs, _fir_lpf_state[a], taps);
        }
    }
    
    // IIR 및 Notch 계수 생성 (Nyquist 폭주 방어)
    if (_cfg.preprocess.iir_hpf.enabled) {
        float cutoff = _cfg.preprocess.iir_hpf.cutoff_hz > nyquist ? nyquist : _cfg.preprocess.iir_hpf.cutoff_hz;
        dsps_biquad_gen_hpf_f32(_hpf_coeffs, cutoff / fs, _cfg.preprocess.iir_hpf.q_factor);
    }
    if (_cfg.preprocess.iir_lpf.enabled) {
        float cutoff = _cfg.preprocess.iir_lpf.cutoff_hz > nyquist ? nyquist : _cfg.preprocess.iir_lpf.cutoff_hz;
        dsps_biquad_gen_lpf_f32(_lpf_coeffs, cutoff / fs, _cfg.preprocess.iir_lpf.q_factor);
    }
    if (_cfg.preprocess.notch.enabled) {
        float target = _cfg.preprocess.notch.target_freq_hz > nyquist ? nyquist : _cfg.preprocess.notch.target_freq_hz;
        dsps_biquad_gen_notch_f32(_notch_coeffs, target / fs, _cfg.preprocess.notch.gain, _cfg.preprocess.notch.q_factor);
    }

    // [3] Mel-Filterbank 평탄화 생성
    float min_mel = 0.0f;
    float max_mel = T20::C10_DSP::MEL_SCALE_CONST * log10f(1.0f + ((fs / 2.0f) / T20::C10_DSP::MEL_FREQ_CONST));
    float mel_step = (max_mel - min_mel) / (float)(T20::C10_DSP::MEL_FILTERS + 1);

    uint16_t bin_points[T20::C10_DSP::MEL_FILTERS + 2];
    for (int i = 0; i < T20::C10_DSP::MEL_FILTERS + 2; i++) {
        float hz = T20::C10_DSP::MEL_FREQ_CONST * (powf(10.0f, (min_mel + i * mel_step) / T20::C10_DSP::MEL_SCALE_CONST) - 1.0f);
        bin_points[i] = (uint16_t)roundf(N * hz / fs);
    }
    
    for (int m = 0; m < T20::C10_DSP::MEL_FILTERS; m++) {
        uint16_t left = bin_points[m], center = bin_points[m + 1], right = bin_points[m + 2];
        for (int k = left; k < center && k < bins; k++)
            _mel_bank_flat[k * mel_padded + m] = (float)(k - left) / (float)(center - left + 1e-12f);
        for (int k = center; k < right && k < bins; k++)
            _mel_bank_flat[k * mel_padded + m] = (float)(right - k) / (float)(right - center + 1e-12f);
    }

    // [4] DCT-II Matrix 생성
    for (int k = 0; k < T20::C10_DSP::MEL_FILTERS; k++) {
        for (int n = 0; n < mfcc_dim; n++) {
            _dct_matrix_flat[k * mfcc_padded + n] = cosf((M_PI / (float)T20::C10_DSP::MEL_FILTERS) * (k + 0.5f) * n); 
        }
    }

    resetNoiseStats();
    memset(_noise_spectrum, 0, 3 * bins_padded * sizeof(float));
    resetFilterStates();

    Serial.printf("[DSP] Engine v230.011 Started (FFT:%d, Win:%d, MFCC:%d)\n", N, (int)_cfg.preprocess.window_type, mfcc_dim);
    return true;
}

void CL_T20_DspPipeline::_generateWindow(float* p_out, int n, EM_T20_WindowType_t type) {
    switch (type) {
        case EN_T20_WINDOW_HAMMING:
            for (int i = 0; i < n; i++) {
                p_out[i] = 0.54f - 0.46f * cosf((2.0f * (float)M_PI * i) / (n - 1));
            }
            break;
        case EN_T20_WINDOW_BLACKMAN:    dsps_wind_blackman_f32(p_out, n); break;
        case EN_T20_WINDOW_FLATTOP:     dsps_wind_flat_top_f32(p_out, n); break;
        case EN_T20_WINDOW_RECTANGULAR: for (int i = 0; i < n; i++) p_out[i] = 1.0f; break;
        case EN_T20_WINDOW_HANN:
        default:                        dsps_wind_hann_f32(p_out, n); break;
    }
}

bool CL_T20_DspPipeline::processFrame(const float* p_time_in, ST_T20_FeatureVector_t* p_vec_out, uint8_t axis_idx) {
    if (!p_time_in || !p_vec_out || axis_idx >= 3) return false;
    
    // SIMD 가속기 작동 전 메모리 16바이트 정렬 여부 강제 검증
    if (((uintptr_t)_work_frame & 15) != 0 || ((uintptr_t)_dct_matrix_flat & 15) != 0) {
        Serial.println(F("[Critical] DSP Buffers are NOT 16-byte aligned! Process Halted."));
        return false; 
    }
    
    memcpy(_work_frame, p_time_in, sizeof(float) * _current_fft_size);

    // [1] 비선형 스파이크 필터 (위상 왜곡 방지를 위해 맨 처음 실행)
    if (_cfg.preprocess.median.enabled) _applyMedianFilter(_work_frame, _cfg.preprocess.median.window_size);

    // [2] 선형 전처리 (DC 편향 제거를 통한 수치적 안정성 확보 및 NaN 방어)
    _removeDC(_work_frame);

    // ESP-DSP SIMD 가속기의 In-place 연산 덮어쓰기 오염(UB)을 막기 위해 
    // 유휴 상태인 _fft_io_buf를 임시 스크래치 버퍼로 활용한 핑퐁(Ping-Pong) 교차 처리
    float* p_in = _work_frame;
    float* p_out = _fft_io_buf; 

    // [3-1] 선형 위상을 보장하는 고차원 FIR 필터 (IIR 보다 선행)
    if (_cfg.preprocess.fir_hpf.enabled) {
        dsps_fir_f32(&_fir_hpf_inst[axis_idx], p_in, p_out, _current_fft_size);
        float* temp = p_in; p_in = p_out; p_out = temp; // 버퍼 스왑
    }
    if (_cfg.preprocess.fir_lpf.enabled) {
        dsps_fir_f32(&_fir_lpf_inst[axis_idx], p_in, p_out, _current_fft_size);
        float* temp = p_in; p_in = p_out; p_out = temp;
    }

    // [3-2] 저연산량 IIR Filter (Cascade 방식)
    if (_cfg.preprocess.iir_hpf.enabled) {
        dsps_biquad_f32(p_in, p_out, _current_fft_size, _hpf_coeffs, _hpf_state[axis_idx]);
        float* temp = p_in; p_in = p_out; p_out = temp;
    }
    if (_cfg.preprocess.iir_lpf.enabled) {
        dsps_biquad_f32(p_in, p_out, _current_fft_size, _lpf_coeffs, _lpf_state[axis_idx]);
        float* temp = p_in; p_in = p_out; p_out = temp;
    }

    // [4] 특정 주파수 톤 제거 (Notch)
    if (_cfg.preprocess.notch.enabled) {
        dsps_biquad_f32(p_in, p_out, _current_fft_size, _notch_coeffs, _notch_state[axis_idx]);
        float* temp = p_in; p_in = p_out; p_out = temp;
    }

    // 핑퐁 연산 결과가 _fft_io_buf에 남아있다면 다시 _work_frame으로 원복
    if (p_in != _work_frame) {
        memcpy(_work_frame, p_in, _current_fft_size * sizeof(float));
    }

    // [5] 타임라인 교정: 노이즈 게이트를 먼저 수행해야 무음 상태의 RMS 모순이 발생하지 않음
    _applyNoiseGate(_work_frame);

    // [6] 필터링이 끝난 깨끗한 파형에서 순수 진동 에너지(RMS) 도출
    _calcRMS(_work_frame, axis_idx);

    // [7] AI 분석용 고역 강조 (오버랩 이월 방지 적용)
    _applyPreEmphasis(_work_frame, axis_idx);

    // [8] Windowing & FFT
    dsps_mul_f32(_work_frame, _window, _work_frame, _current_fft_size, 1, 1, 1);
    _computePowerSpectrum(_work_frame);

    // [9] 노이즈 감산 및 MFCC 조립 
    _learnNoiseSpectrum(axis_idx);
    _applySpectralSubtraction(axis_idx);
    
    // 26크기의 _log_mel을 SIMD에 직접 밀어넣어 발생하는 Over-read 스택 오염 방어
    alignas(16) float safe_log_mel[32] = {0}; 
    _applyMelFilterbank(safe_log_mel);

    alignas(16) float current_mfcc[T20::C10_DSP::MFCC_COEFFS_MAX];
    _computeDCT2(safe_log_mel, current_mfcc);

    p_vec_out->rms[axis_idx] = _current_rms[axis_idx];
    _pushHistory(current_mfcc, axis_idx);

    if (_history_count[axis_idx] >= T20::C10_DSP::MFCC_HISTORY_LEN) {
        _build39DVector(p_vec_out, axis_idx);
        return true;
    }
    return false;
}

void CL_T20_DspPipeline::_removeDC(float* p_data) {
    uint16_t N = _current_fft_size;
    float sum = 0.0f;

    for (int i = 0; i < N; i++) {
        // [컴파일러 최적화 함정 방어] -ffast-math 최적화로 인해 isnan()이 강제 삭제되는 
        // 현상을 막기 위해, IEEE-754 규격의 비트 패턴을 직접 검사하여 발산을 원천 차단
        uint32_t bits;
        memcpy(&bits, &p_data[i], sizeof(uint32_t));
        if ((bits & 0x7F800000) == 0x7F800000) { // 지수부가 모두 1이면 NaN 또는 Inf
            p_data[i] = 0.0f;
        }
        sum += p_data[i];
    }
    
    if (!_cfg.preprocess.remove_dc) return;

    float mean = sum / (float)N;
    for (int i = 0; i < N; i++) {
        p_data[i] -= mean;
    }
}

void CL_T20_DspPipeline::_calcRMS(const float* p_data, uint8_t axis_idx) {
    float sum_sq = 0.0f;
    uint16_t N = _current_fft_size;

    dsps_dotprod_f32(p_data, p_data, &sum_sq, N);
    _current_rms[axis_idx] = sqrtf(fmaxf(sum_sq / (float)N, 1e-12f));
}

void CL_T20_DspPipeline::_applyPreEmphasis(float* p_data, uint8_t axis_idx) {
    if (!_cfg.preprocess.preemphasis.enable) return;
    float alpha = _cfg.preprocess.preemphasis.alpha;
    
    // 오버랩(Overlap) 환경에서 이전 프레임의 상태 변수를 무리하게 이월(Carry-over)하여 
    // 생기는 파열음(Transient Pop)을 원천 차단하기 위해 In-place 역순 연산 수행.
    for (int i = _current_fft_size - 1; i > 0; i--) {
        p_data[i] = p_data[i] - (alpha * p_data[i - 1]);
    }
    // 프레임의 첫 번째 샘플은 연속성 이월을 생략하고 현재 값을 유지 (파열음 방지)
}

void CL_T20_DspPipeline::_applyMedianFilter(float* p_data, int window_size) {
    if (window_size < 3) return;
    
    if (window_size > 7) window_size = 7;
    if (window_size % 2 == 0) window_size--; 
    
    uint16_t N = _current_fft_size;

    if (window_size == 3) {
        float prev2 = p_data[0];
        float prev1 = p_data[1];

        for (int i = 1; i < N - 1; i++) {
            float a = prev2;           
            float b = prev1;           
            float c = p_data[i + 1];   

            float median = fmaxf(fminf(a, b), fminf(fmaxf(a, b), c));
            prev2 = prev1;
            prev1 = c;
            p_data[i] = median;
        }
        return;
    }

    int half = window_size / 2;
    float temp_buf[7];
    
    // In-place 덮어쓰기로 인해 미래 연산이 과거 결과를 참조하여 파형이 재귀 변질(IIR)되는 
    // 위상 왜곡 현상을 막기 위한 원본 캐시 링버퍼 적용
    float orig_history[3]; 
    for (int i = 0; i < half; i++) orig_history[i] = p_data[i]; 

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < window_size; j++) {
            int idx = i + j - half;
            
            // 덮어써진 값이 아닌, 캐시된 안전한 '원본 과거 데이터' 참조
            if (idx < 0) temp_buf[j] = p_data[0];
            else if (idx < i) temp_buf[j] = orig_history[idx % half]; 
            else if (idx >= N) temp_buf[j] = p_data[N - 1];
            else temp_buf[j] = p_data[idx];
        }
        
        // 덮어쓰기 직전, 현재 위치의 원본 데이터를 미래를 위해 캐시에 백업
        if (i < N) orig_history[i % half] = p_data[i];
        
        // 고속 삽입 정렬
        for (int j = 1; j < window_size; j++) {
            float key = temp_buf[j];
            int k = j - 1;
            while (k >= 0 && temp_buf[k] > key) {
                temp_buf[k + 1] = temp_buf[k];
                k--;
            }
            temp_buf[k + 1] = key;
        }
        p_data[i] = temp_buf[half];
    }
}

void CL_T20_DspPipeline::_applyAdaptiveNotch(float* p_data, uint8_t axis_idx) {
    if (!_cfg.preprocess.notch.enabled) return;
    dsps_biquad_f32(p_data, p_data, _current_fft_size, _notch_coeffs, _notch_state[axis_idx]);
}

void CL_T20_DspPipeline::_applyNoiseGate(float* p_data) {
    if (!_cfg.preprocess.noise.enable_gate) return;
    float threshold = _cfg.preprocess.noise.gate_threshold_abs;
    uint16_t N = _current_fft_size; 
    for (int i = 0; i < N; i++) { 
        if (fabsf(p_data[i]) < threshold) p_data[i] = 0.0f;
    }
}

void CL_T20_DspPipeline::_computePowerSpectrum(const float* p_time) {
    uint16_t N = _current_fft_size;
    uint16_t bins = (N / 2) + 1;

    for (int i = 0; i < N; i++) {
        _fft_io_buf[i * 2]     = p_time[i];
        _fft_io_buf[i * 2 + 1] = 0.0f;
    }

    dsps_fft2r_fc32(_fft_io_buf, N);
    dsps_bit_rev2r_fc32(_fft_io_buf, N);

    // [에너지 왜곡 방어] FFT 연산 후 결과가 N배로 증폭되는 것을 방지하기 위해 1/N 스케일링 적용
    float scale = 1.0f / (float)N;
    for (int i = 0; i < bins; i++) {
        float re = _fft_io_buf[i * 2] * scale;
        float im = _fft_io_buf[i * 2 + 1] * scale;
        _power[i] = fmaxf((re * re + im * im), 1e-12f);
    }
}

void CL_T20_DspPipeline::_learnNoiseSpectrum(uint8_t axis_idx) {
    if (_cfg.preprocess.noise.mode == EN_T20_NOISE_OFF) return;

    bool is_learning = _noise_learning_active || (_noise_learned_frames[axis_idx] < _cfg.preprocess.noise.noise_learn_frames);
    if (!is_learning && _cfg.preprocess.noise.mode == EN_T20_NOISE_FIXED) return;

    uint16_t bins = (_current_fft_size / 2) + 1;
    uint16_t bins_padded = (bins + 3) & ~3; 
    
    float* target_noise = _noise_spectrum + (axis_idx * bins_padded);

    if (_cfg.preprocess.noise.mode == EN_T20_NOISE_ADAPTIVE) {
        float alpha = _noise_learning_active ? 0.2f : _cfg.preprocess.noise.adaptive_alpha;
        for (int i = 0; i < bins; i++) {
            target_noise[i] = (1.0f - alpha) * target_noise[i] + alpha * _power[i];
        }
    } else if (is_learning) {  
        float count = (float)_noise_learned_frames[axis_idx];
        for (int i = 0; i < bins; i++) {
            target_noise[i] = ((target_noise[i] * count) + _power[i]) / (count + 1.0f);
        }
    }

    if (_noise_learned_frames[axis_idx] < 0xFFFF) _noise_learned_frames[axis_idx]++;
}

void CL_T20_DspPipeline::_applySpectralSubtraction(uint8_t axis_idx) {
    if (_cfg.preprocess.noise.mode == EN_T20_NOISE_OFF) return;
    if (_cfg.preprocess.noise.mode == EN_T20_NOISE_FIXED && _noise_learned_frames[axis_idx] < _cfg.preprocess.noise.noise_learn_frames) return;

    uint16_t bins = (_current_fft_size / 2) + 1;
    uint16_t bins_padded = (bins + 3) & ~3; 
    float strength = _cfg.preprocess.noise.spectral_subtract_strength;

    float* target_noise = _noise_spectrum + (axis_idx * bins_padded);

    for (int i = 0; i < bins; i++) {
        _power[i] = fmaxf(_power[i] - (strength * target_noise[i]), 1e-12f);
    }
}

void CL_T20_DspPipeline::_applyMelFilterbank(float* p_log_mel_out) {
    uint16_t bins = (_current_fft_size / 2) + 1;
    uint16_t bins_padded = (bins + 3) & ~3;
    uint16_t mel_padded  = (T20::C10_DSP::MEL_FILTERS + 3) & ~3;
    
    alignas(16) float mel_energy[32] = {0}; 

    // 패딩된 차원 크기를 가속기에 명시하여 스택 초과 침범 원천 차단
    dspm_mult_f32(_power, _mel_bank_flat, mel_energy, 1, bins_padded, mel_padded);

    for (int m = 0; m < T20::C10_DSP::MEL_FILTERS; m++) {
        p_log_mel_out[m] = logf(fmaxf(mel_energy[m], 1e-12f));
    }
}

void CL_T20_DspPipeline::_computeDCT2(const float* p_in, float* p_out) {
    uint16_t mel_padded  = (T20::C10_DSP::MEL_FILTERS + 3) & ~3;
    uint16_t mfcc_padded = (_cfg.feature.mfcc_coeffs + 3) & ~3;

    alignas(16) float padded_out[32] = {0}; 
    
    dspm_mult_f32(p_in, _dct_matrix_flat, padded_out, 1, mel_padded, mfcc_padded);
    
    memcpy(p_out, padded_out, _cfg.feature.mfcc_coeffs * sizeof(float));
}

void CL_T20_DspPipeline::_pushHistory(const float* p_mfcc, uint8_t axis_idx) {
    uint16_t dim = _cfg.feature.mfcc_coeffs;
    float (*hist)[T20::C10_DSP::MFCC_COEFFS_MAX] = _mfcc_history[axis_idx];

    if (_history_count[axis_idx] < T20::C10_DSP::MFCC_HISTORY_LEN) {
        memcpy(hist[_history_count[axis_idx]++], p_mfcc, sizeof(float) * dim);
    } else {
        // [메모리 꼬임 방어] 2D 다차원 배열의 물리적 행(Row) 간격과 논리적 차원(dim) 
        // 불일치로 인한 대규모 메모리 꼬임을 막기 위해 행 단위 개별 복사 수행
        for (int i = 0; i < T20::C10_DSP::MFCC_HISTORY_LEN - 1; i++) {
            memcpy(hist[i], hist[i + 1], sizeof(float) * dim);
        }
        memcpy(hist[T20::C10_DSP::MFCC_HISTORY_LEN - 1], p_mfcc, sizeof(float) * dim);
    }
}

void CL_T20_DspPipeline::_build39DVector(ST_T20_FeatureVector_t* p_vec_out, uint8_t axis_idx) {
    uint16_t dim = _cfg.feature.mfcc_coeffs;
    float (*hist)[T20::C10_DSP::MFCC_COEFFS_MAX] = _mfcc_history[axis_idx];

    alignas(16) float delta[T20::C10_DSP::MFCC_COEFFS_MAX];
    alignas(16) float delta2[T20::C10_DSP::MFCC_COEFFS_MAX];

    for (int i = 0; i < dim; i++) {
        delta[i] = (hist[3][i] - hist[1][i]) / 2.0f;
    }

    for (int i = 0; i < dim; i++) {
        delta2[i] = hist[3][i] - (2.0f * hist[2][i]) + hist[1][i];
    }

    memcpy(&p_vec_out->features[axis_idx][0],       hist[2], sizeof(float) * dim); 
    memcpy(&p_vec_out->features[axis_idx][dim],     delta,   sizeof(float) * dim); 
    memcpy(&p_vec_out->features[axis_idx][dim * 2], delta2,  sizeof(float) * dim); 
}

void CL_T20_DspPipeline::resetFilterStates() {
    memset(_fir_hpf_state, 0, sizeof(_fir_hpf_state));
    memset(_fir_lpf_state, 0, sizeof(_fir_lpf_state));
    
    if (_cfg.preprocess.fir_hpf.enabled) {
        for(uint8_t a=0; a < (uint8_t)_cfg.feature.axis_count; a++)
            dsps_fir_init_f32(&_fir_hpf_inst[a], _fir_hpf_coeffs, _fir_hpf_state[a], _cfg.preprocess.fir_hpf.num_taps);
    }
    if (_cfg.preprocess.fir_lpf.enabled) {
        for(uint8_t a=0; a < (uint8_t)_cfg.feature.axis_count; a++)
            dsps_fir_init_f32(&_fir_lpf_inst[a], _fir_lpf_coeffs, _fir_lpf_state[a], _cfg.preprocess.fir_lpf.num_taps);
    }

    memset(_hpf_state, 0, sizeof(_hpf_state));
    memset(_lpf_state, 0, sizeof(_lpf_state));
    memset(_notch_state, 0, sizeof(_notch_state));
    memset(_prev_sample, 0, sizeof(_prev_sample));
    memset(_history_count, 0, sizeof(_history_count));
    memset(_mfcc_history, 0, sizeof(_mfcc_history));
}


float CL_T20_DspPipeline::getBandEnergy(float start_hz, float end_hz) {
    if (!_power) return 0.0f;

    const uint16_t N = _current_fft_size;
    const float fs = T20::C10_DSP::SAMPLE_RATE_HZ;
    const float bin_res = fs / N;
    
    // [수학적/메모리 방어] 음수 주파수가 uint16_t로 캐스팅될 때 발생하는 언더플로우(65535) 및 
    // _power 배열 경계 초과 참조(Out-of-bounds Read)를 막기 위한 클램핑
    if (start_hz < 0.0f) start_hz = 0.0f;
    if (end_hz < start_hz) end_hz = start_hz;


    uint16_t start_bin = (uint16_t)(start_hz / bin_res);
    uint16_t end_bin = (uint16_t)(end_hz / bin_res);
    
    // 하한선뿐만 아니라 상한선 검사도 start_bin/end_bin 양쪽에 적용
    uint16_t max_bin = (N / 2);
    if (end_bin > max_bin) end_bin = max_bin;
    if (start_bin > max_bin) start_bin = max_bin;

    float energy = 0.0f;
    for (uint16_t i = start_bin; i <= end_bin; i++) {
        energy += _power[i];
    }
    return energy;
}

void CL_T20_DspPipeline::_generateFirLpfWindowedSinc(float* coeffs, uint16_t num_taps, float cutoff_hz) {
    float fs = T20::C10_DSP::SAMPLE_RATE_HZ;
    float normalized_cutoff = cutoff_hz / fs;
    
    // OOM 패닉 및 쓰레기값 참조 리스크를 없애기 위해 128 크기의 정적 스택 배열 사용
    alignas(16) float win[128] = {0};

    dsps_wind_blackman_f32(win, num_taps);

    float M = (float)(num_taps - 1);
    float sum = 0.0f;
    
    for (int i = 0; i < num_taps; i++) {
        float x = (float)i - (M / 2.0f);
        // [수학적 결함 방어] Sinc 함수의 0 나누기 예외(NaN 발생)를 Epsilon 비교로 차단
        if (fabsf(x) < 1e-6f) {
            coeffs[i] = 2.0f * normalized_cutoff; 
        } else {
            coeffs[i] = sinf(2.0f * (float)M_PI * normalized_cutoff * x) / ((float)M_PI * x);
        }
        coeffs[i] *= win[i]; 
        sum += coeffs[i];
    }
    
    for (int i = 0; i < num_taps; i++) {
        coeffs[i] /= sum;
    }
}

void CL_T20_DspPipeline::_generateFirHpfWindowedSinc(float* coeffs, uint16_t num_taps, float cutoff_hz) {
    _generateFirLpfWindowedSinc(coeffs, num_taps, cutoff_hz);
    
    uint16_t center = (num_taps - 1) / 2;
    for (int i = 0; i < num_taps; i++) {
        coeffs[i] = -coeffs[i];
    }
    coeffs[center] += 1.0f;
}
