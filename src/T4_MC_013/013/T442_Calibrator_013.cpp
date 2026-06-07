/* ============================================================================
 * File: T442_Calibrator_013.cpp
 * Summary: Offline Calibration & Hardware Profiling Engine
 * ============================================================================
 * * [AI 메모: 마이그레이션 적용 완료 사항]
 * 1. [메모리 단편화 방지]: 모든 8KB 청크 읽기 시 heap_caps_malloc(SPIRAM) 강제.
 * 2. [수학적 무결성]: Auto 보정 시 노이즈 스케일링을 폐기하고 하드 리셋(resetNoiseProfile) 적용.
 * 3. [Pure Math EQ]: Manual 보정 시 IFFT와 윈도윙을 거쳐 63-Tap 계수 도출 및 
 * DC/나이퀴스트 대역 마스킹 적용 완료.
 * 4. [v013 핫스왑]: 웹에서 튜닝한 대역폭(Hz), 한계치(Gain) 값을 T415에서 로드하여 동적 반영.
 * ========================================================================== */

#include "T442_Calibrator_013.hpp"
#include "T450_FsmMgr_015.hpp"
#include "dsps_wind.h"
#include "dsps_fft2r.h"
#include <LittleFS.h>
#include <SD_MMC.h>
#include <cstring>
#include <cmath>

static const char* TAG = "T442_CAL";

struct CalibTaskParam {
    T442_Calibrator* instance;
    char filePath[SmeaConfig::StorageLimit::MAX_PATH_LEN_CONST];
    bool isManual;
};

T442_Calibrator::T442_Calibrator() {
    _refExtractor = nullptr;
}

T442_Calibrator::~T442_Calibrator() {}

bool T442_Calibrator::startManualCalibration(const char* p_pcmFilePath) {
    if (_hCalibTask != nullptr) return false; // 이미 구동 중
    CalibTaskParam* v_param = new CalibTaskParam();
    v_param->instance = this;
    v_param->isManual = true;
    strlcpy(v_param->filePath, p_pcmFilePath, sizeof(v_param->filePath));

    xTaskCreatePinnedToCore(_calibTaskProc, "CalTask", 8192, v_param, 2, &_hCalibTask, 1);
    return true;
}

bool T442_Calibrator::startAutoCalibration(const char* p_pcmFilePath) {
    if (_hCalibTask != nullptr) return false;
    CalibTaskParam* v_param = new CalibTaskParam();
    v_param->instance = this;
    
    // 자동 보정용 임시 파일 경로를 인스턴스에 복사
    strlcpy(v_param->filePath, p_pcmFilePath, sizeof(v_param->filePath));
    
    v_param->isManual = false;

    xTaskCreatePinnedToCore(_calibTaskProc, "CalTask", 8192, v_param, 2, &_hCalibTask, 1);
    return true;
}

void T442_Calibrator::_calibTaskProc(void* p_param) {
    CalibTaskParam* v_param = (CalibTaskParam*)p_param;
    
    if (v_param->isManual) {
        v_param->instance->_processManual(v_param->filePath);
    } else {
        // 구조체에 담아온 임시 파일 경로를 파라미터로 정상 전달
        v_param->instance->_processAuto(v_param->filePath); 
    }

    // 작업 종료 후 FSM을 READY로 원복시키고 스스로 소멸
    T450_FsmManager::getInstance().dispatchCommand(SystemCommand::CMD_CALIB_STOP);
    v_param->instance->_hCalibTask = nullptr;
    delete v_param;
    vTaskDelete(NULL);
}

void T442_Calibrator::_processAuto(const char* p_path) {
    ESP_LOGI(TAG, "Starting Auto Calibration (Gain Balancing)...");
    
    File v_file = SD_MMC.open(p_path, "r");
    if (!v_file) {
        ESP_LOGE(TAG, "Auto Calib PCM File Open Failed!");
        return;
    }

    double v_sumSqL = 0.0;
    double v_sumSqR = 0.0;
    uint32_t v_totalSamples = 0;

    // [최적화] FFT 없이 순수 시간 영역(Time Domain) 에너지(RMS)만 고속 스트리밍 연산
    const uint32_t v_samples = SmeaConfig::CalibLimit::WELCH_CHUNK_SAMPLES_CONST;
    const uint32_t v_bytesToRead = v_samples * 2 * sizeof(int32_t);
    // [v013] SRAM 파편화 방어
    int32_t* v_pcmChunk = (int32_t*)heap_caps_malloc(v_bytesToRead, MALLOC_CAP_SPIRAM);
    
    if (!v_pcmChunk) { v_file.close(); return; }

    float v_scale = SmeaConfig::System::PCM_32BIT_SCALE_CONST;
    
    while (v_file.available() >= v_bytesToRead) {
        v_file.read((uint8_t*)v_pcmChunk, v_bytesToRead);
        for (uint32_t i = 0; i < v_samples; i++) {
            float v_L = (float)v_pcmChunk[i * 2] * v_scale;
            float v_R = (float)v_pcmChunk[i * 2 + 1] * v_scale;
            v_sumSqL += (double)(v_L * v_L);
            v_sumSqR += (double)(v_R * v_R);
        }
        v_totalSamples += v_samples;
        vTaskDelay(pdMS_TO_TICKS(2)); // WDT 방어
    }
    heap_caps_free(v_pcmChunk);
    v_file.close();

    if (v_totalSamples == 0) { SD_MMC.remove(p_path); return; }

    // R채널을 L채널 기준(Reference)에 맞추기 위한 Gain 산출
    float v_rmsL = (float)sqrt(v_sumSqL / v_totalSamples);
    float v_rmsR = (float)sqrt(v_sumSqR / v_totalSamples);
    float v_gainRatio = (v_rmsR > SmeaConfig::System::MATH_EPSILON_CONST) ? (v_rmsL / v_rmsR) : 1.0f;

    // 하드 리미트 방어 (화이트 노이즈 무한 폭발 차단)
    if (v_gainRatio < SmeaConfig::CalibLimit::GAIN_RATIO_MIN_CONST || v_gainRatio > SmeaConfig::CalibLimit::GAIN_RATIO_MAX_CONST) {
        ESP_LOGE(TAG, "Auto Calib Aborted: Gain Out of Bound (Hardware Failure Suspected)");
        SD_MMC.remove(p_path); 
        return;
    }

    DynamicConfig v_cfg = T415_ConfigManager::getInstance().getConfig();
    v_cfg.dsp.calib_gain_R = v_gainRatio;
    
    // [v013 교정] Auto Calib은 임시 튜닝이 아니므로 바로 commitSave() 호출 (또는 updateConfig 후 save)
    T415_ConfigManager::getInstance().updateConfig(v_cfg); // Lazy Write가 아닌 즉각 반영 유도

    // [수학적 모순 방어] 스케일링 폐기 및 노이즈 프로필 초기화
    if (_refExtractor) _refExtractor->resetNoiseProfile();

    // auto_history.csv 이력 보존 로직 (O(1) Append Write)
    File v_csv = SD_MMC.open("/t20_data/calib/auto_history.csv", "a");
    if (v_csv) {
        char v_logLine[64];
        snprintf(v_logLine, sizeof(v_logLine), "%llu,%.4f,%.4f\n", (uint64_t)time(NULL), 1.0f, v_gainRatio);
        v_csv.print(v_logLine);
        v_csv.close();
    }

    SD_MMC.remove(p_path); // 연산이 끝난 10초짜리 임시 PCM 삭제 (SD 용량 낭비 방지)
    ESP_LOGI(TAG, "Auto Calibration Completed. R_Gain: %.3f", v_gainRatio);
}

void T442_Calibrator::_processManual(const char* p_path) {
    ESP_LOGI(TAG, "Starting Manual Calibration (Welch's Method)...");
    
    File v_file = SD_MMC.open(p_path, "r");
    if (!v_file) {
        ESP_LOGE(TAG, "PCM File Open Failed!");
        return;
    }

    memset(_powerAccL, 0, sizeof(_powerAccL));
    memset(_powerAccR, 0, sizeof(_powerAccR));
    uint32_t v_chunkCount = 0;

    const uint32_t v_samples = SmeaConfig::CalibLimit::WELCH_CHUNK_SAMPLES_CONST;
    const uint32_t v_bytesToRead = v_samples * 2 * sizeof(int32_t);
    // [v013] SRAM 파편화 방어
    int32_t* v_pcmChunk = (int32_t*)heap_caps_malloc(v_bytesToRead, MALLOC_CAP_SPIRAM);
    
    if (!v_pcmChunk) { v_file.close(); return; }

    float v_scale = SmeaConfig::System::PCM_32BIT_SCALE_CONST;
    
    while (v_file.available() >= v_bytesToRead) {
        v_file.read((uint8_t*)v_pcmChunk, v_bytesToRead);
        
        for (uint32_t i = 0; i < v_samples; i++) {
            _fftWorkBuf[i * 2] = (float)v_pcmChunk[i * 2] * v_scale;
            _fftWorkBuf[i * 2 + 1] = 0.0f;
        }
        dsps_fft2r_fc32(_fftWorkBuf, v_samples);
        dsps_bit_rev2r_fc32(_fftWorkBuf, v_samples);
        for (uint16_t i = 0; i <= v_samples / 2; i++) {
            _powerAccL[i] += (_fftWorkBuf[i * 2] * _fftWorkBuf[i * 2] + _fftWorkBuf[i * 2 + 1] * _fftWorkBuf[i * 2 + 1]);
        }

        v_chunkCount++;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
    heap_caps_free(v_pcmChunk);
    v_file.close();

    if (v_chunkCount == 0) { SD_MMC.remove(p_path); return; }

    // ========================================================================
    // [Pure Math] 주파수 응답 역산 및 63-Tap FIR 계수 산출 알고리즘
    // ========================================================================
    
    DynamicConfig v_cfg = T415_ConfigManager::getInstance().getConfig(); // 동적 파라미터 로드
    
    // 1. 평균 파워 및 진폭 스펙트럼 도출
    for (uint16_t i = 0; i <= v_samples / 2; i++) {
        _powerAccL[i] /= (float)v_chunkCount;
    }
    
    // 2. 1kHz 대역을 기준(Reference) 게인으로 설정 , 동적 기준 주파수 매핑
    float v_binRes = (float)SmeaConfig::System::SAMPLING_RATE_CONST / SmeaConfig::System::FFT_SIZE_CONST;
    uint16_t v_refBin = (uint16_t)(v_cfg.calib.ref_freq_hz / v_binRes);
    float v_refAmp = sqrtf(fmaxf(_powerAccL[v_refBin], SmeaConfig::System::MATH_EPSILON_12_CONST));

    // 3. 타겟 역산 스펙트럼(Inverse Spectrum) 구성 및 대역 마스킹
    memset(_fftWorkBuf, 0, sizeof(_fftWorkBuf));
    uint16_t v_minBin = (uint16_t)(v_cfg.calib.filter_min_freq_hz / v_binRes);  
    uint16_t v_maxBin = (uint16_t)(v_cfg.calib.filter_max_freq_hz / v_binRes); 
    
    for (uint16_t i = 0; i <= v_samples / 2; i++) {
        float v_amp = sqrtf(fmaxf(_powerAccL[i], SmeaConfig::System::MATH_EPSILON_12_CONST));
        float v_targetGain = 1.0f;

        if (i >= v_minBin && i <= v_maxBin) {
            v_targetGain = v_refAmp / v_amp;
            
            // [수학적 방어] 하드 리미트: 무한대 발산 차단 (약 +/- 12dB 제한) 동적 하드 리미트
            if (v_targetGain > v_cfg.calib.target_gain_max) v_targetGain = v_cfg.calib.target_gain_max;
            if (v_targetGain < v_cfg.calib.target_gain_min) v_targetGain = v_cfg.calib.target_gain_min;
        }

        // 실수부(Real)에 타겟 게인 할당, 허수부(Imag)는 0 (Linear Phase)
        _fftWorkBuf[i * 2] = v_targetGain;
        _fftWorkBuf[i * 2 + 1] = 0.0f;
        
        // IFFT를 위한 대칭(Symmetric) 복사
        if (i > 0 && i < v_samples / 2) {
            _fftWorkBuf[(v_samples - i) * 2] = v_targetGain;
            _fftWorkBuf[(v_samples - i) * 2 + 1] = 0.0f;
        }
    }
    
    // [Zero-Defect 보완] DC 성분(0Hz) 및 나이퀴스트 주파수 강제 격리 (하드웨어 앰프 보호)
    _fftWorkBuf[0] = 1.0f;                 // DC 성분 Bypass
    _fftWorkBuf[1] = 0.0f;
    _fftWorkBuf[v_samples] = 1.0f;         // Nyquist 성분 Bypass
    _fftWorkBuf[v_samples + 1] = 0.0f;

    // 4. 역 푸리에 변환 (IFFT) 
    dsps_fft2r_fc32(_fftWorkBuf, v_samples);
    dsps_bit_rev2r_fc32(_fftWorkBuf, v_samples);
    for (uint32_t i = 0; i < v_samples * 2; i++) _fftWorkBuf[i] /= (float)v_samples;
    
    // 5. 원점 시프트(Shift) 및 Blackman 윈도윙 적용
    const uint16_t v_taps = SmeaConfig::CalibLimit::EQ_FIR_TAPS_CONST;
    uint16_t v_center = v_taps / 2;
    alignas(16) float v_win[64] = {0};
    dsps_wind_blackman_f32(v_win, v_taps);

    memset(v_cfg.dsp.calib_eq_coeffs, 0, sizeof(v_cfg.dsp.calib_eq_coeffs));

    float v_sumAbs = 0.0f; // [Zero-Defect 보완] 정규화용 합계 누적 변수
    
    for (int16_t i = 0; i < v_taps; i++) {
        int16_t v_timeIdx = i - v_center;
        uint16_t v_fftIdx = (v_timeIdx >= 0) ? v_timeIdx : (v_samples + v_timeIdx);
        v_cfg.dsp.calib_eq_coeffs[i] = _fftWorkBuf[v_fftIdx * 2] * v_win[i];
        v_sumAbs += fabsf(v_cfg.dsp.calib_eq_coeffs[i]);
    }

    // [Zero-Defect 보완] L1-Norm 기반 정규화 (디지털 클리핑 차단 및 Headroom 확보)
    if (v_sumAbs > v_cfg.calib.norm_safe_thresh) {
        float v_scaleDown = v_cfg.calib.norm_safe_thresh / v_sumAbs;
        for (int16_t i = 0; i < v_taps; i++) {
            v_cfg.dsp.calib_eq_coeffs[i] *= v_scaleDown;
        }
        ESP_LOGW(TAG, "FIR Normalized (Scale: %.3f)", v_scaleDown);
    }

    // 6. 결과 갱신 및 데이터 무결성 보호 (과거 노이즈 프로필 파기)
    // [v013 교정] Manual Calib 결과도 플래시에 즉각 저장하도록 유도
    T415_ConfigManager::getInstance().updateConfig(v_cfg); 
    if (_refExtractor) _refExtractor->resetNoiseProfile();

    SD_MMC.remove(p_path); // 연산이 끝난 임시 PCM 삭제
    ESP_LOGI(TAG, "Manual Calibration (Pure Math FIR EQ) Completed.");
}
