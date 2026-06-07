/* ============================================================================
 * File: T280_Calibrator_242.cpp
 * Summary: 멀티모달 오프라인 캘리브레이션 및 하드웨어 프로파일링 엔진 구현부
 * ============================================================================
 * [로직 상태 추적 (Logic State Tracker)]
 * - 유지: Blackman 윈도윙을 거친 63-Tap FIR 필터 계수 역산 로직.
 * - 갱신: [교정] 진동 3축 병합 검사 및 auto_history.csv 예지보전 추이 기록 복원.
 * - 신규: [원칙 준수] 무거운 I/O 및 FFT 연산 루프 내부에 esp_task_wdt_reset() 강제 삽입.
 * ========================================================================== */

#include "T280_Calibrator_242.hpp"
#include "esp_log.h"
#include "dsps_wind.h"
#include "dsps_fft2r.h"
#include "esp_task_wdt.h" // WDT 리셋용
#include <LittleFS.h>
#include <SD_MMC.h>
#include <cstring>
#include <cmath>

// 외부 FSM 커맨드 하달용
extern void T240_DispatchCommand(T2_Type::SystemCommand p_cmd);

static const char* TAG = "T240_CAL";

CL_T2_Calibrator::CL_T2_Calibrator() {}

CL_T2_Calibrator::~CL_T2_Calibrator() {}

bool CL_T2_Calibrator::_findLatestCalibFile(char* p_outPath, size_t p_maxLen, bool p_isManual) {
    File v_dir = SD_MMC.open(T2_Def::Path::SD_DIR_RAW_CONST);
    if (!v_dir || !v_dir.isDirectory()) return false;

    time_t v_latestTime = 0;
    bool v_found = false;
    const char* v_prefix = p_isManual ? "calib_man" : "calib_auto";

    File v_file = v_dir.openNextFile();
    while (v_file) {
        if (!v_file.isDirectory() && strstr(v_file.name(), v_prefix) != nullptr) {
            time_t v_ctime = v_file.getLastWrite();
            if (v_ctime > v_latestTime) {
                v_latestTime = v_ctime;
                snprintf(p_outPath, p_maxLen, "%s/%s", T2_Def::Path::SD_DIR_RAW_CONST, v_file.name());
                v_found = true;
            }
        }
        v_file = v_dir.openNextFile();
    }
    return v_found;
}

bool CL_T2_Calibrator::startManualCalibration(const char* p_pcmFilePath) {
    if (_hCalibTask != nullptr) return false;

    CalibTaskParam* v_param = new CalibTaskParam();
    v_param->instance = this;
    v_param->isManual = true;

    if (p_pcmFilePath) {
        strlcpy(v_param->filePath, p_pcmFilePath, sizeof(v_param->filePath));
    } else {
        if (!_findLatestCalibFile(v_param->filePath, sizeof(v_param->filePath), true)) {
            ESP_LOGE(TAG, "No Manual Calib Raw file found!");
            delete v_param;
            return false;
        }
    }

    xTaskCreatePinnedToCore(_calibTaskProc, "CalTask", 8192, v_param, 2, &_hCalibTask, 1);
    return true;
}

bool CL_T2_Calibrator::startAutoCalibration(const char* p_pcmFilePath) {
    if (_hCalibTask != nullptr) return false;

    CalibTaskParam* v_param = new CalibTaskParam();
    v_param->instance = this;
    v_param->isManual = false;

    if (p_pcmFilePath) {
        strlcpy(v_param->filePath, p_pcmFilePath, sizeof(v_param->filePath));
    } else {
        if (!_findLatestCalibFile(v_param->filePath, sizeof(v_param->filePath), false)) {
            ESP_LOGE(TAG, "No Auto Calib Raw file found!");
            delete v_param;
            return false;
        }
    }

    xTaskCreatePinnedToCore(_calibTaskProc, "CalTask", 8192, v_param, 2, &_hCalibTask, 1);
    return true;
}

void CL_T2_Calibrator::_calibTaskProc(void* p_param) {
    CalibTaskParam* v_param = (CalibTaskParam*)p_param;

    if (v_param->isManual) {
        v_param->instance->_processManual(v_param->filePath);
    } else {
        v_param->instance->_processAuto(v_param->filePath);
    }

    T240_DispatchCommand(T2_Type::SystemCommand::CMD_STOP);
    v_param->instance->_hCalibTask = nullptr;
    delete v_param;
    vTaskDelete(NULL);
}

void CL_T2_Calibrator::_processAuto(const char* p_path) {
    ESP_LOGI(TAG, "Starting Auto Calibration (Baseline Noise Profiling)...");

    File v_file = SD_MMC.open(p_path, "r");
    if (!v_file) return;

    v_file.seek(sizeof(T2_Type::FileHeader)); // 헤더 마스킹

    double v_sumSqVib[T2_Def::System::VIB_AXIS_CONST] = {0.0};
    double v_sumSqAudio = 0.0;
    uint32_t v_totalChunks = 0;

    // [교정] SIMD 정렬 파괴 방지를 위한 aligned_alloc 강제 적용
    T2_Type::UnifiedRawChunk* v_chunk = (T2_Type::UnifiedRawChunk*)heap_caps_aligned_alloc(16, sizeof(T2_Type::UnifiedRawChunk), MALLOC_CAP_SPIRAM);
    if (!v_chunk) { v_file.close(); return; }

    while (v_file.available() >= sizeof(T2_Type::UnifiedRawChunk)) {
        v_file.read((uint8_t*)v_chunk, sizeof(T2_Type::UnifiedRawChunk));

        // [교정] 진동 3축(X,Y,Z) 데이터 누락 복원 및 병합 추출
        for (uint8_t axis = 0; axis < T2_Def::System::VIB_AXIS_CONST; axis++) {
            for (uint16_t i = 0; i < T2_Def::System::FFT_SIZE_VIB_CONST; i++) {
                v_sumSqVib[axis] += (double)(v_chunk->vib[axis][i] * v_chunk->vib[axis][i]);
            }
        }

        for (uint16_t i = 0; i < T2_Def::System::FFT_SIZE_AUDIO_CONST; i++) {
            v_sumSqAudio += (double)(v_chunk->audio[i] * v_chunk->audio[i]);
        }

        v_totalChunks++;
        vTaskDelay(pdMS_TO_TICKS(2));
        #ifdef ESP_IDF_VERSION
            esp_task_wdt_reset(); // [교정] WDT 패닉 방어
        #endif
    }

    heap_caps_free(v_chunk);
    v_file.close();

    if (v_totalChunks == 0) { SD_MMC.remove(p_path); return; }

    // 가장 노이즈가 큰(Worst-case) 축을 진동 베이스라인으로 선정
    float v_baselineVib = 0.0f;
    for (uint8_t axis = 0; axis < T2_Def::System::VIB_AXIS_CONST; axis++) {
        float v_rms = (float)sqrt(v_sumSqVib[axis] / (v_totalChunks * T2_Def::System::FFT_SIZE_VIB_CONST));
        if (v_rms > v_baselineVib) v_baselineVib = v_rms;
    }
    float v_baselineAudio = (float)sqrt(v_sumSqAudio / (v_totalChunks * T2_Def::System::FFT_SIZE_AUDIO_CONST));

    T2_Type::DynamicConfig v_cfg = CL_T2_ConfigManager::getInstance().getConfig();

    // Auto Calib 결과로 노이즈 징후 임계값(Threshold)을 동적 보정 (현재 노이즈의 3배를 고장 임계치로 세팅)
    v_cfg.trigger.vib_rms_thresh = fmaxf(v_baselineVib * 3.0f, T2_Def::Decision::RULE_VIB_RMS_THRESH_DEF);
    v_cfg.trigger.audio_rms_thresh = fmaxf(v_baselineAudio * 3.0f, T2_Def::Decision::RULE_AUDIO_RMS_THRESH_DEF);

    CL_T2_ConfigManager::getInstance().updateConfig(v_cfg);

    // [교정] 예지보전을 위한 CSV 이력 로깅 복원 (Trend 분석용)
    File v_csv = SD_MMC.open("/t240_data/auto_history.csv", "a");
    if (v_csv) {
        char v_logLine[64];
        snprintf(v_logLine, sizeof(v_logLine), "%llu,%.6f,%.6f\n", (uint64_t)time(NULL), v_baselineVib, v_baselineAudio);
        v_csv.print(v_logLine);
        v_csv.close();
    }

    SD_MMC.remove(p_path);
    ESP_LOGI(TAG, "Auto Calib Completed. Worst Vib RMS: %.5f, Audio RMS: %.5f", v_baselineVib, v_baselineAudio);
}

void CL_T2_Calibrator::_processManual(const char* p_path) {
    ESP_LOGI(TAG, "Starting Manual Calibration (FIR EQ Inverse Math)...");

    File v_file = SD_MMC.open(p_path, "r");
    if (!v_file) return;

    v_file.seek(sizeof(T2_Type::FileHeader));

    float* v_fftWorkBuf = (float*)heap_caps_aligned_alloc(16, T2_Def::System::FFT_SIZE_AUDIO_CONST * 2 * sizeof(float), MALLOC_CAP_SPIRAM);
    float* v_powerAcc = (float*)heap_caps_aligned_alloc(16, (T2_Def::System::FFT_SIZE_AUDIO_CONST / 2 + 1) * sizeof(float), MALLOC_CAP_SPIRAM);

    // [교정] SIMD 정렬 강제
    T2_Type::UnifiedRawChunk* v_chunk = (T2_Type::UnifiedRawChunk*)heap_caps_aligned_alloc(16, sizeof(T2_Type::UnifiedRawChunk), MALLOC_CAP_SPIRAM);

    if (!v_fftWorkBuf || !v_powerAcc || !v_chunk) {
        if(v_fftWorkBuf) heap_caps_free(v_fftWorkBuf);
        if(v_powerAcc) heap_caps_free(v_powerAcc);
        if(v_chunk) heap_caps_free(v_chunk);
        v_file.close();
        return;
    }

    memset(v_powerAcc, 0, (T2_Def::System::FFT_SIZE_AUDIO_CONST / 2 + 1) * sizeof(float));
    uint32_t v_chunkCount = 0;
    const uint32_t v_samples = T2_Def::System::FFT_SIZE_AUDIO_CONST;

    while (v_file.available() >= sizeof(T2_Type::UnifiedRawChunk)) {
        v_file.read((uint8_t*)v_chunk, sizeof(T2_Type::UnifiedRawChunk));

        for (uint32_t i = 0; i < v_samples; i++) {
            v_fftWorkBuf[i * 2] = v_chunk->audio[i];
            v_fftWorkBuf[i * 2 + 1] = 0.0f;
        }

        dsps_fft2r_fc32(v_fftWorkBuf, v_samples);
        dsps_bit_rev2r_fc32(v_fftWorkBuf, v_samples);

        for (uint16_t i = 0; i <= v_samples / 2; i++) {
            v_powerAcc[i] += (v_fftWorkBuf[i * 2] * v_fftWorkBuf[i * 2] + v_fftWorkBuf[i * 2 + 1] * v_fftWorkBuf[i * 2 + 1]);
        }

        v_chunkCount++;
        vTaskDelay(pdMS_TO_TICKS(5));
        #ifdef ESP_IDF_VERSION
            esp_task_wdt_reset(); // [교정] WDT 패닉 방어
        #endif
    }

    v_file.close();
    heap_caps_free(v_chunk);

    if (v_chunkCount == 0) {
        heap_caps_free(v_fftWorkBuf); heap_caps_free(v_powerAcc);
        SD_MMC.remove(p_path);
        return;
    }

    T2_Type::DynamicConfig v_cfg = CL_T2_ConfigManager::getInstance().getConfig();

    for (uint16_t i = 0; i <= v_samples / 2; i++) {
        v_powerAcc[i] /= (float)v_chunkCount;
    }

    float v_binRes = (float)T2_Def::System::RATE_AUDIO_CONST / T2_Def::System::FFT_SIZE_AUDIO_CONST;
    uint16_t v_refBin = (uint16_t)(1000.0f / v_binRes);
    float v_refAmp = sqrtf(fmaxf(v_powerAcc[v_refBin], T2_Def::System::MATH_EPSILON_12_CONST));

    memset(v_fftWorkBuf, 0, v_samples * 2 * sizeof(float));
    uint16_t v_minBin = (uint16_t)(100.0f / v_binRes);
    uint16_t v_maxBin = (uint16_t)(8000.0f / v_binRes);

    for (uint16_t i = 0; i <= v_samples / 2; i++) {
        float v_amp = sqrtf(fmaxf(v_powerAcc[i], T2_Def::System::MATH_EPSILON_12_CONST));
        float v_targetGain = 1.0f;

        if (i >= v_minBin && i <= v_maxBin) {
            v_targetGain = v_refAmp / v_amp;
            if (v_targetGain > 3.0f) v_targetGain = 3.0f;
            if (v_targetGain < 0.3f) v_targetGain = 0.3f;
        }

        v_fftWorkBuf[i * 2] = v_targetGain;
        v_fftWorkBuf[i * 2 + 1] = 0.0f;

        if (i > 0 && i < v_samples / 2) {
            v_fftWorkBuf[(v_samples - i) * 2] = v_targetGain;
            v_fftWorkBuf[(v_samples - i) * 2 + 1] = 0.0f;
        }
    }

    v_fftWorkBuf[0] = 1.0f; v_fftWorkBuf[1] = 0.0f;
    v_fftWorkBuf[v_samples] = 1.0f; v_fftWorkBuf[v_samples + 1] = 0.0f;

    dsps_fft2r_fc32(v_fftWorkBuf, v_samples);
    dsps_bit_rev2r_fc32(v_fftWorkBuf, v_samples);
    for (uint32_t i = 0; i < v_samples * 2; i++) v_fftWorkBuf[i] /= (float)v_samples;

    const uint16_t v_taps = T2_Def::FeatureLimit::FIR_TAPS_CONST;
    uint16_t v_center = v_taps / 2;
    alignas(16) float v_win[128] = {0};
    dsps_wind_blackman_f32(v_win, v_taps);

    memset(v_cfg.dsp.calib_eq_coeffs, 0, sizeof(v_cfg.dsp.calib_eq_coeffs));

    float v_sumAbs = 0.0f;
    for (int16_t i = 0; i < v_taps; i++) {
        int16_t v_timeIdx = i - v_center;
        uint16_t v_fftIdx = (v_timeIdx >= 0) ? v_timeIdx : (v_samples + v_timeIdx);
        v_cfg.dsp.calib_eq_coeffs[i] = v_fftWorkBuf[v_fftIdx * 2] * v_win[i];
        v_sumAbs += fabsf(v_cfg.dsp.calib_eq_coeffs[i]);
    }

    float v_normSafe = 1.5f;
    if (v_sumAbs > v_normSafe) {
        float v_scaleDown = v_normSafe / v_sumAbs;
        for (int16_t i = 0; i < v_taps; i++) v_cfg.dsp.calib_eq_coeffs[i] *= v_scaleDown;
    }

    CL_T2_ConfigManager::getInstance().updateConfig(v_cfg);
    if (_refExtractor) _refExtractor->resetNoiseProfile(); // 노이즈 프로필 리셋 연동

    heap_caps_free(v_fftWorkBuf);
    heap_caps_free(v_powerAcc);

    SD_MMC.remove(p_path);
    ESP_LOGI(TAG, "Manual Calibration Completed. 63-Tap Inverse FIR Computed.");
}

