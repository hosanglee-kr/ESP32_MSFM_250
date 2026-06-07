/* ============================================================================
 * File: T280_Calibrator_243.cpp
 * Summary: v243 멀티모달 캘리브레이션 엔진 구현부
 * ========================================================================== */
#include "T280_Calibrator_243.hpp"
#include "T220_CfgMgr_243.hpp"
#include "esp_log.h"
#include "dsps_wind.h"
#include "dsps_fft2r.h"
#include "esp_task_wdt.h"
#include <SD_MMC.h>
#include <cstring>
#include <cmath>

static const char* TAG = "T243_CAL";

// FSM 명령 하달 인터페이스 (외부)
extern void T240_DispatchCommand(T2_Type::SystemCommand p_cmd);

CL_T2_Calibrator::CL_T2_Calibrator() {}
CL_T2_Calibrator::~CL_T2_Calibrator() {
    if (_hCalibTask) vTaskDelete(_hCalibTask);
}

bool CL_T2_Calibrator::startAutoCalibration(const char* p_rawFilePath) {
    if (_hCalibTask != nullptr) return false;

    CalibTaskParam* v_param = new CalibTaskParam();
    v_param->instance = this;
    v_param->isManual = false;

    if (p_rawFilePath) {
        strlcpy(v_param->filePath, p_rawFilePath, sizeof(v_param->filePath));
    } else {
        if (!_findLatestCalibFile(v_param->filePath, sizeof(v_param->filePath), false)) {
            ESP_LOGE(TAG, "No Auto Calib file found!");
            delete v_param;
            return false;
        }
    }

    xTaskCreatePinnedToCore(_calibTaskProc, "CalAuto", 8192, v_param, 2, &_hCalibTask, 1);
    return true;
}

bool CL_T2_Calibrator::startManualCalibration(const char* p_rawFilePath) {
    if (_hCalibTask != nullptr) return false;

    CalibTaskParam* v_param = new CalibTaskParam();
    v_param->instance = this;
    v_param->isManual = true;

    if (p_rawFilePath) {
        strlcpy(v_param->filePath, p_rawFilePath, sizeof(v_param->filePath));
    } else {
        if (!_findLatestCalibFile(v_param->filePath, sizeof(v_param->filePath), true)) {
            ESP_LOGE(TAG, "No Manual Calib file found!");
            delete v_param;
            return false;
        }
    }

    xTaskCreatePinnedToCore(_calibTaskProc, "CalMan", 16384, v_param, 2, &_hCalibTask, 1);
    return true;
}

void CL_T2_Calibrator::_calibTaskProc(void* p_param) {
    CalibTaskParam* v_param = (CalibTaskParam*)p_param;

    if (v_param->isManual) v_param->instance->_processManual(v_param->filePath);
    else v_param->instance->_processAuto(v_param->filePath);

    T240_DispatchCommand(T2_Type::SystemCommand::CMD_STOP);
    v_param->instance->_hCalibTask = nullptr;
    delete v_param;
    vTaskDelete(NULL);
}

void CL_T2_Calibrator::_processAuto(const char* p_path) {
    ESP_LOGI(TAG, "Starting Auto Noise Profiling: %s", p_path);

    File v_file = SD_MMC.open(p_path, "r");
    if (!v_file) return;

    v_file.seek(sizeof(T2_Type::FileHeader)); 

    double v_sumSqVib[T2_Def::Vib::Sensor::AXIS_MAX] = {0.0};
    double v_sumSqAudio = 0.0;
    uint32_t v_totalChunks = 0;

    T2_Type::UnifiedRawChunk* v_chunk = (T2_Type::UnifiedRawChunk*)heap_caps_aligned_alloc(16, sizeof(T2_Type::UnifiedRawChunk), MALLOC_CAP_SPIRAM);
    if (!v_chunk) { v_file.close(); return; }

    while (v_file.available() >= sizeof(T2_Type::UnifiedRawChunk)) {
        v_file.read((uint8_t*)v_chunk, sizeof(T2_Type::UnifiedRawChunk));

        for (int a = 0; a < T2_Def::Vib::Sensor::AXIS_MAX; a++) {
            for (int i = 0; i < T2_Def::Vib::Sensor::FFT_SIZE_MAX; i++) {
                v_sumSqVib[a] += (double)(v_chunk->vib[a][i] * v_chunk->vib[a][i]);
            }
        }
        for (int i = 0; i < T2_Def::Audio::Sensor::FFT_SIZE_MAX; i++) {
            v_sumSqAudio += (double)(v_chunk->audio_l[i] * v_chunk->audio_l[i]);
        }
        v_totalChunks++;
        esp_task_wdt_reset();
    }

    heap_caps_free(v_chunk);
    v_file.close();

    if (v_totalChunks == 0) { SD_MMC.remove(p_path); return; }

    float v_baseVib = 0.0f;
    for (int a = 0; a < T2_Def::Vib::Sensor::AXIS_MAX; a++) {
        float v_rms = (float)sqrt(v_sumSqVib[a] / (v_totalChunks * T2_Def::Vib::Sensor::FFT_SIZE_MAX));
        if (v_rms > v_baseVib) v_baseVib = v_rms;
    }
    float v_baseAud = (float)sqrt(v_sumSqAudio / (v_totalChunks * T2_Def::Audio::Sensor::FFT_SIZE_MAX));

    // v243 4-Tier 설정 반영 (노이즈의 3배를 임계치로 자동 설정)
    T2_Type::DynamicConfig v_cfg = CL_T2_ConfigManager::getInstance().getConfig();
    v_cfg.vib_trig.rms_thresh = fmaxf(v_baseVib * 3.0f, 0.01f);
    v_cfg.aud_trig.rms_thresh = fmaxf(v_baseAud * 3.0f, 0.005f);
    CL_T2_ConfigManager::getInstance().updateConfig(v_cfg);

    // 역사 기록 (Predictive Maintenance)
    File v_csv = SD_MMC.open("/t240_data/auto_history.csv", "a");
    if (v_csv) {
        v_csv.printf("%llu,%.6f,%.6f\n", (uint64_t)time(NULL), v_baseVib, v_baseAud);
        v_csv.close();
    }

    SD_MMC.remove(p_path);
    ESP_LOGI(TAG, "Auto Calib Completed. Vib:%.5f, Aud:%.5f", v_baseVib, v_baseAud);
}

void CL_T2_Calibrator::_processManual(const char* p_path) {
    ESP_LOGI(TAG, "Starting Advanced FIR-Inverse Calib: %s", p_path);

    File v_file = SD_MMC.open(p_path, "r");
    if (!v_file) return;

    v_file.seek(sizeof(T2_Type::FileHeader));

    const uint32_t v_samples = T2_Def::Audio::Sensor::FFT_SIZE_MAX;
    float* v_fftWork = (float*)heap_caps_aligned_alloc(16, v_samples * 2 * sizeof(float), MALLOC_CAP_SPIRAM);
    float* v_powerAcc = (float*)heap_caps_aligned_alloc(16, (v_samples / 2 + 1) * sizeof(float), MALLOC_CAP_SPIRAM);
    T2_Type::UnifiedRawChunk* v_chunk = (T2_Type::UnifiedRawChunk*)heap_caps_aligned_alloc(16, sizeof(T2_Type::UnifiedRawChunk), MALLOC_CAP_SPIRAM);

    if (!v_fftWork || !v_powerAcc || !v_chunk) {
        if(v_fftWork) heap_caps_free(v_fftWork);
        if(v_powerAcc) heap_caps_free(v_powerAcc);
        if(v_chunk) heap_caps_free(v_chunk);
        v_file.close(); return;
    }

    memset(v_powerAcc, 0, (v_samples / 2 + 1) * sizeof(float));
    uint32_t v_count = 0;

    while (v_file.available() >= sizeof(T2_Type::UnifiedRawChunk)) {
        v_file.read((uint8_t*)v_chunk, sizeof(T2_Type::UnifiedRawChunk));

        for (uint32_t i = 0; i < v_samples; i++) {
            v_fftWork[i * 2] = v_chunk->audio_l[i];
            v_fftWork[i * 2 + 1] = 0.0f;
        }

        dsps_fft2r_fc32(v_fftWork, v_samples);
        dsps_bit_rev2r_fc32(v_fftWork, v_samples);

        for (uint16_t i = 0; i <= v_samples / 2; i++) {
            v_powerAcc[i] += (v_fftWork[i * 2] * v_fftWork[i * 2] + v_fftWork[i * 2 + 1] * v_fftWork[i * 2 + 1]);
        }
        v_count++;
        esp_task_wdt_reset();
    }
    v_file.close();
    heap_caps_free(v_chunk);

    if (v_count > 0) {
        T2_Type::DynamicConfig v_cfg = CL_T2_ConfigManager::getInstance().getConfig();
        float v_binHz = (float)v_cfg.aud_sensor.sample_rate / v_samples;
        
        // 1. 평균 파워 스펙트럼 도출
        for(uint16_t i=0; i<=v_samples/2; i++) v_powerAcc[i] /= v_count;

        // 2. 기준 주파수(Ref) 진폭 산출
        uint16_t v_refBin = (uint16_t)(v_cfg.aud_calib.ref_freq / v_binHz);
        float v_refAmp = sqrtf(fmaxf(v_powerAcc[v_refBin], T2_Def::Global::System::MATH_EPSILON_12_CONST));

        // 3. 역산 스펙트럼 구성 (Inverse Spectrum)
        memset(v_fftWork, 0, v_samples * 2 * sizeof(float));
        uint16_t v_minBin = (uint16_t)(v_cfg.aud_calib.filt_min / v_binHz);
        uint16_t v_maxBin = (uint16_t)(v_cfg.aud_calib.filt_max / v_binHz);

        for (uint16_t i = 0; i <= v_samples / 2; i++) {
            float v_amp = sqrtf(fmaxf(v_powerAcc[i], T2_Def::Global::System::MATH_EPSILON_12_CONST));
            float v_targetGain = 1.0f;

            if (i >= v_minBin && i <= v_maxBin) {
                v_targetGain = v_refAmp / v_amp;
                // 설정 기반 하드 리미트 적용
                if (v_targetGain > v_cfg.aud_calib.gain_max) v_targetGain = v_cfg.aud_calib.gain_max;
                if (v_targetGain < v_cfg.aud_calib.gain_min) v_targetGain = v_cfg.aud_calib.gain_min;
            }

            v_fftWork[i * 2] = v_targetGain;
            v_fftWork[i * 2 + 1] = 0.0f;
            
            // Symmetric Copy for IFFT
            if (i > 0 && i < v_samples / 2) {
                v_fftWork[(v_samples - i) * 2] = v_targetGain;
                v_fftWork[(v_samples - i) * 2 + 1] = 0.0f;
            }
        }

        // [v013] DC/Nyquist Masking (Bypass)
        v_fftWork[0] = 1.0f; v_fftWork[1] = 0.0f;
        v_fftWork[v_samples] = 1.0f; v_fftWork[v_samples + 1] = 0.0f;

        // 4. IFFT (Time Domain 변환)
        dsps_fft2r_fc32(v_fftWork, v_samples);
        dsps_bit_rev2r_fc32(v_fftWork, v_samples);
        for (uint32_t i = 0; i < v_samples * 2; i++) v_fftWork[i] /= (float)v_samples;
        
        // 5. Shifting & Windowing
        uint16_t v_taps = T2_Def::Shared::Feature::FIR_TAPS_DEF;
        uint16_t v_center = v_taps / 2;
        alignas(16) float v_win[128];
        dsps_wind_blackman_f32(v_win, v_taps);

        float v_sumAbs = 0.0f;
        for (int i = 0; i < v_taps; i++) {
            int16_t v_tIdx = i - v_center;
            uint16_t v_fIdx = (v_tIdx >= 0) ? v_tIdx : (v_samples + v_tIdx);
            v_cfg.aud_calib.eq_coeffs[i] = v_fftWork[v_fIdx * 2] * v_win[i];
            v_sumAbs += fabsf(v_cfg.aud_calib.eq_coeffs[i]);
        }

        // [v013] L1-Norm 정규화 (클리핑 방지)
        if (v_sumAbs > v_cfg.aud_calib.norm_safe) {
            float v_scale = v_cfg.aud_calib.norm_safe / v_sumAbs;
            for (int i = 0; i < v_taps; i++) v_cfg.aud_calib.eq_coeffs[i] *= v_scale;
            ESP_LOGW(TAG, "FIR Normalized by L1-Norm: %.3f", v_scale);
        }

        CL_T2_ConfigManager::getInstance().updateConfig(v_cfg);
        
        // 노이즈 프로필 및 히스토리 초기화 (정합성 확보)
        if (_refExtractor) {
            _refExtractor->resetNoiseProfile();
            _refExtractor->resetHistory();
        }
    }

    heap_caps_free(v_fftWork);
    heap_caps_free(v_powerAcc);
    SD_MMC.remove(p_path);
    ESP_LOGI(TAG, "Advanced Manual Calib Completed.");
}

bool CL_T2_Calibrator::_findLatestCalibFile(char* p_outPath, size_t p_maxLen, bool p_isManual) {
    File v_dir = SD_MMC.open(T2_Def::Global::Storage::SD_DIR_RAW_CONST);
    if (!v_dir || !v_dir.isDirectory()) return false;

    time_t v_latest = 0;
    bool v_found = false;
    const char* v_pref = p_isManual ? "calib_man" : "calib_auto";

    File v_file = v_dir.openNextFile();
    while (v_file) {
        if (!v_file.isDirectory() && strstr(v_file.name(), v_pref)) {
            time_t v_t = v_file.getLastWrite();
            if (v_t > v_latest) {
                v_latest = v_t;
                snprintf(p_outPath, p_maxLen, "%s/%s", T2_Def::Global::Storage::SD_DIR_RAW_CONST, v_file.name());
                v_found = true;
            }
        }
        v_file = v_dir.openNextFile();
    }
    return v_found;
}
