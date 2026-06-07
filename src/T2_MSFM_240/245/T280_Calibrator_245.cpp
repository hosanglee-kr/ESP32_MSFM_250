/* ============================================================================
 * File: T280_Calibrator_245.cpp
 * Summary: 멀티모달 캘리브레이션 엔진 구현부
 * ========================================================================== */
#include "T280_Calibrator_245.hpp"
#include "T220_CfgMgr_245.hpp"
#include "esp_log.h"
#include "dsps_wind.h"
#include "dsps_fft2r.h"
#include "esp_task_wdt.h"
#include <SD_MMC.h>
#include <cstring>
#include <cmath>
#include <string>
#include <functional>

static const char* TAG = "T245_CAL";

// FSM 명령 하달 인터페이스 (외부)
extern void T240_DispatchCommand(T2_Type::EM_SystemCommand_t p_cmd);

/**
 * @brief 파일 확장자 치환 및 디렉터리 경로 변경 유틸리티
 * @details "/bin/"을 "/raw/"로 대체하고 확장자를 새 확장자로 치환
 * @param p_src 원본 파일 경로
 * @param p_dest 치환 결과가 담길 목적지 버퍼
 * @param p_maxLen 목적지 버퍼 최대 크기
 * @param p_newExt 새로 지정할 확장자 (예: "acc", "gyr", "wav")
 */
static void _replacePath(const char* p_src, char* p_dest, size_t p_maxLen, const char* p_newExt) {
    std::string v_str(p_src);
    size_t v_pos = v_str.find("/bin/");
    if (v_pos != std::string::npos) {
        v_str.replace(v_pos, 5, "/raw/");
    }
    size_t v_dot = v_str.find_last_of('.');
    if (v_dot != std::string::npos) {
        v_str = v_str.substr(0, v_dot) + "." + p_newExt;
    }
    strlcpy(p_dest, v_str.c_str(), p_maxLen);
}

/**
 * @brief CL_T2_Calibrator 생성자
 */
CL_T2_Calibrator::CL_T2_Calibrator() {}

/**
 * @brief CL_T2_Calibrator 소멸자
 * @details 실행 중인 캘리브레이션 태스크가 있을 경우 삭제 수행
 */
CL_T2_Calibrator::~CL_T2_Calibrator() {
    if (_hCalibTask) vTaskDelete(_hCalibTask);
}

/**
 * @brief 자동 캘리브레이션 태스크 시작
 * @details 비동기로 신속 캘리브레이션을 수행할 워커 태스크를 생성
 * @param p_rawFilePath 대상 캘리브레이션 파일 경로 (Null 시 최신 파일 검색)
 * @return 태스크 생성 성공 여부
 */
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

/**
 * @brief 수동(정밀) 캘리브레이션 태스크 시작
 * @details 비동기로 고급 FIR 역필터 연산을 수행할 워커 태스크를 생성
 * @param p_rawFilePath 대상 캘리브레이션 파일 경로 (Null 시 최신 파일 검색)
 * @return 태스크 생성 성공 여부
 */
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

/**
 * @brief 캘리브레이션 비동기 FreeRTOS 태스크 프로시저
 * @param p_param CalibTaskParam 파라미터 포인터
 */
void CL_T2_Calibrator::_calibTaskProc(void* p_param) {
    CalibTaskParam* v_param = (CalibTaskParam*)p_param;

    if (v_param->isManual) v_param->instance->_processManual(v_param->filePath);
    else v_param->instance->_processAuto(v_param->filePath);

    T240_DispatchCommand(T2_Type::EM_SystemCommand_t::CMD_STOP);
    v_param->instance->_hCalibTask = nullptr;
    delete v_param;
    vTaskDelete(NULL);
}

/**
 * @brief 자동 캘리브레이션 처리 코어 루틴
 * @details 가속도, 자이로, 오디오 원시 파일을 열어 RMS 기본 노이즈 레벨 측정 및 트리거 문턱값 산출
 * @param p_path 기준 매핑 파일 경로
 */
void CL_T2_Calibrator::_processAuto(const char* p_path) {
    ESP_LOGI(TAG, "Starting Auto Noise Profiling: %s", p_path);

    char v_accPath[T2_Def::Global::StorageLimit::PATH_LEN_MAX];
    char v_gyrPath[T2_Def::Global::StorageLimit::PATH_LEN_MAX];
    char v_wavPath[T2_Def::Global::StorageLimit::PATH_LEN_MAX];
    _replacePath(p_path, v_accPath, sizeof(v_accPath), "acc");
    _replacePath(p_path, v_gyrPath, sizeof(v_gyrPath), "gyr");
    _replacePath(p_path, v_wavPath, sizeof(v_wavPath), "wav");

    // [처리 단위 1] Accel RMS 계산
    double v_sumSqAcc[T2_Def::Accel::Sensor::AXIS_MAX] = {0.0};
    uint32_t v_totalAccChunks = 0;
    File v_accFile = SD_MMC.open(v_accPath, "r");
    if (v_accFile) {
        T2_Type::ST_Raw_Accel_t* v_accChunk = (T2_Type::ST_Raw_Accel_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_Raw_Accel_t), MALLOC_CAP_SPIRAM);
        if (v_accChunk) {
            while (v_accFile.available() >= sizeof(T2_Type::ST_Raw_Accel_t)) {
                v_accFile.read((uint8_t*)v_accChunk, sizeof(T2_Type::ST_Raw_Accel_t));
                for (int a = 0; a < T2_Def::Accel::Sensor::AXIS_MAX; a++) {
                    for (int i = 0; i < T2_Def::Accel::Sensor::FFT_SIZE_MAX; i++) {
                        v_sumSqAcc[a] += (double)(v_accChunk->data[a][i] * v_accChunk->data[a][i]);
                    }
                }
                v_totalAccChunks++;
                esp_task_wdt_reset();
            }
            heap_caps_free(v_accChunk);
        }
        v_accFile.close();
    }

    // [처리 단위 2] Gyro RMS 계산
    double v_sumSqGyr[T2_Def::Gyro::Sensor::AXIS_MAX] = {0.0};
    uint32_t v_totalGyrChunks = 0;
    File v_gyrFile = SD_MMC.open(v_gyrPath, "r");
    if (v_gyrFile) {
        T2_Type::ST_Raw_Gyro_t* v_gyrChunk = (T2_Type::ST_Raw_Gyro_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_Raw_Gyro_t), MALLOC_CAP_SPIRAM);
        if (v_gyrChunk) {
            while (v_gyrFile.available() >= sizeof(T2_Type::ST_Raw_Gyro_t)) {
                v_gyrFile.read((uint8_t*)v_gyrChunk, sizeof(T2_Type::ST_Raw_Gyro_t));
                for (int a = 0; a < T2_Def::Gyro::Sensor::AXIS_MAX; a++) {
                    for (int i = 0; i < T2_Def::Gyro::Sensor::FFT_SIZE_MAX; i++) {
                        v_sumSqGyr[a] += (double)(v_gyrChunk->data[a][i] * v_gyrChunk->data[a][i]);
                    }
                }
                v_totalGyrChunks++;
                esp_task_wdt_reset();
            }
            heap_caps_free(v_gyrChunk);
        }
        v_gyrFile.close();
    }

    // [처리 단위 3] Audio RMS 계산
    double v_sumSqAudioL = 0.0;
    double v_sumSqAudioR = 0.0;
    uint32_t v_totalAudChunks = 0;
    File v_wavFile = SD_MMC.open(v_wavPath, "r");
    if (v_wavFile) {
        v_wavFile.seek(44); // WAV 헤더 건너뛰기
        float* v_audBuf = (float*)heap_caps_aligned_alloc(16, T2_Def::Audio::Sensor::FFT_SIZE_MAX * 2 * sizeof(float), MALLOC_CAP_SPIRAM);
        if (v_audBuf) {
            size_t v_readBytes = T2_Def::Audio::Sensor::FFT_SIZE_MAX * 2 * sizeof(float);
            while (v_wavFile.available() >= v_readBytes) {
                v_wavFile.read((uint8_t*)v_audBuf, v_readBytes);
                for (int i = 0; i < T2_Def::Audio::Sensor::FFT_SIZE_MAX; i++) {
                    v_sumSqAudioL += (double)(v_audBuf[i * 2] * v_audBuf[i * 2]);
                    v_sumSqAudioR += (double)(v_audBuf[i * 2 + 1] * v_audBuf[i * 2 + 1]);
                }
                v_totalAudChunks++;
                esp_task_wdt_reset();
            }
            heap_caps_free(v_audBuf);
        }
        v_wavFile.close();
    }

    // [처리 단위 4] 기준 센서 스케일 계산 및 임계 설정 덮어쓰기
    T2_Type::ST_DynamicConfig_t v_cfg = CL_T2_ConfigManager::getInstance().getConfig();

    float v_baseAcc = 0.0f;
    if (v_totalAccChunks > 0) {
        for (int a = 0; a < T2_Def::Accel::Sensor::AXIS_MAX; a++) {
            float v_rms = (float)sqrt(v_sumSqAcc[a] / (v_totalAccChunks * T2_Def::Accel::Sensor::FFT_SIZE_MAX));
            v_cfg.accel.rms_thresh[a] = fmaxf(v_rms * 3.0f, 0.01f);
            if (v_rms > v_baseAcc) v_baseAcc = v_rms;
        }
    }

    float v_baseGyr = 0.0f;
    if (v_totalGyrChunks > 0) {
        for (int a = 0; a < T2_Def::Gyro::Sensor::AXIS_MAX; a++) {
            float v_rms = (float)sqrt(v_sumSqGyr[a] / (v_totalGyrChunks * T2_Def::Gyro::Sensor::FFT_SIZE_MAX));
            v_cfg.gyro.rms_thresh[a] = fmaxf(v_rms * 3.0f, 0.01f);
            if (v_rms > v_baseGyr) v_baseGyr = v_rms;
        }
    }

    float v_baseAudL = 0.0f;
    float v_baseAudR = 0.0f;
    if (v_totalAudChunks > 0) {
        v_baseAudL = (float)sqrt(v_sumSqAudioL / (v_totalAudChunks * T2_Def::Audio::Sensor::FFT_SIZE_MAX));
        v_baseAudR = (float)sqrt(v_sumSqAudioR / (v_totalAudChunks * T2_Def::Audio::Sensor::FFT_SIZE_MAX));
        float v_maxBaseAud = fmaxf(v_baseAudL, v_baseAudR);
        for (int ch = 0; ch < 2; ch++) {
            v_cfg.audio.rms_thresh[ch] = fmaxf(v_maxBaseAud * 3.0f, 0.005f);
        }
    }

    CL_T2_ConfigManager::getInstance().updateConfig(v_cfg);

    // [처리 단위 5] 프로파일 히스토리 기록 및 임시 파일 정리
    File v_csv = SD_MMC.open("/t240_data/auto_history.csv", "a");
    if (v_csv) {
        v_csv.printf("%llu,%.6f,%.6f,%.6f,%.6f\n", (uint64_t)time(NULL), v_baseAcc, v_baseGyr, v_baseAudL, v_baseAudR);
        v_csv.close();
    }

    SD_MMC.remove(p_path);
    SD_MMC.remove(v_accPath);
    SD_MMC.remove(v_gyrPath);
    SD_MMC.remove(v_wavPath);

    ESP_LOGI(TAG, "Auto Calib Completed. Acc:%.5f, Gyr:%.5f, AudL:%.5f, AudR:%.5f", v_baseAcc, v_baseGyr, v_baseAudL, v_baseAudR);
}

/**
 * @brief 수동(정밀) 캘리브레이션 처리 코어 루틴
 * @details 오디오 주파수 스펙트럼 분석 후 오프라인 Welch 연산으로 역 FIR 이퀄라이저 필터 계수를 생성
 * @param p_path 기준 매핑 파일 경로
 */
void CL_T2_Calibrator::_processManual(const char* p_path) {
    ESP_LOGI(TAG, "Starting Advanced FIR-Inverse Calib: %s", p_path);

    char v_accPath[T2_Def::Global::StorageLimit::PATH_LEN_MAX];
    char v_gyrPath[T2_Def::Global::StorageLimit::PATH_LEN_MAX];
    char v_wavPath[T2_Def::Global::StorageLimit::PATH_LEN_MAX];
    _replacePath(p_path, v_accPath, sizeof(v_accPath), "acc");
    _replacePath(p_path, v_gyrPath, sizeof(v_gyrPath), "gyr");
    _replacePath(p_path, v_wavPath, sizeof(v_wavPath), "wav");

    File v_wavFile = SD_MMC.open(v_wavPath, "r");
    if (!v_wavFile) {
        SD_MMC.remove(p_path);
        SD_MMC.remove(v_accPath);
        SD_MMC.remove(v_gyrPath);
        return;
    }

    v_wavFile.seek(44); // WAV 헤더 스킵

    // [처리 단위 1] PSRAM 메모리 정렬 및 FFT 버퍼 할당
    const uint32_t v_samples = T2_Def::Audio::Sensor::FFT_SIZE_MAX;
    float* v_fftWork = (float*)heap_caps_aligned_alloc(16, v_samples * 2 * sizeof(float), MALLOC_CAP_SPIRAM);
    float* v_powerAcc = (float*)heap_caps_aligned_alloc(16, (v_samples / 2 + 1) * sizeof(float), MALLOC_CAP_SPIRAM);

    if (!v_fftWork || !v_powerAcc) {
        if (v_fftWork) heap_caps_free(v_fftWork);
        if (v_powerAcc) heap_caps_free(v_powerAcc);
        v_wavFile.close();
        SD_MMC.remove(p_path);
        SD_MMC.remove(v_accPath);
        SD_MMC.remove(v_gyrPath);
        SD_MMC.remove(v_wavPath);
        return;
    }

    memset(v_powerAcc, 0, (v_samples / 2 + 1) * sizeof(float));
    uint32_t v_count = 0;

    // [처리 단위 2] 오디오 프레임 단위 FFT 연산 및 파워 스펙트럼 누적
    float* v_audBuf = (float*)heap_caps_aligned_alloc(16, v_samples * 2 * sizeof(float), MALLOC_CAP_SPIRAM);
    if (v_audBuf) {
        size_t v_readBytes = v_samples * 2 * sizeof(float);
        while (v_wavFile.available() >= v_readBytes) {
            v_wavFile.read((uint8_t*)v_audBuf, v_readBytes);

            for (uint32_t i = 0; i < v_samples; i++) {
                v_fftWork[i * 2] = (v_audBuf[i * 2] + v_audBuf[i * 2 + 1]) * 0.5f;
                v_fftWork[i * 2 + 1] = 0.0f;
            }

            dsps_fft2r_fc32(v_fftWork, v_samples);
            dsps_bit_rev_fc32(v_fftWork, v_samples);

            for (uint16_t i = 0; i <= v_samples / 2; i++) {
                v_powerAcc[i] += (v_fftWork[i * 2] * v_fftWork[i * 2] + v_fftWork[i * 2 + 1] * v_fftWork[i * 2 + 1]);
            }
            v_count++;
            esp_task_wdt_reset();
        }
        heap_caps_free(v_audBuf);
    }
    v_wavFile.close();

    // [처리 단위 3] 역산 타겟 게인 스펙트럼 생성 및 IFFT 변환
    if (v_count > 0) {
        T2_Type::ST_DynamicConfig_t v_cfg = CL_T2_ConfigManager::getInstance().getConfig();
        float v_binHz = (float)v_cfg.audio.sample_rate / v_samples;

        // 평균 파워 스펙트럼 도출
        for (uint16_t i = 0; i <= v_samples / 2; i++) v_powerAcc[i] /= v_count;

        // 기준 주파수(Ref) 진폭 산출
        uint16_t v_refBin = (uint16_t)(v_cfg.audio.ref_freq / v_binHz);
        float v_refAmp = sqrtf(fmaxf(v_powerAcc[v_refBin], T2_Def::Global::System::MATH_EPSILON_12_CONST));

        // 역산 스펙트럼 구성 (Inverse Spectrum)
        memset(v_fftWork, 0, v_samples * 2 * sizeof(float));
        uint16_t v_minBin = (uint16_t)(v_cfg.audio.filt_min / v_binHz);
        uint16_t v_maxBin = (uint16_t)(v_cfg.audio.filt_max / v_binHz);

        for (uint16_t i = 0; i <= v_samples / 2; i++) {
            float v_amp = sqrtf(fmaxf(v_powerAcc[i], T2_Def::Global::System::MATH_EPSILON_12_CONST));
            float v_targetGain = 1.0f;

            if (i >= v_minBin && i <= v_maxBin) {
                v_targetGain = v_refAmp / v_amp;
                if (v_targetGain > v_cfg.audio.gain_max) v_targetGain = v_cfg.audio.gain_max;
                if (v_targetGain < v_cfg.audio.gain_min) v_targetGain = v_cfg.audio.gain_min;
            }

            v_fftWork[i * 2] = v_targetGain;
            v_fftWork[i * 2 + 1] = 0.0f;

            if (i > 0 && i < v_samples / 2) {
                v_fftWork[(v_samples - i) * 2] = v_targetGain;
                v_fftWork[(v_samples - i) * 2 + 1] = 0.0f;
            }
        }

        v_fftWork[0] = 1.0f; v_fftWork[1] = 0.0f;
        v_fftWork[v_samples] = 1.0f; v_fftWork[v_samples + 1] = 0.0f;

        // IFFT (Time Domain 변환) - esp-dsp 미지원으로 수동 구현: FFT -> BitRev -> Conjugate/Scaling
        dsps_fft2r_fc32(v_fftWork, v_samples);
        dsps_bit_rev_fc32(v_fftWork, v_samples);
        float v_invN = 1.0f / (float)v_samples;
        for (uint32_t i = 0; i < v_samples; i++) {
            v_fftWork[i * 2]     *= v_invN;
            v_fftWork[i * 2 + 1] *= -v_invN;
        }

        // [처리 단위 4] 블랙맨 윈도우 스케일링 및 L1 노멀라이즈 적용
        uint16_t v_taps = T2_Def::Audio::FeatureLimit::FIR_TAPS_DEF;
        uint16_t v_center = v_taps / 2;
        alignas(16) float v_win[T2_Def::Audio::FeatureLimit::FIR_TAPS_MAX];
        dsps_wind_blackman_f32(v_win, v_taps);

        float v_sumAbs = 0.0f;
        for (int i = 0; i < v_taps; i++) {
            int16_t v_tIdx = i - v_center;
            uint16_t v_fIdx = (v_tIdx >= 0) ? v_tIdx : (v_samples + v_tIdx);
            float v_val = v_fftWork[v_fIdx * 2] * v_win[i];

            v_cfg.audio.eq_coeffs[0][i] = v_val;
            v_cfg.audio.eq_coeffs[1][i] = v_val;
            v_sumAbs += fabsf(v_val);
        }

        if (v_sumAbs > v_cfg.audio.norm_safe) {
            float v_scale = v_cfg.audio.norm_safe / v_sumAbs;
            for (int i = 0; i < v_taps; i++) {
                v_cfg.audio.eq_coeffs[0][i] *= v_scale;
                v_cfg.audio.eq_coeffs[1][i] *= v_scale;
            }
            ESP_LOGW(TAG, "FIR Normalized by L1-Norm: %.3f", v_scale);
        }

        CL_T2_ConfigManager::getInstance().updateConfig(v_cfg);

        if (_refExtractor) {
            _refExtractor->resetNoiseProfile();
            _refExtractor->resetHistory();
        }
    }

    // [처리 단위 5] 리소스 해제 및 임시 원시 파일 일체 정리
    heap_caps_free(v_fftWork);
    heap_caps_free(v_powerAcc);

    SD_MMC.remove(p_path);
    SD_MMC.remove(v_accPath);
    SD_MMC.remove(v_gyrPath);
    SD_MMC.remove(v_wavPath);

    ESP_LOGI(TAG, "Advanced Manual Calib Completed.");
}

/**
 * @brief SD 카드에서 가장 최근에 갱신된 캘리브레이션 입력용 raw 파일 검색
 * @param p_outPath 검색된 경로명을 반환받을 문자열 포인터
 * @param p_maxLen 문자열 버퍼 최대 크기
 * @param p_isManual 수동 캘리브레이션용 calib_man 파일 찾을지 여부
 * @return 파일 발견 성공 여부
 */
bool CL_T2_Calibrator::_findLatestCalibFile(char* p_outPath, size_t p_maxLen, bool p_isManual) {
    time_t v_latest = 0;
    bool v_found = false;
    const char* v_pref = p_isManual ? "calib_man" : "calib_auto";

    std::function<void(const std::string&)> v_scanDir = [&](const std::string& p_dirPath) {
        File v_dir = SD_MMC.open(p_dirPath.c_str());
        if (!v_dir || !v_dir.isDirectory()) return;

        File v_file = v_dir.openNextFile();
        while (v_file) {
            std::string v_name = v_file.name();
            std::string v_fullPath;
            if (!v_name.empty() && v_name.front() == '/') {
                v_fullPath = v_name;
            } else {
                v_fullPath = p_dirPath + "/" + v_name;
            }

            if (v_file.isDirectory()) {
                v_scanDir(v_fullPath);
            } else {
                if (v_name.find(v_pref) != std::string::npos) {
                    time_t v_t = v_file.getLastWrite();
                    if (v_t > v_latest) {
                        v_latest = v_t;
                        strlcpy(p_outPath, v_fullPath.c_str(), p_maxLen);
                        v_found = true;
                    }
                }
            }
            v_file = v_dir.openNextFile();
        }
        v_dir.close();
    };

    v_scanDir(T2_Def::Global::Storage::SD_DIR_RAW_CONST);
    return v_found;
}

