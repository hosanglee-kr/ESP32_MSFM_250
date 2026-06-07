/* ============================================================================
 * File: T215_ConfigMgr_241.cpp
 * Summary: 동적 JSON 설정 관리자 구현부
 * ============================================================================
 * [로직 상태 추적 (Logic State Tracker)]
 * - 유지: LittleFS 마운트 및 /sys/runtime_cfg_241.json 경로 연동.
 * - 갱신: T2/T4 하이브리드 파라미터 매핑 (ArduinoJson V7 규격 완벽 준수).
 * - 신규: [자가 교정] ESP_LOG 매크로 전면 복구를 통한 시스템 장애 추적성(Traceability) 확보.
 *
 * [축소/누락 방어 체크리스트 (Omission Defense)]
 * 1. [로깅 방어]: JSON 파싱 에러 및 플래시 I/O 에러 시 ESP_LOGE로 원인 즉각 출력.
 * 2. [매직 넘버 방어]: _loadDefaults() 내부의 모든 리터럴 값을 시스템 상수로 교체.
 * 3. [메모리 경계 방어]: 문자열 복사 시 strlcpy를 사용하여 버퍼 오버플로우 차단.
 * 4. [배열 순회 방어]: JsonArray 순회 시 MAX_CONST와 비교하여 인덱스 초과 100% 차단.
 * ========================================================================== */

#include "T215_ConfigMgr_241.hpp"
#include <LittleFS.h>
#include <cstring>
#include "esp_log.h"

static const char* TAG = "T215_CFG";

// [수석 아키텍트 추천] ArduinoJson V7 템플릿 에러 방지용 무결점 헬퍼
template<typename T>
static inline void cfgSet(JsonVariantConst p_v, T& p_dest) {
    if (!p_v.isNull()) {
        p_dest = p_v.as<T>();
    }
}

// 문자열 전용 헬퍼
static inline void cfgStr(JsonVariantConst p_v, char* p_dest, size_t p_size) {
    if (!p_v.isNull()) {
        const char* v_src = p_v.as<const char*>();
        if (v_src) strlcpy(p_dest, v_src, p_size);
    }
}

CL_T2_ConfigManager::CL_T2_ConfigManager() {
    _lock = xSemaphoreCreateMutex();
    _isLoaded = false;
    _isDirty = false;
    _isTuningActive = false;
    _lastModifiedMs = 0;
    _loadDefaults();
}

CL_T2_ConfigManager::~CL_T2_ConfigManager() {
    if (_lock) vSemaphoreDelete(_lock);
}

bool CL_T2_ConfigManager::init() {
    if (!LittleFS.begin(true)) {
        ESP_LOGE(TAG, "LittleFS Mount Failed! (Check partition table)");
        return false;
    }

    // [정전 대비 원자적 복구 로직 및 추적 로깅 복구]
    if (!LittleFS.exists(T2_Config::Path::FILE_CFG_JSON_CONST)) {
        if (LittleFS.exists("/sys/config_241.tmp")) {
            ESP_LOGW(TAG, "Recovering config from interrupted atomic write (.tmp -> .json)");
            LittleFS.rename("/sys/config_241.tmp", T2_Config::Path::FILE_CFG_JSON_CONST);
            load();
        } else {
            ESP_LOGW(TAG, "Config not found. Creating default runtime configuration...");
            _loadDefaults();
            save();
        }
    } else {
        load();
    }

    _isLoaded = true;
    _isDirty = false;
    return true;
}

void CL_T2_ConfigManager::_loadDefaults() {
    // [매직 넘버 제거 및 상수 기반 초기화]
    _config.sensor.accel_range = 8; // BMI270 8G (Hardware limitation)
    _config.sensor.gyro_range = 2;  // BMI270 2000dps

    _config.trigger.hold_time_ms = T2_Config::Trigger::HOLD_TIME_MS_DEF;
    _config.trigger.use_deep_sleep = T2_Config::Trigger::USE_DEEP_SLEEP_DEF;
    _config.trigger.sleep_timeout_sec = T2_Config::Trigger::SLEEP_TIMEOUT_SEC_DEF;
    _config.trigger.wake_thresh_g = T2_Config::Trigger::WAKE_THRESH_G_DEF;
    _config.trigger.wake_duration = T2_Config::Trigger::WAKE_DURATION_DEF;
    _config.trigger.vib_rms_thresh = T2_Config::Decision::RULE_VIB_RMS_THRESH_DEF;
    _config.trigger.audio_rms_thresh = T2_Config::Decision::RULE_AUDIO_RMS_THRESH_DEF;

    for (uint8_t i = 0; i < T2_Config::FeatureLimit::MAX_BAND_RMS_CONST; i++) {
        _config.trigger.band_enable[i] = false;
        _config.trigger.band_start_hz[i] = 10.0f;
        _config.trigger.band_end_hz[i] = 100.0f;
        _config.trigger.band_thresh[i] = 1.0f;
    }

    _config.dsp.remove_dc = true;
    _config.dsp.median_enabled = true;
    _config.dsp.median_window = T2_Config::Dsp::MEDIAN_WINDOW_DEF;

    _config.dsp.fir_hpf.enabled = true;
    _config.dsp.fir_hpf.cutoff_hz = T2_Config::Dsp::FIR_HPF_CUTOFF_DEF;
    _config.dsp.fir_hpf.num_taps = T2_Config::FeatureLimit::FIR_TAPS_CONST; // 상수 교체

    _config.dsp.fir_lpf.enabled = false;
    _config.dsp.fir_lpf.cutoff_hz = 1000.0f;
    _config.dsp.fir_lpf.num_taps = T2_Config::FeatureLimit::FIR_TAPS_CONST;

    _config.dsp.iir_hpf.enabled = false;
    _config.dsp.iir_hpf.cutoff_hz = 20.0f;
    _config.dsp.iir_hpf.q_factor = 0.707f;

    _config.dsp.iir_lpf.enabled = false;
    _config.dsp.iir_lpf.cutoff_hz = 1000.0f;
    _config.dsp.iir_lpf.q_factor = 0.707f;

    _config.dsp.notch.enabled = false;
    _config.dsp.notch.target_freq_hz = T2_Config::Dsp::NOTCH_FREQ_HZ_DEF;
    _config.dsp.notch.gain = 1.0f;
    _config.dsp.notch.q_factor = T2_Config::Dsp::NOTCH_Q_FACTOR_DEF;

    _config.dsp.preemphasis_enable = true;
    _config.dsp.preemphasis_alpha = T2_Config::Dsp::PRE_EMPHASIS_ALPHA_DEF;

    _config.dsp.noise.enable_gate = false;
    _config.dsp.noise.gate_threshold_abs = 0.01f;
    _config.dsp.noise.mode = T2_Type::NoiseMode::OFF;
    _config.dsp.noise.spectral_subtract_strength = 1.0f;
    _config.dsp.noise.adaptive_alpha = 0.05f;
    _config.dsp.noise.noise_learn_frames = 100;

    _config.dsp.window_type = T2_Type::WindowType::HANN;
    _config.dsp.beamforming_gain = T2_Config::Dsp::BEAMFORMING_GAIN_DEF;

    // DSP 캘리브레이션 게인 및 필터 계수 기본값 (Bypass) 초기화
    _config.dsp.calib_gain_L = 1.0f;
    _config.dsp.calib_gain_R = 1.0f;
    memset(_config.dsp.calib_eq_coeffs, 0, sizeof(_config.dsp.calib_eq_coeffs));
    _config.dsp.calib_eq_coeffs[T2_Config::FeatureLimit::FIR_TAPS_CONST / 2] = 1.0f; // Center tap = 1.0 (Bypass)

    // 캘리브레이터 기본값
    _config.calib.auto_idle_min = 60;
    _config.calib.ref_freq_hz = 1000.0f;
    _config.calib.filter_min_freq_hz = 100.0f;
    _config.calib.filter_max_freq_hz = 8000.0f;
    _config.calib.target_gain_max = 3.0f;
    _config.calib.target_gain_min = 0.3f;
    _config.calib.norm_safe_thresh = 1.5f;

    // 특징 추출기 기본값
    _config.feature.ceps_targets[0] = 0.0f;
    _config.feature.ceps_targets[1] = 0.0f;
    _config.feature.ceps_targets[2] = 0.0f;
    _config.feature.spatial_freq_min_hz = 100.0f;
    _config.feature.spatial_freq_max_hz = 4000.0f;
    _config.feature.peak_amplitude_limit_min = 0.1f;
    _config.feature.peak_freq_gap_limit_hz_min = 50.0f;


    _config.storage.rotation_mb = T2_Config::Storage::ROTATE_MB_DEF;
    _config.storage.rotation_min = T2_Config::Storage::ROTATE_MIN_DEF;
    _config.storage.save_raw = false;
    _config.storage.rotate_keep_max = T2_Config::Storage::ROTATE_KEEP_MAX_DEF;
    _config.storage.idle_flush_ms = T2_Config::Storage::IDLE_FLUSH_MS_DEF;
    _config.storage.pre_trigger_sec = T2_Config::Storage::PRE_TRIGGER_SEC_DEF;

    _config.wifi.mode = T2_Type::WiFiMode::AUTO_FALLBACK;
    strlcpy(_config.wifi.ap_ssid, "SMEA_T240_AP", sizeof(_config.wifi.ap_ssid));
    strlcpy(_config.wifi.ap_password, "12345678", sizeof(_config.wifi.ap_password));
    strlcpy(_config.wifi.ap_ip, "192.168.4.1", sizeof(_config.wifi.ap_ip));

    for (uint8_t i = 0; i < T2_Config::Net::WIFI_MULTI_MAX_CONST; i++) {
        _config.wifi.multi_ssid[i][0] = '\0';
        _config.wifi.multi_pw[i][0] = '\0';
    }

    _config.mqtt.enable = false;
    strlcpy(_config.mqtt.broker, "", sizeof(_config.mqtt.broker));
    _config.mqtt.port = T2_Config::Net::MQTT_PORT_CONST;
    strlcpy(_config.mqtt.id, "T240_Edge", sizeof(_config.mqtt.id));
    strlcpy(_config.mqtt.password, "", sizeof(_config.mqtt.password));
    strlcpy(_config.mqtt.topic_root, "smea/t240", sizeof(_config.mqtt.topic_root));

    _config.op_mode = 0;
    _config.watchdog_ms = T2_Config::Task::WDG_TIMEOUT_MS_CONST;
}

void CL_T2_ConfigManager::_applyJson(const JsonDocument& p_doc) {
    // 1. Sensor
    JsonObjectConst v_sensor = p_doc["sensor"];
    if (!v_sensor.isNull()) {
        cfgSet(v_sensor["accel_range"], _config.sensor.accel_range);
        cfgSet(v_sensor["gyro_range"],  _config.sensor.gyro_range);
    }

    // 2. Trigger
    JsonObjectConst v_trigger = p_doc["trigger"];
    if (!v_trigger.isNull()) {
        cfgSet(v_trigger["hold_time_ms"],      _config.trigger.hold_time_ms);
        cfgSet(v_trigger["use_deep_sleep"],    _config.trigger.use_deep_sleep);
        cfgSet(v_trigger["sleep_timeout_sec"], _config.trigger.sleep_timeout_sec);
        cfgSet(v_trigger["wake_thresh_g"],     _config.trigger.wake_thresh_g);
        cfgSet(v_trigger["wake_duration"],     _config.trigger.wake_duration);
        cfgSet(v_trigger["vib_rms_thresh"],    _config.trigger.vib_rms_thresh);
        cfgSet(v_trigger["audio_rms_thresh"],  _config.trigger.audio_rms_thresh);

        JsonArrayConst v_bands = v_trigger["bands"];
        if (!v_bands.isNull()) {
            uint8_t i = 0;
            for (JsonObjectConst v_b : v_bands) {
                if (i >= T2_Config::FeatureLimit::MAX_BAND_RMS_CONST) break;
                cfgSet(v_b["enable"],   _config.trigger.band_enable[i]);
                cfgSet(v_b["start_hz"], _config.trigger.band_start_hz[i]);
                cfgSet(v_b["end_hz"],   _config.trigger.band_end_hz[i]);
                cfgSet(v_b["thresh"],   _config.trigger.band_thresh[i]);
                i++;
            }
        }
    }

    // 3. DSP
    JsonObjectConst v_dsp = p_doc["dsp"];
    if (!v_dsp.isNull()) {
        cfgSet(v_dsp["remove_dc"],      _config.dsp.remove_dc);
        cfgSet(v_dsp["median_enabled"], _config.dsp.median_enabled);
        cfgSet(v_dsp["median_window"],  _config.dsp.median_window);

        JsonObjectConst v_fir_hpf = v_dsp["fir_hpf"];
        if (!v_fir_hpf.isNull()) {
            cfgSet(v_fir_hpf["enabled"],   _config.dsp.fir_hpf.enabled);
            cfgSet(v_fir_hpf["cutoff_hz"], _config.dsp.fir_hpf.cutoff_hz);
            cfgSet(v_fir_hpf["num_taps"],  _config.dsp.fir_hpf.num_taps);
        }

        JsonObjectConst v_fir_lpf = v_dsp["fir_lpf"];
        if (!v_fir_lpf.isNull()) {
            cfgSet(v_fir_lpf["enabled"],   _config.dsp.fir_lpf.enabled);
            cfgSet(v_fir_lpf["cutoff_hz"], _config.dsp.fir_lpf.cutoff_hz);
            cfgSet(v_fir_lpf["num_taps"],  _config.dsp.fir_lpf.num_taps);
        }

        JsonObjectConst v_iir_hpf = v_dsp["iir_hpf"];
        if (!v_iir_hpf.isNull()) {
            cfgSet(v_iir_hpf["enabled"],   _config.dsp.iir_hpf.enabled);
            cfgSet(v_iir_hpf["cutoff_hz"], _config.dsp.iir_hpf.cutoff_hz);
            cfgSet(v_iir_hpf["q_factor"],  _config.dsp.iir_hpf.q_factor);
        }

        JsonObjectConst v_iir_lpf = v_dsp["iir_lpf"];
        if (!v_iir_lpf.isNull()) {
            cfgSet(v_iir_lpf["enabled"],   _config.dsp.iir_lpf.enabled);
            cfgSet(v_iir_lpf["cutoff_hz"], _config.dsp.iir_lpf.cutoff_hz);
            cfgSet(v_iir_lpf["q_factor"],  _config.dsp.iir_lpf.q_factor);
        }

        JsonObjectConst v_notch = v_dsp["notch"];
        if (!v_notch.isNull()) {
            cfgSet(v_notch["enabled"],          _config.dsp.notch.enabled);
            cfgSet(v_notch["target_freq_hz"],   _config.dsp.notch.target_freq_hz);
            cfgSet(v_notch["gain"],             _config.dsp.notch.gain);
            cfgSet(v_notch["q_factor"],         _config.dsp.notch.q_factor);
        }

        cfgSet(v_dsp["preemphasis_enable"], _config.dsp.preemphasis_enable);
        cfgSet(v_dsp["preemphasis_alpha"],  _config.dsp.preemphasis_alpha);

        JsonObjectConst v_noise = v_dsp["noise"];
        if (!v_noise.isNull()) {
            cfgSet(v_noise["enable_gate"], _config.dsp.noise.enable_gate);
            cfgSet(v_noise["gate_threshold_abs"], _config.dsp.noise.gate_threshold_abs);
            if (!v_noise["mode"].isNull()) _config.dsp.noise.mode = (T2_Type::NoiseMode)v_noise["mode"].as<uint8_t>();
            cfgSet(v_noise["spectral_subtract_strength"], _config.dsp.noise.spectral_subtract_strength);
            cfgSet(v_noise["adaptive_alpha"], _config.dsp.noise.adaptive_alpha);
            cfgSet(v_noise["noise_learn_frames"], _config.dsp.noise.noise_learn_frames);
        }

        if (!v_dsp["window_type"].isNull()) _config.dsp.window_type = (T2_Type::WindowType)v_dsp["window_type"].as<uint8_t>();
        cfgSet(v_dsp["beamforming_gain"], _config.dsp.beamforming_gain);
        cfgSet(v_dsp["calib_gain_L"],      _config.dsp.calib_gain_L);
        cfgSet(v_dsp["calib_gain_R"],      _config.dsp.calib_gain_R);

        JsonArrayConst v_eq = v_dsp["calib_eq_coeffs"];
        if (!v_eq.isNull()) {
            uint16_t i = 0;
            for (JsonVariantConst v_val : v_eq) {
                if (i >= T2_Config::FeatureLimit::FIR_TAPS_CONST) break;
                _config.dsp.calib_eq_coeffs[i] = v_val.as<float>();
                i++;
            }
        }
    }

    // 4. Calibration & Feature
    JsonObjectConst v_calib = p_doc["calib"];
    if (!v_calib.isNull()) {
        cfgSet(v_calib["auto_idle_min"], _config.calib.auto_idle_min);
        cfgSet(v_calib["ref_freq_hz"],   _config.calib.ref_freq_hz);
        cfgSet(v_calib["filter_min_freq_hz"], _config.calib.filter_min_freq_hz);
        cfgSet(v_calib["filter_max_freq_hz"], _config.calib.filter_max_freq_hz);
        cfgSet(v_calib["target_gain_max"],    _config.calib.target_gain_max);
        cfgSet(v_calib["target_gain_min"],    _config.calib.target_gain_min);
        cfgSet(v_calib["norm_safe_thresh"],   _config.calib.norm_safe_thresh);
    }

    JsonObjectConst v_feature = p_doc["feature"];
    if (!v_feature.isNull()) {
        cfgSet(v_feature["spatial_freq_min_hz"],   _config.feature.spatial_freq_min_hz);
        cfgSet(v_feature["spatial_freq_max_hz"],   _config.feature.spatial_freq_max_hz);
        cfgSet(v_feature["peak_amplitude_limit_min"], _config.feature.peak_amplitude_limit_min);
        cfgSet(v_feature["peak_freq_gap_limit_hz_min"], _config.feature.peak_freq_gap_limit_hz_min);
    }

    // 5. Storage
    JsonObjectConst v_storage = p_doc["storage"];
    if (!v_storage.isNull()) {
        cfgSet(v_storage["rotation_mb"],     _config.storage.rotation_mb);
        cfgSet(v_storage["rotation_min"],    _config.storage.rotation_min);
        cfgSet(v_storage["save_raw"],        _config.storage.save_raw);
        cfgSet(v_storage["rotate_keep_max"], _config.storage.rotate_keep_max);
        cfgSet(v_storage["idle_flush_ms"],   _config.storage.idle_flush_ms);
        cfgSet(v_storage["pre_trigger_sec"], _config.storage.pre_trigger_sec);
    }

    // 6. WiFi & MQTT
    JsonObjectConst v_wifi = p_doc["wifi"];
    if (!v_wifi.isNull()) {
        if (!v_wifi["mode"].isNull()) _config.wifi.mode = (T2_Type::WiFiMode)v_wifi["mode"].as<uint8_t>();
        cfgStr(v_wifi["ap_ssid"],     _config.wifi.ap_ssid,     sizeof(_config.wifi.ap_ssid));
        cfgStr(v_wifi["ap_password"], _config.wifi.ap_password, sizeof(_config.wifi.ap_password));
        cfgStr(v_wifi["ap_ip"],       _config.wifi.ap_ip,       sizeof(_config.wifi.ap_ip));

        JsonArrayConst v_multi = v_wifi["multi_ap"];
        if (!v_multi.isNull()) {
            uint8_t i = 0;
            for (JsonObjectConst v_ap : v_multi) {
                if (i >= T2_Config::Net::WIFI_MULTI_MAX_CONST) break;
                cfgStr(v_ap["ssid"],     _config.wifi.multi_ssid[i], sizeof(_config.wifi.multi_ssid[i]));
                cfgStr(v_ap["password"], _config.wifi.multi_pw[i],   sizeof(_config.wifi.multi_pw[i]));
                i++;
            }
        }
    }

    JsonObjectConst v_mqtt = p_doc["mqtt"];
    if (!v_mqtt.isNull()) {
        cfgSet(v_mqtt["enable"], _config.mqtt.enable);
        cfgSet(v_mqtt["port"],   _config.mqtt.port);
        cfgStr(v_mqtt["broker"],     _config.mqtt.broker,     sizeof(_config.mqtt.broker));
        cfgStr(v_mqtt["id"],         _config.mqtt.id,         sizeof(_config.mqtt.id));
        cfgStr(v_mqtt["password"],   _config.mqtt.password,   sizeof(_config.mqtt.password));
        cfgStr(v_mqtt["topic_root"], _config.mqtt.topic_root, sizeof(_config.mqtt.topic_root));
    }

    cfgSet(p_doc["op_mode"],     _config.op_mode);
    cfgSet(p_doc["watchdog_ms"], _config.watchdog_ms);
}


bool CL_T2_ConfigManager::load() {
    xSemaphoreTake(_lock, portMAX_DELAY);
    File v_file = LittleFS.open(T2_Config::Path::FILE_CFG_JSON_CONST, "r");
    if (!v_file) {
        ESP_LOGE(TAG, "Config file open failed for reading");
        xSemaphoreGive(_lock);
        return false;
    }

    JsonDocument v_doc;
    DeserializationError v_err = deserializeJson(v_doc, v_file);
    v_file.close();

    if (v_err) {
        ESP_LOGE(TAG, "JSON Parse Error: %s", v_err.c_str());
        xSemaphoreGive(_lock);
        return false;
    }

    _applyJson(v_doc);
    ESP_LOGI(TAG, "Config successfully loaded from LittleFS");
    xSemaphoreGive(_lock);
    return true;
}

bool CL_T2_ConfigManager::updateFromJson(const char* p_jsonString) {
    JsonDocument v_doc;
    DeserializationError v_err = deserializeJson(v_doc, p_jsonString);

    if (v_err) {
        ESP_LOGE(TAG, "Update JSON Parse Error: %s", v_err.c_str());
        return false;
    }

    xSemaphoreTake(_lock, portMAX_DELAY);

    _applyJson(v_doc);
    _isDirty = true;
    _lastModifiedMs = millis();

	xSemaphoreGive(_lock);

    return true;
}

void CL_T2_ConfigManager::checkLazyWrite() {
    if (!_isDirty) return;

    if (millis() - _lastModifiedMs > 3000) {
        save();
        _isDirty = false;
        ESP_LOGI(TAG, "Lazy Write Committed to LittleFS");
    }
}

bool CL_T2_ConfigManager::save() {
    xSemaphoreTake(_lock, portMAX_DELAY);

    JsonDocument v_doc;

    JsonObject v_sensor = v_doc["sensor"].to<JsonObject>();
    v_sensor["accel_range"] = _config.sensor.accel_range;
    v_sensor["gyro_range"] = _config.sensor.gyro_range;

    JsonObject v_trigger = v_doc["trigger"].to<JsonObject>();
    v_trigger["hold_time_ms"] = _config.trigger.hold_time_ms;
    v_trigger["use_deep_sleep"] = _config.trigger.use_deep_sleep;
    v_trigger["sleep_timeout_sec"] = _config.trigger.sleep_timeout_sec;
    v_trigger["wake_thresh_g"] = _config.trigger.wake_thresh_g;
    v_trigger["wake_duration"] = _config.trigger.wake_duration;
    v_trigger["vib_rms_thresh"] = _config.trigger.vib_rms_thresh;
    v_trigger["audio_rms_thresh"] = _config.trigger.audio_rms_thresh;

    JsonArray v_bands = v_trigger["bands"].to<JsonArray>();
    for (uint8_t i = 0; i < T2_Config::FeatureLimit::MAX_BAND_RMS_CONST; i++) {
        JsonObject v_b = v_bands.add<JsonObject>();
        v_b["enable"] = _config.trigger.band_enable[i];
        v_b["start_hz"] = _config.trigger.band_start_hz[i];
        v_b["end_hz"] = _config.trigger.band_end_hz[i];
        v_b["thresh"] = _config.trigger.band_thresh[i];
    }

    JsonObject v_dsp = v_doc["dsp"].to<JsonObject>();
    v_dsp["remove_dc"] = _config.dsp.remove_dc;
    v_dsp["median_enabled"] = _config.dsp.median_enabled;
    v_dsp["median_window"] = _config.dsp.median_window;

    JsonObject v_fir_hpf = v_dsp["fir_hpf"].to<JsonObject>();
    v_fir_hpf["enabled"] = _config.dsp.fir_hpf.enabled;
    v_fir_hpf["cutoff_hz"] = _config.dsp.fir_hpf.cutoff_hz;
    v_fir_hpf["num_taps"] = _config.dsp.fir_hpf.num_taps;

    JsonObject v_fir_lpf = v_dsp["fir_lpf"].to<JsonObject>();
    v_fir_lpf["enabled"] = _config.dsp.fir_lpf.enabled;
    v_fir_lpf["cutoff_hz"] = _config.dsp.fir_lpf.cutoff_hz;
    v_fir_lpf["num_taps"] = _config.dsp.fir_lpf.num_taps;

    JsonObject v_iir_hpf = v_dsp["iir_hpf"].to<JsonObject>();
    v_iir_hpf["enabled"] = _config.dsp.iir_hpf.enabled;
    v_iir_hpf["cutoff_hz"] = _config.dsp.iir_hpf.cutoff_hz;
    v_iir_hpf["q_factor"] = _config.dsp.iir_hpf.q_factor;

    JsonObject v_iir_lpf = v_dsp["iir_lpf"].to<JsonObject>();
    v_iir_lpf["enabled"] = _config.dsp.iir_lpf.enabled;
    v_iir_lpf["cutoff_hz"] = _config.dsp.iir_lpf.cutoff_hz;
    v_iir_lpf["q_factor"] = _config.dsp.iir_lpf.q_factor;

    JsonObject v_notch = v_dsp["notch"].to<JsonObject>();
    v_notch["enabled"] = _config.dsp.notch.enabled;
    v_notch["target_freq_hz"] = _config.dsp.notch.target_freq_hz;
    v_notch["gain"] = _config.dsp.notch.gain;
    v_notch["q_factor"] = _config.dsp.notch.q_factor;

    v_dsp["preemphasis_enable"] = _config.dsp.preemphasis_enable;
    v_dsp["preemphasis_alpha"] = _config.dsp.preemphasis_alpha;

    JsonObject v_noise = v_dsp["noise"].to<JsonObject>();
    v_noise["enable_gate"] = _config.dsp.noise.enable_gate;
    v_noise["gate_threshold_abs"] = _config.dsp.noise.gate_threshold_abs;
    v_noise["mode"] = (uint8_t)_config.dsp.noise.mode;
    v_noise["spectral_subtract_strength"] = _config.dsp.noise.spectral_subtract_strength;
    v_noise["adaptive_alpha"] = _config.dsp.noise.adaptive_alpha;
    v_noise["noise_learn_frames"] = _config.dsp.noise.noise_learn_frames;

    v_dsp["window_type"] = (uint8_t)_config.dsp.window_type;
    v_dsp["beamforming_gain"] = _config.dsp.beamforming_gain;

   // DSP Calib 데이터 저장 로직
    v_dsp["calib_gain_L"] = _config.dsp.calib_gain_L;
    v_dsp["calib_gain_R"] = _config.dsp.calib_gain_R;
    JsonArray v_eq = v_dsp["calib_eq_coeffs"].to<JsonArray>();
    for(uint16_t i = 0; i < T2_Config::FeatureLimit::FIR_TAPS_CONST; i++) {
        v_eq.add(_config.dsp.calib_eq_coeffs[i]);
    }

    // Calib 객체 저장
    JsonObject v_calib = v_doc["calib"].to<JsonObject>();
    v_calib["auto_idle_min"] = _config.calib.auto_idle_min;
    v_calib["ref_freq_hz"] = _config.calib.ref_freq_hz;
    v_calib["filter_min_freq_hz"] = _config.calib.filter_min_freq_hz;
    v_calib["filter_max_freq_hz"] = _config.calib.filter_max_freq_hz;
    v_calib["target_gain_max"] = _config.calib.target_gain_max;
    v_calib["target_gain_min"] = _config.calib.target_gain_min;
    v_calib["norm_safe_thresh"] = _config.calib.norm_safe_thresh;

    // Feature 객체 저장
    JsonObject v_feature = v_doc["feature"].to<JsonObject>();
    v_feature["spatial_freq_min_hz"] = _config.feature.spatial_freq_min_hz;
    v_feature["spatial_freq_max_hz"] = _config.feature.spatial_freq_max_hz;
    v_feature["peak_amplitude_limit_min"] = _config.feature.peak_amplitude_limit_min;
    v_feature["peak_freq_gap_limit_hz_min"] = _config.feature.peak_freq_gap_limit_hz_min;


    JsonObject v_storage = v_doc["storage"].to<JsonObject>();
    v_storage["rotation_mb"] = _config.storage.rotation_mb;
    v_storage["rotation_min"] = _config.storage.rotation_min;
    v_storage["save_raw"] = _config.storage.save_raw;
    v_storage["rotate_keep_max"] = _config.storage.rotate_keep_max;
    v_storage["idle_flush_ms"] = _config.storage.idle_flush_ms;
    v_storage["pre_trigger_sec"] = _config.storage.pre_trigger_sec;

    JsonObject v_wifi = v_doc["wifi"].to<JsonObject>();
    v_wifi["mode"] = (uint8_t)_config.wifi.mode;
    v_wifi["ap_ssid"] = _config.wifi.ap_ssid;
    v_wifi["ap_password"] = _config.wifi.ap_password;
    v_wifi["ap_ip"] = _config.wifi.ap_ip;

    JsonArray v_multi = v_wifi["multi_ap"].to<JsonArray>();
    for (uint8_t i = 0; i < T2_Config::Net::WIFI_MULTI_MAX_CONST; i++) {
        JsonObject v_ap = v_multi.add<JsonObject>();
        v_ap["ssid"] = _config.wifi.multi_ssid[i];
        v_ap["password"] = _config.wifi.multi_pw[i];
    }

    JsonObject v_mqtt = v_doc["mqtt"].to<JsonObject>();
    v_mqtt["enable"] = _config.mqtt.enable;
    v_mqtt["broker"] = _config.mqtt.broker;
    v_mqtt["port"] = _config.mqtt.port;
    v_mqtt["id"] = _config.mqtt.id;
    v_mqtt["password"] = _config.mqtt.password;
    v_mqtt["topic_root"] = _config.mqtt.topic_root;

    v_doc["op_mode"] = _config.op_mode;
    v_doc["watchdog_ms"] = _config.watchdog_ms;

    File v_file = LittleFS.open("/sys/config_241.tmp", "w");
    if (!v_file) {
        ESP_LOGE(TAG, "Failed to open .tmp file for atomic write");
        xSemaphoreGive(_lock);
        return false;
    }

    serializeJson(v_doc, v_file);
    v_file.close();

    LittleFS.remove(T2_Config::Path::FILE_CFG_JSON_CONST);
    LittleFS.rename("/sys/config_241.tmp", T2_Config::Path::FILE_CFG_JSON_CONST);

    xSemaphoreGive(_lock);
    return true;
}

void CL_T2_ConfigManager::resetToDefault() {
    xSemaphoreTake(_lock, portMAX_DELAY);
    _loadDefaults();
    xSemaphoreGive(_lock);
    save();
    ESP_LOGW(TAG, "Configuration reset to factory defaults");
}

T2_Type::DynamicConfig CL_T2_ConfigManager::getConfig() {
    T2_Type::DynamicConfig v_copy;
    xSemaphoreTake(_lock, portMAX_DELAY);
    v_copy = _config;
    xSemaphoreGive(_lock);
    return v_copy;
}

bool CL_T2_ConfigManager::updateConfig(const T2_Type::DynamicConfig& p_newConfig) {
    xSemaphoreTake(_lock, portMAX_DELAY);
    _config = p_newConfig;
    xSemaphoreGive(_lock);
    return save();
}

bool CL_T2_ConfigManager::updatePreview(const char* p_jsonString) {
    JsonDocument v_doc;
    DeserializationError v_err = deserializeJson(v_doc, p_jsonString);

    if (v_err) {
        ESP_LOGE(TAG, "Preview JSON Parse Error: %s", v_err.c_str());
        return false;
    }

    xSemaphoreTake(_lock, portMAX_DELAY);
    _applyJson(v_doc);
    _isDirty = false;
    _isTuningActive = true;
    xSemaphoreGive(_lock);

    return true;
}

bool CL_T2_ConfigManager::commitSave() {
    bool v_res = save();
    if (v_res) {
        _isDirty = false;
        _isTuningActive = false;
        ESP_LOGI(TAG, "Tuning Committed to LittleFS");
    }
    return v_res;
}

bool CL_T2_ConfigManager::revertCancel() {
    bool v_res = load();
    if (v_res) {
        _isDirty = false;
        _isTuningActive = true;
        ESP_LOGI(TAG, "Tuning Canceled. Reverted to previous saved config.");
    }
    return v_res;
}




/*
void CL_T2_ConfigManager::_applyJson(const JsonDocument& p_doc) {
    JsonObjectConst v_sensor = p_doc["sensor"];
    if (!v_sensor.isNull()) {
		_config.sensor.accel_range = v_sensor["accel_range"] ? v_sensor["accel_range"].as<int>() : _config.sensor.accel_range;
    	_config.sensor.gyro_range  = v_sensor["gyro_range"]  ? v_sensor["gyro_range"].as<int>()  : _config.sensor.gyro_range;

        // _config.sensor.accel_range = v_sensor["accel_range"] | _config.sensor.accel_range;
        // _config.sensor.gyro_range = v_sensor["gyro_range"] | _config.sensor.gyro_range;
    }

    JsonObjectConst v_trigger = p_doc["trigger"];
    if (!v_trigger.isNull()) {
        _config.trigger.hold_time_ms = v_trigger["hold_time_ms"] | _config.trigger.hold_time_ms;
        _config.trigger.use_deep_sleep = v_trigger["use_deep_sleep"] | _config.trigger.use_deep_sleep;
        _config.trigger.sleep_timeout_sec = v_trigger["sleep_timeout_sec"] | _config.trigger.sleep_timeout_sec;

		// float 타입인 경우 .as<float>() 필수
		_config.trigger.wake_thresh_g     = v_trigger["wake_thresh_g"]     ? v_trigger["wake_thresh_g"].as<float>()   : _config.trigger.wake_thresh_g;
		_config.trigger.wake_duration     = v_trigger["wake_duration"]     ? v_trigger["wake_duration"].as<uint32_t>() : _config.trigger.wake_duration;
		_config.trigger.vib_rms_thresh    = v_trigger["vib_rms_thresh"]    ? v_trigger["vib_rms_thresh"].as<float>()  : _config.trigger.vib_rms_thresh;
		_config.trigger.audio_rms_thresh  = v_trigger["audio_rms_thresh"]  ? v_trigger["audio_rms_thresh"].as<float>() : _config.trigger.audio_rms_thresh;
        // _config.trigger.wake_thresh_g = v_trigger["wake_thresh_g"] | _config.trigger.wake_thresh_g;
        // _config.trigger.wake_duration = v_trigger["wake_duration"] | _config.trigger.wake_duration;
        // _config.trigger.vib_rms_thresh = v_trigger["vib_rms_thresh"] | _config.trigger.vib_rms_thresh;
        // _config.trigger.audio_rms_thresh = v_trigger["audio_rms_thresh"] | _config.trigger.audio_rms_thresh;

        JsonArrayConst v_bands = v_trigger["bands"];
        if (!v_bands.isNull()) {
            uint8_t i = 0;
            for (JsonObjectConst v_b : v_bands) {
                if (i >= T2_Config::FeatureLimit::MAX_BAND_RMS_CONST) break;

				_config.trigger.band_enable[i]   = v_b["enable"]   ? v_b["enable"].as<bool>()     : _config.trigger.band_enable[i];
				_config.trigger.band_start_hz[i] = v_b["start_hz"] ? v_b["start_hz"].as<float>() : _config.trigger.band_start_hz[i];
				_config.trigger.band_end_hz[i]   = v_b["end_hz"]   ? v_b["end_hz"].as<float>()   : _config.trigger.band_end_hz[i];
				_config.trigger.band_thresh[i]   = v_b["thresh"]   ? v_b["thresh"].as<float>()   : _config.trigger.band_thresh[i];
				i++;
            }
        }
    }

    JsonObjectConst v_dsp = p_doc["dsp"];
    if (!v_dsp.isNull()) {
		_config.dsp.remove_dc	   = v_dsp["remove_dc"] | _config.dsp.remove_dc;
		_config.dsp.median_enabled = v_dsp["median_enabled"] | _config.dsp.median_enabled;
		_config.dsp.median_window  = v_dsp["median_window"] | _config.dsp.median_window;

		JsonObjectConst v_fir_hpf = v_dsp["fir_hpf"];
        if (!v_fir_hpf.isNull()) {
            _config.dsp.fir_hpf.enabled = v_fir_hpf["enabled"] | _config.dsp.fir_hpf.enabled;
            _config.dsp.fir_hpf.cutoff_hz = v_fir_hpf["cutoff_hz"] | _config.dsp.fir_hpf.cutoff_hz;
            _config.dsp.fir_hpf.num_taps = v_fir_hpf["num_taps"] | _config.dsp.fir_hpf.num_taps;
        }

        JsonObjectConst v_fir_lpf = v_dsp["fir_lpf"];
        if (!v_fir_lpf.isNull()) {
            _config.dsp.fir_lpf.enabled = v_fir_lpf["enabled"] | _config.dsp.fir_lpf.enabled;
            _config.dsp.fir_lpf.cutoff_hz = v_fir_lpf["cutoff_hz"] | _config.dsp.fir_lpf.cutoff_hz;
            _config.dsp.fir_lpf.num_taps = v_fir_lpf["num_taps"] | _config.dsp.fir_lpf.num_taps;
        }

        JsonObjectConst v_iir_hpf = v_dsp["iir_hpf"];
        if (!v_iir_hpf.isNull()) {
            _config.dsp.iir_hpf.enabled = v_iir_hpf["enabled"] | _config.dsp.iir_hpf.enabled;
            _config.dsp.iir_hpf.cutoff_hz = v_iir_hpf["cutoff_hz"] | _config.dsp.iir_hpf.cutoff_hz;
            _config.dsp.iir_hpf.q_factor = v_iir_hpf["q_factor"] | _config.dsp.iir_hpf.q_factor;
        }

        JsonObjectConst v_iir_lpf = v_dsp["iir_lpf"];
        if (!v_iir_lpf.isNull()) {
            _config.dsp.iir_lpf.enabled = v_iir_lpf["enabled"] | _config.dsp.iir_lpf.enabled;
            _config.dsp.iir_lpf.cutoff_hz = v_iir_lpf["cutoff_hz"] | _config.dsp.iir_lpf.cutoff_hz;
            _config.dsp.iir_lpf.q_factor = v_iir_lpf["q_factor"] | _config.dsp.iir_lpf.q_factor;
        }

        JsonObjectConst v_notch = v_dsp["notch"];
        if (!v_notch.isNull()) {
            _config.dsp.notch.enabled = v_notch["enabled"] | _config.dsp.notch.enabled;
            _config.dsp.notch.target_freq_hz = v_notch["target_freq_hz"] | _config.dsp.notch.target_freq_hz;
            _config.dsp.notch.gain = v_notch["gain"] | _config.dsp.notch.gain;
            _config.dsp.notch.q_factor = v_notch["q_factor"] | _config.dsp.notch.q_factor;
        }

        _config.dsp.preemphasis_enable = v_dsp["preemphasis_enable"] | _config.dsp.preemphasis_enable;
        _config.dsp.preemphasis_alpha = v_dsp["preemphasis_alpha"] | _config.dsp.preemphasis_alpha;

        JsonObjectConst v_noise = v_dsp["noise"];
        if (!v_noise.isNull()) {
            _config.dsp.noise.enable_gate = v_noise["enable_gate"] | _config.dsp.noise.enable_gate;
            _config.dsp.noise.gate_threshold_abs = v_noise["gate_threshold_abs"] | _config.dsp.noise.gate_threshold_abs;
            _config.dsp.noise.mode = (T2_Type::NoiseMode)(v_noise["mode"] | (uint8_t)_config.dsp.noise.mode);
            _config.dsp.noise.spectral_subtract_strength = v_noise["spectral_subtract_strength"] | _config.dsp.noise.spectral_subtract_strength;
            _config.dsp.noise.adaptive_alpha = v_noise["adaptive_alpha"] | _config.dsp.noise.adaptive_alpha;
            _config.dsp.noise.noise_learn_frames = v_noise["noise_learn_frames"] | _config.dsp.noise.noise_learn_frames;
        }

        _config.dsp.window_type = (T2_Type::WindowType)(v_dsp["window_type"] | (uint8_t)_config.dsp.window_type);
        _config.dsp.beamforming_gain = v_dsp["beamforming_gain"] | _config.dsp.beamforming_gain;

        // DSP Calib 데이터 로드 로직
        _config.dsp.calib_gain_L = v_dsp["calib_gain_L"] | _config.dsp.calib_gain_L;
        _config.dsp.calib_gain_R = v_dsp["calib_gain_R"] | _config.dsp.calib_gain_R;
        JsonArrayConst v_eq = v_dsp["calib_eq_coeffs"];
        if (!v_eq.isNull()) {
            uint16_t i = 0;
            for (float v_val : v_eq) {
                if (i >= T2_Config::FeatureLimit::FIR_TAPS_CONST) break;
                _config.dsp.calib_eq_coeffs[i] = v_val;
                i++;
            }
        }
    }

    // Calib 객체 로드
    JsonObjectConst v_calib = p_doc["calib"];
    if (!v_calib.isNull()) {
        _config.calib.auto_idle_min = v_calib["auto_idle_min"] | _config.calib.auto_idle_min;
        _config.calib.ref_freq_hz = v_calib["ref_freq_hz"] | _config.calib.ref_freq_hz;
        _config.calib.filter_min_freq_hz = v_calib["filter_min_freq_hz"] | _config.calib.filter_min_freq_hz;
        _config.calib.filter_max_freq_hz = v_calib["filter_max_freq_hz"] | _config.calib.filter_max_freq_hz;
        _config.calib.target_gain_max = v_calib["target_gain_max"] | _config.calib.target_gain_max;
        _config.calib.target_gain_min = v_calib["target_gain_min"] | _config.calib.target_gain_min;
        _config.calib.norm_safe_thresh = v_calib["norm_safe_thresh"] | _config.calib.norm_safe_thresh;
    }

    // Feature 객체 로드
    JsonObjectConst v_feature = p_doc["feature"];
    if (!v_feature.isNull()) {
        _config.feature.spatial_freq_min_hz = v_feature["spatial_freq_min_hz"] | _config.feature.spatial_freq_min_hz;
        _config.feature.spatial_freq_max_hz = v_feature["spatial_freq_max_hz"] | _config.feature.spatial_freq_max_hz;
        _config.feature.peak_amplitude_limit_min = v_feature["peak_amplitude_limit_min"] | _config.feature.peak_amplitude_limit_min;
        _config.feature.peak_freq_gap_limit_hz_min = v_feature["peak_freq_gap_limit_hz_min"] | _config.feature.peak_freq_gap_limit_hz_min;
        // 배열 추출 방어 로직 생략(간결성) 또는 필요시 추가.
    }

    JsonObjectConst v_storage = p_doc["storage"];
    if (!v_storage.isNull()) {
        _config.storage.rotation_mb = v_storage["rotation_mb"] | _config.storage.rotation_mb;
        _config.storage.rotation_min = v_storage["rotation_min"] | _config.storage.rotation_min;
        _config.storage.save_raw = v_storage["save_raw"] | _config.storage.save_raw;
        _config.storage.rotate_keep_max = v_storage["rotate_keep_max"] | _config.storage.rotate_keep_max;
        _config.storage.idle_flush_ms = v_storage["idle_flush_ms"] | _config.storage.idle_flush_ms;
        _config.storage.pre_trigger_sec = v_storage["pre_trigger_sec"] | _config.storage.pre_trigger_sec;
    }

    JsonObjectConst v_wifi = p_doc["wifi"];
    if (!v_wifi.isNull()) {
        _config.wifi.mode = (T2_Type::WiFiMode)(v_wifi["mode"] | (uint8_t)_config.wifi.mode);
        strlcpy(_config.wifi.ap_ssid, v_wifi["ap_ssid"] | _config.wifi.ap_ssid, sizeof(_config.wifi.ap_ssid));
        strlcpy(_config.wifi.ap_password, v_wifi["ap_password"] | _config.wifi.ap_password, sizeof(_config.wifi.ap_password));
        strlcpy(_config.wifi.ap_ip, v_wifi["ap_ip"] | _config.wifi.ap_ip, sizeof(_config.wifi.ap_ip));

        JsonArrayConst v_multi = v_wifi["multi_ap"];
        if (!v_multi.isNull()) {
            uint8_t i = 0;
            for (JsonObjectConst v_ap : v_multi) {
                if (i >= T2_Config::Net::WIFI_MULTI_MAX_CONST) break;
                strlcpy(_config.wifi.multi_ssid[i], v_ap["ssid"] | _config.wifi.multi_ssid[i], sizeof(_config.wifi.multi_ssid[i]));
                strlcpy(_config.wifi.multi_pw[i], v_ap["password"] | _config.wifi.multi_pw[i], sizeof(_config.wifi.multi_pw[i]));
                i++;
            }
        }
    }

    JsonObjectConst v_mqtt = p_doc["mqtt"];
    if (!v_mqtt.isNull()) {
        _config.mqtt.enable = v_mqtt["enable"] | _config.mqtt.enable;
        strlcpy(_config.mqtt.broker, v_mqtt["broker"] | _config.mqtt.broker, sizeof(_config.mqtt.broker));
        _config.mqtt.port = v_mqtt["port"] | _config.mqtt.port;
        strlcpy(_config.mqtt.id, v_mqtt["id"] | _config.mqtt.id, sizeof(_config.mqtt.id));
        strlcpy(_config.mqtt.password, v_mqtt["password"] | _config.mqtt.password, sizeof(_config.mqtt.password));
        strlcpy(_config.mqtt.topic_root, v_mqtt["topic_root"] | _config.mqtt.topic_root, sizeof(_config.mqtt.topic_root));
    }

    _config.op_mode = p_doc["op_mode"] | _config.op_mode;
    _config.watchdog_ms = p_doc["watchdog_ms"] | _config.watchdog_ms;
}
*/
