/* 
============================================================================
 * File: T415_ConfigMgr_013.cpp
 * Summary: Dynamic JSON Configuration Manager Implementation
 * * [AI 메모: 마이그레이션 적용 사항]
 * 1. Web API의 Partial Update 로직 처리를 위한 _applyJson 모듈화 완료.
 * 2. [Const-Correctness 방어]: ArduinoJson V7 환경 const 객체 추출 오류 방어.
 * 3. [v013 Phase 1 적용]: updatePreview, commitSave, revertCancel 구현을 통해 
 * 플래시 마모 없는 실시간 차트 피드백 튜닝 기반을 완벽 구축. (Lock-free 감지)
 * ========================================================================== 
 */
#include "T415_ConfigMgr_013.hpp"
#include "esp_log.h"

static const char* TAG = "T415_CFG";


T415_ConfigManager::T415_ConfigManager() {
    _lock = xSemaphoreCreateMutex();
    _isLoaded = false;
    _isDirty = false;
    _isTuningActive = false; // [v013]
    _lastModifiedMs = 0;
    _loadDefaults();
}

T415_ConfigManager::~T415_ConfigManager() {
    if (_lock) vSemaphoreDelete(_lock);
}



bool T415_ConfigManager::init() {
    if (!LittleFS.begin(true)) {
        ESP_LOGE(TAG, "LittleFS Mount Failed!");
        return false;
    }

    // [항목 15 반영] 정전(Power Loss)으로 인한 Atomic Write 실패 복구 로직
    if (!LittleFS.exists(SmeaConfig::Path::SYS_CFG_JSON_DEF)) {
        if (LittleFS.exists("/sys/config.tmp")) {
            ESP_LOGW(TAG, "Recovering config from interrupted atomic write (.tmp -> .json)");
            LittleFS.rename("/sys/config.tmp", SmeaConfig::Path::SYS_CFG_JSON_DEF);
            load();
        } else {
            ESP_LOGW(TAG, "Config not found. Creating default...");
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

void T415_ConfigManager::_loadDefaults() {
    _config.dsp.window_ms = SmeaConfig::Dsp::WINDOW_MS_DEF;
    _config.dsp.hop_ms = SmeaConfig::Dsp::HOP_MS_DEF;
    _config.dsp.notch_freq_hz = SmeaConfig::Dsp::NOTCH_FREQ_HZ_DEF;
    _config.dsp.notch_freq_2_hz = SmeaConfig::Dsp::NOTCH_FREQ_2_HZ_DEF;
    _config.dsp.notch_q_factor = SmeaConfig::Dsp::NOTCH_Q_FACTOR_DEF;
    _config.dsp.pre_emphasis_alpha = SmeaConfig::Dsp::PRE_EMPHASIS_ALPHA_DEF;
    _config.dsp.beamforming_gain = SmeaConfig::Dsp::BEAMFORMING_GAIN_DEF;
    _config.dsp.fir_lpf_cutoff = SmeaConfig::Dsp::FIR_LPF_CUTOFF_DEF;
    _config.dsp.fir_hpf_cutoff = SmeaConfig::Dsp::FIR_HPF_CUTOFF_DEF;
    _config.dsp.median_window = SmeaConfig::Dsp::MEDIAN_WINDOW_DEF;
    _config.dsp.noise_gate_thresh = SmeaConfig::Dsp::NOISE_GATE_THRESH_DEF;
    _config.dsp.noise_learn_frames = SmeaConfig::Dsp::NOISE_LEARN_FRAMES_DEF;
    _config.dsp.spectral_sub_gain = SmeaConfig::Dsp::SPECTRAL_SUB_GAIN_DEF;
    
    _config.dsp.calib_gain_L = 1.0f;
    _config.dsp.calib_gain_R = 1.0f;
    // FIR EQ 필터는 기본적으로 Bypass(가운데 탭만 1.0)로 초기화
    memset(_config.dsp.calib_eq_coeffs, 0, sizeof(_config.dsp.calib_eq_coeffs));
    _config.dsp.calib_eq_coeffs[SmeaConfig::CalibLimit::EQ_FIR_TAPS_CONST / 2] = 1.0f;
    
    _config.calib.auto_idle_min = SmeaConfig::Calib::AUTO_IDLE_MIN_DEF;
    _config.calib.ref_freq_hz   = SmeaConfig::Calib::REF_FREQ_HZ_DEF;
    
    _config.calib.filter_min_freq_hz = SmeaConfig::Calib::FILTER_MIN_FREQ_HZ_DEF;
    _config.calib.filter_max_freq_hz = SmeaConfig::Calib::FILTER_MAX_FREQ_HZ_DEF;
    _config.calib.target_gain_max    = SmeaConfig::Calib::TARGET_GAIN_MAX_DEF;
    _config.calib.target_gain_min    = SmeaConfig::Calib::TARGET_GAIN_MIN_DEF;
    _config.calib.norm_safe_thresh   = SmeaConfig::Calib::NORM_SAFE_THRESH_DEF;

    _config.feature.band_rms_count = SmeaConfig::Feature::BAND_RMS_COUNT_DEF;
    for (uint8_t i = 0; i < SmeaConfig::FeatureLimit::MAX_BAND_RMS_COUNT_CONST; i++) {
        _config.feature.band_ranges[i][0] = SmeaConfig::Feature::BAND_RANGES_DEF[i][0];
        _config.feature.band_ranges[i][1] = SmeaConfig::Feature::BAND_RANGES_DEF[i][1];
    }
    for (uint8_t i = 0; i < SmeaConfig::FeatureLimit::CEPS_TARGET_COUNT_CONST; i++) {
        _config.feature.ceps_targets[i] = 0.0f;
    }
    _config.feature.spatial_freq_min_hz = 100.0f;
    _config.feature.spatial_freq_max_hz = 4000.0f;
    _config.feature.peak_amplitude_limit_min = SmeaConfig::Feature::PEAK_AMPLITUDE_MIN_DEF;
    _config.feature.peak_freq_gap_limit_hz_min = SmeaConfig::Feature::PEAK_FREQ_GAP_HZ_MIN_DEF;

    _config.decision.rule_enrg_threshold = SmeaConfig::Decision::RULE_ENRG_THRESHOLD_DEF;
    _config.decision.rule_stddev_threshold = SmeaConfig::Decision::RULE_STDDEV_THRESHOLD_DEF;
    _config.decision.test_ng_min_energy = SmeaConfig::Decision::TEST_NG_MIN_ENERGY_DEF;
    _config.decision.min_trigger_count = SmeaConfig::Decision::MIN_TRIGGER_COUNT_DEF;
    _config.decision.noise_profile_sec = SmeaConfig::Decision::NOISE_PROFILE_SEC_DEF;
    _config.decision.valid_start_sec = SmeaConfig::Decision::VALID_START_SEC_DEF;
    _config.decision.valid_end_sec = SmeaConfig::Decision::VALID_END_SEC_DEF;
    _config.decision.sta_lta_threshold = SmeaConfig::Decision::STA_LTA_THRESHOLD_DEF;

    _config.storage.pre_trigger_sec = SmeaConfig::Storage::PRE_TRIGGER_SEC_DEF;
    _config.storage.rotate_mb = SmeaConfig::Storage::ROTATE_MB_DEF;
    _config.storage.rotate_min = SmeaConfig::Storage::ROTATE_MIN_DEF;
    _config.storage.idle_flush_ms = SmeaConfig::Storage::IDLE_FLUSH_MS_DEF;

    strlcpy(_config.mqtt.mqtt_broker, "", sizeof(_config.mqtt.mqtt_broker));
    _config.mqtt.retry_interval_ms = SmeaConfig::Mqtt::RETRY_INTERVAL_MS_DEF;
    _config.mqtt.default_port = SmeaConfig::Mqtt::DEFAULT_PORT_DEF;
    // MQTT 5.0 / QoS 기본값
    _config.mqtt.protocol_ver = SmeaConfig::Mqtt::PROTOCOL_VER_DEF;
    _config.mqtt.qos = SmeaConfig::Mqtt::QOS_DEF;

    _config.wifi.mode = SmeaConfig::Network::WIFI_MODE_DEF;
    strlcpy(_config.wifi.ap_ssid, SmeaConfig::Network::AP_SSID_DEF, sizeof(_config.wifi.ap_ssid));
    strlcpy(_config.wifi.ap_password, SmeaConfig::Network::AP_PW_DEF, sizeof(_config.wifi.ap_password));
    strlcpy(_config.wifi.ap_ip, SmeaConfig::Network::AP_IP_DEF, sizeof(_config.wifi.ap_ip));

    for (uint8_t i = 0; i < SmeaConfig::NetworkLimit::MAX_MULTI_AP_CONST; i++) {
        _config.wifi.multi_ap[i].ssid[0] = '\0';
        _config.wifi.multi_ap[i].password[0] = '\0';
        _config.wifi.multi_ap[i].use_static_ip = false;
        _config.wifi.multi_ap[i].local_ip[0] = '\0';
        _config.wifi.multi_ap[i].gateway[0] = '\0';
        _config.wifi.multi_ap[i].subnet[0] = '\0';
        _config.wifi.multi_ap[i].dns1[0] = '\0';
        _config.wifi.multi_ap[i].dns2[0] = '\0';
    }
}

void T415_ConfigManager::_applyJson(const JsonDocument& p_doc) {
    JsonObjectConst v_dsp = p_doc["dsp"];
    if (!v_dsp.isNull()) {
        _config.dsp.window_ms 			= v_dsp["window_ms"] 			| _config.dsp.window_ms;
        _config.dsp.hop_ms 				= v_dsp["hop_ms"] 				| _config.dsp.hop_ms;
        _config.dsp.notch_freq_hz 		= v_dsp["notch_freq_hz"] 		| _config.dsp.notch_freq_hz;
        _config.dsp.notch_freq_2_hz 	= v_dsp["notch_freq_2_hz"] 		| _config.dsp.notch_freq_2_hz;
        _config.dsp.notch_q_factor 		= v_dsp["notch_q_factor"] 		| _config.dsp.notch_q_factor;
        _config.dsp.pre_emphasis_alpha 	= v_dsp["pre_emphasis_alpha"] 	| _config.dsp.pre_emphasis_alpha;
        _config.dsp.beamforming_gain 	= v_dsp["beamforming_gain"]     | _config.dsp.beamforming_gain;
        _config.dsp.fir_lpf_cutoff 		= v_dsp["fir_lpf_cutoff"]       | _config.dsp.fir_lpf_cutoff;
        _config.dsp.fir_hpf_cutoff 		= v_dsp["fir_hpf_cutoff"]       | _config.dsp.fir_hpf_cutoff;
        _config.dsp.median_window 		= v_dsp["median_window"]        | _config.dsp.median_window;
        _config.dsp.noise_gate_thresh 	= v_dsp["noise_gate_thresh"]    | _config.dsp.noise_gate_thresh;
        _config.dsp.noise_learn_frames 	= v_dsp["noise_learn_frames"]   | _config.dsp.noise_learn_frames;
        _config.dsp.spectral_sub_gain 	= v_dsp["spectral_sub_gain"]    | _config.dsp.spectral_sub_gain;
        
        _config.dsp.calib_gain_L = v_dsp["calib_gain_L"] | _config.dsp.calib_gain_L;
        _config.dsp.calib_gain_R = v_dsp["calib_gain_R"] | _config.dsp.calib_gain_R;
        JsonArrayConst v_eq = v_dsp["calib_eq_coeffs"];
        if (!v_eq.isNull()) {
            uint16_t i = 0;
            for (float v_val : v_eq) {
                if (i >= SmeaConfig::CalibLimit::EQ_FIR_TAPS_CONST) break;
                _config.dsp.calib_eq_coeffs[i] = v_val;
                i++;
            }
        }

    }
    
    JsonObjectConst v_calib = p_doc["calib"];
    if (!v_calib.isNull()) {
        _config.calib.auto_idle_min = v_calib["auto_idle_min"] | _config.calib.auto_idle_min;
        _config.calib.ref_freq_hz = v_calib["ref_freq_hz"] | _config.calib.ref_freq_hz;
        
        _config.calib.filter_min_freq_hz = v_calib["filter_min_freq_hz"] | _config.calib.filter_min_freq_hz;
        _config.calib.filter_max_freq_hz = v_calib["filter_max_freq_hz"] | _config.calib.filter_max_freq_hz;
        _config.calib.target_gain_max    = v_calib["target_gain_max"] | _config.calib.target_gain_max;
        _config.calib.target_gain_min    = v_calib["target_gain_min"] | _config.calib.target_gain_min;
        _config.calib.norm_safe_thresh   = v_calib["norm_safe_thresh"] | _config.calib.norm_safe_thresh;
    }

    JsonObjectConst v_feature = p_doc["feature"];
    if (!v_feature.isNull()) {
        _config.feature.band_rms_count = v_feature["band_rms_count"] | _config.feature.band_rms_count;
        JsonArrayConst v_ranges = v_feature["band_ranges"];
        if (!v_ranges.isNull()) {
            uint8_t i = 0;
            for (JsonArrayConst v_band : v_ranges) {
                if (i >= SmeaConfig::FeatureLimit::MAX_BAND_RMS_COUNT_CONST) break;
                _config.feature.band_ranges[i][0] = v_band[0] | _config.feature.band_ranges[i][0];
                _config.feature.band_ranges[i][1] = v_band[1] | _config.feature.band_ranges[i][1];
                i++;
            }
        }
        JsonArrayConst v_ceps = v_feature["ceps_targets"];
        if (!v_ceps.isNull()) {
            uint8_t i = 0;
            for (float v_val : v_ceps) {
                if (i >= SmeaConfig::FeatureLimit::CEPS_TARGET_COUNT_CONST) break;
                _config.feature.ceps_targets[i] = v_val;
                i++;
            }
        }
        _config.feature.spatial_freq_min_hz = v_feature["spatial_freq_min_hz"] | _config.feature.spatial_freq_min_hz;
        _config.feature.spatial_freq_max_hz = v_feature["spatial_freq_max_hz"] | _config.feature.spatial_freq_max_hz;
        _config.feature.peak_amplitude_limit_min = v_feature["peak_amplitude_limit_min"] | _config.feature.peak_amplitude_limit_min;
        _config.feature.peak_freq_gap_limit_hz_min = v_feature["peak_freq_gap_limit_hz_min"] | _config.feature.peak_freq_gap_limit_hz_min;
    }

    JsonObjectConst v_decision = p_doc["decision"];
    if (!v_decision.isNull()) {
        _config.decision.rule_enrg_threshold = v_decision["rule_enrg_threshold"] | _config.decision.rule_enrg_threshold;
        _config.decision.rule_stddev_threshold = v_decision["rule_stddev_threshold"] | _config.decision.rule_stddev_threshold;
        _config.decision.test_ng_min_energy = v_decision["test_ng_min_energy"] | _config.decision.test_ng_min_energy;
        _config.decision.min_trigger_count = v_decision["min_trigger_count"] | _config.decision.min_trigger_count;
        _config.decision.noise_profile_sec = v_decision["noise_profile_sec"] | _config.decision.noise_profile_sec;
        _config.decision.valid_start_sec = v_decision["valid_start_sec"] | _config.decision.valid_start_sec;
        _config.decision.valid_end_sec = v_decision["valid_end_sec"] | _config.decision.valid_end_sec;
        _config.decision.sta_lta_threshold = v_decision["sta_lta_threshold"] | _config.decision.sta_lta_threshold;
    }

    JsonObjectConst v_storage = p_doc["storage"];
    if (!v_storage.isNull()) {
        _config.storage.pre_trigger_sec = v_storage["pre_trigger_sec"] | _config.storage.pre_trigger_sec;
        _config.storage.rotate_mb = v_storage["rotate_mb"] | _config.storage.rotate_mb;
        _config.storage.rotate_min = v_storage["rotate_min"] | _config.storage.rotate_min;
        _config.storage.idle_flush_ms = v_storage["idle_flush_ms"] | _config.storage.idle_flush_ms;
    }

    JsonObjectConst v_mqtt = p_doc["mqtt"];
    if (!v_mqtt.isNull()) {
        strlcpy(_config.mqtt.mqtt_broker, v_mqtt["mqtt_broker"] | _config.mqtt.mqtt_broker, sizeof(_config.mqtt.mqtt_broker));
        _config.mqtt.retry_interval_ms = v_mqtt["retry_interval_ms"] | _config.mqtt.retry_interval_ms;
        _config.mqtt.default_port = v_mqtt["default_port"] | _config.mqtt.default_port;
        // MQTT 5.0 / QoS JSON 파싱 맵핑 추가
        _config.mqtt.protocol_ver = v_mqtt["protocol_ver"] | _config.mqtt.protocol_ver;
        _config.mqtt.qos = v_mqtt["qos"] | _config.mqtt.qos;
    }

    JsonObjectConst v_wifi = p_doc["wifi"];
    if (!v_wifi.isNull()) {
        _config.wifi.mode = v_wifi["mode"] | _config.wifi.mode;
        strlcpy(_config.wifi.ap_ssid, v_wifi["ap_ssid"] | _config.wifi.ap_ssid, SmeaConfig::NetworkLimit::MAX_SSID_LEN_CONST);
        strlcpy(_config.wifi.ap_password, v_wifi["ap_password"] | _config.wifi.ap_password, SmeaConfig::NetworkLimit::MAX_PW_LEN_CONST);
        strlcpy(_config.wifi.ap_ip, v_wifi["ap_ip"] | _config.wifi.ap_ip, SmeaConfig::NetworkLimit::MAX_IP_LEN_CONST);

        JsonArrayConst v_multi = v_wifi["multi_ap"];
        if (!v_multi.isNull()) {
            uint8_t i = 0;
            for (JsonObjectConst v_ap : v_multi) {
                if (i >= SmeaConfig::NetworkLimit::MAX_MULTI_AP_CONST) break;
                strlcpy(_config.wifi.multi_ap[i].ssid, v_ap["ssid"] | _config.wifi.multi_ap[i].ssid, SmeaConfig::NetworkLimit::MAX_SSID_LEN_CONST);
                strlcpy(_config.wifi.multi_ap[i].password, v_ap["password"] | _config.wifi.multi_ap[i].password, SmeaConfig::NetworkLimit::MAX_PW_LEN_CONST);
                _config.wifi.multi_ap[i].use_static_ip = v_ap["use_static_ip"] | _config.wifi.multi_ap[i].use_static_ip;
                strlcpy(_config.wifi.multi_ap[i].local_ip, v_ap["local_ip"] | _config.wifi.multi_ap[i].local_ip, SmeaConfig::NetworkLimit::MAX_IP_LEN_CONST);
                strlcpy(_config.wifi.multi_ap[i].gateway, v_ap["gateway"] | _config.wifi.multi_ap[i].gateway, SmeaConfig::NetworkLimit::MAX_IP_LEN_CONST);
                strlcpy(_config.wifi.multi_ap[i].subnet, v_ap["subnet"] | _config.wifi.multi_ap[i].subnet, SmeaConfig::NetworkLimit::MAX_IP_LEN_CONST);
                strlcpy(_config.wifi.multi_ap[i].dns1, v_ap["dns1"] | _config.wifi.multi_ap[i].dns1, SmeaConfig::NetworkLimit::MAX_IP_LEN_CONST);
                strlcpy(_config.wifi.multi_ap[i].dns2, v_ap["dns2"] | _config.wifi.multi_ap[i].dns2, SmeaConfig::NetworkLimit::MAX_IP_LEN_CONST);
                i++;
            }
        }
    }
}

bool T415_ConfigManager::load() {
    xSemaphoreTake(_lock, portMAX_DELAY);
    File v_file = LittleFS.open(SmeaConfig::Path::SYS_CFG_JSON_DEF, "r");
    if (!v_file) {
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
    xSemaphoreGive(_lock);
    return true;
}

// Lazy Write (지연 쓰기) 적용
// 웹 API에서 설정 변경이 들어올 때마다 파일을 직접 저장(save)하지 않고, 
// 메모리 객체(_config)만 업데이트한 뒤 _isDirty 플래그를 세웁니다.
bool T415_ConfigManager::updateFromJson(const char* p_jsonString) {
    JsonDocument v_doc;
    DeserializationError v_err = deserializeJson(v_doc, p_jsonString);

    if (v_err) {
        ESP_LOGE(TAG, "Update JSON Parse Error: %s", v_err.c_str());
        return false;
    }

    xSemaphoreTake(_lock, portMAX_DELAY);
    _applyJson(v_doc);
    _isDirty = true;
    _lastModifiedMs = millis(); // 타이머 리셋
    xSemaphoreGive(_lock);

    return true; // 즉시 파일 시스템에 접근하지 않고 빠져나감 (GC Freeze 방지)
}

// 메인 루프에서 호출되는 Lazy Write 커미터
void T415_ConfigManager::checkLazyWrite() {
    if (!_isDirty) return;

    // 마지막 사용자 조작 후 3초(3000ms) 동안 추가 변경이 없을 때만 물리적 파일에 1회 원자적 저장
    if (millis() - _lastModifiedMs > 3000) {
        save();
        _isDirty = false;
        ESP_LOGI(TAG, "Lazy Write Committed to LittleFS");
    }
}

bool T415_ConfigManager::save() {
    xSemaphoreTake(_lock, portMAX_DELAY);

    JsonDocument v_doc;

    JsonObject v_dsp = v_doc["dsp"].to<JsonObject>();
    v_dsp["window_ms"] = _config.dsp.window_ms;
    v_dsp["hop_ms"] = _config.dsp.hop_ms;
    v_dsp["notch_freq_hz"] = _config.dsp.notch_freq_hz;
    v_dsp["notch_freq_2_hz"] = _config.dsp.notch_freq_2_hz;
    v_dsp["notch_q_factor"] = _config.dsp.notch_q_factor;
    v_dsp["pre_emphasis_alpha"] = _config.dsp.pre_emphasis_alpha;
    v_dsp["beamforming_gain"] = _config.dsp.beamforming_gain;
    v_dsp["fir_lpf_cutoff"] = _config.dsp.fir_lpf_cutoff;
    v_dsp["fir_hpf_cutoff"] = _config.dsp.fir_hpf_cutoff;
    v_dsp["median_window"] = _config.dsp.median_window;
    v_dsp["noise_gate_thresh"] = _config.dsp.noise_gate_thresh;
    v_dsp["noise_learn_frames"] = _config.dsp.noise_learn_frames;
    v_dsp["spectral_sub_gain"] = _config.dsp.spectral_sub_gain;
    
    v_dsp["calib_gain_L"] = _config.dsp.calib_gain_L;
    v_dsp["calib_gain_R"] = _config.dsp.calib_gain_R;
    JsonArray v_eq = v_dsp["calib_eq_coeffs"].to<JsonArray>();
    for(uint16_t i = 0; i < SmeaConfig::CalibLimit::EQ_FIR_TAPS_CONST; i++) {
        v_eq.add(_config.dsp.calib_eq_coeffs[i]);
    }
    
    JsonObject v_calib = v_doc["calib"].to<JsonObject>();
    v_calib["auto_idle_min"] = _config.calib.auto_idle_min;
    v_calib["ref_freq_hz"] = _config.calib.ref_freq_hz;

    v_calib["filter_min_freq_hz"] = _config.calib.filter_min_freq_hz;
    v_calib["filter_max_freq_hz"] = _config.calib.filter_max_freq_hz;
    v_calib["target_gain_max"]    = _config.calib.target_gain_max;
    v_calib["target_gain_min"]    = _config.calib.target_gain_min;
    v_calib["norm_safe_thresh"]   = _config.calib.norm_safe_thresh;

    JsonObject v_feature = v_doc["feature"].to<JsonObject>();
    v_feature["band_rms_count"] = _config.feature.band_rms_count;
    JsonArray v_ranges = v_feature["band_ranges"].to<JsonArray>();
    for (uint8_t i = 0; i < SmeaConfig::FeatureLimit::MAX_BAND_RMS_COUNT_CONST; i++) {
        JsonArray v_band = v_ranges.add<JsonArray>();
        v_band.add(_config.feature.band_ranges[i][0]);
        v_band.add(_config.feature.band_ranges[i][1]);
    }
    JsonArray v_ceps = v_feature["ceps_targets"].to<JsonArray>();
    for (uint8_t i = 0; i < SmeaConfig::FeatureLimit::CEPS_TARGET_COUNT_CONST; i++) {
        v_ceps.add(_config.feature.ceps_targets[i]);
    }
    v_feature["spatial_freq_min_hz"] = _config.feature.spatial_freq_min_hz;
    v_feature["spatial_freq_max_hz"] = _config.feature.spatial_freq_max_hz;
    v_feature["peak_amplitude_limit_min"] = _config.feature.peak_amplitude_limit_min;
    v_feature["peak_freq_gap_limit_hz_min"] = _config.feature.peak_freq_gap_limit_hz_min;

    JsonObject v_decision = v_doc["decision"].to<JsonObject>();
    v_decision["rule_enrg_threshold"] = _config.decision.rule_enrg_threshold;
    v_decision["rule_stddev_threshold"] = _config.decision.rule_stddev_threshold;
    v_decision["test_ng_min_energy"] = _config.decision.test_ng_min_energy;
    v_decision["min_trigger_count"] = _config.decision.min_trigger_count;
    v_decision["noise_profile_sec"] = _config.decision.noise_profile_sec;
    v_decision["valid_start_sec"] = _config.decision.valid_start_sec;
    v_decision["valid_end_sec"] = _config.decision.valid_end_sec;
    v_decision["sta_lta_threshold"] = _config.decision.sta_lta_threshold;

    JsonObject v_storage = v_doc["storage"].to<JsonObject>();
    v_storage["pre_trigger_sec"] = _config.storage.pre_trigger_sec;
    v_storage["rotate_mb"] = _config.storage.rotate_mb;
    v_storage["rotate_min"] = _config.storage.rotate_min;
    v_storage["idle_flush_ms"] = _config.storage.idle_flush_ms;

    JsonObject v_mqtt = v_doc["mqtt"].to<JsonObject>();
    v_mqtt["mqtt_broker"] = _config.mqtt.mqtt_broker;
    v_mqtt["retry_interval_ms"] = _config.mqtt.retry_interval_ms;
    v_mqtt["default_port"] = _config.mqtt.default_port;
    // [교정] MQTT 5.0 / QoS Flash 저장 맵핑 추가
    v_mqtt["protocol_ver"] = _config.mqtt.protocol_ver;
    v_mqtt["qos"] = _config.mqtt.qos;

    JsonObject v_wifi = v_doc["wifi"].to<JsonObject>();
    v_wifi["mode"] = _config.wifi.mode;
    v_wifi["ap_ssid"] = _config.wifi.ap_ssid;
    v_wifi["ap_password"] = _config.wifi.ap_password;
    v_wifi["ap_ip"] = _config.wifi.ap_ip;

    JsonArray v_multi = v_wifi["multi_ap"].to<JsonArray>();
    for (uint8_t i = 0; i < SmeaConfig::NetworkLimit::MAX_MULTI_AP_CONST; i++) {
        JsonObject v_ap = v_multi.add<JsonObject>();
        v_ap["ssid"] = _config.wifi.multi_ap[i].ssid;
        v_ap["password"] = _config.wifi.multi_ap[i].password;
        v_ap["use_static_ip"] = _config.wifi.multi_ap[i].use_static_ip;
        v_ap["local_ip"] = _config.wifi.multi_ap[i].local_ip;
        v_ap["gateway"] = _config.wifi.multi_ap[i].gateway;
        v_ap["subnet"] = _config.wifi.multi_ap[i].subnet;
        v_ap["dns1"] = _config.wifi.multi_ap[i].dns1;
        v_ap["dns2"] = _config.wifi.multi_ap[i].dns2;
    }

    File v_file = LittleFS.open("/sys/config.tmp", "w");
    if (!v_file) {
        xSemaphoreGive(_lock);
        return false;
    }

    serializeJson(v_doc, v_file);
    v_file.close();

    LittleFS.remove(SmeaConfig::Path::SYS_CFG_JSON_DEF);
    LittleFS.rename("/sys/config.tmp", SmeaConfig::Path::SYS_CFG_JSON_DEF);

    xSemaphoreGive(_lock);
    return true;
}

void T415_ConfigManager::resetToDefault() {
    xSemaphoreTake(_lock, portMAX_DELAY);
    _loadDefaults();
    xSemaphoreGive(_lock);
    save();
}

DynamicConfig T415_ConfigManager::getConfig() {
    DynamicConfig v_copy;
    xSemaphoreTake(_lock, portMAX_DELAY);
    v_copy = _config;
    xSemaphoreGive(_lock);
    return v_copy;
}

bool T415_ConfigManager::updateConfig(const DynamicConfig& p_newConfig) {
    xSemaphoreTake(_lock, portMAX_DELAY);
    _config = p_newConfig;
    xSemaphoreGive(_lock);
    return save();
}


// [v013 신규 메서드] 1. Preview (메모리에만 덮어쓰고 플래시는 건드리지 않음)
bool T415_ConfigManager::updatePreview(const char* p_jsonString) {
    JsonDocument v_doc;
    DeserializationError v_err = deserializeJson(v_doc, p_jsonString);

    if (v_err) {
        ESP_LOGE(TAG, "Preview JSON Parse Error: %s", v_err.c_str());
        return false;
    }

    xSemaphoreTake(_lock, portMAX_DELAY);
    _applyJson(v_doc);
    
    // Lazy Write용 깃발(_isDirty)은 세우지 않습니다. (플래시 쓰기 완벽 차단)
    _isDirty = false; 
    
    // 대신 DSP 핫패스가 즉시 캐시를 리로드하도록 Lock-free 깃발을 세웁니다.
    _isTuningActive = true; 
    xSemaphoreGive(_lock);

    return true; 
}

// [v013 신규 메서드] 2. Commit Save (현재 메모리 값을 플래시에 원자적 기록)
bool T415_ConfigManager::commitSave() {
    bool v_res = save(); // 내부적으로 Mutex 잡고 플래시에 기록
    if (v_res) {
        _isDirty = false;
        _isTuningActive = false; // 튜닝 확정 (더 이상 리로드 불필요)
        ESP_LOGI(TAG, "Tuning Committed to LittleFS");
    }
    return v_res;
}

// [v013 신규 메서드] 3. Revert Cancel (메모리 파기 및 기존 플래시 값으로 복원)
bool T415_ConfigManager::revertCancel() {
    // 1. 플래시에서 마지막 정상 저장본을 RAM으로 읽어옴 (Mutex 내부 처리)
    bool v_res = load(); 
    
    if (v_res) {
        _isDirty = false;
        
        // 2. RAM의 값이 옛날 정상 값으로 돌아갔으므로, 
        // DSP 캐시 엔진이 이를 인지하고 옛날 값으로 원복하도록 깃발을 한번 펄럭여 줍니다.
        _isTuningActive = true; 
        
        ESP_LOGI(TAG, "Tuning Canceled. Reverted to previous saved config.");
    }
    return v_res;
}

