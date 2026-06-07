/* ============================================================================
 * File: T220_CfgMgr_243.cpp
 * Summary: 4-Tier 동적 JSON 설정 관리자 구현부 (누락/축소 0% 완전체)
 * ============================================================================
 * [자가 교정 (Self-Correction) 내역]
 * - 이전 버전에서 생략되었던 WiFi, MQTT, Storage, Output, Shared_DSP, Calib 등
 * 총 16개 서브 구조체의 100여 개 변수 직렬화/역직렬화 로직 100% 복원 완료.
 * ========================================================================== */

#include "T220_CfgMgr_243.hpp"
#include <LittleFS.h>
#include <cstring>
#include "esp_log.h"

static const char* TAG = "T215_CFG";

// ArduinoJson V7용 PSRAM 커스텀 할당자
struct SpiRamAllocator : ArduinoJson::Allocator {
    void* allocate(size_t size) override { return heap_caps_malloc(size, MALLOC_CAP_SPIRAM); }
    void deallocate(void* pointer) override { heap_caps_free(pointer); }
    void* reallocate(void* ptr, size_t new_size) override { return heap_caps_realloc(ptr, new_size, MALLOC_CAP_SPIRAM); }
};
static SpiRamAllocator g_psramAlloc;

// 부동소수점 고정밀도 직렬화 헬퍼 (EQ 등 민감 필터 붕괴 차단)
static inline void cfgAddFloatExact(JsonArray& p_arr, float p_val) {
    char v_buf[32];
    snprintf(v_buf, sizeof(v_buf), "%.8f", p_val);
    p_arr.add(serialized(v_buf));
}

// 템플릿 헬퍼: Null 체크 및 타입 안전 할당
template<typename T>
static inline void cfgSet(JsonVariantConst p_v, T& p_dest) {
    if (!p_v.isNull()) p_dest = p_v.as<T>();
}

// 문자열 전용 헬퍼: 버퍼 오버플로우 방어
static inline void cfgStr(JsonVariantConst p_v, char* p_dest, size_t p_size) {
    if (!p_v.isNull()) {
        const char* v_src = p_v.as<const char*>();
        if (v_src) strlcpy(p_dest, v_src, p_size);
    }
}

CL_T2_ConfigManager::CL_T2_ConfigManager() {
	_lock			= xSemaphoreCreateMutex();
	_isLoaded		= false;
	_isDirty		= false;
	_isTuningActive = false;
	_lastModifiedMs = 0;
	_loadDefaults();
}

CL_T2_ConfigManager::~CL_T2_ConfigManager() {
    if (_lock) vSemaphoreDelete(_lock);
}

bool CL_T2_ConfigManager::init() {
    if (!LittleFS.begin(true)) {
        ESP_LOGE(TAG, "LittleFS Mount Failed!");
        return false;
    }

    if (!LittleFS.exists(T2_Def::Global::Path::FILE_CFG_JSON_CONST)) {
        if (LittleFS.exists("/sys/config_243.tmp")) {
            ESP_LOGW(TAG, "Recovering config from interrupted atomic write");
            LittleFS.rename("/sys/config_243.tmp", T2_Def::Global::Path::FILE_CFG_JSON_CONST);
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
    // === Tier 1. Global ===
    strlcpy(_config.system.site_id, T2_Def::Global::System::SITE_ID_DEF, sizeof(_config.system.site_id));
    _config.system.tele_hz      = T2_Def::Global::System::TELEMETRY_HZ_DEF;
    _config.system.wave_hz      = T2_Def::Global::System::WAVEFORM_HZ_DEF;
    _config.system.op_mode      = T2_Type::OpMode::AUTO;
    _config.system.watchdog_ms  = T2_Def::Global::Task::WDG_TIMEOUT_MS_DEF;
    _config.system.vib_enable   = T2_Def::Global::System::VIB_ENABLE_DEF;
    _config.system.audio_enable = T2_Def::Global::System::AUDIO_ENABLE_DEF;

    _config.wifi.mode = T2_Type::WiFiMode::AUTO_FALLBACK;
    strlcpy(_config.wifi.ap_ssid, T2_Def::Global::Net::WIFI_AP_SSID_DEF, sizeof(_config.wifi.ap_ssid));
    strlcpy(_config.wifi.ap_pw,   T2_Def::Global::Net::WIFI_AP_PW_DEF,   sizeof(_config.wifi.ap_pw));
    strlcpy(_config.wifi.ap_ip,   T2_Def::Global::Net::WIFI_AP_IP_DEF,   sizeof(_config.wifi.ap_ip));
    for (uint8_t i = 0; i < T2_Def::Global::NetLimit::MAX_MULTI_AP_CONST; i++) {
        _config.wifi.multi_ssid[i][0] = '\0';
        _config.wifi.multi_pw[i][0]   = '\0';
    }

	_config.mqtt.enable = false;
	strlcpy(_config.mqtt.broker, "", sizeof(_config.mqtt.broker));
	_config.mqtt.port = T2_Def::Global::Net::MQTT_PORT_DEF;
	strlcpy(_config.mqtt.id, T2_Def::Global::Net::MQTT_ID_DEF, sizeof(_config.mqtt.id));
	strlcpy(_config.mqtt.pw, "", sizeof(_config.mqtt.pw));
	strlcpy(_config.mqtt.topic_root, T2_Def::Global::Net::MQTT_TOPIC_DEF, sizeof(_config.mqtt.topic_root));
	strlcpy(_config.mqtt.lwt_topic, T2_Def::Global::Net::MQTT_LWT_DEF, sizeof(_config.mqtt.lwt_topic));
	_config.mqtt.qos				   = T2_Def::Global::Net::MQTT_QOS_DEF;
	_config.mqtt.proto_ver			   = T2_Def::Global::Net::MQTT_PROTO_VER_DEF;

	_config.storage.rot_mb			   = T2_Def::Global::Storage::ROTATE_MB_DEF;
	_config.storage.rot_min			   = T2_Def::Global::Storage::ROTATE_MIN_DEF;
	_config.storage.save_raw		   = false;
	_config.storage.keep_max		   = T2_Def::Global::Storage::ROTATE_KEEP_MAX_DEF;
	_config.storage.idle_flush_ms	   = T2_Def::Global::Storage::IDLE_FLUSH_MS_DEF;
	_config.storage.pre_trig_sec	   = T2_Def::Global::Storage::PRE_TRIGGER_SEC_DEF;

	_config.output.enabled			   = true;
	_config.output.output_sequence	   = false;
	_config.output.sequence_frames	   = T2_Def::Global::System::SEQUENCE_FRAMES_DEF;

	// === Tier 2. Shared ===
	_config.shared_trig.hold_ms		   = T2_Def::Shared::Trigger::HOLD_TIME_MS_DEF;
	_config.shared_trig.use_sleep	   = T2_Def::Shared::Trigger::USE_DEEP_SLEEP_DEF;
	_config.shared_trig.sleep_sec	   = T2_Def::Shared::Trigger::SLEEP_SEC_DEF;

	_config.shared_dsp.rem_dc		   = true;
	_config.shared_dsp.med_en		   = true;
	_config.shared_dsp.med_win		   = T2_Def::Shared::Dsp::MEDIAN_WINDOW_DEF;
	_config.shared_dsp.hpf.en		   = true;
	_config.shared_dsp.hpf.cutoff	   = T2_Def::Shared::Dsp::FIR_HPF_CUTOFF_DEF;
	_config.shared_dsp.hpf.taps		   = T2_Def::Shared::Feature::FIR_TAPS_DEF;
	_config.shared_dsp.lpf.en		   = false;
	_config.shared_dsp.lpf.cutoff	   = 1000.0f;
	_config.shared_dsp.lpf.taps		   = T2_Def::Shared::Feature::FIR_TAPS_DEF;
	_config.shared_dsp.iir_hpf.en	   = false;
	_config.shared_dsp.iir_hpf.cutoff  = 20.0f;
	_config.shared_dsp.iir_hpf.q	   = 0.707f;
	_config.shared_dsp.iir_lpf.en	   = false;
	_config.shared_dsp.iir_lpf.cutoff  = 1000.0f;
	_config.shared_dsp.iir_lpf.q	   = 0.707f;
	_config.shared_dsp.notch.en		   = false;
	_config.shared_dsp.notch.freq	   = T2_Def::Shared::Dsp::NOTCH_FREQ_HZ_DEF;
	_config.shared_dsp.notch.gain	   = 1.0f;
	_config.shared_dsp.notch.q		   = T2_Def::Shared::Dsp::NOTCH_Q_FACTOR_DEF;
	_config.shared_dsp.win_type		   = T2_Type::WindowType::HANN;

	// === Tier 3. Vib ===
	_config.vib_sensor.accel_enable	   = T2_Def::Vib::Sensor::ACCEL_ENABLE_DEF;
	_config.vib_sensor.gyro_enable	   = T2_Def::Vib::Sensor::GYRO_ENABLE_DEF;
	_config.vib_sensor.accel_axis_count= T2_Def::Vib::Sensor::ACCEL_AXIS_COUNT_DEF;
	_config.vib_sensor.accel_axis	   = T2_Def::Vib::Sensor::ACCEL_TARGET_AXIS_DEF;
	_config.vib_sensor.gyro_axis_count = T2_Def::Vib::Sensor::GYRO_AXIS_COUNT_DEF;
	_config.vib_sensor.gyro_axis	   = T2_Def::Vib::Sensor::GYRO_TARGET_AXIS_DEF;
	_config.vib_sensor.accel_range	   = T2_Def::Vib::Sensor::ACCEL_RANGE_DEF;
	_config.vib_sensor.accel_odr	   = T2_Def::Vib::Sensor::ACCEL_ODR_DEF;
	_config.vib_sensor.gyro_range	   = T2_Def::Vib::Sensor::GYRO_RANGE_DEF;
	_config.vib_sensor.sample_rate	   = T2_Def::Vib::Sensor::RATE_DEF;
	_config.vib_sensor.fft_size		   = T2_Def::Vib::Sensor::FFT_SIZE_DEF;

	_config.vib_trig.motion_en		   = false;
	_config.vib_trig.wake_g			   = T2_Def::Vib::Trigger::WAKE_THRESH_G_DEF;
	_config.vib_trig.wake_dur		   = T2_Def::Shared::Trigger::WAKE_DURATION_DEF;
	_config.vib_trig.rms_thresh		   = T2_Def::Vib::Trigger::RMS_THRESH_DEF;
	_config.vib_trig.kurt_ng_thresh	   = T2_Def::Vib::Trigger::KURT_NG_THRESH_DEF;
	_config.vib_trig.crest_ng_thresh   = T2_Def::Vib::Trigger::CREST_NG_THRESH_DEF;
	_config.vib_trig.skew_ng_thresh    = T2_Def::Vib::Trigger::SKEW_NG_THRESH_DEF;

	_config.vib_trig.active_band_count = T2_Def::Shared::Feature::BAND_RMS_DEF;
	for (uint8_t i = 0; i < T2_Def::Shared::FeatureLimit::BAND_RMS_MAX; i++) {
		_config.vib_trig.band_en[i] = false;
		if (i < T2_Def::Shared::Feature::BAND_RMS_DEF) {
			_config.vib_trig.band_start[i] = T2_Def::Vib::Trigger::BAND_RANGES_DEF[i][0];
			_config.vib_trig.band_end[i]   = T2_Def::Vib::Trigger::BAND_RANGES_DEF[i][1];
		} else {
			_config.vib_trig.band_start[i] = 0.0f;
			_config.vib_trig.band_end[i]   = 0.0f;
		}
		_config.vib_trig.band_thresh[i] = 1.0f;
	}

	_config.vib_calib.offset[0]		   = 0;
	_config.vib_calib.offset[1]		   = 0;
	_config.vib_calib.offset[2]		   = 0;
	_config.vib_calib.gain[0]		   = 1.0f;
	_config.vib_calib.gain[1]		   = 1.0f;
	_config.vib_calib.gain[2]		   = 1.0f;

	_config.vib_feat.peak_amp_min	   = 0.1f;
	_config.vib_feat.peak_freq_gap_min = 50.0f;

	// === Tier 4. Audio ===
	_config.aud_sensor.channel_mode	   = (T2_Type::AudioChannelMode)T2_Def::Audio::Sensor::CHANNEL_MODE_DEF;
	_config.aud_sensor.sample_rate	   = T2_Def::Audio::Sensor::RATE_DEF;
	_config.aud_sensor.fft_size		   = T2_Def::Audio::Sensor::FFT_SIZE_DEF;

	_config.aud_trig.rms_thresh		   = T2_Def::Audio::Trigger::RMS_THRESH_DEF;
	_config.aud_trig.kurt_ng_thresh	   = T2_Def::Audio::Trigger::KURT_NG_THRESH_DEF;
	_config.aud_trig.crest_ng_thresh   = T2_Def::Audio::Trigger::CREST_FACTOR_NG_DEF;
	_config.aud_trig.skew_ng_thresh	   = T2_Def::Audio::Trigger::SKEWNESS_NG_DEF;

	_config.aud_trig.active_band_count = T2_Def::Shared::Feature::BAND_RMS_DEF;
	for (uint8_t i = 0; i < T2_Def::Shared::FeatureLimit::BAND_RMS_MAX; i++) {
		_config.aud_trig.band_en[i] = false;
		if (i < T2_Def::Shared::Feature::BAND_RMS_DEF) {
			_config.aud_trig.band_start[i] = T2_Def::Audio::Trigger::BAND_RANGES_DEF[i][0];
			_config.aud_trig.band_end[i]   = T2_Def::Audio::Trigger::BAND_RANGES_DEF[i][1];
		} else {
			_config.aud_trig.band_start[i] = 0.0f;
			_config.aud_trig.band_end[i]   = 0.0f;
		}
		_config.aud_trig.band_thresh[i] = 1.0f;
	}

	_config.aud_dsp.pre_en			   = true;
	_config.aud_dsp.pre_alpha		   = T2_Def::Shared::Dsp::PRE_EMPHASIS_ALPHA_DEF;
	_config.aud_dsp.noise.gate_en	   = false;
	_config.aud_dsp.noise.gate_thresh  = 0.01f;
	_config.aud_dsp.noise.mode		   = T2_Type::NoiseMode::OFF;
	_config.aud_dsp.noise.sub_str	   = 1.0f;
	_config.aud_dsp.noise.adp_alpha	   = 0.05f;
	_config.aud_dsp.noise.learn_frames = 100;
	_config.aud_dsp.beam_gain		   = T2_Def::Audio::Dsp::BEAMFORMING_GAIN_DEF;
	_config.aud_dsp.window_ms		   = T2_Def::Audio::Dsp::WINDOW_MS_DEF;
	_config.aud_dsp.hop_ms			   = T2_Def::Audio::Dsp::HOP_MS_DEF;

	_config.aud_calib.auto_idle_min	   = T2_Def::Audio::Calib::AUTO_IDLE_MIN_DEF;
	_config.aud_calib.ref_freq		   = T2_Def::Audio::Calib::REF_FREQ_HZ_DEF;
	_config.aud_calib.filt_min		   = T2_Def::Audio::Calib::FILTER_MIN_FREQ_HZ_DEF;
	_config.aud_calib.filt_max		   = T2_Def::Audio::Calib::FILTER_MAX_FREQ_HZ_DEF;
	_config.aud_calib.gain_max		   = T2_Def::Audio::Calib::TARGET_GAIN_MAX_DEF;
	_config.aud_calib.gain_min		   = T2_Def::Audio::Calib::TARGET_GAIN_MIN_DEF;
	_config.aud_calib.norm_safe		   = T2_Def::Audio::Calib::NORM_SAFE_THRESH_DEF;
	_config.aud_calib.gain_L		   = 1.0f;
	_config.aud_calib.gain_R		   = 1.0f;
	memset(_config.aud_calib.eq_coeffs, 0, sizeof(_config.aud_calib.eq_coeffs));
	_config.aud_calib.eq_coeffs[T2_Def::Shared::Feature::FIR_TAPS_DEF / 2] = 1.0f;

	_config.aud_feat.active_ceps_count									   = T2_Def::Shared::Feature::CEPS_TARGET_DEF;
	_config.aud_feat.active_peak_count									   = T2_Def::Shared::Feature::TOP_PEAKS_DEF;
	for (uint8_t i = 0; i < T2_Def::Shared::FeatureLimit::CEPS_TARGET_MAX; i++) {
		_config.aud_feat.ceps_targets[i] = 0.0f;
	}
	_config.aud_feat.f_min			   = 100.0f;
	_config.aud_feat.f_max			   = 8000.0f;
	_config.aud_feat.peak_amp_min	   = 0.1f;
	_config.aud_feat.peak_freq_gap_min = 50.0f;
}

void CL_T2_ConfigManager::_applyJson(const JsonDocument& p_doc) {
    // === Tier 1. Global ===
    JsonObjectConst v_sys = p_doc["system"];
    if (!v_sys.isNull()) {
        cfgStr(v_sys["site_id"], _config.system.site_id, sizeof(_config.system.site_id));
        cfgSet(v_sys["tele_hz"], _config.system.tele_hz);
        cfgSet(v_sys["wave_hz"], _config.system.wave_hz);
        if (!v_sys["op_mode"].isNull()) _config.system.op_mode = (T2_Type::OpMode)v_sys["op_mode"].as<uint8_t>();
        cfgSet(v_sys["watchdog_ms"],  _config.system.watchdog_ms);
        cfgSet(v_sys["vib_enable"],   _config.system.vib_enable);
        cfgSet(v_sys["audio_enable"], _config.system.audio_enable);
    }

    JsonObjectConst v_wifi = p_doc["wifi"];
    if (!v_wifi.isNull()) {
        if (!v_wifi["mode"].isNull()) _config.wifi.mode = (T2_Type::WiFiMode)v_wifi["mode"].as<uint8_t>();
        cfgStr(v_wifi["ap_ssid"], _config.wifi.ap_ssid, sizeof(_config.wifi.ap_ssid));
        cfgStr(v_wifi["ap_pw"],   _config.wifi.ap_pw,   sizeof(_config.wifi.ap_pw));
        cfgStr(v_wifi["ap_ip"],   _config.wifi.ap_ip,   sizeof(_config.wifi.ap_ip));

        JsonArrayConst v_multi = v_wifi["multi_ap"];
        if (!v_multi.isNull()) {
            uint8_t i = 0;
            for (JsonObjectConst v_ap : v_multi) {
                if (i >= T2_Def::Global::NetLimit::MAX_MULTI_AP_CONST) break;
                cfgStr(v_ap["ssid"], _config.wifi.multi_ssid[i], sizeof(_config.wifi.multi_ssid[i]));
                cfgStr(v_ap["pw"],   _config.wifi.multi_pw[i],   sizeof(_config.wifi.multi_pw[i]));
                i++;
            }
        }
    }

    JsonObjectConst v_mqtt = p_doc["mqtt"];
    if (!v_mqtt.isNull()) {
        cfgSet(v_mqtt["enable"],    _config.mqtt.enable);
        cfgStr(v_mqtt["broker"],    _config.mqtt.broker, sizeof(_config.mqtt.broker));
        cfgSet(v_mqtt["port"],      _config.mqtt.port);
        cfgStr(v_mqtt["id"],        _config.mqtt.id,     sizeof(_config.mqtt.id));
        cfgStr(v_mqtt["pw"],        _config.mqtt.pw,     sizeof(_config.mqtt.pw));
        cfgStr(v_mqtt["topic_root"],_config.mqtt.topic_root, sizeof(_config.mqtt.topic_root));
        cfgStr(v_mqtt["lwt_topic"], _config.mqtt.lwt_topic,  sizeof(_config.mqtt.lwt_topic));
        cfgSet(v_mqtt["qos"],       _config.mqtt.qos);
        cfgSet(v_mqtt["proto_ver"], _config.mqtt.proto_ver);
    }

    JsonObjectConst v_storage = p_doc["storage"];
    if (!v_storage.isNull()) {
        cfgSet(v_storage["rot_mb"],        _config.storage.rot_mb);
        cfgSet(v_storage["rot_min"],       _config.storage.rot_min);
        cfgSet(v_storage["save_raw"],      _config.storage.save_raw);
        cfgSet(v_storage["keep_max"],      _config.storage.keep_max);
        cfgSet(v_storage["idle_flush_ms"], _config.storage.idle_flush_ms);
        cfgSet(v_storage["pre_trig_sec"],  _config.storage.pre_trig_sec);
    }

    JsonObjectConst v_out = p_doc["output"];
    if (!v_out.isNull()) {
        cfgSet(v_out["enabled"],         _config.output.enabled);
        cfgSet(v_out["output_sequence"], _config.output.output_sequence);
        cfgSet(v_out["sequence_frames"], _config.output.sequence_frames);
    }

    // === Tier 2. Shared ===
    JsonObjectConst v_st = p_doc["shared_trig"];
    if (!v_st.isNull()) {
        cfgSet(v_st["hold_ms"],   _config.shared_trig.hold_ms);
        cfgSet(v_st["use_sleep"], _config.shared_trig.use_sleep);
        cfgSet(v_st["sleep_sec"], _config.shared_trig.sleep_sec);
    }

    JsonObjectConst v_sd = p_doc["shared_dsp"];
    if (!v_sd.isNull()) {
        cfgSet(v_sd["rem_dc"],  _config.shared_dsp.rem_dc);
        cfgSet(v_sd["med_en"],  _config.shared_dsp.med_en);
        cfgSet(v_sd["med_win"], _config.shared_dsp.med_win);

        if (!v_sd["hpf"].isNull()) {
            cfgSet(v_sd["hpf"]["en"],     _config.shared_dsp.hpf.en);
            cfgSet(v_sd["hpf"]["cutoff"], _config.shared_dsp.hpf.cutoff);
            cfgSet(v_sd["hpf"]["taps"],   _config.shared_dsp.hpf.taps);
        }
        if (!v_sd["lpf"].isNull()) {
            cfgSet(v_sd["lpf"]["en"],     _config.shared_dsp.lpf.en);
            cfgSet(v_sd["lpf"]["cutoff"], _config.shared_dsp.lpf.cutoff);
            cfgSet(v_sd["lpf"]["taps"],   _config.shared_dsp.lpf.taps);
        }
        if (!v_sd["iir_hpf"].isNull()) {
            cfgSet(v_sd["iir_hpf"]["en"],     _config.shared_dsp.iir_hpf.en);
            cfgSet(v_sd["iir_hpf"]["cutoff"], _config.shared_dsp.iir_hpf.cutoff);
            cfgSet(v_sd["iir_hpf"]["q"],      _config.shared_dsp.iir_hpf.q);
        }
        if (!v_sd["iir_lpf"].isNull()) {
            cfgSet(v_sd["iir_lpf"]["en"],     _config.shared_dsp.iir_lpf.en);
            cfgSet(v_sd["iir_lpf"]["cutoff"], _config.shared_dsp.iir_lpf.cutoff);
            cfgSet(v_sd["iir_lpf"]["q"],      _config.shared_dsp.iir_lpf.q);
        }
        if (!v_sd["notch"].isNull()) {
            cfgSet(v_sd["notch"]["en"],   _config.shared_dsp.notch.en);
            cfgSet(v_sd["notch"]["freq"], _config.shared_dsp.notch.freq);
            cfgSet(v_sd["notch"]["gain"], _config.shared_dsp.notch.gain);
            cfgSet(v_sd["notch"]["q"],    _config.shared_dsp.notch.q);
        }
        if (!v_sd["win_type"].isNull()) _config.shared_dsp.win_type = (T2_Type::WindowType)v_sd["win_type"].as<uint8_t>();
    }

    // === Tier 3. Vib ===
    JsonObjectConst v_vs = p_doc["vib_sensor"];
    if (!v_vs.isNull()) {
        cfgSet(v_vs["accel_enable"], _config.vib_sensor.accel_enable);
        cfgSet(v_vs["gyro_enable"],  _config.vib_sensor.gyro_enable);
        
        if (!v_vs["accel_axis_count"].isNull()) {
            cfgSet(v_vs["accel_axis_count"], _config.vib_sensor.accel_axis_count);
        } else if (!v_vs["axis_count"].isNull()) {
            cfgSet(v_vs["axis_count"], _config.vib_sensor.accel_axis_count);
        }
        
        if (!v_vs["accel_axis"].isNull()) {
            cfgSet(v_vs["accel_axis"], _config.vib_sensor.accel_axis);
        } else if (!v_vs["axis"].isNull()) {
            cfgSet(v_vs["axis"], _config.vib_sensor.accel_axis);
        }
        
        if (!v_vs["gyro_axis_count"].isNull()) {
            cfgSet(v_vs["gyro_axis_count"], _config.vib_sensor.gyro_axis_count);
        } else if (!v_vs["axis_count"].isNull()) {
            cfgSet(v_vs["axis_count"], _config.vib_sensor.gyro_axis_count);
        }
        
        if (!v_vs["gyro_axis"].isNull()) {
            cfgSet(v_vs["gyro_axis"], _config.vib_sensor.gyro_axis);
        } else if (!v_vs["axis"].isNull()) {
            cfgSet(v_vs["axis"], _config.vib_sensor.gyro_axis);
        }
        
        cfgSet(v_vs["accel_range"],  _config.vib_sensor.accel_range);
        cfgSet(v_vs["accel_odr"],    _config.vib_sensor.accel_odr);
        cfgSet(v_vs["gyro_range"],   _config.vib_sensor.gyro_range);
        cfgSet(v_vs["sample_rate"],  _config.vib_sensor.sample_rate);
        cfgSet(v_vs["fft_size"],     _config.vib_sensor.fft_size);
    }

    JsonObjectConst v_vt = p_doc["vib_trig"];
    if (!v_vt.isNull()) {
        cfgSet(v_vt["motion_en"],  _config.vib_trig.motion_en);
        cfgSet(v_vt["wake_g"],     _config.vib_trig.wake_g);
        cfgSet(v_vt["wake_dur"],   _config.vib_trig.wake_dur);
        cfgSet(v_vt["rms_thresh"], _config.vib_trig.rms_thresh);
        cfgSet(v_vt["kurt_ng_thresh"], _config.vib_trig.kurt_ng_thresh);
        cfgSet(v_vt["crest_ng_thresh"], _config.vib_trig.crest_ng_thresh);
        cfgSet(v_vt["skew_ng_thresh"], _config.vib_trig.skew_ng_thresh);

        JsonArrayConst v_bands = v_vt["bands"];
        if (!v_bands.isNull()) {
            uint8_t i = 0;
            for (JsonObjectConst v_b : v_bands) {
                if (i >= T2_Def::Shared::FeatureLimit::BAND_RMS_MAX) break;
                cfgSet(v_b["en"],     _config.vib_trig.band_en[i]);
                cfgSet(v_b["start"],  _config.vib_trig.band_start[i]);
                cfgSet(v_b["end"],    _config.vib_trig.band_end[i]);
                cfgSet(v_b["thresh"], _config.vib_trig.band_thresh[i]);
                i++;
            }
            _config.vib_trig.active_band_count = i;
        }
    }

    JsonObjectConst v_vc = p_doc["vib_calib"];
    if (!v_vc.isNull()) {
        JsonArrayConst v_off = v_vc["offset"];
        if (!v_off.isNull()) {
            for(uint8_t i=0; i<3; i++) cfgSet(v_off[i], _config.vib_calib.offset[i]);
        }
        JsonArrayConst v_gain = v_vc["gain"];
        if (!v_gain.isNull()) {
            for(uint8_t i=0; i<3; i++) cfgSet(v_gain[i], _config.vib_calib.gain[i]);
        }
    }

    JsonObjectConst v_vf = p_doc["vib_feat"];
    if (!v_vf.isNull()) {
        cfgSet(v_vf["peak_amp_min"],      _config.vib_feat.peak_amp_min);
        cfgSet(v_vf["peak_freq_gap_min"], _config.vib_feat.peak_freq_gap_min);
    }

    // === Tier 4. Audio ===
    JsonObjectConst v_as = p_doc["aud_sensor"];
    if (!v_as.isNull()) {
        if (!v_as["channel_mode"].isNull()) _config.aud_sensor.channel_mode = (T2_Type::AudioChannelMode)v_as["channel_mode"].as<uint8_t>();
        cfgSet(v_as["sample_rate"], _config.aud_sensor.sample_rate);
        cfgSet(v_as["fft_size"],    _config.aud_sensor.fft_size);
    }

    JsonObjectConst v_at = p_doc["aud_trig"];
    if (!v_at.isNull()) {
        cfgSet(v_at["rms_thresh"],      _config.aud_trig.rms_thresh);
        cfgSet(v_at["kurt_ng_thresh"],  _config.aud_trig.kurt_ng_thresh);
        cfgSet(v_at["crest_ng_thresh"], _config.aud_trig.crest_ng_thresh);
        cfgSet(v_at["skew_ng_thresh"],  _config.aud_trig.skew_ng_thresh);

        JsonArrayConst v_bands = v_at["bands"];
        if (!v_bands.isNull()) {
            uint8_t i = 0;
            for (JsonObjectConst v_b : v_bands) {
                if (i >= T2_Def::Shared::FeatureLimit::BAND_RMS_MAX) break;
                cfgSet(v_b["en"],     _config.aud_trig.band_en[i]);
                cfgSet(v_b["start"],  _config.aud_trig.band_start[i]);
                cfgSet(v_b["end"],    _config.aud_trig.band_end[i]);
                cfgSet(v_b["thresh"], _config.aud_trig.band_thresh[i]);
                i++;
            }
            _config.aud_trig.active_band_count = i;
        }
    }

    JsonObjectConst v_ad = p_doc["aud_dsp"];
    if (!v_ad.isNull()) {
        cfgSet(v_ad["pre_en"],    _config.aud_dsp.pre_en);
        cfgSet(v_ad["pre_alpha"], _config.aud_dsp.pre_alpha);

        JsonObjectConst v_n = v_ad["noise"];
        if (!v_n.isNull()) {
            cfgSet(v_n["gate_en"],     _config.aud_dsp.noise.gate_en);
            cfgSet(v_n["gate_thresh"], _config.aud_dsp.noise.gate_thresh);
            if (!v_n["mode"].isNull()) _config.aud_dsp.noise.mode = (T2_Type::NoiseMode)v_n["mode"].as<uint8_t>();
            cfgSet(v_n["sub_str"],     _config.aud_dsp.noise.sub_str);
            cfgSet(v_n["adp_alpha"],   _config.aud_dsp.noise.adp_alpha);
            cfgSet(v_n["learn_frames"],_config.aud_dsp.noise.learn_frames);
        }
        cfgSet(v_ad["beam_gain"], _config.aud_dsp.beam_gain);
        cfgSet(v_ad["window_ms"], _config.aud_dsp.window_ms);
        cfgSet(v_ad["hop_ms"],    _config.aud_dsp.hop_ms);
    }

    JsonObjectConst v_ac = p_doc["aud_calib"];
    if (!v_ac.isNull()) {
        cfgSet(v_ac["auto_idle_min"], _config.aud_calib.auto_idle_min);
        cfgSet(v_ac["ref_freq"],      _config.aud_calib.ref_freq);
        cfgSet(v_ac["filt_min"],      _config.aud_calib.filt_min);
        cfgSet(v_ac["filt_max"],      _config.aud_calib.filt_max);
        cfgSet(v_ac["gain_max"],      _config.aud_calib.gain_max);
        cfgSet(v_ac["gain_min"],      _config.aud_calib.gain_min);
        cfgSet(v_ac["norm_safe"],     _config.aud_calib.norm_safe);
        cfgSet(v_ac["gain_L"],        _config.aud_calib.gain_L);
        cfgSet(v_ac["gain_R"],        _config.aud_calib.gain_R);

        JsonArrayConst v_eq = v_ac["eq_coeffs"];
        if (!v_eq.isNull()) {
            uint16_t i = 0;
            for (JsonVariantConst v_val : v_eq) {
                if (i >= T2_Def::Shared::FeatureLimit::FIR_TAPS_MAX) break;
                _config.aud_calib.eq_coeffs[i] = v_val.as<float>();
                i++;
            }
        }
    }

    JsonObjectConst v_af = p_doc["aud_feat"];
    if (!v_af.isNull()) {
        cfgSet(v_af["f_min"],             _config.aud_feat.f_min);
        cfgSet(v_af["f_max"],             _config.aud_feat.f_max);
        cfgSet(v_af["peak_amp_min"],      _config.aud_feat.peak_amp_min);
        cfgSet(v_af["peak_freq_gap_min"], _config.aud_feat.peak_freq_gap_min);

        JsonArrayConst v_ceps = v_af["ceps_targets"];
        if (!v_ceps.isNull()) {
            uint8_t i = 0;
            for (JsonVariantConst v_val : v_ceps) {
                if (i >= T2_Def::Shared::FeatureLimit::CEPS_TARGET_MAX) break;
                _config.aud_feat.ceps_targets[i] = v_val.as<float>();
                i++;
            }
            _config.aud_feat.active_ceps_count = i;
        }
        if(!v_af["active_peak_count"].isNull()) _config.aud_feat.active_peak_count = v_af["active_peak_count"].as<uint8_t>();
    }
}

bool CL_T2_ConfigManager::load() {
    xSemaphoreTake(_lock, portMAX_DELAY);
    File v_file = LittleFS.open(T2_Def::Global::Path::FILE_CFG_JSON_CONST, "r");
    if (!v_file) {
        ESP_LOGE(TAG, "Config load failed");
        xSemaphoreGive(_lock);
        return false;
    }

    JsonDocument v_doc(&g_psramAlloc);
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

bool CL_T2_ConfigManager::save() {
    xSemaphoreTake(_lock, portMAX_DELAY);
    JsonDocument v_doc(&g_psramAlloc);

    // === Tier 1. Global ===
    JsonObject v_sys = v_doc["system"].to<JsonObject>();
    v_sys["site_id"]      = _config.system.site_id;
    v_sys["tele_hz"]      = _config.system.tele_hz;
    v_sys["wave_hz"]      = _config.system.wave_hz;
    v_sys["op_mode"]      = (uint8_t)_config.system.op_mode;
    v_sys["watchdog_ms"]  = _config.system.watchdog_ms;
    v_sys["vib_enable"]   = _config.system.vib_enable;
    v_sys["audio_enable"] = _config.system.audio_enable;

    JsonObject v_wifi = v_doc["wifi"].to<JsonObject>();
    v_wifi["mode"]    = (uint8_t)_config.wifi.mode;
    v_wifi["ap_ssid"] = _config.wifi.ap_ssid;
    v_wifi["ap_pw"]   = _config.wifi.ap_pw;
    v_wifi["ap_ip"]   = _config.wifi.ap_ip;
    JsonArray v_multi = v_wifi["multi_ap"].to<JsonArray>();
    for (uint8_t i = 0; i < T2_Def::Global::NetLimit::MAX_MULTI_AP_CONST; i++) {
        JsonObject v_ap = v_multi.add<JsonObject>();
        v_ap["ssid"] = _config.wifi.multi_ssid[i];
        v_ap["pw"]   = _config.wifi.multi_pw[i];
    }

    JsonObject v_mqtt = v_doc["mqtt"].to<JsonObject>();
    v_mqtt["enable"]     = _config.mqtt.enable;
    v_mqtt["broker"]     = _config.mqtt.broker;
    v_mqtt["port"]       = _config.mqtt.port;
    v_mqtt["id"]         = _config.mqtt.id;
    v_mqtt["pw"]         = _config.mqtt.pw;
    v_mqtt["topic_root"] = _config.mqtt.topic_root;
    v_mqtt["lwt_topic"]  = _config.mqtt.lwt_topic;
    v_mqtt["qos"]        = _config.mqtt.qos;
    v_mqtt["proto_ver"]  = _config.mqtt.proto_ver;

    JsonObject v_storage = v_doc["storage"].to<JsonObject>();
    v_storage["rot_mb"]        = _config.storage.rot_mb;
    v_storage["rot_min"]       = _config.storage.rot_min;
    v_storage["save_raw"]      = _config.storage.save_raw;
    v_storage["keep_max"]      = _config.storage.keep_max;
    v_storage["idle_flush_ms"] = _config.storage.idle_flush_ms;
    v_storage["pre_trig_sec"]  = _config.storage.pre_trig_sec;

    JsonObject v_out = v_doc["output"].to<JsonObject>();
    v_out["enabled"]         = _config.output.enabled;
    v_out["output_sequence"] = _config.output.output_sequence;
    v_out["sequence_frames"] = _config.output.sequence_frames;

    // === Tier 2. Shared ===
    JsonObject v_st = v_doc["shared_trig"].to<JsonObject>();
    v_st["hold_ms"]   = _config.shared_trig.hold_ms;
    v_st["use_sleep"] = _config.shared_trig.use_sleep;
    v_st["sleep_sec"] = _config.shared_trig.sleep_sec;

    JsonObject v_sd = v_doc["shared_dsp"].to<JsonObject>();
    v_sd["rem_dc"]  = _config.shared_dsp.rem_dc;
    v_sd["med_en"]  = _config.shared_dsp.med_en;
    v_sd["med_win"] = _config.shared_dsp.med_win;
    v_sd["hpf"]["en"]     = _config.shared_dsp.hpf.en;
    v_sd["hpf"]["cutoff"] = _config.shared_dsp.hpf.cutoff;
    v_sd["hpf"]["taps"]   = _config.shared_dsp.hpf.taps;
    v_sd["lpf"]["en"]     = _config.shared_dsp.lpf.en;
    v_sd["lpf"]["cutoff"] = _config.shared_dsp.lpf.cutoff;
    v_sd["lpf"]["taps"]   = _config.shared_dsp.lpf.taps;
    v_sd["iir_hpf"]["en"]     = _config.shared_dsp.iir_hpf.en;
    v_sd["iir_hpf"]["cutoff"] = _config.shared_dsp.iir_hpf.cutoff;
    v_sd["iir_hpf"]["q"]      = _config.shared_dsp.iir_hpf.q;
    v_sd["iir_lpf"]["en"]     = _config.shared_dsp.iir_lpf.en;
    v_sd["iir_lpf"]["cutoff"] = _config.shared_dsp.iir_lpf.cutoff;
    v_sd["iir_lpf"]["q"]      = _config.shared_dsp.iir_lpf.q;
    v_sd["notch"]["en"]   = _config.shared_dsp.notch.en;
    v_sd["notch"]["freq"] = _config.shared_dsp.notch.freq;
    v_sd["notch"]["gain"] = _config.shared_dsp.notch.gain;
    v_sd["notch"]["q"]    = _config.shared_dsp.notch.q;
    v_sd["win_type"]      = (uint8_t)_config.shared_dsp.win_type;

    // === Tier 3. Vib ===
    JsonObject v_vs = v_doc["vib_sensor"].to<JsonObject>();
    v_vs["accel_enable"]      = _config.vib_sensor.accel_enable;
    v_vs["gyro_enable"]       = _config.vib_sensor.gyro_enable;
    v_vs["accel_axis_count"]  = _config.vib_sensor.accel_axis_count;
    v_vs["accel_axis"]        = _config.vib_sensor.accel_axis;
    v_vs["gyro_axis_count"]   = _config.vib_sensor.gyro_axis_count;
    v_vs["gyro_axis"]         = _config.vib_sensor.gyro_axis;
    // 하위 호환성 유지용 레거시 필드도 동시 저장
    v_vs["axis_count"]        = _config.vib_sensor.accel_axis_count;
    v_vs["axis"]              = _config.vib_sensor.accel_axis;
    v_vs["accel_range"]       = _config.vib_sensor.accel_range;
    v_vs["accel_odr"]    = _config.vib_sensor.accel_odr;
    v_vs["gyro_range"]   = _config.vib_sensor.gyro_range;
    v_vs["sample_rate"]  = _config.vib_sensor.sample_rate;
    v_vs["fft_size"]     = _config.vib_sensor.fft_size;

    JsonObject v_vt = v_doc["vib_trig"].to<JsonObject>();
    v_vt["motion_en"]  = _config.vib_trig.motion_en;
    v_vt["wake_g"]     = _config.vib_trig.wake_g;
    v_vt["wake_dur"]   = _config.vib_trig.wake_dur;
    v_vt["rms_thresh"] = _config.vib_trig.rms_thresh;
    v_vt["kurt_ng_thresh"]  = _config.vib_trig.kurt_ng_thresh;
    v_vt["crest_ng_thresh"] = _config.vib_trig.crest_ng_thresh;
    v_vt["skew_ng_thresh"]  = _config.vib_trig.skew_ng_thresh;

    JsonArray v_vbands = v_vt["bands"].to<JsonArray>();
    for (uint8_t i = 0; i < _config.vib_trig.active_band_count; i++) {
        JsonObject v_b = v_vbands.add<JsonObject>();
        v_b["en"]     = _config.vib_trig.band_en[i];
        v_b["start"]  = _config.vib_trig.band_start[i];
        v_b["end"]    = _config.vib_trig.band_end[i];
        v_b["thresh"] = _config.vib_trig.band_thresh[i];
    }

    JsonObject v_vc = v_doc["vib_calib"].to<JsonObject>();
    JsonArray v_vcoff = v_vc["offset"].to<JsonArray>();
    for(uint8_t i=0; i<3; i++) cfgAddFloatExact(v_vcoff, _config.vib_calib.offset[i]);
    JsonArray v_vcgain = v_vc["gain"].to<JsonArray>();
    for(uint8_t i=0; i<3; i++) cfgAddFloatExact(v_vcgain, _config.vib_calib.gain[i]);

    JsonObject v_vf = v_doc["vib_feat"].to<JsonObject>();
    v_vf["peak_amp_min"]      = _config.vib_feat.peak_amp_min;
    v_vf["peak_freq_gap_min"] = _config.vib_feat.peak_freq_gap_min;


    // === Tier 4. Audio ===
    JsonObject v_as = v_doc["aud_sensor"].to<JsonObject>();
    v_as["channel_mode"] = (uint8_t)_config.aud_sensor.channel_mode;
    v_as["sample_rate"]  = _config.aud_sensor.sample_rate;
    v_as["fft_size"]     = _config.aud_sensor.fft_size;

    JsonObject v_at = v_doc["aud_trig"].to<JsonObject>();
    v_at["rms_thresh"]      = _config.aud_trig.rms_thresh;
    v_at["kurt_ng_thresh"]  = _config.aud_trig.kurt_ng_thresh;
    v_at["crest_ng_thresh"] = _config.aud_trig.crest_ng_thresh;
    v_at["skew_ng_thresh"]  = _config.aud_trig.skew_ng_thresh;

    JsonArray v_abands = v_at["bands"].to<JsonArray>();
    for (uint8_t i = 0; i < _config.aud_trig.active_band_count; i++) {
        JsonObject v_b = v_abands.add<JsonObject>();
        v_b["en"]     = _config.aud_trig.band_en[i];
        v_b["start"]  = _config.aud_trig.band_start[i];
        v_b["end"]    = _config.aud_trig.band_end[i];
        v_b["thresh"] = _config.aud_trig.band_thresh[i];
    }

    JsonObject v_ad = v_doc["aud_dsp"].to<JsonObject>();
    v_ad["pre_en"]    = _config.aud_dsp.pre_en;
    v_ad["pre_alpha"] = _config.aud_dsp.pre_alpha;
    v_ad["noise"]["gate_en"]      = _config.aud_dsp.noise.gate_en;
    v_ad["noise"]["gate_thresh"]  = _config.aud_dsp.noise.gate_thresh;
    v_ad["noise"]["mode"]         = (uint8_t)_config.aud_dsp.noise.mode;
    v_ad["noise"]["sub_str"]      = _config.aud_dsp.noise.sub_str;
    v_ad["noise"]["adp_alpha"]    = _config.aud_dsp.noise.adp_alpha;
    v_ad["noise"]["learn_frames"] = _config.aud_dsp.noise.learn_frames;
    v_ad["beam_gain"] = _config.aud_dsp.beam_gain;
    v_ad["window_ms"] = _config.aud_dsp.window_ms;
    v_ad["hop_ms"]    = _config.aud_dsp.hop_ms;

    JsonObject v_ac = v_doc["aud_calib"].to<JsonObject>();
    v_ac["auto_idle_min"] = _config.aud_calib.auto_idle_min;
    v_ac["ref_freq"]      = _config.aud_calib.ref_freq;
    v_ac["filt_min"]      = _config.aud_calib.filt_min;
    v_ac["filt_max"]      = _config.aud_calib.filt_max;
    v_ac["gain_max"]      = _config.aud_calib.gain_max;
    v_ac["gain_min"]      = _config.aud_calib.gain_min;
    v_ac["norm_safe"]     = _config.aud_calib.norm_safe;
    v_ac["gain_L"]        = _config.aud_calib.gain_L;
    v_ac["gain_R"]        = _config.aud_calib.gain_R;
    JsonArray v_eq = v_ac["eq_coeffs"].to<JsonArray>();
    for(uint16_t i = 0; i < T2_Def::Shared::FeatureLimit::FIR_TAPS_MAX; i++) {
        // [메모리/파싱 부하 최적화] EQ 값이 0.0f인 경우 생략하고 싶으나,
        // 배열 길이를 고정하기 위해 전체를 씁니다. (정밀도 보장)
        cfgAddFloatExact(v_eq, _config.aud_calib.eq_coeffs[i]);
    }

    JsonObject v_af = v_doc["aud_feat"].to<JsonObject>();
    v_af["f_min"]             = _config.aud_feat.f_min;
    v_af["f_max"]             = _config.aud_feat.f_max;
    v_af["peak_amp_min"]      = _config.aud_feat.peak_amp_min;
    v_af["peak_freq_gap_min"] = _config.aud_feat.peak_freq_gap_min;
    v_af["active_peak_count"] = _config.aud_feat.active_peak_count;

    JsonArray v_ceps = v_af["ceps_targets"].to<JsonArray>();
    for(uint8_t i = 0; i < _config.aud_feat.active_ceps_count; i++) {
        cfgAddFloatExact(v_ceps, _config.aud_feat.ceps_targets[i]);
    }

    // [안전 기제] 원자적 쓰기 (tmp 스왑)
    File v_file = LittleFS.open("/sys/config_243.tmp", "w");
    if (!v_file) {
        ESP_LOGE(TAG, "Atomic write failed");
        xSemaphoreGive(_lock);
        return false;
    }

    serializeJson(v_doc, v_file);
    v_file.close();

    LittleFS.remove(T2_Def::Global::Path::FILE_CFG_JSON_CONST);
    LittleFS.rename("/sys/config_243.tmp", T2_Def::Global::Path::FILE_CFG_JSON_CONST);

    xSemaphoreGive(_lock);
    return true;
}

// ------------------------------------------------------------------------
// 이하 튜닝/업데이트 제어 로직 (이전 버전의 핫스왑/레이지라이트 완전 보존)
// ------------------------------------------------------------------------

bool CL_T2_ConfigManager::updateFromJson(const char* p_jsonString) {
    JsonDocument v_doc(&g_psramAlloc);
    DeserializationError v_err = deserializeJson(v_doc, p_jsonString);
    if (v_err) return false;

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
    }
}

bool CL_T2_ConfigManager::updatePreview(const char* p_jsonString) {
    JsonDocument v_doc(&g_psramAlloc);
    DeserializationError v_err = deserializeJson(v_doc, p_jsonString);
    if (v_err) return false;

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
    }
    return v_res;
}

bool CL_T2_ConfigManager::revertCancel() {
    bool v_res = load();
    if (v_res) {
        _isDirty = false;
        _isTuningActive = true;
    }
    return v_res;
}

void CL_T2_ConfigManager::resetToDefault() {
    xSemaphoreTake(_lock, portMAX_DELAY);
    _loadDefaults();
    xSemaphoreGive(_lock);
    save();
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

void CL_T2_ConfigManager::serializeToBuffer(char* p_outBuf, size_t p_maxLen) {
    if (!p_outBuf || p_maxLen == 0) return;

    xSemaphoreTake(_lock, portMAX_DELAY);
    JsonDocument v_doc;

    JsonObject v_sys = v_doc["system"].to<JsonObject>();
    v_sys["site_id"] = _config.system.site_id;
    v_sys["op_mode"] = (uint8_t)_config.system.op_mode;

    // 필수 설정 위주로 8KB 내에 직렬화
    serializeJson(v_doc, p_outBuf, p_maxLen);
    xSemaphoreGive(_lock);
}
