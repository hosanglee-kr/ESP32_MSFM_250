/* ============================================================================
 * File: T220_CfgMgr_245.cpp
 * Summary: 4-Tier 동적 JSON 설정 관리자 구현부
 * ============================================================================ */

#include "T220_CfgMgr_245.hpp"
#include <LittleFS.h>
#include <cstring>
#include "esp_log.h"

static const char* TAG = "T245_CFG";

// [이슈 11] g_psramAlloc 제거 — 10KB 미만 JSON을 PSRAM에 담으면 캐시 미스 오버헤드가 발생함. 기본 내부 힙 할당(Internal SRAM) 사용.

/**
 * @brief 정밀도 손실 없는 float 팩터 추가 헬퍼
 * @details 기존 snprintf를 통한 고정 소수점 포맷 변환 연산을 제거하고 ArduinoJson의 효율적인 기본 float 변환을 활용합니다.
 * @param p_arr 대상 JsonArray
 * @param p_val 기입할 float 값
 */
static inline void cfgAddFloatExact(JsonArray& p_arr, float p_val) {
    p_arr.add(p_val);
}

/**
 * @brief Null 체크 및 타입 안전 값 할당 템플릿
 * @param p_v JsonVariantConst 소스 값
 * @param p_dest 목적지 변퍼 레퍼런스
 */
template<typename T>
static inline void cfgSet(JsonVariantConst p_v, T& p_dest) {
    if (!p_v.isNull()) p_dest = p_v.as<T>();
}

/**
 * @brief 문자열 복사 및 버퍼 오버플로우 방어 헬퍼
 * @param p_v JsonVariantConst 소스 문자열
 * @param p_dest 목적지 char 배열 포인터
 * @param p_size 목적지 버퍼 바이트 크기
 */
static inline void cfgStr(JsonVariantConst p_v, char* p_dest, size_t p_size) {
    if (!p_v.isNull()) {
        const char* v_src = p_v.as<const char*>();
        if (v_src) strlcpy(p_dest, v_src, p_size);
    }
}

/**
 * @brief 배열 및 단일형 멀티 대응 설정 템플릿
 * @param p_v JsonVariantConst 소스 variant
 * @param p_dest 대상 배열 레퍼런스
 */
template<typename T, size_t N>
static inline void cfgSetArray(JsonVariantConst p_v, T (&p_dest)[N]) {
    if (p_v.isNull()) return;
    if (p_v.is<JsonArrayConst>()) {
        JsonArrayConst v_arr = p_v.as<JsonArrayConst>();
        size_t i = 0;
        for (JsonVariantConst v_item : v_arr) {
            if (i >= N) break;
            p_dest[i] = v_item.as<T>();
            i++;
        }
    } else {
        T v_val = p_v.as<T>();
        for (size_t i = 0; i < N; i++) p_dest[i] = v_val;
    }
}

/**
 * @brief DSP 파이프라인 개별 JSON 서브 오브젝트 파싱
 * @param p_obj DSP 설정 JSON 오브젝트
 * @param p_dsp DSP 설정 저장 대상 구조체 레퍼런스
 */
static void parseDspConfig(JsonObjectConst p_obj, T2_Type::ST_Dsp_Config_t& p_dsp) {
    if (p_obj.isNull()) return;
    cfgSet(p_obj["rem_dc"],  p_dsp.rem_dc);
    cfgSet(p_obj["med_en"],  p_dsp.med_en);
    cfgSet(p_obj["med_win"], p_dsp.med_win);

    JsonObjectConst v_hpf = p_obj["hpf"];
    if (!v_hpf.isNull()) {
        cfgSet(v_hpf["en"],     p_dsp.hpf.en);
        cfgSet(v_hpf["cutoff"], p_dsp.hpf.cutoff);
        cfgSet(v_hpf["taps"],   p_dsp.hpf.taps);
    }
    JsonObjectConst v_lpf = p_obj["lpf"];
    if (!v_lpf.isNull()) {
        cfgSet(v_lpf["en"],     p_dsp.lpf.en);
        cfgSet(v_lpf["cutoff"], p_dsp.lpf.cutoff);
        cfgSet(v_lpf["taps"],   p_dsp.lpf.taps);
    }
    JsonObjectConst v_ihpf = p_obj["iir_hpf"];
    if (!v_ihpf.isNull()) {
        cfgSet(v_ihpf["en"],     p_dsp.iir_hpf.en);
        cfgSet(v_ihpf["cutoff"], p_dsp.iir_hpf.cutoff);
        cfgSet(v_ihpf["q"],      p_dsp.iir_hpf.q);
    }
    JsonObjectConst v_ilpf = p_obj["iir_lpf"];
    if (!v_ilpf.isNull()) {
        cfgSet(v_ilpf["en"],     p_dsp.iir_lpf.en);
        cfgSet(v_ilpf["cutoff"], p_dsp.iir_lpf.cutoff);
        cfgSet(v_ilpf["q"],      p_dsp.iir_lpf.q);
    }
    JsonObjectConst v_notch = p_obj["notch"];
    if (!v_notch.isNull()) {
        cfgSet(v_notch["en"],   p_dsp.notch.en);
        cfgSet(v_notch["freq"], p_dsp.notch.freq);
        cfgSet(v_notch["gain"], p_dsp.notch.gain);
        cfgSet(v_notch["q"],    p_dsp.notch.q);
    }
    JsonObjectConst v_notch2 = p_obj["notch2"];
    if (!v_notch2.isNull()) {
        cfgSet(v_notch2["en"],   p_dsp.notch2.en);
        cfgSet(v_notch2["freq"], p_dsp.notch2.freq);
        cfgSet(v_notch2["gain"], p_dsp.notch2.gain);
        cfgSet(v_notch2["q"],    p_dsp.notch2.q);
    }
    if (!p_obj["win_type"].isNull()) {
        p_dsp.win_type = (T2_Type::EM_WindowType_t)p_obj["win_type"].as<uint8_t>();
    }
}

/**
 * @brief DSP 파이프라인 개별 JSON 서브 오브젝트 직렬화
 * @param p_obj DSP JSON 오브젝트
 * @param p_dsp DSP 설정 대상 구조체 레퍼런스
 */
static void serializeDspConfig(JsonObject p_obj, const T2_Type::ST_Dsp_Config_t& p_dsp) {
    p_obj["rem_dc"]  = p_dsp.rem_dc;
    p_obj["med_en"]  = p_dsp.med_en;
    p_obj["med_win"] = p_dsp.med_win;

    JsonObject v_hpf = p_obj["hpf"].to<JsonObject>();
    v_hpf["en"]     = p_dsp.hpf.en;
    v_hpf["cutoff"] = p_dsp.hpf.cutoff;
    v_hpf["taps"]   = p_dsp.hpf.taps;

    JsonObject v_lpf = p_obj["lpf"].to<JsonObject>();
    v_lpf["en"]     = p_dsp.lpf.en;
    v_lpf["cutoff"] = p_dsp.lpf.cutoff;
    v_lpf["taps"]   = p_dsp.lpf.taps;

    JsonObject v_ihpf = p_obj["iir_hpf"].to<JsonObject>();
    v_ihpf["en"]     = p_dsp.iir_hpf.en;
    v_ihpf["cutoff"] = p_dsp.iir_hpf.cutoff;
    v_ihpf["q"]      = p_dsp.iir_hpf.q;

    JsonObject v_ilpf = p_obj["iir_lpf"].to<JsonObject>();
    v_ilpf["en"]     = p_dsp.iir_lpf.en;
    v_ilpf["cutoff"] = p_dsp.iir_lpf.cutoff;
    v_ilpf["q"]      = p_dsp.iir_lpf.q;

    JsonObject v_notch = p_obj["notch"].to<JsonObject>();
    v_notch["en"]   = p_dsp.notch.en;
    v_notch["freq"] = p_dsp.notch.freq;
    v_notch["gain"] = p_dsp.notch.gain;
    v_notch["q"]    = p_dsp.notch.q;

    JsonObject v_notch2 = p_obj["notch2"].to<JsonObject>();
    v_notch2["en"]   = p_dsp.notch2.en;
    v_notch2["freq"] = p_dsp.notch2.freq;
    v_notch2["gain"] = p_dsp.notch2.gain;
    v_notch2["q"]    = p_dsp.notch2.q;

    p_obj["win_type"] = (uint8_t)p_dsp.win_type;
}

/**
 * @brief CL_T2_ConfigManager 생성자
 * @details Mutex 락 초기화 및 기본값 적재 수행
 */
CL_T2_ConfigManager::CL_T2_ConfigManager() {
    _lock           = xSemaphoreCreateMutex();
    _isLoaded       = false;
    _isDirty        = false;
    _isTuningActive = false;
    _lastModifiedMs = 0;
    _loadDefaults();
}

/**
 * @brief CL_T2_ConfigManager 소멸자
 * @details Mutex 락 리소스 정리
 */
CL_T2_ConfigManager::~CL_T2_ConfigManager() {
    if (_lock) vSemaphoreDelete(_lock);
}

/**
 * @brief 설정 관리자 초기화
 * @details LittleFS 파일 시스템을 마운트하고 설정 파일 존재 여부를 검사하여 복구 혹은 기본값 세팅을 로드
 * @return 초기화 성공 여부
 */
bool CL_T2_ConfigManager::init() {
    // [이슈 4] LittleFS.begin(false): 마운트 실패 시 자동 포맷 금지. 데이터 보호를 위해 에러 리턴 후 수동 복구 유도.
    if (!LittleFS.begin(false)) {
        ESP_LOGE(TAG, "LittleFS Mount Failed! Format may be needed. Aborting to protect data.");
        return false;
    }

    // [처리 단위 1] 백업 임시 파일 복구 검사 및 로딩 처리
    if (!LittleFS.exists(T2_Def::Global::Path::FILE_CFG_JSON_CONST)) {
        if (LittleFS.exists("/sys/config_245.tmp")) {
            ESP_LOGW(TAG, "Recovering config from interrupted atomic write");
            LittleFS.rename("/sys/config_245.tmp", T2_Def::Global::Path::FILE_CFG_JSON_CONST);
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

/**
 * @brief 4-Tier 시스템 설정 기본값 로드
 * @details 시스템 동작, 네트워크(Wi-Fi, MQTT, NTP), 저장소, 판정 알고리즘, 센서 및 DSP 파라미터들의 초기 기본 설정값을 구성
 */
void CL_T2_ConfigManager::_loadDefaults() {
    // === Tier 1. Global ===
    strlcpy(_config.system.site_id, T2_Def::Global::System::SITE_ID_DEF, sizeof(_config.system.site_id));
    _config.system.tele_hz      = T2_Def::Global::System::TELEMETRY_HZ_DEF;
    _config.system.wave_hz      = T2_Def::Global::System::WAVEFORM_HZ_DEF;
    _config.system.op_mode      = T2_Type::EM_OpMode_t::AUTO;
    _config.system.watchdog_ms  = T2_Def::Global::Task::WDG_TIMEOUT_MS_DEF;

    _config.wifi.mode = T2_Type::EM_WiFiMode_t::AUTO_FALLBACK;
    strlcpy(_config.wifi.ap_ssid, T2_Def::Global::Net::WIFI_AP_SSID_DEF, sizeof(_config.wifi.ap_ssid));
    strlcpy(_config.wifi.ap_pw,   T2_Def::Global::Net::WIFI_AP_PW_DEF,   sizeof(_config.wifi.ap_pw));
    strlcpy(_config.wifi.ap_ip,   T2_Def::Global::Net::WIFI_AP_IP_DEF,   sizeof(_config.wifi.ap_ip));
    for (uint8_t i = 0; i < T2_Def::Global::NetLimit::NET_MULTI_AP_MAX; i++) {
        _config.wifi.multi_ssid[i][0] = '\0';
        _config.wifi.multi_pw[i][0]   = '\0';
    }
    _config.wifi.disconnect_delay_ms = T2_Def::Global::NetLimit::NET_WIFI_DISCONNECT_DELAY_MS_DEF;  // [이슈 9] WiFi 연결 유지 딜레이 설정

    _config.mqtt.enable = false;
    strlcpy(_config.mqtt.broker, "", sizeof(_config.mqtt.broker));
    _config.mqtt.port = T2_Def::Global::Net::MQTT_PORT_DEF;
    strlcpy(_config.mqtt.id, T2_Def::Global::Net::MQTT_ID_DEF, sizeof(_config.mqtt.id));
    strlcpy(_config.mqtt.pw, "", sizeof(_config.mqtt.pw));
    strlcpy(_config.mqtt.topic_root, T2_Def::Global::Net::MQTT_TOPIC_DEF, sizeof(_config.mqtt.topic_root));
    strlcpy(_config.mqtt.lwt_topic, T2_Def::Global::Net::MQTT_LWT_DEF, sizeof(_config.mqtt.lwt_topic));
    _config.mqtt.qos        = T2_Def::Global::Net::MQTT_QOS_DEF;
    _config.mqtt.proto_ver  = T2_Def::Global::Net::MQTT_PROTO_VER_DEF;
    
    // [이슈 9] NTP 타임서버 기본값 초기화
    strlcpy(_config.mqtt.ntp_server1, T2_Def::Global::Net::NTP_SERVER_1_CONST, sizeof(_config.mqtt.ntp_server1));
    strlcpy(_config.mqtt.ntp_server2, T2_Def::Global::Net::NTP_SERVER_2_CONST, sizeof(_config.mqtt.ntp_server2));
    strlcpy(_config.mqtt.ntp_tz,      T2_Def::Global::Net::NTP_TZ_INFO_CONST,  sizeof(_config.mqtt.ntp_tz));

    _config.storage.rot_mb       = T2_Def::Global::Storage::ROTATE_MB_DEF;
    _config.storage.rot_min      = T2_Def::Global::Storage::ROTATE_MIN_DEF;
    _config.storage.save_raw     = false;
    _config.storage.keep_max     = T2_Def::Global::Storage::ROTATE_KEEP_MAX_DEF;
    _config.storage.idle_flush_ms= T2_Def::Global::Storage::IDLE_FLUSH_MS_DEF;
    _config.storage.pre_trig_sec = T2_Def::Global::Storage::PRE_TRIGGER_SEC_DEF;

    _config.output.enabled         = true;
    _config.output.output_sequence  = false;
    _config.output.sequence_frames  = T2_Def::Global::System::SEQUENCE_FRAMES_DEF;

    // [이슈 8] Decision 파라미터 기본값
    _config.decision.max_trial_count   = T2_Def::Global::Decision::MAX_TRIAL_COUNT_DEF;
    _config.decision.sta_lta_threshold = T2_Def::Global::Decision::STA_LTA_THRESHOLD_DEF;
    _config.decision.min_trigger_count = T2_Def::Global::Decision::MIN_TRIGGER_COUNT_DEF;
    _config.decision.valid_start_sec   = T2_Def::Global::Decision::VALID_START_SEC_DEF;
    _config.decision.valid_end_sec     = T2_Def::Global::Decision::VALID_END_SEC_DEF;

    // === Global Trigger & Dsp ===
    _config.trig_hold_ms   = T2_Def::Global::Trigger::HOLD_TIME_MS_DEF;
    _config.trig_use_sleep = T2_Def::Global::Trigger::USE_DEEP_SLEEP_DEF;
    _config.trig_sleep_sec = T2_Def::Global::Trigger::SLEEP_SEC_DEF;

    // === Tier 2. Accel ===
    _config.accel.enable            = T2_Def::Accel::Sensor::ENABLE_DEF;
    _config.accel.axis_mask         = T2_Def::Accel::Sensor::AXIS_MASK_DEF;
    _config.accel.range             = T2_Def::Accel::Sensor::RANGE_DEF;
    _config.accel.odr               = T2_Def::Accel::Sensor::ODR_REG_DEF;
    _config.accel.bwp               = T2_Def::Accel::Sensor::BWP_REG_DEF;
    _config.accel.filter_perf       = T2_Def::Accel::Sensor::PERF_MODE_DEF;
    _config.accel.fifo_watermark    = T2_Def::Imu::Hardware::FIFO_WATERMARK_LIMIT;
    _config.accel.sample_rate       = T2_Def::Accel::Sensor::RATE_DEF;
    _config.accel.fft_size          = T2_Def::Accel::Sensor::FFT_SIZE_DEF;

    _config.accel.motion_en         = false;
    _config.accel.wake_g            = T2_Def::Accel::Trigger::WAKE_THRESH_G_DEF;
    _config.accel.wake_dur          = T2_Def::Global::Trigger::WAKE_DURATION_DEF;

    for (int i = 0; i < T2_Def::Accel::Sensor::AXIS_MAX; i++) {
        _config.accel.rms_thresh[i]      = T2_Def::Accel::Trigger::RMS_THRESH_DEF;
        _config.accel.kurt_ng_thresh[i]  = T2_Def::Accel::Trigger::KURT_NG_THRESH_DEF;
        _config.accel.crest_ng_thresh[i] = T2_Def::Accel::Trigger::CREST_NG_THRESH_DEF;
        _config.accel.skew_ng_thresh[i]  = T2_Def::Accel::Trigger::SKEW_NG_THRESH_DEF;
        _config.accel.offset[i]          = 0.0f;
        _config.accel.gain[i]            = 1.0f;
    }

    _config.accel.active_band_count = T2_Def::Accel::FeatureLimit::BAND_DEF;
    for (uint8_t i = 0; i < T2_Def::Accel::FeatureLimit::BAND_MAX; i++) {
        _config.accel.band_en[i] = false;
        if (i < T2_Def::Accel::FeatureLimit::BAND_DEF) {
            _config.accel.band_start[i] = T2_Def::Accel::Trigger::BAND_RANGES_DEF[i][0];
            _config.accel.band_end[i]   = T2_Def::Accel::Trigger::BAND_RANGES_DEF[i][1];
        } else {
            _config.accel.band_start[i] = 0.0f;
            _config.accel.band_end[i]   = 0.0f;
        }
        for (int a = 0; a < T2_Def::Accel::Sensor::AXIS_MAX; a++) {
            _config.accel.band_thresh[a][i] = 1.0f;
        }
    }
    _config.accel.peak_amp_min      = 0.1f;
    _config.accel.peak_freq_gap_min = 50.0f;

    // Accel DSP Default
    _config.accel.dsp.rem_dc        = true;
    _config.accel.dsp.med_en        = true;
    _config.accel.dsp.med_win       = 3;
    _config.accel.dsp.hpf.en        = true;
    _config.accel.dsp.hpf.cutoff    = T2_Def::Accel::Dsp::HPF_CUTOFF_DEF;
    _config.accel.dsp.hpf.taps      = T2_Def::Accel::FeatureLimit::FIR_TAPS_DEF;
    _config.accel.dsp.lpf.en        = false;
    _config.accel.dsp.lpf.cutoff    = T2_Def::Accel::Dsp::LPF_CUTOFF_DEF;
    _config.accel.dsp.lpf.taps      = T2_Def::Accel::FeatureLimit::FIR_TAPS_DEF;
    _config.accel.dsp.iir_hpf.en    = false;
    _config.accel.dsp.iir_hpf.cutoff= 20.0f;
    _config.accel.dsp.iir_hpf.q     = 0.707f;
    _config.accel.dsp.iir_lpf.en    = false;
    _config.accel.dsp.iir_lpf.cutoff= 1000.0f;
    _config.accel.dsp.iir_lpf.q     = 0.707f;
    _config.accel.dsp.notch.en      = false;
    _config.accel.dsp.notch.freq    = T2_Def::Accel::Dsp::NOTCH_FREQ_DEF;
    _config.accel.dsp.notch.gain    = 1.0f;
    _config.accel.dsp.notch.q       = T2_Def::Accel::Dsp::NOTCH_Q_DEF;
    _config.accel.dsp.notch2.en      = false;
    _config.accel.dsp.notch2.freq    = T2_Def::Accel::Dsp::NOTCH2_FREQ_DEF;
    _config.accel.dsp.notch2.gain    = 1.0f;
    _config.accel.dsp.notch2.q       = T2_Def::Accel::Dsp::NOTCH_Q_DEF;
    _config.accel.dsp.win_type      = T2_Type::EM_WindowType_t::HANN;

    // === Tier 3. Gyro ===
    _config.gyro.enable             = T2_Def::Gyro::Sensor::ENABLE_DEF;
    _config.gyro.axis_mask          = T2_Def::Gyro::Sensor::AXIS_MASK_DEF;
    _config.gyro.range              = T2_Def::Gyro::Sensor::RANGE_DEF;
    _config.gyro.odr                = T2_Def::Gyro::Sensor::ODR_REG_DEF;
    _config.gyro.bwp                = T2_Def::Gyro::Sensor::BWP_REG_DEF;
    _config.gyro.filter_perf        = T2_Def::Gyro::Sensor::PERF_MODE_DEF;
    _config.gyro.noise_perf         = T2_Def::Gyro::Sensor::PERF_MODE_DEF;
    _config.gyro.sample_rate        = T2_Def::Gyro::Sensor::RATE_DEF;
    _config.gyro.fft_size           = T2_Def::Gyro::Sensor::FFT_SIZE_DEF;

    for (int i = 0; i < T2_Def::Gyro::Sensor::AXIS_MAX; i++) {
        _config.gyro.rms_thresh[i]      = T2_Def::Gyro::Trigger::RMS_THRESH_DEF;
        _config.gyro.kurt_ng_thresh[i]  = T2_Def::Gyro::Trigger::KURT_NG_THRESH_DEF;
        _config.gyro.crest_ng_thresh[i] = T2_Def::Gyro::Trigger::CREST_NG_THRESH_DEF;
        _config.gyro.skew_ng_thresh[i]  = T2_Def::Gyro::Trigger::SKEW_NG_THRESH_DEF;
        _config.gyro.offset[i]          = 0.0f;
        _config.gyro.gain[i]            = 1.0f;
    }

    _config.gyro.active_band_count = T2_Def::Gyro::FeatureLimit::BAND_DEF;
    for (uint8_t i = 0; i < T2_Def::Gyro::FeatureLimit::BAND_MAX; i++) {
        _config.gyro.band_en[i] = false;
        if (i < T2_Def::Gyro::FeatureLimit::BAND_DEF) {
            _config.gyro.band_start[i] = T2_Def::Gyro::Trigger::BAND_RANGES_DEF[i][0];
            _config.gyro.band_end[i]   = T2_Def::Gyro::Trigger::BAND_RANGES_DEF[i][1];
        } else {
            _config.gyro.band_start[i] = 0.0f;
            _config.gyro.band_end[i]   = 0.0f;
        }
        for (int a = 0; a < T2_Def::Gyro::Sensor::AXIS_MAX; a++) {
            _config.gyro.band_thresh[a][i] = 1.0f;
        }
    }
    _config.gyro.peak_amp_min      = 0.1f;
    _config.gyro.peak_freq_gap_min = 50.0f;

    // Gyro DSP Default
    _config.gyro.dsp.rem_dc         = true;
    _config.gyro.dsp.med_en         = true;
    _config.gyro.dsp.med_win        = 3;
    _config.gyro.dsp.hpf.en         = true;
    _config.gyro.dsp.hpf.cutoff     = T2_Def::Gyro::Dsp::HPF_CUTOFF_DEF;
    _config.gyro.dsp.hpf.taps       = T2_Def::Gyro::FeatureLimit::FIR_TAPS_DEF;
    _config.gyro.dsp.lpf.en         = false;
    _config.gyro.dsp.lpf.cutoff     = T2_Def::Gyro::Dsp::LPF_CUTOFF_DEF;
    _config.gyro.dsp.lpf.taps       = T2_Def::Gyro::FeatureLimit::FIR_TAPS_DEF;
    _config.gyro.dsp.iir_hpf.en     = false;
    _config.gyro.dsp.iir_hpf.cutoff = 20.0f;
    _config.gyro.dsp.iir_hpf.q      = 0.707f;
    _config.gyro.dsp.iir_lpf.en     = false;
    _config.gyro.dsp.iir_lpf.cutoff = 1000.0f;
    _config.gyro.dsp.iir_lpf.q      = 0.707f;
    _config.gyro.dsp.notch.en       = false;
    _config.gyro.dsp.notch.freq     = T2_Def::Gyro::Dsp::NOTCH_FREQ_DEF;
    _config.gyro.dsp.notch.gain     = 1.0f;
    _config.gyro.dsp.notch.q        = T2_Def::Gyro::Dsp::NOTCH_Q_DEF;
    _config.gyro.dsp.notch2.en       = false;
    _config.gyro.dsp.notch2.freq     = T2_Def::Gyro::Dsp::NOTCH2_FREQ_DEF;
    _config.gyro.dsp.notch2.gain     = 1.0f;
    _config.gyro.dsp.notch2.q        = T2_Def::Gyro::Dsp::NOTCH_Q_DEF;
    _config.gyro.dsp.win_type       = T2_Type::EM_WindowType_t::HANN;

    // === Tier 4. Audio ===
    _config.audio.enable            = T2_Def::Audio::Sensor::ENABLE_DEF;
    _config.audio.channel_mask      = T2_Def::Audio::Sensor::CHANNEL_MASK_DEF;
    _config.audio.sample_rate       = T2_Def::Audio::Sensor::RATE_DEF;
    _config.audio.fft_size          = T2_Def::Audio::Sensor::FFT_SIZE_DEF;
    _config.audio.mel_bands         = T2_Def::Audio::FeatureLimit::MEL_BANDS_DEF;

    for (int ch = 0; ch < T2_Def::Audio::Sensor::CHANNELS_MAX; ch++) {
        _config.audio.rms_thresh[ch]      = T2_Def::Audio::Trigger::RMS_THRESH_DEF;
        _config.audio.kurt_ng_thresh[ch]  = T2_Def::Audio::Trigger::KURT_NG_THRESH_DEF;
        _config.audio.crest_ng_thresh[ch] = T2_Def::Audio::Trigger::CREST_NG_THRESH_DEF;
        _config.audio.skew_ng_thresh[ch]  = T2_Def::Audio::Trigger::SKEW_NG_THRESH_DEF;
        _config.audio.gain_ch[ch]         = 1.0f;
        memset(_config.audio.eq_coeffs[ch], 0, sizeof(_config.audio.eq_coeffs[ch]));
        // [이슈 3] 센터 탭 인덱스를 FIR_TAPS_MAX/2(127)로 정정 (기존 FIR_TAPS_DEF/2=31 오류 수정)
        _config.audio.eq_coeffs[ch][T2_Def::Audio::FeatureLimit::FIR_TAPS_MAX / 2] = 1.0f;
    }

    _config.audio.active_band_count = T2_Def::Audio::FeatureLimit::BAND_DEF;
    for (uint8_t i = 0; i < T2_Def::Audio::FeatureLimit::BAND_MAX; i++) {
        _config.audio.band_en[i] = false;
        if (i < T2_Def::Audio::FeatureLimit::BAND_DEF) {
            _config.audio.band_start[i] = T2_Def::Audio::Trigger::BAND_RANGES_DEF[i][0];
            _config.audio.band_end[i]   = T2_Def::Audio::Trigger::BAND_RANGES_DEF[i][1];
        } else {
            _config.audio.band_start[i] = 0.0f;
            _config.audio.band_end[i]   = 0.0f;
        }
        for (int ch = 0; ch < T2_Def::Audio::Sensor::CHANNELS_MAX; ch++) {
            _config.audio.band_thresh[ch][i] = 1.0f;
        }
    }

    _config.audio.noise.gate_en      = false;
    _config.audio.noise.gate_thresh  = T2_Def::Audio::Dsp::NOISE_GATE_THRESH_DEF;
    _config.audio.noise.mode         = T2_Type::EM_NoiseMode_t::OFF;
    _config.audio.noise.sub_str      = T2_Def::Audio::Dsp::SPECTRAL_SUB_GAIN_DEF;
    _config.audio.noise.adp_alpha    = T2_Def::Audio::Dsp::NOISE_LEARN_ALPHA_DEF;
    _config.audio.noise.learn_frames = 100;

    _config.audio.pre_en             = true;
    _config.audio.pre_alpha          = T2_Def::Audio::Dsp::PRE_EMPHASIS_ALPHA_DEF;
    _config.audio.beam_gain          = T2_Def::Audio::Dsp::BEAMFORMING_GAIN_DEF;
    _config.audio.window_ms          = T2_Def::Audio::Dsp::WINDOW_MS_DEF;
    _config.audio.hop_ms             = T2_Def::Audio::Dsp::HOP_MS_DEF;

    _config.audio.auto_idle_min      = T2_Def::Audio::Calib::AUTO_IDLE_MIN_DEF;
    _config.audio.ref_freq           = T2_Def::Audio::Calib::REF_FREQ_HZ_DEF;
    _config.audio.filt_min           = T2_Def::Audio::Calib::FILTER_MIN_FREQ_HZ_DEF;
    _config.audio.filt_max           = T2_Def::Audio::Calib::FILTER_MAX_FREQ_HZ_DEF;
    _config.audio.gain_max           = T2_Def::Audio::Calib::TARGET_GAIN_MAX_DEF;
    _config.audio.gain_min           = T2_Def::Audio::Calib::TARGET_GAIN_MIN_DEF;
    _config.audio.norm_safe          = T2_Def::Audio::Calib::NORM_SAFE_THRESH_DEF;

    _config.audio.active_ceps_count  = T2_Def::Audio::FeatureLimit::CEPS_TARGET_DEF;
    _config.audio.active_peak_count  = T2_Def::Audio::FeatureLimit::TOP_PEAKS_DEF;
    for (uint8_t i = 0; i < T2_Def::Audio::FeatureLimit::CEPS_TARGET_MAX; i++) {
        _config.audio.ceps_targets[i] = 0.0f;
    }
    _config.audio.f_min              = 100.0f;
    _config.audio.f_max              = 8000.0f;
    _config.audio.peak_amp_min       = 0.1f;
    _config.audio.peak_freq_gap_min  = 50.0f;

    // Audio DSP Default
    _config.audio.dsp.rem_dc         = true;
    _config.audio.dsp.med_en         = true;
    _config.audio.dsp.med_win        = 3;
    _config.audio.dsp.hpf.en         = true;
    _config.audio.dsp.hpf.cutoff     = T2_Def::Audio::Dsp::HPF_CUTOFF_DEF;
    _config.audio.dsp.hpf.taps       = T2_Def::Audio::FeatureLimit::FIR_TAPS_DEF;
    _config.audio.dsp.lpf.en         = false;
    _config.audio.dsp.lpf.cutoff     = T2_Def::Audio::Dsp::LPF_CUTOFF_DEF;
    _config.audio.dsp.lpf.taps       = T2_Def::Audio::FeatureLimit::FIR_TAPS_DEF;
    _config.audio.dsp.iir_hpf.en     = false;
    _config.audio.dsp.iir_hpf.cutoff = 20.0f;
    _config.audio.dsp.iir_hpf.q      = 0.707f;
    _config.audio.dsp.iir_lpf.en     = false;
    _config.audio.dsp.iir_lpf.cutoff = 1000.0f;
    _config.audio.dsp.iir_lpf.q      = 0.707f;
    _config.audio.dsp.notch.en       = false;
    _config.audio.dsp.notch.freq     = T2_Def::Audio::Dsp::NOTCH_FREQ_DEF;
    _config.audio.dsp.notch.gain     = 1.0f;
    _config.audio.dsp.notch.q        = T2_Def::Audio::Dsp::NOTCH_Q_DEF;
    _config.audio.dsp.notch2.en       = false;
    _config.audio.dsp.notch2.freq     = T2_Def::Audio::Dsp::NOTCH2_FREQ_DEF;
    _config.audio.dsp.notch2.gain     = 1.0f;
    _config.audio.dsp.notch2.q        = T2_Def::Audio::Dsp::NOTCH_Q_DEF;
    _config.audio.dsp.win_type       = T2_Type::EM_WindowType_t::HANN;
}

/**
 * @brief 수신한 JSON 문서를 4-Tier 시스템 설정 구조체에 적용
 * @details JSON 각 필드를 검사하여 메모리 상의 _config 데이터에 원자적으로 반영하며, 주요 데이터 범위 한계 검증을 통과시킴
 * @param p_doc 파싱이 완료된 JsonDocument 레퍼런스
 */
void CL_T2_ConfigManager::_applyJson(const JsonDocument& p_doc) {
    // [처리 단위 1] system 파라미터 파싱
    JsonObjectConst v_sys = p_doc["system"];
    if (!v_sys.isNull()) {
        cfgStr(v_sys["site_id"], _config.system.site_id, sizeof(_config.system.site_id));
        cfgSet(v_sys["tele_hz"], _config.system.tele_hz);
        cfgSet(v_sys["wave_hz"], _config.system.wave_hz);
        if (!v_sys["op_mode"].isNull()) {
            _config.system.op_mode = (T2_Type::EM_OpMode_t)v_sys["op_mode"].as<uint8_t>();
        }
        cfgSet(v_sys["watchdog_ms"], _config.system.watchdog_ms);
    }

    // [처리 단위 2] wifi 파라미터 파싱
    JsonObjectConst v_wifi = p_doc["wifi"];
    if (!v_wifi.isNull()) {
        if (!v_wifi["mode"].isNull()) {
            _config.wifi.mode = (T2_Type::EM_WiFiMode_t)v_wifi["mode"].as<uint8_t>();
        }
        cfgStr(v_wifi["ap_ssid"], _config.wifi.ap_ssid, sizeof(_config.wifi.ap_ssid));
        cfgStr(v_wifi["ap_pw"],   _config.wifi.ap_pw,   sizeof(_config.wifi.ap_pw));
        cfgStr(v_wifi["ap_ip"],   _config.wifi.ap_ip,   sizeof(_config.wifi.ap_ip));

        JsonArrayConst v_multi = v_wifi["multi_ap"];
        if (!v_multi.isNull()) {
            uint8_t i = 0;
            for (JsonObjectConst v_ap : v_multi) {
                if (i >= T2_Def::Global::NetLimit::NET_MULTI_AP_MAX) break;
                cfgStr(v_ap["ssid"], _config.wifi.multi_ssid[i], sizeof(_config.wifi.multi_ssid[i]));
                cfgStr(v_ap["pw"],   _config.wifi.multi_pw[i],   sizeof(_config.wifi.multi_pw[i]));
                i++;
            }
        }
        cfgSet(v_wifi["disconnect_delay_ms"], _config.wifi.disconnect_delay_ms);  // [이슈 9] WiFi 연결 해제 지연 ms
    }

    // [처리 단위 3] mqtt 및 NTP 파라미터 파싱
    JsonObjectConst v_mqtt = p_doc["mqtt"];
    if (!v_mqtt.isNull()) {
        cfgSet(v_mqtt["enable"],     _config.mqtt.enable);
        cfgStr(v_mqtt["broker"],     _config.mqtt.broker, sizeof(_config.mqtt.broker));
        cfgSet(v_mqtt["port"],       _config.mqtt.port);
        cfgStr(v_mqtt["id"],         _config.mqtt.id,     sizeof(_config.mqtt.id));
        cfgStr(v_mqtt["pw"],         _config.mqtt.pw,     sizeof(_config.mqtt.pw));
        cfgStr(v_mqtt["topic_root"], _config.mqtt.topic_root, sizeof(_config.mqtt.topic_root));
        cfgStr(v_mqtt["lwt_topic"],  _config.mqtt.lwt_topic,  sizeof(_config.mqtt.lwt_topic));
        cfgSet(v_mqtt["qos"],        _config.mqtt.qos);
        cfgSet(v_mqtt["proto_ver"],  _config.mqtt.proto_ver);
        // [이슈 9] NTP 파라미터 파싱
        cfgStr(v_mqtt["ntp_server1"], _config.mqtt.ntp_server1, sizeof(_config.mqtt.ntp_server1));
        cfgStr(v_mqtt["ntp_server2"], _config.mqtt.ntp_server2, sizeof(_config.mqtt.ntp_server2));
        cfgStr(v_mqtt["ntp_tz"],      _config.mqtt.ntp_tz,      sizeof(_config.mqtt.ntp_tz));
    }

    // [처리 단위 4] storage 파라미터 파싱 (안전 상한 검증 포함)
    JsonObjectConst v_storage = p_doc["storage"];
    if (!v_storage.isNull()) {
        cfgSet(v_storage["rot_mb"],         _config.storage.rot_mb);
        cfgSet(v_storage["rot_min"],        _config.storage.rot_min);
        cfgSet(v_storage["save_raw"],       _config.storage.save_raw);
        cfgSet(v_storage["keep_max"],       _config.storage.keep_max);
        cfgSet(v_storage["idle_flush_ms"],  _config.storage.idle_flush_ms);
        // [이슈 6] pre_trig_sec 상한(10초) 검증 — 과도하게 크면 힙 단편화 및 OOM 발생
        uint8_t v_preTrigsec = _config.storage.pre_trig_sec;
        cfgSet(v_storage["pre_trig_sec"], v_preTrigsec);
        constexpr uint8_t PRE_TRIG_MAX = 10;
        if (v_preTrigsec > PRE_TRIG_MAX) v_preTrigsec = PRE_TRIG_MAX;
        _config.storage.pre_trig_sec = v_preTrigsec;
    }

    // [처리 단위 5] output 프레임 설정 파싱
    JsonObjectConst v_out = p_doc["output"];
    if (!v_out.isNull()) {
        cfgSet(v_out["enabled"],         _config.output.enabled);
        cfgSet(v_out["output_sequence"], _config.output.output_sequence);
        // [이슈 1] sequence_frames 상한(SEQUENCE_FRAMES_MAX=32) 범위 검증
        uint16_t v_frames = _config.output.sequence_frames;
        cfgSet(v_out["sequence_frames"], v_frames);
        if (v_frames > T2_Def::Global::System::SEQUENCE_FRAMES_MAX)
            v_frames = T2_Def::Global::System::SEQUENCE_FRAMES_MAX;
        _config.output.sequence_frames = v_frames;
    }

    // [처리 단위 6] decision 판정 파라미터 파싱
    JsonObjectConst v_dec = p_doc["decision"];
    if (!v_dec.isNull()) {
        cfgSet(v_dec["max_trial_count"],   _config.decision.max_trial_count);
        cfgSet(v_dec["sta_lta_threshold"], _config.decision.sta_lta_threshold);
        cfgSet(v_dec["min_trigger_count"], _config.decision.min_trigger_count);
        cfgSet(v_dec["valid_start_sec"],   _config.decision.valid_start_sec);
        cfgSet(v_dec["valid_end_sec"],     _config.decision.valid_end_sec);
        // 범위 안전 제한
        if (_config.decision.max_trial_count > T2_Def::Global::Decision::MAX_TRIAL_COUNT_MAX)
            _config.decision.max_trial_count = T2_Def::Global::Decision::MAX_TRIAL_COUNT_MAX;
    }

    // [처리 단위 7] trigger 글로벌 슬립 및 홀드 파라미터 파싱
    JsonObjectConst v_trig = p_doc["trigger"];
    if (!v_trig.isNull()) {
        cfgSet(v_trig["hold_ms"],   _config.trig_hold_ms);
        cfgSet(v_trig["use_sleep"], _config.trig_use_sleep);
        cfgSet(v_trig["sleep_sec"], _config.trig_sleep_sec);
    }

    // [처리 단위 8] accel 가속도 센서 대역 및 DSP 파라미터 파싱
    JsonObjectConst v_acc = p_doc["accel"];
    if (!v_acc.isNull()) {
        cfgSet(v_acc["enable"],         _config.accel.enable);
        cfgSet(v_acc["axis_mask"],      _config.accel.axis_mask);
        cfgSet(v_acc["range"],          _config.accel.range);
        cfgSet(v_acc["odr"],            _config.accel.odr);
        cfgSet(v_acc["bwp"],            _config.accel.bwp);
        cfgSet(v_acc["filter_perf"],    _config.accel.filter_perf);
        cfgSet(v_acc["fifo_watermark"], _config.accel.fifo_watermark);
        cfgSet(v_acc["sample_rate"],    _config.accel.sample_rate);
        cfgSet(v_acc["fft_size"],       _config.accel.fft_size);
        // [이슈 1] fft_size 유효성 검증: 2의 거듭제곱 및 FFT_SIZE_MAX 이하 확인
        {
            uint32_t v = _config.accel.fft_size;
            auto isPow2 = [](uint32_t x){ return x > 0 && (x & (x-1)) == 0; };
            if (!isPow2(v) || v > T2_Def::Accel::Sensor::FFT_SIZE_MAX)
                _config.accel.fft_size = T2_Def::Accel::Sensor::FFT_SIZE_DEF;
        }
        cfgSet(v_acc["noise_perf"],     _config.accel.noise_perf);  // [이슈 18]

        cfgSet(v_acc["motion_en"],      _config.accel.motion_en);
        cfgSet(v_acc["wake_g"],         _config.accel.wake_g);
        cfgSet(v_acc["wake_dur"],       _config.accel.wake_dur);

        cfgSetArray(v_acc["rms_thresh"],      _config.accel.rms_thresh);
        cfgSetArray(v_acc["kurt_ng_thresh"],  _config.accel.kurt_ng_thresh);
        cfgSetArray(v_acc["crest_ng_thresh"], _config.accel.crest_ng_thresh);
        cfgSetArray(v_acc["skew_ng_thresh"],  _config.accel.skew_ng_thresh);

        JsonArrayConst v_bands = v_acc["bands"];
        if (!v_bands.isNull()) {
            uint8_t i = 0;
            for (JsonObjectConst v_b : v_bands) {
                if (i >= T2_Def::Accel::FeatureLimit::BAND_MAX) break;
                cfgSet(v_b["en"],    _config.accel.band_en[i]);
                cfgSet(v_b["start"], _config.accel.band_start[i]);
                cfgSet(v_b["end"],   _config.accel.band_end[i]);

                if (!v_b["thresh"].isNull()) {
                    if (v_b["thresh"].is<JsonArrayConst>()) {
                        JsonArrayConst v_arr = v_b["thresh"].as<JsonArrayConst>();
                        for (size_t a = 0; a < T2_Def::Accel::Sensor::AXIS_MAX; a++) {
                            if (a < v_arr.size()) _config.accel.band_thresh[a][i] = v_arr[a].as<float>();
                        }
                    } else {
                        float v_val = v_b["thresh"].as<float>();
                        for (size_t a = 0; a < T2_Def::Accel::Sensor::AXIS_MAX; a++) {
                            _config.accel.band_thresh[a][i] = v_val;
                        }
                    }
                }
                i++;
            }
            _config.accel.active_band_count = i;
        }

        cfgSetArray(v_acc["offset"], _config.accel.offset);
        cfgSetArray(v_acc["gain"],   _config.accel.gain);
        cfgSet(v_acc["peak_amp_min"],      _config.accel.peak_amp_min);
        cfgSet(v_acc["peak_freq_gap_min"], _config.accel.peak_freq_gap_min);

        parseDspConfig(v_acc["dsp"], _config.accel.dsp);
    }

    // [처리 단위 9] gyro 자이로 센서 파라미터 파싱
    JsonObjectConst v_gyr = p_doc["gyro"];
    if (!v_gyr.isNull()) {
        cfgSet(v_gyr["enable"],      _config.gyro.enable);
        cfgSet(v_gyr["axis_mask"],   _config.gyro.axis_mask);
        cfgSet(v_gyr["range"],       _config.gyro.range);
        cfgSet(v_gyr["odr"],         _config.gyro.odr);
        cfgSet(v_gyr["bwp"],         _config.gyro.bwp);
        cfgSet(v_gyr["filter_perf"], _config.gyro.filter_perf);
        cfgSet(v_gyr["noise_perf"],  _config.gyro.noise_perf);
        cfgSet(v_gyr["sample_rate"], _config.gyro.sample_rate);
        cfgSet(v_gyr["fft_size"],    _config.gyro.fft_size);

        cfgSetArray(v_gyr["rms_thresh"],      _config.gyro.rms_thresh);
        cfgSetArray(v_gyr["kurt_ng_thresh"],  _config.gyro.kurt_ng_thresh);
        cfgSetArray(v_gyr["crest_ng_thresh"], _config.gyro.crest_ng_thresh);
        cfgSetArray(v_gyr["skew_ng_thresh"],  _config.gyro.skew_ng_thresh);

        JsonArrayConst v_bands = v_gyr["bands"];
        if (!v_bands.isNull()) {
            uint8_t i = 0;
            for (JsonObjectConst v_b : v_bands) {
                if (i >= T2_Def::Gyro::FeatureLimit::BAND_MAX) break;
                cfgSet(v_b["en"],    _config.gyro.band_en[i]);
                cfgSet(v_b["start"], _config.gyro.band_start[i]);
                cfgSet(v_b["end"],   _config.gyro.band_end[i]);

                if (!v_b["thresh"].isNull()) {
                    if (v_b["thresh"].is<JsonArrayConst>()) {
                        JsonArrayConst v_arr = v_b["thresh"].as<JsonArrayConst>();
                        for (size_t a = 0; a < T2_Def::Gyro::Sensor::AXIS_MAX; a++) {
                            if (a < v_arr.size()) _config.gyro.band_thresh[a][i] = v_arr[a].as<float>();
                        }
                    } else {
                        float v_val = v_b["thresh"].as<float>();
                        for (size_t a = 0; a < T2_Def::Gyro::Sensor::AXIS_MAX; a++) {
                            _config.gyro.band_thresh[a][i] = v_val;
                        }
                    }
                }
                i++;
            }
            _config.gyro.active_band_count = i;
        }

        cfgSetArray(v_gyr["offset"], _config.gyro.offset);
        cfgSetArray(v_gyr["gain"],   _config.gyro.gain);
        cfgSet(v_gyr["peak_amp_min"],      _config.gyro.peak_amp_min);
        cfgSet(v_gyr["peak_freq_gap_min"], _config.gyro.peak_freq_gap_min);

        parseDspConfig(v_gyr["dsp"], _config.gyro.dsp);
    }

    // [처리 단위 10] audio 마이크 센서 및 멜 대역, 노이즈게이트 등 파이프라인 파싱
    JsonObjectConst v_aud = p_doc["audio"];
    if (!v_aud.isNull()) {
        cfgSet(v_aud["enable"],       _config.audio.enable);
        cfgSet(v_aud["channel_mask"], _config.audio.channel_mask);
        cfgSet(v_aud["sample_rate"],  _config.audio.sample_rate);
        cfgSet(v_aud["fft_size"],     _config.audio.fft_size);

        uint8_t v_melBands = _config.audio.mel_bands;
        cfgSet(v_aud["mel_bands"], v_melBands);
        if (v_melBands == 0 || v_melBands > T2_Def::Audio::FeatureLimit::MEL_BANDS_MAX) {
            v_melBands = T2_Def::Audio::FeatureLimit::MEL_BANDS_DEF;
        }
        _config.audio.mel_bands = v_melBands;

        cfgSetArray(v_aud["rms_thresh"],      _config.audio.rms_thresh);
        cfgSetArray(v_aud["kurt_ng_thresh"],  _config.audio.kurt_ng_thresh);
        cfgSetArray(v_aud["crest_ng_thresh"], _config.audio.crest_ng_thresh);
        cfgSetArray(v_aud["skew_ng_thresh"],  _config.audio.skew_ng_thresh);

        JsonArrayConst v_bands = v_aud["bands"];
        if (!v_bands.isNull()) {
            uint8_t i = 0;
            for (JsonObjectConst v_b : v_bands) {
                if (i >= T2_Def::Audio::FeatureLimit::BAND_MAX) break;
                cfgSet(v_b["en"],    _config.audio.band_en[i]);
                cfgSet(v_b["start"], _config.audio.band_start[i]);
                cfgSet(v_b["end"],   _config.audio.band_end[i]);

                if (!v_b["thresh"].isNull()) {
                    if (v_b["thresh"].is<JsonArrayConst>()) {
                        JsonArrayConst v_arr = v_b["thresh"].as<JsonArrayConst>();
                        for (size_t ch = 0; ch < T2_Def::Audio::Sensor::CHANNELS_MAX; ch++) {
                            if (ch < v_arr.size()) _config.audio.band_thresh[ch][i] = v_arr[ch].as<float>();
                        }
                    } else {
                        float v_val = v_b["thresh"].as<float>();
                        for (size_t ch = 0; ch < T2_Def::Audio::Sensor::CHANNELS_MAX; ch++) {
                            _config.audio.band_thresh[ch][i] = v_val;
                        }
                    }
                }
                i++;
            }
            _config.audio.active_band_count = i;
        }

        JsonObjectConst v_noise = v_aud["noise"];
        if (!v_noise.isNull()) {
            cfgSet(v_noise["gate_en"],     _config.audio.noise.gate_en);
            cfgSet(v_noise["gate_thresh"], _config.audio.noise.gate_thresh);
            if (!v_noise["mode"].isNull()) {
                _config.audio.noise.mode = (T2_Type::EM_NoiseMode_t)v_noise["mode"].as<uint8_t>();
            }
            cfgSet(v_noise["sub_str"],     _config.audio.noise.sub_str);
            cfgSet(v_noise["adp_alpha"],   _config.audio.noise.adp_alpha);
            cfgSet(v_noise["learn_frames"],_config.audio.noise.learn_frames);
        }

        cfgSet(v_aud["pre_en"],    _config.audio.pre_en);
        cfgSet(v_aud["pre_alpha"], _config.audio.pre_alpha);
        // [이슈 7] beam_gain (0.0f ~ 1.0f) 범위 유효성 검증 추가
        float v_beamGain = _config.audio.beam_gain;
        cfgSet(v_aud["beam_gain"], v_beamGain);
        if (v_beamGain < 0.0f) v_beamGain = 0.0f;
        if (v_beamGain > 1.0f) v_beamGain = 1.0f;
        _config.audio.beam_gain = v_beamGain;
        cfgSet(v_aud["window_ms"], _config.audio.window_ms);
        cfgSet(v_aud["hop_ms"],    _config.audio.hop_ms);

        cfgSet(v_aud["auto_idle_min"], _config.audio.auto_idle_min);
        cfgSet(v_aud["ref_freq"],      _config.audio.ref_freq);
        cfgSet(v_aud["filt_min"],      _config.audio.filt_min);
        cfgSet(v_aud["filt_max"],      _config.audio.filt_max);
        cfgSet(v_aud["gain_max"],      _config.audio.gain_max);
        cfgSet(v_aud["gain_min"],      _config.audio.gain_min);
        cfgSet(v_aud["norm_safe"],     _config.audio.norm_safe);
        cfgSetArray(v_aud["gain_ch"],  _config.audio.gain_ch);

        JsonArrayConst v_eq = v_aud["eq_coeffs"];
        if (!v_eq.isNull()) {
            if (v_eq[0].is<JsonArrayConst>()) {
                for (size_t ch = 0; ch < T2_Def::Audio::Sensor::CHANNELS_MAX; ch++) {
                    JsonArrayConst v_chEq = v_eq[ch].as<JsonArrayConst>();
                    size_t idx = 0;
                    for (JsonVariantConst v_val : v_chEq) {
                        if (idx >= T2_Def::Audio::FeatureLimit::FIR_TAPS_MAX) break;
                        _config.audio.eq_coeffs[ch][idx] = v_val.as<float>();
                        idx++;
                    }
                }
            } else {
                size_t idx = 0;
                for (JsonVariantConst v_val : v_eq) {
                    if (idx >= T2_Def::Audio::FeatureLimit::FIR_TAPS_MAX) break;
                    float v_fVal = v_val.as<float>();
                    for (size_t ch = 0; ch < T2_Def::Audio::Sensor::CHANNELS_MAX; ch++) {
                        _config.audio.eq_coeffs[ch][idx] = v_fVal;
                    }
                    idx++;
                }
            }
        }

        cfgSet(v_aud["active_ceps_count"], _config.audio.active_ceps_count);
        cfgSet(v_aud["active_peak_count"], _config.audio.active_peak_count);

        JsonArrayConst v_ceps = v_aud["ceps_targets"];
        if (!v_ceps.isNull()) {
            uint8_t idx = 0;
            for (JsonVariantConst v_val : v_ceps) {
                if (idx >= T2_Def::Audio::FeatureLimit::CEPS_TARGET_MAX) break;
                _config.audio.ceps_targets[idx] = v_val.as<float>();
                idx++;
            }
            _config.audio.active_ceps_count = idx;
        }

        cfgSet(v_aud["f_min"],             _config.audio.f_min);
        cfgSet(v_aud["f_max"],             _config.audio.f_max);
        cfgSet(v_aud["peak_amp_min"],      _config.audio.peak_amp_min);
        cfgSet(v_aud["peak_freq_gap_min"], _config.audio.peak_freq_gap_min);

        parseDspConfig(v_aud["dsp"], _config.audio.dsp);
    }
}

/**
 * @brief LittleFS 플래시 파일에서 JSON 데이터 독출 및 RAM 적재
 * @return 로드 성공 여부
 */
bool CL_T2_ConfigManager::load() {
    xSemaphoreTake(_lock, portMAX_DELAY);
    File v_file = LittleFS.open(T2_Def::Global::Path::FILE_CFG_JSON_CONST, "r");
    if (!v_file) {
        ESP_LOGE(TAG, "Config load failed");
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

/**
 * @brief 현재 RAM 상의 설정을 JSON 직렬화 후 LittleFS 플래시에 원자적(Atomic)으로 기입
 * @details 임시 백업 파일 쓰기 후 플러시 완료를 확인하고 치환을 수행하여 전원 유실 피해 방지
 * @return 쓰기 및 교체 성공 여부
 */
bool CL_T2_ConfigManager::save() {
    xSemaphoreTake(_lock, portMAX_DELAY);
    JsonDocument v_doc;

    // [처리 단위 1] system, wifi, mqtt 글로벌 파라미터 직렬화
    JsonObject v_sys = v_doc["system"].to<JsonObject>();
    v_sys["site_id"]      = _config.system.site_id;
    v_sys["tele_hz"]      = _config.system.tele_hz;
    v_sys["wave_hz"]      = _config.system.wave_hz;
    v_sys["op_mode"]      = (uint8_t)_config.system.op_mode;
    v_sys["watchdog_ms"]  = _config.system.watchdog_ms;

    JsonObject v_wifi = v_doc["wifi"].to<JsonObject>();
    v_wifi["mode"]    = (uint8_t)_config.wifi.mode;
    v_wifi["ap_ssid"] = _config.wifi.ap_ssid;
    v_wifi["ap_pw"]   = _config.wifi.ap_pw;
    v_wifi["ap_ip"]   = _config.wifi.ap_ip;
    JsonArray v_multi = v_wifi["multi_ap"].to<JsonArray>();
    for (uint8_t i = 0; i < T2_Def::Global::NetLimit::NET_MULTI_AP_MAX; i++) {
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

    // [처리 단위 2] storage, output, trigger 파라미터 직렬화
    JsonObject v_storage = v_doc["storage"].to<JsonObject>();
    v_storage["rot_mb"]         = _config.storage.rot_mb;
    v_storage["rot_min"]        = _config.storage.rot_min;
    v_storage["save_raw"]       = _config.storage.save_raw;
    v_storage["keep_max"]       = _config.storage.keep_max;
    v_storage["idle_flush_ms"]  = _config.storage.idle_flush_ms;
    v_storage["pre_trig_sec"]   = _config.storage.pre_trig_sec;

    JsonObject v_out = v_doc["output"].to<JsonObject>();
    v_out["enabled"]         = _config.output.enabled;
    v_out["output_sequence"] = _config.output.output_sequence;
    v_out["sequence_frames"] = _config.output.sequence_frames;

    JsonObject v_trig = v_doc["trigger"].to<JsonObject>();
    v_trig["hold_ms"]   = _config.trig_hold_ms;
    v_trig["use_sleep"] = _config.trig_use_sleep;
    v_trig["sleep_sec"] = _config.trig_sleep_sec;

    // [처리 단위 3] accel 가속도 대역 및 센서 임계 팩터 직렬화
    JsonObject v_acc = v_doc["accel"].to<JsonObject>();
    v_acc["enable"]         = _config.accel.enable;
    v_acc["axis_mask"]      = _config.accel.axis_mask;
    v_acc["range"]          = _config.accel.range;
    v_acc["odr"]            = _config.accel.odr;
    v_acc["bwp"]            = _config.accel.bwp;
    v_acc["filter_perf"]    = _config.accel.filter_perf;
    v_acc["fifo_watermark"] = _config.accel.fifo_watermark;
    v_acc["sample_rate"]    = _config.accel.sample_rate;
    v_acc["fft_size"]       = _config.accel.fft_size;

    v_acc["motion_en"]      = _config.accel.motion_en;
    v_acc["wake_g"]         = _config.accel.wake_g;
    v_acc["wake_dur"]       = _config.accel.wake_dur;

    JsonArray v_rmsThV = v_acc["rms_thresh"].to<JsonArray>();
    for(int a=0; a<T2_Def::Accel::Sensor::AXIS_MAX; a++) cfgAddFloatExact(v_rmsThV, _config.accel.rms_thresh[a]);
    JsonArray v_kurtThV = v_acc["kurt_ng_thresh"].to<JsonArray>();
    for(int a=0; a<T2_Def::Accel::Sensor::AXIS_MAX; a++) cfgAddFloatExact(v_kurtThV, _config.accel.kurt_ng_thresh[a]);
    JsonArray v_crestThV = v_acc["crest_ng_thresh"].to<JsonArray>();
    for(int a=0; a<T2_Def::Accel::Sensor::AXIS_MAX; a++) cfgAddFloatExact(v_crestThV, _config.accel.crest_ng_thresh[a]);
    JsonArray v_skewThV = v_acc["skew_ng_thresh"].to<JsonArray>();
    for(int a=0; a<T2_Def::Accel::Sensor::AXIS_MAX; a++) cfgAddFloatExact(v_skewThV, _config.accel.skew_ng_thresh[a]);

    JsonArray v_vbands = v_acc["bands"].to<JsonArray>();
    for (uint8_t i = 0; i < _config.accel.active_band_count; i++) {
        JsonObject v_b = v_vbands.add<JsonObject>();
        v_b["en"]    = _config.accel.band_en[i];
        v_b["start"] = _config.accel.band_start[i];
        v_b["end"]   = _config.accel.band_end[i];

        JsonArray v_thArr = v_b["thresh"].to<JsonArray>();
        for(int a=0; a<T2_Def::Accel::Sensor::AXIS_MAX; a++) cfgAddFloatExact(v_thArr, _config.accel.band_thresh[a][i]);
    }

    JsonArray v_vcoff = v_acc["offset"].to<JsonArray>();
    for(uint8_t i=0; i<T2_Def::Accel::Sensor::AXIS_MAX; i++) cfgAddFloatExact(v_vcoff, _config.accel.offset[i]);
    JsonArray v_vcgain = v_acc["gain"].to<JsonArray>();
    for(uint8_t i=0; i<T2_Def::Accel::Sensor::AXIS_MAX; i++) cfgAddFloatExact(v_vcgain, _config.accel.gain[i]);

    v_acc["peak_amp_min"]      = _config.accel.peak_amp_min;
    v_acc["peak_freq_gap_min"] = _config.accel.peak_freq_gap_min;

    serializeDspConfig(v_acc["dsp"].to<JsonObject>(), _config.accel.dsp);

    // [처리 단위 4] gyro 자이로 센서 설정 파라미터 직렬화
    JsonObject v_gyr = v_doc["gyro"].to<JsonObject>();
    v_gyr["enable"]      = _config.gyro.enable;
    v_gyr["axis_mask"]   = _config.gyro.axis_mask;
    v_gyr["range"]       = _config.gyro.range;
    v_gyr["odr"]         = _config.gyro.odr;
    v_gyr["bwp"]         = _config.gyro.bwp;
    v_gyr["filter_perf"] = _config.gyro.filter_perf;
    v_gyr["noise_perf"]  = _config.gyro.noise_perf;
    v_gyr["sample_rate"] = _config.gyro.sample_rate;
    v_gyr["fft_size"]    = _config.gyro.fft_size;

    JsonArray v_rmsThG = v_gyr["rms_thresh"].to<JsonArray>();
    for(int a=0; a<T2_Def::Gyro::Sensor::AXIS_MAX; a++) cfgAddFloatExact(v_rmsThG, _config.gyro.rms_thresh[a]);
    JsonArray v_kurtThG = v_gyr["kurt_ng_thresh"].to<JsonArray>();
    for(int a=0; a<T2_Def::Gyro::Sensor::AXIS_MAX; a++) cfgAddFloatExact(v_kurtThG, _config.gyro.kurt_ng_thresh[a]);
    JsonArray v_crestThG = v_gyr["crest_ng_thresh"].to<JsonArray>();
    for(int a=0; a<T2_Def::Gyro::Sensor::AXIS_MAX; a++) cfgAddFloatExact(v_crestThG, _config.gyro.crest_ng_thresh[a]);
    JsonArray v_skewThG = v_gyr["skew_ng_thresh"].to<JsonArray>();
    for(int a=0; a<T2_Def::Gyro::Sensor::AXIS_MAX; a++) cfgAddFloatExact(v_skewThG, _config.gyro.skew_ng_thresh[a]);

    JsonArray v_gbands = v_gyr["bands"].to<JsonArray>();
    for (uint8_t i = 0; i < _config.gyro.active_band_count; i++) {
        JsonObject v_b = v_gbands.add<JsonObject>();
        v_b["en"]    = _config.gyro.band_en[i];
        v_b["start"] = _config.gyro.band_start[i];
        v_b["end"]   = _config.gyro.band_end[i];

        JsonArray v_thArr = v_b["thresh"].to<JsonArray>();
        for(int a=0; a<T2_Def::Gyro::Sensor::AXIS_MAX; a++) cfgAddFloatExact(v_thArr, _config.gyro.band_thresh[a][i]);
    }

    JsonArray v_gcoff = v_gyr["offset"].to<JsonArray>();
    for(uint8_t i=0; i<T2_Def::Gyro::Sensor::AXIS_MAX; i++) cfgAddFloatExact(v_gcoff, _config.gyro.offset[i]);
    JsonArray v_gcgain = v_gyr["gain"].to<JsonArray>();
    for(uint8_t i=0; i<T2_Def::Gyro::Sensor::AXIS_MAX; i++) cfgAddFloatExact(v_gcgain, _config.gyro.gain[i]);

    v_gyr["peak_amp_min"]      = _config.gyro.peak_amp_min;
    v_gyr["peak_freq_gap_min"] = _config.gyro.peak_freq_gap_min;

    serializeDspConfig(v_gyr["dsp"].to<JsonObject>(), _config.gyro.dsp);

    // [처리 단위 5] audio 오디오 및 FIR 이퀄라이저 탭 설정 직렬화
    JsonObject v_aud = v_doc["audio"].to<JsonObject>();
    v_aud["enable"]       = _config.audio.enable;
    v_aud["channel_mask"] = _config.audio.channel_mask;
    v_aud["sample_rate"]  = _config.audio.sample_rate;
    v_aud["fft_size"]     = _config.audio.fft_size;
    v_aud["mel_bands"]    = _config.audio.mel_bands;

    JsonArray v_rmsThA = v_aud["rms_thresh"].to<JsonArray>();
    for(int ch=0; ch<T2_Def::Audio::Sensor::CHANNELS_MAX; ch++) cfgAddFloatExact(v_rmsThA, _config.audio.rms_thresh[ch]);
    JsonArray v_kurtThA = v_aud["kurt_ng_thresh"].to<JsonArray>();
    for(int ch=0; ch<T2_Def::Audio::Sensor::CHANNELS_MAX; ch++) cfgAddFloatExact(v_kurtThA, _config.audio.kurt_ng_thresh[ch]);
    JsonArray v_crestThA = v_aud["crest_ng_thresh"].to<JsonArray>();
    for(int ch=0; ch<T2_Def::Audio::Sensor::CHANNELS_MAX; ch++) cfgAddFloatExact(v_crestThA, _config.audio.crest_ng_thresh[ch]);
    JsonArray v_skewThA = v_aud["skew_ng_thresh"].to<JsonArray>();
    for(int ch=0; ch<T2_Def::Audio::Sensor::CHANNELS_MAX; ch++) cfgAddFloatExact(v_skewThA, _config.audio.skew_ng_thresh[ch]);

    JsonArray v_abands = v_aud["bands"].to<JsonArray>();
    for (uint8_t i = 0; i < _config.audio.active_band_count; i++) {
        JsonObject v_b = v_abands.add<JsonObject>();
        v_b["en"]    = _config.audio.band_en[i];
        v_b["start"] = _config.audio.band_start[i];
        v_b["end"]   = _config.audio.band_end[i];

        JsonArray v_thArr = v_b["thresh"].to<JsonArray>();
        for(int ch=0; ch<T2_Def::Audio::Sensor::CHANNELS_MAX; ch++) cfgAddFloatExact(v_thArr, _config.audio.band_thresh[ch][i]);
    }

    JsonObject v_noise = v_aud["noise"].to<JsonObject>();
    v_noise["gate_en"]     = _config.audio.noise.gate_en;
    v_noise["gate_thresh"] = _config.audio.noise.gate_thresh;
    v_noise["mode"]        = (uint8_t)_config.audio.noise.mode;
    v_noise["sub_str"]     = _config.audio.noise.sub_str;
    v_noise["adp_alpha"]   = _config.audio.noise.adp_alpha;
    v_noise["learn_frames"]= _config.audio.noise.learn_frames;

    v_aud["pre_en"]    = _config.audio.pre_en;
    v_aud["pre_alpha"] = _config.audio.pre_alpha;
    v_aud["beam_gain"] = _config.audio.beam_gain;
    v_aud["window_ms"] = _config.audio.window_ms;
    v_aud["hop_ms"]    = _config.audio.hop_ms;

    v_aud["auto_idle_min"] = _config.audio.auto_idle_min;
    v_aud["ref_freq"]      = _config.audio.ref_freq;
    v_aud["filt_min"]      = _config.audio.filt_min;
    v_aud["filt_max"]      = _config.audio.filt_max;
    v_aud["gain_max"]      = _config.audio.gain_max;
    v_aud["gain_min"]      = _config.audio.gain_min;
    v_aud["norm_safe"]     = _config.audio.norm_safe;

    JsonArray v_gch = v_aud["gain_ch"].to<JsonArray>();
    for (int ch = 0; ch < T2_Def::Audio::Sensor::CHANNELS_MAX; ch++) v_gch.add(_config.audio.gain_ch[ch]);

    // [이슈 12] eq_coeffs 직렬화를 FIR_TAPS_DEF(63)로 축소 (파일 크기 다이어트 및 쓰기 오버헤드 경감)
    JsonArray v_eq = v_aud["eq_coeffs"].to<JsonArray>();
    for (size_t ch = 0; ch < T2_Def::Audio::Sensor::CHANNELS_MAX; ch++) {
        JsonArray v_chEq = v_eq.add<JsonArray>();
        for (size_t idx = 0; idx < T2_Def::Audio::FeatureLimit::FIR_TAPS_DEF; idx++) {
            v_chEq.add(_config.audio.eq_coeffs[ch][idx]);
        }
    }

    v_aud["active_ceps_count"] = _config.audio.active_ceps_count;
    v_aud["active_peak_count"] = _config.audio.active_peak_count;

    JsonArray v_ceps = v_aud["ceps_targets"].to<JsonArray>();
    for (uint8_t i = 0; i < _config.audio.active_ceps_count; i++) v_ceps.add(_config.audio.ceps_targets[i]);

    v_aud["f_min"]             = _config.audio.f_min;
    v_aud["f_max"]             = _config.audio.f_max;
    v_aud["peak_amp_min"]      = _config.audio.peak_amp_min;
    v_aud["peak_freq_gap_min"] = _config.audio.peak_freq_gap_min;

    serializeDspConfig(v_aud["dsp"].to<JsonObject>(), _config.audio.dsp);

    // [처리 단위 6] Atomic File Commit 수행
    File v_tmp = LittleFS.open("/sys/config_245.tmp", "w");
    if (!v_tmp) {
        xSemaphoreGive(_lock);
        return false;
    }
    serializeJson(v_doc, v_tmp);
    v_tmp.flush();  // [이슈 5] 플래시 쓰기 완료 동기화 보장 (원자적 쓰기의 데이터 무결성 보장)
    v_tmp.close();

    LittleFS.remove(T2_Def::Global::Path::FILE_CFG_JSON_CONST);
    LittleFS.rename("/sys/config_245.tmp", T2_Def::Global::Path::FILE_CFG_JSON_CONST);

    _isDirty = false;
    xSemaphoreGive(_lock);
    return true;
}

/**
 * @brief 설정 공장 초기화 수행 후 플래시 파일 즉각 반영
 */
void CL_T2_ConfigManager::resetToDefault() {
    xSemaphoreTake(_lock, portMAX_DELAY);
    _loadDefaults();
    xSemaphoreGive(_lock);
    save();
}

/**
 * @brief 설정값 구조체 안전 스냅샷 획득
 * @return 복사된 ST_DynamicConfig_t 구조체 값
 */
T2_Type::ST_DynamicConfig_t CL_T2_ConfigManager::getConfig() {
    xSemaphoreTake(_lock, portMAX_DELAY);
    T2_Type::ST_DynamicConfig_t v_snap = _config;
    xSemaphoreGive(_lock);
    return v_snap;
}

/**
 * @brief 설정 구조체 전면 갱신 및 플래시 즉각 동기화
 * @param p_newConfig 복사 반영할 새 설정 구조체
 * @return 쓰기 완료 여부
 */
bool CL_T2_ConfigManager::updateConfig(const T2_Type::ST_DynamicConfig_t& p_newConfig) {
    xSemaphoreTake(_lock, portMAX_DELAY);
    _config = p_newConfig;
    xSemaphoreGive(_lock);
    return save();
}

/**
 * @brief 외부 JSON 문자열을 병합 후 Dirty 마크 및 Lazy Write 타임스탬프 예약
 * @param p_jsonString 갱신용 partial JSON 문자열
 * @return 파싱 및 병합 성공 여부
 */
bool CL_T2_ConfigManager::updateFromJson(const char* p_jsonString) {
    xSemaphoreTake(_lock, portMAX_DELAY);
    JsonDocument v_doc;  // [이슈 11] g_psramAlloc 제거 — 10KB 미만은 내부 힙 SRAM 할당이 압도적 이득
    DeserializationError v_err = deserializeJson(v_doc, p_jsonString);
    if (v_err) {
        ESP_LOGE(TAG, "Update Json Error: %s", v_err.c_str());
        xSemaphoreGive(_lock);
        return false;
    }
    _applyJson(v_doc);
    _isDirty = true;
    _lastModifiedMs = (uint32_t)millis();
    xSemaphoreGive(_lock);
    return true;
}

/**
 * @brief 백그라운드 지연 쓰기 스케줄러 처리
 * @details 최종 변경 발생 후 LAZY_WRITE_MS_DEF(예: 3초) 이상 경과 시 플래시 실제 기록 처리
 */
void CL_T2_ConfigManager::checkLazyWrite() {
    if (!_isDirty || _isTuningActive) return;
    uint32_t v_now = (uint32_t)millis();
    if (v_now - _lastModifiedMs > T2_Def::Global::Task::LAZY_WRITE_MS_DEF) {
        ESP_LOGI(TAG, "Executing Lazy Write to flash...");
        save();
    }
}

/**
 * @brief 실시간 튜닝(Preview) RAM 값 업데이트
 * @details 플래시에 영향을 주지 않고 메모리 설정값만 즉각 변경 및 튜닝 활성 상태 기입
 * @param p_jsonString 튜닝용 JSON 페이로드 문자열
 * @return 병합 갱신 완료 여부
 */
bool CL_T2_ConfigManager::updatePreview(const char* p_jsonString) {
    xSemaphoreTake(_lock, portMAX_DELAY);
    JsonDocument v_doc;  // [이슈 11] g_psramAlloc 제거
    DeserializationError v_err = deserializeJson(v_doc, p_jsonString);
    if (v_err) {
        xSemaphoreGive(_lock);
        return false;
    }
    _applyJson(v_doc);
    _isTuningActive = true; 
    xSemaphoreGive(_lock);
    return true;
}

/**
 * @brief 튜닝 설정값 영구 반영 확정
 * @return save 결과
 */
bool CL_T2_ConfigManager::commitSave() {
    _isTuningActive = false;
    return save();
}

/**
 * @brief 튜닝 설정 취소 및 직전 플래시 저장값으로 롤백
 * @return load 결과
 */
bool CL_T2_ConfigManager::revertCancel() {
    _isTuningActive = false;
    return load();
}

/**
 * @brief MLOps 바이너리 헤더 저장 및 연동용 핵심 키 데이터 직렬화 기입
 * @param p_outBuf 타겟 목적지 문자 버퍼 포인터
 * @param p_maxLen 목적지 버퍼 바이트 제한
 */
void CL_T2_ConfigManager::serializeToBuffer(char* p_outBuf, size_t p_maxLen) {
    xSemaphoreTake(_lock, portMAX_DELAY);
    JsonDocument v_doc;  // [이슈 11] g_psramAlloc 제거

    v_doc["ver"]   = T2_Def::Global::System::VERSION_STR;
    v_doc["acc_r"] = _config.accel.range;
    v_doc["acc_s"] = _config.accel.sample_rate;
    v_doc["gyr_r"] = _config.gyro.range;
    v_doc["gyr_s"] = _config.gyro.sample_rate;
    v_doc["aud_s"] = _config.audio.sample_rate;

    size_t v_written = serializeJson(v_doc, p_outBuf, p_maxLen);
    if (v_written < p_maxLen) p_outBuf[v_written] = '\0';
    xSemaphoreGive(_lock);
}
