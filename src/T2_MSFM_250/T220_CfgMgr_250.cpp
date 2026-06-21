/* ============================================================================
 * File: T220_CfgMgr_250.cpp
 * Summary: 4-Tier 동적 JSON 설정 관리자 구현부
 * ============================================================================ */

#include "T220_CfgMgr_250.hpp"
#include <LittleFS.h>
#include <cstring>
#include "esp_log.h"
#include "esp_rom_crc.h"

static const char* TAG = "T220_CFG";

// [이슈 11] g_psramAlloc 제거 — 10KB 미만 JSON을 PSRAM에 담으면 캐시 미스 오버헤드가 발생함. 기본 내부 힙 할당 사용.

// 함수설명: 정밀도 손실 없이 float 인수를 JsonArray에 추가합니다. (p_arr: 대상 배열, p_val: 추가할 값)
static inline void cfgAddFloatExact(JsonArray& p_arr, float p_val) {
    p_arr.add(p_val);
}

// 함수설명: Null 체크를 거쳐 안전하게 값을 복사 반영합니다. (p_v: 소스 variant, p_dest: 목적지 변수 참조)
template<typename T>
static inline void cfgSet(JsonVariantConst p_v, T& p_dest) {
    if (!p_v.isNull()) p_dest = p_v.as<T>();
}

// 함수설명: bitset의 개별 비트 참조(std::bitset::reference)에 대한 cfgSet 특수화 오버로드입니다.
static inline void cfgSet(JsonVariantConst p_v, std::bitset<16>::reference p_dest) {
    if (!p_v.isNull()) p_dest = p_v.as<bool>();
}

// 함수설명: Null 체크를 거친 후 지정 버퍼 크기 내에서 안전하게 문자열을 복사합니다. (p_v: 소스 variant, p_dest: 목적지 버퍼, p_size: 크기)
static inline void cfgStr(JsonVariantConst p_v, char* p_dest, size_t p_size) {
    if (!p_v.isNull()) {
        const char* v_src = p_v.as<const char*>();
        if (v_src) strlcpy(p_dest, v_src, p_size);
    }
}

// 함수설명: 배열 타입의 JSON 입력을 안전하게 파싱하여 배열 메모리에 적재합니다. (p_v: 소스 variant, p_dest: 대상 배열 참조)
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

// 함수설명: DSP 파이프라인의 개별 필터 설정을 JSON 오브젝트로부터 파싱합니다. (p_obj: 소스 JSON 객체, p_dsp: 타겟 구조체 참조)
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

// 함수설명: 메모리(RAM) 상의 DSP 파라미터들을 출력용 JSON 객체로 직렬화 출력합니다. (p_obj: 타겟 JSON 객체, p_dsp: 소스 구조체 참조)
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

// 함수설명: 설정 관리자 생성자이며, Mutex 및 멤버 변수를 기본 초기화합니다.
CL_T2_ConfigManager::CL_T2_ConfigManager() {
    _lock           = xSemaphoreCreateMutex();
    _isLoaded       = false;
    _isDirty        = false;
    _isTuningActive = false;
    _lastModifiedMs = 0;
    _loadDefaults();
}

// 함수설명: 설정 관리자 소멸자이며, 사용된 Mutex 동기화 객체를 해제합니다.
CL_T2_ConfigManager::~CL_T2_ConfigManager() {
    if (_lock) vSemaphoreDelete(_lock);
}

// 함수설명: LittleFS 마운트 상태 검증 및 부팅 시 원자적 복구 작업을 개시합니다. (반환값: 성공 여부)
bool CL_T2_ConfigManager::init() {
    // [신규] WAL 드라이버 초기화
    _walDriver.init();

    // [이슈 4] LittleFS.begin(false): 마운트 실패 시 자동 포맷 금지. 수동 복구 해야 함.
    if (!LittleFS.begin(false)) {
        ESP_LOGE(TAG, "LittleFS Mount Failed! Format may be needed. Aborting to protect data.");
        return false;
    }

    // [처리 단위 1] 백업 파일 config_246.tmp 확인 및 설정 로드 시도
    if (!LittleFS.exists(T2_Def::Global::Path::FILE_CFG_JSON_CONST)) {
        if (LittleFS.exists(T2_Def::Global::Path::FILE_CFG_TMP_CONST)) {
            ESP_LOGW(TAG, "Recovering config from interrupted atomic write");
            LittleFS.rename(T2_Def::Global::Path::FILE_CFG_TMP_CONST, T2_Def::Global::Path::FILE_CFG_JSON_CONST);
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

// 함수설명: 4-Tier 시스템 설정 구조체의 기본값들을 초기 적재합니다.
void CL_T2_ConfigManager::_loadDefaults() {
    // === Tier 1. Global ===
    strlcpy(_dynConfig.system.site_id, T2_Def::Global::System::SITE_ID_DEF, sizeof(_dynConfig.system.site_id));
    _dynConfig.system.tele_hz      = T2_Def::Global::System::TELEMETRY_HZ_DEF;
    _dynConfig.system.wave_hz      = T2_Def::Global::System::WAVEFORM_HZ_DEF;
    _dynConfig.system.op_mode      = T2_Type::EM_OpMode_t::AUTO;
    _dynConfig.system.watchdog_ms  = T2_Def::Global::Task::WDG_TIMEOUT_MS_DEF;

    _dynConfig.wifi.mode = T2_Type::EM_WiFiMode_t::AUTO_FALLBACK;
    strlcpy(_dynConfig.wifi.ap_ssid, T2_Def::Global::Net::WIFI_AP_SSID_DEF, sizeof(_dynConfig.wifi.ap_ssid));
    strlcpy(_dynConfig.wifi.ap_pw,   T2_Def::Global::Net::WIFI_AP_PW_DEF,   sizeof(_dynConfig.wifi.ap_pw));
    strlcpy(_dynConfig.wifi.ap_ip,   T2_Def::Global::Net::WIFI_AP_IP_DEF,   sizeof(_dynConfig.wifi.ap_ip));
    for (uint8_t i = 0; i < T2_Def::Global::NetLimit::NET_MULTI_AP_MAX; i++) {
        _dynConfig.wifi.multi_ssid[i][0] = '\0';
        _dynConfig.wifi.multi_pw[i][0]   = '\0';
    }
    _dynConfig.wifi.disconnect_delay_ms = T2_Def::Global::NetLimit::NET_WIFI_DISCONNECT_DELAY_MS_DEF;  // [이슈 9] WiFi 연결 유지 딜레이

    _dynConfig.mqtt.enable = false;
    strlcpy(_dynConfig.mqtt.broker, "", sizeof(_dynConfig.mqtt.broker));
    _dynConfig.mqtt.port = T2_Def::Global::Net::MQTT_PORT_DEF;
    strlcpy(_dynConfig.mqtt.id, T2_Def::Global::Net::MQTT_ID_DEF, sizeof(_dynConfig.mqtt.id));
    strlcpy(_dynConfig.mqtt.pw, "", sizeof(_dynConfig.mqtt.pw));
    strlcpy(_dynConfig.mqtt.topic_root, T2_Def::Global::Net::MQTT_TOPIC_DEF, sizeof(_dynConfig.mqtt.topic_root));
    strlcpy(_dynConfig.mqtt.lwt_topic, T2_Def::Global::Net::MQTT_LWT_DEF, sizeof(_dynConfig.mqtt.lwt_topic));
    _dynConfig.mqtt.qos        = T2_Def::Global::Net::MQTT_QOS_DEF;
    _dynConfig.mqtt.proto_ver  = T2_Def::Global::Net::MQTT_PROTO_VER_DEF;

    // [이슈 9] NTP 타임서버 기본값 초기화
    strlcpy(_dynConfig.ntp.ntp_server1, T2_Def::Global::Net::NTP_SERVER_1_CONST, sizeof(_dynConfig.ntp.ntp_server1));
    strlcpy(_dynConfig.ntp.ntp_server2, T2_Def::Global::Net::NTP_SERVER_2_CONST, sizeof(_dynConfig.ntp.ntp_server2));
    strlcpy(_dynConfig.ntp.ntp_tz,      T2_Def::Global::Net::NTP_TZ_INFO_CONST,  sizeof(_dynConfig.ntp.ntp_tz));

    _dynConfig.storage.rot_mb       = T2_Def::Global::Storage::ROTATE_MB_DEF;
    _dynConfig.storage.rot_min      = T2_Def::Global::Storage::ROTATE_MIN_DEF;
    _dynConfig.storage.save_raw     = false;
    _dynConfig.storage.keep_max     = T2_Def::Global::Storage::ROTATE_KEEP_MAX_DEF;
    _dynConfig.storage.idle_flush_ms= T2_Def::Global::Storage::IDLE_FLUSH_MS_DEF;
    _dynConfig.storage.pre_trig_sec = T2_Def::Global::Storage::PRE_TRIGGER_SEC_DEF;

    _dynConfig.output.enabled         = true;
    _dynConfig.output.output_sequence  = false;
    _dynConfig.output.sequence_frames  = T2_Def::Global::System::SEQUENCE_FRAMES_DEF;

    // [이슈 8] Decision 파라미터 기본값
    _dynConfig.decision.max_trial_count   = T2_Def::Global::Decision::MAX_TRIAL_COUNT_DEF;
    _dynConfig.decision.sta_lta_threshold = T2_Def::Global::Decision::STA_LTA_THRESHOLD_DEF;
    _dynConfig.decision.min_trigger_count = T2_Def::Global::Decision::MIN_TRIGGER_COUNT_DEF;
    _dynConfig.decision.valid_start_sec   = T2_Def::Global::Decision::VALID_START_SEC_DEF;
    _dynConfig.decision.valid_end_sec     = T2_Def::Global::Decision::VALID_END_SEC_DEF;

    // === Global Trigger & Dsp ===
    _dynConfig.trig_hold_ms   = T2_Def::Global::Trigger::HOLD_TIME_MS_DEF;
    _dynConfig.trig_use_sleep = T2_Def::Global::Trigger::USE_DEEP_SLEEP_DEF;
    _dynConfig.trig_sleep_sec = T2_Def::Global::Trigger::SLEEP_SEC_DEF;

    // === Tier 2. Accel ===
    _dynConfig.accel.enable            = T2_Def::Accel::Sensor::ENABLE_DEF;
    _dynConfig.accel.axis_mask         = T2_Def::Accel::Sensor::AXIS_MASK_DEF;
    _dynConfig.accel.range             = T2_Def::Accel::Sensor::RANGE_DEF;
    _dynConfig.accel.odr               = T2_Def::Accel::Sensor::ODR_REG_DEF;
    _dynConfig.accel.bwp               = T2_Def::Accel::Sensor::BWP_REG_DEF;
    _dynConfig.accel.filter_perf       = T2_Def::Accel::Sensor::PERF_MODE_DEF;
    _dynConfig.accel.fifo_watermark    = T2_Def::Imu::Hardware::FIFO_WATERMARK_LIMIT;
    _dynConfig.accel.sample_rate       = T2_Def::Accel::Sensor::RATE_DEF;
    _dynConfig.accel.fft_size          = T2_Def::Accel::Sensor::FFT_SIZE_DEF;

    _dynConfig.accel.motion_en         = false;
    _dynConfig.accel.wake_g            = T2_Def::Accel::Trigger::WAKE_THRESH_G_DEF;
    _dynConfig.accel.wake_dur          = T2_Def::Global::Trigger::WAKE_DURATION_DEF;

    for (int i = 0; i < T2_Def::Accel::Sensor::AXIS_MAX; i++) {
        _dynConfig.accel.rms_thresh[i]      = T2_Def::Accel::Trigger::RMS_THRESH_DEF;
        _dynConfig.accel.kurt_ng_thresh[i]  = T2_Def::Accel::Trigger::KURT_NG_THRESH_DEF;
        _dynConfig.accel.crest_ng_thresh[i] = T2_Def::Accel::Trigger::CREST_NG_THRESH_DEF;
        _dynConfig.accel.skew_ng_thresh[i]  = T2_Def::Accel::Trigger::SKEW_NG_THRESH_DEF;
        _dynConfig.accel.offset[i]          = 0.0f;
        _dynConfig.accel.gain[i]            = 1.0f;
    }

    _dynConfig.accel.active_band_count = T2_Def::Accel::FeatureLimit::BAND_DEF;
    for (uint8_t i = 0; i < T2_Def::Accel::FeatureLimit::BAND_MAX; i++) {
        _dynConfig.accel.band_en[i] = false;
        if (i < T2_Def::Accel::FeatureLimit::BAND_DEF) {
            _dynConfig.accel.band_start[i] = T2_Def::Accel::Trigger::BAND_RANGES_DEF[i][0];
            _dynConfig.accel.band_end[i]   = T2_Def::Accel::Trigger::BAND_RANGES_DEF[i][1];
        } else {
            _dynConfig.accel.band_start[i] = 0.0f;
            _dynConfig.accel.band_end[i]   = 0.0f;
        }
        for (int a = 0; a < T2_Def::Accel::Sensor::AXIS_MAX; a++) {
            _dynConfig.accel.band_thresh[a][i] = 1.0f;
        }
    }
    _dynConfig.accel.peak_amp_min      = 0.1f;
    _dynConfig.accel.peak_freq_gap_min = 50.0f;

    // Accel DSP Default
    _dynConfig.accel.dsp.rem_dc        = true;
    _dynConfig.accel.dsp.med_en        = true;
    _dynConfig.accel.dsp.med_win       = 3;
    _dynConfig.accel.dsp.hpf.en        = true;
    _dynConfig.accel.dsp.hpf.cutoff    = T2_Def::Accel::Dsp::HPF_CUTOFF_DEF;
    _dynConfig.accel.dsp.hpf.taps      = T2_Def::Accel::FeatureLimit::FIR_TAPS_DEF;
    _dynConfig.accel.dsp.lpf.en        = false;
    _dynConfig.accel.dsp.lpf.cutoff    = T2_Def::Accel::Dsp::LPF_CUTOFF_DEF;
    _dynConfig.accel.dsp.lpf.taps      = T2_Def::Accel::FeatureLimit::FIR_TAPS_DEF;
    _dynConfig.accel.dsp.iir_hpf.en    = false;
    _dynConfig.accel.dsp.iir_hpf.cutoff= 20.0f;
    _dynConfig.accel.dsp.iir_hpf.q     = 0.707f;
    _dynConfig.accel.dsp.iir_lpf.en    = false;
    _dynConfig.accel.dsp.iir_lpf.cutoff= 1000.0f;
    _dynConfig.accel.dsp.iir_lpf.q     = 0.707f;
    _dynConfig.accel.dsp.notch.en      = false;
    _dynConfig.accel.dsp.notch.freq    = T2_Def::Accel::Dsp::NOTCH_FREQ_DEF;
    _dynConfig.accel.dsp.notch.gain    = 1.0f;
    _dynConfig.accel.dsp.notch.q       = T2_Def::Accel::Dsp::NOTCH_Q_DEF;
    _dynConfig.accel.dsp.notch2.en      = false;
    _dynConfig.accel.dsp.notch2.freq    = T2_Def::Accel::Dsp::NOTCH2_FREQ_DEF;
    _dynConfig.accel.dsp.notch2.gain    = 1.0f;
    _dynConfig.accel.dsp.notch2.q       = T2_Def::Accel::Dsp::NOTCH_Q_DEF;
    _dynConfig.accel.dsp.win_type      = T2_Type::EM_WindowType_t::HANN;

    // === Tier 3. Gyro ===
    _dynConfig.gyro.enable             = T2_Def::Gyro::Sensor::ENABLE_DEF;
    _dynConfig.gyro.axis_mask          = T2_Def::Gyro::Sensor::AXIS_MASK_DEF;
    _dynConfig.gyro.range              = T2_Def::Gyro::Sensor::RANGE_DEF;
    _dynConfig.gyro.odr                = T2_Def::Gyro::Sensor::ODR_REG_DEF;
    _dynConfig.gyro.bwp                = T2_Def::Gyro::Sensor::BWP_REG_DEF;
    _dynConfig.gyro.filter_perf        = T2_Def::Gyro::Sensor::PERF_MODE_DEF;
    _dynConfig.gyro.noise_perf         = T2_Def::Gyro::Sensor::PERF_MODE_DEF;
    _dynConfig.gyro.sample_rate        = T2_Def::Gyro::Sensor::RATE_DEF;
    _dynConfig.gyro.fft_size           = T2_Def::Gyro::Sensor::FFT_SIZE_DEF;

    for (int i = 0; i < T2_Def::Gyro::Sensor::AXIS_MAX; i++) {
        _dynConfig.gyro.rms_thresh[i]      = T2_Def::Gyro::Trigger::RMS_THRESH_DEF;
        _dynConfig.gyro.kurt_ng_thresh[i]  = T2_Def::Gyro::Trigger::KURT_NG_THRESH_DEF;
        _dynConfig.gyro.crest_ng_thresh[i] = T2_Def::Gyro::Trigger::CREST_NG_THRESH_DEF;
        _dynConfig.gyro.skew_ng_thresh[i]  = T2_Def::Gyro::Trigger::SKEW_NG_THRESH_DEF;
        _dynConfig.gyro.offset[i]          = 0.0f;
        _dynConfig.gyro.gain[i]            = 1.0f;
    }

    _dynConfig.gyro.active_band_count = T2_Def::Gyro::FeatureLimit::BAND_DEF;
    for (uint8_t i = 0; i < T2_Def::Gyro::FeatureLimit::BAND_MAX; i++) {
        _dynConfig.gyro.band_en[i] = false;
        if (i < T2_Def::Gyro::FeatureLimit::BAND_DEF) {
            _dynConfig.gyro.band_start[i] = T2_Def::Gyro::Trigger::BAND_RANGES_DEF[i][0];
            _dynConfig.gyro.band_end[i]   = T2_Def::Gyro::Trigger::BAND_RANGES_DEF[i][1];
        } else {
            _dynConfig.gyro.band_start[i] = 0.0f;
            _dynConfig.gyro.band_end[i]   = 0.0f;
        }
        for (int a = 0; a < T2_Def::Gyro::Sensor::AXIS_MAX; a++) {
            _dynConfig.gyro.band_thresh[a][i] = 1.0f;
        }
    }
    _dynConfig.gyro.peak_amp_min      = 0.1f;
    _dynConfig.gyro.peak_freq_gap_min = 50.0f;

    // Gyro DSP Default
    _dynConfig.gyro.dsp.rem_dc         = true;
    _dynConfig.gyro.dsp.med_en         = true;
    _dynConfig.gyro.dsp.med_win        = 3;
    _dynConfig.gyro.dsp.hpf.en         = true;
    _dynConfig.gyro.dsp.hpf.cutoff     = T2_Def::Gyro::Dsp::HPF_CUTOFF_DEF;
    _dynConfig.gyro.dsp.hpf.taps       = T2_Def::Gyro::FeatureLimit::FIR_TAPS_DEF;
    _dynConfig.gyro.dsp.lpf.en         = false;
    _dynConfig.gyro.dsp.lpf.cutoff     = T2_Def::Gyro::Dsp::LPF_CUTOFF_DEF;
    _dynConfig.gyro.dsp.lpf.taps       = T2_Def::Gyro::FeatureLimit::FIR_TAPS_DEF;
    _dynConfig.gyro.dsp.iir_hpf.en     = false;
    _dynConfig.gyro.dsp.iir_hpf.cutoff = 20.0f;
    _dynConfig.gyro.dsp.iir_hpf.q      = 0.707f;
    _dynConfig.gyro.dsp.iir_lpf.en     = false;
    _dynConfig.gyro.dsp.iir_lpf.cutoff = 1000.0f;
    _dynConfig.gyro.dsp.iir_lpf.q      = 0.707f;
    _dynConfig.gyro.dsp.notch.en       = false;
    _dynConfig.gyro.dsp.notch.freq     = T2_Def::Gyro::Dsp::NOTCH_FREQ_DEF;
    _dynConfig.gyro.dsp.notch.gain     = 1.0f;
    _dynConfig.gyro.dsp.notch.q        = T2_Def::Gyro::Dsp::NOTCH_Q_DEF;
    _dynConfig.gyro.dsp.notch2.en       = false;
    _dynConfig.gyro.dsp.notch2.freq     = T2_Def::Gyro::Dsp::NOTCH2_FREQ_DEF;
    _dynConfig.gyro.dsp.notch2.gain     = 1.0f;
    _dynConfig.gyro.dsp.notch2.q        = T2_Def::Gyro::Dsp::NOTCH_Q_DEF;
    _dynConfig.gyro.dsp.win_type       = T2_Type::EM_WindowType_t::HANN;

    // === Tier 4. Audio ===
    _dynConfig.audio.enable            = T2_Def::Audio::Sensor::ENABLE_DEF;
    _dynConfig.audio.channel_mask      = T2_Def::Audio::Sensor::CHANNEL_MASK_DEF;
    _dynConfig.audio.sample_rate       = T2_Def::Audio::Sensor::RATE_DEF;
    _dynConfig.audio.fft_size          = T2_Def::Audio::Sensor::FFT_SIZE_DEF;
    _dynConfig.audio.mel_bands         = T2_Def::Audio::FeatureLimit::MEL_BANDS_DEF;

    for (int ch = 0; ch < T2_Def::Audio::Sensor::CHANNELS_MAX; ch++) {
        _dynConfig.audio.rms_thresh[ch]      = T2_Def::Audio::Trigger::RMS_THRESH_DEF;
        _dynConfig.audio.kurt_ng_thresh[ch]  = T2_Def::Audio::Trigger::KURT_NG_THRESH_DEF;
        _dynConfig.audio.crest_ng_thresh[ch] = T2_Def::Audio::Trigger::CREST_NG_THRESH_DEF;
        _dynConfig.audio.skew_ng_thresh[ch]  = T2_Def::Audio::Trigger::SKEW_NG_THRESH_DEF;
        _dynConfig.audio.gain_ch[ch]         = 1.0f;
        memset(_dynConfig.audio.eq_coeffs[ch], 0, sizeof(_dynConfig.audio.eq_coeffs[ch]));
        // [이슈 3] 센터 탭 인덱스를 FIR_TAPS_MAX/2(127)로 정정 (기존 FIR_TAPS_DEF/2=31 오류 수정)
        _dynConfig.audio.eq_coeffs[ch][T2_Def::Audio::FeatureLimit::FIR_TAPS_MAX / 2] = 1.0f;
    }

    _dynConfig.audio.active_band_count = T2_Def::Audio::FeatureLimit::BAND_DEF;
    for (uint8_t i = 0; i < T2_Def::Audio::FeatureLimit::BAND_MAX; i++) {
        _dynConfig.audio.band_en[i] = false;
        if (i < T2_Def::Audio::FeatureLimit::BAND_DEF) {
            _dynConfig.audio.band_start[i] = T2_Def::Audio::Trigger::BAND_RANGES_DEF[i][0];
            _dynConfig.audio.band_end[i]   = T2_Def::Audio::Trigger::BAND_RANGES_DEF[i][1];
        } else {
            _dynConfig.audio.band_start[i] = 0.0f;
            _dynConfig.audio.band_end[i]   = 0.0f;
        }
        for (int ch = 0; ch < T2_Def::Audio::Sensor::CHANNELS_MAX; ch++) {
            _dynConfig.audio.band_thresh[ch][i] = 1.0f;
        }
    }

    _dynConfig.audio.noise.gate_en      = false;
    _dynConfig.audio.noise.gate_thresh  = T2_Def::Audio::Dsp::NOISE_GATE_THRESH_DEF;
    _dynConfig.audio.noise.mode         = T2_Type::EM_NoiseMode_t::OFF;
    _dynConfig.audio.noise.sub_str      = T2_Def::Audio::Dsp::SPECTRAL_SUB_GAIN_DEF;
    _dynConfig.audio.noise.adp_alpha    = T2_Def::Audio::Dsp::NOISE_LEARN_ALPHA_DEF;
    _dynConfig.audio.noise.learn_frames = 100;

    _dynConfig.audio.pre_en             = true;
    _dynConfig.audio.pre_alpha          = T2_Def::Audio::Dsp::PRE_EMPHASIS_ALPHA_DEF;
    _dynConfig.audio.beam_gain          = T2_Def::Audio::Dsp::BEAMFORMING_GAIN_DEF;
    _dynConfig.audio.window_ms          = T2_Def::Audio::Dsp::WINDOW_MS_DEF;
    _dynConfig.audio.hop_ms             = T2_Def::Audio::Dsp::HOP_MS_DEF;

    _dynConfig.audio.auto_idle_min      = T2_Def::Audio::Calib::AUTO_IDLE_MIN_DEF;
    _dynConfig.audio.ref_freq           = T2_Def::Audio::Calib::REF_FREQ_HZ_DEF;
    _dynConfig.audio.filt_min           = T2_Def::Audio::Calib::FILTER_MIN_FREQ_HZ_DEF;
    _dynConfig.audio.filt_max           = T2_Def::Audio::Calib::FILTER_MAX_FREQ_HZ_DEF;
    _dynConfig.audio.gain_max           = T2_Def::Audio::Calib::TARGET_GAIN_MAX_DEF;
    _dynConfig.audio.gain_min           = T2_Def::Audio::Calib::TARGET_GAIN_MIN_DEF;
    _dynConfig.audio.norm_safe          = T2_Def::Audio::Calib::NORM_SAFE_THRESH_DEF;

    _dynConfig.audio.active_ceps_count  = T2_Def::Audio::FeatureLimit::CEPS_TARGET_DEF;
    _dynConfig.audio.active_peak_count  = T2_Def::Audio::FeatureLimit::TOP_PEAKS_DEF;
    for (uint8_t i = 0; i < T2_Def::Audio::FeatureLimit::CEPS_TARGET_MAX; i++) {
        _dynConfig.audio.ceps_targets[i] = 0.0f;
    }
    _dynConfig.audio.f_min              = 100.0f;
    _dynConfig.audio.f_max              = 8000.0f;
    _dynConfig.audio.peak_amp_min       = 0.1f;
    _dynConfig.audio.peak_freq_gap_min  = 50.0f;

    // Audio DSP Default
    _dynConfig.audio.dsp.rem_dc         = true;
    _dynConfig.audio.dsp.med_en         = true;
    _dynConfig.audio.dsp.med_win        = 3;
    _dynConfig.audio.dsp.hpf.en         = true;
    _dynConfig.audio.dsp.hpf.cutoff     = T2_Def::Audio::Dsp::HPF_CUTOFF_DEF;
    _dynConfig.audio.dsp.hpf.taps       = T2_Def::Audio::FeatureLimit::FIR_TAPS_DEF;
    _dynConfig.audio.dsp.lpf.en         = false;
    _dynConfig.audio.dsp.lpf.cutoff     = T2_Def::Audio::Dsp::LPF_CUTOFF_DEF;
    _dynConfig.audio.dsp.lpf.taps       = T2_Def::Audio::FeatureLimit::FIR_TAPS_DEF;
    _dynConfig.audio.dsp.iir_hpf.en     = false;
    _dynConfig.audio.dsp.iir_hpf.cutoff = 20.0f;
    _dynConfig.audio.dsp.iir_hpf.q      = 0.707f;
    _dynConfig.audio.dsp.iir_lpf.en     = false;
    _dynConfig.audio.dsp.iir_lpf.cutoff = 1000.0f;
    _dynConfig.audio.dsp.iir_lpf.q      = 0.707f;
    _dynConfig.audio.dsp.notch.en       = false;
    _dynConfig.audio.dsp.notch.freq     = T2_Def::Audio::Dsp::NOTCH_FREQ_DEF;
    _dynConfig.audio.dsp.notch.gain     = 1.0f;
    _dynConfig.audio.dsp.notch.q        = T2_Def::Audio::Dsp::NOTCH_Q_DEF;
    _dynConfig.audio.dsp.notch2.en       = false;
    _dynConfig.audio.dsp.notch2.freq     = T2_Def::Audio::Dsp::NOTCH2_FREQ_DEF;
    _dynConfig.audio.dsp.notch2.gain     = 1.0f;
    _dynConfig.audio.dsp.notch2.q        = T2_Def::Audio::Dsp::NOTCH_Q_DEF;
    _dynConfig.audio.dsp.win_type       = T2_Type::EM_WindowType_t::HANN;
}

// 함수설명: 수신한 JSON 문서를 4-Tier 시스템 설정 구조체에 적용합니다. (p_doc: 파싱된 JSON 문서 참조)
void CL_T2_ConfigManager::_applyJson(const JsonDocument& p_doc) {
    // [처리 단위 1] system 파라미터 파싱
    JsonObjectConst v_sys = p_doc["system"];
    if (!v_sys.isNull()) {
        cfgStr(v_sys["site_id"], _dynConfig.system.site_id, sizeof(_dynConfig.system.site_id));
        cfgSet(v_sys["tele_hz"], _dynConfig.system.tele_hz);
        cfgSet(v_sys["wave_hz"], _dynConfig.system.wave_hz);
        if (!v_sys["op_mode"].isNull()) {
            _dynConfig.system.op_mode = (T2_Type::EM_OpMode_t)v_sys["op_mode"].as<uint8_t>();
        }
        cfgSet(v_sys["watchdog_ms"], _dynConfig.system.watchdog_ms);
    }

    // [처리 단위 2] wifi 파라미터 파싱
    JsonObjectConst v_wifi = p_doc["wifi"];
    if (!v_wifi.isNull()) {
        if (!v_wifi["mode"].isNull()) {
            _dynConfig.wifi.mode = (T2_Type::EM_WiFiMode_t)v_wifi["mode"].as<uint8_t>();
        }
        cfgStr(v_wifi["ap_ssid"], _dynConfig.wifi.ap_ssid, sizeof(_dynConfig.wifi.ap_ssid));
        cfgStr(v_wifi["ap_pw"],   _dynConfig.wifi.ap_pw,   sizeof(_dynConfig.wifi.ap_pw));
        cfgStr(v_wifi["ap_ip"],   _dynConfig.wifi.ap_ip,   sizeof(_dynConfig.wifi.ap_ip));

        JsonArrayConst v_multi = v_wifi["multi_ap"];
        if (!v_multi.isNull()) {
            uint8_t i = 0;
            for (JsonObjectConst v_ap : v_multi) {
                if (i >= T2_Def::Global::NetLimit::NET_MULTI_AP_MAX) break;
                cfgStr(v_ap["ssid"], _dynConfig.wifi.multi_ssid[i], sizeof(_dynConfig.wifi.multi_ssid[i]));
                cfgStr(v_ap["pw"],   _dynConfig.wifi.multi_pw[i],   sizeof(_dynConfig.wifi.multi_pw[i]));
                i++;
            }
        }
        cfgSet(v_wifi["disconnect_delay_ms"], _dynConfig.wifi.disconnect_delay_ms);  // [이슈 9] WiFi 연결 해제 지연 ms
    }

    // [처리 단위 3] mqtt 파라미터 파싱
    JsonObjectConst v_mqtt = p_doc["mqtt"];
    if (!v_mqtt.isNull()) {
        cfgSet(v_mqtt["enable"],     _dynConfig.mqtt.enable);
        cfgStr(v_mqtt["broker"],     _dynConfig.mqtt.broker, sizeof(_dynConfig.mqtt.broker));
        cfgSet(v_mqtt["port"],       _dynConfig.mqtt.port);
        cfgStr(v_mqtt["id"],         _dynConfig.mqtt.id,     sizeof(_dynConfig.mqtt.id));
        cfgStr(v_mqtt["pw"],         _dynConfig.mqtt.pw,     sizeof(_dynConfig.mqtt.pw));
        cfgStr(v_mqtt["topic_root"], _dynConfig.mqtt.topic_root, sizeof(_dynConfig.mqtt.topic_root));
        cfgStr(v_mqtt["lwt_topic"],  _dynConfig.mqtt.lwt_topic,  sizeof(_dynConfig.mqtt.lwt_topic));
        cfgSet(v_mqtt["qos"],        _dynConfig.mqtt.qos);
        cfgSet(v_mqtt["proto_ver"],  _dynConfig.mqtt.proto_ver);
    }

    // [처리 단위 4] NTP 파라미터 파싱
    JsonObjectConst v_ntp = p_doc["ntp"];
    if (!v_ntp.isNull()) {
        cfgStr(v_ntp["ntp_server1"], _dynConfig.ntp.ntp_server1, sizeof(_dynConfig.ntp.ntp_server1));
        cfgStr(v_ntp["ntp_server2"], _dynConfig.ntp.ntp_server2, sizeof(_dynConfig.ntp.ntp_server2));
        cfgStr(v_ntp["ntp_tz"],      _dynConfig.ntp.ntp_tz,      sizeof(_dynConfig.ntp.ntp_tz));
    }

    // [처리 단위 5] storage 파라미터 파싱 (안전 상한 검증 포함)
    JsonObjectConst v_storage = p_doc["storage"];
    if (!v_storage.isNull()) {
        cfgSet(v_storage["rot_mb"],         _dynConfig.storage.rot_mb);
        cfgSet(v_storage["rot_min"],        _dynConfig.storage.rot_min);
        cfgSet(v_storage["save_raw"],       _dynConfig.storage.save_raw);
        cfgSet(v_storage["keep_max"],       _dynConfig.storage.keep_max);
        cfgSet(v_storage["idle_flush_ms"],  _dynConfig.storage.idle_flush_ms);
        // [이슈 6] pre_trig_sec 상한(10초) 검증 — 과도하게 크면 힙 단편화 및 OOM 발생
        uint8_t v_preTrigsec = _dynConfig.storage.pre_trig_sec;
        cfgSet(v_storage["pre_trig_sec"], v_preTrigsec);
        constexpr uint8_t PRE_TRIG_MAX = 10;
        if (v_preTrigsec > PRE_TRIG_MAX) v_preTrigsec = PRE_TRIG_MAX;
        _dynConfig.storage.pre_trig_sec = v_preTrigsec;
    }

    // [처리 단위 6] output 프레임 설정 파싱
    JsonObjectConst v_out = p_doc["output"];
    if (!v_out.isNull()) {
        cfgSet(v_out["enabled"],         _dynConfig.output.enabled);
        cfgSet(v_out["output_sequence"], _dynConfig.output.output_sequence);
        // [이슈 1] sequence_frames 상한(SEQUENCE_FRAMES_MAX=32) 범위 검증
        uint16_t v_frames = _dynConfig.output.sequence_frames;
        cfgSet(v_out["sequence_frames"], v_frames);
        if (v_frames > T2_Def::Global::System::SEQUENCE_FRAMES_MAX)
            v_frames = T2_Def::Global::System::SEQUENCE_FRAMES_MAX;
        _dynConfig.output.sequence_frames = v_frames;
    }

    // [처리 단위 7] decision 판정 파라미터 파싱
    JsonObjectConst v_dec = p_doc["decision"];
    if (!v_dec.isNull()) {
        cfgSet(v_dec["max_trial_count"],   _dynConfig.decision.max_trial_count);
        cfgSet(v_dec["sta_lta_threshold"], _dynConfig.decision.sta_lta_threshold);
        cfgSet(v_dec["min_trigger_count"], _dynConfig.decision.min_trigger_count);
        cfgSet(v_dec["valid_start_sec"],   _dynConfig.decision.valid_start_sec);
        cfgSet(v_dec["valid_end_sec"],     _dynConfig.decision.valid_end_sec);
        // 범위 안전 제한
        if (_dynConfig.decision.max_trial_count > T2_Def::Global::Decision::MAX_TRIAL_COUNT_MAX)
            _dynConfig.decision.max_trial_count = T2_Def::Global::Decision::MAX_TRIAL_COUNT_MAX;
    }

    // [처리 단위 8] trigger 글로벌 슬립 및 홀드 파라미터 파싱
    JsonObjectConst v_trig = p_doc["trigger"];
    if (!v_trig.isNull()) {
        cfgSet(v_trig["hold_ms"],   _dynConfig.trig_hold_ms);
        cfgSet(v_trig["use_sleep"], _dynConfig.trig_use_sleep);
        cfgSet(v_trig["sleep_sec"], _dynConfig.trig_sleep_sec);
    }

    // [처리 단위 9] accel 가속도 센서 대역 및 DSP 파라미터 파싱
    JsonObjectConst v_acc = p_doc["accel"];
    if (!v_acc.isNull()) {
        cfgSet(v_acc["enable"],         _dynConfig.accel.enable);
        cfgSet(v_acc["axis_mask"],      _dynConfig.accel.axis_mask);
        cfgSet(v_acc["range"],          _dynConfig.accel.range);
        cfgSet(v_acc["odr"],            _dynConfig.accel.odr);
        cfgSet(v_acc["bwp"],            _dynConfig.accel.bwp);
        cfgSet(v_acc["filter_perf"],    _dynConfig.accel.filter_perf);
        cfgSet(v_acc["fifo_watermark"], _dynConfig.accel.fifo_watermark);
        cfgSet(v_acc["sample_rate"],    _dynConfig.accel.sample_rate);
        cfgSet(v_acc["fft_size"],       _dynConfig.accel.fft_size);
        // [이슈 1] fft_size 유효성 검증: 2의 거듭제곱 및 FFT_SIZE_MAX 이하 확인
        {
            uint32_t v = _dynConfig.accel.fft_size;
            auto isPow2 = [](uint32_t x){ return x > 0 && (x & (x-1)) == 0; };
            if (!isPow2(v) || v > T2_Def::Accel::Sensor::FFT_SIZE_MAX)
                _dynConfig.accel.fft_size = T2_Def::Accel::Sensor::FFT_SIZE_DEF;
        }
        cfgSet(v_acc["noise_perf"],     _dynConfig.accel.noise_perf);  // [이슈 18] 가속도 저소음 모드 연동

        cfgSet(v_acc["motion_en"],      _dynConfig.accel.motion_en);
        cfgSet(v_acc["wake_g"],         _dynConfig.accel.wake_g);
        cfgSet(v_acc["wake_dur"],       _dynConfig.accel.wake_dur);

        cfgSetArray(v_acc["rms_thresh"],      _dynConfig.accel.rms_thresh);
        cfgSetArray(v_acc["kurt_ng_thresh"],  _dynConfig.accel.kurt_ng_thresh);
        cfgSetArray(v_acc["crest_ng_thresh"], _dynConfig.accel.crest_ng_thresh);
        cfgSetArray(v_acc["skew_ng_thresh"],  _dynConfig.accel.skew_ng_thresh);

        JsonArrayConst v_bands = v_acc["bands"];
        if (!v_bands.isNull()) {
            uint8_t i = 0;
            for (JsonObjectConst v_b : v_bands) {
                if (i >= T2_Def::Accel::FeatureLimit::BAND_MAX) break;
                cfgSet(v_b["en"],    _dynConfig.accel.band_en[i]);
                cfgSet(v_b["start"], _dynConfig.accel.band_start[i]);
                cfgSet(v_b["end"],   _dynConfig.accel.band_end[i]);

                if (!v_b["thresh"].isNull()) {
                    if (v_b["thresh"].is<JsonArrayConst>()) {
                        JsonArrayConst v_arr = v_b["thresh"].as<JsonArrayConst>();
                        for (size_t a = 0; a < T2_Def::Accel::Sensor::AXIS_MAX; a++) {
                            if (a < v_arr.size()) _dynConfig.accel.band_thresh[a][i] = v_arr[a].as<float>();
                        }
                    } else {
                        float v_val = v_b["thresh"].as<float>();
                        for (size_t a = 0; a < T2_Def::Accel::Sensor::AXIS_MAX; a++) {
                            _dynConfig.accel.band_thresh[a][i] = v_val;
                        }
                    }
                }
                i++;
            }
            _dynConfig.accel.active_band_count = i;
        }

        cfgSetArray(v_acc["offset"], _dynConfig.accel.offset);
        cfgSetArray(v_acc["gain"],   _dynConfig.accel.gain);
        cfgSet(v_acc["peak_amp_min"],      _dynConfig.accel.peak_amp_min);
        cfgSet(v_acc["peak_freq_gap_min"], _dynConfig.accel.peak_freq_gap_min);

        parseDspConfig(v_acc["dsp"], _dynConfig.accel.dsp);
    }

    // [처리 단위 10] gyro 자이로 센서 파라미터 파싱
    JsonObjectConst v_gyr = p_doc["gyro"];
    if (!v_gyr.isNull()) {
        cfgSet(v_gyr["enable"],      _dynConfig.gyro.enable);
        cfgSet(v_gyr["axis_mask"],   _dynConfig.gyro.axis_mask);
        cfgSet(v_gyr["range"],       _dynConfig.gyro.range);
        cfgSet(v_gyr["odr"],         _dynConfig.gyro.odr);
        cfgSet(v_gyr["bwp"],         _dynConfig.gyro.bwp);
        cfgSet(v_gyr["filter_perf"], _dynConfig.gyro.filter_perf);
        cfgSet(v_gyr["noise_perf"],  _dynConfig.gyro.noise_perf);
        cfgSet(v_gyr["sample_rate"], _dynConfig.gyro.sample_rate);
        cfgSet(v_gyr["fft_size"],    _dynConfig.gyro.fft_size);

        cfgSetArray(v_gyr["rms_thresh"],      _dynConfig.gyro.rms_thresh);
        cfgSetArray(v_gyr["kurt_ng_thresh"],  _dynConfig.gyro.kurt_ng_thresh);
        cfgSetArray(v_gyr["crest_ng_thresh"], _dynConfig.gyro.crest_ng_thresh);
        cfgSetArray(v_gyr["skew_ng_thresh"],  _dynConfig.gyro.skew_ng_thresh);

        JsonArrayConst v_bands = v_gyr["bands"];
        if (!v_bands.isNull()) {
            uint8_t i = 0;
            for (JsonObjectConst v_b : v_bands) {
                if (i >= T2_Def::Gyro::FeatureLimit::BAND_MAX) break;
                cfgSet(v_b["en"],    _dynConfig.gyro.band_en[i]);
                cfgSet(v_b["start"], _dynConfig.gyro.band_start[i]);
                cfgSet(v_b["end"],   _dynConfig.gyro.band_end[i]);

                if (!v_b["thresh"].isNull()) {
                    if (v_b["thresh"].is<JsonArrayConst>()) {
                        JsonArrayConst v_arr = v_b["thresh"].as<JsonArrayConst>();
                        for (size_t a = 0; a < T2_Def::Gyro::Sensor::AXIS_MAX; a++) {
                            if (a < v_arr.size()) _dynConfig.gyro.band_thresh[a][i] = v_arr[a].as<float>();
                        }
                    } else {
                        float v_val = v_b["thresh"].as<float>();
                        for (size_t a = 0; a < T2_Def::Gyro::Sensor::AXIS_MAX; a++) {
                            _dynConfig.gyro.band_thresh[a][i] = v_val;
                        }
                    }
                }
                i++;
            }
            _dynConfig.gyro.active_band_count = i;
        }

        cfgSetArray(v_gyr["offset"], _dynConfig.gyro.offset);
        cfgSetArray(v_gyr["gain"],   _dynConfig.gyro.gain);
        cfgSet(v_gyr["peak_amp_min"],      _dynConfig.gyro.peak_amp_min);
        cfgSet(v_gyr["peak_freq_gap_min"], _dynConfig.gyro.peak_freq_gap_min);

        parseDspConfig(v_gyr["dsp"], _dynConfig.gyro.dsp);
    }

    // [처리 단위 11] audio 마이크 센서 및 멜 대역, 노이즈게이트 등 파이프라인 파싱
    JsonObjectConst v_aud = p_doc["audio"];
    if (!v_aud.isNull()) {
        cfgSet(v_aud["enable"],       _dynConfig.audio.enable);
        cfgSet(v_aud["channel_mask"], _dynConfig.audio.channel_mask);
        cfgSet(v_aud["sample_rate"],  _dynConfig.audio.sample_rate);
        cfgSet(v_aud["fft_size"],     _dynConfig.audio.fft_size);

        uint8_t v_melBands = _dynConfig.audio.mel_bands;
        cfgSet(v_aud["mel_bands"], v_melBands);
        if (v_melBands == 0 || v_melBands > T2_Def::Audio::FeatureLimit::MEL_BANDS_MAX) {
            v_melBands = T2_Def::Audio::FeatureLimit::MEL_BANDS_DEF;
        }
        _dynConfig.audio.mel_bands = v_melBands;

        cfgSetArray(v_aud["rms_thresh"],      _dynConfig.audio.rms_thresh);
        cfgSetArray(v_aud["kurt_ng_thresh"],  _dynConfig.audio.kurt_ng_thresh);
        cfgSetArray(v_aud["crest_ng_thresh"], _dynConfig.audio.crest_ng_thresh);
        cfgSetArray(v_aud["skew_ng_thresh"],  _dynConfig.audio.skew_ng_thresh);

        JsonArrayConst v_bands = v_aud["bands"];
        if (!v_bands.isNull()) {
            uint8_t i = 0;
            for (JsonObjectConst v_b : v_bands) {
                if (i >= T2_Def::Audio::FeatureLimit::BAND_MAX) break;
                cfgSet(v_b["en"],    _dynConfig.audio.band_en[i]);
                cfgSet(v_b["start"], _dynConfig.audio.band_start[i]);
                cfgSet(v_b["end"],   _dynConfig.audio.band_end[i]);

                if (!v_b["thresh"].isNull()) {
                    if (v_b["thresh"].is<JsonArrayConst>()) {
                        JsonArrayConst v_arr = v_b["thresh"].as<JsonArrayConst>();
                        for (size_t ch = 0; ch < T2_Def::Audio::Sensor::CHANNELS_MAX; ch++) {
                            if (ch < v_arr.size()) _dynConfig.audio.band_thresh[ch][i] = v_arr[ch].as<float>();
                        }
                    } else {
                        float v_val = v_b["thresh"].as<float>();
                        for (size_t ch = 0; ch < T2_Def::Audio::Sensor::CHANNELS_MAX; ch++) {
                            _dynConfig.audio.band_thresh[ch][i] = v_val;
                        }
                    }
                }
                i++;
            }
            _dynConfig.audio.active_band_count = i;
        }

        JsonObjectConst v_noise = v_aud["noise"];
        if (!v_noise.isNull()) {
            cfgSet(v_noise["gate_en"],     _dynConfig.audio.noise.gate_en);
            cfgSet(v_noise["gate_thresh"], _dynConfig.audio.noise.gate_thresh);
            if (!v_noise["mode"].isNull()) {
                _dynConfig.audio.noise.mode = (T2_Type::EM_NoiseMode_t)v_noise["mode"].as<uint8_t>();
            }
            cfgSet(v_noise["sub_str"],     _dynConfig.audio.noise.sub_str);
            cfgSet(v_noise["adp_alpha"],   _dynConfig.audio.noise.adp_alpha);
            cfgSet(v_noise["learn_frames"],_dynConfig.audio.noise.learn_frames);
        }

        cfgSet(v_aud["pre_en"],    _dynConfig.audio.pre_en);
        cfgSet(v_aud["pre_alpha"], _dynConfig.audio.pre_alpha);
        // [이슈 7] beam_gain (0.0f ~ 1.0f) 범위 유효성 검증 추가
        float v_beamGain = _dynConfig.audio.beam_gain;
        cfgSet(v_aud["beam_gain"], v_beamGain);
        if (v_beamGain < 0.0f) v_beamGain = 0.0f;
        if (v_beamGain > 1.0f) v_beamGain = 1.0f;
        _dynConfig.audio.beam_gain = v_beamGain;
        cfgSet(v_aud["window_ms"], _dynConfig.audio.window_ms);
        cfgSet(v_aud["hop_ms"],    _dynConfig.audio.hop_ms);

        cfgSet(v_aud["auto_idle_min"], _dynConfig.audio.auto_idle_min);
        cfgSet(v_aud["ref_freq"],      _dynConfig.audio.ref_freq);
        cfgSet(v_aud["filt_min"],      _dynConfig.audio.filt_min);
        cfgSet(v_aud["filt_max"],      _dynConfig.audio.filt_max);
        cfgSet(v_aud["gain_max"],      _dynConfig.audio.gain_max);
        cfgSet(v_aud["gain_min"],      _dynConfig.audio.gain_min);
        cfgSet(v_aud["norm_safe"],     _dynConfig.audio.norm_safe);
        cfgSetArray(v_aud["gain_ch"],  _dynConfig.audio.gain_ch);

        JsonArrayConst v_eq = v_aud["eq_coeffs"];
        if (!v_eq.isNull()) {
            if (v_eq[0].is<JsonArrayConst>()) {
                for (size_t ch = 0; ch < T2_Def::Audio::Sensor::CHANNELS_MAX; ch++) {
                    JsonArrayConst v_chEq = v_eq[ch].as<JsonArrayConst>();
                    size_t idx = 0;
                    for (JsonVariantConst v_val : v_chEq) {
                        if (idx >= T2_Def::Audio::FeatureLimit::FIR_TAPS_MAX) break;
                        _dynConfig.audio.eq_coeffs[ch][idx] = v_val.as<float>();
                        idx++;
                    }
                }
            } else {
                size_t idx = 0;
                for (JsonVariantConst v_val : v_eq) {
                    if (idx >= T2_Def::Audio::FeatureLimit::FIR_TAPS_MAX) break;
                    float v_fVal = v_val.as<float>();
                    for (size_t ch = 0; ch < T2_Def::Audio::Sensor::CHANNELS_MAX; ch++) {
                        _dynConfig.audio.eq_coeffs[ch][idx] = v_fVal;
                    }
                    idx++;
                }
            }
        }

        cfgSet(v_aud["active_ceps_count"], _dynConfig.audio.active_ceps_count);
        cfgSet(v_aud["active_peak_count"], _dynConfig.audio.active_peak_count);

        JsonArrayConst v_ceps = v_aud["ceps_targets"];
        if (!v_ceps.isNull()) {
            uint8_t idx = 0;
            for (JsonVariantConst v_val : v_ceps) {
                if (idx >= T2_Def::Audio::FeatureLimit::CEPS_TARGET_MAX) break;
                _dynConfig.audio.ceps_targets[idx] = v_val.as<float>();
                idx++;
            }
            _dynConfig.audio.active_ceps_count = idx;
        }

        cfgSet(v_aud["f_min"],             _dynConfig.audio.f_min);
        cfgSet(v_aud["f_max"],             _dynConfig.audio.f_max);
        cfgSet(v_aud["peak_amp_min"],      _dynConfig.audio.peak_amp_min);
        cfgSet(v_aud["peak_freq_gap_min"], _dynConfig.audio.peak_freq_gap_min);

        parseDspConfig(v_aud["dsp"], _dynConfig.audio.dsp);
    }
}

// 함수설명: LittleFS 파일시스템에서 JSON 설정 파일을 읽어와 동적 설정 구조체에 반영합니다. (반환값: 성공 여부)
bool CL_T2_ConfigManager::load() {
    xSemaphoreTake(_lock, portMAX_DELAY);

    // [신규] WAL 파티션에서 최신 설정을 먼저 로드 시도
    if (_walDriver.loadLatestConfig(_dynConfig)) {
        _isLoaded = true;
        xSemaphoreGive(_lock);
        return true;
    }

    File v_file = LittleFS.open(T2_Def::Global::Path::FILE_CFG_JSON_CONST, "r");
    if (!v_file) {
        ESP_LOGE(TAG, "Config load failed");
        xSemaphoreGive(_lock);
        return false;
    }

    // [처리 단위 1] JSON 파일 역직렬화 수행
    JsonDocument v_doc;
    DeserializationError v_err = deserializeJson(v_doc, v_file);
    v_file.close();

    if (v_err) {
        ESP_LOGE(TAG, "JSON Parse Error: %s", v_err.c_str());
        xSemaphoreGive(_lock);
        return false;
    }

    // [처리 단위 2] 역직렬화된 JSON 데이터를 구조체 멤버에 반영
    _applyJson(v_doc);
    xSemaphoreGive(_lock);
    return true;
}

// 함수설명: 현재 동적 설정 구조체의 멤버 값들을 JSON 문서로 직렬화하여 LittleFS 설정 파일에 저장합니다. (반환값: 성공 여부)
// ============================================================================
// [수정/추가] 누락된 ntp, decision, wifi 딜레이 설정 반영 및 eq_coeffs 배열 잘림 현상 수정
// ============================================================================
bool CL_T2_ConfigManager::save() {
    xSemaphoreTake(_lock, portMAX_DELAY);
    JsonDocument v_doc;

    // [처리 단위 1] Tier 1. Global 설정 직렬화
    JsonObject v_sys = v_doc["system"].to<JsonObject>();
    v_sys["site_id"]      = _dynConfig.system.site_id;
    v_sys["tele_hz"]      = _dynConfig.system.tele_hz;
    v_sys["wave_hz"]      = _dynConfig.system.wave_hz;
    v_sys["op_mode"]      = (uint8_t)_dynConfig.system.op_mode;
    v_sys["watchdog_ms"]  = _dynConfig.system.watchdog_ms;

    JsonObject v_wifi = v_doc["wifi"].to<JsonObject>();
    v_wifi["mode"]    = (uint8_t)_dynConfig.wifi.mode;
    v_wifi["ap_ssid"] = _dynConfig.wifi.ap_ssid;
    v_wifi["ap_pw"]   = _dynConfig.wifi.ap_pw;
    v_wifi["ap_ip"]   = _dynConfig.wifi.ap_ip;
    JsonArray v_multi = v_wifi["multi_ap"].to<JsonArray>();
    for (uint8_t i = 0; i < T2_Def::Global::NetLimit::NET_MULTI_AP_MAX; i++) {
        JsonObject v_ap = v_multi.add<JsonObject>();
        v_ap["ssid"] = _dynConfig.wifi.multi_ssid[i];
        v_ap["pw"]   = _dynConfig.wifi.multi_pw[i];
    }
    // [추가] 누락되었던 WiFi 재연결 지연 설정값 반영
    v_wifi["disconnect_delay_ms"] = _dynConfig.wifi.disconnect_delay_ms;

    JsonObject v_mqtt = v_doc["mqtt"].to<JsonObject>();
    v_mqtt["enable"]     = _dynConfig.mqtt.enable;
    v_mqtt["broker"]     = _dynConfig.mqtt.broker;
    v_mqtt["port"]       = _dynConfig.mqtt.port;
    v_mqtt["id"]         = _dynConfig.mqtt.id;
    v_mqtt["pw"]         = _dynConfig.mqtt.pw;
    v_mqtt["topic_root"] = _dynConfig.mqtt.topic_root;
    v_mqtt["lwt_topic"]  = _dynConfig.mqtt.lwt_topic;
    v_mqtt["qos"]        = _dynConfig.mqtt.qos;
    v_mqtt["proto_ver"]  = _dynConfig.mqtt.proto_ver;

    // [추가] 완전히 누락되었던 NTP 설정 블록 직렬화 반영
    JsonObject v_ntp = v_doc["ntp"].to<JsonObject>();
    v_ntp["ntp_server1"] = _dynConfig.ntp.ntp_server1;
    v_ntp["ntp_server2"] = _dynConfig.ntp.ntp_server2;
    v_ntp["ntp_tz"]      = _dynConfig.ntp.ntp_tz;

    JsonObject v_storage = v_doc["storage"].to<JsonObject>();
    v_storage["rot_mb"]         = _dynConfig.storage.rot_mb;
    v_storage["rot_min"]        = _dynConfig.storage.rot_min;
    v_storage["save_raw"]       = _dynConfig.storage.save_raw;
    v_storage["keep_max"]       = _dynConfig.storage.keep_max;
    v_storage["idle_flush_ms"]  = _dynConfig.storage.idle_flush_ms;
    v_storage["pre_trig_sec"]   = _dynConfig.storage.pre_trig_sec;

    JsonObject v_out = v_doc["output"].to<JsonObject>();
    v_out["enabled"]         = _dynConfig.output.enabled;
    v_out["output_sequence"] = _dynConfig.output.output_sequence;
    v_out["sequence_frames"] = _dynConfig.output.sequence_frames;

    // [추가] 완전히 누락되었던 Decision (MLOps 판정) 파라미터 블록 직렬화 반영
    JsonObject v_dec = v_doc["decision"].to<JsonObject>();
    v_dec["max_trial_count"]   = _dynConfig.decision.max_trial_count;
    v_dec["sta_lta_threshold"] = _dynConfig.decision.sta_lta_threshold;
    v_dec["min_trigger_count"] = _dynConfig.decision.min_trigger_count;
    v_dec["valid_start_sec"]   = _dynConfig.decision.valid_start_sec;
    v_dec["valid_end_sec"]     = _dynConfig.decision.valid_end_sec;

    // [처리 단위 2] Global Trigger 설정 직렬화
    JsonObject v_trig = v_doc["trigger"].to<JsonObject>();
    v_trig["hold_ms"]   = _dynConfig.trig_hold_ms;
    v_trig["use_sleep"] = _dynConfig.trig_use_sleep;
    v_trig["sleep_sec"] = _dynConfig.trig_sleep_sec;

    // [처리 단위 3] Tier 2. Accel 가속도 설정 직렬화
    JsonObject v_acc = v_doc["accel"].to<JsonObject>();
    v_acc["enable"]         = _dynConfig.accel.enable;
    v_acc["axis_mask"]      = _dynConfig.accel.axis_mask;
    v_acc["range"]          = _dynConfig.accel.range;
    v_acc["odr"]            = _dynConfig.accel.odr;
    v_acc["bwp"]            = _dynConfig.accel.bwp;
    v_acc["filter_perf"]    = _dynConfig.accel.filter_perf;
    v_acc["fifo_watermark"] = _dynConfig.accel.fifo_watermark;
    v_acc["sample_rate"]    = _dynConfig.accel.sample_rate;
    v_acc["fft_size"]       = _dynConfig.accel.fft_size;
    v_acc["noise_perf"]     = _dynConfig.accel.noise_perf; // [추가] Accel 노이즈 모드 직렬화

    v_acc["motion_en"]      = _dynConfig.accel.motion_en;
    v_acc["wake_g"]         = _dynConfig.accel.wake_g;
    v_acc["wake_dur"]       = _dynConfig.accel.wake_dur;

    JsonArray v_rmsThV = v_acc["rms_thresh"].to<JsonArray>();
    for(int a=0; a<T2_Def::Accel::Sensor::AXIS_MAX; a++) cfgAddFloatExact(v_rmsThV, _dynConfig.accel.rms_thresh[a]);
    JsonArray v_kurtThV = v_acc["kurt_ng_thresh"].to<JsonArray>();
    for(int a=0; a<T2_Def::Accel::Sensor::AXIS_MAX; a++) cfgAddFloatExact(v_kurtThV, _dynConfig.accel.kurt_ng_thresh[a]);
    JsonArray v_crestThV = v_acc["crest_ng_thresh"].to<JsonArray>();
    for(int a=0; a<T2_Def::Accel::Sensor::AXIS_MAX; a++) cfgAddFloatExact(v_crestThV, _dynConfig.accel.crest_ng_thresh[a]);
    JsonArray v_skewThV = v_acc["skew_ng_thresh"].to<JsonArray>();
    for(int a=0; a<T2_Def::Accel::Sensor::AXIS_MAX; a++) cfgAddFloatExact(v_skewThV, _dynConfig.accel.skew_ng_thresh[a]);

    JsonArray v_vbands = v_acc["bands"].to<JsonArray>();
    for (uint8_t i = 0; i < _dynConfig.accel.active_band_count; i++) {
        JsonObject v_b = v_vbands.add<JsonObject>();
        v_b["en"]    = (bool)_dynConfig.accel.band_en[i];
        v_b["start"] = _dynConfig.accel.band_start[i];
        v_b["end"]   = _dynConfig.accel.band_end[i];

        JsonArray v_thArr = v_b["thresh"].to<JsonArray>();
        for(int a=0; a<T2_Def::Accel::Sensor::AXIS_MAX; a++) cfgAddFloatExact(v_thArr, _dynConfig.accel.band_thresh[a][i]);
    }

    JsonArray v_vcoff = v_acc["offset"].to<JsonArray>();
    for(uint8_t i=0; i<T2_Def::Accel::Sensor::AXIS_MAX; i++) cfgAddFloatExact(v_vcoff, _dynConfig.accel.offset[i]);
    JsonArray v_vcgain = v_acc["gain"].to<JsonArray>();
    for(uint8_t i=0; i<T2_Def::Accel::Sensor::AXIS_MAX; i++) cfgAddFloatExact(v_vcgain, _dynConfig.accel.gain[i]);

    v_acc["peak_amp_min"]      = _dynConfig.accel.peak_amp_min;
    v_acc["peak_freq_gap_min"] = _dynConfig.accel.peak_freq_gap_min;

    serializeDspConfig(v_acc["dsp"].to<JsonObject>(), _dynConfig.accel.dsp);

    // [처리 단위 4] Tier 3. Gyro 자이로 설정 직렬화
    JsonObject v_gyr = v_doc["gyro"].to<JsonObject>();
    v_gyr["enable"]      = _dynConfig.gyro.enable;
    v_gyr["axis_mask"]   = _dynConfig.gyro.axis_mask;
    v_gyr["range"]       = _dynConfig.gyro.range;
    v_gyr["odr"]         = _dynConfig.gyro.odr;
    v_gyr["bwp"]         = _dynConfig.gyro.bwp;
    v_gyr["filter_perf"] = _dynConfig.gyro.filter_perf;
    v_gyr["noise_perf"]  = _dynConfig.gyro.noise_perf;
    v_gyr["sample_rate"] = _dynConfig.gyro.sample_rate;
    v_gyr["fft_size"]    = _dynConfig.gyro.fft_size;

    JsonArray v_rmsThG = v_gyr["rms_thresh"].to<JsonArray>();
    for(int a=0; a<T2_Def::Gyro::Sensor::AXIS_MAX; a++) cfgAddFloatExact(v_rmsThG, _dynConfig.gyro.rms_thresh[a]);
    JsonArray v_kurtThG = v_gyr["kurt_ng_thresh"].to<JsonArray>();
    for(int a=0; a<T2_Def::Gyro::Sensor::AXIS_MAX; a++) cfgAddFloatExact(v_kurtThG, _dynConfig.gyro.kurt_ng_thresh[a]);
    JsonArray v_crestThG = v_gyr["crest_ng_thresh"].to<JsonArray>();
    for(int a=0; a<T2_Def::Gyro::Sensor::AXIS_MAX; a++) cfgAddFloatExact(v_crestThG, _dynConfig.gyro.crest_ng_thresh[a]);
    JsonArray v_skewThG = v_gyr["skew_ng_thresh"].to<JsonArray>();
    for(int a=0; a<T2_Def::Gyro::Sensor::AXIS_MAX; a++) cfgAddFloatExact(v_skewThG, _dynConfig.gyro.skew_ng_thresh[a]);

    JsonArray v_gbands = v_gyr["bands"].to<JsonArray>();
    for (uint8_t i = 0; i < _dynConfig.gyro.active_band_count; i++) {
        JsonObject v_b = v_gbands.add<JsonObject>();
        v_b["en"]    = (bool)_dynConfig.gyro.band_en[i];
        v_b["start"] = _dynConfig.gyro.band_start[i];
        v_b["end"]   = _dynConfig.gyro.band_end[i];

        JsonArray v_thArr = v_b["thresh"].to<JsonArray>();
        for(int a=0; a<T2_Def::Gyro::Sensor::AXIS_MAX; a++) cfgAddFloatExact(v_thArr, _dynConfig.gyro.band_thresh[a][i]);
    }

    JsonArray v_gcoff = v_gyr["offset"].to<JsonArray>();
    for(uint8_t i=0; i<T2_Def::Gyro::Sensor::AXIS_MAX; i++) cfgAddFloatExact(v_gcoff, _dynConfig.gyro.offset[i]);
    JsonArray v_gcgain = v_gyr["gain"].to<JsonArray>();
    for(uint8_t i=0; i<T2_Def::Gyro::Sensor::AXIS_MAX; i++) cfgAddFloatExact(v_gcgain, _dynConfig.gyro.gain[i]);

    v_gyr["peak_amp_min"]      = _dynConfig.gyro.peak_amp_min;
    v_gyr["peak_freq_gap_min"] = _dynConfig.gyro.peak_freq_gap_min;

    serializeDspConfig(v_gyr["dsp"].to<JsonObject>(), _dynConfig.gyro.dsp);

    // [처리 단위 5] Tier 4. Audio 설정 직렬화
    JsonObject v_aud = v_doc["audio"].to<JsonObject>();
    v_aud["enable"]       = _dynConfig.audio.enable;
    v_aud["channel_mask"] = _dynConfig.audio.channel_mask;
    v_aud["sample_rate"]  = _dynConfig.audio.sample_rate;
    v_aud["fft_size"]     = _dynConfig.audio.fft_size;
    v_aud["mel_bands"]    = _dynConfig.audio.mel_bands;

    JsonArray v_rmsThA = v_aud["rms_thresh"].to<JsonArray>();
    for(int ch=0; ch<T2_Def::Audio::Sensor::CHANNELS_MAX; ch++) cfgAddFloatExact(v_rmsThA, _dynConfig.audio.rms_thresh[ch]);
    JsonArray v_kurtThA = v_aud["kurt_ng_thresh"].to<JsonArray>();
    for(int ch=0; ch<T2_Def::Audio::Sensor::CHANNELS_MAX; ch++) cfgAddFloatExact(v_kurtThA, _dynConfig.audio.kurt_ng_thresh[ch]);
    JsonArray v_crestThA = v_aud["crest_ng_thresh"].to<JsonArray>();
    for(int ch=0; ch<T2_Def::Audio::Sensor::CHANNELS_MAX; ch++) cfgAddFloatExact(v_crestThA, _dynConfig.audio.crest_ng_thresh[ch]);
    JsonArray v_skewThA = v_aud["skew_ng_thresh"].to<JsonArray>();
    for(int ch=0; ch<T2_Def::Audio::Sensor::CHANNELS_MAX; ch++) cfgAddFloatExact(v_skewThA, _dynConfig.audio.skew_ng_thresh[ch]);

    JsonArray v_abands = v_aud["bands"].to<JsonArray>();
    for (uint8_t i = 0; i < _dynConfig.audio.active_band_count; i++) {
        JsonObject v_b = v_abands.add<JsonObject>();
        v_b["en"]    = (bool)_dynConfig.audio.band_en[i];
        v_b["start"] = _dynConfig.audio.band_start[i];
        v_b["end"]   = _dynConfig.audio.band_end[i];

        JsonArray v_thArr = v_b["thresh"].to<JsonArray>();
        for(int ch=0; ch<T2_Def::Audio::Sensor::CHANNELS_MAX; ch++) cfgAddFloatExact(v_thArr, _dynConfig.audio.band_thresh[ch][i]);
    }

    JsonObject v_noise = v_aud["noise"].to<JsonObject>();
    v_noise["gate_en"]     = _dynConfig.audio.noise.gate_en;
    v_noise["gate_thresh"] = _dynConfig.audio.noise.gate_thresh;
    v_noise["mode"]        = (uint8_t)_dynConfig.audio.noise.mode;
    v_noise["sub_str"]     = _dynConfig.audio.noise.sub_str;
    v_noise["adp_alpha"]   = _dynConfig.audio.noise.adp_alpha;
    v_noise["learn_frames"]= _dynConfig.audio.noise.learn_frames;

    v_aud["pre_en"]    = _dynConfig.audio.pre_en;
    v_aud["pre_alpha"] = _dynConfig.audio.pre_alpha;
    v_aud["beam_gain"] = _dynConfig.audio.beam_gain;
    v_aud["window_ms"] = _dynConfig.audio.window_ms;
    v_aud["hop_ms"]    = _dynConfig.audio.hop_ms;

    v_aud["auto_idle_min"] = _dynConfig.audio.auto_idle_min;
    v_aud["ref_freq"]      = _dynConfig.audio.ref_freq;
    v_aud["filt_min"]      = _dynConfig.audio.filt_min;
    v_aud["filt_max"]      = _dynConfig.audio.filt_max;
    v_aud["gain_max"]      = _dynConfig.audio.gain_max;
    v_aud["gain_min"]      = _dynConfig.audio.gain_min;
    v_aud["norm_safe"]     = _dynConfig.audio.norm_safe;

    JsonArray v_gch = v_aud["gain_ch"].to<JsonArray>();
    for (int ch = 0; ch < T2_Def::Audio::Sensor::CHANNELS_MAX; ch++) v_gch.add(_dynConfig.audio.gain_ch[ch]);

    JsonArray v_eq = v_aud["eq_coeffs"].to<JsonArray>();
    for (size_t ch = 0; ch < T2_Def::Audio::Sensor::CHANNELS_MAX; ch++) {
        JsonArray v_chEq = v_eq.add<JsonArray>();
        // [수정] FIR_TAPS_DEF(63) -> FIR_TAPS_MAX(255) 로 변경하여 정밀 필터 계수 잘림 방지
        for (size_t idx = 0; idx < T2_Def::Audio::FeatureLimit::FIR_TAPS_MAX; idx++) {
            v_chEq.add(_dynConfig.audio.eq_coeffs[ch][idx]);
        }
    }

    v_aud["active_ceps_count"] = _dynConfig.audio.active_ceps_count;
    v_aud["active_peak_count"] = _dynConfig.audio.active_peak_count;

    JsonArray v_ceps = v_aud["ceps_targets"].to<JsonArray>();
    for (uint8_t i = 0; i < _dynConfig.audio.active_ceps_count; i++) v_ceps.add(_dynConfig.audio.ceps_targets[i]);

    v_aud["f_min"]             = _dynConfig.audio.f_min;
    v_aud["f_max"]             = _dynConfig.audio.f_max;
    v_aud["peak_amp_min"]      = _dynConfig.audio.peak_amp_min;
    v_aud["peak_freq_gap_min"] = _dynConfig.audio.peak_freq_gap_min;

    serializeDspConfig(v_aud["dsp"].to<JsonObject>(), _dynConfig.audio.dsp);    

    // [처리 단위 6] 원자적 쓰기(Atomic Write) 수행
    File v_tmp = LittleFS.open(T2_Def::Global::Path::FILE_CFG_TMP_CONST, "w");
    if (!v_tmp) {
        xSemaphoreGive(_lock);
        return false;
    }
    serializeJson(v_doc, v_tmp);
    v_tmp.flush();
    v_tmp.close();

    LittleFS.remove(T2_Def::Global::Path::FILE_CFG_JSON_CONST);
    LittleFS.rename(T2_Def::Global::Path::FILE_CFG_TMP_CONST, T2_Def::Global::Path::FILE_CFG_JSON_CONST);

    // [신규] LittleFS 성공 시 WAL 파티션도 싱크 커밋 및 다음 슬롯 준비
    _walDriver.commitConfigFast(_dynConfig);
    _walDriver.prepareNextSlot();

    _isDirty = false;
    xSemaphoreGive(_lock);
    return true;
}

// 함수설명: 설정 구성을 초기 기본값 상태로 되돌리고 플래시 파일에 즉시 저장합니다.
void CL_T2_ConfigManager::resetToDefault() {
    xSemaphoreTake(_lock, portMAX_DELAY);
    _loadDefaults();
    xSemaphoreGive(_lock);
    save();
}

// 함수설명: 현재 활성화된 동적 설정 구조체 데이터의 스냅샷 복사본을 반환합니다. (반환값: 동적 설정 구조체 복사본)
T2_Type::ST_DynamicConfig_t CL_T2_ConfigManager::getConfig() {
    xSemaphoreTake(_lock, portMAX_DELAY);
    T2_Type::ST_DynamicConfig_t v__dynConfig_snap = _dynConfig;
    xSemaphoreGive(_lock);
    return v__dynConfig_snap;
}

// 함수설명: 동적 설정 전체를 새로운 내용으로 교체한 후 플래시에 저장합니다. (p_dynConfig_new: 교체할 설정 구조체, 반환값: 저장 성공 여부)
bool CL_T2_ConfigManager::updateConfig(const T2_Type::ST_DynamicConfig_t& p_dynConfig_new) {
    xSemaphoreTake(_lock, portMAX_DELAY);
    _dynConfig = p_dynConfig_new;
    xSemaphoreGive(_lock);
    return save();
}

// 함수설명: 동적 설정을 새로운 내용으로 메모리 상에만 갱신하고 더티 플래그를 설정하여 지연 저장을 유도합니다. (p_dynConfig_new: 갱신할 설정 구조체, 반환값: 성공 여부)
bool CL_T2_ConfigManager::updateConfigLazy(const T2_Type::ST_DynamicConfig_t& p_dynConfig_new) {
    xSemaphoreTake(_lock, portMAX_DELAY);
    _dynConfig = p_dynConfig_new;
    _isDirty = true;
    _lastModifiedMs = (uint32_t)millis();

    // [신규] 즉각적인 WAL 커밋으로 초동 전원 무결성 보존 (지연 없음)
    _walDriver.commitConfigFast(_dynConfig);

    xSemaphoreGive(_lock);
    return true;
}

// 함수설명: JSON 문자열을 수신하여 설정을 갱신하고 더티 플래그를 설정하여 지연 쓰기가 실행되도록 합니다. (p_jsonString: 수신된 JSON 설정 문자열, 반환값: 성공 여부)
bool CL_T2_ConfigManager::updateFromJson(const char* p_jsonString) {
    xSemaphoreTake(_lock, portMAX_DELAY);
    JsonDocument v_doc;
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

// 함수설명: 더티 플래그가 설정된 지연 쓰기 대기 상태에서 일정 시간이 경과하면 플래시에 최종 저장합니다.
void CL_T2_ConfigManager::checkLazyWrite() {
    if (!_isDirty || _isTuningActive) return;
    uint32_t v_now = (uint32_t)millis();
    if (v_now - _lastModifiedMs > T2_Def::Global::Task::LAZY_WRITE_MS_DEF) {
        ESP_LOGI(TAG, "Executing Lazy Write to flash...");
        save();
    }
}

// 함수설명: 튜닝 모드 진입 시 임시 JSON 문자열을 통해 메모리 설정을 미리보기 적용합니다. (p_jsonString: 임시 미리보기 설정 JSON 문자열, 반환값: 성공 여부)
bool CL_T2_ConfigManager::updatePreview(const char* p_jsonString) {
    xSemaphoreTake(_lock, portMAX_DELAY);
    JsonDocument v_doc;
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

// 함수설명: 미리보기 적용된 튜닝 설정을 최종 확정하여 플래시에 저장합니다. (반환값: 저장 성공 여부)
bool CL_T2_ConfigManager::commitSave() {
    _isTuningActive = false;
    return save();
}

// 함수설명: 튜닝 미리보기를 취소하고 원래 플래시에 저장되어 있던 설정값을 다시 복원합니다. (반환값: 복원 성공 여부)
bool CL_T2_ConfigManager::revertCancel() {
    _isTuningActive = false;
    bool res = load();
    if (res) {
        // [이슈 10.req_250_001.md 항목 124번 해결] 취소 후 복원된 실제 정상 설정을 DSP 엔진 필터에 재로드
        extern void T240_ReloadDspFilters();
        T240_ReloadDspFilters();
    }
    return res;
}

// 함수설명: 현재 주요 동적 설정의 핵심 필드들만 수집하여 축소된 JSON 형태로 버퍼에 직렬화 출력합니다. (p_outBuf: 출력할 문자 버퍼, p_maxLen: 최대 크기 제한)
void CL_T2_ConfigManager::serializeToBuffer(char* p_outBuf, size_t p_maxLen) {
    xSemaphoreTake(_lock, portMAX_DELAY);
    JsonDocument v_doc;

    v_doc["ver"]   = T2_Def::Global::System::VERSION_STR;
    v_doc["acc_r"] = _dynConfig.accel.range;
    v_doc["acc_s"] = _dynConfig.accel.sample_rate;
    v_doc["gyr_r"] = _dynConfig.gyro.range;
    v_doc["gyr_s"] = _dynConfig.gyro.sample_rate;
    v_doc["aud_s"] = _dynConfig.audio.sample_rate;

    size_t v_written = serializeJson(v_doc, p_outBuf, p_maxLen);
    if (v_written < p_maxLen) p_outBuf[v_written] = '\0';
    xSemaphoreGive(_lock);
}

// ============================================================================
// [신규] CL_T2_WalDriver 구현부
// ============================================================================

CL_T2_WalDriver::CL_T2_WalDriver()
    : _partition(nullptr), _slotSize(0), _totalSlots(0), _nextSlotIdx(0), _latestSeqId(0) {
    _walLock = xSemaphoreCreateMutex();
}

CL_T2_WalDriver::~CL_T2_WalDriver() {
    if (_walLock) {
        vSemaphoreDelete(_walLock);
        _walLock = nullptr;
    }
}

bool CL_T2_WalDriver::init() {
    _partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, (esp_partition_subtype_t)0x99, "wal");
    if (!_partition) {
        ESP_LOGE("WAL", "Failed to find 'wal' raw partition! Check partitions_16MB.csv.");
        return false;
    }

    // 슬롯 크기는 (Header + Config)를 4KB 섹터 경계로 올림 정렬
    uint32_t rawSize = sizeof(ST_WalHeader_t) + sizeof(T2_Type::ST_DynamicConfig_t);
    _slotSize = ((rawSize + 4095) / 4096) * 4096;
    _totalSlots = _partition->size / _slotSize;
    _nextSlotIdx = 0;
    _latestSeqId = 0;

    ESP_LOGI("WAL", "Initialized. Partition Size: %u, Slot Size: %u, Total Slots: %u", 
             _partition->size, _slotSize, _totalSlots);
    return true;
}

bool CL_T2_WalDriver::loadLatestConfig(T2_Type::ST_DynamicConfig_t& p_cfg) {
    if (!_partition || _totalSlots == 0) return false;

    xSemaphoreTake(_walLock, portMAX_DELAY);
    ST_WalHeader_t bestHeader = {0};
    uint32_t bestSlotIdx = 0xFFFFFFFF;
    bool foundAny = false;

    ST_WalHeader_t tempHeader;
    for (uint32_t i = 0; i < _totalSlots; i++) {
        esp_err_t err = esp_partition_read(_partition, i * _slotSize, &tempHeader, sizeof(ST_WalHeader_t));
        if (err != ESP_OK) continue;

        if (tempHeader.magic == 0x57414C32) {
            if (!foundAny || tempHeader.sequence_id > bestHeader.sequence_id) {
                T2_Type::ST_DynamicConfig_t tempCfg;
                err = esp_partition_read(_partition, i * _slotSize + sizeof(ST_WalHeader_t), &tempCfg, sizeof(T2_Type::ST_DynamicConfig_t));
                if (err == ESP_OK) {
                    uint32_t crc = esp_rom_crc32_le(0, (const uint8_t*)&tempCfg, sizeof(T2_Type::ST_DynamicConfig_t));
                    if (crc == tempHeader.crc32) {
                        bestHeader = tempHeader;
                        bestSlotIdx = i;
                        foundAny = true;
                    } else {
                        ESP_LOGW("WAL", "CRC mismatch on slot %u (seq: %u)", i, tempHeader.sequence_id);
                    }
                }
            }
        }
    }

    if (foundAny && bestSlotIdx != 0xFFFFFFFF) {
        esp_err_t err = esp_partition_read(_partition, bestSlotIdx * _slotSize + sizeof(ST_WalHeader_t), &p_cfg, sizeof(T2_Type::ST_DynamicConfig_t));
        if (err == ESP_OK) {
            _latestSeqId = bestHeader.sequence_id;
            _nextSlotIdx = (bestSlotIdx + 1) % _totalSlots;
            ESP_LOGI("WAL", "Successfully loaded config from slot %u, sequence_id: %u", bestSlotIdx, _latestSeqId);
            xSemaphoreGive(_walLock);
            return true;
        }
    }

    ESP_LOGW("WAL", "No valid WAL record found. Falling back to LittleFS.");
    xSemaphoreGive(_walLock);
    return false;
}

bool CL_T2_WalDriver::commitConfigFast(const T2_Type::ST_DynamicConfig_t& p_cfg) {
    if (!_partition || _totalSlots == 0) return false;

    xSemaphoreTake(_walLock, portMAX_DELAY);
    uint32_t slotIdx = _nextSlotIdx;
    _latestSeqId++;

    ST_WalHeader_t header;
    header.magic = 0x57414C32;
    header.sequence_id = _latestSeqId;
    header.data_len = sizeof(T2_Type::ST_DynamicConfig_t);
    header.crc32 = esp_rom_crc32_le(0, (const uint8_t*)&p_cfg, sizeof(T2_Type::ST_DynamicConfig_t));

    esp_err_t err = esp_partition_write(_partition, slotIdx * _slotSize, &header, sizeof(ST_WalHeader_t));
    if (err != ESP_OK) {
        ESP_LOGE("WAL", "Failed to write header to slot %u", slotIdx);
        xSemaphoreGive(_walLock);
        return false;
    }

    err = esp_partition_write(_partition, slotIdx * _slotSize + sizeof(ST_WalHeader_t), &p_cfg, sizeof(T2_Type::ST_DynamicConfig_t));
    if (err != ESP_OK) {
        ESP_LOGE("WAL", "Failed to write config data to slot %u", slotIdx);
        xSemaphoreGive(_walLock);
        return false;
    }

    ESP_LOGI("WAL", "Committed config fast to slot %u, seq: %u", slotIdx, _latestSeqId);
    _nextSlotIdx = (slotIdx + 1) % _totalSlots;
    xSemaphoreGive(_walLock);
    return true;
}

bool CL_T2_WalDriver::prepareNextSlot() {
    if (!_partition || _totalSlots == 0) return false;

    xSemaphoreTake(_walLock, portMAX_DELAY);
    uint32_t slotIdx = _nextSlotIdx;
    esp_err_t err = esp_partition_erase_range(_partition, slotIdx * _slotSize, _slotSize);
    if (err != ESP_OK) {
        ESP_LOGE("WAL", "Failed to erase next slot %u", slotIdx);
        xSemaphoreGive(_walLock);
        return false;
    }

    ESP_LOGI("WAL", "Pre-erased slot %u for next write", slotIdx);
    xSemaphoreGive(_walLock);
    return true;
}

bool CL_T2_WalDriver::clearAll() {
    if (!_partition) return false;
    xSemaphoreTake(_walLock, portMAX_DELAY);
    esp_err_t err = esp_partition_erase_range(_partition, 0, _partition->size);
    if (err == ESP_OK) {
        _nextSlotIdx = 0;
        _latestSeqId = 0;
        ESP_LOGI("WAL", "Cleared all WAL slots.");
        xSemaphoreGive(_walLock);
        return true;
    }
    xSemaphoreGive(_walLock);
    return false;
}
