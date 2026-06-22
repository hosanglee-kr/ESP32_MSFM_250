/* ============================================================================
 * File: T220_CfgMgr_250.cpp
 * Summary: 4-Tier 동적 JSON 설정 관리자 구현부
 * ============================================================================ */

 #include "T215_Type_250.hpp"
#include "T220_CfgMgr_250.hpp"
#include <LittleFS.h>
#include <cstring>
#include "esp_log.h"
#include "esp_rom_crc.h"

static const char* TAG = "T220_CFG";

// [이슈 11] g_psramAlloc 제거 — 10KB 미만 JSON을 PSRAM에 담으면 캐시 미스 오버헤드가 발생함. 기본 내부 힙 할당 사용.

// ----------------------------------------------------------------------------
// 1. DSP 설정 블록 구현
// ----------------------------------------------------------------------------
void T2_20_Cfg_J2V_DspCfg(JsonObjectConst p_obj, T2_Type::ST_Dsp_Config_t& p_dsp) {
    if (p_obj.isNull()) return;
    T2_20_Cfg_J2V_Val(p_obj["rem_dc"],  p_dsp.rem_dc);
    T2_20_Cfg_J2V_Val(p_obj["med_en"],  p_dsp.med_en);
    T2_20_Cfg_J2V_Val(p_obj["med_win"], p_dsp.med_win);

    JsonObjectConst hpf = p_obj["hpf"];
    if (!hpf.isNull()) {
        T2_20_Cfg_J2V_Val(hpf["en"],     p_dsp.hpf.en);
        T2_20_Cfg_J2V_Val(hpf["cutoff"], p_dsp.hpf.cutoff);
        T2_20_Cfg_J2V_Val(hpf["taps"],   p_dsp.hpf.taps);
    }
    JsonObjectConst lpf = p_obj["lpf"];
    if (!lpf.isNull()) {
        T2_20_Cfg_J2V_Val(lpf["en"],     p_dsp.lpf.en);
        T2_20_Cfg_J2V_Val(lpf["cutoff"], p_dsp.lpf.cutoff);
        T2_20_Cfg_J2V_Val(lpf["taps"],   p_dsp.lpf.taps);
    }
    JsonObjectConst ihpf = p_obj["iir_hpf"];
    if (!ihpf.isNull()) {
        T2_20_Cfg_J2V_Val(ihpf["en"],     p_dsp.iir_hpf.en);
        T2_20_Cfg_J2V_Val(ihpf["cutoff"], p_dsp.iir_hpf.cutoff);
        T2_20_Cfg_J2V_Val(ihpf["q"],      p_dsp.iir_hpf.q);
    }
    JsonObjectConst ilpf = p_obj["iir_lpf"];
    if (!ilpf.isNull()) {
        T2_20_Cfg_J2V_Val(ilpf["en"],     p_dsp.iir_lpf.en);
        T2_20_Cfg_J2V_Val(ilpf["cutoff"], p_dsp.iir_lpf.cutoff);
        T2_20_Cfg_J2V_Val(ilpf["q"],      p_dsp.iir_lpf.q);
    }
    JsonObjectConst notch = p_obj["notch"];
    if (!notch.isNull()) {
        T2_20_Cfg_J2V_Val(notch["en"],   p_dsp.notch.en);
        T2_20_Cfg_J2V_Val(notch["freq"], p_dsp.notch.freq);
        T2_20_Cfg_J2V_Val(notch["gain"], p_dsp.notch.gain);
        T2_20_Cfg_J2V_Val(notch["q"],    p_dsp.notch.q);
    }
    JsonObjectConst notch2 = p_obj["notch2"];
    if (!notch2.isNull()) {
        T2_20_Cfg_J2V_Val(notch2["en"],   p_dsp.notch2.en);
        T2_20_Cfg_J2V_Val(notch2["freq"], p_dsp.notch2.freq);
        T2_20_Cfg_J2V_Val(notch2["gain"], p_dsp.notch2.gain);
        T2_20_Cfg_J2V_Val(notch2["q"],    p_dsp.notch2.q);
    }
    if (!p_obj["win_type"].isNull()) {
        uint8_t wType = p_obj["win_type"].as<uint8_t>();
        if (wType < (uint8_t)T2_Type::EM_WindowType_t::COUNT) {
            p_dsp.win_type = (T2_Type::EM_WindowType_t)wType;
        }
    }
}

void T2_20_Cfg_V2J_DspCfg(JsonObject p_obj, const T2_Type::ST_Dsp_Config_t& p_dsp) {
    p_obj["rem_dc"]  = p_dsp.rem_dc;
    p_obj["med_en"]  = p_dsp.med_en;
    p_obj["med_win"] = p_dsp.med_win;

    JsonObject hpf = p_obj["hpf"].to<JsonObject>();
    hpf["en"]     = p_dsp.hpf.en;
    hpf["cutoff"] = p_dsp.hpf.cutoff;
    hpf["taps"]   = p_dsp.hpf.taps;

    JsonObject lpf = p_obj["lpf"].to<JsonObject>();
    lpf["en"]     = p_dsp.lpf.en;
    lpf["cutoff"] = p_dsp.lpf.cutoff;
    lpf["taps"]   = p_dsp.lpf.taps;

    JsonObject ihpf = p_obj["iir_hpf"].to<JsonObject>();
    ihpf["en"]     = p_dsp.iir_hpf.en;
    ihpf["cutoff"] = p_dsp.iir_hpf.cutoff;
    ihpf["q"]      = p_dsp.iir_hpf.q;

    JsonObject ilpf = p_obj["iir_lpf"].to<JsonObject>();
    ilpf["en"]     = p_dsp.iir_lpf.en;
    ilpf["cutoff"] = p_dsp.iir_lpf.cutoff;
    ilpf["q"]      = p_dsp.iir_lpf.q;

    JsonObject notch = p_obj["notch"].to<JsonObject>();
    notch["en"]   = p_dsp.notch.en;
    notch["freq"] = p_dsp.notch.freq;
    notch["gain"] = p_dsp.notch.gain;
    notch["q"]    = p_dsp.notch.q;

    JsonObject notch2 = p_obj["notch2"].to<JsonObject>();
    notch2["en"]   = p_dsp.notch2.en;
    notch2["freq"] = p_dsp.notch2.freq;
    notch2["gain"] = p_dsp.notch2.gain;
    notch2["q"]    = p_dsp.notch2.q;

    p_obj["win_type"] = (uint8_t)p_dsp.win_type;
}

// ----------------------------------------------------------------------------
// 2. 도메인별 그룹 매핑 함수 구현
// ----------------------------------------------------------------------------

// 2.1 System
void T2_20_Cfg_J2V_SystemCfg(JsonObjectConst p_jsonObj, T2_Type::ST_Global_System_t& p_SystemCfg) {
    if (p_jsonObj.isNull()) return;
    T2_20_Cfg_J2V_Str(p_jsonObj["site_id"], p_SystemCfg.site_id, sizeof(p_SystemCfg.site_id));
    T2_20_Cfg_J2V_Val(p_jsonObj["tele_hz"], p_SystemCfg.tele_hz);
    T2_20_Cfg_J2V_Val(p_jsonObj["wave_hz"], p_SystemCfg.wave_hz);
    if (!p_jsonObj["op_mode"].isNull()) {
        uint8_t modeVal = p_jsonObj["op_mode"].as<uint8_t>();
        if (modeVal < (uint8_t)T2_Type::EM_OpMode_t::COUNT) {
            p_SystemCfg.op_mode = (T2_Type::EM_OpMode_t)modeVal;
        }
    }
    T2_20_Cfg_J2V_Val(p_jsonObj["watchdog_ms"], p_SystemCfg.watchdog_ms);
}

void T2_20_Cfg_V2J_SystemCfg(JsonObject p_jsonObj, const T2_Type::ST_Global_System_t& p_SystemCfg) {
    T2_20_Cfg_V2J_Str(p_jsonObj, "site_id", p_SystemCfg.site_id);
    T2_20_Cfg_V2J_Val(p_jsonObj, "tele_hz", p_SystemCfg.tele_hz);
    T2_20_Cfg_V2J_Val(p_jsonObj, "wave_hz", p_SystemCfg.wave_hz);
    T2_20_Cfg_V2J_Val(p_jsonObj, "op_mode", (uint8_t)p_SystemCfg.op_mode);
    T2_20_Cfg_V2J_Val(p_jsonObj, "watchdog_ms", p_SystemCfg.watchdog_ms);
}

// 2.2 p_WifiCfg
void T2_20_Cfg_J2V_WifiCfg(JsonObjectConst p_jsonObj, T2_Type::ST_Global_WiFi_t& p_WifiCfg) {
    if (p_jsonObj.isNull()) return;
    if (!p_jsonObj["mode"].isNull()) {
        uint8_t modeVal = p_jsonObj["mode"].as<uint8_t>();
        if (modeVal < (uint8_t)T2_Type::EM_WiFiMode_t::COUNT) {
            p_WifiCfg.mode = (T2_Type::EM_WiFiMode_t)modeVal;
        }
    }
    T2_20_Cfg_J2V_Str(p_jsonObj["ap_ssid"], p_WifiCfg.ap_ssid, sizeof(p_WifiCfg.ap_ssid));
    T2_20_Cfg_J2V_Str(p_jsonObj["ap_pw"],   p_WifiCfg.ap_pw,   sizeof(p_WifiCfg.ap_pw));
    T2_20_Cfg_J2V_Str(p_jsonObj["ap_ip"],   p_WifiCfg.ap_ip,   sizeof(p_WifiCfg.ap_ip));

    JsonArrayConst multi = p_jsonObj["multi_ap"];
    if (!multi.isNull()) {
        uint8_t i = 0;
        for (JsonObjectConst ap : multi) {
            if (i >= T2_Def::Global::NetLimit::NET_MULTI_AP_MAX) break;
            T2_20_Cfg_J2V_Str(ap["ssid"], p_WifiCfg.multi_ssid[i], sizeof(p_WifiCfg.multi_ssid[i]));
            T2_20_Cfg_J2V_Str(ap["pw"],   p_WifiCfg.multi_pw[i],   sizeof(p_WifiCfg.multi_pw[i]));
            i++;
        }
    }
    T2_20_Cfg_J2V_Val(p_jsonObj["disconnect_delay_ms"], p_WifiCfg.disconnect_delay_ms);
}

void T2_20_Cfg_V2J_WifiCfg(JsonObject p_jsonObj, const T2_Type::ST_Global_WiFi_t& p_WifiCfg) {
    T2_20_Cfg_V2J_Val(p_jsonObj, "mode", (uint8_t)p_WifiCfg.mode);
    T2_20_Cfg_V2J_Str(p_jsonObj, "ap_ssid", p_WifiCfg.ap_ssid);
    T2_20_Cfg_V2J_Str(p_jsonObj, "ap_pw",   p_WifiCfg.ap_pw);
    T2_20_Cfg_V2J_Str(p_jsonObj, "ap_ip",   p_WifiCfg.ap_ip);

    JsonArray multi = p_jsonObj["multi_ap"].to<JsonArray>();
    for (uint8_t i = 0; i < T2_Def::Global::NetLimit::NET_MULTI_AP_MAX; i++) {
        JsonObject ap = multi.add<JsonObject>();
        T2_20_Cfg_V2J_Str(ap, "ssid", p_WifiCfg.multi_ssid[i]);
        T2_20_Cfg_V2J_Str(ap, "pw",   p_WifiCfg.multi_pw[i]);
    }
    T2_20_Cfg_V2J_Val(p_jsonObj, "disconnect_delay_ms", p_WifiCfg.disconnect_delay_ms);
}

// 2.3 Mqtt
void T2_20_Cfg_J2V_MqttCfg(JsonObjectConst p_jsonObj, T2_Type::ST_Global_Mqtt_t& p_MqttCfg) {
    if (p_jsonObj.isNull()) return;
    T2_20_Cfg_J2V_Val(p_jsonObj["enable"], p_MqttCfg.enable);
    T2_20_Cfg_J2V_Str(p_jsonObj["broker"], p_MqttCfg.broker, sizeof(p_MqttCfg.broker));
    T2_20_Cfg_J2V_Val(p_jsonObj["port"],   p_MqttCfg.port);
    T2_20_Cfg_J2V_Str(p_jsonObj["id"],     p_MqttCfg.id, sizeof(p_MqttCfg.id));
    T2_20_Cfg_J2V_Str(p_jsonObj["pw"],     p_MqttCfg.pw, sizeof(p_MqttCfg.pw));
    T2_20_Cfg_J2V_Str(p_jsonObj["topic_root"], p_MqttCfg.topic_root, sizeof(p_MqttCfg.topic_root));
    T2_20_Cfg_J2V_Str(p_jsonObj["lwt_topic"],  p_MqttCfg.lwt_topic, sizeof(p_MqttCfg.lwt_topic));
    T2_20_Cfg_J2V_Val(p_jsonObj["qos"],       p_MqttCfg.qos);
    T2_20_Cfg_J2V_Val(p_jsonObj["proto_ver"], p_MqttCfg.proto_ver);
}

void T2_20_Cfg_V2J_MqttCfg(JsonObject p_jsonObj, const T2_Type::ST_Global_Mqtt_t& p_MqttCfg) {
    T2_20_Cfg_V2J_Val(p_jsonObj, "enable", p_MqttCfg.enable);
    T2_20_Cfg_V2J_Str(p_jsonObj, "broker", p_MqttCfg.broker);
    T2_20_Cfg_V2J_Val(p_jsonObj, "port",   p_MqttCfg.port);
    T2_20_Cfg_V2J_Str(p_jsonObj, "id",     p_MqttCfg.id);
    T2_20_Cfg_V2J_Str(p_jsonObj, "pw",     p_MqttCfg.pw);
    T2_20_Cfg_V2J_Str(p_jsonObj, "topic_root", p_MqttCfg.topic_root);
    T2_20_Cfg_V2J_Str(p_jsonObj, "lwt_topic",  p_MqttCfg.lwt_topic);
    T2_20_Cfg_V2J_Val(p_jsonObj, "qos",       p_MqttCfg.qos);
    T2_20_Cfg_V2J_Val(p_jsonObj, "proto_ver", p_MqttCfg.proto_ver);
}

// 2.4 Ntp
void T2_20_Cfg_J2V_NtpCfg(JsonObjectConst p_jsonObj, T2_Type::ST_Global_NTP_t& p_NtpCfg) {
    if (p_jsonObj.isNull()) return;
    T2_20_Cfg_J2V_Str(p_jsonObj["ntp_server1"], p_NtpCfg.ntp_server1, sizeof(p_NtpCfg.ntp_server1));
    T2_20_Cfg_J2V_Str(p_jsonObj["ntp_server2"], p_NtpCfg.ntp_server2, sizeof(p_NtpCfg.ntp_server2));
    T2_20_Cfg_J2V_Str(p_jsonObj["ntp_tz"],      p_NtpCfg.ntp_tz,      sizeof(p_NtpCfg.ntp_tz));
}

void T2_20_Cfg_V2J_NtpCfg(JsonObject p_jsonObj, const T2_Type::ST_Global_NTP_t& p_NtpCfg) {
    T2_20_Cfg_V2J_Str(p_jsonObj, "ntp_server1", p_NtpCfg.ntp_server1);
    T2_20_Cfg_V2J_Str(p_jsonObj, "ntp_server2", p_NtpCfg.ntp_server2);
    T2_20_Cfg_V2J_Str(p_jsonObj, "ntp_tz",      p_NtpCfg.ntp_tz);
}

// 2.5 Storage
void T2_20_Cfg_J2V_StorageCfg(JsonObjectConst p_jsonObj, T2_Type::ST_Global_Storage_t& p_StorageCfg) {
    if (p_jsonObj.isNull()) return;
    T2_20_Cfg_J2V_Val(p_jsonObj["rot_mb"],         p_StorageCfg.rot_mb);
    T2_20_Cfg_J2V_Val(p_jsonObj["rot_min"],        p_StorageCfg.rot_min);
    T2_20_Cfg_J2V_Val(p_jsonObj["save_raw"],       p_StorageCfg.save_raw);
    T2_20_Cfg_J2V_Val(p_jsonObj["keep_max"],       p_StorageCfg.keep_max);
    T2_20_Cfg_J2V_Val(p_jsonObj["idle_flush_ms"],  p_StorageCfg.idle_flush_ms);
    uint8_t pre = p_StorageCfg.pre_trig_sec;
    T2_20_Cfg_J2V_Val(p_jsonObj["pre_trig_sec"], pre);
    if (pre > 10) pre = 10;   // 안전 상한 가드
    p_StorageCfg.pre_trig_sec = pre;
}

void T2_20_Cfg_V2J_StorageCfg(JsonObject p_jsonObj, const T2_Type::ST_Global_Storage_t& p_StorageCfg) {
    T2_20_Cfg_V2J_Val(p_jsonObj, "rot_mb",         p_StorageCfg.rot_mb);
    T2_20_Cfg_V2J_Val(p_jsonObj, "rot_min",        p_StorageCfg.rot_min);
    T2_20_Cfg_V2J_Val(p_jsonObj, "save_raw",       p_StorageCfg.save_raw);
    T2_20_Cfg_V2J_Val(p_jsonObj, "keep_max",       p_StorageCfg.keep_max);
    T2_20_Cfg_V2J_Val(p_jsonObj, "idle_flush_ms",  p_StorageCfg.idle_flush_ms);
    T2_20_Cfg_V2J_Val(p_jsonObj, "pre_trig_sec",   p_StorageCfg.pre_trig_sec);
}

// 2.6 Output
void T2_20_Cfg_J2V_OutputCfg(JsonObjectConst p_jsonObj, T2_Type::ST_Global_Output_t& p_OutputCfg) {
    if (p_jsonObj.isNull()) return;
    T2_20_Cfg_J2V_Val(p_jsonObj["enabled"],         p_OutputCfg.enabled);
    T2_20_Cfg_J2V_Val(p_jsonObj["output_sequence"], p_OutputCfg.output_sequence);
    uint16_t frames = p_OutputCfg.sequence_frames;
    T2_20_Cfg_J2V_Val(p_jsonObj["sequence_frames"], frames);
    if (frames > T2_Def::Global::System::SEQUENCE_FRAMES_MAX)
        frames = T2_Def::Global::System::SEQUENCE_FRAMES_MAX;
    p_OutputCfg.sequence_frames = frames;
}

void T2_20_Cfg_V2J_OutputCfg(JsonObject p_jsonObj, const T2_Type::ST_Global_Output_t& p_OutputCfg) {
    T2_20_Cfg_V2J_Val(p_jsonObj, "enabled",         p_OutputCfg.enabled);
    T2_20_Cfg_V2J_Val(p_jsonObj, "output_sequence", p_OutputCfg.output_sequence);
    T2_20_Cfg_V2J_Val(p_jsonObj, "sequence_frames", p_OutputCfg.sequence_frames);
}

// 2.7 Decision
void T2_20_Cfg_J2V_DecisionCfg(JsonObjectConst p_jsonObj, T2_Type::ST_Global_Decision_t& p_DecisionCfg) {
    if (p_jsonObj.isNull()) return;
    T2_20_Cfg_J2V_Val(p_jsonObj["max_trial_count"],   p_DecisionCfg.max_trial_count);
    T2_20_Cfg_J2V_Val(p_jsonObj["sta_lta_threshold"], p_DecisionCfg.sta_lta_threshold);
    T2_20_Cfg_J2V_Val(p_jsonObj["min_trigger_count"], p_DecisionCfg.min_trigger_count);
    T2_20_Cfg_J2V_Val(p_jsonObj["valid_start_sec"],   p_DecisionCfg.valid_start_sec);
    T2_20_Cfg_J2V_Val(p_jsonObj["valid_end_sec"],     p_DecisionCfg.valid_end_sec);
    if (p_DecisionCfg.max_trial_count > T2_Def::Global::Decision::MAX_TRIAL_COUNT_MAX)
        p_DecisionCfg.max_trial_count = T2_Def::Global::Decision::MAX_TRIAL_COUNT_MAX;
}

void T2_20_Cfg_V2J_DecisionCfg(JsonObject p_jsonObj, const T2_Type::ST_Global_Decision_t& p_DecisionCfg) {
    T2_20_Cfg_V2J_Val(p_jsonObj, "max_trial_count",   p_DecisionCfg.max_trial_count);
    T2_20_Cfg_V2J_Val(p_jsonObj, "sta_lta_threshold", p_DecisionCfg.sta_lta_threshold);
    T2_20_Cfg_V2J_Val(p_jsonObj, "min_trigger_count", p_DecisionCfg.min_trigger_count);
    T2_20_Cfg_V2J_Val(p_jsonObj, "valid_start_sec",   p_DecisionCfg.valid_start_sec);
    T2_20_Cfg_V2J_Val(p_jsonObj, "valid_end_sec",     p_DecisionCfg.valid_end_sec);
}

// 2.8 Trigger
void T2_20_Cfg_J2V_TriggerCfg(JsonObjectConst p_jsonObj, T2_Type::ST_DynamicConfig_t& p_DynamicConfig) {
    if (p_jsonObj.isNull()) return;
    T2_20_Cfg_J2V_Val(p_jsonObj["hold_ms"],   p_DynamicConfig.trig_hold_ms);
    T2_20_Cfg_J2V_Val(p_jsonObj["use_sleep"], p_DynamicConfig.trig_use_sleep);
    T2_20_Cfg_J2V_Val(p_jsonObj["sleep_sec"], p_DynamicConfig.trig_sleep_sec);
}

void T2_20_Cfg_V2J_TriggerCfg(JsonObject p_jsonObj, const T2_Type::ST_DynamicConfig_t& p_DynamicConfig) {
    T2_20_Cfg_V2J_Val(p_jsonObj, "hold_ms",   p_DynamicConfig.trig_hold_ms);
    T2_20_Cfg_V2J_Val(p_jsonObj, "use_sleep", p_DynamicConfig.trig_use_sleep);
    T2_20_Cfg_V2J_Val(p_jsonObj, "sleep_sec", p_DynamicConfig.trig_sleep_sec);
}

// 2.9 Accel
void T2_20_Cfg_J2V_AccelCfg(JsonObjectConst p_jsonObj, T2_Type::ST_Accel_Config_t& p_AccCfg) {
    if (p_jsonObj.isNull()) return;

    T2_20_Cfg_J2V_Val(p_jsonObj["enable"],         p_AccCfg.enable);
    T2_20_Cfg_J2V_Val(p_jsonObj["axis_mask"],      p_AccCfg.axis_mask);
    T2_20_Cfg_J2V_Val(p_jsonObj["range"],          p_AccCfg.range);
    T2_20_Cfg_J2V_Val(p_jsonObj["odr"],            p_AccCfg.odr);
    T2_20_Cfg_J2V_Val(p_jsonObj["bwp"],            p_AccCfg.bwp);
    T2_20_Cfg_J2V_Val(p_jsonObj["filter_perf"],    p_AccCfg.filter_perf);
    T2_20_Cfg_J2V_Val(p_jsonObj["fifo_watermark"], p_AccCfg.fifo_watermark);
    T2_20_Cfg_J2V_Val(p_jsonObj["sample_rate"],    p_AccCfg.sample_rate);
    T2_20_Cfg_J2V_Val(p_jsonObj["fft_size"],       p_AccCfg.fft_size);
    // fft_size 유효성 검증
    {
        uint32_t v = p_AccCfg.fft_size;
        auto isPow2 = [](uint32_t x){ return x > 0 && (x & (x-1)) == 0; };
        if (!isPow2(v) || v > T2_Def::Accel::Sensor::FFT_SIZE_MAX)
            p_AccCfg.fft_size = T2_Def::Accel::Sensor::FFT_SIZE_DEF;
    }
    T2_20_Cfg_J2V_Val(p_jsonObj["noise_perf"],     p_AccCfg.noise_perf);

    T2_20_Cfg_J2V_Val(p_jsonObj["motion_en"], p_AccCfg.motion_en);
    T2_20_Cfg_J2V_Val(p_jsonObj["wake_g"],    p_AccCfg.wake_g);
    T2_20_Cfg_J2V_Val(p_jsonObj["wake_dur"],  p_AccCfg.wake_dur);

    T2_20_Cfg_J2V_Array(p_jsonObj["rms_thresh"],      p_AccCfg.rms_thresh);
    T2_20_Cfg_J2V_Array(p_jsonObj["kurt_ng_thresh"],  p_AccCfg.kurt_ng_thresh);
    T2_20_Cfg_J2V_Array(p_jsonObj["crest_ng_thresh"], p_AccCfg.crest_ng_thresh);
    T2_20_Cfg_J2V_Array(p_jsonObj["skew_ng_thresh"],  p_AccCfg.skew_ng_thresh);

    // bands
    JsonArrayConst v_bands = p_jsonObj["bands"];
    if (!v_bands.isNull())
        p_AccCfg.active_band_count = T2_20_Cfg_J2V_Bands<
            decltype(p_AccCfg.band_en), decltype(p_AccCfg.band_start),
            decltype(p_AccCfg.band_end), decltype(p_AccCfg.band_thresh),
            T2_Def::Accel::Sensor::AXIS_MAX, T2_Def::Accel::FeatureLimit::BAND_MAX>(
            v_bands, p_AccCfg.band_en, p_AccCfg.band_start, p_AccCfg.band_end, p_AccCfg.band_thresh);

    T2_20_Cfg_J2V_Array(p_jsonObj["offset"], p_AccCfg.offset);
    T2_20_Cfg_J2V_Array(p_jsonObj["gain"],   p_AccCfg.gain);
    T2_20_Cfg_J2V_Val(p_jsonObj["peak_amp_min"],      p_AccCfg.peak_amp_min);
    T2_20_Cfg_J2V_Val(p_jsonObj["peak_freq_gap_min"], p_AccCfg.peak_freq_gap_min);

    T2_20_Cfg_J2V_DspCfg(p_jsonObj["dsp"], p_AccCfg.dsp);
}

void T2_20_Cfg_V2J_AccelCfg(JsonObject p_jsonObj, const T2_Type::ST_Accel_Config_t& p_AccCfg) {
    T2_20_Cfg_V2J_Val(p_jsonObj, "enable",         p_AccCfg.enable);
    T2_20_Cfg_V2J_Val(p_jsonObj, "axis_mask",      p_AccCfg.axis_mask);
    T2_20_Cfg_V2J_Val(p_jsonObj, "range",          p_AccCfg.range);
    T2_20_Cfg_V2J_Val(p_jsonObj, "odr",            p_AccCfg.odr);
    T2_20_Cfg_V2J_Val(p_jsonObj, "bwp",            p_AccCfg.bwp);
    T2_20_Cfg_V2J_Val(p_jsonObj, "filter_perf",    p_AccCfg.filter_perf);
    T2_20_Cfg_V2J_Val(p_jsonObj, "fifo_watermark", p_AccCfg.fifo_watermark);
    T2_20_Cfg_V2J_Val(p_jsonObj, "sample_rate",    p_AccCfg.sample_rate);
    T2_20_Cfg_V2J_Val(p_jsonObj, "fft_size",       p_AccCfg.fft_size);
    T2_20_Cfg_V2J_Val(p_jsonObj, "noise_perf",     p_AccCfg.noise_perf);

    T2_20_Cfg_V2J_Val(p_jsonObj, "motion_en", p_AccCfg.motion_en);
    T2_20_Cfg_V2J_Val(p_jsonObj, "wake_g",    p_AccCfg.wake_g);
    T2_20_Cfg_V2J_Val(p_jsonObj, "wake_dur",  p_AccCfg.wake_dur);

    T2_20_Cfg_V2J_Array(p_jsonObj["rms_thresh"].to<JsonArray>(),      p_AccCfg.rms_thresh);
    T2_20_Cfg_V2J_Array(p_jsonObj["kurt_ng_thresh"].to<JsonArray>(),  p_AccCfg.kurt_ng_thresh);
    T2_20_Cfg_V2J_Array(p_jsonObj["crest_ng_thresh"].to<JsonArray>(), p_AccCfg.crest_ng_thresh);
    T2_20_Cfg_V2J_Array(p_jsonObj["skew_ng_thresh"].to<JsonArray>(),  p_AccCfg.skew_ng_thresh);

    T2_20_Cfg_V2J_Bands<decltype(p_AccCfg.band_en), decltype(p_AccCfg.band_start), decltype(p_AccCfg.band_end),
                        decltype(p_AccCfg.band_thresh), T2_Def::Accel::Sensor::AXIS_MAX>(
                        p_jsonObj["bands"].to<JsonArray>(),
                        p_AccCfg.active_band_count,
                        p_AccCfg.band_en, p_AccCfg.band_start, p_AccCfg.band_end, p_AccCfg.band_thresh);

    T2_20_Cfg_V2J_Array(p_jsonObj["offset"].to<JsonArray>(), p_AccCfg.offset);
    T2_20_Cfg_V2J_Array(p_jsonObj["gain"].to<JsonArray>(),   p_AccCfg.gain);
    T2_20_Cfg_V2J_Val(p_jsonObj, "peak_amp_min",      p_AccCfg.peak_amp_min);
    T2_20_Cfg_V2J_Val(p_jsonObj, "peak_freq_gap_min", p_AccCfg.peak_freq_gap_min);

    T2_20_Cfg_V2J_DspCfg(p_jsonObj["dsp"].to<JsonObject>(), p_AccCfg.dsp);
}

// 2.10 Gyro
void T2_20_Cfg_J2V_GyroCfg(JsonObjectConst p_jsonObj, T2_Type::ST_Gyro_Config_t& p_GyrCfg) {
    if (p_jsonObj.isNull()) return;
    T2_20_Cfg_J2V_Val(p_jsonObj["enable"],      p_GyrCfg.enable);
    T2_20_Cfg_J2V_Val(p_jsonObj["axis_mask"],   p_GyrCfg.axis_mask);
    T2_20_Cfg_J2V_Val(p_jsonObj["range"],       p_GyrCfg.range);
    T2_20_Cfg_J2V_Val(p_jsonObj["odr"],         p_GyrCfg.odr);
    T2_20_Cfg_J2V_Val(p_jsonObj["bwp"],         p_GyrCfg.bwp);
    T2_20_Cfg_J2V_Val(p_jsonObj["filter_perf"], p_GyrCfg.filter_perf);
    T2_20_Cfg_J2V_Val(p_jsonObj["noise_perf"],  p_GyrCfg.noise_perf);
    T2_20_Cfg_J2V_Val(p_jsonObj["sample_rate"], p_GyrCfg.sample_rate);
    T2_20_Cfg_J2V_Val(p_jsonObj["fft_size"],    p_GyrCfg.fft_size);

    T2_20_Cfg_J2V_Array(p_jsonObj["rms_thresh"],      p_GyrCfg.rms_thresh);
    T2_20_Cfg_J2V_Array(p_jsonObj["kurt_ng_thresh"],  p_GyrCfg.kurt_ng_thresh);
    T2_20_Cfg_J2V_Array(p_jsonObj["crest_ng_thresh"], p_GyrCfg.crest_ng_thresh);
    T2_20_Cfg_J2V_Array(p_jsonObj["skew_ng_thresh"],  p_GyrCfg.skew_ng_thresh);

    JsonArrayConst v_bands = p_jsonObj["bands"];
    if (!v_bands.isNull())
        p_GyrCfg.active_band_count = T2_20_Cfg_J2V_Bands<
            decltype(p_GyrCfg.band_en), decltype(p_GyrCfg.band_start),
            decltype(p_GyrCfg.band_end), decltype(p_GyrCfg.band_thresh),
            T2_Def::Gyro::Sensor::AXIS_MAX, T2_Def::Gyro::FeatureLimit::BAND_MAX>(
            v_bands, p_GyrCfg.band_en, p_GyrCfg.band_start, p_GyrCfg.band_end, p_GyrCfg.band_thresh);

    T2_20_Cfg_J2V_Array(p_jsonObj["offset"], p_GyrCfg.offset);
    T2_20_Cfg_J2V_Array(p_jsonObj["gain"],   p_GyrCfg.gain);
    T2_20_Cfg_J2V_Val(p_jsonObj["peak_amp_min"],      p_GyrCfg.peak_amp_min);
    T2_20_Cfg_J2V_Val(p_jsonObj["peak_freq_gap_min"], p_GyrCfg.peak_freq_gap_min);

    T2_20_Cfg_J2V_DspCfg(p_jsonObj["dsp"], p_GyrCfg.dsp);
}

void T2_20_Cfg_V2J_GyroCfg(JsonObject p_jsonObj, const T2_Type::ST_Gyro_Config_t& p_GyrCfg) {
    T2_20_Cfg_V2J_Val(p_jsonObj, "enable",      p_GyrCfg.enable);
    T2_20_Cfg_V2J_Val(p_jsonObj, "axis_mask",   p_GyrCfg.axis_mask);
    T2_20_Cfg_V2J_Val(p_jsonObj, "range",       p_GyrCfg.range);
    T2_20_Cfg_V2J_Val(p_jsonObj, "odr",         p_GyrCfg.odr);
    T2_20_Cfg_V2J_Val(p_jsonObj, "bwp",         p_GyrCfg.bwp);
    T2_20_Cfg_V2J_Val(p_jsonObj, "filter_perf", p_GyrCfg.filter_perf);
    T2_20_Cfg_V2J_Val(p_jsonObj, "noise_perf",  p_GyrCfg.noise_perf);
    T2_20_Cfg_V2J_Val(p_jsonObj, "sample_rate", p_GyrCfg.sample_rate);
    T2_20_Cfg_V2J_Val(p_jsonObj, "fft_size",    p_GyrCfg.fft_size);

    T2_20_Cfg_V2J_Array(p_jsonObj["rms_thresh"].to<JsonArray>(),      p_GyrCfg.rms_thresh);
    T2_20_Cfg_V2J_Array(p_jsonObj["kurt_ng_thresh"].to<JsonArray>(),  p_GyrCfg.kurt_ng_thresh);
    T2_20_Cfg_V2J_Array(p_jsonObj["crest_ng_thresh"].to<JsonArray>(), p_GyrCfg.crest_ng_thresh);
    T2_20_Cfg_V2J_Array(p_jsonObj["skew_ng_thresh"].to<JsonArray>(),  p_GyrCfg.skew_ng_thresh);

    T2_20_Cfg_V2J_Bands<decltype(p_GyrCfg.band_en), decltype(p_GyrCfg.band_start), decltype(p_GyrCfg.band_end),
                        decltype(p_GyrCfg.band_thresh), T2_Def::Gyro::Sensor::AXIS_MAX>(
                        p_jsonObj["bands"].to<JsonArray>(),
                        p_GyrCfg.active_band_count,
                        p_GyrCfg.band_en, p_GyrCfg.band_start, p_GyrCfg.band_end, p_GyrCfg.band_thresh);

    T2_20_Cfg_V2J_Array(p_jsonObj["offset"].to<JsonArray>(), p_GyrCfg.offset);
    T2_20_Cfg_V2J_Array(p_jsonObj["gain"].to<JsonArray>(),   p_GyrCfg.gain);
    T2_20_Cfg_V2J_Val(p_jsonObj, "peak_amp_min",      p_GyrCfg.peak_amp_min);
    T2_20_Cfg_V2J_Val(p_jsonObj, "peak_freq_gap_min", p_GyrCfg.peak_freq_gap_min);

    T2_20_Cfg_V2J_DspCfg(p_jsonObj["dsp"].to<JsonObject>(), p_GyrCfg.dsp);
}

// 2.11 Audio
void T2_20_Cfg_J2V_AudioCfg(JsonObjectConst p_jsonObj, T2_Type::ST_Audio_Config_t& p_AudioCfg) {
    if (p_jsonObj.isNull()) return;
    T2_20_Cfg_J2V_Val(p_jsonObj["enable"],       p_AudioCfg.enable);
    T2_20_Cfg_J2V_Val(p_jsonObj["channel_mask"], p_AudioCfg.channel_mask);
    T2_20_Cfg_J2V_Val(p_jsonObj["sample_rate"],  p_AudioCfg.sample_rate);
    T2_20_Cfg_J2V_Val(p_jsonObj["fft_size"],     p_AudioCfg.fft_size);

    uint8_t melB = p_AudioCfg.melband_size;
    T2_20_Cfg_J2V_Val(p_jsonObj["melband_size"], melB);
    if (melB == 0 || melB > T2_Def::Audio::FeatureLimit::MEL_BANDS_MAX) {
        melB = T2_Def::Audio::FeatureLimit::MEL_BANDS_DEF;
    }
    p_AudioCfg.melband_size = melB;

    T2_20_Cfg_J2V_Array(p_jsonObj["rms_thresh"],      p_AudioCfg.rms_thresh);
    T2_20_Cfg_J2V_Array(p_jsonObj["kurt_ng_thresh"],  p_AudioCfg.kurt_ng_thresh);
    T2_20_Cfg_J2V_Array(p_jsonObj["crest_ng_thresh"], p_AudioCfg.crest_ng_thresh);
    T2_20_Cfg_J2V_Array(p_jsonObj["skew_ng_thresh"],  p_AudioCfg.skew_ng_thresh);

    JsonArrayConst v_bands = p_jsonObj["bands"];
    if (!v_bands.isNull())
        p_AudioCfg.active_band_count = T2_20_Cfg_J2V_Bands<
            decltype(p_AudioCfg.band_en), decltype(p_AudioCfg.band_start),
            decltype(p_AudioCfg.band_end), decltype(p_AudioCfg.band_thresh),
            T2_Def::Audio::Sensor::CHANNELS_MAX, T2_Def::Audio::FeatureLimit::BAND_MAX>(
            v_bands, p_AudioCfg.band_en, p_AudioCfg.band_start, p_AudioCfg.band_end, p_AudioCfg.band_thresh);

    JsonObjectConst noise = p_jsonObj["noise"];
    if (!noise.isNull()) {
        T2_20_Cfg_J2V_Val(noise["gate_en"],     p_AudioCfg.noise.gate_en);
        T2_20_Cfg_J2V_Val(noise["gate_thresh"], p_AudioCfg.noise.gate_thresh);
        if (!noise["mode"].isNull()) {
            uint8_t nMode = noise["mode"].as<uint8_t>();
            if (nMode < (uint8_t)T2_Type::EM_NoiseMode_t::COUNT) {
                p_AudioCfg.noise.mode = (T2_Type::EM_NoiseMode_t)nMode;
            }
        }
        T2_20_Cfg_J2V_Val(noise["sub_str"],     p_AudioCfg.noise.sub_str);
        T2_20_Cfg_J2V_Val(noise["adp_alpha"],   p_AudioCfg.noise.adp_alpha);
        T2_20_Cfg_J2V_Val(noise["learn_frames"],p_AudioCfg.noise.learn_frames);
    }

    T2_20_Cfg_J2V_Val(p_jsonObj["pre_en"],    p_AudioCfg.pre_en);
    T2_20_Cfg_J2V_Val(p_jsonObj["pre_alpha"], p_AudioCfg.pre_alpha);

    float bGain = p_AudioCfg.beam_gain;
    T2_20_Cfg_J2V_Val(p_jsonObj["beam_gain"], bGain);
    if (bGain < 0.0f) bGain = 0.0f;
    if (bGain > 1.0f) bGain = 1.0f;
    p_AudioCfg.beam_gain = bGain;

    T2_20_Cfg_J2V_Val(p_jsonObj["window_ms"], p_AudioCfg.window_ms);
    T2_20_Cfg_J2V_Val(p_jsonObj["hop_ms"],    p_AudioCfg.hop_ms);

    T2_20_Cfg_J2V_Val(p_jsonObj["auto_idle_min"], p_AudioCfg.auto_idle_min);
    T2_20_Cfg_J2V_Val(p_jsonObj["ref_freq"],      p_AudioCfg.ref_freq);
    T2_20_Cfg_J2V_Val(p_jsonObj["filt_min"],      p_AudioCfg.filt_min);
    T2_20_Cfg_J2V_Val(p_jsonObj["filt_max"],      p_AudioCfg.filt_max);
    T2_20_Cfg_J2V_Val(p_jsonObj["gain_max"],      p_AudioCfg.gain_max);
    T2_20_Cfg_J2V_Val(p_jsonObj["gain_min"],      p_AudioCfg.gain_min);
    T2_20_Cfg_J2V_Val(p_jsonObj["norm_safe"],     p_AudioCfg.norm_safe);
    T2_20_Cfg_J2V_Array(p_jsonObj["gain_ch"],     p_AudioCfg.gain_ch);

    JsonArrayConst eq = p_jsonObj["eq_coeffs"];
    if (!eq.isNull()) {
        if (eq[0].is<JsonArrayConst>()) {
            for (size_t ch = 0; ch < T2_Def::Audio::Sensor::CHANNELS_MAX; ch++) {
                JsonArrayConst chEq = eq[ch].as<JsonArrayConst>();
                size_t idx = 0;
                for (JsonVariantConst val : chEq) {
                    if (idx >= T2_Def::Audio::FeatureLimit::FIR_TAPS_MAX) break;
                    p_AudioCfg.eq_coeffs[ch][idx] = val.as<float>();
                    idx++;
                }
            }
        } else {
            size_t idx = 0;
            for (JsonVariantConst val : eq) {
                if (idx >= T2_Def::Audio::FeatureLimit::FIR_TAPS_MAX) break;
                float fVal = val.as<float>();
                for (size_t ch = 0; ch < T2_Def::Audio::Sensor::CHANNELS_MAX; ch++) {
                    p_AudioCfg.eq_coeffs[ch][idx] = fVal;
                }
                idx++;
            }
        }
    }

    T2_20_Cfg_J2V_Val(p_jsonObj["active_ceps_count"], p_AudioCfg.active_ceps_count);
    T2_20_Cfg_J2V_Val(p_jsonObj["active_peak_count"], p_AudioCfg.active_peak_count);

    JsonArrayConst ceps = p_jsonObj["ceps_targets"];
    if (!ceps.isNull()) {
        uint8_t idx = 0;
        for (JsonVariantConst val : ceps) {
            if (idx >= T2_Def::Audio::FeatureLimit::CEPS_TARGET_MAX) break;
            p_AudioCfg.ceps_targets[idx] = val.as<float>();
            idx++;
        }
        p_AudioCfg.active_ceps_count = idx;
    }

    T2_20_Cfg_J2V_Val(p_jsonObj["f_min"],             p_AudioCfg.f_min);
    T2_20_Cfg_J2V_Val(p_jsonObj["f_max"],             p_AudioCfg.f_max);
    T2_20_Cfg_J2V_Val(p_jsonObj["peak_amp_min"],      p_AudioCfg.peak_amp_min);
    T2_20_Cfg_J2V_Val(p_jsonObj["peak_freq_gap_min"], p_AudioCfg.peak_freq_gap_min);

    T2_20_Cfg_J2V_DspCfg(p_jsonObj["dsp"], p_AudioCfg.dsp);
}

void T2_20_Cfg_V2J_AudioCfg(JsonObject p_jsonObj, const T2_Type::ST_Audio_Config_t& p_AudioCfg) {
    T2_20_Cfg_V2J_Val(p_jsonObj, "enable",       p_AudioCfg.enable);
    T2_20_Cfg_V2J_Val(p_jsonObj, "channel_mask", p_AudioCfg.channel_mask);
    T2_20_Cfg_V2J_Val(p_jsonObj, "sample_rate",  p_AudioCfg.sample_rate);
    T2_20_Cfg_V2J_Val(p_jsonObj, "fft_size",     p_AudioCfg.fft_size);
    T2_20_Cfg_V2J_Val(p_jsonObj, "melband_size",    p_AudioCfg.melband_size);

    T2_20_Cfg_V2J_Array(p_jsonObj["rms_thresh"].to<JsonArray>(),      p_AudioCfg.rms_thresh);
    T2_20_Cfg_V2J_Array(p_jsonObj["kurt_ng_thresh"].to<JsonArray>(),  p_AudioCfg.kurt_ng_thresh);
    T2_20_Cfg_V2J_Array(p_jsonObj["crest_ng_thresh"].to<JsonArray>(), p_AudioCfg.crest_ng_thresh);
    T2_20_Cfg_V2J_Array(p_jsonObj["skew_ng_thresh"].to<JsonArray>(),  p_AudioCfg.skew_ng_thresh);

    T2_20_Cfg_V2J_Bands<decltype(p_AudioCfg.band_en), decltype(p_AudioCfg.band_start), decltype(p_AudioCfg.band_end),
                        decltype(p_AudioCfg.band_thresh), T2_Def::Audio::Sensor::CHANNELS_MAX>(
                        p_jsonObj["bands"].to<JsonArray>(),
                        p_AudioCfg.active_band_count,
                        p_AudioCfg.band_en, p_AudioCfg.band_start, p_AudioCfg.band_end, p_AudioCfg.band_thresh);

    JsonObject noise = p_jsonObj["noise"].to<JsonObject>();
    noise["gate_en"]     = p_AudioCfg.noise.gate_en;
    noise["gate_thresh"] = p_AudioCfg.noise.gate_thresh;
    noise["mode"]        = (uint8_t)p_AudioCfg.noise.mode;
    noise["sub_str"]     = p_AudioCfg.noise.sub_str;
    noise["adp_alpha"]   = p_AudioCfg.noise.adp_alpha;
    noise["learn_frames"]= p_AudioCfg.noise.learn_frames;

    T2_20_Cfg_V2J_Val(p_jsonObj, "pre_en",    p_AudioCfg.pre_en);
    T2_20_Cfg_V2J_Val(p_jsonObj, "pre_alpha", p_AudioCfg.pre_alpha);
    T2_20_Cfg_V2J_Val(p_jsonObj, "beam_gain", p_AudioCfg.beam_gain);
    T2_20_Cfg_V2J_Val(p_jsonObj, "window_ms", p_AudioCfg.window_ms);
    T2_20_Cfg_V2J_Val(p_jsonObj, "hop_ms",    p_AudioCfg.hop_ms);

    T2_20_Cfg_V2J_Val(p_jsonObj, "auto_idle_min", p_AudioCfg.auto_idle_min);
    T2_20_Cfg_V2J_Val(p_jsonObj, "ref_freq",      p_AudioCfg.ref_freq);
    T2_20_Cfg_V2J_Val(p_jsonObj, "filt_min",      p_AudioCfg.filt_min);
    T2_20_Cfg_V2J_Val(p_jsonObj, "filt_max",      p_AudioCfg.filt_max);
    T2_20_Cfg_V2J_Val(p_jsonObj, "gain_max",      p_AudioCfg.gain_max);
    T2_20_Cfg_V2J_Val(p_jsonObj, "gain_min",      p_AudioCfg.gain_min);
    T2_20_Cfg_V2J_Val(p_jsonObj, "norm_safe",     p_AudioCfg.norm_safe);

    T2_20_Cfg_V2J_Array(p_jsonObj["gain_ch"].to<JsonArray>(), p_AudioCfg.gain_ch);

    JsonArray eq = p_jsonObj["eq_coeffs"].to<JsonArray>();
    for (size_t ch = 0; ch < T2_Def::Audio::Sensor::CHANNELS_MAX; ch++) {
        JsonArray chEq = eq.add<JsonArray>();
        for (size_t idx = 0; idx < T2_Def::Audio::FeatureLimit::FIR_TAPS_MAX; idx++) {
            chEq.add(p_AudioCfg.eq_coeffs[ch][idx]);
        }
    }

    T2_20_Cfg_V2J_Val(p_jsonObj, "active_ceps_count", p_AudioCfg.active_ceps_count);
    T2_20_Cfg_V2J_Val(p_jsonObj, "active_peak_count", p_AudioCfg.active_peak_count);

    JsonArray ceps = p_jsonObj["ceps_targets"].to<JsonArray>();
    for (uint8_t i = 0; i < p_AudioCfg.active_ceps_count; i++) ceps.add(p_AudioCfg.ceps_targets[i]);

    T2_20_Cfg_V2J_Val(p_jsonObj, "f_min",             p_AudioCfg.f_min);
    T2_20_Cfg_V2J_Val(p_jsonObj, "f_max",             p_AudioCfg.f_max);
    T2_20_Cfg_V2J_Val(p_jsonObj, "peak_amp_min",      p_AudioCfg.peak_amp_min);
    T2_20_Cfg_V2J_Val(p_jsonObj, "peak_freq_gap_min", p_AudioCfg.peak_freq_gap_min);

    T2_20_Cfg_V2J_DspCfg(p_jsonObj["dsp"].to<JsonObject>(), p_AudioCfg.dsp);
}


// 설정 관리자 생성자이며, Mutex 및 멤버 변수를 기본 초기화합니다.
CL_T2_ConfigManager::CL_T2_ConfigManager() {
    _lock           = xSemaphoreCreateMutex();
    _isLoaded       = false;
    _isDirty        = false;
    _isTuningActive = false;
    _lastModifiedMs = 0;
    _loadDefaults();
}

// 설정 관리자 소멸자이며, 사용된 Mutex 동기화 객체를 해제합니다.
CL_T2_ConfigManager::~CL_T2_ConfigManager() {
    if (_lock) vSemaphoreDelete(_lock);
}

// LittleFS 마운트 상태 검증 및 부팅 시 원자적 복구 작업을 개시합니다. (반환값: 성공 여부)
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

// 4-Tier 시스템 설정 구조체의 기본값들을 초기 적재합니다.
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

    // NTP 타임서버 기본값 초기화
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

    // Decision 파라미터 기본값
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
    _dynConfig.gyro.dsp.notch2.en      = false;
    _dynConfig.gyro.dsp.notch2.freq    = T2_Def::Gyro::Dsp::NOTCH2_FREQ_DEF;
    _dynConfig.gyro.dsp.notch2.gain    = 1.0f;
    _dynConfig.gyro.dsp.notch2.q       = T2_Def::Gyro::Dsp::NOTCH_Q_DEF;
    _dynConfig.gyro.dsp.win_type       = T2_Type::EM_WindowType_t::HANN;

    // === Tier 4. Audio ===
    _dynConfig.audio.enable            = T2_Def::Audio::Sensor::ENABLE_DEF;
    _dynConfig.audio.channel_mask      = T2_Def::Audio::Sensor::CHANNEL_MASK_DEF;
    _dynConfig.audio.sample_rate       = T2_Def::Audio::Sensor::RATE_DEF;
    _dynConfig.audio.fft_size          = T2_Def::Audio::Sensor::FFT_SIZE_DEF;
    _dynConfig.audio.melband_size      = T2_Def::Audio::FeatureLimit::MEL_BANDS_DEF;

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

// 수신한 JSON 문서를 4-Tier 시스템 설정 구조체에 적용합니다. (p_doc: 파싱된 JSON 문서 참조)
void CL_T2_ConfigManager::_applyJson(const JsonDocument& p_doc) {
    T2_20_Cfg_J2V_SystemCfg(p_doc["system"], _dynConfig.system);
    T2_20_Cfg_J2V_WifiCfg(p_doc["wifi"], _dynConfig.wifi);
    T2_20_Cfg_J2V_MqttCfg(p_doc["mqtt"], _dynConfig.mqtt);
    T2_20_Cfg_J2V_NtpCfg(p_doc["ntp"], _dynConfig.ntp);
    T2_20_Cfg_J2V_StorageCfg(p_doc["storage"], _dynConfig.storage);
    T2_20_Cfg_J2V_OutputCfg(p_doc["output"], _dynConfig.output);
    T2_20_Cfg_J2V_DecisionCfg(p_doc["decision"], _dynConfig.decision);
    T2_20_Cfg_J2V_TriggerCfg(p_doc["trigger"], _dynConfig);
    T2_20_Cfg_J2V_AccelCfg(p_doc["accel"], _dynConfig.accel);
    T2_20_Cfg_J2V_GyroCfg(p_doc["gyro"], _dynConfig.gyro);
    T2_20_Cfg_J2V_AudioCfg(p_doc["audio"], _dynConfig.audio);
}

// LittleFS 파일시스템에서 JSON 설정 파일을 읽어와 동적 설정 구조체에 반영합니다. (반환값: 성공 여부)
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

// 현재 동적 설정 구조체의 멤버 값들을 JSON 문서로 직렬화하여 LittleFS 설정 파일에 저장합니다. (반환값: 성공 여부)
// ============================================================================
// [수정/추가] 누락된 ntp, decision, wifi 딜레이 설정 반영 및 eq_coeffs 배열 잘림 현상 수정
// ============================================================================
bool CL_T2_ConfigManager::save() {
    xSemaphoreTake(_lock, portMAX_DELAY);
    JsonDocument v_doc;

    T2_20_Cfg_V2J_SystemCfg	 (v_doc["system"].to<JsonObject>()		, _dynConfig.system	    );
    T2_20_Cfg_V2J_WifiCfg	 (v_doc["wifi"].to<JsonObject>()		, _dynConfig.wifi	    );
    T2_20_Cfg_V2J_MqttCfg	 (v_doc["mqtt"].to<JsonObject>()		, _dynConfig.mqtt	    );
    T2_20_Cfg_V2J_NtpCfg	 (v_doc["ntp"].to<JsonObject>()	        , _dynConfig.ntp		);
    T2_20_Cfg_V2J_StorageCfg (v_doc["storage"].to<JsonObject>()	    , _dynConfig.storage	);
    T2_20_Cfg_V2J_OutputCfg	 (v_doc["output"].to<JsonObject>()	    , _dynConfig.output		);
    T2_20_Cfg_V2J_DecisionCfg(v_doc["decision"].to<JsonObject>()    , _dynConfig.decision	);
    T2_20_Cfg_V2J_TriggerCfg (v_doc["trigger"].to<JsonObject>()	    , _dynConfig			);
    T2_20_Cfg_V2J_AccelCfg	 (v_doc["accel"].to<JsonObject>()	    , _dynConfig.accel		);
    T2_20_Cfg_V2J_GyroCfg	 (v_doc["gyro"].to<JsonObject>()	    , _dynConfig.gyro	    );
    T2_20_Cfg_V2J_AudioCfg	 (v_doc["audio"].to<JsonObject>()	    , _dynConfig.audio	    );

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

// 설정 구성을 초기 기본값 상태로 되돌리고 플래시 파일에 즉시 저장합니다.
void CL_T2_ConfigManager::resetToDefault() {
    xSemaphoreTake(_lock, portMAX_DELAY);
    _loadDefaults();
    xSemaphoreGive(_lock);
    save();
}

// 현재 활성화된 동적 설정 구조체 데이터의 스냅샷 복사본을 반환합니다. (반환값: 동적 설정 구조체 복사본)
T2_Type::ST_DynamicConfig_t CL_T2_ConfigManager::getConfig() {
    xSemaphoreTake(_lock, portMAX_DELAY);
    T2_Type::ST_DynamicConfig_t v__dynConfig_snap = _dynConfig;
    xSemaphoreGive(_lock);
    return v__dynConfig_snap;
}

// 동적 설정 전체를 새로운 내용으로 교체한 후 플래시에 저장합니다. (p_dynConfig_new: 교체할 설정 구조체, 반환값: 저장 성공 여부)
bool CL_T2_ConfigManager::updateConfig(const T2_Type::ST_DynamicConfig_t& p_dynConfig_new) {
    xSemaphoreTake(_lock, portMAX_DELAY);
    _dynConfig = p_dynConfig_new;
    xSemaphoreGive(_lock);
    return save();
}

// 동적 설정을 새로운 내용으로 메모리 상에만 갱신하고 더티 플래그를 설정하여 지연 저장을 유도합니다. (p_dynConfig_new: 갱신할 설정 구조체, 반환값: 성공 여부)
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

// JSON 문자열을 수신하여 설정을 갱신하고 더티 플래그를 설정하여 지연 쓰기가 실행되도록 합니다. (p_jsonString: 수신된 JSON 설정 문자열, 반환값: 성공 여부)
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

// 더티 플래그가 설정된 지연 쓰기 대기 상태에서 일정 시간이 경과하면 플래시에 최종 저장합니다.
void CL_T2_ConfigManager::checkLazyWrite() {
    if (!_isDirty || _isTuningActive) return;
    uint32_t v_now = (uint32_t)millis();
    if (v_now - _lastModifiedMs > T2_Def::Global::Task::LAZY_WRITE_MS_DEF) {
        ESP_LOGI(TAG, "Executing Lazy Write to flash...");
        save();
    }
}

// 튜닝 모드 진입 시 임시 JSON 문자열을 통해 메모리 설정을 미리보기 적용합니다. (p_jsonString: 임시 미리보기 설정 JSON 문자열, 반환값: 성공 여부)
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

// 미리보기 적용된 튜닝 설정을 최종 확정하여 플래시에 저장합니다. (반환값: 저장 성공 여부)
bool CL_T2_ConfigManager::commitSave() {
    _isTuningActive = false;
    return save();
}

// 튜닝 미리보기를 취소하고 원래 플래시에 저장되어 있던 설정값을 다시 복원합니다. (반환값: 복원 성공 여부)
bool CL_T2_ConfigManager::revertCancel() {
    _isTuningActive = false;
    bool res = load();
    if (res) {
        // [이슈 10.req_250_001.md 항목 124번 해결] 취소 후 복원된 실제 정상 설정을 DSP 엔진 필터에 재로드
        extern void T2_90_Fsm_ReloadDspFilters();
        T2_90_Fsm_ReloadDspFilters();
    }
    return res;
}

// 현재 주요 동적 설정의 핵심 필드들만 수집하여 축소된 JSON 형태로 버퍼에 직렬화 출력합니다. (p_outBuf: 출력할 문자 버퍼, p_maxLen: 최대 크기 제한)
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
// CL_T2_WalDriver 구현부
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
    uint32_t v_rawSize = sizeof(ST_WalHeader_t) + sizeof(T2_Type::ST_DynamicConfig_t);
    _slotSize = ((v_rawSize + 4095) / 4096) * 4096;
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
    ST_WalHeader_t 	v_bestHeader 	= {0};
    uint32_t 		v_bestSlotIdx 	= 0xFFFFFFFF;
    bool 			v_foundAny 		= false;

    ST_WalHeader_t v_tempWalHeader;
    for (uint32_t i = 0; i < _totalSlots; i++) {
        esp_err_t err = esp_partition_read(_partition, i * _slotSize, &v_tempWalHeader, sizeof(ST_WalHeader_t));
        if (err != ESP_OK) continue;

        if (v_tempWalHeader.magic == 0x57414C32) {
            if (!v_foundAny || v_tempWalHeader.sequence_id > v_bestHeader.sequence_id) {
                T2_Type::ST_DynamicConfig_t tempCfg;
                err = esp_partition_read(_partition, i * _slotSize + sizeof(ST_WalHeader_t), &tempCfg, sizeof(T2_Type::ST_DynamicConfig_t));
                if (err == ESP_OK) {
                    uint32_t crc = esp_rom_crc32_le(0, (const uint8_t*)&tempCfg, sizeof(T2_Type::ST_DynamicConfig_t));
                    if (crc == v_tempWalHeader.crc32) {
                        v_bestHeader = v_tempWalHeader;
                        v_bestSlotIdx = i;
                        v_foundAny = true;
                    } else {
                        ESP_LOGW("WAL", "CRC mismatch on slot %u (seq: %u)", i, v_tempWalHeader.sequence_id);
                    }
                }
            }
        }
    }

    if (v_foundAny && v_bestSlotIdx != 0xFFFFFFFF) {
        esp_err_t err = esp_partition_read(_partition, v_bestSlotIdx * _slotSize + sizeof(ST_WalHeader_t), &p_cfg, sizeof(T2_Type::ST_DynamicConfig_t));
        if (err == ESP_OK) {
            _latestSeqId = v_bestHeader.sequence_id;
            _nextSlotIdx = (v_bestSlotIdx + 1) % _totalSlots;
            ESP_LOGI("WAL", "Successfully loaded config from slot %u, sequence_id: %u", v_bestSlotIdx, _latestSeqId);
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
    uint32_t v_slotIdx = _nextSlotIdx;
    _latestSeqId++;

    ST_WalHeader_t header;
    header.magic = 0x57414C32;
    header.sequence_id = _latestSeqId;
    header.data_len = sizeof(T2_Type::ST_DynamicConfig_t);
    header.crc32 = esp_rom_crc32_le(0, (const uint8_t*)&p_cfg, sizeof(T2_Type::ST_DynamicConfig_t));

    esp_err_t err = esp_partition_write(_partition, v_slotIdx * _slotSize, &header, sizeof(ST_WalHeader_t));
    if (err != ESP_OK) {
        ESP_LOGE("WAL", "Failed to write header to slot %u", v_slotIdx);
        xSemaphoreGive(_walLock);
        return false;
    }

    err = esp_partition_write(_partition, v_slotIdx * _slotSize + sizeof(ST_WalHeader_t), &p_cfg, sizeof(T2_Type::ST_DynamicConfig_t));
    if (err != ESP_OK) {
        ESP_LOGE("WAL", "Failed to write config data to slot %u", v_slotIdx);
        xSemaphoreGive(_walLock);
        return false;
    }

    ESP_LOGI("WAL", "Committed config fast to slot %u, seq: %u", v_slotIdx, _latestSeqId);
    _nextSlotIdx = (v_slotIdx + 1) % _totalSlots;
    xSemaphoreGive(_walLock);
    return true;
}

bool CL_T2_WalDriver::prepareNextSlot() {
    if (!_partition || _totalSlots == 0) return false;

    xSemaphoreTake(_walLock, portMAX_DELAY);
    uint32_t v_slotIdx = _nextSlotIdx;
    esp_err_t err = esp_partition_erase_range(_partition, v_slotIdx * _slotSize, _slotSize);
    if (err != ESP_OK) {
        ESP_LOGE("WAL", "Failed to erase next slot %u", v_slotIdx);
        xSemaphoreGive(_walLock);
        return false;
    }

    ESP_LOGI("WAL", "Pre-erased slot %u for next write", v_slotIdx);
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
