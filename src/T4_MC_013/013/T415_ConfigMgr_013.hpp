/* 
============================================================================
 * [SMEA-100 핵심 구현 원칙 및 AI 셀프 회고 바이블]
 * 1. [동적 설정 관리]: 런타임에 웹 API로 변경 가능한 설정값들을 JSON으로 관리.
 * 2. [스레드 안전성]: FSM 태스크와 Web 서버 콜백이 동시 접근하므로 반드시
 * Mutex 락(_lock)을 통해 구조체를 복사(Copy)하여 반환한다.
 * 3. [버전 호환]: ArduinoJson V7.4.x의 동적 메모리 할당(JsonDocument) 규격 준수.
 * 4. [네이밍 컨벤션 엄수]: private(_), 매개변수(p_), 로컬변수(v_)
 * 5. [명시적 구조체 설계]: 메모리 오염 방지 및 타입 안정성을 위해 명시적 구조체 정의.
 * 6. [매직 넘버 철폐]: 버퍼 길이, AP 접속 기본값 등 하드코딩 요소를 T410으로 이관.
 * 7. [v013 추가 - 핫스왑 UI/UX]: 슬라이더 조작 시 플래시를 쓰지 않고 메모리만 
 * 갱신하는 Preview, 명시적 Save, 그리고 Cancel(Rollback) 기능을 분리하여 
 * 플래시 마모(Wear-out)를 0%로 만들고 실시간 차트 피드백을 보장한다.
 * ============================================================================
 * File: T415_ConfigMgr_013.hpp
 * Summary: Dynamic JSON Configuration Manager (Singleton)
 * ========================================================================== 
*/
 
#pragma once

#include "T410_Def_013.hpp" 
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <cstring>

struct ST_Config_Dsp {
    float window_ms;
    float hop_ms;
    float notch_freq_hz;
    float notch_freq_2_hz;
    float notch_q_factor;
    float pre_emphasis_alpha;
    float beamforming_gain;
    float fir_lpf_cutoff;
    float fir_hpf_cutoff;
    int   median_window;
    float noise_gate_thresh;
    int   noise_learn_frames;
    float spectral_sub_gain;
    
    // 캘리브레이션 적용 계수
    float calib_gain_L;
    float calib_gain_R;
    float calib_eq_coeffs[SmeaConfig::CalibLimit::EQ_FIR_TAPS_CONST];
    
};

struct ST_Config_Feature {
    uint8_t band_rms_count;
    float   band_ranges[SmeaConfig::FeatureLimit::MAX_BAND_RMS_COUNT_CONST][2];
    float   ceps_targets[SmeaConfig::FeatureLimit::CEPS_TARGET_COUNT_CONST];
    float   spatial_freq_min_hz;
    float   spatial_freq_max_hz;
    float   peak_amplitude_limit_min;
    float   peak_freq_gap_limit_hz_min;
};

struct ST_Config_Decision {
    float rule_enrg_threshold;
    float rule_stddev_threshold;
    float test_ng_min_energy;
    int   min_trigger_count;
    float noise_profile_sec;
    float valid_start_sec;
    float valid_end_sec;
    float sta_lta_threshold;
};

struct ST_Config_Storage {
    uint8_t  pre_trigger_sec;
    uint32_t rotate_mb;
    uint32_t rotate_min;
    uint32_t idle_flush_ms;
};

struct ST_Config_Calib {
    uint32_t auto_idle_min;
    float    ref_freq_hz;
    float    filter_min_freq_hz;
    float    filter_max_freq_hz;
    float    target_gain_max;
    float    target_gain_min;
    float    norm_safe_thresh;
};

struct ST_Config_Mqtt {
    char mqtt_broker[SmeaConfig::NetworkLimit::MAX_BROKER_LEN_CONST];
    uint32_t retry_interval_ms;
    uint16_t default_port;
    // MQTT 버전 및 QoS 선택 변수 승격
    uint8_t  protocol_ver; 
    uint8_t  qos;          
};


struct ST_Config_WiFi_AP {
    char ssid[SmeaConfig::NetworkLimit::MAX_SSID_LEN_CONST];
    char password[SmeaConfig::NetworkLimit::MAX_PW_LEN_CONST];
    bool use_static_ip;
    char local_ip[SmeaConfig::NetworkLimit::MAX_IP_LEN_CONST];
    char gateway[SmeaConfig::NetworkLimit::MAX_IP_LEN_CONST];
    char subnet[SmeaConfig::NetworkLimit::MAX_IP_LEN_CONST];
    char dns1[SmeaConfig::NetworkLimit::MAX_IP_LEN_CONST];
    char dns2[SmeaConfig::NetworkLimit::MAX_IP_LEN_CONST];
};

struct ST_Config_WiFi {
    uint8_t mode;
    char ap_ssid[SmeaConfig::NetworkLimit::MAX_SSID_LEN_CONST];
    char ap_password[SmeaConfig::NetworkLimit::MAX_PW_LEN_CONST];
    char ap_ip[SmeaConfig::NetworkLimit::MAX_IP_LEN_CONST];
    ST_Config_WiFi_AP multi_ap[SmeaConfig::NetworkLimit::MAX_MULTI_AP_CONST];
};

struct DynamicConfig {
    ST_Config_Dsp      dsp;
    ST_Config_Feature  feature;
    ST_Config_Decision decision;
    ST_Config_Storage  storage;
    ST_Config_Calib    calib;
    ST_Config_Mqtt     mqtt;
    ST_Config_WiFi     wifi;
};


class T415_ConfigManager {
private:
    DynamicConfig     _config;
    SemaphoreHandle_t _lock;
    bool              _isLoaded;

    // Lazy Write (지연 쓰기) 제어 플래그
    bool              _isDirty;         // (기존 용도) 네트워크 등 단순 지연 쓰기용
    uint32_t          _lastModifiedMs;
    
    // [v013 추가] Lock-free 동기화를 위한 volatile 깃발 (UI/UX 핫스왑용)
    volatile bool     _isTuningActive;

public:
    static T415_ConfigManager& getInstance() {
        static T415_ConfigManager v_instance;
        return v_instance;
    }

    bool init();
    bool load();
    bool save();
    void resetToDefault();
    DynamicConfig getConfig();

    bool updateConfig(const DynamicConfig& p_newConfig);
    
    // JSON 문자열 수신 시 파일에 즉시 쓰지 않고 메모리 병합 후 _isDirty 플래그 활성화
    bool updateFromJson(const char* p_jsonString);

    // 메인 루프에서 주기적으로 호출되어 디바운스 완료 시 실제 저장 수행
    void checkLazyWrite();
    
    // [v013 추가] Web UI/UX 튜닝 전용 메서드 3종 세트
    bool updatePreview(const char* p_jsonString); // RAM만 즉각 갱신 + 깃발 On
    bool commitSave();                            // 플래시에 영구 저장 + 깃발 Off
    bool revertCancel();                          // 로드 원복 + 깃발 펄럭임(DSP 동기화)

    // DSP 엔진 등 핫패스에서 락(Lock) 없이 변경 여부를 즉시 검사하는 함수
    bool isTuningActive() const { return _isTuningActive; }
    void clearTuningFlag() { _isTuningActive = false; }


private:
    T415_ConfigManager();
    ~T415_ConfigManager();

    void _loadDefaults();
    void _applyJson(const JsonDocument& p_doc);
};

