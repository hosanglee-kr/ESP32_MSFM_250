/* ============================================================================
 * File: T215_CfgMgr_242.hpp
 * Summary: 동적 JSON 설정 관리자 (SSOT & 핫스왑 튜닝 지원)
 * ============================================================================
 * [로직 상태 추적 (Logic State Tracker)]
 * - 유지: T2 프로젝트의 진동 스마트 트리거(RMS/Band) 파라미터 및 절전(Deep Sleep) 설정 관리.
 * - 갱신: T4(SMEA-100)의 Preview/Save/Cancel 핫스왑 UI/UX 튜닝 로직 이식 완료.
 * - 신규: 이종 센서(진동+소음) 융합을 위한 다중 필터(FIR, IIR, Notch) 및 노이즈 게이트 계층화.
 *
 * [축소/누락 방어 체크리스트 (Omission Defense)]
 * 1. [스레드 안전성]: 모든 public 메서드에 xSemaphoreTake/Give를 적용하여 Web 태스크와 메인 루프 간의 Race Condition 차단.
 * 2. [원자적 저장]: save() 수행 시 .tmp 파일 생성 후 rename을 수행하여 정전 시 파일 증발 방지.
 * 3. [플래시 마모 방어]: updatePreview()는 RAM만 갱신하며 플래시 쓰기를 유발하지 않음.
 * 4. [포인터 오염 방지]: ArduinoJson V7 호환성을 위해 읽기 작업 시 반드시 JsonObjectConst 사용.
 * ========================================================================== */
#pragma once

#include "T210_Def_242.hpp"
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

class CL_T2_ConfigManager {
private:
    T2_Type::DynamicConfig _config;
    SemaphoreHandle_t        _lock;
    bool                     _isLoaded;

    // Lazy Write (지연 쓰기) 제어 플래그
    bool                     _isDirty;
    uint32_t                 _lastModifiedMs;

    // Lock-free 동기화를 위한 volatile 깃발 (UI/UX 핫스왑용)
    volatile bool            _isTuningActive;

    CL_T2_ConfigManager();
    ~CL_T2_ConfigManager();

    void _loadDefaults();
    void _applyJson(const JsonDocument& p_doc);

public:
    static CL_T2_ConfigManager& getInstance() {
        static CL_T2_ConfigManager v_instance;
        return v_instance;
    }

    bool init();
    bool load();
    bool save();
    void resetToDefault();

    T2_Type::DynamicConfig getConfig();
    bool updateConfig(const T2_Type::DynamicConfig& p_newConfig);

    // JSON 문자열 수신 시 파일에 즉시 쓰지 않고 메모리 병합 후 _isDirty 플래그 활성화
    bool updateFromJson(const char* p_jsonString);

    // 메인 루프에서 주기적으로 호출되어 디바운스 완료 시 실제 저장 수행
    void checkLazyWrite();

    // Web UI/UX 튜닝 전용 메서드 3종 세트
    bool updatePreview(const char* p_jsonString); // RAM만 즉각 갱신 + 깃발 On
    bool commitSave();                            // 플래시에 영구 저장 + 깃발 Off
    bool revertCancel();                          // 기존 플래시 값으로 복원 + 깃발 On

    // DSP 파이프라인 등에서 튜닝 변경 사항을 감지하기 위한 Lock-free 함수
    bool isTuningActive() const { return _isTuningActive; }
    void clearTuningFlag() { _isTuningActive = false; }
};
