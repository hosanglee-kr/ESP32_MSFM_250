/* ============================================================================
 * File: T220_CfgMgr_244.hpp
 * Summary: 4-Tier 도메인 통합 JSON 설정 관리자 (SSOT & 핫스왑 튜닝 지원)
 * ============================================================================
 * [시스템 로직 상태 (Logic State Tracker)]
 * - 기반 타입: T220_Type_243_7.hpp의 DynamicConfig 구조체 1:1 매핑.
 * - 주요 역할: LittleFS 연동, JSON 직렬화/역직렬화, RAM-Flash 간 동기화.
 *
 * [구현 원칙 (Core Principles)]
 * 1. [스레드 안전성]: Semaphore 기반 Mutex 적용으로 다중 Task 간 Race Condition 차단.
 * 2. [원자적 저장]: .tmp 파일 스왑 방식을 통한 정전 시 파일 파손(Corruption) 방지.
 * 3. [플래시 보호]: Lazy Write(지연 쓰기) 및 Preview(RAM 갱신) 로직으로 Flash I/O 최소화.
 * 4. [상태 무결성]: volatile 키워드 활용으로 Lock-free 상태 조회 지원.
 * ========================================================================== */

#pragma once

#include "T215_Type_244.hpp"
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

class CL_T2_ConfigManager {
private:
    T2_Type::DynamicConfig _config;                 // 4-Tier 통합 설정 단일 진실 공급원 (SSOT)
    SemaphoreHandle_t      _lock;                   // 다중 Task 접근 제어용 Mutex 핸들
    bool                   _isLoaded;               // 플래시 메모리에서 로드 완료 여부

    // Lazy Write (지연 쓰기) 제어 플래그
    bool                   _isDirty;                // RAM 값과 Flash 값이 불일치함을 나타내는 더티 플래그
    uint32_t               _lastModifiedMs;         // 마지막 설정 변경 발생 타임스탬프 (ms)

    // Lock-free 동기화를 위한 volatile 깃발 (Web UI 핫스왑 튜닝용)
    volatile bool          _isTuningActive;         // 현재 튜닝(Preview) 모드 진행 여부 지시자

    CL_T2_ConfigManager();                          // 싱글톤 패턴을 위한 private 생성자
    ~CL_T2_ConfigManager();                         // 소멸자 (Mutex 해제)

    void _loadDefaults();                           // 4-Tier 전체 기본값(DEF) 메모리 할당
    void _applyJson(const JsonDocument& p_doc);     // 파싱된 JSON 트리를 DynamicConfig 구조체에 병합

public:
    // 싱글톤 인스턴스 반환
    static CL_T2_ConfigManager& getInstance() {
        static CL_T2_ConfigManager v_instance;
        return v_instance;
    }

    // 시스템 생명주기 제어 메서드
    bool init();                                    // LittleFS 마운트 및 초기 설정 로드 (또는 복구)
    bool load();                                    // 플래시 파일에서 JSON 파싱 후 RAM 적재
    bool save();                                    // 현재 RAM 구조체를 JSON 변환 후 플래시에 원자적 기록
    void resetToDefault();                          // 공장 초기화 수행 후 플래시 즉시 저장

    // 데이터 접근 및 조작 메서드
    T2_Type::DynamicConfig getConfig();             // 현재 시스템 설정 구조체 스냅샷 복사본 반환 (Thread-Safe)
    bool updateConfig(const T2_Type::DynamicConfig& p_newConfig); // 전체 구조체 덮어쓰기 및 플래시 즉시 저장

    // 지연 쓰기(Lazy Write) 기반 부분 업데이트
    bool updateFromJson(const char* p_jsonString);  // 수신된 JSON 문자열 병합 후 지연 쓰기 예약 (_isDirty On)
    void checkLazyWrite();                          // 메인 루프에서 주기적 호출되어 타임아웃 시 플래시 실제 기록 수행

    // Web UI/UX 실시간 튜닝 전용 메서드 3종
    bool updatePreview(const char* p_jsonString);   // 플래시 기록 없이 RAM만 즉각 갱신 + 튜닝 깃발 On
    bool commitSave();                              // 임시 적용된 RAM 설정을 플래시에 영구 저장 + 튜닝 깃발 Off
    bool revertCancel();                            // 임시 설정을 폐기하고 기존 플래시 값으로 복원 + 튜닝 깃발 Off

    // DSP 파이프라인 외부 상태 관측 메서드
    bool isTuningActive() const { return _isTuningActive; } // Lock 획득 없이 튜닝 진행 상태 확인 (Lock-free)
    void clearTuningFlag() { _isTuningActive = false; }     // 파이프라인에서 튜닝 변경 사항 인지 후 깃발 초기화

    // [v243 신규] MLOps 추적용 설정 덤프 (바이너리 헤더 저장용)
    void serializeToBuffer(char* p_outBuf, size_t p_maxLen);
};
