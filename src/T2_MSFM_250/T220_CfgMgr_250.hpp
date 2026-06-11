/* ============================================================================
 * File: T220_CfgMgr_250.hpp
 * Summary: 4-Tier 도메인 통합 JSON 설정 관리자
 * ============================================================================
 * [시스템 로직 상태 (Logic State Tracker)]
 * - 기반 타입: ST_DynamicConfig_t 구조체 1:1 매핑.
 * - 주요 역할: LittleFS 연동, JSON 직렬화/역직렬화, RAM-Flash 간 동기화.
 * ========================================================================== */

#pragma once

#include "T215_Type_250.hpp"
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "esp_partition.h"

// --- [신규] 플래시 소거 지연 방지용 Raw WAL 드라이버 ---
struct ST_WalHeader_t {
    uint32_t magic;         // 0x57414C32 ('WAL2')
    uint32_t sequence_id;   // 시퀀스 번호
    uint32_t data_len;      // ST_DynamicConfig_t 크기
    uint32_t crc32;         // 데이터 무결성 검증용
};

class CL_T2_WalDriver {
private:
    const esp_partition_t* _partition;
    uint32_t               _slotSize;       // 4KB 정렬된 슬롯 크기
    uint32_t               _totalSlots;     // 파티션 내 전체 슬롯 수
    uint32_t               _nextSlotIdx;    // 다음에 쓸 슬롯 인덱스
    uint32_t               _latestSeqId;    // 최신 시퀀스 ID
    SemaphoreHandle_t      _walLock;        // [신규] 파티션 락 추가

public:
    CL_T2_WalDriver();
    ~CL_T2_WalDriver();

    bool init();
    bool loadLatestConfig(T2_Type::ST_DynamicConfig_t& p_cfg);
    bool commitConfigFast(const T2_Type::ST_DynamicConfig_t& p_cfg);
    bool prepareNextSlot();
    bool clearAll();
};

class CL_T2_ConfigManager {
private:
    T2_Type::ST_DynamicConfig_t _dynConfig;              // 4-Tier 통합 설정 단일 진실 공급원 (SSOT)
    SemaphoreHandle_t           _lock;                   // 다중 Task 접근 제어용 Mutex 핸들
    bool                        _isLoaded;               // 플래시 메모리에서 로드 완료 여부
    CL_T2_WalDriver             _walDriver;              // WAL 드라이버

    // Lazy Write (지연 쓰기) 제어 플래그
    bool                        _isDirty;                // RAM 값과 Flash 값이 불일치함을 나타내는 더티 플래그
    uint32_t                    _lastModifiedMs;         // 마지막 설정 변경 발생 타임스탬프 (ms)

    // Lock-free 동기화를 위한 volatile 깃발 (Web UI 핫스왑 튜닝용)
    volatile bool               _isTuningActive;         // 현재 튜닝(Preview) 모드 진행 여부 지시자

    CL_T2_ConfigManager();                          // 싱글톤 패턴을 위한 private 생성자
    ~CL_T2_ConfigManager();                         // 소멸자 (Mutex 해제)

    void _loadDefaults();                           // 4-Tier 전체 기본값(DEF) 메모리 할당
    void _applyJson(const JsonDocument& p_doc);     // 파싱된 JSON 트리를 ST_DynamicConfig_t 구조체에 병합

public:
    // 함수설명: 설정 매니저 싱글톤 인스턴스를 반환합니다.
    static CL_T2_ConfigManager& getInstance() {
        static CL_T2_ConfigManager v_instance;
        return v_instance;
    }

    // 함수설명: LittleFS 파일 시스템을 초기화 및 마운트합니다. (반환값: 성공 여부)
    bool init();

    // 함수설명: 플래시에서 JSON을 로드하여 RAM 메모리에 적재합니다. (반환값: 성공 여부)
    bool load();

    // 함수설명: 현재 RAM 설정을 JSON 변환 후 플래시에 원자적으로 영구 저장합니다. (반환값: 성공 여부)
    bool save();

    // 함수설명: 설정을 공장 초기화 기본값으로 복원하고 플래시에 즉시 저장합니다.
    void resetToDefault();

    // 함수설명: 현재 설정 구조체 스냅샷 복사본을 안전하게 반환합니다. (반환값: 설정 구조체 데이터)
    T2_Type::ST_DynamicConfig_t getConfig();

    // 함수설명: 전체 설정을 즉각 반영하고 플래시에 동기 쓰기 저장합니다. (p_newConfig: 신규 설정 구조체, 반환값: 성공 여부)
    bool updateConfig(const T2_Type::ST_DynamicConfig_t& p_newConfig);

    // 함수설명: 전체 설정을 즉각 반영하되 플래시에는 지연 쓰기를 예약합니다. (p_newConfig: 신규 설정 구조체, 반환값: 성공 여부)
    bool updateConfigLazy(const T2_Type::ST_DynamicConfig_t& p_newConfig);

    // 함수설명: 수신된 JSON 문자열을 설정에 부분 반영하고 지연 쓰기를 예약합니다. (p_jsonString: JSON 문자열 포인터, 반환값: 성공 여부)
    bool updateFromJson(const char* p_jsonString);

    // 함수설명: 설정 변경 시 지연 시간 만료를 체크하여 플래시에 백그라운드 쓰기를 수행합니다.
    void checkLazyWrite();

    // 함수설명: 플래시 기록 없이 임시로 RAM 설정만 갱신하여 핫스왑 튜닝 프리뷰를 제공합니다. (p_jsonString: JSON 문자열 포인터, 반환값: 성공 여부)
    bool updatePreview(const char* p_jsonString);

    // 함수설명: 임시 적용 중인 튜닝 설정을 플래시에 영구 저장 확정합니다. (반환값: 성공 여부)
    bool commitSave();

    // 함수설명: 임시 적용 중인 튜닝 설정을 취소하고 마지막 플래시 값으로 복원합니다. (반환값: 성공 여부)
    bool revertCancel();

    // 함수설명: 튜닝 프리뷰 상태가 활성인지 확인합니다. (반환값: 튜닝 진행 여부)
    bool isTuningActive() const { return _isTuningActive; }

    // 함수설명: 튜닝 프리뷰 활성 플래그를 비활성화로 해제합니다.
    void clearTuningFlag() { _isTuningActive = false; }

    // 함수설명: 시스템 주요 파라미터를 간략 포맷의 버퍼로 직렬화 출력합니다. (p_outBuf: 출력 버퍼, p_maxLen: 최대 바이트 크기)
    void serializeToBuffer(char* p_outBuf, size_t p_maxLen);
};


