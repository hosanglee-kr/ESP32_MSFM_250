/* ============================================================================
 * File: T260_Storage_242.hpp
 * Summary: 멀티모달 비동기 스토리지 엔진 (A-DSE) 및 프리트리거 매니저 풀버전
 * ============================================================================
 * [로직 상태 추적 (Logic State Tracker)]
 * - 유지: T2의 SD/LittleFS 이중 스토리지 지원 및 날짜(YYYY/MM/DD) 계층 폴더.
 * - 갱신: openSession() 내 블로킹 I/O를 비동기 링버퍼 고속 복사로 위임(Rule #28).
 * - 신규: 이종 센서(Vib+Audio) Raw 데이터 통합 저장을 위한 UnifiedRawChunk 규격.
 *
 * [축소/누락 방어 체크리스트 (Omission Defense)]
 * 1. [WDT 방어]: 백그라운드 태스크(_storageTaskProc) 내에 vTaskDelay 양보 적용.
 * 2. [파일 파괴 방어]: SD 카드 웨어레벨링 Freeze 방어용 f_expand(10MB 선할당) 유지.
 * 3. [시계열 정합성]: 로테이션 검사 시 millis() 오버플로우 대비 esp_timer_get_time() 적용.
 * 4. [원자성 방어]: 인덱스 파일(.json) 저장 시 .tmp 생성 후 rename() 원자적 처리 유지.
 * ========================================================================== */
#pragma once

#include "T210_Def_242.hpp"
#include <FS.h>
#include <LittleFS.h>
#include <SD_MMC.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>


class CL_T2_StorageManager {
private:
    static constexpr uint16_t ASYNC_RING_CAPACITY = 256;

    bool _sessionOpen;
    bool _ioError;
    uint32_t _recordCount;
    uint32_t _rotationSubSeq;

    File _activeFile;
    File _rawFile;

    char _activePath[T2_Def::StorageLimit::MAX_PATH_LEN_CONST];
    char _activeRawPath[T2_Def::StorageLimit::MAX_PATH_LEN_CONST];
    char _currentPrefix[32];

    uint32_t _sessionStartTick;
    uint32_t _writtenBytes;
    uint32_t _rawWrittenBytes;

    struct IndexItem {
        char path[T2_Def::StorageLimit::MAX_PATH_LEN_CONST];
        char raw_path[T2_Def::StorageLimit::MAX_PATH_LEN_CONST];
        uint32_t size_bytes;
        uint64_t created_epoch;
        uint32_t record_count;
    } _indexItems[T2_Def::StorageLimit::MAX_ROTATE_LIST_CONST];

    uint16_t _indexCount;
    SemaphoreHandle_t _lock;

    // --- 프리트리거 버퍼 (PSRAM, 유휴 상태 임시 보존) ---
    T2_Type::UnifiedFeatureSlot* _preFeatBuf;
    T2_Type::UnifiedRawChunk* _preRawBuf;
    uint16_t _preCapacity;
    uint16_t _preHeadFeat, _preCountFeat;
    uint16_t _preHeadRaw, _preCountRaw;

    // --- 비동기 기록 링버퍼 (PSRAM, 파일 기록 대기열) ---
    T2_Type::UnifiedFeatureSlot* _asyncFeatRing;
    T2_Type::UnifiedRawChunk* _asyncRawRing;
    uint16_t _asyncHead;
    uint16_t _asyncTail;

	// RTOS 스택 오버플로우 방어용 PSRAM 바운스 버퍼
    T2_Type::UnifiedFeatureSlot* _bounceFeat;
    T2_Type::UnifiedRawChunk* _bounceRaw;

    TaskHandle_t _hStorageTask;

    void _allocatePreBuffer();
    void _buildDailyDirectoryPath(char* p_outPath, size_t p_maxLen);
    void _preAllocateFile(File& p_file, uint32_t p_bytes);
    void _writeFileHeader(File& p_file, uint16_t p_structSize);

    void _flushPreBufferToRing();
    void _processAsyncRingBuffer();
    static void _storageTaskProc(void* p_param);

    void _handleRotation();
    void _appendIndexItem();
    bool _writeIndexFileAtomic();
    bool _loadIndexJson();

public:
    CL_T2_StorageManager();
    ~CL_T2_StorageManager();

    bool init();

    // 로깅 세션 시작 및 종료
    bool openSession(const char* p_prefix = "trg", const char* p_overrideDir = nullptr);
    void closeSession(const char* p_reason = "end_normal");

    // 1ms 이내 무지연(Non-blocking) 데이터 푸시 (FSM 또는 DSP 태스크에서 호출)
    bool pushFrame(const T2_Type::UnifiedFeatureSlot* p_featSlot, const T2_Type::UnifiedRawChunk* p_rawChunk);

    bool flush();
    void checkRotation(); 		// 메인 루프에서 주기적 호출하여 로테이션 검사
    bool attemptRecovery(); 	// SD카드 마운트 해제 시 자가 복구 시도

    const char* getLastRawPath() const { return _activeRawPath; }	// 캘리브레이터나 FSM에서 참조하기 위한 경로 제공
};
