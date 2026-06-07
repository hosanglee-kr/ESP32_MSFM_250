/* ============================================================================
 * File: T260_Storage_244.hpp
 * Summary: 멀티모달 비동기 스토리지 엔진 및 프리트리거 매니저 (v243)
 * ============================================================================
 * [로직 상태 추적 (Logic State Tracker)]
 * - 유지: 백그라운드 태스크 기반 SD_MMC 비차단(Non-blocking) 기록 아키텍처.
 * - 갱신: v243 UnifiedFeatureSlot(800B) 및 UnifiedRawChunk 규격 정합.
 * - 신규: [MLOps] 파일 헤더 내 8KB JSON 설정 덤프(config_dump) 기록 기능 탑재.
 * ========================================================================== */
#pragma once

#include "T210_Def_244.hpp"
#include "T215_Type_244.hpp"
#include <FS.h>
#include <LittleFS.h>
#include <SD_MMC.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

class CL_T2_StorageManager {
private:
    static constexpr uint16_t ASYNC_RING_CAPACITY = 128; // RawChunk 크기 고려 조정 (30KB * 128 = ~3.8MB)

    bool _sessionOpen;
    bool _ioError;
    uint32_t _recordCount;
    uint32_t _rotationSubSeq;

    File _activeFile;    // 특징량(.bin) 기록용
    File _rawFile;       // 원시 파형(.raw) 기록용

    char _activePath[T2_Def::Global::StorageLimit::MAX_PATH_LEN_CONST];
    char _activeRawPath[T2_Def::Global::StorageLimit::MAX_PATH_LEN_CONST];
    char _currentPrefix[T2_Def::Global::StorageLimit::MAX_PREFIX_LEN_CONST];

    uint32_t _sessionStartTick;
    uint32_t _writtenBytes;
    uint32_t _rawWrittenBytes;

    // 인덱스 관리 (로테이션 최신성 유지용)
    struct IndexItem {
        char path[T2_Def::Global::StorageLimit::MAX_PATH_LEN_CONST];
        char raw_path[T2_Def::Global::StorageLimit::MAX_PATH_LEN_CONST];
        uint32_t size_bytes;
        uint64_t created_epoch;
        uint32_t record_count;
    } _indexItems[T2_Def::Global::StorageLimit::MAX_ROTATE_LIST_MAX];

    uint16_t _indexCount;
    SemaphoreHandle_t _lock;

    // --- 프리트리거 버퍼 (PSRAM) ---
    T2_Type::UnifiedFeatureSlot* _preFeatBuf;
    T2_Type::UnifiedRawChunk*    _preRawBuf;
    uint16_t _preCapacity;
    uint16_t _preHead, _preCount;

    // --- 비동기 기록 링버퍼 (PSRAM, 파일 I/O 지연 격리용) ---
    T2_Type::UnifiedFeatureSlot* _asyncFeatRing;
    T2_Type::UnifiedRawChunk*    _asyncRawRing;
    uint16_t _asyncHead;
    uint16_t _asyncTail;

    // PSRAM 바운스 버퍼 (스택 보호용)
    T2_Type::UnifiedFeatureSlot* _bounceFeat;
    T2_Type::UnifiedRawChunk*    _bounceRaw;

    TaskHandle_t _hStorageTask;

public:
    CL_T2_StorageManager();
    ~CL_T2_StorageManager();

    static CL_T2_StorageManager& getInstance() {
        static CL_T2_StorageManager v_inst;
        return v_inst;
    }

    bool init();
    bool openSession(const char* p_prefix, const char* p_overrideDir = nullptr);
    void closeSession(const char* p_reason);
    bool pushFrame(const T2_Type::UnifiedFeatureSlot* p_feat, const T2_Type::UnifiedRawChunk* p_raw);

    // [v013 이식] 마지막 바이너리 파일 경로 반환 (교정용)
    const char* getLastPath() const { return _activePath; }

    bool flush();
    void checkRotation();
    bool attemptRecovery();
    bool isSessionOpen() const { return _sessionOpen; }
    bool hasIoError() const { return _ioError; }

private:
    // 내부 헬퍼
    void _allocateBuffers();
    void _buildDailyPath(char* p_outPath, size_t p_maxLen, bool p_isRaw);
    void _writeBinHeader(File& p_file);
    void _writeWavHeader(File& p_file, uint32_t p_dataSize); // [v013 이식] WAV 헤더 주입
    void _preAllocateFile(File& p_file, uint32_t p_bytes);

    void _flushPreBufferToRing();
    void _processRingIO();
    static void _storageTaskProc(void* p_param);

    // 인덱스 및 기타 관리
    bool _saveIndexAtomic();
    bool _loadIndex();
    void _appendIndexItem();
};
