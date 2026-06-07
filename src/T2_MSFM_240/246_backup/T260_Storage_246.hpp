/* ============================================================================
 * File: T260_Storage_246.hpp
 * Summary: 멀티모달 비동기 스토리지 엔진 및 개별 Raw 파일 저장기 (v245 개정판)
 * ============================================================================ */

#pragma once

#include "T210_Def_246.hpp"
#include "T215_Type_247.hpp"
#include <FS.h>
#include <LittleFS.h>
#include <SD_MMC.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

class CL_T2_StorageManager {
private:
    static constexpr uint16_t ASYNC_RING_CAPACITY = 128;

    bool _sessionOpen;
    bool _ioError;
    uint32_t _recordCount;
    uint32_t _rotationSubSeq;

    File _activeFile;    // 특징량 (.bin)
    File _wavFile;       // 오디오 (.wav)
    File _accFile;       // 가속도 (.acc)
    File _gyrFile;       // 자이로 (.gyr)

    char _activePath[T2_Def::Global::StorageLimit::PATH_LEN_MAX];
    char _wavPath[T2_Def::Global::StorageLimit::PATH_LEN_MAX];
    char _accPath[T2_Def::Global::StorageLimit::PATH_LEN_MAX];
    char _gyrPath[T2_Def::Global::StorageLimit::PATH_LEN_MAX];
    char _currentPrefix[T2_Def::Global::StorageLimit::PREFIX_LEN_MAX];

    uint32_t _sessionStartTick;
    uint32_t _writtenBytes;
    uint32_t _wavWrittenBytes;
    uint32_t _accWrittenBytes;
    uint32_t _gyrWrittenBytes;

    // 인덱스 관리 (로테이션)
    struct IndexItem {
        char path[T2_Def::Global::StorageLimit::PATH_LEN_MAX];
        char wav_path[T2_Def::Global::StorageLimit::PATH_LEN_MAX];
        char acc_path[T2_Def::Global::StorageLimit::PATH_LEN_MAX];
        char gyr_path[T2_Def::Global::StorageLimit::PATH_LEN_MAX];
        uint32_t size_bytes;
        uint64_t created_epoch;
        uint32_t record_count;
    } _indexItems[T2_Def::Global::StorageLimit::ROTATE_LIST_MAX];

    uint16_t _indexCount;
    SemaphoreHandle_t _lock;

    // --- 프리트리거 버퍼 (PSRAM) ---
    T2_Type::ST_UnifiedFeatureSlot_t* _preFeatBuf;
    T2_Type::ST_Raw_Accel_t*       _preAccBuf;
    T2_Type::ST_Raw_Gyro_t*        _preGyrBuf;
    T2_Type::ST_Raw_Audio_t*       _preAudBuf;
    uint16_t _preCapacity;
    uint16_t _preHead, _preCount;

    // --- 비동기 기록 링버퍼 (PSRAM) ---
    T2_Type::ST_UnifiedFeatureSlot_t* _asyncFeatRing;
    T2_Type::ST_Raw_Accel_t*       _asyncAccRing;
    T2_Type::ST_Raw_Gyro_t*        _asyncGyrRing;
    T2_Type::ST_Raw_Audio_t*       _asyncAudRing;
    uint16_t _asyncHead;
    uint16_t _asyncTail;

    // PSRAM 바운스 버퍼 (스택 보호용)
    T2_Type::ST_UnifiedFeatureSlot_t* _bounceFeat;
    T2_Type::ST_Raw_Accel_t*       _bounceAcc;
    T2_Type::ST_Raw_Gyro_t*        _bounceGyr;
    T2_Type::ST_Raw_Audio_t*       _bounceAud;

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

    // v245 분리형 수집 Raw 파형을 받도록 pushFrame 인터페이스 리팩토링
    bool pushFrame(const T2_Type::ST_UnifiedFeatureSlot_t* p_feat,
                   const T2_Type::ST_Raw_Accel_t* p_rawAcc,
                   const T2_Type::ST_Raw_Gyro_t* p_rawGyr,
                   const T2_Type::ST_Raw_Audio_t* p_rawAud);

    const char* getLastPath() const { return _activePath; }

    bool flush();
    void checkRotation();
    bool attemptRecovery();
    bool isSessionOpen() const { return _sessionOpen; }
    bool hasIoError() const { return _ioError; }

private:
    void _allocateBuffers();
    void _buildDailyPath(char* p_outPath, size_t p_maxLen, const char* p_ext);
    void _writeBinHeader(File& p_file);
    void _writeWavHeader(File& p_file, uint32_t p_dataSize);
    void _preAllocateFile(File& p_file, uint32_t p_bytes);

    void _flushPreBufferToRing();
    void _processRingIO();
    static void _storageTaskProc(void* p_param);

    bool _saveIndexAtomic();
    bool _loadIndex();
    void _appendIndexItem();
};
