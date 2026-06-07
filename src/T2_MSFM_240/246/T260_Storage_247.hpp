/* ============================================================================
 * File: T260_Storage_247.hpp
 * Summary: 멀티모달 비동기 스토리지 엔진 및 개별 Raw 파일 저장기 (v245 개정판)
 * ============================================================================ */


// TODO: Phase 4 - T260_Storage_247 Extractor Parameter Alignment
//   T260_Storage_247.cpp 내의 _processRingIO() 동작 중 호출되는
//   extractAccel, extractAudio 등의 특징 추출 함수들이
//	 쪼개진 개별 슬롯 구조체(ST_FeatureSlot_Aud_t, ST_FeatureSlot_Vib_t)를 인자로 수용하여 처리할 수 있도록
//	 파라미터 정합성 리팩토링이 연쇄적으로 이어져야 합니다.


#pragma once

#include "T210_Def_247.hpp"
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
	static constexpr uint8_t  STORAGE_QUEUE_LEN   = 32;

    bool     _sessionOpen;
    bool     _ioError;
    uint32_t _recordCount;
    uint32_t _rotationSubSeq;

	// 도메인별 분리형 파일 디스크립터
	File _audBinFile;    // 오디오 특징량 (.aud.bin)
    File _vibBinFile;    // 진동 특징량 (.vib.bin)
    File _wavFile;       // 오디오 원시 파형 (.wav)
    File _accFile;       // 가속도 원시 파형 (.acc)
    File _gyrFile;       // 자이로 원시 파형 (.gyr)

	// 도메인별 절대 경로 버퍼
    char _audBinPath[T2_Def::Global::StorageLimit::PATH_LEN_MAX];
    char _vibBinPath[T2_Def::Global::StorageLimit::PATH_LEN_MAX];
    char _wavPath[T2_Def::Global::StorageLimit::PATH_LEN_MAX];
    char _accPath[T2_Def::Global::StorageLimit::PATH_LEN_MAX];
    char _gyrPath[T2_Def::Global::StorageLimit::PATH_LEN_MAX];
    char _currentPrefix[T2_Def::Global::StorageLimit::PREFIX_LEN_MAX];

	uint32_t _sessionStartTick;
    uint32_t _audBinWrittenBytes;
    uint32_t _vibBinWrittenBytes;
    uint32_t _wavWrittenBytes;
    uint32_t _accWrittenBytes;
    uint32_t _gyrWrittenBytes;


    // 인덱스 매니저 구조 확장 (5대 채널 경로 동시 추적)
    struct IndexItem {
        char aud_bin_path[T2_Def::Global::StorageLimit::PATH_LEN_MAX];
        char vib_bin_path[T2_Def::Global::StorageLimit::PATH_LEN_MAX];
        char wav_path[T2_Def::Global::StorageLimit::PATH_LEN_MAX];
        char acc_path[T2_Def::Global::StorageLimit::PATH_LEN_MAX];
        char gyr_path[T2_Def::Global::StorageLimit::PATH_LEN_MAX];
        uint32_t size_bytes;
        uint64_t created_epoch;
        uint32_t record_count;
    } _indexItems[T2_Def::Global::StorageLimit::ROTATE_LIST_MAX];

    uint16_t          _indexCount;
    SemaphoreHandle_t _lock;

	// 도메인별 비동기 전송용 프리토스 고속 큐 핸들 (인덱스만 전송)
    QueueHandle_t _qAudStorage;
    QueueHandle_t _qVibStorage;

    // --- 프리트리거 버퍼 (PSRAM) ---
    T2_Type::ST_FeatureSlot_Aud_t* _preAudBuf;
    T2_Type::ST_FeatureSlot_Vib_t* _preVibBuf;
    T2_Type::ST_Raw_Accel_t* _preAccBuf;
    T2_Type::ST_Raw_Gyro_t* _preGyrBuf;
    T2_Type::ST_Raw_Audio_t* _preAudRawBuf;

	uint16_t _preAudCapacity;
    uint16_t _preVibCapacity;
    uint16_t _preAudHead, _preAudCount;
    uint16_t _preVibHead, _preVibCount;


    // --- 비동기 기록 링버퍼 (PSRAM) ---
    T2_Type::ST_FeatureSlot_Aud_t* _asyncAudRing;
    T2_Type::ST_FeatureSlot_Vib_t* _asyncVibRing;
    T2_Type::ST_Raw_Accel_t* _asyncAccRing;
    T2_Type::ST_Raw_Gyro_t* _asyncGyrRing;
    T2_Type::ST_Raw_Audio_t* _asyncAudRawRing;

    uint16_t _asyncAudHead, _asyncAudTail;
    uint16_t _asyncVibHead, _asyncVibTail;

    // DMA 캐시 정렬 조건 충족 내부 바운스 버퍼 (SRAM/PSRAM 16B 정렬)
    T2_Type::ST_FeatureSlot_Aud_t* _bounceAudFeat;
    T2_Type::ST_FeatureSlot_Vib_t* _bounceVibFeat;
    T2_Type::ST_Raw_Accel_t* _bounceAcc;
    T2_Type::ST_Raw_Gyro_t* _bounceGyr;
    T2_Type::ST_Raw_Audio_t* _bounceAudRaw;

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

	// 파이프라인 분리에 대응하는 이원화 인터페이스 정의
    bool pushAudioFrame(const T2_Type::ST_FeatureSlot_Aud_t* p_audFeat,
                        const T2_Type::ST_Raw_Audio_t* p_rawAud);

    bool pushVibFrame(const T2_Type::ST_FeatureSlot_Vib_t* p_vibFeat,
                      const T2_Type::ST_Raw_Accel_t* p_rawAcc,
                      const T2_Type::ST_Raw_Gyro_t* p_rawGyr);


    const char* getLastPath() const { return _vibBinPath; }

    bool flush();
    void checkRotation();
    bool attemptRecovery();
    bool isSessionOpen() const { return _sessionOpen; }
    bool hasIoError() const { return _ioError; }

private:
    void _allocateBuffers();
    void _buildDailyPath(char* p_outPath, size_t p_maxLen, const char* p_ext);
    void _writeBinHeader(fs::File&, bool);
    void _writeWavHeader(File& p_file, uint32_t p_dataSize);
    void _preAllocateFile(File& p_file, uint32_t p_bytes);

    void _flushPreBufferToRing();
    void _processRingIO();
    static void _storageTaskProc(void* p_param);

    bool _saveIndexAtomic();
    bool _loadIndex();
    void _appendIndexItem();
};



