/* ============================================================================
 * File: T260_Storage_250.cpp
 * Summary: 멀티모달 비동기 스토리지 엔진 구현부
 * ============================================================================ */

#include "T260_Storage_250.hpp"
#include "T220_CfgMgr_250.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <sys/time.h>
#include <cstring>
#include <ArduinoJson.h>
#include <Preferences.h>

static const char* TAG = "T260_STRG";

CL_T2_StorageManager::CL_T2_StorageManager()
    : _sessionOpen(false),
      _ioError(false),
      _recordCount(0),
      _indexCount(0),
      _rotationSubSeq(0),
      _audBinWrittenBytes(0),
      _vibBinWrittenBytes(0),
      _wavWrittenBytes(0),
      _accWrittenBytes(0),
      _gyrWrittenBytes(0),
      _sessionStartTick(0),
      _sessionStartUs(0),

      // [수정] 분리형 프리트리거 버퍼 포인터 초기화
      _preAudBuf(nullptr),
      _preVibBuf(nullptr),
      _preAccBuf(nullptr),
      _preGyrBuf(nullptr),
      _preAudRawBuf(nullptr),
      _preAudCapacity(0),
      _preVibCapacity(0),
      _preAudHead(0), _preAudCount(0),
      _preVibHead(0), _preVibCount(0),

      // [수정] 분리형 비동기 기록 링버퍼 포인터 초기화
      _asyncAudRing(nullptr),
      _asyncVibRing(nullptr),
      _asyncAccRing(nullptr),
      _asyncGyrRing(nullptr),
      _asyncAudRawRing(nullptr),
      _asyncAudHead(0), _asyncAudTail(0),
      _asyncVibHead(0), _asyncVibTail(0),

      // [수정] 분리형 내부 바운스 버퍼 포인터 초기화
      _bounceAudFeat(nullptr),
      _bounceVibFeat(nullptr),
      _bounceAcc(nullptr),
      _bounceGyr(nullptr),
      _bounceAudRaw(nullptr),

      // 통신 큐 및 태스크 핸들 초기화
      _qAudStorage(nullptr),
      _qVibStorage(nullptr),
      _hStorageTask(nullptr) {

    // 전역 자원 제어용 고속 재귀 뮤텍스 생성
    _lock = xSemaphoreCreateRecursiveMutex();
    _fsLock = xSemaphoreCreateMutex(); // [신규] LittleFS 뮤텍스 생성

    memset(&_triggerReason, 0, sizeof(_triggerReason));

    // 경로 및 접두사 버퍼 안전 초기화
    memset(_audBinPath, 0, sizeof(_audBinPath));
    memset(_vibBinPath, 0, sizeof(_vibBinPath));
    memset(_wavPath, 0, sizeof(_wavPath));
    memset(_accPath, 0, sizeof(_accPath));
    memset(_gyrPath, 0, sizeof(_gyrPath));
    memset(_currentPrefix, 0, sizeof(_currentPrefix));
}

CL_T2_StorageManager::~CL_T2_StorageManager() {
    if (_hStorageTask) {
        vTaskDelete(_hStorageTask);
        _hStorageTask = nullptr;
    }

    if (_lock) { vSemaphoreDelete(_lock); _lock = nullptr; }
    if (_fsLock) { vSemaphoreDelete(_fsLock); _fsLock = nullptr; }

    // 프리트리거 도메인 분리 버퍼 해제
    if (_preAudBuf)    { heap_caps_free(_preAudBuf);    _preAudBuf = nullptr; }
    if (_preVibBuf)    { heap_caps_free(_preVibBuf);    _preVibBuf = nullptr; }
    if (_preAccBuf)    { heap_caps_free(_preAccBuf);    _preAccBuf = nullptr; }
    if (_preGyrBuf)    { heap_caps_free(_preGyrBuf);    _preGyrBuf = nullptr; }
    if (_preAudRawBuf) { heap_caps_free(_preAudRawBuf); _preAudRawBuf = nullptr; }

    // 비동기 링버퍼 도메인 분리 풀 해제
    if (_asyncAudRing)    { heap_caps_free(_asyncAudRing);    _asyncAudRing = nullptr; }
    if (_asyncVibRing)    { heap_caps_free(_asyncVibRing);    _asyncVibRing = nullptr; }
    if (_asyncAccRing)    { heap_caps_free(_asyncAccRing);    _asyncAccRing = nullptr; }
    if (_asyncGyrRing)    { heap_caps_free(_asyncGyrRing);    _asyncGyrRing = nullptr; }
    if (_asyncAudRawRing) { heap_caps_free(_asyncAudRawRing); _asyncAudRawRing = nullptr; }

    // 고속 바운스 버퍼 해제
    if (_bounceAudFeat) { heap_caps_free(_bounceAudFeat); _bounceAudFeat = nullptr; }
    if (_bounceVibFeat) { heap_caps_free(_bounceVibFeat); _bounceVibFeat = nullptr; }
    if (_bounceAcc)     { heap_caps_free(_bounceAcc);     _bounceAcc = nullptr; }
    if (_bounceGyr)     { heap_caps_free(_bounceGyr);     _bounceGyr = nullptr; }
    if (_bounceAudRaw)  { heap_caps_free(_bounceAudRaw);  _bounceAudRaw = nullptr; }
}

bool CL_T2_StorageManager::init() {
    _allocateBuffers();

    if (!SD_MMC.cardSize()) {
        ESP_LOGE(TAG, "SD Card not mounted!");
        _ioError = true;
    }

    _loadIndex();

    // NVS에서 _rotationSubSeq 복구
    Preferences prefs;
    prefs.begin("storage", true);
    _rotationSubSeq = prefs.getUShort("file_seq", 0);
    prefs.end();

    if (!_hStorageTask) {
        xTaskCreatePinnedToCore(_storageTaskProc, "StorageTask",
                                T2_Def::Global::Task::STORAGE_STACK_DEF, this,
                                T2_Def::Global::Task::STORAGE_PRIO_DEF, &_hStorageTask, 0);
    }

    ESP_LOGI(TAG, "Storage Engine Initialized (Async Queue: %d, Seq: %d)", CL_T2_StorageManager::ASYNC_RING_CAPACITY, _rotationSubSeq);
    return true;
}

bool CL_T2_StorageManager::openSession(const char* p_prefix, uint64_t p_triggerTimestamp, const T2_Type::ST_TriggerReason_t& p_reason, const char* p_overrideDir) {
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    if (_sessionOpen) { xSemaphoreGiveRecursive(_lock); return true; }

    _sessionStartUs = p_triggerTimestamp;
    _triggerReason = p_reason;

    strncpy(_currentPrefix, p_prefix, sizeof(_currentPrefix) - 1);

    // 파일 경로 개별 수립
    _buildDailyPath(_audBinPath, sizeof(_audBinPath), "aud.bin");
    _buildDailyPath(_vibBinPath, sizeof(_vibBinPath), "vib.bin");
    _buildDailyPath(_wavPath, sizeof(_wavPath), "wav");
    _buildDailyPath(_accPath, sizeof(_accPath), "acc");
    _buildDailyPath(_gyrPath, sizeof(_gyrPath), "gyr");

    // 오디오 바이너리 오픈 및 헤더 기록
    _audBinFile = SD_MMC.open(_audBinPath, FILE_WRITE);
    if (_audBinFile) {
        _preAllocateFile(_audBinFile, T2_Def::Global::StorageLimit::PREALLOC_BYTES_MAX);
        _writeBinHeader(_audBinFile, true); // Audio Flag = true
    }

    // 진동 바이너리 오픈 및 헤더 기록
    _vibBinFile = SD_MMC.open(_vibBinPath, FILE_WRITE);
    if (!_vibBinFile) {
        ESP_LOGE(TAG, "Fatal: Failed to open vibration bin file!");
        _ioError = true;
        if (_audBinFile) _audBinFile.close();
        xSemaphoreGiveRecursive(_lock);
        return false;
    }
    _preAllocateFile(_vibBinFile, T2_Def::Global::StorageLimit::PREALLOC_BYTES_MAX);
    _writeBinHeader(_vibBinFile, false); // Audio Flag = false

    const auto& v_cfg = CL_T2_ConfigManager::getInstance().getConfig();
    if (v_cfg.storage.save_raw) {
        if (v_cfg.accel.enable) {
            _accFile = SD_MMC.open(_accPath, FILE_WRITE);
            if (_accFile) _preAllocateFile(_accFile, T2_Def::Global::StorageLimit::PREALLOC_BYTES_MAX);
        }
        if (v_cfg.gyro.enable) {
            _gyrFile = SD_MMC.open(_gyrPath, FILE_WRITE);
            if (_gyrFile) _preAllocateFile(_gyrFile, T2_Def::Global::StorageLimit::PREALLOC_BYTES_MAX);
        }
        if (v_cfg.audio.enable) {
            _wavFile = SD_MMC.open(_wavPath, FILE_WRITE);
            if (_wavFile) {
                _preAllocateFile(_wavFile, T2_Def::Global::StorageLimit::PREALLOC_BYTES_MAX);
                _writeWavHeader(_wavFile, 0);
            }
        }
    }


    // 통신 오디오/진동 큐 신규 클리어 및 인덱싱 리셋
	_qAudStorage = xQueueCreate(CL_T2_StorageManager::STORAGE_QUEUE_LEN, sizeof(uint8_t));
    _qVibStorage = xQueueCreate(CL_T2_StorageManager::STORAGE_QUEUE_LEN, sizeof(uint8_t));

    // _qAudStorage = xQueueCreate(STORAGE_QUEUE_LEN, sizeof(uint8_t));
    // _qVibStorage = xQueueCreate(STORAGE_QUEUE_LEN, sizeof(uint8_t));

    _sessionOpen = true;
    _recordCount = 0;
    _audBinWrittenBytes = sizeof(T2_Type::ST_FileHeader_t);
    _vibBinWrittenBytes = sizeof(T2_Type::ST_FileHeader_t);
    _wavWrittenBytes = 0; _accWrittenBytes = 0; _gyrWrittenBytes = 0;
    _sessionStartTick = (uint32_t)(esp_timer_get_time() / 1000);

    dumpPreTriggerToSession(); // 프리트리거를 세션 파일들로 즉각 덤프

    ESP_LOGI(TAG, "Domain Separated Session Opened Successful.");
    xSemaphoreGiveRecursive(_lock);
    return true;
}

bool CL_T2_StorageManager::openSession(const char* p_prefix, const char* p_overrideDir) {
    T2_Type::ST_TriggerReason_t defaultReason = {0};
    defaultReason.trigger_axis = 0xFF;
    strncpy(defaultReason.metric_name, "MANUAL", sizeof(defaultReason.metric_name) - 1);
    uint64_t dummyT0 = esp_timer_get_time();
    return openSession(p_prefix, dummyT0, defaultReason, p_overrideDir);
}

void CL_T2_StorageManager::closeSession(const char* p_reason) {
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    if (!_sessionOpen) { xSemaphoreGiveRecursive(_lock); return; }

    flush();

    // 개별 바이너리 파일 디스크립터 정리 및 레코드 정보 영속화
    if (_audBinFile) _audBinFile.close();
    if (_vibBinFile) {
        _vibBinFile.seek(offsetof(T2_Type::ST_FileHeader_t, total));
        _vibBinFile.write((uint8_t*)&_recordCount, sizeof(_recordCount));
        _vibBinFile.close();
    }
    if (_wavFile) { _writeWavHeader(_wavFile, _wavWrittenBytes); _wavFile.close(); }
    if (_accFile) _accFile.close();
    if (_gyrFile) _gyrFile.close();

    // 동기화용 큐 완전 소멸
    if (_qAudStorage) { vQueueDelete(_qAudStorage); _qAudStorage = nullptr; }
    if (_qVibStorage) { vQueueDelete(_qVibStorage); _qVibStorage = nullptr; }

    _appendIndexItem();
    _saveIndexAtomic();

    _sessionOpen = false;
    ESP_LOGI(TAG, "Async Shared Session Closed (%s). Master Count: %d", p_reason, _recordCount);
    xSemaphoreGiveRecursive(_lock);
}


bool CL_T2_StorageManager::pushAudioFrame(const T2_Type::ST_FeatureSlot_Aud_t* p_audFeat, const T2_Type::ST_Raw_Audio_t* p_rawAud) {
    if (_ioError) return false;
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);

    if (!_sessionOpen) {
        if (_preAudBuf) {
            _preAudBuf[_preAudHead] = *p_audFeat;
            if (_preAudRawBuf && p_rawAud) _preAudRawBuf[_preAudHead] = *p_rawAud;
            _preAudHead = (_preAudHead + 1) % _preAudCapacity;
            if (_preAudCount < _preAudCapacity) _preAudCount++;
        }
    } else {
        uint16_t v_nextHead = (_asyncAudHead + 1) % CL_T2_StorageManager::ASYNC_RING_CAPACITY;
        if (v_nextHead != _asyncAudTail) {
            _asyncAudRing[_asyncAudHead] = *p_audFeat;
            if (_asyncAudRawRing && p_rawAud) _asyncAudRawRing[_asyncAudHead] = *p_rawAud; // _asyncHead 오타 전면 수정

            uint8_t v_slotIdx = _asyncAudHead;
            _asyncAudHead = v_nextHead;
            xQueueSend(_qAudStorage, &v_slotIdx, 0);
        } else {
            ESP_LOGW(TAG, "Audio Storage Queue Full! Dropped.");
        }
    }
    xSemaphoreGiveRecursive(_lock);
    return true;
}

bool CL_T2_StorageManager::pushVibFrame(const T2_Type::ST_FeatureSlot_Vib_t* p_vibFeat, const T2_Type::ST_Raw_Accel_t* p_rawAcc, const T2_Type::ST_Raw_Gyro_t* p_rawGyr) {
    if (_ioError) return false;
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);

    if (!_sessionOpen) {
        if (_preVibBuf) {
            _preVibBuf[_preVibHead] = *p_vibFeat;
            if (_preAccBuf && p_rawAcc) _preAccBuf[_preVibHead] = *p_rawAcc;
            if (_preGyrBuf && p_rawGyr) _preGyrBuf[_preVibHead] = *p_rawGyr;
            _preVibHead = (_preVibHead + 1) % _preVibCapacity;
            if (_preVibCount < _preVibCapacity) _preVibCount++;
        }
    } else {
        uint16_t v_nextHead = (_asyncVibHead + 1) % CL_T2_StorageManager::ASYNC_RING_CAPACITY;
        if (v_nextHead != _asyncVibTail) {
            _asyncVibRing[_asyncVibHead] = *p_vibFeat;
            if (_asyncAccRing && p_rawAcc) _asyncAccRing[_asyncVibHead] = *p_rawAcc;
            if (_asyncGyrRing && p_rawGyr) _asyncGyrRing[_asyncVibHead] = *p_rawGyr;

            uint8_t v_slotIdx = _asyncVibHead;
            _asyncVibHead = v_nextHead;
            xQueueSend(_qVibStorage, &v_slotIdx, 0);
        } else {
            ESP_LOGW(TAG, "Vibration Storage Queue Full! Dropped.");
        }
    }
    xSemaphoreGiveRecursive(_lock);
    return true;
}

void CL_T2_StorageManager::_allocateBuffers() {
    const auto& v_cfg = CL_T2_ConfigManager::getInstance().getConfig();

    // 오디오 및 진동 개별 수집 주기 상수를 고려하여 프리트리거 프레임 깊이 계산
    _preAudCapacity = v_cfg.storage.pre_trig_sec * (1000 / 24); // 24.3ms 주기 역산
    _preVibCapacity = v_cfg.storage.pre_trig_sec * (1000 / 640); // 640ms 주기 역산
    if (_preAudCapacity == 0) _preAudCapacity = 1;
    if (_preVibCapacity == 0) _preVibCapacity = 1;

    // 프리트리거 링버퍼 동적 할당 (PSRAM)
    _preAudBuf    = (T2_Type::ST_FeatureSlot_Aud_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_FeatureSlot_Aud_t) * _preAudCapacity, MALLOC_CAP_SPIRAM);
    _preVibBuf    = (T2_Type::ST_FeatureSlot_Vib_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_FeatureSlot_Vib_t) * _preVibCapacity, MALLOC_CAP_SPIRAM);
    _preAccBuf    = (T2_Type::ST_Raw_Accel_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_Raw_Accel_t) * _preVibCapacity, MALLOC_CAP_SPIRAM);
    _preGyrBuf    = (T2_Type::ST_Raw_Gyro_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_Raw_Gyro_t) * _preVibCapacity, MALLOC_CAP_SPIRAM);
    _preAudRawBuf = (T2_Type::ST_Raw_Audio_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_Raw_Audio_t) * _preAudCapacity, MALLOC_CAP_SPIRAM);

    // 비동기 기록 스트리밍용 인덱싱 링버퍼 풀 동적 할당 (PSRAM)
    _asyncAudRing    = (T2_Type::ST_FeatureSlot_Aud_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_FeatureSlot_Aud_t) * CL_T2_StorageManager::ASYNC_RING_CAPACITY, MALLOC_CAP_SPIRAM);
    _asyncVibRing    = (T2_Type::ST_FeatureSlot_Vib_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_FeatureSlot_Vib_t) * CL_T2_StorageManager::ASYNC_RING_CAPACITY, MALLOC_CAP_SPIRAM);
    _asyncAccRing    = (T2_Type::ST_Raw_Accel_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_Raw_Accel_t) * CL_T2_StorageManager::ASYNC_RING_CAPACITY, MALLOC_CAP_SPIRAM);
    _asyncGyrRing    = (T2_Type::ST_Raw_Gyro_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_Raw_Gyro_t) * CL_T2_StorageManager::ASYNC_RING_CAPACITY, MALLOC_CAP_SPIRAM);
    _asyncAudRawRing = (T2_Type::ST_Raw_Audio_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_Raw_Audio_t) * CL_T2_StorageManager::ASYNC_RING_CAPACITY, MALLOC_CAP_SPIRAM);

    // 고속 바운스 버퍼 할당
    _bounceAudFeat = (T2_Type::ST_FeatureSlot_Aud_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_FeatureSlot_Aud_t), MALLOC_CAP_SPIRAM);
    _bounceVibFeat = (T2_Type::ST_FeatureSlot_Vib_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_FeatureSlot_Vib_t), MALLOC_CAP_SPIRAM);
    _bounceAcc     = (T2_Type::ST_Raw_Accel_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_Raw_Accel_t), MALLOC_CAP_SPIRAM);
    _bounceGyr     = (T2_Type::ST_Raw_Gyro_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_Raw_Gyro_t), MALLOC_CAP_SPIRAM);
    _bounceAudRaw  = (T2_Type::ST_Raw_Audio_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_Raw_Audio_t), MALLOC_CAP_SPIRAM);

    if (!_preAudBuf || !_preVibBuf || !_preAccBuf || !_preGyrBuf || !_preAudRawBuf ||
        !_asyncAudRing || !_asyncVibRing || !_asyncAccRing || !_asyncGyrRing || !_asyncAudRawRing ||
        !_bounceAudFeat || !_bounceVibFeat || !_bounceAcc || !_bounceGyr || !_bounceAudRaw) {
        ESP_LOGE(TAG, "Critical: Multimodal Storage PSRAM Allocation Failed!");
        _ioError = true;
    }
}

void CL_T2_StorageManager::_buildDailyPath(char* p_outPath, size_t p_maxLen, const char* p_ext) {
    struct tm v_timeinfo;
    time_t v_now = time(NULL);
    localtime_r(&v_now, &v_timeinfo);

    bool isRaw = (strcmp(p_ext, "bin") != 0);
    const char* v_baseDir = isRaw ? T2_Def::Global::Storage::SD_DIR_RAW_CONST : T2_Def::Global::Storage::SD_DIR_BIN_CONST;

    char v_sub[32];
    snprintf(v_sub, sizeof(v_sub), "/%04d", v_timeinfo.tm_year + 1900);
    SD_MMC.mkdir((String(v_baseDir) + v_sub).c_str());

    snprintf(v_sub, sizeof(v_sub), "/%04d/%02d", v_timeinfo.tm_year + 1900, v_timeinfo.tm_mon + 1);
    SD_MMC.mkdir((String(v_baseDir) + v_sub).c_str());

    char v_dir[64];
    snprintf(v_dir, sizeof(v_dir), "%s/%04d/%02d/%02d", v_baseDir,
             v_timeinfo.tm_year + 1900, v_timeinfo.tm_mon + 1, v_timeinfo.tm_mday);
    SD_MMC.mkdir(v_dir);

    snprintf(p_outPath, p_maxLen, "%s/%s_%02d%02d%02d_%02d.%s",
             v_dir, _currentPrefix, v_timeinfo.tm_hour, v_timeinfo.tm_min, v_timeinfo.tm_sec,
             _rotationSubSeq, p_ext);
}

void CL_T2_StorageManager::_writeBinHeader(File& p_file, bool p_isAudio) {
    T2_Type::ST_FileHeader_t v_hdr;
    memset(&v_hdr, 0, sizeof(v_hdr));

    if (p_isAudio) {
        memcpy(v_hdr.magic, "T2AU", 4);
        v_hdr.struct_size = sizeof(T2_Type::ST_FeatureSlot_Aud_t);
    } else {
        memcpy(v_hdr.magic, "T2VI", 4);
        v_hdr.struct_size = sizeof(T2_Type::ST_FeatureSlot_Vib_t);
    }
    v_hdr.ver = 246; // v246 아키텍처 식별 명시

    const auto& v_cfg = CL_T2_ConfigManager::getInstance().getConfig();
    v_hdr.s_rate = v_cfg.accel.sample_rate;
    v_hdr.fft    = v_cfg.accel.fft_size;
    v_hdr.mfcc_d = T2_Def::AI::Tensor::MFCC_DIM_DEF;
    v_hdr.accel_mask = v_cfg.accel.axis_mask;
    v_hdr.gyro_mask  = v_cfg.gyro.axis_mask;
    v_hdr.audio_mask = (uint8_t)v_cfg.audio.channel_mask;
    v_hdr.trigger_t0 = _sessionStartUs;
    v_hdr.reason = _triggerReason;

    CL_T2_ConfigManager::getInstance().serializeToBuffer(v_hdr.config_dump, sizeof(v_hdr.config_dump));
    p_file.write((uint8_t*)&v_hdr, sizeof(v_hdr));
}

void CL_T2_StorageManager::_writeWavHeader(File& p_file, uint32_t p_dataSize) {
    if (!p_file) return;

    const auto& v_cfg = CL_T2_ConfigManager::getInstance().getConfig();
    uint32_t v_srate = v_cfg.audio.sample_rate;
    uint16_t v_chans = 2; // Stereo 고정
    uint16_t v_bps   = 32; // 32bit Float

    uint8_t v_hdr[44];
    memcpy(v_hdr, "RIFF", 4);
    uint32_t v_chunkSize = p_dataSize + 36;
    memcpy(v_hdr + 4, &v_chunkSize, 4);
    memcpy(v_hdr + 8, "WAVE", 4);
    memcpy(v_hdr + 12, "fmt ", 4);
    uint32_t v_sub1Size = 16;
    memcpy(v_hdr + 16, &v_sub1Size, 4);
    uint16_t v_format = 3; // 3 = IEEE Float
    memcpy(v_hdr + 20, &v_format, 2);
    memcpy(v_hdr + 22, &v_chans, 2);
    memcpy(v_hdr + 24, &v_srate, 4);
    uint32_t v_byteRate = v_srate * v_chans * (v_bps / 8);
    memcpy(v_hdr + 28, &v_byteRate, 4);
    uint16_t v_blockAlign = v_chans * (v_bps / 8);
    memcpy(v_hdr + 32, &v_blockAlign, 2);
    memcpy(v_hdr + 34, &v_bps, 2);
    memcpy(v_hdr + 36, "data", 4);
    memcpy(v_hdr + 40, &p_dataSize, 4);

    p_file.seek(0);
    p_file.write(v_hdr, 44);
}

void CL_T2_StorageManager::_preAllocateFile(File& p_file, uint32_t p_bytes) {
    p_file.seek(p_bytes - 1);
    p_file.write(0);
    p_file.seek(0);
}

void CL_T2_StorageManager::_storageTaskProc(void* p_param) {
    CL_T2_StorageManager* v_this = (CL_T2_StorageManager*)p_param;
    while(1) {
        v_this->_processRingIO();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ============================================================================
 * File: T260_Storage_250.cpp (_processRingIO 데이터 찢어짐 가드 교정 완전체)
 * ============================================================================ */

void CL_T2_StorageManager::_processRingIO() {
    if (!_sessionOpen || _ioError) return;

    // [처리 단위 1] 오디오 도메인 비동기 저장 드레인 (최대 8개 청크 연속 처리 버스트 상한 제한)
    uint8_t v_audIdx;
    uint8_t v_audBurst = 0;
    while (xQueueReceive(_qAudStorage, &v_audIdx, 0) == pdTRUE && v_audBurst++ < 8) {
        xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
        // 정적 구조체 내부 배열 데이터 전체를 안전하게 딥카피 (포인터 얕은 복사 리스크 파괴)
        *_bounceAudFeat = _asyncAudRing[v_audIdx];
        *_bounceAudRaw  = _asyncAudRawRing[v_audIdx];
        _asyncAudTail   = (_asyncAudTail + 1) % CL_T2_StorageManager::ASYNC_RING_CAPACITY;
        xSemaphoreGiveRecursive(_lock);

        if (_audBinFile) {
            _audBinFile.write((uint8_t*)_bounceAudFeat, sizeof(T2_Type::ST_FeatureSlot_Aud_t));
            _audBinWrittenBytes += sizeof(T2_Type::ST_FeatureSlot_Aud_t);
        }
        if (_wavFile) {
            // Internal SRAM 바운스 스페이스를 활용한 인터리빙 가속
            alignas(16) static float s_interleave[T2_Def::Audio::Sensor::FFT_SIZE_MAX * 2];
            for (uint32_t i = 0; i < T2_Def::Audio::Sensor::FFT_SIZE_MAX; i++) {
                s_interleave[i * 2]     = _bounceAudRaw->data[0][i]; // Left Channel
                s_interleave[i * 2 + 1] = _bounceAudRaw->data[1][i]; // Right Channel
            }
            _wavFile.write((uint8_t*)s_interleave, sizeof(s_interleave));
            _wavWrittenBytes += sizeof(s_interleave);
        }
    }

    // [처리 단위 2] 진동 도메인 비동기 저장 드레인 (최대 2개 청크 연속 처리 버스트 상한 제한)
    uint8_t v_vibIdx;
    uint8_t v_vibBurst = 0;
    while (xQueueReceive(_qVibStorage, &v_vibIdx, 0) == pdTRUE && v_vibBurst++ < 2) {
        xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
        *_bounceVibFeat = _asyncVibRing[v_vibIdx];
        *_bounceAcc     = _asyncAccRing[v_vibIdx];
        *_bounceGyr     = _asyncGyrRing[v_vibIdx];
        _asyncVibTail   = (_asyncVibTail + 1) % CL_T2_StorageManager::ASYNC_RING_CAPACITY;
        xSemaphoreGiveRecursive(_lock);

        if (_vibBinFile) {
            _vibBinFile.write((uint8_t*)_bounceVibFeat, sizeof(T2_Type::ST_FeatureSlot_Vib_t));
            _vibBinWrittenBytes += sizeof(T2_Type::ST_FeatureSlot_Vib_t);
            _recordCount++; // 진동 세트 도달 기준 진단 마스터 프레임 인덱스 카운트 업
        }
        if (_accFile) {
            _accFile.write((uint8_t*)_bounceAcc, sizeof(T2_Type::ST_Raw_Accel_t));
            _accWrittenBytes += sizeof(T2_Type::ST_Raw_Accel_t);
        }
        if (_gyrFile) {
            _gyrFile.write((uint8_t*)_bounceGyr, sizeof(T2_Type::ST_Raw_Gyro_t));
            _gyrWrittenBytes += sizeof(T2_Type::ST_Raw_Gyro_t);
        }
    }
}

void CL_T2_StorageManager::dumpPreTriggerToSession() {
    // 2.21 & 2.8 모순 해결을 위한 스냅샷 바운스 버퍼 도입
    // _lock을 획득하고 링 버퍼에 있는 데이터를 임시 로컬 메모리 버퍼로 빠르게 일괄 복사한 후 _lock을 즉시 해제
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    
    uint16_t audCount = _preAudCount;
    uint16_t audReadIdx = (audCount == _preAudCapacity) ? _preAudHead : 0;
    uint16_t vibCount = _preVibCount;
    uint16_t vibReadIdx = (vibCount == _preVibCapacity) ? _preVibHead : 0;

    // 임시 복사용 바운스 버퍼를 스택/힙에 임시 확보하여 복사 진행 (SD write는 Lock 밖에서 수행)
    T2_Type::ST_FeatureSlot_Aud_t* tempAudBuf = nullptr;
    T2_Type::ST_Raw_Audio_t* tempAudRawBuf = nullptr;
    T2_Type::ST_FeatureSlot_Vib_t* tempVibBuf = nullptr;
    T2_Type::ST_Raw_Accel_t* tempAccBuf = nullptr;
    T2_Type::ST_Raw_Gyro_t* tempGyrBuf = nullptr;

    if (audCount > 0) {
        tempAudBuf = (T2_Type::ST_FeatureSlot_Aud_t*)heap_caps_malloc(sizeof(T2_Type::ST_FeatureSlot_Aud_t) * audCount, MALLOC_CAP_SPIRAM);
        tempAudRawBuf = (T2_Type::ST_Raw_Audio_t*)heap_caps_malloc(sizeof(T2_Type::ST_Raw_Audio_t) * audCount, MALLOC_CAP_SPIRAM);
        for (uint16_t i = 0; i < audCount; i++) {
            if (tempAudBuf) tempAudBuf[i] = _preAudBuf[audReadIdx];
            if (tempAudRawBuf) tempAudRawBuf[i] = _preAudRawBuf[audReadIdx];
            audReadIdx = (audReadIdx + 1) % _preAudCapacity;
        }
    }

    if (vibCount > 0) {
        tempVibBuf = (T2_Type::ST_FeatureSlot_Vib_t*)heap_caps_malloc(sizeof(T2_Type::ST_FeatureSlot_Vib_t) * vibCount, MALLOC_CAP_SPIRAM);
        tempAccBuf = (T2_Type::ST_Raw_Accel_t*)heap_caps_malloc(sizeof(T2_Type::ST_Raw_Accel_t) * vibCount, MALLOC_CAP_SPIRAM);
        tempGyrBuf = (T2_Type::ST_Raw_Gyro_t*)heap_caps_malloc(sizeof(T2_Type::ST_Raw_Gyro_t) * vibCount, MALLOC_CAP_SPIRAM);
        for (uint16_t i = 0; i < vibCount; i++) {
            if (tempVibBuf) tempVibBuf[i] = _preVibBuf[vibReadIdx];
            if (tempAccBuf) tempAccBuf[i] = _preAccBuf[vibReadIdx];
            if (tempGyrBuf) tempGyrBuf[i] = _preGyrBuf[vibReadIdx];
            vibReadIdx = (vibReadIdx + 1) % _preVibCapacity;
        }
    }

    _preAudCount = 0; _preAudHead = 0;
    _preVibCount = 0; _preVibHead = 0;
    xSemaphoreGiveRecursive(_lock);

    // Lock 해제 후 안전하게 SD_MMC 카드에 순차 Write 수행
    if (tempAudBuf && tempAudRawBuf) {
        for (uint16_t i = 0; i < audCount; i++) {
            if (_audBinFile) {
                _audBinFile.write((uint8_t*)&tempAudBuf[i], sizeof(T2_Type::ST_FeatureSlot_Aud_t));
                _audBinWrittenBytes += sizeof(T2_Type::ST_FeatureSlot_Aud_t);
            }
            if (_wavFile) {
                alignas(16) static float s_interleave[T2_Def::Audio::Sensor::FFT_SIZE_MAX * 2];
                for (uint32_t j = 0; j < T2_Def::Audio::Sensor::FFT_SIZE_MAX; j++) {
                    s_interleave[j * 2]     = tempAudRawBuf[i].data[0][j];
                    s_interleave[j * 2 + 1] = tempAudRawBuf[i].data[1][j];
                }
                _wavFile.write((uint8_t*)s_interleave, sizeof(s_interleave));
                _wavWrittenBytes += sizeof(s_interleave);
            }
        }
    }

    if (tempVibBuf && tempAccBuf && tempGyrBuf) {
        for (uint16_t i = 0; i < vibCount; i++) {
            if (_vibBinFile) {
                _vibBinFile.write((uint8_t*)&tempVibBuf[i], sizeof(T2_Type::ST_FeatureSlot_Vib_t));
                _vibBinWrittenBytes += sizeof(T2_Type::ST_FeatureSlot_Vib_t);
                _recordCount++;
            }
            if (_accFile) {
                _accFile.write((uint8_t*)&tempAccBuf[i], sizeof(T2_Type::ST_Raw_Accel_t));
                _accWrittenBytes += sizeof(T2_Type::ST_Raw_Accel_t);
            }
            if (_gyrFile) {
                _gyrFile.write((uint8_t*)&tempGyrBuf[i], sizeof(T2_Type::ST_Raw_Gyro_t));
                _gyrWrittenBytes += sizeof(T2_Type::ST_Raw_Gyro_t);
            }
        }
    }

    if (tempAudBuf) heap_caps_free(tempAudBuf);
    if (tempAudRawBuf) heap_caps_free(tempAudRawBuf);
    if (tempVibBuf) heap_caps_free(tempVibBuf);
    if (tempAccBuf) heap_caps_free(tempAccBuf);
    if (tempGyrBuf) heap_caps_free(tempGyrBuf);
}

void CL_T2_StorageManager::_flushPreBufferToRing() {
    dumpPreTriggerToSession();
}

bool CL_T2_StorageManager::saveNoiseProfile(const float* p_profile, size_t p_size) {
    if (p_profile == nullptr || p_size == 0) return false;
    xSemaphoreTake(_fsLock, portMAX_DELAY);
    File v_file = LittleFS.open("/sys/noise_profile.bin", "w");
    if (!v_file) {
        xSemaphoreGive(_fsLock);
        return false;
    }
    size_t written = v_file.write(reinterpret_cast<const uint8_t*>(p_profile), p_size * sizeof(float));
    v_file.close();
    xSemaphoreGive(_fsLock);
    return (written == p_size * sizeof(float));
}

bool CL_T2_StorageManager::loadNoiseProfile(float* p_profile, size_t p_size) {
    if (p_profile == nullptr || p_size == 0) return false;
    xSemaphoreTake(_fsLock, portMAX_DELAY);
    if (!LittleFS.exists("/sys/noise_profile.bin")) {
        xSemaphoreGive(_fsLock);
        return false;
    }
    File v_file = LittleFS.open("/sys/noise_profile.bin", "r");
    if (!v_file) {
        xSemaphoreGive(_fsLock);
        return false;
    }
    size_t read_bytes = v_file.read(reinterpret_cast<uint8_t*>(p_profile), p_size * sizeof(float));
    v_file.close();
    xSemaphoreGive(_fsLock);
    return (read_bytes == p_size * sizeof(float));
}

void CL_T2_StorageManager::checkRotation() {
    if (!_sessionOpen) return;

    const auto& v_cfg = CL_T2_ConfigManager::getInstance().getConfig();
    uint32_t v_now = (uint32_t)(esp_timer_get_time() / 1000);

    // 총 기록된 물리 특징량 바이너리 크기를 합산하여 로테이션 트리거링 판정
    uint32_t v_totalFeatureBytes = _vibBinWrittenBytes + _audBinWrittenBytes;

    bool v_req = false;
    if (v_totalFeatureBytes > (uint32_t)v_cfg.storage.rot_mb * 1024 * 1024) v_req = true;
    if (v_now - _sessionStartTick > (uint32_t)v_cfg.storage.rot_min * 60 * 1000) v_req = true;

    if (v_req) {
        _rotationSubSeq++;
        Preferences prefs;
        prefs.begin("storage", false);
        prefs.putUShort("file_seq", _rotationSubSeq);
        prefs.end();

        char v_prevPrefix[32];
        strncpy(v_prevPrefix, _currentPrefix, sizeof(v_prevPrefix) - 1);
        v_prevPrefix[sizeof(v_prevPrefix) - 1] = '\0';

        closeSession("rotation");
        openSession(v_prevPrefix, _sessionStartUs, _triggerReason);
    }
}

bool CL_T2_StorageManager::_saveIndexAtomic() {
    File v_tmp = LittleFS.open(T2_Def::Global::Path::FILE_IDX_TMP_CONST, "w");
    if (!v_tmp) return false;

    JsonDocument v_doc;
    JsonArray v_arr = v_doc["files"].to<JsonArray>();
    for (uint16_t i = 0; i < _indexCount; i++) {
        JsonObject v_obj = v_arr.add<JsonObject>();
        v_obj["aud_bin_path"] = _indexItems[i].aud_bin_path;
        v_obj["vib_bin_path"] = _indexItems[i].vib_bin_path;
        v_obj["wav_path"]     = _indexItems[i].wav_path;
        v_obj["acc_path"]     = _indexItems[i].acc_path;
        v_obj["gyr_path"]     = _indexItems[i].gyr_path;
        v_obj["size"]         = _indexItems[i].size_bytes;
        v_obj["created"]      = _indexItems[i].created_epoch;
        v_obj["records"]      = _indexItems[i].record_count;
    }

    serializeJson(v_doc, v_tmp);
    v_tmp.close();

    LittleFS.remove(T2_Def::Global::Path::FILE_IDX_JSON_CONST);
    return LittleFS.rename(T2_Def::Global::Path::FILE_IDX_TMP_CONST, T2_Def::Global::Path::FILE_IDX_JSON_CONST);
}

bool CL_T2_StorageManager::_loadIndex() {
    if (!LittleFS.exists(T2_Def::Global::Path::FILE_IDX_JSON_CONST)) return false;

    File v_file = LittleFS.open(T2_Def::Global::Path::FILE_IDX_JSON_CONST, "r");
    if (!v_file) return false;

    JsonDocument v_doc;
    DeserializationError v_err = deserializeJson(v_doc, v_file);
    v_file.close();
    if (v_err) return false;

    JsonArrayConst v_arr = v_doc["files"];
    _indexCount = 0;
    for (JsonObjectConst v_obj : v_arr) {
        if (_indexCount >= T2_Def::Global::StorageLimit::ROTATE_LIST_MAX) break;
        strlcpy(_indexItems[_indexCount].aud_bin_path, v_obj["aud_bin_path"] | "", T2_Def::Global::StorageLimit::PATH_LEN_MAX);
        strlcpy(_indexItems[_indexCount].vib_bin_path, v_obj["vib_bin_path"] | "", T2_Def::Global::StorageLimit::PATH_LEN_MAX);
        strlcpy(_indexItems[_indexCount].wav_path,     v_obj["wav_path"] | "",     T2_Def::Global::StorageLimit::PATH_LEN_MAX);
        strlcpy(_indexItems[_indexCount].acc_path,     v_obj["acc_path"] | "",     T2_Def::Global::StorageLimit::PATH_LEN_MAX);
        strlcpy(_indexItems[_indexCount].gyr_path,     v_obj["gyr_path"] | "",     T2_Def::Global::StorageLimit::PATH_LEN_MAX);
        _indexItems[_indexCount].size_bytes    = v_obj["size"] | 0;
        _indexItems[_indexCount].created_epoch = v_obj["created"] | 0;
        _indexItems[_indexCount].record_count  = v_obj["records"] | 0;
        _indexCount++;
    }
    return true;
}

void CL_T2_StorageManager::_appendIndexItem() {
    if (_indexCount >= T2_Def::Global::StorageLimit::ROTATE_LIST_MAX) {
        // 예비 리스트 만료 시 인덱스 선두 파일 디스크 삭제 유도 및 쉬프트
        if (SD_MMC.exists(_indexItems[0].aud_bin_path)) SD_MMC.remove(_indexItems[0].aud_bin_path);
        if (SD_MMC.exists(_indexItems[0].vib_bin_path)) SD_MMC.remove(_indexItems[0].vib_bin_path);
        if (SD_MMC.exists(_indexItems[0].wav_path))     SD_MMC.remove(_indexItems[0].wav_path);
        if (SD_MMC.exists(_indexItems[0].acc_path))     SD_MMC.remove(_indexItems[0].acc_path);
        if (SD_MMC.exists(_indexItems[0].gyr_path))     SD_MMC.remove(_indexItems[0].gyr_path);

        for (uint16_t i = 0; i < T2_Def::Global::StorageLimit::ROTATE_LIST_MAX - 1; i++) {
            _indexItems[i] = _indexItems[i + 1];
        }
        _indexCount--;
    }

    strlcpy(_indexItems[_indexCount].aud_bin_path, _audBinPath, T2_Def::Global::StorageLimit::PATH_LEN_MAX);
    strlcpy(_indexItems[_indexCount].vib_bin_path, _vibBinPath, T2_Def::Global::StorageLimit::PATH_LEN_MAX);
    strlcpy(_indexItems[_indexCount].wav_path,     _wavPath,    T2_Def::Global::StorageLimit::PATH_LEN_MAX);
    strlcpy(_indexItems[_indexCount].acc_path,     _accPath,    T2_Def::Global::StorageLimit::PATH_LEN_MAX);
    strlcpy(_indexItems[_indexCount].gyr_path,     _gyrPath,    T2_Def::Global::StorageLimit::PATH_LEN_MAX);

    _indexItems[_indexCount].size_bytes    = _vibBinWrittenBytes + _audBinWrittenBytes;
    _indexItems[_indexCount].created_epoch = (uint64_t)time(NULL);
    _indexItems[_indexCount].record_count  = _recordCount;
    _indexCount++;
}

bool CL_T2_StorageManager::flush() {
    if (!_sessionOpen || _ioError) return false;
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);

    // 이원화된 특징량 및 Raw 스트리밍 파일 강제 싱크
    if (_audBinFile) _audBinFile.flush();
    if (_vibBinFile) _vibBinFile.flush();
    if (_wavFile)    _wavFile.flush();
    if (_accFile)    _accFile.flush();
    if (_gyrFile)    _gyrFile.flush();

    xSemaphoreGiveRecursive(_lock);
    return true;
}

bool CL_T2_StorageManager::attemptRecovery() {
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    if (!_ioError) {
        xSemaphoreGiveRecursive(_lock); return true;
    }

    ESP_LOGW(TAG, "Attempting SD Card Auto-Recovery...");
    SD_MMC.end();
    vTaskDelay(pdMS_TO_TICKS(200));

    if (SD_MMC.begin(T2_Def::Global::Path::MOUNT_SD_CONST, true)) {
        _ioError = false;
        ESP_LOGI(TAG, "SD Card Recovery Successful!");
    } else {
        ESP_LOGE(TAG, "SD Card Recovery Failed.");
    }

    xSemaphoreGiveRecursive(_lock);
    return !_ioError;
}
