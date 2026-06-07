/* ============================================================================
 * File: T260_Storage_243.cpp
 * Summary: 멀티모달 비동기 스토리지 엔진 구현부 (v243)
 * ========================================================================== */
#include "T260_Storage_243.hpp"
#include "T220_CfgMgr_243.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <sys/time.h>
#include <cstring>
#include <ArduinoJson.h>

static const char* TAG = "T243_STRG";

CL_T2_StorageManager::CL_T2_StorageManager()
    : _sessionOpen(false), _ioError(false), _recordCount(0), _indexCount(0), _rotationSubSeq(0),
      _preFeatBuf(nullptr), _preRawBuf(nullptr), _asyncFeatRing(nullptr), _asyncRawRing(nullptr),
      _bounceFeat(nullptr), _bounceRaw(nullptr), _hStorageTask(nullptr) {
    _lock = xSemaphoreCreateRecursiveMutex();
}

CL_T2_StorageManager::~CL_T2_StorageManager() {
    if (_hStorageTask) vTaskDelete(_hStorageTask);
    if (_preFeatBuf) heap_caps_free(_preFeatBuf);
    if (_preRawBuf) heap_caps_free(_preRawBuf);
    if (_asyncFeatRing) heap_caps_free(_asyncFeatRing);
    if (_asyncRawRing) heap_caps_free(_asyncRawRing);
    if (_bounceFeat) heap_caps_free(_bounceFeat);
    if (_bounceRaw) heap_caps_free(_bounceRaw);
}

bool CL_T2_StorageManager::init() {
    _allocateBuffers();

    // SD_MMC 인터페이스 활성화 확인 (상위 FSM에서 호출 전제)
    if (!SD_MMC.cardSize()) {
        ESP_LOGE(TAG, "SD Card not mounted!");
        _ioError = true;
    }

    _loadIndex();

    if (!_hStorageTask) {
        xTaskCreatePinnedToCore(_storageTaskProc, "StorageTask",
                                T2_Def::Global::Task::STORAGE_STACK_DEF, this,
                                T2_Def::Global::Task::STORAGE_PRIO_DEF, &_hStorageTask, 0);
    }

    ESP_LOGI(TAG, "Storage Engine Initialized (Async Queue: %d)", ASYNC_RING_CAPACITY);
    return true;
}

bool CL_T2_StorageManager::openSession(const char* p_prefix, const char* p_overrideDir) {
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    if (_sessionOpen) { xSemaphoreGiveRecursive(_lock); return true; }

    strncpy(_currentPrefix, p_prefix, sizeof(_currentPrefix) - 1);

    _buildDailyPath(_activePath, sizeof(_activePath), false);
    _buildDailyPath(_activeRawPath, sizeof(_activeRawPath), true);

    _activeFile = SD_MMC.open(_activePath, FILE_WRITE);
    if (!_activeFile) {
        ESP_LOGE(TAG, "Failed to open bin file: %s", _activePath);
        _ioError = true;
        xSemaphoreGiveRecursive(_lock);
        return false;
    }

    _preAllocateFile(_activeFile, T2_Def::Global::StorageLimit::PREALLOC_BYTES_DEF);
    _writeBinHeader(_activeFile);

    if (CL_T2_ConfigManager::getInstance().getConfig().storage.save_raw) {
        _rawFile = SD_MMC.open(_activeRawPath, FILE_WRITE);
        if (_rawFile) {
            _preAllocateFile(_rawFile, T2_Def::Global::StorageLimit::PREALLOC_BYTES_DEF);
            // [v013 이식] 초기 WAV 헤더 기록 (데이터 크기 0)
            _writeWavHeader(_rawFile, 0);
        }
    }

    _sessionOpen = true;
    _recordCount = 0;
    _writtenBytes = sizeof(T2_Type::FileHeader);
    _rawWrittenBytes = 0;
    _sessionStartTick = (uint32_t)(esp_timer_get_time() / 1000);

    // 프리트리거 데이터 이동
    _flushPreBufferToRing();

    ESP_LOGI(TAG, "Session Opened: %s", _activePath);
    xSemaphoreGiveRecursive(_lock);
    return true;
}

void CL_T2_StorageManager::closeSession(const char* p_reason) {
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    if (!_sessionOpen) { xSemaphoreGiveRecursive(_lock); return; }

    flush(); // 잔여 데이터 처리

    if (_activeFile) {
        // 헤더의 최종 프레임 카운트 업데이트
        _activeFile.seek(offsetof(T2_Type::FileHeader, total));
        _activeFile.write((uint8_t*)&_recordCount, sizeof(_recordCount));
        _activeFile.close();
    }

    if (_rawFile) {
        // [v013 이식] 종료 전 실제 기록된 데이터 크기로 WAV 헤더 업데이트
        _writeWavHeader(_rawFile, _rawWrittenBytes);
        _rawFile.close();
    }

    _appendIndexItem();
    _saveIndexAtomic();

    _sessionOpen = false;
    ESP_LOGI(TAG, "Session Closed (%s): %d frames", p_reason, _recordCount);
    xSemaphoreGiveRecursive(_lock);
}

bool CL_T2_StorageManager::pushFrame(const T2_Type::UnifiedFeatureSlot* p_feat, const T2_Type::UnifiedRawChunk* p_raw) {
    if (_ioError) return false;

    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);

    if (!_sessionOpen) {
        // 모니터링 중: 프리트리거 버퍼에 순환 저장
        if (_preFeatBuf) {
            _preFeatBuf[_preHead] = *p_feat;
            if (_preRawBuf && p_raw) _preRawBuf[_preHead] = *p_raw;

            _preHead = (_preHead + 1) % _preCapacity;
            if (_preCount < _preCapacity) _preCount++;
        }
    } else {
        // 기록 중: 비동기 링버퍼에 추가
        uint16_t v_nextHead = (_asyncHead + 1) % ASYNC_RING_CAPACITY;
        if (v_nextHead != _asyncTail) {
            _asyncFeatRing[_asyncHead] = *p_feat;
            if (_asyncRawRing && p_raw) _asyncRawRing[_asyncHead] = *p_raw;
            _asyncHead = v_nextHead;
        } else {
            ESP_LOGW(TAG, "Async Ring Full! Data dropped.");
        }
    }

    xSemaphoreGiveRecursive(_lock);
    return true;
}

void CL_T2_StorageManager::_allocateBuffers() {
    _preCapacity = CL_T2_ConfigManager::getInstance().getConfig().storage.pre_trig_sec * 10; // Hz=10 가정
    if (_preCapacity == 0) _preCapacity = 1;

    _preFeatBuf = (T2_Type::UnifiedFeatureSlot*)heap_caps_aligned_alloc(16, sizeof(T2_Type::UnifiedFeatureSlot) * _preCapacity, MALLOC_CAP_SPIRAM);
    _preRawBuf  = (T2_Type::UnifiedRawChunk*)heap_caps_aligned_alloc(16, sizeof(T2_Type::UnifiedRawChunk) * _preCapacity, MALLOC_CAP_SPIRAM);

    _asyncFeatRing = (T2_Type::UnifiedFeatureSlot*)heap_caps_aligned_alloc(16, sizeof(T2_Type::UnifiedFeatureSlot) * ASYNC_RING_CAPACITY, MALLOC_CAP_SPIRAM);
    _asyncRawRing  = (T2_Type::UnifiedRawChunk*)heap_caps_aligned_alloc(16, sizeof(T2_Type::UnifiedRawChunk) * ASYNC_RING_CAPACITY, MALLOC_CAP_SPIRAM);

    _bounceFeat = (T2_Type::UnifiedFeatureSlot*)heap_caps_aligned_alloc(16, sizeof(T2_Type::UnifiedFeatureSlot), MALLOC_CAP_SPIRAM);
    _bounceRaw  = (T2_Type::UnifiedRawChunk*)heap_caps_aligned_alloc(16, sizeof(T2_Type::UnifiedRawChunk), MALLOC_CAP_SPIRAM);
}

void CL_T2_StorageManager::_buildDailyPath(char* p_outPath, size_t p_maxLen, bool p_isRaw) {
    struct tm v_timeinfo;
    time_t v_now = time(NULL);
    localtime_r(&v_now, &v_timeinfo);

    const char* v_baseDir = p_isRaw ? T2_Def::Global::Storage::SD_DIR_RAW_CONST : T2_Def::Global::Storage::SD_DIR_BIN_CONST;
    char v_dir[64];

    // [v243 보완] 연/월/일 계층형 디렉토리 순차 생성
    char v_sub[32];
    snprintf(v_sub, sizeof(v_sub), "/%04d", v_timeinfo.tm_year + 1900);
    SD_MMC.mkdir((String(v_baseDir) + v_sub).c_str());

    snprintf(v_sub, sizeof(v_sub), "/%04d/%02d", v_timeinfo.tm_year + 1900, v_timeinfo.tm_mon + 1);
    SD_MMC.mkdir((String(v_baseDir) + v_sub).c_str());

    snprintf(v_dir, sizeof(v_dir), "%s/%04d/%02d/%02d", v_baseDir,
             v_timeinfo.tm_year + 1900, v_timeinfo.tm_mon + 1, v_timeinfo.tm_mday);
    SD_MMC.mkdir(v_dir);

    snprintf(p_outPath, p_maxLen, "%s/%s_%02d%02d%02d_%02d.%s",
             v_dir, _currentPrefix, v_timeinfo.tm_hour, v_timeinfo.tm_min, v_timeinfo.tm_sec,
             _rotationSubSeq, p_isRaw ? "raw" : "bin");
}

void CL_T2_StorageManager::_writeBinHeader(File& p_file) {
    T2_Type::FileHeader v_hdr;
    memset(&v_hdr, 0, sizeof(v_hdr));
    memcpy(v_hdr.magic, "T240", 4);
    v_hdr.ver = 243;
    v_hdr.struct_size = sizeof(T2_Type::UnifiedFeatureSlot);

    const auto& v_cfg = CL_T2_ConfigManager::getInstance().getConfig();
    v_hdr.s_rate = v_cfg.vib_sensor.sample_rate;
    v_hdr.fft    = v_cfg.vib_sensor.fft_size;
    v_hdr.axes   = v_cfg.vib_sensor.accel_axis_count;
    v_hdr.mfcc_d = T2_Def::Audio::FeatureLimit::MFCC_DIM_DEF;

    // [MLOps] 현재 설정을 JSON으로 덤프
    CL_T2_ConfigManager::getInstance().serializeToBuffer(v_hdr.config_dump, sizeof(v_hdr.config_dump));

    p_file.write((uint8_t*)&v_hdr, sizeof(v_hdr));
}

// [v013 이식] WAV 헤더 규격 작성 (32bit IEEE Float, Stereo)
void CL_T2_StorageManager::_writeWavHeader(File& p_file, uint32_t p_dataSize) {
    if (!p_file) return;

    const auto& v_cfg = CL_T2_ConfigManager::getInstance().getConfig();
    uint32_t v_srate = v_cfg.aud_sensor.sample_rate;
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

void CL_T2_StorageManager::_storageTaskProc(void* p_param) {
    CL_T2_StorageManager* v_this = (CL_T2_StorageManager*)p_param;
    while(1) {
        v_this->_processRingIO();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void CL_T2_StorageManager::_processRingIO() {
    if (!_sessionOpen || _ioError) return;

    while (_asyncTail != _asyncHead) {
        xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
        *_bounceFeat = _asyncFeatRing[_asyncTail];
        bool v_hasRaw = false;
        if (_asyncRawRing) {
            *_bounceRaw = _asyncRawRing[_asyncTail];
            v_hasRaw = true;
        }
        _asyncTail = (_asyncTail + 1) % ASYNC_RING_CAPACITY;
        xSemaphoreGiveRecursive(_lock);

        // 실제 파일 쓰기 (락 해제 상태에서 수행하여 타 태스크 영향 최소화)
        if (_activeFile) {
            _activeFile.write((uint8_t*)_bounceFeat, sizeof(T2_Type::UnifiedFeatureSlot));
            _recordCount++;
            _writtenBytes += sizeof(T2_Type::UnifiedFeatureSlot);
        }

        if (_rawFile && v_hasRaw) {
            _rawFile.write((uint8_t*)_bounceRaw, sizeof(T2_Type::UnifiedRawChunk));
            _rawWrittenBytes += sizeof(T2_Type::UnifiedRawChunk);
        }
    }
}

void CL_T2_StorageManager::_flushPreBufferToRing() {
    if (_preCount == 0) return;

    uint16_t v_readIdx = (_preCount == _preCapacity) ? _preHead : 0;
    for (uint16_t i = 0; i < _preCount; i++) {
        uint16_t v_nextHead = (_asyncHead + 1) % ASYNC_RING_CAPACITY;
        if (v_nextHead == _asyncTail) break;

        _asyncFeatRing[_asyncHead] = _preFeatBuf[v_readIdx];
        if (_asyncRawRing && _preRawBuf) _asyncRawRing[_asyncHead] = _preRawBuf[v_readIdx];

        _asyncHead = v_nextHead;
        v_readIdx = (v_readIdx + 1) % _preCapacity;
    }
    _preCount = 0;
    _preHead = 0;
}

void CL_T2_StorageManager::_preAllocateFile(File& p_file, uint32_t p_bytes) {
    // FATFS f_expand와 유사한 동작 (파일 포인터를 끝으로 이동시켜 물리 섹션 확보)
    p_file.seek(p_bytes - 1);
    p_file.write(0);
    p_file.seek(0);
}

void CL_T2_StorageManager::checkRotation() {
    if (!_sessionOpen) return;

    const auto& v_cfg = CL_T2_ConfigManager::getInstance().getConfig();
    uint32_t v_now = (uint32_t)(esp_timer_get_time() / 1000);

    bool v_req = false;
    if (_writtenBytes > v_cfg.storage.rot_mb * 1024 * 1024) v_req = true;
    if (v_now - _sessionStartTick > v_cfg.storage.rot_min * 60 * 1000) v_req = true;

    if (v_req) {
        _rotationSubSeq++;
        char v_prevPrefix[32];
        strncpy(v_prevPrefix, _currentPrefix, 31);
        closeSession("rotation");
        openSession(v_prevPrefix);
    }
}

// 인덱스 및 기타 관리 함수
bool CL_T2_StorageManager::_saveIndexAtomic() {
    File v_tmp = LittleFS.open("/sys/index_243.tmp", "w");
    if (!v_tmp) return false;

    JsonDocument v_doc;
    JsonArray v_arr = v_doc["files"].to<JsonArray>();
    for (uint16_t i = 0; i < _indexCount; i++) {
        JsonObject v_obj = v_arr.add<JsonObject>();
        v_obj["path"] = _indexItems[i].path;
        v_obj["raw_path"] = _indexItems[i].raw_path;
        v_obj["size"] = _indexItems[i].size_bytes;
        v_obj["created"] = _indexItems[i].created_epoch;
        v_obj["records"] = _indexItems[i].record_count;
    }

    serializeJson(v_doc, v_tmp);
    v_tmp.close();

    LittleFS.remove("/sys/runtime_idx_243.json");
    LittleFS.rename("/sys/index_243.tmp", "/sys/runtime_idx_243.json");
    return true;
}

bool CL_T2_StorageManager::_loadIndex() {
    if (!LittleFS.exists("/sys/runtime_idx_243.json")) return false;

    File v_file = LittleFS.open("/sys/runtime_idx_243.json", "r");
    if (!v_file) return false;

    JsonDocument v_doc;
    DeserializationError v_err = deserializeJson(v_doc, v_file);
    v_file.close();
    if (v_err) return false;

    JsonArrayConst v_arr = v_doc["files"];
    _indexCount = 0;
    for (JsonObjectConst v_obj : v_arr) {
        if (_indexCount >= T2_Def::Global::StorageLimit::MAX_ROTATE_LIST_MAX) break;
        strlcpy(_indexItems[_indexCount].path, v_obj["path"] | "", T2_Def::Global::StorageLimit::MAX_PATH_LEN_CONST);
        strlcpy(_indexItems[_indexCount].raw_path, v_obj["raw_path"] | "", T2_Def::Global::StorageLimit::MAX_PATH_LEN_CONST);
        _indexItems[_indexCount].size_bytes = v_obj["size"] | 0;
        _indexItems[_indexCount].created_epoch = v_obj["created"] | 0;
        _indexItems[_indexCount].record_count = v_obj["records"] | 0;
        _indexCount++;
    }
    return true;
}

void CL_T2_StorageManager::_appendIndexItem() {
    if (_indexCount >= T2_Def::Global::StorageLimit::MAX_ROTATE_LIST_MAX) {
        // [GC] 가장 오래된 파일 삭제 및 목록 시프트
        if (SD_MMC.exists(_indexItems[0].path)) SD_MMC.remove(_indexItems[0].path);
        if (SD_MMC.exists(_indexItems[0].raw_path)) SD_MMC.remove(_indexItems[0].raw_path);

        for (uint16_t i = 0; i < T2_Def::Global::StorageLimit::MAX_ROTATE_LIST_MAX - 1; i++) {
            _indexItems[i] = _indexItems[i + 1];
        }
        _indexCount--;
    }

    strlcpy(_indexItems[_indexCount].path, _activePath, T2_Def::Global::StorageLimit::MAX_PATH_LEN_CONST);
    strlcpy(_indexItems[_indexCount].raw_path, _activeRawPath, T2_Def::Global::StorageLimit::MAX_PATH_LEN_CONST);
    _indexItems[_indexCount].size_bytes = _writtenBytes;
    _indexItems[_indexCount].created_epoch = (uint64_t)time(NULL);
    _indexItems[_indexCount].record_count = _recordCount;
    _indexCount++;
}

bool CL_T2_StorageManager::flush() {
    if (!_sessionOpen || _ioError) return false;
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    if (_activeFile) _activeFile.flush();
    if (_rawFile) _rawFile.flush();
    xSemaphoreGiveRecursive(_lock);
    return true;
}

bool CL_T2_StorageManager::attemptRecovery() {
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    if (!_ioError) {
        xSemaphoreGiveRecursive(_lock); return true;
    }

    ESP_LOGW(TAG, "Attempting SD Card Auto-Recovery (v243)...");
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
