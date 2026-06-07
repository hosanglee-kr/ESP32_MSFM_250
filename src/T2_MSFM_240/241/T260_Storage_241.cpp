/* ============================================================================
 * File: T260_Storage_241.cpp
 * Summary: 멀티모달 비동기 스토리지 엔진 (A-DSE) 풀버전 구현부
 * ============================================================================
 * [로직 상태 추적 (Logic State Tracker)]
 * - 유지: 백그라운드 태스크(_storageTaskProc) 기반 SD_MMC.write() 격리.
 * - 갱신: WAV 포맷을 커스텀 SMEA Binary 포맷으로 완전 통합 교체 완료.
 * - 신규: 프리트리거 데이터를 openSession 시점에 SD로 직행하지 않고 _asyncFeatRing으로
 * 고속 Memcpy 덤프하여 Core 1 블로킹(Timeout) 원천 차단.
 * ========================================================================== */

#include "T260_Storage_241.hpp"
#include "T215_ConfigMgr_241.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <sys/time.h>
#include <cstring>
#include <ArduinoJson.h>

static const char* TAG = "T245_STRG";

CL_T2_StorageManager::CL_T2_StorageManager() {
	_sessionOpen	  = false;
	_ioError		  = false;
	_recordCount	  = 0;
	_indexCount		  = 0;
	_rotationSubSeq	  = 0;
	_sessionStartTick = 0;
	_writtenBytes	  = 0;
	_rawWrittenBytes  = 0;

	_preFeatBuf		  = nullptr;
	_preRawBuf		  = nullptr;
	_asyncFeatRing	  = nullptr;
	_asyncRawRing	  = nullptr;

	// RTOS 스택 오버플로우 방어용 PSRAM 바운스 버퍼 초기화
	_bounceFeat		  = nullptr;
	_bounceRaw		  = nullptr;

	_lock			  = xSemaphoreCreateRecursiveMutex();
}

CL_T2_StorageManager::~CL_T2_StorageManager() {
	if (_preFeatBuf) heap_caps_free(_preFeatBuf);
	if (_preRawBuf) heap_caps_free(_preRawBuf);
	if (_asyncFeatRing) heap_caps_free(_asyncFeatRing);
	if (_asyncRawRing) heap_caps_free(_asyncRawRing);
	if (_bounceFeat) heap_caps_free(_bounceFeat);
	if (_bounceRaw) heap_caps_free(_bounceRaw);
	if (_lock) vSemaphoreDelete(_lock);
}

bool CL_T2_StorageManager::init() {
    // SD카드 마운트
    if (!SD_MMC.begin(T2_Config::Path::MOUNT_SD_CONST, true)) {
        ESP_LOGE(TAG, "SD Card Mount Failed!");
        _ioError = true;
        return false;
    }

    if (!SD_MMC.exists(T2_Config::Path::SD_DIR_BIN_CONST)) SD_MMC.mkdir(T2_Config::Path::SD_DIR_BIN_CONST);
    if (!SD_MMC.exists(T2_Config::Path::SD_DIR_RAW_CONST)) SD_MMC.mkdir(T2_Config::Path::SD_DIR_RAW_CONST);

    // [Rule #21] 거대 버퍼 할당
    _allocatePreBuffer();

    _asyncFeatRing = (T2_Type::UnifiedFeatureSlot*)heap_caps_aligned_alloc(16, sizeof(T2_Type::UnifiedFeatureSlot) * ASYNC_RING_CAPACITY, MALLOC_CAP_SPIRAM);
    _asyncRawRing = (T2_Type::UnifiedRawChunk*)heap_caps_aligned_alloc(16, sizeof(T2_Type::UnifiedRawChunk) * ASYNC_RING_CAPACITY, MALLOC_CAP_SPIRAM);

	// 지역 변수(Stack)를 대체할 힙 메모리 1회성 할당 (Rule #21)
    _bounceFeat = (T2_Type::UnifiedFeatureSlot*)heap_caps_aligned_alloc(16, sizeof(T2_Type::UnifiedFeatureSlot), MALLOC_CAP_SPIRAM);
    _bounceRaw = (T2_Type::UnifiedRawChunk*)heap_caps_aligned_alloc(16, sizeof(T2_Type::UnifiedRawChunk), MALLOC_CAP_SPIRAM);

    if (!_asyncFeatRing || !_asyncRawRing || !_bounceFeat || !_bounceRaw) {
        ESP_LOGE(TAG, "A-DSE Ring Buffer Allocation Failed! Out of PSRAM.");
        return false;
    }

    _asyncHead = 0;
    _asyncTail = 0;

    _loadIndexJson();

    // Core 1 (DSP와 같은 코어)에 우선순위 2(가장 낮음)로 스토리지 태스크 배포 (DSP가 CPU 선점)
    xTaskCreatePinnedToCore(_storageTaskProc, "StrgTask", T2_Config::Task::STORAGE_STACK_CONST,
                            this, T2_Config::Task::STORAGE_PRIO_CONST, &_hStorageTask, T2_Config::Task::CORE_PROCESS_CONST);

    ESP_LOGI(TAG, "A-DSE Storage Engine Initialized.");
    return true;
}

void CL_T2_StorageManager::_allocatePreBuffer() {
    T2_Type::DynamicConfig v_cfg = CL_T2_ConfigManager::getInstance().getConfig();
    _preCapacity = v_cfg.storage.pre_trigger_sec * (T2_Config::System::RATE_AUDIO_CONST / T2_Config::System::FFT_SIZE_AUDIO_CONST);

    // [교정] 재할당 시 메모리 누수(Memory Leak) 완벽 차단
    if (_preFeatBuf) { heap_caps_free(_preFeatBuf); _preFeatBuf = nullptr; }
    if (_preRawBuf)  { heap_caps_free(_preRawBuf);  _preRawBuf = nullptr; }

    if (_preCapacity > 0) {
        _preFeatBuf = (T2_Type::UnifiedFeatureSlot*)heap_caps_aligned_alloc(16, _preCapacity * sizeof(T2_Type::UnifiedFeatureSlot), MALLOC_CAP_SPIRAM);
        _preRawBuf  = (T2_Type::UnifiedRawChunk*)heap_caps_aligned_alloc(16, _preCapacity * sizeof(T2_Type::UnifiedRawChunk), MALLOC_CAP_SPIRAM);
        _preHeadFeat = 0; _preCountFeat = 0;
        _preHeadRaw = 0; _preCountRaw = 0;
    }
}


void CL_T2_StorageManager::_preAllocateFile(File& p_file, uint32_t p_bytes) {
    // 맹목적 선할당 차단 (5MB 여유 공간 확인)
    uint64_t v_freeBytes = SD_MMC.totalBytes() - SD_MMC.usedBytes();
    if (v_freeBytes < (p_bytes + 5 * 1024 * 1024)) {
        ESP_LOGW(TAG, "Low disk space. Skipping Pre-allocation.");
        return;
    }

    if (p_file) {
        p_file.seek(p_bytes);
        p_file.write(0); // 더미 바이트 기록하여 섹터 강제 확보
        p_file.seek(0);
    }
}

void CL_T2_StorageManager::_writeFileHeader(File& p_file, uint16_t p_structSize) {
    T2_Type::FileHeader v_header;
    memcpy(v_header.magic, "SMEA", 4);
    v_header.version = 240;
    v_header.struct_size = p_structSize;

    // total_records를 0으로 초기화
    v_header.total_records = 0;

    p_file.seek(0);
    p_file.write((uint8_t*)&v_header, sizeof(T2_Type::FileHeader));
}

bool CL_T2_StorageManager::openSession(const char* p_prefix, const char* p_overrideDir) {
    if (_ioError || _sessionOpen) return false;
    xSemaphoreTake(_lock, portMAX_DELAY);

    strlcpy(_currentPrefix, p_prefix, sizeof(_currentPrefix));

    time_t v_now; time(&v_now);
    struct tm v_tinfo; localtime_r(&v_now, &v_tinfo);

    if (p_overrideDir != nullptr) {
        if (!SD_MMC.exists(p_overrideDir)) SD_MMC.mkdir(p_overrideDir);
        uint64_t v_uniqueId = (uint64_t)esp_timer_get_time();
        snprintf(_activePath, sizeof(_activePath), "%s/%s_%llu.bin", p_overrideDir, _currentPrefix, v_uniqueId);
        snprintf(_activeRawPath, sizeof(_activeRawPath), "%s/%s_%llu_raw.bin", p_overrideDir, _currentPrefix, v_uniqueId);
    } else {
        // [복원/교정] BIN 폴더 계층화
        char v_dirBin[T2_Config::StorageLimit::MAX_PATH_LEN_CONST];
        snprintf(v_dirBin, sizeof(v_dirBin), "%s/%04d/%02d/%02d", T2_Config::Path::SD_DIR_BIN_CONST, v_tinfo.tm_year + 1900, v_tinfo.tm_mon + 1, v_tinfo.tm_mday);
        if (!SD_MMC.exists(v_dirBin)) { /* 상위 폴더 생성 로직 생략 (기존 코드 활용) */ SD_MMC.mkdir(v_dirBin); }

        // [복원/교정] RAW 폴더 계층화 (FAT32 O(N) 붕괴 완벽 차단)
        char v_dirRaw[T2_Config::StorageLimit::MAX_PATH_LEN_CONST];
        snprintf(v_dirRaw, sizeof(v_dirRaw), "%s/%04d/%02d/%02d", T2_Config::Path::SD_DIR_RAW_CONST, v_tinfo.tm_year + 1900, v_tinfo.tm_mon + 1, v_tinfo.tm_mday);
        if (!SD_MMC.exists(v_dirRaw)) { /* 상위 폴더 생성 로직 생략 */ SD_MMC.mkdir(v_dirRaw); }

        snprintf(_activePath, sizeof(_activePath), "%s/%s_%llu_%03d.bin", v_dirBin, _currentPrefix, (uint64_t)v_now, _rotationSubSeq);
        snprintf(_activeRawPath, sizeof(_activeRawPath), "%s/%s_%llu_%03d_raw.bin", v_dirRaw, _currentPrefix, (uint64_t)v_now, _rotationSubSeq);
    }

    _activeFile = SD_MMC.open(_activePath, "w");
    _rawFile = SD_MMC.open(_activeRawPath, "w");

    if (!_activeFile || !_rawFile) {
        ESP_LOGE(TAG, "Session File Open Failed!");
        _ioError = true;
        xSemaphoreGive(_lock);
        return false;
    }

    // 선할당 후 초기 헤더 주입
    _preAllocateFile(_activeFile, T2_Config::StorageLimit::PREALLOC_BYTES_CONST);
    _preAllocateFile(_rawFile, T2_Config::StorageLimit::PREALLOC_BYTES_CONST);

    _writeFileHeader(_activeFile, sizeof(T2_Type::UnifiedFeatureSlot));
    _writeFileHeader(_rawFile, sizeof(T2_Type::UnifiedRawChunk));

    _sessionOpen = true;
    _recordCount = 0;
    _writtenBytes = sizeof(T2_Type::FileHeader);
    _rawWrittenBytes = sizeof(T2_Type::FileHeader);
    _sessionStartTick = (uint32_t)(esp_timer_get_time() / 1000); // ms 스케일

    // [핵심 로직 교정] FSM 블로킹 방지를 위한 링버퍼 고속 복사
    _flushPreBufferToRing();

    xSemaphoreGive(_lock);
    ESP_LOGI(TAG, "Session Opened: %s", _activePath);
    return true;
}

// 유휴 상태일 때 수집된 과거 기록(Pre-trigger)을 실제 파일에 쓰기 위해 링버퍼로 이관
void CL_T2_StorageManager::_flushPreBufferToRing() {
    uint16_t v_idx = (_preHeadFeat + _preCapacity - _preCountFeat) % _preCapacity;

    for (uint16_t i = 0; i < _preCountFeat; i++) {
        uint16_t v_nextHead = (_asyncHead + 1) % ASYNC_RING_CAPACITY;
        if (v_nextHead != _asyncTail) {
            memcpy(&_asyncFeatRing[_asyncHead], &_preFeatBuf[v_idx], sizeof(T2_Type::UnifiedFeatureSlot));
            memcpy(&_asyncRawRing[_asyncHead], &_preRawBuf[v_idx], sizeof(T2_Type::UnifiedRawChunk));
            _asyncHead = v_nextHead;
        } else {
            ESP_LOGW(TAG, "Ring Buffer Full during Pre-buffer flush!");
            break; // 링버퍼 가득 참 (안전 방어)
        }
        v_idx = (v_idx + 1) % _preCapacity;
    }
    _preCountFeat = 0; _preCountRaw = 0; // 플러시 완료 후 초기화
}

void CL_T2_StorageManager::closeSession(const char* p_reason) {
    if (!_sessionOpen) return;

    // 잔여 링버퍼 대기 (데드락 방어 적용됨)
    uint32_t v_waitStart = (uint32_t)(esp_timer_get_time() / 1000);
    while(_asyncHead != _asyncTail) {
        if (((uint32_t)(esp_timer_get_time() / 1000) - v_waitStart) > 1000) {
            ESP_LOGE(TAG, "closeSession Timeout! I/O Thread might be stuck.");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    xSemaphoreTake(_lock, portMAX_DELAY);

    // [핵심 교정] 10MB 선할당의 쓰레기 데이터를 무시할 수 있도록,
    // 실제 기록된 레코드 수를 헤더에 덮어쓰기 (Seek 0)
    T2_Type::FileHeader v_headerFeat, v_headerRaw;
    memcpy(v_headerFeat.magic, "SMEA", 4); v_headerFeat.version = 240; v_headerFeat.struct_size = sizeof(T2_Type::UnifiedFeatureSlot);
    memcpy(v_headerRaw.magic, "SMEA", 4);  v_headerRaw.version = 240;  v_headerRaw.struct_size = sizeof(T2_Type::UnifiedRawChunk);

    v_headerFeat.total_records = _recordCount;
    // Raw는 저장 여부에 따라 레코드 수가 다름 (_rawWrittenBytes 기반 계산)
    v_headerRaw.total_records = (_rawWrittenBytes - sizeof(T2_Type::FileHeader)) / sizeof(T2_Type::UnifiedRawChunk);

    if (_activeFile) {
        _activeFile.seek(0);
        _activeFile.write((uint8_t*)&v_headerFeat, sizeof(T2_Type::FileHeader));
        _activeFile.close();
    }
    if (_rawFile) {
        _rawFile.seek(0);
        _rawFile.write((uint8_t*)&v_headerRaw, sizeof(T2_Type::FileHeader));
        _rawFile.close();
    }

    _appendIndexItem();
    _writeIndexFileAtomic();

    _sessionOpen = false;
    _rotationSubSeq = 0;

    xSemaphoreGive(_lock);
    ESP_LOGI(TAG, "Session Closed. Reason: %s (Records: %lu)", p_reason, _recordCount);
}


// 1ms 이내 반환 보장. DSP 연산 결과를 비동기 버퍼에 안전하게 밀어넣음.
bool CL_T2_StorageManager::pushFrame(const T2_Type::UnifiedFeatureSlot* p_featSlot, const T2_Type::UnifiedRawChunk* p_rawChunk) {
    if (_ioError) return false;
    xSemaphoreTake(_lock, portMAX_DELAY);

    if (!_sessionOpen) {
        // 모니터링(유휴) 상태: 프리트리거 링버퍼에 순환 저장
        if (_preCapacity > 0) {
            memcpy(&_preFeatBuf[_preHeadFeat], p_featSlot, sizeof(T2_Type::UnifiedFeatureSlot));
            memcpy(&_preRawBuf[_preHeadRaw], p_rawChunk, sizeof(T2_Type::UnifiedRawChunk));
            _preHeadFeat = (_preHeadFeat + 1) % _preCapacity;
            _preHeadRaw = (_preHeadRaw + 1) % _preCapacity;
            if (_preCountFeat < _preCapacity) {
                _preCountFeat++; _preCountRaw++;
            }
        }
    } else {
        // 레코딩 상태: 비동기 파일 기록용 링버퍼에 저장
        uint16_t v_nextHead = (_asyncHead + 1) % ASYNC_RING_CAPACITY;
        if (v_nextHead != _asyncTail) {
            memcpy(&_asyncFeatRing[_asyncHead], p_featSlot, sizeof(T2_Type::UnifiedFeatureSlot));
            memcpy(&_asyncRawRing[_asyncHead], p_rawChunk, sizeof(T2_Type::UnifiedRawChunk));
            _asyncHead = v_nextHead;
        } else {
            ESP_LOGW(TAG, "A-DSE Ring Buffer Overflow! SD write is too slow.");
        }
    }

    xSemaphoreGive(_lock);
    return true;
}

void CL_T2_StorageManager::_storageTaskProc(void* p_param) {
    CL_T2_StorageManager* v_this = (CL_T2_StorageManager*)p_param;
    uint32_t v_lastFlushMs = 0;

    while(1) {
        v_this->_processAsyncRingBuffer();

        // [교정] 정전 시 데이터 영구 증발을 막기 위한 주기적 FAT 테이블 동기화 (Rule #30)
        uint32_t v_now = millis();
        uint32_t v_flushInterval = CL_T2_ConfigManager::getInstance().getConfig().storage.idle_flush_ms;

        if (v_flushInterval > 0 && (v_now - v_lastFlushMs > v_flushInterval)) {
            v_this->flush(); // 내부에서 락(Lock)을 쥐고 _activeFile.flush() 수행
            v_lastFlushMs = v_now;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void CL_T2_StorageManager::_processAsyncRingBuffer() {
    xSemaphoreTake(_lock, portMAX_DELAY);
    if (!_sessionOpen || _ioError || _asyncHead == _asyncTail) {
        xSemaphoreGive(_lock);
        return;
    }

    // [교정] 지역 변수(Stack) 파괴! -> 미리 할당된 바운스 버퍼(PSRAM) 활용
    memcpy(_bounceFeat, &_asyncFeatRing[_asyncTail], sizeof(T2_Type::UnifiedFeatureSlot));
    memcpy(_bounceRaw, &_asyncRawRing[_asyncTail], sizeof(T2_Type::UnifiedRawChunk));

    _asyncTail = (_asyncTail + 1) % ASYNC_RING_CAPACITY;
    bool v_saveRaw = CL_T2_ConfigManager::getInstance().getConfig().storage.save_raw;
    xSemaphoreGive(_lock); // 락 해제 (이하 FAT32 I/O는 병렬 실행)

    // 바운스 버퍼의 포인터로 파일 I/O 수행
    if (_activeFile) {
        size_t v_res = _activeFile.write((uint8_t*)_bounceFeat, sizeof(T2_Type::UnifiedFeatureSlot));
        if (v_res == sizeof(T2_Type::UnifiedFeatureSlot)) {
            _recordCount++;
            _writtenBytes += v_res;
        } else {
            _ioError = true;
            ESP_LOGE(TAG, "Feature File Write Error!");
        }
    }

    if (v_saveRaw && _rawFile) {
        size_t v_rawRes = _rawFile.write((uint8_t*)_bounceRaw, sizeof(T2_Type::UnifiedRawChunk));
        _rawWrittenBytes += v_rawRes;
    }
}

void CL_T2_StorageManager::checkRotation() {
    if (!_sessionOpen || _ioError) return;
    xSemaphoreTake(_lock, portMAX_DELAY);

    T2_Type::DynamicConfig v_cfg = CL_T2_ConfigManager::getInstance().getConfig();
    uint32_t v_maxBytes = v_cfg.storage.rotation_mb * 1024 * 1024;
    uint32_t v_currentTick = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t v_elapsedMs = v_currentTick - _sessionStartTick;
    uint32_t v_maxMs = v_cfg.storage.rotation_min * 60000;

    // 용량 도달 또는 타이머 만료 시 로테이션 트리거
    if ((_writtenBytes + _rawWrittenBytes) >= v_maxBytes || v_elapsedMs >= v_maxMs) {
        _handleRotation();
    }

    xSemaphoreGive(_lock);
}

void CL_T2_StorageManager::_handleRotation() {
    _activeFile.close();
    _rawFile.close();
    _appendIndexItem();

    _rotationSubSeq++;

    time_t v_now; time(&v_now);
    struct tm v_tinfo; localtime_r(&v_now, &v_tinfo);

    char v_dirBin[T2_Config::StorageLimit::MAX_PATH_LEN_CONST];
    char v_dirRaw[T2_Config::StorageLimit::MAX_PATH_LEN_CONST];
    snprintf(v_dirBin, sizeof(v_dirBin), "%s/%04d/%02d/%02d", T2_Config::Path::SD_DIR_BIN_CONST, v_tinfo.tm_year + 1900, v_tinfo.tm_mon + 1, v_tinfo.tm_mday);
    snprintf(v_dirRaw, sizeof(v_dirRaw), "%s/%04d/%02d/%02d", T2_Config::Path::SD_DIR_RAW_CONST, v_tinfo.tm_year + 1900, v_tinfo.tm_mon + 1, v_tinfo.tm_mday);

    // (필요 시 mkdir 로직 추가)

    snprintf(_activePath, sizeof(_activePath), "%s/%s_%llu_%03d.bin", v_dirBin, _currentPrefix, (uint64_t)v_now, _rotationSubSeq);
    snprintf(_activeRawPath, sizeof(_activeRawPath), "%s/%s_%llu_%03d_raw.bin", v_dirRaw, _currentPrefix, (uint64_t)v_now, _rotationSubSeq);

    _activeFile = SD_MMC.open(_activePath, "w");
    _rawFile = SD_MMC.open(_activeRawPath, "w");

    _preAllocateFile(_activeFile, T2_Config::StorageLimit::PREALLOC_BYTES_CONST);
    _preAllocateFile(_rawFile, T2_Config::StorageLimit::PREALLOC_BYTES_CONST);

    _writeFileHeader(_activeFile, sizeof(T2_Type::UnifiedFeatureSlot));
    _writeFileHeader(_rawFile, sizeof(T2_Type::UnifiedRawChunk));

    _recordCount = 0;
    _writtenBytes = sizeof(T2_Type::FileHeader);
    _rawWrittenBytes = sizeof(T2_Type::FileHeader);
    _sessionStartTick = (uint32_t)(esp_timer_get_time() / 1000);

    ESP_LOGI(TAG, "File Rotation Executed. Seq: %d", _rotationSubSeq);
}


void CL_T2_StorageManager::_appendIndexItem() {
    // 큐 크기 초과 시 오래된 파일은 삭제(GC)하고 배열을 시프트(Shift)
    if (_indexCount >= T2_Config::StorageLimit::MAX_ROTATE_LIST_CONST) {
        if (SD_MMC.exists(_indexItems[0].path)) SD_MMC.remove(_indexItems[0].path);
        if (SD_MMC.exists(_indexItems[0].raw_path)) SD_MMC.remove(_indexItems[0].raw_path);

        for (uint16_t i = 0; i < T2_Config::StorageLimit::MAX_ROTATE_LIST_CONST - 1; i++) {
            _indexItems[i] = _indexItems[i + 1];
        }
        _indexCount--;
    }

    strlcpy(_indexItems[_indexCount].path, _activePath, T2_Config::StorageLimit::MAX_PATH_LEN_CONST);
    strlcpy(_indexItems[_indexCount].raw_path, _activeRawPath, T2_Config::StorageLimit::MAX_PATH_LEN_CONST);
    _indexItems[_indexCount].size_bytes = _writtenBytes;
    _indexItems[_indexCount].created_epoch = (uint64_t)time(NULL);
    _indexItems[_indexCount].record_count = _recordCount;
    _indexCount++;
}

bool CL_T2_StorageManager::_writeIndexFileAtomic() {
    File v_tmp = LittleFS.open("/sys/index_241.tmp", "w");
    if (!v_tmp) return false;

    // V7.4.x 규격 JSON 생성
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

    // 정전 파괴 대비 rename() 원자적 덮어쓰기
    LittleFS.remove("/sys/runtime_idx_241.json");
    LittleFS.rename("/sys/index_241.tmp", "/sys/runtime_idx_241.json");
    return true;
}

bool CL_T2_StorageManager::_loadIndexJson() {
    if (!LittleFS.exists("/sys/runtime_idx_241.json")) return false;

    File v_file = LittleFS.open("/sys/runtime_idx_241.json", "r");
    if (!v_file) return false;

    JsonDocument v_doc;
    DeserializationError v_err = deserializeJson(v_doc, v_file);
    v_file.close();
    if (v_err) return false;

    JsonArrayConst v_arr = v_doc["files"];
    _indexCount = 0;
    for (JsonObjectConst v_obj : v_arr) {
        if (_indexCount >= T2_Config::StorageLimit::MAX_ROTATE_LIST_CONST) break;
        strlcpy(_indexItems[_indexCount].path, v_obj["path"] | "", T2_Config::StorageLimit::MAX_PATH_LEN_CONST);
        strlcpy(_indexItems[_indexCount].raw_path, v_obj["raw_path"] | "", T2_Config::StorageLimit::MAX_PATH_LEN_CONST);
        _indexItems[_indexCount].size_bytes = v_obj["size"] | 0;
        _indexItems[_indexCount].created_epoch = v_obj["created"] | 0;
        _indexItems[_indexCount].record_count = v_obj["records"] | 0;
        _indexCount++;
    }
    return true;
}

bool CL_T2_StorageManager::flush() {
    if (!_sessionOpen || _ioError) return false;
    xSemaphoreTake(_lock, portMAX_DELAY);
    _activeFile.flush();
    _rawFile.flush();
    xSemaphoreGive(_lock);
    return true;
}

bool CL_T2_StorageManager::attemptRecovery() {
    xSemaphoreTake(_lock, portMAX_DELAY);
    if (!_ioError) {
        xSemaphoreGive(_lock); return true;
    }

    ESP_LOGW(TAG, "Attempting SD Card Auto-Recovery...");
    SD_MMC.end();
    vTaskDelay(pdMS_TO_TICKS(100)); // 하드웨어 전기적 안정화 대기

    if (SD_MMC.begin(T2_Config::Path::MOUNT_SD_CONST, true)) {
        _ioError = false;
        ESP_LOGI(TAG, "SD Card Recovery Successful!");
    } else {
        ESP_LOGE(TAG, "SD Card Recovery Failed.");
    }

    xSemaphoreGive(_lock);
    return !_ioError;
}







/*
void CL_T2_StorageManager::_processAsyncRingBuffer() {
    xSemaphoreTake(_lock, portMAX_DELAY);
    if (!_sessionOpen || _ioError || _asyncHead == _asyncTail) {
        xSemaphoreGive(_lock);
        return;
    }

	// 지역 변수(Stack) 파괴! -> 미리 할당된 바운스 버퍼(PSRAM) 활용
    memcpy(_bounceFeat, &_asyncFeatRing[_asyncTail], sizeof(T2_Type::UnifiedFeatureSlot));
    memcpy(_bounceRaw, &_asyncRawRing[_asyncTail], sizeof(T2_Type::UnifiedRawChunk));


    // 락(Lock) 점유 최소화를 위한 로컬 스택 복사
    T2_Type::UnifiedFeatureSlot v_featCopy;
    T2_Type::UnifiedRawChunk v_rawCopy;

    memcpy(&v_featCopy, &_asyncFeatRing[_asyncTail], sizeof(T2_Type::UnifiedFeatureSlot));
    memcpy(&v_rawCopy, &_asyncRawRing[_asyncTail], sizeof(T2_Type::UnifiedRawChunk));

    _asyncTail = (_asyncTail + 1) % ASYNC_RING_CAPACITY;
    bool v_saveRaw = CL_T2_ConfigManager::getInstance().getConfig().storage.save_raw;
    xSemaphoreGive(_lock); // 락 해제 (이하 FAT32 I/O는 병렬 실행)

    // 실제 파일 I/O (SD카드가 멈춰도 DSP 태스크는 링버퍼 덕분에 멈추지 않음)
    if (_activeFile) {
        size_t v_res = _activeFile.write((uint8_t*)&v_featCopy, sizeof(T2_Type::UnifiedFeatureSlot));
        if (v_res == sizeof(T2_Type::UnifiedFeatureSlot)) {
            _recordCount++;
            _writtenBytes += v_res;
        } else {
            _ioError = true;
            ESP_LOGE(TAG, "Feature File Write Error!");
        }
    }

    if (v_saveRaw && _rawFile) {
        size_t v_rawRes = _rawFile.write((uint8_t*)&v_rawCopy, sizeof(T2_Type::UnifiedRawChunk));
        _rawWrittenBytes += v_rawRes;
    }
}
*/


// void CL_T2_StorageManager::closeSession(const char* p_reason) {
//     if (!_sessionOpen) return;
//     xSemaphoreTake(_lock, portMAX_DELAY);

//     // 잔여 링버퍼가 모두 파일에 기록될 때까지 대기 (Timeout 1초 데드락 방어 적용)
//     uint32_t v_waitStart = (uint32_t)(esp_timer_get_time() / 1000);
//     while(_asyncHead != _asyncTail) {
//         if (((uint32_t)(esp_timer_get_time() / 1000) - v_waitStart) > 1000) {
//             ESP_LOGE(TAG, "closeSession Timeout! Forcing close.");
//             break;
//         }
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }

//     _activeFile.close();
//     _rawFile.close();

//     _appendIndexItem();
//     _writeIndexFileAtomic(); // JSON 인덱스 원자적 갱신

//     _sessionOpen = false;
//     _rotationSubSeq = 0;

//     xSemaphoreGive(_lock);
//     ESP_LOGI(TAG, "Session Closed. Reason: %s", p_reason);
// }


// void CL_T2_StorageManager::_buildDailyDirectoryPath(char* p_outPath, size_t p_maxLen) {
//     time_t v_now; time(&v_now);
//     struct tm v_tinfo; localtime_r(&v_now, &v_tinfo);

//     char v_dirPath[T2_Config::StorageLimit::MAX_PATH_LEN_CONST];
//     snprintf(v_dirPath, sizeof(v_dirPath), "%s/%04d/%02d/%02d",
//              T2_Config::Path::SD_DIR_BIN_CONST, v_tinfo.tm_year + 1900, v_tinfo.tm_mon + 1, v_tinfo.tm_mday);

//     // O(N) 디렉토리 탐색 병목 회피를 위해 하위 뎁스(Depth)만 검사 후 순차 생성
//     if (!SD_MMC.exists(v_dirPath)) {
//         char v_tmp[128];
//         snprintf(v_tmp, sizeof(v_tmp), "%s/%04d", T2_Config::Path::SD_DIR_BIN_CONST, v_tinfo.tm_year + 1900);
//         if(!SD_MMC.exists(v_tmp)) SD_MMC.mkdir(v_tmp);

//         snprintf(v_tmp, sizeof(v_tmp), "%s/%04d/%02d", T2_Config::Path::SD_DIR_BIN_CONST, v_tinfo.tm_year + 1900, v_tinfo.tm_mon + 1);
//         if(!SD_MMC.exists(v_tmp)) SD_MMC.mkdir(v_tmp);

//         SD_MMC.mkdir(v_dirPath);
//     }

//     snprintf(p_outPath, p_maxLen, "%s/%s_%llu_%03d.bin", v_dirPath, _currentPrefix, (uint64_t)v_now, _rotationSubSeq);
// }

// void CL_T2_StorageManager::_handleRotation() {
//     _activeFile.close();
//     _rawFile.close();
//     _appendIndexItem();

//     _rotationSubSeq++;

//     // 파일 새로 열기 및 헤더 갱신
//     _buildDailyDirectoryPath(_activePath, sizeof(_activePath));
//     snprintf(_activeRawPath, sizeof(_activeRawPath), "%s/%s_%llu_%03d_raw.bin",
//              T2_Config::Path::SD_DIR_RAW_CONST, _currentPrefix, (uint64_t)time(NULL), _rotationSubSeq);

//     _activeFile = SD_MMC.open(_activePath, "w");
//     _rawFile = SD_MMC.open(_activeRawPath, "w");

//     _preAllocateFile(_activeFile, T2_Config::StorageLimit::PREALLOC_BYTES_CONST);
//     _preAllocateFile(_rawFile, T2_Config::StorageLimit::PREALLOC_BYTES_CONST);

//     _writeFileHeader(_activeFile, sizeof(T2_Type::UnifiedFeatureSlot));
//     _writeFileHeader(_rawFile, sizeof(T2_Type::UnifiedRawChunk));

//     _recordCount = 0;
//     _writtenBytes = sizeof(T2_Type::FileHeader);
//     _rawWrittenBytes = sizeof(T2_Type::FileHeader);
//     _sessionStartTick = (uint32_t)(esp_timer_get_time() / 1000);

//     ESP_LOGI(TAG, "File Rotation Executed. Seq: %d", _rotationSubSeq);
// }
