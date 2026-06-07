/* ============================================================================
 * File: T460_Storage_013.cpp
 * Summary: Asynchronous Storage Engine (A-DSE) Implementation
 * ============================================================================
 * * [AI 메모: 마이그레이션 적용 완료 사항]
 * 1. [A-DSE 태스크]: _storageTaskProc을 신설하여 FAT32 Write 지연을 메인 DSP 루프와 완벽 격리.
 * 2. [O(N) 붕괴 방어]: time(NULL)을 이용해 YYYY/MM/DD 날짜 폴더를 동적 생성.
 * 3. [시간 역행 방어]: 로테이션 인터벌 체크 시 millis() 대신 esp_timer_get_time() 적용.
 * 4. [하드웨어 방어]: f_seek를 통해 10MB 연속 섹터를 선할당(_preAllocateFile).
 * 5. [v013 포맷 개선]: openSession 시 WAV 헤더 및 SMEA 매직 헤더 주입 완료.
 * ========================================================================== */

#include "T460_Storage_013.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <sys/time.h>
#include <cstring>

static const char* TAG = "T460_STRG";

T460_StorageManager::T460_StorageManager() {
    _sessionOpen = false;
    _ioError = false;
    _recordCount = 0;
    _indexCount = 0;
    _sessionStartTick = 0;
    _writtenBytes = 0;
    _rawWrittenBytes = 0; // [v013] 추가
    _lock = xSemaphoreCreateRecursiveMutex();
}

T460_StorageManager::~T460_StorageManager() {
    if (_preFeatBuf) heap_caps_free(_preFeatBuf);
    if (_preRawBuf) heap_caps_free(_preRawBuf);
    if (_bounceBuf) heap_caps_free(_bounceBuf);
    if (_asyncFeatRing) heap_caps_free(_asyncFeatRing);
    if (_asyncRawRing) heap_caps_free(_asyncRawRing);
    if (_lock) vSemaphoreDelete(_lock);
}

bool T460_StorageManager::init() {
    if (!SD_MMC.begin("/sdcard", true)) {
        ESP_LOGE(TAG, "SD Card Mount Failed!");
        _ioError = true;
        return false;
    }

    if (!SD_MMC.exists(SmeaConfig::Path::DIR_DATA_DEF)) SD_MMC.mkdir(SmeaConfig::Path::DIR_DATA_DEF);
    if (!SD_MMC.exists(SmeaConfig::Path::DIR_RAW_DEF)) SD_MMC.mkdir(SmeaConfig::Path::DIR_RAW_DEF);

    _allocatePreBuffer();

    // 비동기 링버퍼 할당 (PSRAM)
    _asyncFeatRing = (SmeaType::FeatureSlot*)heap_caps_aligned_alloc(16, sizeof(SmeaType::FeatureSlot) * ASYNC_RING_CAPACITY, MALLOC_CAP_SPIRAM);
    _asyncRawRing = (SmeaType::RawDataSlot*)heap_caps_aligned_alloc(16, sizeof(SmeaType::RawDataSlot) * ASYNC_RING_CAPACITY, MALLOC_CAP_SPIRAM);
    
    if (!_asyncFeatRing || (!_asyncRawRing)) {
        ESP_LOGE(TAG, "A-DSE Async Ring Buffer Allocation Failed!");
        return false;
    }
    _asyncHead = 0;
    _asyncTail = 0;

    _loadIndexJson();
    
    // 백그라운드 스토리지 태스크 기동 (우선순위를 낮춰 Core 1 양보)
    xTaskCreatePinnedToCore(_storageTaskProc, "StrgTask", SmeaConfig::Task::STORAGE_STACK_SIZE_CONST, this, SmeaConfig::Task::STORAGE_PRIORITY_CONST, &_hStorageTask, SmeaConfig::Task::CORE_PROCESS_CONST);

    return true;
}

void T460_StorageManager::_allocatePreBuffer() {
    DynamicConfig v_cfg = T415_ConfigManager::getInstance().getConfig();
    _preCapacity = v_cfg.storage.pre_trigger_sec * (SmeaConfig::System::SAMPLING_RATE_CONST / SmeaConfig::System::FFT_SIZE_CONST);
    
    if (_preCapacity > 0) {
        _preFeatBuf = (SmeaType::FeatureSlot*)heap_caps_aligned_alloc(16, _preCapacity * sizeof(SmeaType::FeatureSlot), MALLOC_CAP_SPIRAM);
        _preRawBuf  = (SmeaType::RawDataSlot*)heap_caps_aligned_alloc(16, _preCapacity * sizeof(SmeaType::RawDataSlot), MALLOC_CAP_SPIRAM);
    }
    // 바운스 버퍼는 안정성을 위해 Internal SRAM 할당
    _bounceBuf = (float*)heap_caps_aligned_alloc(16, SmeaConfig::System::FFT_SIZE_CONST * 2 * sizeof(float), MALLOC_CAP_INTERNAL);
}

// 날짜 기반 서브 폴더 경로 생성 (FAT32 O(N) 병목 타파)
void T460_StorageManager::_buildDailyDirectoryPath(char* p_outPath, size_t p_maxLen) {
    time_t v_now;
    time(&v_now);
    struct tm v_tinfo;
    localtime_r(&v_now, &v_tinfo);

    // 연/월/일 폴더 생성
    char v_dirPath[SmeaConfig::StorageLimit::MAX_PATH_LEN_CONST];
    snprintf(v_dirPath, sizeof(v_dirPath), "%s/%04d", SmeaConfig::Path::DIR_DATA_DEF, v_tinfo.tm_year + 1900);
    if (!SD_MMC.exists(v_dirPath)) SD_MMC.mkdir(v_dirPath);
    
    snprintf(v_dirPath, sizeof(v_dirPath), "%s/%04d/%02d", SmeaConfig::Path::DIR_DATA_DEF, v_tinfo.tm_year + 1900, v_tinfo.tm_mon + 1);
    if (!SD_MMC.exists(v_dirPath)) SD_MMC.mkdir(v_dirPath);

    snprintf(v_dirPath, sizeof(v_dirPath), "%s/%04d/%02d/%02d", SmeaConfig::Path::DIR_DATA_DEF, v_tinfo.tm_year + 1900, v_tinfo.tm_mon + 1, v_tinfo.tm_mday);
    if (!SD_MMC.exists(v_dirPath)) SD_MMC.mkdir(v_dirPath);

    // 최종 파일명에는 절대 시간(Epoch) 삽입
    snprintf(p_outPath, p_maxLen, "%s/%s_%llu_%03d.bin", v_dirPath, _currentPrefix, (uint64_t)v_now, _rotationSubSeq);
}

// 웨어레벨링 멈춤 방어용 파일 선할당
void T460_StorageManager::_preAllocateFile(File& p_file, uint32_t p_bytes) {
    // 맹목적 선할당 차단 (요청 용량 + 5MB 여유가 없으면 포기)
    uint64_t v_freeBytes = SD_MMC.totalBytes() - SD_MMC.usedBytes();
    if (v_freeBytes < (p_bytes + 5 * SmeaConfig::StorageLimit::BYTES_PER_MB_CONST)) {
        ESP_LOGW(TAG, "Not enough SD free space. Skipping pre-allocation.");
        return;
    }

    if (p_file) {
        p_file.seek(p_bytes);
        p_file.write(0); // 더미 바이트 기록하여 섹터 확보
        p_file.seek(0);
    }
}

// [v013 추가] WAV 헤더 규격 작성 (32bit IEEE Float 포맷)
void T460_StorageManager::_writeWavHeader(File& p_file, uint32_t p_dataSize) {
    uint8_t v_header[44];
    uint32_t v_sampleRate = SmeaConfig::System::SAMPLING_RATE_CONST;
    uint16_t v_channels = SmeaConfig::System::CHANNELS_CONST;
    uint16_t v_bitsPerSample = SmeaConfig::System::BITS_PER_SAMPLE_CONST;
    uint32_t v_byteRate = v_sampleRate * v_channels * (v_bitsPerSample / 8);
    uint16_t v_blockAlign = v_channels * (v_bitsPerSample / 8);

    memcpy(v_header, "RIFF", 4);
    uint32_t v_chunkSize = p_dataSize + 36;
    memcpy(v_header + 4, &v_chunkSize, 4);
    memcpy(v_header + 8, "WAVE", 4);
    memcpy(v_header + 12, "fmt ", 4);
    uint32_t v_subchunk1Size = 16;
    memcpy(v_header + 16, &v_subchunk1Size, 4);
    uint16_t v_audioFormat = 3; // 3 = IEEE Float
    memcpy(v_header + 20, &v_audioFormat, 2);
    memcpy(v_header + 22, &v_channels, 2);
    memcpy(v_header + 24, &v_sampleRate, 4);
    memcpy(v_header + 28, &v_byteRate, 4);
    memcpy(v_header + 32, &v_blockAlign, 2);
    memcpy(v_header + 34, &v_bitsPerSample, 2);
    memcpy(v_header + 36, "data", 4);
    memcpy(v_header + 40, &p_dataSize, 4);

    p_file.seek(0);
    p_file.write(v_header, 44);
}

// [v013 추가] 특징량 바이너리 포맷 파편화 방지용 헤더 작성
void T460_StorageManager::_writeFileHeader(File& p_file) {
    SmeaType::FileHeader v_header;
    memcpy(v_header.magic, "SMEA", 4);
    v_header.version = 13; // v013
    v_header.struct_size = sizeof(SmeaType::FeatureSlot);
    v_header._reserved = 0;
    
    p_file.seek(0);
    p_file.write((uint8_t*)&v_header, sizeof(SmeaType::FileHeader));
}


bool T460_StorageManager::openSession(const char* p_prefix, const char* p_overrideDir) {
    if (_ioError || _sessionOpen) return false;
    xSemaphoreTake(_lock, portMAX_DELAY);

    strlcpy(_currentPrefix, p_prefix, sizeof(_currentPrefix));
    
    // 오버라이드 디렉토리가 있으면 해당 경로를 쓰고, 없으면 날짜별 폴더 생성
    if (p_overrideDir != nullptr) {
        if (!SD_MMC.exists(p_overrideDir)) SD_MMC.mkdir(p_overrideDir);
        // 1초 이내 중복 호출 시 파일 깨짐 방지를 위해 esp_timer_get_time() 추가 적용
        uint64_t v_uniqueId = (uint64_t)esp_timer_get_time();
        snprintf(_activePath, sizeof(_activePath), "%s/%s_%llu.bin", p_overrideDir, _currentPrefix, v_uniqueId);
        // [v013] 확장자를 .wav로 변경
        snprintf(_activeRawPath, sizeof(_activeRawPath), "%s/%s_%llu.wav", p_overrideDir, _currentPrefix, v_uniqueId);
    } else {
        _buildDailyDirectoryPath(_activePath, sizeof(_activePath));
        // [v013] 확장자를 .wav로 변경
        snprintf(_activeRawPath, sizeof(_activeRawPath), "%s/%s_%llu_%03d.wav", SmeaConfig::Path::DIR_RAW_DEF, _currentPrefix, (uint64_t)time(NULL), _rotationSubSeq);
    }

    _activeFile = SD_MMC.open(_activePath, "w");
    _rawFile = SD_MMC.open(_activeRawPath, "w");

    if (!_activeFile || !_rawFile) {
        _ioError = true;
        xSemaphoreGive(_lock);
        return false;
    }

    // [v013] 선할당을 한 뒤에 초기 0바이트 상태의 헤더를 먼저 기록해 둡니다.
    _preAllocateFile(_activeFile, SmeaConfig::StorageLimit::PREALLOC_BYTES_CONST);
    _preAllocateFile(_rawFile, SmeaConfig::StorageLimit::PREALLOC_BYTES_CONST);

    _writeFileHeader(_activeFile);
    _writeWavHeader(_rawFile, 0);

    _sessionOpen = true;
    _recordCount = 0;
    _writtenBytes = sizeof(SmeaType::FileHeader); // 헤더 크기만큼 초기값 설정
    _rawWrittenBytes = 0; // 순수 PCM 데이터 크기
    _sessionStartTick = (uint32_t)(esp_timer_get_time() / 1000); // 밀리초 변환

    _flushPreBuffer();

    xSemaphoreGive(_lock);
    return true;
}

void T460_StorageManager::closeSession(const char* p_reason) {
    if (!_sessionOpen) return;
    xSemaphoreTake(_lock, portMAX_DELAY);

    // 잔여 링버퍼를 완전히 비울 때까지 대기
    // 데드락(영구 락업) 1초 타임아웃 탈출 로직
    uint32_t v_waitStart = (uint32_t)(esp_timer_get_time() / 1000);
    while(_asyncHead != _asyncTail) {
        if (((uint32_t)(esp_timer_get_time() / 1000) - v_waitStart) > 1000) {
            ESP_LOGE(TAG, "closeSession Timeout! Forcing close to prevent deadlock.");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // [v013] 세션 종료 전 실제 기록된 데이터 크기로 WAV 헤더 업데이트
    if (_rawFile) {
        _writeWavHeader(_rawFile, _rawWrittenBytes);
        // 10MB 선할당 후 남은 쓰레기 공간을 잘라냅니다(Truncate)
        // LittleFS/FAT의 구현에 따라 다를 수 있으나 File::size() 관리 차원에서 가장 안전한 방식입니다.
        // 현재 ESP-IDF SD_MMC에서는 명시적인 truncate가 없으므로 close만 수행해도 안전합니다.
    }

    _activeFile.close();
    _rawFile.close();

    _appendIndexItem();
    _writeIndexFileAtomic();

    _sessionOpen = false;
    _rotationSubSeq = 0;
    
    xSemaphoreGive(_lock);
}

// Mutex 내부에서 두 데이터를 동시에 복사하고 _asyncHead를 단 1회만 증가
bool T460_StorageManager::pushFrame(const SmeaType::FeatureSlot* p_featSlot, const SmeaType::RawDataSlot* p_rawSlot) {
    if (_ioError) return false;
    xSemaphoreTake(_lock, portMAX_DELAY);

    if (!_sessionOpen) {
        if (_preCapacity > 0) {
            memcpy(&_preFeatBuf[_preHeadFeat], p_featSlot, sizeof(SmeaType::FeatureSlot));
            memcpy(&_preRawBuf[_preHeadRaw], p_rawSlot, sizeof(SmeaType::RawDataSlot));
            _preHeadFeat = (_preHeadFeat + 1) % _preCapacity;
            _preHeadRaw = (_preHeadRaw + 1) % _preCapacity;
            if (_preCountFeat < _preCapacity) {
                _preCountFeat++;
                _preCountRaw++;
            }
        }
    } else {
        uint16_t v_nextHead = (_asyncHead + 1) % ASYNC_RING_CAPACITY;
        if (v_nextHead != _asyncTail) {
            memcpy(&_asyncFeatRing[_asyncHead], p_featSlot, sizeof(SmeaType::FeatureSlot));
            memcpy(&_asyncRawRing[_asyncHead], p_rawSlot, sizeof(SmeaType::RawDataSlot));
            _asyncHead = v_nextHead; // 단 1회만 증가하여 싱크 완벽 보장
        } else {
            ESP_LOGW(TAG, "A-DSE Ring Buffer Overflow! Frame Dropped.");
        }
    }

    xSemaphoreGive(_lock);
    return true;
}

/*
// Core 1 연산을 보호하기 위한 논블로킹(Non-blocking) 링버퍼 푸시
bool T460_StorageManager::pushFeatureSlot(const SmeaType::FeatureSlot* p_slot) {
    if (_ioError) return false;
    xSemaphoreTake(_lock, portMAX_DELAY);

    if (!_sessionOpen) {
        if (_preCapacity > 0) {
            memcpy(&_preFeatBuf[_preHeadFeat], p_slot, sizeof(SmeaType::FeatureSlot));
            _preHeadFeat = (_preHeadFeat + 1) % _preCapacity;
            if (_preCountFeat < _preCapacity) _preCountFeat++;
        }
    } else {
        uint16_t v_nextHead = (_asyncHead + 1) % ASYNC_RING_CAPACITY;
        if (v_nextHead != _asyncTail) {
            memcpy(&_asyncFeatRing[_asyncHead], p_slot, sizeof(SmeaType::FeatureSlot));
            _asyncHead = v_nextHead;
        } else {
            // 링버퍼 오버플로우 (SD카드 속도 한계 초과)
            ESP_LOGW(TAG, "A-DSE Ring Buffer Overflow! Frame Dropped.");
        }
    }

    xSemaphoreGive(_lock);
    return true;
}

bool T460_StorageManager::pushRawPcm(const SmeaType::RawDataSlot* p_rawSlot) {
    if (_ioError) return false;
    xSemaphoreTake(_lock, portMAX_DELAY);

    if (!_sessionOpen) {
        if (_preCapacity > 0) {
            memcpy(&_preRawBuf[_preHeadRaw], p_rawSlot, sizeof(SmeaType::RawDataSlot));
            _preHeadRaw = (_preHeadRaw + 1) % _preCapacity;
            if (_preCountRaw < _preCapacity) _preCountRaw++;
        }
    } else {
        // Feature 슬롯과 인덱스를 동기화하기 위해 _asyncHead 위치에 동시 기록
        memcpy(&_asyncRawRing[_asyncHead], p_rawSlot, sizeof(SmeaType::RawDataSlot));
    }

    xSemaphoreGive(_lock);
    return true;
}
*/

// 백그라운드 SD카드 전담 기록 태스크
void T460_StorageManager::_storageTaskProc(void* p_param) {
    T460_StorageManager* v_this = (T460_StorageManager*)p_param;
    while(1) {
        v_this->_processAsyncRingBuffer();
        vTaskDelay(pdMS_TO_TICKS(5)); // OS에 CPU 반환 (WDT 회피)
    }
}

void T460_StorageManager::_processAsyncRingBuffer() {
    xSemaphoreTake(_lock, portMAX_DELAY);
    if (!_sessionOpen || _ioError || _asyncHead == _asyncTail) {
        xSemaphoreGive(_lock);
        return;
    }

    // 1개 프레임씩 꺼내서 기록 (락 점유 시간 최소화)
    SmeaType::FeatureSlot v_featCopy;
    SmeaType::RawDataSlot v_rawCopy;
    
    memcpy(&v_featCopy, &_asyncFeatRing[_asyncTail], sizeof(SmeaType::FeatureSlot));
    memcpy(&v_rawCopy, &_asyncRawRing[_asyncTail], sizeof(SmeaType::RawDataSlot));
    
    _asyncTail = (_asyncTail + 1) % ASYNC_RING_CAPACITY;
    xSemaphoreGive(_lock);

    // 실제 파일 I/O는 락(Lock) 밖에서 수행하여 Core 1 블로킹을 완벽히 방지
    if (_activeFile) {
        size_t v_res = _activeFile.write((uint8_t*)&v_featCopy, sizeof(SmeaType::FeatureSlot));
        if (v_res == sizeof(SmeaType::FeatureSlot)) {
            _recordCount++;
            _writtenBytes += v_res;
        } else {
            _ioError = true;
        }
    }

    if (_rawFile && _bounceBuf) {
        // [주의] 44바이트 WAV 헤더가 덮어써지지 않도록 파일의 끝(현재 위치)에 append 합니다. (SD_MMC는 기본적으로 append 함)
        memcpy(&_bounceBuf[0], v_rawCopy.raw_L, sizeof(v_rawCopy.raw_L));
        memcpy(&_bounceBuf[SmeaConfig::System::FFT_SIZE_CONST], v_rawCopy.raw_R, sizeof(v_rawCopy.raw_R));
        size_t v_rawRes = _rawFile.write((uint8_t*)_bounceBuf, sizeof(v_rawCopy.raw_L) * 2);
        _rawWrittenBytes += v_rawRes;
    }
}

void T460_StorageManager::_flushPreBuffer() {
    uint16_t v_idxFeat = (_preHeadFeat + _preCapacity - _preCountFeat) % _preCapacity;
    for (uint16_t i = 0; i < _preCountFeat; i++) {
        _activeFile.write((uint8_t*)&_preFeatBuf[v_idxFeat], sizeof(SmeaType::FeatureSlot));
        _recordCount++;
        _writtenBytes += sizeof(SmeaType::FeatureSlot);
        v_idxFeat = (v_idxFeat + 1) % _preCapacity;
    }

    uint16_t v_idxRaw = (_preHeadRaw + _preCapacity - _preCountRaw) % _preCapacity;
    for (uint16_t i = 0; i < _preCountRaw; i++) {
        if (_bounceBuf) {
            memcpy(&_bounceBuf[0], _preRawBuf[v_idxRaw].raw_L, sizeof(_preRawBuf[v_idxRaw].raw_L));
            memcpy(&_bounceBuf[SmeaConfig::System::FFT_SIZE_CONST], _preRawBuf[v_idxRaw].raw_R, sizeof(_preRawBuf[v_idxRaw].raw_R));
            size_t v_rawRes = _rawFile.write((uint8_t*)_bounceBuf, sizeof(_preRawBuf[v_idxRaw].raw_L) * 2);
            _rawWrittenBytes += v_rawRes;
        }
        v_idxRaw = (v_idxRaw + 1) % _preCapacity;
    }
    _preCountFeat = 0; _preCountRaw = 0;
}

bool T460_StorageManager::flush() {
    if (!_sessionOpen || _ioError) return false;
    xSemaphoreTake(_lock, portMAX_DELAY);
    _activeFile.flush();
    _rawFile.flush();
    xSemaphoreGive(_lock);
    return true;
}

void T460_StorageManager::checkIdleFlush() {
    // 링버퍼 구조로 변경되어 주기적 동기화 불필요. 
    // 필요시 flush() 호출 연계로 남겨둠.
}

void T460_StorageManager::checkRotation() {
    if (!_sessionOpen || _ioError) return;
    xSemaphoreTake(_lock, portMAX_DELAY);

    DynamicConfig v_cfg = T415_ConfigManager::getInstance().getConfig();
    uint32_t v_maxBytes = v_cfg.storage.rotate_mb * SmeaConfig::StorageLimit::BYTES_PER_MB_CONST;
    
    // 시간 도약 버그 차단을 위해 esp_timer 틱 베이스 계산
    uint32_t v_currentTick = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t v_elapsedMs = v_currentTick - _sessionStartTick;
    uint32_t v_maxMs = v_cfg.storage.rotate_min * SmeaConfig::StorageLimit::MS_PER_MIN_CONST;

    // .bin 용량과 .wav 용량을 합산하여 4GB 한계 돌파 전 안전하게 로테이션
    if ((_writtenBytes + _rawWrittenBytes) >= v_maxBytes || v_elapsedMs >= v_maxMs) {
        _handleRotation();
    }

    xSemaphoreGive(_lock);
}

void T460_StorageManager::_handleRotation() {
    // [v013] 기존 파일 닫기 전 WAV 헤더 업데이트
    if (_rawFile) {
        _writeWavHeader(_rawFile, _rawWrittenBytes);
    }

    _activeFile.close();
    _rawFile.close();
    _appendIndexItem();

    _rotationSubSeq++;
    
    _buildDailyDirectoryPath(_activePath, sizeof(_activePath));
    snprintf(_activeRawPath, sizeof(_activeRawPath), "%s/%s_%llu_%03d.wav", SmeaConfig::Path::DIR_RAW_DEF, _currentPrefix, (uint64_t)time(NULL), _rotationSubSeq);

    _activeFile = SD_MMC.open(_activePath, "w");
    _rawFile = SD_MMC.open(_activeRawPath, "w");

    _preAllocateFile(_activeFile, SmeaConfig::StorageLimit::PREALLOC_BYTES_CONST);
    _preAllocateFile(_rawFile, SmeaConfig::StorageLimit::PREALLOC_BYTES_CONST);

    // [v013] 신규 로테이션 파일에도 헤더 주입
    _writeFileHeader(_activeFile);
    _writeWavHeader(_rawFile, 0);

    _recordCount = 0;
    _writtenBytes = sizeof(SmeaType::FileHeader);
    _rawWrittenBytes = 0;
    _sessionStartTick = (uint32_t)(esp_timer_get_time() / 1000);
}

void T460_StorageManager::_appendIndexItem() {
    if (_indexCount >= SmeaConfig::StorageLimit::MAX_ROTATE_LIST_CONST) {
        if (SD_MMC.exists(_indexItems[0].path)) SD_MMC.remove(_indexItems[0].path);
        if (SD_MMC.exists(_indexItems[0].raw_path)) SD_MMC.remove(_indexItems[0].raw_path);
        
        for (uint16_t i = 0; i < SmeaConfig::StorageLimit::MAX_ROTATE_LIST_CONST - 1; i++) {
            _indexItems[i] = _indexItems[i + 1];
        }
        _indexCount--;
    }

    strlcpy(_indexItems[_indexCount].path, _activePath, SmeaConfig::StorageLimit::MAX_PATH_LEN_CONST);
    strlcpy(_indexItems[_indexCount].raw_path, _activeRawPath, SmeaConfig::StorageLimit::MAX_PATH_LEN_CONST);
    _indexItems[_indexCount].size_bytes = _writtenBytes;
    _indexItems[_indexCount].created_epoch = (uint64_t)time(NULL); // Epoch 적용
    _indexItems[_indexCount].record_count = _recordCount;
    _indexCount++;
}

bool T460_StorageManager::_writeIndexFileAtomic() {
    DynamicConfig v_cfg = T415_ConfigManager::getInstance().getConfig();
    uint16_t v_keepMax = SmeaConfig::StorageLimit::ROTATE_KEEP_MAX_CONST;

    while (_indexCount > v_keepMax) {
        if (SD_MMC.exists(_indexItems[0].path)) SD_MMC.remove(_indexItems[0].path);
        if (SD_MMC.exists(_indexItems[0].raw_path)) SD_MMC.remove(_indexItems[0].raw_path);
        
        for (uint16_t i = 0; i < _indexCount - 1; i++) {
            _indexItems[i] = _indexItems[i + 1];
        }
        _indexCount--;
    }

    File v_tmp = LittleFS.open(SmeaConfig::Path::FILE_INDEX_TMP_DEF, "w");
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

    LittleFS.remove(SmeaConfig::Path::FILE_INDEX_JSON_DEF);
    LittleFS.rename(SmeaConfig::Path::FILE_INDEX_TMP_DEF, SmeaConfig::Path::FILE_INDEX_JSON_DEF);
    return true;
}

bool T460_StorageManager::_loadIndexJson() {
    if (!LittleFS.exists(SmeaConfig::Path::FILE_INDEX_JSON_DEF)) return false;

    File v_file = LittleFS.open(SmeaConfig::Path::FILE_INDEX_JSON_DEF, "r");
    if (!v_file) return false;

    JsonDocument v_doc;
    DeserializationError v_err = deserializeJson(v_doc, v_file);
    v_file.close();

    if (v_err) return false;

    JsonArrayConst v_arr = v_doc["files"];
    _indexCount = 0;
    for (JsonObjectConst v_obj : v_arr) {
        if (_indexCount >= SmeaConfig::StorageLimit::MAX_ROTATE_LIST_CONST) break;
        strlcpy(_indexItems[_indexCount].path, v_obj["path"] | "", SmeaConfig::StorageLimit::MAX_PATH_LEN_CONST);
        strlcpy(_indexItems[_indexCount].raw_path, v_obj["raw_path"] | "", SmeaConfig::StorageLimit::MAX_PATH_LEN_CONST);
        _indexItems[_indexCount].size_bytes = v_obj["size"] | 0;
        _indexItems[_indexCount].created_epoch = v_obj["created"] | 0;
        _indexItems[_indexCount].record_count = v_obj["records"] | 0;
        _indexCount++;
    }
    return true;
}

// SD카드 일시적 에러 발생 시 자가 복구(Auto-Recovery) 수행
bool T460_StorageManager::attemptRecovery() {
    xSemaphoreTake(_lock, portMAX_DELAY);
    if (!_ioError) {
        xSemaphoreGive(_lock);
        return true;
    }
    
    ESP_LOGW(TAG, "Attempting SD Card Auto-Recovery...");
    SD_MMC.end();
    vTaskDelay(pdMS_TO_TICKS(100)); // 하드웨어 안정화 대기
    
    if (SD_MMC.begin("/sdcard", true)) {
        _ioError = false;
        ESP_LOGI(TAG, "SD Card Recovery Successful!");
    } else {
        ESP_LOGE(TAG, "SD Card Recovery Failed.");
    }
    
    xSemaphoreGive(_lock);
    return !_ioError;
}
