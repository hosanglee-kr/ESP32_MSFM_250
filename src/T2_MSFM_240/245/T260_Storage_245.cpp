/* ============================================================================
 * File: T260_Storage_245.cpp
 * Summary: 멀티모달 비동기 스토리지 엔진 구현부
 * ============================================================================ */

#include "T260_Storage_245.hpp"
#include "T220_CfgMgr_245.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <sys/time.h>
#include <cstring>
#include <ArduinoJson.h>

static const char* TAG = "T245_STRG";

/**
 * @brief CL_T2_StorageManager 생성자
 */
CL_T2_StorageManager::CL_T2_StorageManager()
    : _sessionOpen(false), _ioError(false), _recordCount(0), _indexCount(0), _rotationSubSeq(0),
      _preFeatBuf(nullptr), _preAccBuf(nullptr), _preGyrBuf(nullptr), _preAudBuf(nullptr),
      _asyncFeatRing(nullptr), _asyncAccRing(nullptr), _asyncGyrRing(nullptr), _asyncAudRing(nullptr),
      _bounceFeat(nullptr), _bounceAcc(nullptr), _bounceGyr(nullptr), _bounceAud(nullptr),
      _hStorageTask(nullptr) {
    _lock = xSemaphoreCreateRecursiveMutex();
}

/**
 * @brief CL_T2_StorageManager 소멸자 (동적 메모리 자원 해제)
 */
CL_T2_StorageManager::~CL_T2_StorageManager() {
    if (_hStorageTask) vTaskDelete(_hStorageTask);
    if (_preFeatBuf) heap_caps_free(_preFeatBuf);
    if (_preAccBuf) heap_caps_free(_preAccBuf);
    if (_preGyrBuf) heap_caps_free(_preGyrBuf);
    if (_preAudBuf) heap_caps_free(_preAudBuf);

    if (_asyncFeatRing) heap_caps_free(_asyncFeatRing);
    if (_asyncAccRing) heap_caps_free(_asyncAccRing);
    if (_asyncGyrRing) heap_caps_free(_asyncGyrRing);
    if (_asyncAudRing) heap_caps_free(_asyncAudRing);

    if (_bounceFeat) heap_caps_free(_bounceFeat);
    if (_bounceAcc) heap_caps_free(_bounceAcc);
    if (_bounceGyr) heap_caps_free(_bounceGyr);
    if (_bounceAud) heap_caps_free(_bounceAud);
}

/**
 * @brief 스토리지 힙 버퍼를 구성하고 백그라운드 쓰기 태스크 개시
 * @return 구동 초기화 여부
 */
bool CL_T2_StorageManager::init() {
    _allocateBuffers();

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

/**
 * @brief 새로운 세션 파일을 생성하고 사전에 프리 로드된 데이터를 링 버퍼에 적재
 * @param p_prefix 파일 이름 접두사
 * @param p_overrideDir 폴더 강제 맵 경로
 * @return 세션 시작 결과
 */
bool CL_T2_StorageManager::openSession(const char* p_prefix, const char* p_overrideDir) {
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    if (_sessionOpen) { xSemaphoreGiveRecursive(_lock); return true; }

    strncpy(_currentPrefix, p_prefix, sizeof(_currentPrefix) - 1);

    // [처리 단위 1] 매 캘린더 날짜 디렉터리 경로 생성 및 생성 타깃 주소 설정
    _buildDailyPath(_activePath, sizeof(_activePath), "bin");
    _buildDailyPath(_wavPath, sizeof(_wavPath), "wav");
    _buildDailyPath(_accPath, sizeof(_accPath), "acc");
    _buildDailyPath(_gyrPath, sizeof(_gyrPath), "gyr");

    _activeFile = SD_MMC.open(_activePath, FILE_WRITE);
    if (!_activeFile) {
        ESP_LOGE(TAG, "Failed to open bin file: %s", _activePath);
        _ioError = true;
        xSemaphoreGiveRecursive(_lock);
        return false;
    }

    // 파일 크기 선 할당 및 헤더 작성
    _preAllocateFile(_activeFile, T2_Def::Global::StorageLimit::PREALLOC_BYTES_MAX);
    _writeBinHeader(_activeFile);

    const auto& v_cfg = CL_T2_ConfigManager::getInstance().getConfig();

    // [처리 단위 2] 설정에 따른 원시 데이터 개별 로깅 파일 오픈 및 선 할당
    if (v_cfg.storage.save_raw) {
        if (v_cfg.accel.enable) {
            _accFile = SD_MMC.open(_accPath, FILE_WRITE);
            if (_accFile) {
                _preAllocateFile(_accFile, T2_Def::Global::StorageLimit::PREALLOC_BYTES_MAX);
            }
        }
        if (v_cfg.gyro.enable) {
            _gyrFile = SD_MMC.open(_gyrPath, FILE_WRITE);
            if (_gyrFile) {
                _preAllocateFile(_gyrFile, T2_Def::Global::StorageLimit::PREALLOC_BYTES_MAX);
            }
        }
        if (v_cfg.audio.enable) {
            _wavFile = SD_MMC.open(_wavPath, FILE_WRITE);
            if (_wavFile) {
                _preAllocateFile(_wavFile, T2_Def::Global::StorageLimit::PREALLOC_BYTES_MAX);
                _writeWavHeader(_wavFile, 0);
            }
        }
    }

    // 세션 파라미터 셋업
    _sessionOpen = true;
    _recordCount = 0;
    _writtenBytes = sizeof(T2_Type::ST_FileHeader_t);
    _wavWrittenBytes = 0;
    _accWrittenBytes = 0;
    _gyrWrittenBytes = 0;
    _sessionStartTick = (uint32_t)(esp_timer_get_time() / 1000);

    // [처리 단위 3] 비동기 적재 버퍼로의 프리트리거 데이터 이전 개시
    _flushPreBufferToRing();

    ESP_LOGI(TAG, "Session Opened: %s", _activePath);
    xSemaphoreGiveRecursive(_lock);
    return true;
}

/**
 * @brief 현재 세션 파일 스트림을 닫고 인덱스 이력 목록을 업데이트하여 LittleFS에 저장
 * @param p_reason 종료 사유 정보
 */
void CL_T2_StorageManager::closeSession(const char* p_reason) {
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    if (!_sessionOpen) { xSemaphoreGiveRecursive(_lock); return; }

    flush();

    // [처리 단위 1] 특징 데이터 총 프레임 카운트 헤더 영역 역추적 기입 후 닫기
    if (_activeFile) {
        _activeFile.seek(offsetof(T2_Type::ST_FileHeader_t, total));
        _activeFile.write((uint8_t*)&_recordCount, sizeof(_recordCount));
        _activeFile.close();
    }

    // [처리 단위 2] WAV 원시 오디오 최종 데이터 사이즈 헤더 영역 소급 기입 후 닫기
    if (_wavFile) {
        _writeWavHeader(_wavFile, _wavWrittenBytes);
        _wavFile.close();
    }

    if (_accFile) _accFile.close();
    if (_gyrFile) _gyrFile.close();

    // [처리 단위 3] 세션 파일 이력을 저장 인덱스에 반영하고 XML/JSON 영속 동기화
    _appendIndexItem();
    _saveIndexAtomic();

    _sessionOpen = false;
    ESP_LOGI(TAG, "Session Closed (%s): %d frames", p_reason, _recordCount);
    xSemaphoreGiveRecursive(_lock);
}

/**
 * @brief 특징 데이터 및 센서 3차원 원시 시퀀스를 프리트리거 링버퍼 또는 비동기 링버퍼에 적재
 */
bool CL_T2_StorageManager::pushFrame(const T2_Type::ST_UnifiedFeatureSlot_t* p_feat,
                                     const T2_Type::ST_Raw_Accel_t* p_rawAcc,
                                     const T2_Type::ST_Raw_Gyro_t* p_rawGyr,
                                     const T2_Type::ST_Raw_Audio_t* p_rawAud) {
    if (_ioError) return false;

    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);

    // [처리 단위 1] 세션 비활성화 상태: 순환형 프리트리거(Pre-Trigger) 버퍼에 밀어 넣음
    if (!_sessionOpen) {
        if (_preFeatBuf) {
            _preFeatBuf[_preHead] = *p_feat;
            if (_preAccBuf && p_rawAcc) _preAccBuf[_preHead] = *p_rawAcc;
            if (_preGyrBuf && p_rawGyr) _preGyrBuf[_preHead] = *p_rawGyr;
            if (_preAudBuf && p_rawAud) _preAudBuf[_preHead] = *p_rawAud;

            _preHead = (_preHead + 1) % _preCapacity;
            if (_preCount < _preCapacity) _preCount++;
        }
    } else {
        // [처리 단위 2] 세션 활성화 상태: 백그라운드 태스크 기입을 위해 비동기 링 버퍼에 push
        uint16_t v_nextHead = (_asyncHead + 1) % ASYNC_RING_CAPACITY;
        if (v_nextHead != _asyncTail) {
            _asyncFeatRing[_asyncHead] = *p_feat;
            if (_asyncAccRing && p_rawAcc) _asyncAccRing[_asyncHead] = *p_rawAcc;
            if (_asyncGyrRing && p_rawGyr) _asyncGyrRing[_asyncHead] = *p_rawGyr;
            if (_asyncAudRing && p_rawAud) _asyncAudRing[_asyncHead] = *p_rawAud;
            _asyncHead = v_nextHead;
        } else {
            ESP_LOGW(TAG, "Async Ring Full! Data dropped.");
        }
    }

    xSemaphoreGiveRecursive(_lock);
    return true;
}

/**
 * @brief 스토리지용 링버퍼 메모리 PSRAM 할당 작업
 */
void CL_T2_StorageManager::_allocateBuffers() {
    const auto& v_cfg = CL_T2_ConfigManager::getInstance().getConfig();
    _preCapacity = v_cfg.storage.pre_trig_sec * T2_Def::Global::System::TELEMETRY_HZ_DEF;
    if (_preCapacity == 0) _preCapacity = 1;

    _preFeatBuf = (T2_Type::ST_UnifiedFeatureSlot_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_UnifiedFeatureSlot_t) * _preCapacity, MALLOC_CAP_SPIRAM);
    _preAccBuf  = (T2_Type::ST_Raw_Accel_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_Raw_Accel_t) * _preCapacity, MALLOC_CAP_SPIRAM);
    _preGyrBuf  = (T2_Type::ST_Raw_Gyro_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_Raw_Gyro_t) * _preCapacity, MALLOC_CAP_SPIRAM);
    _preAudBuf  = (T2_Type::ST_Raw_Audio_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_Raw_Audio_t) * _preCapacity, MALLOC_CAP_SPIRAM);

    _asyncFeatRing = (T2_Type::ST_UnifiedFeatureSlot_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_UnifiedFeatureSlot_t) * ASYNC_RING_CAPACITY, MALLOC_CAP_SPIRAM);
    _asyncAccRing  = (T2_Type::ST_Raw_Accel_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_Raw_Accel_t) * ASYNC_RING_CAPACITY, MALLOC_CAP_SPIRAM);
    _asyncGyrRing  = (T2_Type::ST_Raw_Gyro_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_Raw_Gyro_t) * ASYNC_RING_CAPACITY, MALLOC_CAP_SPIRAM);
    _asyncAudRing  = (T2_Type::ST_Raw_Audio_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_Raw_Audio_t) * ASYNC_RING_CAPACITY, MALLOC_CAP_SPIRAM);

    _bounceFeat = (T2_Type::ST_UnifiedFeatureSlot_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_UnifiedFeatureSlot_t), MALLOC_CAP_SPIRAM);
    _bounceAcc  = (T2_Type::ST_Raw_Accel_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_Raw_Accel_t), MALLOC_CAP_SPIRAM);
    _bounceGyr  = (T2_Type::ST_Raw_Gyro_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_Raw_Gyro_t), MALLOC_CAP_SPIRAM);
    _bounceAud  = (T2_Type::ST_Raw_Audio_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_Raw_Audio_t), MALLOC_CAP_SPIRAM);

    if (!_preFeatBuf || !_preAccBuf || !_preGyrBuf || !_preAudBuf ||
        !_asyncFeatRing || !_asyncAccRing || !_asyncGyrRing || !_asyncAudRing ||
        !_bounceFeat || !_bounceAcc || !_bounceGyr || !_bounceAud) {
        ESP_LOGE(TAG, "Storage buffers PSRAM Allocation Failed!");
        _ioError = true;
    }
}

/**
 * @brief 날짜별(연/월/일) 구조 디렉터리 자동 확인 및 물리적 파일 상대 경로 지정
 */
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

/**
 * @brief 바이너리 특징 저장 데이터셋 규격 파일 헤더 정보 작성
 */
void CL_T2_StorageManager::_writeBinHeader(File& p_file) {
    T2_Type::ST_FileHeader_t v_hdr;
    memset(&v_hdr, 0, sizeof(v_hdr));
    memcpy(v_hdr.magic, "T240", 4);
    v_hdr.ver = 245;
    v_hdr.struct_size = sizeof(T2_Type::ST_UnifiedFeatureSlot_t);

    const auto& v_cfg = CL_T2_ConfigManager::getInstance().getConfig();
    v_hdr.s_rate = v_cfg.accel.sample_rate;
    v_hdr.fft    = v_cfg.accel.fft_size;
    v_hdr.mfcc_d = T2_Def::AI::Tensor::MFCC_DIM_DEF;

    v_hdr.accel_mask = v_cfg.accel.axis_mask;
    v_hdr.gyro_mask  = v_cfg.gyro.axis_mask;
    v_hdr.audio_mask = (uint8_t)v_cfg.audio.channel_mask;
    v_hdr._res       = 0;

    CL_T2_ConfigManager::getInstance().serializeToBuffer(v_hdr.config_dump, sizeof(v_hdr.config_dump));
    p_file.write((uint8_t*)&v_hdr, sizeof(v_hdr));
}

/**
 * @brief PCM RIFF WAV 포맷 규격 헤더 기입
 */
void CL_T2_StorageManager::_writeWavHeader(File& p_file, uint32_t p_dataSize) {
    if (!p_file) return;

    const auto& v_cfg = CL_T2_ConfigManager::getInstance().getConfig();
    uint32_t v_srate = v_cfg.audio.sample_rate;
    uint16_t v_chans = 2;
    uint16_t v_bps   = 32;

    uint8_t v_hdr[44];
    memcpy(v_hdr, "RIFF", 4);
    uint32_t v_chunkSize = p_dataSize + 36;
    memcpy(v_hdr + 4, &v_chunkSize, 4);
    memcpy(v_hdr + 8, "WAVE", 4);
    memcpy(v_hdr + 12, "fmt ", 4);
    uint32_t v_sub1Size = 16;
    memcpy(v_hdr + 16, &v_sub1Size, 4);
    uint16_t v_format = 3;
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

/**
 * @brief 파일 시스템의 사전 기록용 파일 바이트 공간 고속 선점 할당
 */
void CL_T2_StorageManager::_preAllocateFile(File& p_file, uint32_t p_bytes) {
    p_file.seek(p_bytes - 1);
    p_file.write(0);
    p_file.seek(0);
}

/**
 * @brief 비동기 백그라운드 태스크 기동 프로세스
 */
void CL_T2_StorageManager::_storageTaskProc(void* p_param) {
    CL_T2_StorageManager* v_this = (CL_T2_StorageManager*)p_param;
    while(1) {
        v_this->_processRingIO();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief 링 버퍼에 적재된 비동기 쓰기 작업을 SD 카드로 순차적 물리 I/O 출력
 */
void CL_T2_StorageManager::_processRingIO() {
    if (!_sessionOpen || _ioError) return;

    while (_asyncTail != _asyncHead) {
        xSemaphoreTakeRecursive(_lock, portMAX_DELAY);

        *_bounceFeat = _asyncFeatRing[_asyncTail];

        bool v_hasAcc = false;
        if (_asyncAccRing) {
            *_bounceAcc = _asyncAccRing[_asyncTail];
            v_hasAcc = true;
        }
        bool v_hasGyr = false;
        if (_asyncGyrRing) {
            *_bounceGyr = _asyncGyrRing[_asyncTail];
            v_hasGyr = true;
        }
        bool v_hasAud = false;
        if (_asyncAudRing) {
            *_bounceAud = _asyncAudRing[_asyncTail];
            v_hasAud = true;
        }

        _asyncTail = (_asyncTail + 1) % ASYNC_RING_CAPACITY;
        xSemaphoreGiveRecursive(_lock);

        // [처리 단위 1] 이진 특징량 파일 복사 및 기입
        if (_activeFile) {
            _activeFile.write((uint8_t*)_bounceFeat, sizeof(T2_Type::ST_UnifiedFeatureSlot_t));
            _recordCount++;
            _writtenBytes += sizeof(T2_Type::ST_UnifiedFeatureSlot_t);
        }

        // [처리 단위 2] 3축 가속도 raw 파일 기입
        if (_accFile && v_hasAcc) {
            _accFile.write((uint8_t*)_bounceAcc, sizeof(T2_Type::ST_Raw_Accel_t));
            _accWrittenBytes += sizeof(T2_Type::ST_Raw_Accel_t);
        }

        // [처리 단위 3] 3축 자이로 raw 파일 기입
        if (_gyrFile && v_hasGyr) {
            _gyrFile.write((uint8_t*)_bounceGyr, sizeof(T2_Type::ST_Raw_Gyro_t));
            _gyrWrittenBytes += sizeof(T2_Type::ST_Raw_Gyro_t);
        }

        // [처리 단위 4] 2채널 오디오 raw 파일에 대해 인터리빙 결합 후 기입
        if (_wavFile && v_hasAud) {
            alignas(16) float v_interleaved[T2_Def::Audio::Sensor::FFT_SIZE_MAX * 2];
            for (uint32_t i = 0; i < T2_Def::Audio::Sensor::FFT_SIZE_MAX; i++) {
                v_interleaved[i * 2]     = _bounceAud->data[0][i];
                v_interleaved[i * 2 + 1] = _bounceAud->data[1][i];
            }
            _wavFile.write((uint8_t*)v_interleaved, sizeof(v_interleaved));
            _wavWrittenBytes += sizeof(v_interleaved);
        }
    }
}

/**
 * @brief 감지 직전 누적 보관하고 있던 프리트리거 링버퍼 데이터를 비동기 쓰기 링버퍼로 벌크 전송
 */
void CL_T2_StorageManager::_flushPreBufferToRing() {
    if (_preCount == 0) return;

    uint16_t v_readIdx = (_preCount == _preCapacity) ? _preHead : 0;
    for (uint16_t i = 0; i < _preCount; i++) {
        uint16_t v_nextHead = (_asyncHead + 1) % ASYNC_RING_CAPACITY;
        if (v_nextHead == _asyncTail) break;

        _asyncFeatRing[_asyncHead] = _preFeatBuf[v_readIdx];
        if (_asyncAccRing && _preAccBuf) _asyncAccRing[_asyncHead] = _preAccBuf[v_readIdx];
        if (_asyncGyrRing && _preGyrBuf) _asyncGyrRing[_asyncHead] = _preGyrBuf[v_readIdx];
        if (_asyncAudRing && _preAudBuf) _asyncAudRing[_asyncHead] = _preAudBuf[v_readIdx];

        _asyncHead = v_nextHead;
        v_readIdx = (v_readIdx + 1) % _preCapacity;
    }
    _preCount = 0;
    _preHead = 0;
}

/**
 * @brief 용량 제한 또는 설정된 최대 기록 시간에 도달할 경우 기존 세션을 닫고 다음 세션 파일 전개
 */
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

/**
 * @brief 파일 목록 인덱스를 LittleFS 영역에 원자적(Atomic) 트랜잭션 방식으로 안전 기입
 */
bool CL_T2_StorageManager::_saveIndexAtomic() {
    File v_tmp = LittleFS.open("/sys/index_245.tmp", "w");
    if (!v_tmp) return false;

    JsonDocument v_doc;
    JsonArray v_arr = v_doc["files"].to<JsonArray>();
    for (uint16_t i = 0; i < _indexCount; i++) {
        JsonObject v_obj = v_arr.add<JsonObject>();
        v_obj["path"] = _indexItems[i].path;
        v_obj["wav_path"] = _indexItems[i].wav_path;
        v_obj["acc_path"] = _indexItems[i].acc_path;
        v_obj["gyr_path"] = _indexItems[i].gyr_path;
        v_obj["size"] = _indexItems[i].size_bytes;
        v_obj["created"] = _indexItems[i].created_epoch;
        v_obj["records"] = _indexItems[i].record_count;
    }

    serializeJson(v_doc, v_tmp);
    v_tmp.close();

    LittleFS.remove("/sys/runtime_idx_245.json");
    LittleFS.rename("/sys/index_245.tmp", "/sys/runtime_idx_245.json");
    return true;
}

/**
 * @brief LittleFS로부터 누적 파일 인덱스 테이블을 로드
 */
bool CL_T2_StorageManager::_loadIndex() {
    if (!LittleFS.exists("/sys/runtime_idx_245.json")) return false;

    File v_file = LittleFS.open("/sys/runtime_idx_245.json", "r");
    if (!v_file) return false;

    JsonDocument v_doc;
    DeserializationError v_err = deserializeJson(v_doc, v_file);
    v_file.close();
    if (v_err) return false;

    JsonArrayConst v_arr = v_doc["files"];
    _indexCount = 0;
    for (JsonObjectConst v_obj : v_arr) {
        if (_indexCount >= T2_Def::Global::StorageLimit::ROTATE_LIST_MAX) break;
        strlcpy(_indexItems[_indexCount].path,     v_obj["path"] | "",     T2_Def::Global::StorageLimit::PATH_LEN_MAX);
        strlcpy(_indexItems[_indexCount].wav_path, v_obj["wav_path"] | "", T2_Def::Global::StorageLimit::PATH_LEN_MAX);
        strlcpy(_indexItems[_indexCount].acc_path, v_obj["acc_path"] | "", T2_Def::Global::StorageLimit::PATH_LEN_MAX);
        strlcpy(_indexItems[_indexCount].gyr_path, v_obj["gyr_path"] | "", T2_Def::Global::StorageLimit::PATH_LEN_MAX);
        _indexItems[_indexCount].size_bytes    = v_obj["size"] | 0;
        _indexItems[_indexCount].created_epoch = v_obj["created"] | 0;
        _indexItems[_indexCount].record_count  = v_obj["records"] | 0;
        _indexCount++;
    }
    return true;
}

/**
 * @brief 신규 기입이 종료된 세션 파일의 주소들을 인덱스 테이블 끝부분에 덧붙이고, 최대 수량 충족 시 회전(삭제) 개시
 */
void CL_T2_StorageManager::_appendIndexItem() {
    if (_indexCount >= T2_Def::Global::StorageLimit::ROTATE_LIST_MAX) {
        if (SD_MMC.exists(_indexItems[0].path)) SD_MMC.remove(_indexItems[0].path);
        if (SD_MMC.exists(_indexItems[0].wav_path)) SD_MMC.remove(_indexItems[0].wav_path);
        if (SD_MMC.exists(_indexItems[0].acc_path)) SD_MMC.remove(_indexItems[0].acc_path);
        if (SD_MMC.exists(_indexItems[0].gyr_path)) SD_MMC.remove(_indexItems[0].gyr_path);

        for (uint16_t i = 0; i < T2_Def::Global::StorageLimit::ROTATE_LIST_MAX - 1; i++) {
            _indexItems[i] = _indexItems[i + 1];
        }
        _indexCount--;
    }

    strlcpy(_indexItems[_indexCount].path,     _activePath, T2_Def::Global::StorageLimit::PATH_LEN_MAX);
    strlcpy(_indexItems[_indexCount].wav_path, _wavPath,    T2_Def::Global::StorageLimit::PATH_LEN_MAX);
    strlcpy(_indexItems[_indexCount].acc_path, _accPath,    T2_Def::Global::StorageLimit::PATH_LEN_MAX);
    strlcpy(_indexItems[_indexCount].gyr_path, _gyrPath,    T2_Def::Global::StorageLimit::PATH_LEN_MAX);
    _indexItems[_indexCount].size_bytes    = _writtenBytes;
    _indexItems[_indexCount].created_epoch = (uint64_t)time(NULL);
    _indexItems[_indexCount].record_count  = _recordCount;
    _indexCount++;
}

/**
 * @brief 버퍼 즉시 플러시(SD 카드 동기화)
 */
bool CL_T2_StorageManager::flush() {
    if (!_sessionOpen || _ioError) return false;
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    if (_activeFile) _activeFile.flush();
    if (_wavFile) _wavFile.flush();
    if (_accFile) _accFile.flush();
    if (_gyrFile) _gyrFile.flush();
    xSemaphoreGiveRecursive(_lock);
    return true;
}

/**
 * @brief SD 카드 통신 단선 오류 시 리셋 및 자동 복구 시도
 */
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
