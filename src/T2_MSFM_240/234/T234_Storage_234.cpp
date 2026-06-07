
/* 
============================================================================
 * File: T234_Storage_234.cpp
 * Summary: Storage Engine Implementation with Pre-Trigger Buffering & Fail-Safe
 * * * * [AI 메모: 제공 기능 요약] * * *
 * 1. Zero-Copy DMA: 내부 SRAM 핑퐁 버퍼 3개를 순환하며 SD카드 기록 오버헤드를 극소화.
 * 2. 가변 차원 직렬화: 1/3축 모드 및 MFCC 크기에 맞춰 패딩을 제외한 순수 데이터만 Packing하여 파일 크기 최적화.
 * 3. Pre-Trigger Buffering: 유휴 상태일 때 PSRAM 링버퍼에 과거 데이터를 상시 유지, 트리거 시 일괄 플러시하여 사고 징후 완벽 캡처.
 * 4. 자동 파일 분할(Rotation): 설정 용량(MB)/시간(Min) 도달 시 끊김 없이 새 파일로 분할하고 오래된 파일 삭제.
 *
 * * * * [AI 메모: 구현 및 유지보수 주의사항] * * *
 * 1. 프리-트리거 링버퍼(_pre_buf)는 크기가 크므로 반드시 MALLOC_CAP_SPIRAM(PSRAM)으로 할당해야 합니다.
 * 2. pushVector 함수는 레코딩 상태(_session_open)와 무관하게 상시 호출되어야 링버퍼가 정상 작동합니다.
 * 3. DMA 슬롯 커밋(_commitSlot) 도중 I/O 블로킹이 길어지면 FIFO 오버플로우가 발생할 수 있으므로 idle_flush_ms 튜닝이 중요합니다.
 *
 * * * * [AI 셀프 회고 및 구현 원칙 - Phase 3: 무결점 스토리지 및 데이터 보존] * * *
 * * [방어 1: 다중 스레드 동시 접근에 의한 상태 파괴 (Race Condition) 방어]
 * - 실수: processTask와 recorderTask가 동일한 Storage 객체에 동시 접근하여 파일 시스템 파괴 및 Use-After-Free 패닉 유발.
 * - 원칙: StorageService 내부에 Recursive Mutex를 도입하여 모든 Public API를 스레드 세이프(Thread-safe)하게 격리할 것.
 * * [방어 2: 파일 로테이션 붕괴 및 SD 시한폭탄 차단]
 * - 실수: 인덱스(JSON) 저장 시 배열 '추가(Append)'와 '저장(Write)'을 섞어 써서 최신 세션이 중복 증식하고, Raw 파형 파일 경로가 누락되어 SD 용량이 100% 고갈됨.
 * - 원칙: 배열 추가(_appendIndexItem)와 파일 저장(_writeIndexFile) 로직을 완벽히 분리하고, ST_IndexItem에 raw_path를 명시하여 파일 쌍(Pair)을 동시 삭제할 것.
 * * [방어 3: 정전 시 설정 파일 영구 파괴(0 Byte) 방어]
 * - 실수: LittleFS에 "w" 모드로 여는 순간 파일 크기가 0이 되므로, 기록 중 정전 시 기기가 벽돌(Brick)이 됨.
 * - 원칙: JSON 저장 시 반드시 ".tmp" 확장자로 먼저 기록하고 성공 시 rename() 하는 원자적 쓰기(Atomic Write)를 적용할 것.
 * * [방어 4: SD 카드 강제 탈거(Hot-plug) 무한 패닉 방어]
 * - 실수: 기록 중 SD 카드가 뽑히면 write() 실패를 무시하고 무한 재시도하여 Task Watchdog 패닉 유발.
 * - 원칙: I/O 에러 발생 시 _io_error 플래그를 세우고 파일 핸들을 즉각 닫아 무의미한 I/O 시도를 원천 차단할 것.
 * * [방어 5: Raw 파형 기록 시 DMA 캐시 충돌 패닉 (Cache Disabled)]
 * - 실수: PSRAM에 위치한 raw_buffer를 SD_MMC.write()로 직결하여 하드웨어 캐시 미스 패닉 유발.
 * - 원칙: pushRaw() 내부에도 MALLOC_CAP_INTERNAL 바운스 버퍼를 동적 할당하여 안전한 내부 SRAM 영역을 경유해 기록할 것.
 * * [방어 6: 0 나누기 수학적 패닉(OOM) 방어]
 * - 실수: 프리트리거 할당 시 hop_size 0 유입 시 FPS가 무한대가 되어 수십 MB PSRAM 할당 시도로 즉사.
 * - 원칙: 나눗셈 연산 전 분모가 될 변수(hop_size 등)는 반드시 1 이상으로 클램핑(Clamping) 할 것.
 * ========================================================================== 
 */



#include "T234_Storage_234.h"
#include "T219_Config_234.h" // JSON 직렬화 엔진 포함
#include <ArduinoJson.h>
#include <Preferences.h> // NVS 사용
#include <time.h>        // 시간 포맷팅


CL_T20_StorageService::CL_T20_StorageService() {
    _lock = xSemaphoreCreateRecursiveMutex(); // 동일 스레드 재진입 허용 Mutex
    _session_open = false;
    _io_error = false;
    _record_count = 0;
    _backend = EN_T20_STORAGE_LITTLEFS;
    _dma_active_slot = 0;
    _batch_count = 0;
    _last_push_ms = 0;
    _watermark_high = 8;     // T20::C10_Rec::BATCH_WMARK_HIGH
    _idle_flush_ms = 250;    // T20::C10_Rec::BATCH_IDLE_FLUSH_MS
    _index_count = 0;
    _rotate_keep_max = 8;    // T20::C10_Rec::ROTATE_KEEP_MAX

    _written_bytes = 0;
    _session_start_ms = 0;

    memset(_active_path, 0, sizeof(_active_path));
    memset(_active_raw_path, 0, sizeof(_active_raw_path)); 
    memset(_last_error, 0, sizeof(_last_error));
    memset(_current_prefix, 0, sizeof(_current_prefix)); 

    
    memset(_dma_slot_used, 0, sizeof(_dma_slot_used));
    memset(_dma_slots, 0, sizeof(_dma_slots));
    memset(_index_items, 0, sizeof(_index_items));
}



// 소멸자에서 PSRAM 링버퍼 메모리 반환
CL_T20_StorageService::~CL_T20_StorageService() {
    if (_lock) vSemaphoreDelete(_lock); // [커널 메모리 누수 방지
    if (_pre_buf) {
        heap_caps_free(_pre_buf);
        _pre_buf = nullptr;
    }
}


bool CL_T20_StorageService::begin(const ST_T20_SdmmcProfile_t& profile) {
    _loadIndexJson();
    
    // 시스템 부팅 시 딱 한 번만 NVS를 업데이트하여 플래시 수명 파괴를 원천 차단
    Preferences prefs;
    prefs.begin(T20::C10_NVS::NAMESPACE, false);
    
    _boot_file_seq = prefs.getUInt(T20::C10_NVS::KEY_FILE_SEQ, 1);
    prefs.putUInt(T20::C10_NVS::KEY_FILE_SEQ, _boot_file_seq + 1);
    
    prefs.end();

    if (profile.clk_pin != 0xFFU) { // PIN_NOT_SET
        SD_MMC.setPins(profile.clk_pin, profile.cmd_pin, profile.d0_pin,
                       profile.d1_pin, profile.d2_pin, profile.d3_pin);
    }

    if (SD_MMC.begin("/sdcard", profile.use_1bit_mode)) {
        _backend = EN_T20_STORAGE_SDMMC;
        if (!SD_MMC.exists("/t20_data")) SD_MMC.mkdir("/t20_data");
        if (!SD_MMC.exists("/t20_data/bin")) SD_MMC.mkdir("/t20_data/bin");
        writeEvent(profile.use_1bit_mode ? "sd_mount_1bit_ok" : "sd_mount_4bit_ok");
    } else {
        _backend = EN_T20_STORAGE_LITTLEFS;
        LittleFS.begin(true);
        if (!LittleFS.exists("/fallback")) LittleFS.mkdir("/fallback");
        writeEvent("sd_mount_fail_fallback_fs");
    }
    return true;
}


void CL_T20_StorageService::setConfig(const ST_T20_Config_t& cfg) {
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    _current_cfg = cfg;
    _allocatePreBuffer(); // 세션 오픈 전이라도, 설정이 주입되면 즉시 링버퍼를 메모리에 확보합니다.
    xSemaphoreGiveRecursive(_lock);
}


// ============================================================================
// 1. 세션 오픈: 파일 생성 및 프리트리거 버퍼 점검
// ============================================================================

bool CL_T20_StorageService::openSession(const ST_T20_Config_t& cfg, const char* prefix) {
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    
    if (_session_open) {
        xSemaphoreGiveRecursive(_lock);
        return false;
    }
    _current_cfg = cfg;
    
    // 설정 갱신 및 버퍼 재확인 (이미 setConfig로 할당되어 있다면 무시됨)
    setConfig(cfg);
    
    // 매번 NVS를 읽고 쓰는 기존 코드 삭제
    _rotation_sub_seq++; // 세션이 열릴 때마다 서브 시퀀스만 증가
    
    _io_error = false; // 새 세션 진입 시 에러 해제
    
    // 수동/자동 기록 꼬리표 유지
    strlcpy(_current_prefix, prefix, sizeof(_current_prefix));

    // 파일명 생성 (타임스탬프 기반)
    struct tm timeinfo;
    char time_buffer[64];
    if (getLocalTime(&timeinfo, 10)) {
        // 파일명에 서브 시퀀스 번호 포함
        snprintf(time_buffer, sizeof(time_buffer), "%04lu_%03u_%04d%02d%02d_%02d%02d%02d",
                 (unsigned long)_boot_file_seq, _rotation_sub_seq,
                 timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                 timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    } else {
        snprintf(time_buffer, sizeof(time_buffer), "%04lu_%03u_notime", (unsigned long)_boot_file_seq, _rotation_sub_seq);
    }

     // 접두어(Prefix) 적용: rec_trg_0001_...bin 또는 rec_man_0002_...bin 형태
    if (_backend == EN_T20_STORAGE_SDMMC) {
        snprintf(_active_path, sizeof(_active_path), "%s/rec_%s_%s.bin", T20::C10_Path::SD_DIR_BIN, prefix, time_buffer);
    } else {
        snprintf(_active_path, sizeof(_active_path), "/fallback/rec_%s_%s.bin", prefix, time_buffer);
    }

    _active_file = (_backend == EN_T20_STORAGE_SDMMC) ? SD_MMC.open(_active_path, "w") : LittleFS.open(_active_path, "w");
    
    if (!_active_file) {
        xSemaphoreGiveRecursive(_lock);
        return false;
    }

    // 확장된 바이너리 헤더 작성
    ST_T20_RecorderBinaryHeader_t header;
    memset(&header, 0, sizeof(header));
    header.magic = T20::C10_Rec::BINARY_MAGIC;
    header.version = 219; 
    header.header_size = sizeof(header);
    header.sample_rate_hz = (uint32_t)T20::C10_DSP::SAMPLE_RATE_HZ;
    header.fft_size = (uint16_t)cfg.feature.fft_size; 
    header.mfcc_dim = cfg.feature.mfcc_coeffs;       
    header.active_axes = (uint8_t)cfg.feature.axis_count; 

    String json_str;
    CL_T20_ConfigJson::buildJsonString(cfg, json_str);
    strlcpy(header.config_dump, json_str.c_str(), sizeof(header.config_dump));

    if (_active_file.write((const uint8_t*)&header, sizeof(header)) != sizeof(header)) {
        _io_error = true; // 초기 헤더 기록 실패 시에도 에러 상태를 명확히 인지
        _active_file.close();
        setLastError("session_open_header_write_fail");
        xSemaphoreGiveRecursive(_lock);
        return false;
    }

    // Raw 저장 모드 시 3축 대응 파일 오픈 및 트래킹 경로 저장
    // 방어: Raw 파일도 동일하게 타임스탬프 유지 및 전역 트래킹 변수에 저장
    memset(_active_raw_path, 0, sizeof(_active_raw_path));
    if (cfg.storage.save_raw) {
        if (_backend == EN_T20_STORAGE_SDMMC) {
            snprintf(_active_raw_path, sizeof(_active_raw_path), "/t20_data/raw/raw_%s_%s.bin", prefix, time_buffer);
            _raw_file = SD_MMC.open(_active_raw_path, "w");
        } else {
            snprintf(_active_raw_path, sizeof(_active_raw_path), "/fallback/raw_%s_%s.bin", prefix, time_buffer);
            _raw_file = LittleFS.open(_active_raw_path, "w");
        }
    }

    _record_count = 0;
    _written_bytes = sizeof(header);
    _session_start_ms = millis();
    _session_open = true;
    
    xSemaphoreGiveRecursive(_lock);
    return true;
}

void CL_T20_StorageService::closeSession(const char* reason) {
    
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    
    if (!_session_open) {
        xSemaphoreGiveRecursive(_lock);
        return;
    }
    
    flush();

    // 방어: 물리적 에러가 발생하지 않은 정상 파일만 헤더 꼬리표 부착
    if (_active_file && !_io_error) {
		_active_file.seek(offsetof(ST_T20_RecorderBinaryHeader_t, record_count));	
		_active_file.write((const uint8_t*)&_record_count, sizeof(_record_count));
        _active_file.close();
    }
    if (_raw_file) _raw_file.close();

    _session_open = false;
    writeEvent(reason);
    
    // A2 로테이션 붕괴 방어: 배열 추가 로직과 JSON 저장을 완벽히 분리
    _appendIndexItem(); 
    _writeIndexFile();
    _handleRotation();
    
    xSemaphoreGiveRecursive(_lock);
}



// ============================================================================
// 3. 진입점: 상시 데이터 수신 및 분기 처리
// ============================================================================

bool CL_T20_StorageService::pushVector(const ST_T20_FeatureVector_t* p_vec) {
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    
    if (!p_vec){
        xSemaphoreGiveRecursive(_lock);
        return false;
    }

    // [상태 1] 레코딩 중이 아닐 때 -> 프리트리거 링버퍼에 상시 기록 (덮어쓰기)
    if (!_session_open) {
        if (_pre_capacity > 0 && _pre_buf) {
            memcpy(&_pre_buf[_pre_head], p_vec, sizeof(ST_T20_FeatureVector_t));
            _pre_head = (_pre_head + 1) % _pre_capacity;
            if (_pre_count < _pre_capacity) _pre_count++;
        }
        xSemaphoreGiveRecursive(_lock);
        return true; 
    }

    // [상태 2] 세션이 열려있고, 과거 데이터가 쌓여있다면 -> 즉시 일괄 플러시
    // 트리거로 인해 방금 openSession이 호출된 직후의 첫 프레임 진입 시 동작함
    if (_pre_count > 0) {
        _flushPreBuffer();
    }

    // [상태 3] 현재 들어온 실시간 데이터 기록
    bool v_rst = _pushToDma(p_vec);
    xSemaphoreGiveRecursive(_lock);
    return v_rst;
}

// Raw 데이터 기록 (3축 모드 시 3축 파형 통합 기록 대응 가능)
// TODO 기존 함수 대비 누락 기능 체쿠 필요
bool CL_T20_StorageService::pushRaw(const float* p_raw_x, const float* p_raw_y, const float* p_raw_z, uint16_t len, uint8_t active_axes) {
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY); // [추가] 스레드 안전성 확보
    
    if (!_session_open || !_raw_file || !p_raw_x || _io_error) {
        xSemaphoreGiveRecursive(_lock);
        return false;
    }

    size_t total_bytes = len * active_axes * sizeof(float);
    
    // [변경/추가] A7 DMA 패닉 방어: PSRAM 데이터를 SD 카드로 직결 시 발생하는 Cache Miss 패닉 차단.
    // 1축/3축 구분 없이 무조건 내부 SRAM 바운스 버퍼를 동적 할당하여 경유시킵니다.
    float* bounce_buf = (float*)heap_caps_malloc(total_bytes, MALLOC_CAP_INTERNAL);
    if (!bounce_buf) {
        setLastError("raw_bounce_buf_oom");
        xSemaphoreGiveRecursive(_lock);
        return false; 
    }

    if (active_axes == 1) {
        memcpy(bounce_buf, p_raw_x, total_bytes);
    } else {
        for (uint16_t i = 0; i < len; i++) {
            bounce_buf[i * 3 + 0] = p_raw_x[i];
            bounce_buf[i * 3 + 1] = p_raw_y[i];
            bounce_buf[i * 3 + 2] = p_raw_z[i];
        }
    }
    
    size_t total_written = _raw_file.write((const uint8_t*)bounce_buf, total_bytes);
    heap_caps_free(bounce_buf); // 즉각 반환
    
    if (total_written != total_bytes) {
        _io_error = true;
        _raw_file.close();
        setLastError("raw_write_fail_sd_error");
        xSemaphoreGiveRecursive(_lock);
        return false;
    }
    
    _written_bytes += total_written;
    xSemaphoreGiveRecursive(_lock);
    return true;
}

bool CL_T20_StorageService::flush() {
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    
    if (!_session_open || _io_error) {
        xSemaphoreGiveRecursive(_lock);
        return false;
    }

    bool ok = _commitSlot(_dma_active_slot);
    _batch_count = 0;
    
    xSemaphoreGiveRecursive(_lock);
    return ok;
}

void CL_T20_StorageService::checkIdleFlush() {
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    
    if (_session_open && _batch_count > 0 && (millis() - _last_push_ms) >= _idle_flush_ms) {
        flush();
    }
    xSemaphoreGiveRecursive(_lock);
}

bool CL_T20_StorageService::_commitSlot(uint8_t slot_idx) {
    uint16_t used = _dma_slot_used[slot_idx];
    if (used == 0) return true;
    if (!_active_file) return false;
    
    size_t written = _active_file.write((const uint8_t*)_dma_slots[slot_idx], used);
    
    // SD 카드 탈거 시 즉시 세션을 죽이고 I/O 포기
    if (written != used) {
        _io_error = true;
        _active_file.close();
        setLastError("dma_commit_fail_sd_error");
        return false;
    }

    _dma_slot_used[slot_idx] = 0;
    return true;
}

void CL_T20_StorageService::writeEvent(const char* event_msg) {
    if (event_msg) strlcpy(_last_error, event_msg, sizeof(_last_error));
}

void CL_T20_StorageService::setLastError(const char* err_msg) {
    if (err_msg) strlcpy(_last_error, err_msg, sizeof(_last_error));
}


void CL_T20_StorageService::_handleRotation() {
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    
    if (_index_count <= _rotate_keep_max) {
        xSemaphoreGiveRecursive(_lock);
        return;
    }

    bool any_deleted = false;
    
    // 0이면 무제한이므로 삭제 로직 수행 안함
    if (_rotate_keep_max == 0) {
        xSemaphoreGiveRecursive(_lock);
        return;
    }
    
     // 허용치를 초과한 개수만큼 무조건 맨 앞(인덱스 0) 요소 삭제 시도
    while (_index_count > _rotate_keep_max) {
        char old_path[128];
        char old_raw_path[128]; 
        strlcpy(old_path, _index_items[0].path, sizeof(old_path));
        strlcpy(old_raw_path, _index_items[0].raw_path, sizeof(old_raw_path));

        if (old_path[0] != '\0') {
            if (_backend == EN_T20_STORAGE_SDMMC) SD_MMC.remove(old_path);
            else LittleFS.remove(old_path);
        }
        
        // A1 방어: Raw 파형 파일도 함께 쌍(Pair)으로 삭제하여 SD 용량 누수 원천 차단
        if (old_raw_path[0] != '\0') {
            if (_backend == EN_T20_STORAGE_SDMMC) SD_MMC.remove(old_raw_path);
            else LittleFS.remove(old_raw_path);
        }
        
        // 삭제 성공 여부와 무관하게(SD 에러 등) 인덱스에서는 무조건 배제하여 배열 오버플로우 영구 방어
        for (uint16_t i = 1; i < _index_count; ++i) {
            _index_items[i - 1] = _index_items[i];
        }
        _index_count--;
        any_deleted = true;
    }

    if (any_deleted) _writeIndexFile();
    
    xSemaphoreGiveRecursive(_lock);
}

void CL_T20_StorageService::_appendIndexItem() {
    if (_index_count < MAX_ROTATE_LIST) {
        strlcpy(_index_items[_index_count].path, _active_path, 128);
        strlcpy(_index_items[_index_count].raw_path, _active_raw_path, 128); // [추가]
        _index_items[_index_count].record_count = _record_count;
        
        // 깨진 파일 크기 계측 제외
        if (!_io_error) {
            _index_items[_index_count].size_bytes = (_backend == EN_T20_STORAGE_SDMMC) ? SD_MMC.open(_active_path, "r").size() : LittleFS.open(_active_path, "r").size();
        } else {
            _index_items[_index_count].size_bytes = 0; 
        }
        
        _index_items[_index_count].created_ms = millis();
        _index_count++;
    }
}

// A6 원자적 쓰기(Atomic Write) 도입: 정전 시 파일 증발을 막기 위해 임시 파일(.tmp) 생성 후 이름 변경
bool CL_T20_StorageService::_writeIndexFile() {
    File file = LittleFS.open("/sys/recorder_index.tmp", "w");
    if (!file) return false;

    JsonDocument doc;
    doc["count"] = _index_count;
    JsonArray arr = doc["items"].to<JsonArray>();

    for (uint16_t i = 0; i < _index_count; i++) {
        JsonObject item = arr.add<JsonObject>();
        item["path"] = _index_items[i].path;
        item["raw_path"] = _index_items[i].raw_path; // [추가]
        item["record_count"] = _index_items[i].record_count;
        item["size_bytes"] = _index_items[i].size_bytes;
        item["created_ms"] = _index_items[i].created_ms;
    }

    serializeJson(doc, file);
    file.close();

    // 방어 6: remove()를 호출하면 그 찰나의 정전 시 파일이 완전히 증발하므로 삭제. 
    // rename() 자체가 원자적 덮어쓰기(Atomic Overwrite)를 완벽히 보장함.
    LittleFS.rename("/sys/recorder_index.tmp", "/sys/recorder_index.json");
    return true;
}


bool CL_T20_StorageService::_loadIndexJson() {
    if (!LittleFS.exists("/sys/recorder_index.json")) return false;
    File file = LittleFS.open("/sys/recorder_index.json", "r");
    if (!file) return false;

    JsonDocument doc;
    if (deserializeJson(doc, file)) { file.close(); return false; }
    file.close();

    _index_count = doc["count"] | 0;
    JsonArray arr = doc["items"].as<JsonArray>();

    uint16_t i = 0;
    for (JsonObject item : arr) {
        if (i >= MAX_ROTATE_LIST) break;
        strlcpy(_index_items[i].path, item["path"] | "", 128);
        strlcpy(_index_items[i].raw_path, item["raw_path"] | "", 128); 
        _index_items[i].record_count = item["record_count"] | 0;
        _index_items[i].size_bytes = item["size_bytes"] | 0;
        _index_items[i].created_ms = item["created_ms"] | 0;
        i++;
    }
    return true;
}


// 로테이션 감시 루틴
void CL_T20_StorageService::checkRotation() {
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);

    if (!_session_open) {
        xSemaphoreGiveRecursive(_lock);
        return;
    }
    bool do_rotate = false;

    // 1. 용량 기반 (MB)
    if (_current_cfg.storage.rotation_mb > 0) {
        if (_written_bytes >= (_current_cfg.storage.rotation_mb * 1024 * 1024)) do_rotate = true;
    }
    // 2. 시간 기반 (Min)
    if (_current_cfg.storage.rotation_min > 0) {
        if ((millis() - _session_start_ms) >= (_current_cfg.storage.rotation_min * 60000)) do_rotate = true;
    }

    if (do_rotate) {
        closeSession("rotate");
        // 방어 7: 기본값("trg") 대신 현재 세션의 접두어를 그대로 물려주어 출처 보존
        openSession(_current_cfg, _current_prefix); 
    }
    xSemaphoreGiveRecursive(_lock);
}


// ============================================================================
// 2. 프리-트리거 링버퍼 메모리 동적 할당 관리
// ============================================================================
void CL_T20_StorageService::_allocatePreBuffer() {
    // pre_trigger_sec이 0으로 설정된 경우 기존 메모리 해제 로직 추가
    if (_current_cfg.storage.pre_trigger_sec == 0) {
        if (_pre_buf) {
            heap_caps_free(_pre_buf);
            _pre_buf = nullptr;
            _pre_capacity = 0;
            _pre_head = 0;
            _pre_count = 0;
        }
        return;
    }
    
    // A3 방어: JSON 파싱 오류로 인한 hop_size 0 유입 시 Divide by Zero 및 OOM 즉사 차단
    uint16_t safe_hop = _current_cfg.feature.hop_size;
    if (safe_hop == 0) safe_hop = 1;
    // 초당 프레임 수(FPS) = 1600Hz / Hop Size
    float fps = T20::C10_DSP::SAMPLE_RATE_HZ / (float)safe_hop;

    uint16_t req_capacity = (uint16_t)(fps * _current_cfg.storage.pre_trigger_sec);

    // 용량이 변경되었거나 아직 할당되지 않은 경우 PSRAM에 재할당
    if (_pre_capacity != req_capacity) {
        if (_pre_buf) heap_caps_free(_pre_buf);
        // SIMD 구조체 호환을 위해 16바이트 정렬 할당 적용
        _pre_buf = (ST_T20_FeatureVector_t*)heap_caps_aligned_alloc(16, req_capacity * sizeof(ST_T20_FeatureVector_t), MALLOC_CAP_SPIRAM);
        
        if (_pre_buf) {
            _pre_capacity = req_capacity;
            _pre_head = 0;
            _pre_count = 0;
        } else {
            Serial.println(F("[Storage] Pre-Trigger PSRAM Allocation Failed!"));
            _pre_capacity = 0;
        }
    }
    
    // [상세 설명]
    // 포인터 초기화 (_pre_head = 0, _pre_count = 0)는 위처럼 메모리가 "새로 할당"될 때만 수행합니다.
    // 만약 이미 할당되어 있다면, 세션이 방금 열렸다 하더라도 버퍼 인덱스를 초기화하지 않고 그대로 둡니다.
    // 이유: 세션이 닫혀 있던 유휴 시간 동안 링버퍼에 과거의 정상/진동 데이터가 차곡차곡 쌓이고 있었으므로,
    // 세션이 열리는 즉시 이 '과거 기록 이력'을 그대로 보존하여 플러시(Flush)해야 사고 직전의 상황 분석이 가능하기 때문입니다.
}


// ============================================================================
// 4. 프리-트리거 과거 데이터 일괄 플러시
// ============================================================================
void CL_T20_StorageService::_flushPreBuffer() {
    if (_pre_count == 0 || !_pre_buf) return;

    // 링버퍼에서 가장 오래된 데이터의 인덱스 계산 (데이터가 꽉 찼으면 head가 가장 오래된 데이터)
    uint16_t oldest_idx = (_pre_count == _pre_capacity) ? _pre_head : 0;
    uint16_t count_to_flush = _pre_count;
    
    // 무한 루프나 중복 기록 방지를 위해 플래그 즉시 초기화
    _pre_count = 0; 
    _pre_head = 0;

    // 가장 오래된 과거 데이터부터 순서대로 DMA 기록 로직 태우기
    for (uint16_t i = 0; i < count_to_flush; i++) {
        uint16_t idx = (oldest_idx + i) % _pre_capacity;
        _pushToDma(&_pre_buf[idx]);
    }
    
    Serial.printf("[Storage] Flushed %d pre-trigger frames to SD.\n", count_to_flush);
}


// ============================================================================
// 5. DMA 슬롯 직렬화 및 물리적 기록 헬퍼
// ============================================================================
bool CL_T20_StorageService::_pushToDma(const ST_T20_FeatureVector_t* p_vec) {
    if (_io_error) return false; // I/O 에러 상태면 즉시 무시
    
    const uint8_t axes = p_vec->active_axes;
    const uint16_t dim_per_axis = _current_cfg.feature.mfcc_coeffs * 3; 
    const uint16_t feature_bytes = (dim_per_axis * axes * sizeof(float));
    
    // [수정됨] 누락되었던 status_flags, rms, band_energy 크기를 반영하여 총 프레임 사이즈 재계산
    const uint16_t total_frame_size = sizeof(p_vec->timestamp_ms) + 
                                      sizeof(p_vec->frame_id) + 
                                      sizeof(p_vec->active_axes) + 
                                      sizeof(p_vec->status_flags) + 
                                      sizeof(p_vec->rms) +
                                      sizeof(p_vec->band_energy) +
                                      feature_bytes;

    uint8_t slot = _dma_active_slot;
    if ((_dma_slot_used[slot] + total_frame_size) > DMA_SLOT_BYTES) {
        if (!_commitSlot(slot)) return false;
        _dma_active_slot = (slot + 1) % DMA_SLOT_COUNT;
        slot = _dma_active_slot;
        if (_dma_slot_used[slot] > 0) return false; 
    }

    uint8_t* ptr = &_dma_slots[slot][_dma_slot_used[slot]];
    
    // [수정됨] 분석 필수 데이터 누락 없이 모두 복사
    memcpy(ptr, &p_vec->timestamp_ms, sizeof(uint64_t)); ptr += sizeof(uint64_t);
    memcpy(ptr, &p_vec->frame_id, sizeof(uint32_t));     ptr += sizeof(uint32_t);
    memcpy(ptr, &p_vec->active_axes, sizeof(uint8_t));   ptr += sizeof(uint8_t);
    memcpy(ptr, &p_vec->status_flags, sizeof(uint8_t));  ptr += sizeof(uint8_t);
    memcpy(ptr, &p_vec->rms, sizeof(p_vec->rms));        ptr += sizeof(p_vec->rms);
    memcpy(ptr, &p_vec->band_energy, sizeof(p_vec->band_energy)); ptr += sizeof(p_vec->band_energy);
    
    for (uint8_t a = 0; a < axes; a++) {
        memcpy(ptr, &p_vec->features[a][0], dim_per_axis * sizeof(float));
        ptr += (dim_per_axis * sizeof(float));
    }

    _dma_slot_used[slot] += total_frame_size;
    _written_bytes += total_frame_size;
    _record_count++;
    _batch_count++;
    _last_push_ms = millis();

    if (_batch_count >= _watermark_high) flush();
    return true;
}

