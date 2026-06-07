/* ============================================================================
 * File: T460_Storage_013.hpp
 * Summary: Asynchronous Storage Engine (A-DSE) with WAV & Versioning
 * [SMEA-100 핵심 구현 원칙 및 AI 셀프 회고 바이블]
 * 본 주석은 프로젝트 전반에 걸쳐 절대 삭제되어서는 안 되며,
 * 코드를 수정/확장할 때마다 항상 최신 상태로 유지하고 점검해야 합니다.
 * ============================================================================
 * [1. 시스템 아키텍처 및 하드웨어 구현 원칙]
 * 1. 매직넘버 철폐: 모든 설정은 SmeaConfig 네임스페이스 상수화.
 * 2. 16-Byte 정렬(SIMD 최적화): alignas(16) 강제 정렬.
 * 3. [네이밍 컨벤션 엄수]: private(_), 매개변수(p_), 로컬변수(v_)
 * 4. [동적/정적 상수 승격]: 경로 길이, 단위 변환 상수 등 모든 매직넘버를
 * 제거하고 T410/T415 중앙 설정으로 위임한다.
 * 5. [자료형 엄수]: 일반 int 대신 <cstdint> 기반의 고정 크기 정수형(uint32_t 등) 사용.
 *
 * [2. AI 가 자주 반복하는 실수 및 방어 원칙 (Self-Reflection)]
 * 1. (실수) 다중 스레드(FSM / Web) 동시 접근 시 상태 파괴 (Race Condition).
 * -> (방어) 스토리지 클래스 내부에 xSemaphoreCreateRecursiveMutex 적용.
 * 2. (실수) LittleFS 파일 "w" 쓰기 시 정전 발생 -> 0바이트 파괴 (벽돌화).
 * -> (방어) 인덱스 파일은 무조건 .tmp로 기록 후 rename()하는 원자적 쓰기(Atomic Write) 적용.
 * 3. (실수) SD 카드 탈거 시 무한 I/O 재시도로 인한 Task Watchdog 패닉.
 * -> (방어) I/O 에러 시 _ioError 플래그를 세워 I/O 시도를 원천 차단.
 * 4. (실수) PSRAM 데이터(Raw)를 SD 카드로 직결하여 DMA 캐시 충돌 패닉.
 * -> (방어) Raw 기록 시 MALLOC_CAP_INTERNAL(내부 SRAM) Bounce Buffer 할당 후 경유.
 *
 * [3. v013 고도화 적용: A-DSE 비동기 스토리지 엔진 아키텍처]
 * 1. [SD 블로킹 차단]: Core 1은 이제 직접 파일에 쓰지 않고 PSRAM Ring Buffer에
 * 데이터를 푸시(Non-blocking)한 뒤 즉시 10ms 연산으로 복귀합니다. 실제 기록은
 * _storageTask가 백그라운드에서 전담합니다.
 * 2. [O(N) 탐색 지연 차단]: 단일 폴더 쏠림 현상을 막기 위해 Epoch Time 기반의
 * 날짜별 디렉토리(/YYYY/MM/DD/)를 동적으로 생성하여 파일을 분산시킵니다.
 * 3. [웨어레벨링 멈춤 차단]: 파일 생성 직후 f_seek()를 이용해 10MB 더미 공간을
 * 선할당(Pre-allocate)하여 SD카드 내부 컨트롤러의 프리즈를 막습니다.
 * 4. [포맷 호환성 보장]: Raw 파일에 44바이트 WAV 헤더를 씌워 브라우저 및 Python에서 
 * 즉시 재생 가능하게 하고, 특징량(Feature) 파일에는 SMEA 매직넘버와 버전(013)을 주입.
 * ========================================================================== */
#pragma once

#include "T410_Def_013.hpp"
#include "T420_Types_013.hpp"
#include "T415_ConfigMgr_013.hpp" 
#include <FS.h>
#include <LittleFS.h>
#include <SD_MMC.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <cstdint>

class T460_StorageManager {
public:
    T460_StorageManager();
    ~T460_StorageManager();

    bool init();

    bool openSession(const char* p_prefix = "trg", const char* p_overrideDir = nullptr);
    void closeSession(const char* p_reason = "end_normal");

    // 인덱스 엇갈림 방지를 위한 단일 프레임 동시 푸시
    bool pushFrame(const SmeaType::FeatureSlot* p_featSlot, const SmeaType::RawDataSlot* p_rawSlot);
    // 논블로킹 링버퍼 푸시로 변경
    // bool pushFeatureSlot(const SmeaType::FeatureSlot* p_slot);
    // bool pushRawPcm(const SmeaType::RawDataSlot* p_rawSlot);

    // SD카드 일시적 I/O 에러 자가 복구용
    bool hasIoError() const { return _ioError; }
    bool attemptRecovery();

    bool flush();
    void checkIdleFlush();
    void checkRotation();

    bool isOpen() const { return _sessionOpen; }
    const char* getLastError() const { return _lastError; }
    
    const char* getLastRawPath() const { return _activeRawPath; }


private:
    void _handleRotation();
    void _appendIndexItem();
    bool _writeIndexFileAtomic();
    bool _loadIndexJson();

    void _flushPreBuffer();
    void _allocatePreBuffer();
    
    // 날짜 기반 서브 디렉토리 생성 및 선할당
    void _buildDailyDirectoryPath(char* p_outPath, size_t p_maxLen);
    void _preAllocateFile(File& p_file, uint32_t p_bytes);

    // [v013 추가] WAV 헤더 쓰기 및 파일 포맷 규격화 메서드
    void _writeWavHeader(File& p_file, uint32_t p_dataSize);
    void _writeFileHeader(File& p_file);

    // 비동기 처리용 태스크 및 링버퍼
    static void _storageTaskProc(void* p_param);
    void _processAsyncRingBuffer();

private:
    File _activeFile;     
    File _rawFile;        

    bool _sessionOpen;
    bool _ioError;        

    uint32_t _recordCount;
    char _activePath[SmeaConfig::StorageLimit::MAX_PATH_LEN_CONST];
    char _activeRawPath[SmeaConfig::StorageLimit::MAX_PATH_LEN_CONST]; 
    char _lastError[SmeaConfig::StorageLimit::MAX_PATH_LEN_CONST];
    char _currentPrefix[SmeaConfig::StorageLimit::MAX_PREFIX_LEN_CONST];

    uint32_t _sessionStartTick; // millis() 대신 esp_timer 틱 사용
    uint32_t _writtenBytes;
    uint32_t _rawWrittenBytes;  // [v013 추가] WAV 헤더 갱신용 Raw 데이터 크기 추적

    // --- 파일 로테이션 트래킹 구조체 ---
    struct IndexItem {
        char path[SmeaConfig::StorageLimit::MAX_PATH_LEN_CONST];
        char raw_path[SmeaConfig::StorageLimit::MAX_PATH_LEN_CONST];
        uint32_t size_bytes;
        uint64_t created_epoch; // ms -> epoch time
        uint32_t record_count;
    } _indexItems[SmeaConfig::StorageLimit::MAX_ROTATE_LIST_CONST];

    uint16_t _indexCount;

    SemaphoreHandle_t _lock; 

    // --- Pre-trigger 듀얼 링버퍼 ---
    SmeaType::FeatureSlot* _preFeatBuf = nullptr;
    SmeaType::RawDataSlot* _preRawBuf  = nullptr; 
    float* _bounceBuf = nullptr; 

    uint16_t _preCapacity = 0;
    uint16_t _preHeadFeat = 0, _preCountFeat = 0;
    uint16_t _preHeadRaw  = 0, _preCountRaw  = 0;

    // --- A-DSE PSRAM 비동기 링버퍼 ---
    static constexpr uint16_t ASYNC_RING_CAPACITY = 200; // 약 2초 분량 완충 
    SmeaType::FeatureSlot* _asyncFeatRing = nullptr;
    SmeaType::RawDataSlot* _asyncRawRing = nullptr;
    
    // 멀티 스레드 동기화를 위한 volatile 강제 지정
    volatile uint16_t _asyncHead = 0;
    volatile uint16_t _asyncTail = 0;
    
    TaskHandle_t _hStorageTask = nullptr;

    uint32_t _bootFileSeq = 0;
    uint16_t _rotationSubSeq = 0;
};

