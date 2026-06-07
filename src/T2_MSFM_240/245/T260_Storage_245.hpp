/* ============================================================================
 * File: T260_Storage_245.hpp
 * Summary: 멀티모달 비동기 스토리지 엔진 및 개별 Raw 파일 저장기
 * ============================================================================ */

#pragma once

#include "T210_Def_245.hpp"
#include "T215_Type_245.hpp"
#include <FS.h>
#include <LittleFS.h>
#include <SD_MMC.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

/**
 * @class CL_T2_StorageManager
 * @brief SD카드 및 LittleFS와의 파일 입출력을 담당하며, 대용량 센서 데이터를 프리트리거 및 비동기 링 버퍼를 활용해 백그라운드 태스크에서 쓰기(Write) 연산하는 클래스
 */
class CL_T2_StorageManager {
private:
    static constexpr uint16_t ASYNC_RING_CAPACITY = 128; ///< 비동기 파일 입출력 링 버퍼 최대 크기

    bool     _sessionOpen;      ///< 파일 쓰기 세션이 열려 활성화된 상태인지 여부
    bool     _ioError;          ///< SD 카드 등 물리 미디어 쓰기 실패 상태 플래그
    uint32_t _recordCount;      ///< 현재 세션 파일 내 기록된 레코드 개수
    uint32_t _rotationSubSeq;   ///< 로테이션 시 시퀀스 넘버 관리를 위한 일련번호

    File     _activeFile;       ///< 특징량 결과 저장용 바이너리 파일 객체 (.bin)
    File     _wavFile;          ///< 마이크 오디오 원시 파형 저장용 파일 객체 (.wav)
    File     _accFile;          ///< 가속도계 원시 신호 저장용 파일 객체 (.acc)
    File     _gyrFile;          ///< 자이로계 원시 신호 저장용 파일 객체 (.gyr)

    char     _activePath[T2_Def::Global::StorageLimit::PATH_LEN_MAX];
    char     _wavPath[T2_Def::Global::StorageLimit::PATH_LEN_MAX];
    char     _accPath[T2_Def::Global::StorageLimit::PATH_LEN_MAX];
    char     _gyrPath[T2_Def::Global::StorageLimit::PATH_LEN_MAX];
    char     _currentPrefix[T2_Def::Global::StorageLimit::PREFIX_LEN_MAX];

    uint32_t _sessionStartTick;
    uint32_t _writtenBytes;
    uint32_t _wavWrittenBytes;
    uint32_t _accWrittenBytes;
    uint32_t _gyrWrittenBytes;

    /**
     * @struct IndexItem
     * @brief 저장된 파일 이력 관리를 위한 인덱스 레코드 구조체
     */
    struct IndexItem {
        char     path[T2_Def::Global::StorageLimit::PATH_LEN_MAX];
        char     wav_path[T2_Def::Global::StorageLimit::PATH_LEN_MAX];
        char     acc_path[T2_Def::Global::StorageLimit::PATH_LEN_MAX];
        char     gyr_path[T2_Def::Global::StorageLimit::PATH_LEN_MAX];
        uint32_t size_bytes;
        uint64_t created_epoch;
        uint32_t record_count;
    } _indexItems[T2_Def::Global::StorageLimit::ROTATE_LIST_MAX];

    uint16_t          _indexCount; ///< 현재 저장소 내 이력 관리 파일 수
    SemaphoreHandle_t _lock;       ///< 데이터 무결성 스레드 락 세마포어

    // --- 프리트리거 버퍼 (PSRAM) ---
    T2_Type::ST_UnifiedFeatureSlot_t* _preFeatBuf; ///< 프리트리거 특징량 지동 저장소
    T2_Type::ST_Raw_Accel_t*       _preAccBuf;  ///< 프리트리거 가속도 원시데이터 저장소
    T2_Type::ST_Raw_Gyro_t*        _preGyrBuf;  ///< 프리트리거 자이로 원시데이터 저장소
    T2_Type::ST_Raw_Audio_t*       _preAudBuf;  ///< 프리트리거 오디오 원시데이터 저장소
    uint16_t                          _preCapacity;
    uint16_t                          _preHead, _preCount;

    // --- 비동기 기록 링버퍼 (PSRAM) ---
    T2_Type::ST_UnifiedFeatureSlot_t* _asyncFeatRing;
    T2_Type::ST_Raw_Accel_t*       _asyncAccRing;
    T2_Type::ST_Raw_Gyro_t*        _asyncGyrRing;
    T2_Type::ST_Raw_Audio_t*       _asyncAudRing;
    uint16_t                          _asyncHead;
    uint16_t                          _asyncTail;

    // PSRAM 바운스 버퍼 (스택 오버플로우 방지 및 전사 정렬)
    T2_Type::ST_UnifiedFeatureSlot_t* _bounceFeat;
    T2_Type::ST_Raw_Accel_t*       _bounceAcc;
    T2_Type::ST_Raw_Gyro_t*        _bounceGyr;
    T2_Type::ST_Raw_Audio_t*       _bounceAud;

    TaskHandle_t _hStorageTask;    ///< 파일 백그라운드 쓰기 스레드 태스크 핸들

public:
    CL_T2_StorageManager();
    ~CL_T2_StorageManager();

    /**
     * @brief 싱글톤 인스턴스 반환
     */
    static CL_T2_StorageManager& getInstance() {
        static CL_T2_StorageManager v_inst;
        return v_inst;
    }

    /**
     * @brief LittleFS, SD_MMC 초기화 및 저장소 디렉터리 생성
     * @return 마운트 및 구동 성공 여부
     */
    bool init();

    /**
     * @brief 신규 데이터 저장 세션을 오프닝하고 필요한 Raw 파일 스트림을 엶
     * @param p_prefix 파일명 선두 문자열 (예: trg_auto, man)
     * @param p_overrideDir 특정 저장 폴더 강제 지정 시 경로
     * @return 오픈 성공 여부
     */
    bool openSession(const char* p_prefix, const char* p_overrideDir = nullptr);

    /**
     * @brief 열려있는 모든 파일 스트림을 닫고 헤더 및 메타데이터를 파일에 최종 기입 완료
     * @param p_reason 닫는 사유에 대한 로그
     */
    void closeSession(const char* p_reason);

    /**
     * @brief 취득 특징량 및 원시 센서 데이터 구조체를 비동기 기록 링 버퍼에 push
     * @param p_feat 특징량 결과물
     * @param p_rawAcc 가속도 raw 파형
     * @param p_rawGyr 자이로 raw 파형
     * @param p_rawAud 오디오 raw 파형
     * @return 버퍼 push 성공 여부
     */
    bool pushFrame(const T2_Type::ST_UnifiedFeatureSlot_t* p_feat,
                   const T2_Type::ST_Raw_Accel_t* p_rawAcc,
                   const T2_Type::ST_Raw_Gyro_t* p_rawGyr,
                   const T2_Type::ST_Raw_Audio_t* p_rawAud);

    /**
     * @brief 현재 활성화된 바이너리 특징량 결과 파일 경로 획득
     */
    const char* getLastPath() const { return _activePath; }

    /**
     * @brief 링 버퍼 상에 대기 중인 모든 데이터를 즉시 디바이스에 물리적 동기화(Flush)
     */
    bool flush();

    /**
     * @brief 현재 총 스토리지 크기를 평가하고 보관 파일 최대 용량 초과 시 자동 삭제(로테이션)
     */
    void checkRotation();

    /**
     * @brief 쓰기 오류 감지 시 LittleFS/SD카드 장치를 강제 재마운트 시도
     */
    bool attemptRecovery();

    /**
     * @brief 세션 열림 여부 확인
     */
    bool isSessionOpen() const { return _sessionOpen; }

    /**
     * @brief IO 장치 오류 지속 상태 확인
     */
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
