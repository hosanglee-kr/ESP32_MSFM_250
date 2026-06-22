/* ============================================================================
 * File: T260_Storage_250.hpp
 * Summary: 멀티모달 비동기 스토리지 엔진 및 개별 Raw 파일 저장기
 * ============================================================================ */


// TODO: Phase 4 - T260_Storage_247 Extractor Parameter Alignment
//   T260_Storage_250.cpp 내의 _processRingIO() 동작 중 호출되는
//   extractAccel, extractAudio 등의 특징 추출 함수들이
//	 쪼개진 개별 슬롯 구조체(ST_FeatureSlot_Aud_t, ST_FeatureSlot_Vib_t)를 인자로 수용하여 처리할 수 있도록
//	 파라미터 정합성 리팩토링이 연쇄적으로 이어져야 합니다.


#pragma once

#include "T210_Def_250.hpp"
#include "T215_Type_250.hpp"
#include <FS.h>
#include <LittleFS.h>
#include <SD_MMC.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>



class CL_T2_StorageManager {
private:
    static constexpr uint16_t ASYNC_RING_CAPACITY = 128;  // 비동기 링 버퍼 용량
	static constexpr uint8_t  STORAGE_QUEUE_LEN   = 32;   // 저장소 큐 길이

    bool     _sessionOpen;                              // 세션 오픈 여부
    bool     _ioError;                                  // I/O 에러 여부
    uint32_t _recordCount;                              // 레코드 카운트
    uint32_t _rotationSubSeq;                           // 로테이션 서브 시퀀스

    uint64_t _sessionStartUs; 							// 프리트리거 T0 절대 시간축 기록용
    T2_Type::ST_TriggerReason_t _triggerReason; 		// 트리거 원인 기록용

	// 도메인별 분리형 파일 디스크립터
	File _audBinFile;    								// 오디오 특징량 파일(.aud.bin)
    File _vibBinFile;    								// 진동 특징량 파일(.vib.bin)
    File _wavFile;       								// 오디오 원시 파형 파일(.wav)
    File _accFile;       								// 가속도 원시 파형 파일(.acc)
    File _gyrFile;       								// 자이로 원시 파형 파일(.gyr)

	// 도메인별 절대 경로 버퍼
    char _audBinPath[T2_Def::Global::StorageLimit::PATH_LEN_MAX];		// 오디오 특징량 파일(.aud.bin) 경로
    char _vibBinPath[T2_Def::Global::StorageLimit::PATH_LEN_MAX];		// 진동 특징량 파일(.vib.bin) 경로
    char _wavPath[T2_Def::Global::StorageLimit::PATH_LEN_MAX];			// 오디오 원시 파형 파일(.wav) 경로
    char _accPath[T2_Def::Global::StorageLimit::PATH_LEN_MAX];			// 가속도 원시 파형 파일(.acc) 경로
    char _gyrPath[T2_Def::Global::StorageLimit::PATH_LEN_MAX];			// 자이로 원시 파형 파일(.gyr) 경로

    char _currentPrefix[T2_Def::Global::StorageLimit::PREFIX_LEN_MAX];	// 현재 프리픽스

	uint32_t _sessionStartTick;							// 세션 시작 틱
    uint32_t _audBinWrittenBytes;						// 오디오 특징량 파일(.aud.bin) 쓰기 바이트 수
    uint32_t _vibBinWrittenBytes;						// 진동 특징량 파일(.vib.bin) 쓰기 바이트 수
    uint32_t _wavWrittenBytes;						    // 오디오 원시 파형 파일(.wav) 쓰기 바이트 수
    uint32_t _accWrittenBytes;							// 가속도 원시 파형 파일(.acc) 쓰기 바이트 수
    uint32_t _gyrWrittenBytes;						    // 자이로 원시 파형 파일(.gyr) 쓰기 바이트 수


    // 인덱스 매니저 구조 확장 (5대 채널 경로 동시 추적)
    struct IndexItem {
        char aud_bin_path[T2_Def::Global::StorageLimit::PATH_LEN_MAX];		// 오디오 특징량 파일(.aud.bin) 경로
        char vib_bin_path[T2_Def::Global::StorageLimit::PATH_LEN_MAX];		// 진동 특징량 파일(.vib.bin) 경로
        char wav_path[T2_Def::Global::StorageLimit::PATH_LEN_MAX];			// 오디오 원시 파형 파일(.wav) 경로
        char acc_path[T2_Def::Global::StorageLimit::PATH_LEN_MAX];			// 가속도 원시 파형 파일(.acc) 경로
        char gyr_path[T2_Def::Global::StorageLimit::PATH_LEN_MAX];			// 자이로 원시 파형 파일(.gyr) 경로
        uint32_t size_bytes;												// 파일 크기(바이트)
        uint64_t created_epoch;												// 생성 시간(epoch)
        uint32_t record_count;												// 레코드 개수
    } _indexItems[T2_Def::Global::StorageLimit::ROTATE_LIST_MAX];

    uint16_t          			_indexCount;								// 인덱스 개수
    SemaphoreHandle_t 			_lock;										// 락
    SemaphoreHandle_t 			_fsLock; 									// LittleFS 다중 접근 방지 뮤텍스

	// 도메인별 비동기 전송용 프리토스 고속 큐 핸들 (인덱스만 전송)
    QueueHandle_t 				_qAudStorage;								// 오디오 특징량 저장 큐 핸들
    QueueHandle_t 				_qVibStorage;								// 진동 특징량 저장 큐 핸들

    // --- 프리트리거 버퍼 (PSRAM) ---
    T2_Type::ST_FeatureSlot_Aud_t* 	_preAudBuf;								// 오디오 특징량 버퍼
    T2_Type::ST_FeatureSlot_Vib_t* 	_preVibBuf;								// 진동 특징량 버퍼
    T2_Type::ST_Raw_Accel_t* 		_preAccBuf;								// 가속도 원시 파형 버퍼
    T2_Type::ST_Raw_Gyro_t* 		_preGyrBuf;								// 자이로 원시 파형 버퍼
    T2_Type::ST_Raw_Audio_t* 		_preAudRawBuf;							// 오디오 원시 파형 버퍼

	uint16_t 					_preAudCapacity;							// 오디오 특징량 버퍼 용량
    uint16_t 					_preVibCapacity;							// 진동 특징량 버퍼 용량
    uint16_t 					_preAudHead, _preAudCount;					// 오디오 특징량 버퍼 헤드 및 카운트
    uint16_t 					_preVibHead, _preVibCount;					// 진동 특징량 버퍼 헤드 및 카운트


    // --- 비동기 기록 링버퍼 (PSRAM) ---
    T2_Type::ST_FeatureSlot_Aud_t* 		_asyncAudRing;						// 오디오 특징량 링버퍼
    T2_Type::ST_FeatureSlot_Vib_t* 		_asyncVibRing;						// 진동 특징량 링버퍼
    T2_Type::ST_Raw_Accel_t* 			_asyncAccRing;						// 가속도 원시 파형 링버퍼
    T2_Type::ST_Raw_Gyro_t* 			_asyncGyrRing;						// 자이로 원시 파형 링버퍼
    T2_Type::ST_Raw_Audio_t* 			_asyncAudRawRing;					// 오디오 원시 파형 링버퍼

    uint16_t 							_asyncAudHead, _asyncAudTail;		// 오디오 특징량 링버퍼 헤드 및 테일
    uint16_t 							_asyncVibHead, _asyncVibTail;		// 진동 특징량 링버퍼 헤드 및 테일

    // DMA 캐시 정렬 조건 충족 내부 바운스 버퍼 (SRAM/PSRAM 16B 정렬)
    T2_Type::ST_FeatureSlot_Aud_t* 		_bounceAudFeat;						// 오디오 특징량 바운스 버퍼
    T2_Type::ST_FeatureSlot_Vib_t* 		_bounceVibFeat;						// 진동 특징량 바운스 버퍼
    T2_Type::ST_Raw_Accel_t* 			_bounceAcc;							// 가속도 원시 파형 바운스 버퍼
    T2_Type::ST_Raw_Gyro_t* 			_bounceGyr;							// 자이로 원시 파형 바운스 버퍼
    T2_Type::ST_Raw_Audio_t* 			_bounceAudRaw;						// 오디오 원시 파형 바운스 버퍼

    TaskHandle_t 						_hStorageTask;						// 저장소 태스크 핸들

public:
    CL_T2_StorageManager();
    ~CL_T2_StorageManager();

    static CL_T2_StorageManager& getInstance() {
        static CL_T2_StorageManager v_inst;
        return v_inst;
    }

	// 저장소 초기화
    bool init();

    // 세션 열기
    bool openSession(const char* p_prefix, uint64_t p_triggerTimestamp, const T2_Type::ST_TriggerReason_t& p_reason, const char* p_overrideDir = nullptr);
    bool openSession(const char* p_prefix, const char* p_overrideDir = nullptr);

	// 세션 닫기
    void closeSession(const char* p_reason);

	// 오디오 특징량 저장
    bool pushAudioFrame(const T2_Type::ST_FeatureSlot_Aud_t* p_audFeat,
                        const T2_Type::ST_Raw_Audio_t* p_rawAud);

	// 진동 특징량 저장
    bool pushVibFrame(const T2_Type::ST_FeatureSlot_Vib_t* p_vibFeat,
                      const T2_Type::ST_Raw_Accel_t* p_rawAcc,
                      const T2_Type::ST_Raw_Gyro_t* p_rawGyr);

	// 마지막 파일 경로
    const char* getLastPath() const { return _vibBinPath; }

	// 버퍼 플러시
    bool flush();
	// 회전 체크
    void checkRotation();
	// 복구 시도
    bool attemptRecovery();
	// 세션 오픈 여부 확인
    bool isSessionOpen() const { return _sessionOpen; }
	// IO 에러 여부 확인
    bool hasIoError() const { return _ioError; }

    // 프리트리거를 세션 오픈 시 덤프
    void dumpPreTriggerToSession();

    // 노이즈 프로필 저장
    bool saveNoiseProfile(const float* p_profile, size_t p_size);
    // 노이즈 프로필 로드
    bool loadNoiseProfile(float* p_profile, size_t p_size);

    // 락 핸들 얻기
    SemaphoreHandle_t getFsLock() { return _fsLock; }

private:
	// 버퍼 할당
    void _allocateBuffers();
    // 일별 경로 빌드 v2
    void _buildDailyPath_v2(char* p_outPath, size_t p_maxLen, const char* p_ext);
	// 일별 경로 빌드 v1
    void _buildDailyPath_v1(char* p_outPath, size_t p_maxLen, const char* p_ext);

    // 바이너리 헤더 쓰기
    void _writeBinHeader(fs::File&, bool);
    // WAV 헤더 쓰기
    void _writeWavHeader(File& p_file, uint32_t p_dataSize);
    // 파일 사전 할당
    void _preAllocateFile(File& p_file, uint32_t p_bytes);

    // 프리버퍼 플러시
    void _flushPreBufferToRing();
    // 링버퍼 프로세스
    void _processRingIO();
    // 저장소 태스크 프로세스
    static void _storageTaskProc(void* p_param);

    // 인덱스 원자적 저장
    bool _saveIndexAtomic();
    // 인덱스 로드
    bool _loadIndex();
	// 인덱스 아이템 추가
    void _appendIndexItem();
};



