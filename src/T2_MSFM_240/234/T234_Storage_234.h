/* ============================================================================
 * File: T234_Storage_234.h
 * Summary: Data Logging & Storage Management Engine
 * Description: Zero-Copy DMA, 이벤트 트래킹, 인덱스 로테이션
 * ========================================================================== */
#pragma once

#include <FS.h>
#include <LittleFS.h>
#include <SD_MMC.h>
#include <freertos/FreeRTOS.h> 
#include <freertos/semphr.h>   

#include "T210_Def_234.h"

class CL_T20_StorageService {
   public:
	CL_T20_StorageService();
	~CL_T20_StorageService();

	// 스토리지 마운트 및 초기화
	bool		begin(const ST_T20_SdmmcProfile_t& profile);
	
	void        setConfig(const ST_T20_Config_t& cfg); 

	// 설정 구조체를 직접 전달받아 내부에서 헤더와 파일명을 자동 생성
	// 파일명 접두어(prefix)를 받아 Manual/Trigger 생성 파일을 분리합니다.
	// 호환성을 위해 기본값은 "trg"로 설정합니다.
	bool		openSession(const ST_T20_Config_t& cfg, const char* prefix = "trg");
	void		closeSession(const char* reason = "end_normal");

	// Zero-Copy DMA 슬롯 기반 고속 기록
	bool		pushVector(const ST_T20_FeatureVector_t* p_vec);
	// Raw 파형 저장
	bool pushRaw(const float* p_raw_x, const float* p_raw_y, const float* p_raw_z, uint16_t len, uint8_t active_axes);

	// 타임아웃 및 강제 플러시
	bool		flush();
	void		checkIdleFlush();

	// 용량/시간 기반 로테이션 검사
	void		checkRotation();

	// 상태 및 감사(Audit) 트래킹 API
	void		writeEvent(const char* event_msg);
	void		setLastError(const char* err_msg);
	const char* getLastError() const {
		return _last_error;
	}

	bool isOpen() const {
		return _session_open;
	}
	uint32_t getRecordCount() const {
		return _record_count;
	}

   private:
	bool _commitSlot(uint8_t slot_idx);
	void _handleRotation();
	
	// 로테이션 붕괴를 막기 위해 인덱스 추가와 파일 저장을 분리
	void _appendIndexItem(); 
	bool _writeIndexFile();
	bool _loadIndexJson();
	
	// DMA 슬롯 저장 전용 내부 함수 및 pretrigger 버퍼 관리
    bool _pushToDma(const ST_T20_FeatureVector_t* p_vec);
    void _flushPreBuffer();
    void _allocatePreBuffer();

   private:
	EM_T20_StorageBackend_t	  _backend;
	File					  _active_file;
	File					  _raw_file;  // Raw 데이터 기록용 파일 핸들

	bool					  _session_open;
	bool                      _io_error; // SD 탈거 시 무한 I/O 루프 마비 차단용 플래그
	
	uint32_t				  _record_count;
	char					  _active_path[128];
	char                      _active_raw_path[128]; // Raw 로테이션 누락 방지용 트래킹 변수
	char					  _last_error[128];
    char                      _current_prefix[16]; // 방어 7: 출처 변질 차단을 위한 현재 세션 접두어 트래킹



	ST_T20_Config_t			  _current_cfg;		  // 현재 세션의 설정값
	uint32_t				  _session_start_ms;  // 세션 시작 시간 (시간 로테이션용)
	uint32_t				  _written_bytes;	  // 기록된 바이트 수 (용량 로테이션용)

	// --- DMA 버퍼링 ---
	static constexpr uint8_t  DMA_SLOT_COUNT = 3;
	static constexpr uint32_t DMA_SLOT_BYTES = 4096;

	alignas(32) uint8_t _dma_slots[DMA_SLOT_COUNT][DMA_SLOT_BYTES];
	uint16_t				  _dma_slot_used[DMA_SLOT_COUNT];
	uint8_t					  _dma_active_slot;

	// --- 배치 및 플러시 조건 ---
	uint16_t				  _batch_count;
	uint16_t				  _watermark_high;
	uint32_t				  _last_push_ms;
	uint32_t				  _idle_flush_ms;

	// --- 파일 로테이션 인덱스 ---
	static constexpr uint16_t MAX_ROTATE_LIST = 16;
	struct ST_IndexItem {
		char	 path[128];
		char     raw_path[128]; // 파형 파일 동반 삭제를 위한 트래킹
		uint32_t size_bytes;
		uint32_t created_ms;
		uint32_t record_count;
	} _index_items[MAX_ROTATE_LIST];

	uint16_t _index_count;
	uint16_t _rotate_keep_max;
	
	SemaphoreHandle_t _lock; // 다중 스레드 동시 접근 방어를 위한 재귀적 잠금 장치
	
	// pretrigger 링버퍼 변수
    ST_T20_FeatureVector_t* _pre_buf = nullptr;
    uint16_t                _pre_capacity = 0;
    uint16_t                _pre_head = 0;
    uint16_t                _pre_count = 0;
    
    uint32_t _boot_file_seq = 0;
    uint16_t _rotation_sub_seq = 0;


};


    