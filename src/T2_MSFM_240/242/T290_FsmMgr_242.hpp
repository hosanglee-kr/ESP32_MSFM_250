/* ============================================================================
 * File: T250_FsmMgr_245.hpp
 * Summary: Multimodal FSM Orchestrator (Spinlock & Memory Tiling)
 * ============================================================================
 * [축소/누락 방어 점검 (Omission Defense)]
 * 1. Spinlock 원자적 제어: _stateMux를 사용하여 외부 API 스레드와 루프 간
 *    상태 전이 충돌(Data Race)을 막음.
 * 2. Memory Tiling 포인터: PSRAM의 RawDataSlot 전체를 직접 연산하지 않고,
 *    Internal SRAM인 _prcBufL/R/X/Y/Z 로 단숨에 복사하여 Cache Miss 100% 방지.
 * 3. Graceful Close 예약: _stopReasonCmd를 이용해 I/O 큐가 찰 경우 재시도하며,
 *    찌꺼기 데이터가 모두 비워진 후에만 세션을 닫아 이벤트 여진 데이터 보호.
 * ========================================================================== */

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <SPI.h>

#include "T210_Def_242.hpp"
#include "T215_CfgMgr_242.hpp"
#include "T230_Sensor_242.hpp"
#include "T240_DspEng_242.hpp"
#include "T245_FeatExtra_242.hpp"
#include "T260_Storage_242.hpp"
#include "T248_SeqBuild_242.hpp"
#include "T280_Calibrator_242.hpp"
#include "T230_Sensor_242.hpp"
#include "T270_Commu_242.hpp"

// CPP에서 사용 중인 비동기 세션 커맨드 정의 복원
enum class AsyncSessionCmd : uint8_t {
    OPEN_AUTO = 1, OPEN_MANUAL, OPEN_CALIB_MAN,
    CLOSE_NORMAL, CLOSE_MANUAL, CLOSE_CALIB_DONE, CLOSE_CALIB_ABORT
};

class CL_T2_FsmManager {
private:
	portMUX_TYPE					 _stateMux		= portMUX_INITIALIZER_UNLOCKED;
	volatile T2_Type::SystemState _systemState = T2_Type::SystemState::INIT;

	TaskHandle_t					 _hCaptureTask	= nullptr;
	TaskHandle_t					 _hProcessTask	= nullptr;

	QueueHandle_t					 _qFreeSlotIdx	= nullptr;
	QueueHandle_t					 _qReadySlotIdx = nullptr;
	QueueHandle_t					 _qSessionCmd	= nullptr;	// A-DSE I/O 위임 큐

	// [교정] 모든 하위 융합 엔진 멤버 선언 (이름 동기화)
    CL_T2_SensorEngine               _sensor;
    CL_T2_DspEngine                  _dsp;
    CL_T2_FeatureExtractor           _extractor;
    CL_T2_StorageManager             _storage;
    CL_T2_SequenceBuilder            _seqBuilder;
    CL_T2_Calibrator                 _calibrator;
    CL_T2_Communicator               _comm;

	// --- PSRAM 거대 데이터 풀 (Zero-Allocation) ---
	T2_Type::UnifiedFeatureSlot*     _featurePool = nullptr;
    T2_Type::UnifiedRawChunk* 	     _rawPool = nullptr;


	// CPP에서 접근하는 전송 버퍼 및 포인터
    float*                          _prcBeamformed = nullptr;
    T2_Type::PktTelemetry*          _txTelemetryBuf = nullptr;
    T2_Type::PktWaveform*           _txWaveformBuf = nullptr;
    T2_Type::PktSpectrum*           _txSpectrumBuf = nullptr;

	// Core 0 수집 스택 오버플로우 방어용 PSRAM 스크래치 버퍼
    float*                          _tmpAudL = nullptr;
    float*                          _tmpAudR = nullptr;

	// CPP 로직에서 요구하는 상태 변수들
	volatile uint8_t				 _currentTrialNo	= 0;
	volatile uint32_t				 _recordStartTick	= 0;
	volatile uint32_t				 _readyStartTick	= 0;
	volatile uint8_t				 _stopReasonCmd		= 0;
	volatile bool					 _reqSeqReset		= false;
	volatile bool					 _isrTriggerActive	= false;
	volatile bool					 _isManualRecording = false;
	uint32_t						 _dropCount			= 0;

	// 통신 스트림 제어 변수
	uint8_t							 _accTele = 0, _telemetryHz = 10;
	uint8_t							 _accWave = 0, _waveformHz = 0;

public:
    static CL_T2_FsmManager& getInstance() {
        static CL_T2_FsmManager v_instance;
        return v_instance;
    }

    void begin();
    void setSystemState(T2_Type::SystemState p_nextState);
    T2_Type::SystemState getCurrentState() const { return _systemState; }

    void dispatchCommand(T2_Type::SystemCommand p_cmd);
    void handleExternalTrigger(bool p_isActive);
    void runMaintenanceTask();

    // 스트림 전송 주기 설정
    void setStreamFreq(uint8_t t_hz, uint8_t s_hz, uint8_t w_hz) {
        _telemetryHz = t_hz; /* _spectrumHz = s_hz; */ _waveformHz = w_hz;
    }

    void requestStop(uint8_t p_reasonCmd, bool p_forceOverride = false) {
        portENTER_CRITICAL(&_stateMux);
        if (_stopReasonCmd == 0 || p_forceOverride) _stopReasonCmd = p_reasonCmd;
        portEXIT_CRITICAL(&_stateMux);
    }

private:
    CL_T2_FsmManager();
    ~CL_T2_FsmManager();

    static void _captureTask(void* p_param);
    static void _processingTask(void* p_param);

    void _handleSensorStateSync();
    void _handleTriggerLogic();
    void _acquireData(uint8_t p_idx);

    void _handleCalibratingSlot(T2_Type::UnifiedFeatureSlot& p_slot, T2_Type::UnifiedRawChunk& p_raw);
    void _handleNormalSlot(uint8_t p_slotIdx, T2_Type::UnifiedFeatureSlot& p_slot, T2_Type::UnifiedRawChunk& p_raw);
    T2_Type::DetectionResult _runHybridDecision(const T2_Type::UnifiedFeatureSlot& p_slot);
    void _broadcastStreams(const T2_Type::UnifiedFeatureSlot& p_slot, const T2_Type::UnifiedRawChunk& p_raw);
    void _checkGracefulClose();
};

