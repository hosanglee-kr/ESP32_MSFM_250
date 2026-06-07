/* ============================================================================
 * [SMEA-100 핵심 구현 원칙 및 AI 셀프 회고 바이블 (v015 최종 고도화)]
 * 1. [메모리/병목 방어]: FSM 풀(100개)과 이중 큐를 통한 10ms 윈도우 방어.
 * 2. [네이밍 컨벤션]: private(_), 매개변수(p_), 로컬변수(v_) 엄수.
 * 3. [시계열 보호]: 64비트 Epoch 시간과 esp_timer 상대시간 분리 적용.
 * * 4. [v015 무결점 및 멀티코어 방어 아키텍처 탑재 내역]:
 * - 연산 최적화 (Zero Cache-Miss): 무거운 PSRAM 데이터를 DSP 연산 직전 
 * 초고속 Internal SRAM(_prcBuf)으로 단숨에 블록 복사하여 CPU 쓰래싱 원천 차단.
 * - 스택 다이어트 (Stack Diet): _captureTask 및 _processingTask의 거대 로컬 배열을
 * 힙 기반 SRAM 포인터로 교체하여 12KB 스택 메모리 해방.
 * - 원자적 상태 제어 (Thread-Safe): Spinlock(_stateMux)을 도입하여 LwIP 콜백과 
 * Core 루프 간의 상태 천이 충돌을 100% 방어.
 * - 영구 락업 방어 (Queue Full Retry): FAT32 I/O 큐가 꽉 찼을 경우, 명령을 
 * 증발시키지 않고 다음 루프에서 재시도하여 스토리지 식물인간화 방지.
 * - 우아한 종료 (Graceful Close): _checkGracefulClose()를 통해 파이프라인 
 * 잔류 찌꺼기 데이터를 완벽히 비운 뒤에만 상태를 천이하고 세션을 닫음.
 * - 텐서 데이터 레이스 방어: Core 0 I2S Drop 시 텐서를 직접 깨지 않고, 
 * 위임 깃발(_reqSeqReset)을 세워 Core 1이 안전한 타이밍에 리셋하도록 유도.
 * - God-Function 해체: 거대했던 태스크 함수들을 4개의 직관적인 헬퍼 함수로 분해.
 * ============================================================================
 * File: T450_FsmMgr_015.hpp
 * Summary: FSM Orchestrator & Multi-task Coordinator
 * ========================================================================== 
 */
#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "T410_Def_013.hpp"
#include "T415_ConfigMgr_013.hpp" 
#include "T430_DspEng_013.hpp"
#include "T440_FeatExtra_013.hpp"
#include "T480_MicEng_013.hpp" 
#include "T460_Storage_013.hpp"
#include "T445_SeqBd_013.hpp"   
#include "T470_Commu_014.hpp" 
#include "T442_Calibrator_013.hpp"

class T450_FsmManager {
public:
    // 비동기 세션 I/O 오프로딩 및 캘리브레이션 락업 방어용 명령 체계
    enum class AsyncSessionCmd : uint8_t {
        OPEN_AUTO = 1,
        OPEN_MANUAL = 2,
        OPEN_CALIB_MAN = 3,
        OPEN_CALIB_AUTO = 4,
        CLOSE_NORMAL = 5,
        CLOSE_FORCE_DROP = 6,
        CLOSE_CALIB_DONE = 7,   // 10초 채운 정상 완료 (연산 수행)
        CLOSE_CALIB_ABORT = 8,  // 강제 취소 (연산 생략)
        CLOSE_REBOOT = 9,
        CLOSE_MANUAL = 10
    };

private:
    // (멀티코어 안전성 보장용 스핀락)
    portMUX_TYPE _stateMux = portMUX_INITIALIZER_UNLOCKED; 
    volatile SystemState _systemState = SystemState::INIT;

    TaskHandle_t _hCaptureTask = nullptr;
    TaskHandle_t _hProcessTask = nullptr;

    QueueHandle_t _qFreeSlotIdx = nullptr;
    QueueHandle_t _qReadySlotIdx = nullptr;
    
    // 비동기 I/O 오프로딩 큐 (사이즈 20 확장)
    QueueHandle_t _qSessionCmd = nullptr; 

    T480_MicEngine           _micEngine;
    T430_DspEngine           _dspEngine;
    T440_FeatureExtractor    _extractor;
    T460_StorageManager      _storage;
    T445_SequenceBuilder     _seqBuilder;
    T470_Communicator        _communicator;
    T442_Calibrator          _calibrator;

    SmeaType::FeatureSlot* _featurePool = nullptr;
    SmeaType::RawDataSlot* _rawPool = nullptr;
    
    // 거대 스택 오버플로우 차단용 힙 포인터 (Zero-copy 송출용)
    SmeaType::PktTelemetry* _txTelemetryBuf = nullptr;
    SmeaType::PktWaveform* _txWaveformBuf  = nullptr;
    SmeaType::PktSpectrum* _txSpectrumBuf  = nullptr;

    volatile bool _isrTriggerActive = false;

    // 절대 역행하지 않는 esp_timer 틱 사용 (ms 단위 저장)
    volatile uint32_t _recordStartTick = 0;
    volatile bool _isManualRecording = false;
    
    // 우아한 종료 사유 캐싱 플래그
    volatile uint8_t _stopReasonCmd = 0; 
    
    // 코어 간 Data Race 방어용 volatile 제어 변수들
    volatile uint8_t _currentTrialNo = 0; 
    volatile uint32_t _readyStartSec = 0;
    volatile uint32_t _dropCount = 0;
    volatile bool _reqSeqReset = false; // Core 0 -> Core 1 텐서 리셋 위임 깃발

    // 스트림 스로틀링(Throttling) 제어 변수 및 누산기
    volatile uint8_t _telemetryHz = 10;
    volatile uint8_t _spectrumHz  = 0;
    volatile uint8_t _waveformHz  = 0;
    uint32_t         _frameCount  = 0;
    uint16_t         _accTele     = 0;
    uint16_t         _accWave     = 0;
    uint16_t         _accSpec     = 0;
    
    // PSRAM 캐시 미스 완벽 방어 및 스택 다이어트를 위한 Internal SRAM 포인터
    float* _capBufL = nullptr;
    float* _capBufR = nullptr;
    float* _prcBufL = nullptr;
    float* _prcBufR = nullptr;
    float* _prcBeamformed = nullptr;

public:
    static T450_FsmManager& getInstance() {
        static T450_FsmManager v_instance;
        return v_instance;
    }

    void begin();
    void setSystemState(SystemState p_nextState);
    void handleExternalTrigger(bool p_isActive);

    SystemState getCurrentState() const { return _systemState; }
    void runMaintenanceTask();
    void dispatchCommand(SystemCommand p_cmd);
    
    // 원자성을 완벽히 보장하는 종료 사유 예약 함수
    void requestStop(uint8_t p_reasonCmd, bool p_forceOverride = false) {
        portENTER_CRITICAL(&_stateMux);
        if (_stopReasonCmd == 0 || p_forceOverride) {
            _stopReasonCmd = p_reasonCmd;
        }
        portEXIT_CRITICAL(&_stateMux);
    }
    
    // Web API 연동형 스트림 송출 주기 설정
    void setStreamFrequencies(uint8_t p_teleHz, uint8_t p_specHz, uint8_t p_waveHz) {
        _telemetryHz = p_teleHz;
        _spectrumHz = p_specHz;
        _waveformHz = p_waveHz;
    }

private:
    T450_FsmManager() = default;

    static void _captureTask(void* p_param);
    static void _processingTask(void* p_param);

    DetectionResult _runHybridDecision(uint8_t p_slotIdx);

    // _captureTask 세부 분리 헬퍼 함수
    void _handleMicStateTransition();
    void _handleTriggerLogic();
    void _acquireAndQueueData(SystemState& p_lastAcqState, float* p_windowL, float* p_windowR);

    // _processingTask 세부 분리 헬퍼 함수
    void _handleCalibratingSlot(SmeaType::FeatureSlot& p_slot, SmeaType::RawDataSlot& p_raw);
    void _handleNormalSlot(uint8_t p_slotIdx, SmeaType::FeatureSlot& p_slot, SmeaType::RawDataSlot& p_raw, float* p_beamformed);
    void _broadcastStreams(SmeaType::FeatureSlot& p_slot, float* p_beamformed);
    void _checkGracefulClose();
};
