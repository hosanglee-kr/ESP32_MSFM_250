/* ============================================================================
 * File: T290_FsmMgr_248.hpp
 * Summary: v245 4-Tier 시스템 통합 오케스트레이터 (State Machine)
 * ============================================================================
 * [로직 상태 추적 (Logic State Tracker)]
 * - 유지: Core 0(Capture) / Core 1(Processing) 이중 태스크 구조.
 * - 갱신: v245 도메인 격리에 따른 개별 acc, gyr, audio raw 풀 및 제어 구조 정합.
 * - 신규: [원칙 준수] PSRAM Tiling을 통한 캐시 미스 차단 및 SIMD 가속 준비.
 * ========================================================================== */
#pragma once

#include "T210_Def_248.hpp"
#include "T215_Type_248.hpp"
#include "T220_CfgMgr_248.hpp"
#include "T230_Sensor_248.hpp"
#include "T240_DspEng_248.hpp"
#include "T245_FeatExtra_248.hpp"
#include "T248_SeqBuild_248.hpp"
#include "T250_TriggerEng_248.hpp"
#include "T260_Storage_248.hpp"
#include "T270_Commu_248.hpp"
#include "T280_Calibrator_248.hpp"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <atomic>
#include <Arduino.h>

class PreemptiveSafetyInterlock {
private:
    std::atomic<bool> _is_emergency_fault;
    std::atomic<uint16_t> _critical_fault_map;
    const uint8_t _relay_pin;

public:
    PreemptiveSafetyInterlock(uint8_t pin)
        : _is_emergency_fault(false), _critical_fault_map(0), _relay_pin(pin) {
        pinMode(_relay_pin, OUTPUT);
        digitalWrite(_relay_pin, HIGH); 
    }

    void triggerEmergencyFault(uint16_t fault_bit) {
        _critical_fault_map.fetch_or(fault_bit);
        _is_emergency_fault.store(true, std::memory_order_release);
        digitalWrite(_relay_pin, LOW); 
    }

    inline bool checkEmergencyStatus() const {
        return _is_emergency_fault.load(std::memory_order_acquire);
    }

    uint16_t getCriticalFaultMap() const {
        return _critical_fault_map.load();
    }

    void clearEmergencyLatch() {
        _critical_fault_map.store(0);
        _is_emergency_fault.store(false, std::memory_order_release);
        digitalWrite(_relay_pin, HIGH);
    }
};

class SafetyLifecycleManager {
private:
    bool _is_alarm_latched;
    PreemptiveSafetyInterlock& _interlock;

public:
    SafetyLifecycleManager(PreemptiveSafetyInterlock& interlock)
        : _is_alarm_latched(false), _interlock(interlock) {}

    void evaluateRuleEngine(bool is_density_ng, uint16_t fault_type) {
        if (_is_alarm_latched || _interlock.checkEmergencyStatus()) {
            return;
        }
        if (is_density_ng) {
            _is_alarm_latched = true;
            _interlock.triggerEmergencyFault(fault_type);
        }
    }

    void processManualReset();

    bool isAlarmLatched() const { return _is_alarm_latched; }
    void clearAlarmLatch() { _is_alarm_latched = false; }
};

class CL_T2_FsmManager;
class CL_T2_FsmManager {
private:
    CL_T2_SensorEngine      _sensor;
    CL_T2_DspEngine         _dsp;
    CL_T2_FeatureExtractor  _extractor;
    CL_T2_TriggerEngine     _trigger;
    CL_T2_SequenceBuilder   _seqBuilder;
    CL_T2_StorageManager    _storage;
    CL_T2_Communicator      _comm;
    CL_T2_Calibrator        _calibrator;

    // 상태 관리
    portMUX_TYPE            			_stateMux = portMUX_INITIALIZER_UNLOCKED;
    volatile T2_Type::EM_SystemState_t 	_state    = T2_Type::EM_SystemState_t::INIT;

    // 태스크 핸들 및 통신 큐
    TaskHandle_t 			_hImuAcqTask 	= nullptr;
    TaskHandle_t 			_hAudioTask 	= nullptr;
    TaskHandle_t 			_hVibTask 		= nullptr;

    // 연산 핫패스용 Internal SRAM 고속 버퍼 (캐시 미스 방지 및 가속)
    float* 								_prcBeamformed 	= nullptr;

    // 텔레메트리 전송용 버퍼 (WebSocket)
    T2_Type::ST_PktTelemetry_t*       	_pktTele 		= nullptr;
    T2_Type::ST_PktWaveformAudio_t*   	_pktWaveAudio 	= nullptr;
    T2_Type::ST_PktWaveformAccel_t*   	_pktWaveAccel 	= nullptr;
    T2_Type::ST_PktWaveformGyro_t*    	_pktWaveGyro 	= nullptr;
    T2_Type::ST_PktSpectrum_t*        	_pktSpec 		= nullptr;
    T2_Type::ST_PktSequence_t*        	_pktSeq 		= nullptr;

    // 런타임 제어 및 스트리밍 누적기
    uint8_t  							_currentTrial 	= 0;
    uint32_t 							_lastTick 		= 0;
    bool     							_isManualRecording = false;

    // 스트리밍 주율 제어 (Hz)
    uint8_t  							_telemetryHz = 10;
    uint8_t  							_waveformHz = 0;
    uint8_t  							_spectrumHz = 0;
    uint8_t  							_sequenceHz = 0;
    uint8_t  							_accTele = 0, _accWave = 0, _accSpec = 0, _accSeq = 0;

    // 비동기 세션 제어 및 Graceful Close용
    QueueHandle_t _qSessionCmd = nullptr;
    uint8_t       _stopReasonCmd = 0;

    T2_Type::ST_SharedContext_t* _sharedCtx = nullptr; // 공유 메모리 포인터
    // 비동기 텐서 조립을 위한 정적 패딩 버퍼 (VLA 방지)
    float _flatTensor[T2_Def::AI::Tensor::MFCC_DIM_DEF];

    // 프리엠프티브 하드웨어 차단 및 수명주기 래치 관리자 (Tier 4)
    PreemptiveSafetyInterlock* _interlock = nullptr;
    SafetyLifecycleManager*    _safetyManager = nullptr;

private:
    // 내부 메서드
    CL_T2_FsmManager();
    ~CL_T2_FsmManager();

    static void _imuAcqTask(void* p_param);

    static void _audioProcessTask(void* p_param);
    static void _vibProcessTask(void* p_param);


    // 통합 슬롯 대신 분리된 오디오/진동 슬롯을 참조하도록 변경
    void _broadcastStreams(const T2_Type::ST_FeatureSlot_Aud_t& p_audSlot,
                           const T2_Type::ST_FeatureSlot_Vib_t& p_vibSlot,
                           const float* p_rawAudL, const float* p_rawAudR);


    void _checkGracefulClose();

    void _handleTriggerResult(T2_Type::EM_DetectionResult_t p_res,
                              const T2_Type::ST_FeatureSlot_Aud_t& p_audSlot,
                              const T2_Type::ST_FeatureSlot_Vib_t& p_vibSlot);


public:
    static CL_T2_FsmManager& getInstance() {
        static CL_T2_FsmManager v_inst;
        return v_inst;
    }

    // 시스템 초기화 및 하위 엔진 기동
    bool init();

    // 메인 루프에서 호출되는 관리 태스크 (NTP 체크, 지연 쓰기 등)
    void runMaintenance();

    // 전역 명령 분배기 (Web/MQTT/Key 등으로부터 유입)
    void dispatchCommand(T2_Type::EM_SystemCommand_t p_cmd);

    // 상태 조회
    T2_Type::EM_SystemState_t getState() const { return _state; }
    void setState(T2_Type::EM_SystemState_t p_newState);

    void reloadDspFilters();

    // 수동 복구 및 파이프라인 가드 연쇄 해제 오케스트레이션
    void processManualReset();
};

