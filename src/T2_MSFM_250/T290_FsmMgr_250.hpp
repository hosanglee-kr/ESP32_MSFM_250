/* ============================================================================
 * File: T290_FsmMgr_250.hpp
 * Summary: 4-Tier 시스템 통합 오케스트레이터 (State Machine)
 * ============================================================================
 * [로직 상태 추적 (Logic State Tracker)]
 * - 유지: Core 0(Capture) / Core 1(Processing) 이중 태스크 구조.
 * - 갱신: 도메인 격리에 따른 개별 acc, gyr, audio raw 풀 및 제어 구조 정합.
 * - 신규: [원칙 준수] PSRAM Tiling을 통한 캐시 미스 차단 및 SIMD 가속 준비.
 * ========================================================================== */
#pragma once

#include "T210_Def_250.hpp"
#include "T215_Type_250.hpp"
#include "T220_CfgMgr_250.hpp"
#include "T230_Sensor_250.hpp"
#include "T240_DspEng_250.hpp"
#include "T245_FeatExtra_250.hpp"
#include "T248_SeqBuild_250.hpp"
#include "T250_Trigger_250.hpp"
#include "T260_Storage_250.hpp"
#include "T270_Commu_250.hpp"
#include "T280_Calibrator_250.hpp"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <atomic>
#include <Arduino.h>

/* ============================================================================
 * [클래스 명세서] PreemptiveSafetyInterlock (하드웨어 래칭)
 * ============================================================================
 * 개요: 시스템 레벨 비상 정지 래칭 및 하드웨어 차단(릴레이) 제어.
 * 목적: 소프트웨어적 복구(Reset)가 불가능한 치명적 오류 발생 시, 즉시 전원 공급을
 *        물리적으로 차단(Fail-Safe)하여 시스템 보호 및 2차 사고 방지.
 * 구조: 영구 오작동(무한 래치) 방지를 위한 '하드웨어 릴레이(보조 전원)' 기반 설계.
 * ========================================================================== */
class PreemptiveSafetyInterlock {
private:
    std::atomic<bool>       _is_emergency_fault;    // 비상 상태 플래그
    std::atomic<uint16_t>   _critical_fault_map;    // 비상 상태 코드 (중복 방지)
    const uint8_t           _relay_pin;             // 하드웨어 릴레이 핀 번호

public:
    // 생성자: 핀 번호를 받아 릴레이를 초기화하고 안전 상태로 설정
    PreemptiveSafetyInterlock(uint8_t p_pin)
        : _is_emergency_fault(false), _critical_fault_map(0), _relay_pin(p_pin) {
        pinMode(_relay_pin, OUTPUT);
        digitalWrite(_relay_pin, HIGH);
    }

    // 하드웨어 차단 트리거: 비상 플래그 설정 및 릴레이 즉시 OFF (LOW 인가)
    void triggerEmergencyFault(uint16_t p_faultBit) {
        _critical_fault_map.fetch_or(p_faultBit);                    // 비트 OR 연산을 통해 오류 코드 누적
        _is_emergency_fault.store(true, std::memory_order_release);  // 비상 상태 플래그 설정
        digitalWrite(_relay_pin, LOW);                               // 릴레이 OFF (전원 차단)
    }

    // 비상 상태 확인 (Lock-Free)
    inline bool checkEmergencyStatus() const {
        return _is_emergency_fault.load(std::memory_order_acquire);  // 비상 상태 확인
    }

    // 오류 코드 확인 (Lock-Free)
    uint16_t getCriticalFaultMap() const {
        return _critical_fault_map.load();                           // 오류 코드 확인
    }

    // 비상 래치 해제: 수동 초기화 시 호출 (외부 로직 연계 필요)
    void clearEmergencyLatch() {
        _critical_fault_map.store(0);                               // 오류 코드 초기화
        _is_emergency_fault.store(false, std::memory_order_release);  // 비상 상태 해제
        digitalWrite(_relay_pin, HIGH);                               // 릴레이 ON (전원 복구)
    }
};

/* ============================================================================
 * [클래스 명세서] SafetyLifecycleManager (소프트웨어 래칭 및 룰 엔진)
 * ============================================================================
 * 개요: 비상 상태(Emergency Fault)의 소프트웨어적 수명주기(Lifecycle) 관리 및
 *        결정론적 복구(Deterministic Recovery) 제어.
 * 목적: 하드웨어 차단(Interlock)과 연계하여 시스템의 '비상-정상' 전환을 제어하고,
 *        재현 불가능한 래치 상태를 방지하며 안전 규정(Safety Rule)을 강제한다.
 * 구조: PreemptiveSafetyInterlock(하드웨어)과 연계된 2단계 래칭 구조.
 * ========================================================================== */
class SafetyLifecycleManager {
private:
    bool                        _is_alarm_latched;  // 소프트웨어 래치 플래그
    PreemptiveSafetyInterlock&  _interlock;         // 하드웨어 차단기 참조

public:
    // 생성자: 하드웨어 차단기와 연계하여 초기화
    SafetyLifecycleManager(PreemptiveSafetyInterlock& p_interlock)
        : _is_alarm_latched(false), _interlock(p_interlock) {}

    // [룰 엔진] 비상 상태 결정 및 하드웨어 래칭 적용
    void evaluateRuleEngine(bool p_isDensityNg, uint16_t p_faultType) {
        if (_is_alarm_latched || _interlock.checkEmergencyStatus()) {  // 이미 래치되었거나 하드웨어 차단 상태면 종료
            return;
        }
        if (p_isDensityNg) {            // 밀도 감지 결과가 NG인 경우
            _is_alarm_latched = true;                       // 소프트웨어 래치
            _interlock.triggerEmergencyFault(p_faultType);  // 하드웨어 차단
        }
    }

    // [수동 리셋] 관리자 권한 기반 래치 해제 절차
    void processManualReset(bool p_isSafeCondition) {
        if (!p_isSafeCondition) {                               // 안전 조건 미충족 시 리셋 거부
            return;
        }
        _is_alarm_latched = false;                              // 소프트웨어 래치 해제
        _interlock.clearEmergencyLatch();                       // 하드웨어 차단 해제
    }

    // 래치 상태 확인
    bool isAlarmLatched() const { return _is_alarm_latched; }
    // 소프트웨어 래치 클리어 (경고 로깅 필요)
    void clearAlarmLatch() { _is_alarm_latched = false; }
};

// 상태 머신 매니저
class CL_T2_FsmManager;
class CL_T2_FsmManager {
private:

    static CL_T2_FsmManager* s_pInstance;   // ISR 역참조용 싱글톤 포인터

    // ISR에서 private 멤버 접근 허용
    friend void T2_90_IMU_watermark_isr();

    CL_T2_SensorEngine      _sensor;        // 센서 엔진
    CL_T2_DspEngine         _dsp;           // DSP 엔진
    CL_T2_FeatureExtractor  _extractor;     // 특징 추출 엔진
    CL_T2_TriggerEngine     _trigger;       // 트리거 엔진
    CL_T2_SequenceBuilder   _seqBuilder;    // 시퀀스 빌더
    CL_T2_StorageManager    _storage;       // 저장소 매니저
    CL_T2_Communicator      _comm;          // 통신 매니저
    CL_T2_Calibrator        _calibrator;    // 캘리브레이터

    // 상태 관리
    portMUX_TYPE            			_stateMux = portMUX_INITIALIZER_UNLOCKED;  // 상태 뮤텍스
    volatile T2_Type::EM_SystemState_t 	_state    = T2_Type::EM_SystemState_t::INIT;  // 시스템 상태

    // 태스크 핸들 및 통신 큐
    TaskHandle_t 			_hImuAcqTask 	= nullptr;  // IMU 획득 태스크 핸들 
    TaskHandle_t 			_hAudioTask 	= nullptr;  // 오디오 태스크 핸들
    TaskHandle_t 			_hVibTask 		= nullptr;  // 진동 태스크 핸들

    // 연산 핫패스용 Internal SRAM 고속 버퍼 (캐시 미스 방지 및 가속)
    float* 								_prcBeamformed 	= nullptr;

    // 텔레메트리 전송용 버퍼 (WebSocket)
    T2_Type::ST_PktTelemetry_t*       	_pktTele 		= nullptr;  // 텔레메트리 패킷
    T2_Type::ST_PktWaveformAudio_t*   	_pktWaveAudio 	= nullptr;  // 오디오 파형 패킷
    T2_Type::ST_PktWaveformAccel_t*   	_pktWaveAccel 	= nullptr;  // 가속도 파형 패킷
    T2_Type::ST_PktWaveformGyro_t*    	_pktWaveGyro 	= nullptr;  // 자이로 파형 패킷
    T2_Type::ST_PktSpectrum_t*        	_pktSpec 		= nullptr;  // 스펙트럼 패킷
    T2_Type::ST_PktSequence_t*        	_pktSeq 		= nullptr;  // 시퀀스 패킷

    // 런타임 제어 및 스트리밍 누적기
    uint8_t  							_currentTrial 	= 0;        // 현재 트라이얼 횟수
    uint32_t 							_lastTick 		= 0;        // 마지막 틱
    bool     							_isManualRecording = false; // 수동 녹음 여부

    // 스트리밍 주율 제어 (Hz)
    uint8_t  							_telemetryHz = 10;          // 텔레메트리 주율
    uint8_t  							_waveformHz = 0;            // 파형 주율
    uint8_t  							_spectrumHz = 0;            // 스펙트럼 주율
    uint8_t  							_sequenceHz = 0;            // 시퀀스 주율
    uint8_t  							_accTele = 0, _accWave = 0, _accSpec = 0, _accSeq = 0; // 누적 카운터(주율)

    // 비동기 세션 제어 및 Graceful Close용
    QueueHandle_t                   _qSessionCmd = nullptr;     // 세션 제어용 큐
    uint8_t                         _stopReasonCmd = 0;           // 중지 이유 코드

    T2_Type::ST_SharedContext_t*    _sharedCtx = nullptr;       // 공유 메모리 포인터    
    
    // 비동기 텐서 조립을 위한 정적 패딩 버퍼 (VLA 방지)        
    float _flatTensor[T2_Def::AI::Tensor::MFCC_DIM_DEF];

    // 프리엠프티브 하드웨어 차단 및 수명주기 래치 관리자 (Tier 4)
    PreemptiveSafetyInterlock* _interlock = nullptr;       // 하드웨어 차단기
    SafetyLifecycleManager*    _safetyManager = nullptr;   // 소프트웨어 래치 관리자

private:
    // 내부 메서드
    CL_T2_FsmManager();
    ~CL_T2_FsmManager();

    // IMU 획득 태스크
    static void _imuAcqTask(void* p_param);

    // 오디오 처리 태스크
    static void _audioProcessTask(void* p_param);
    // 진동 처리 태스크
    static void _vibProcessTask(void* p_param);

    // 스트리밍 브로드캐스트
    void _broadcastStreams(const T2_Type::ST_FeatureSlot_Aud_t& p_audSlot,
                           const T2_Type::ST_FeatureSlot_Vib_t& p_vibSlot,
                           const float* p_rawAudL, const float* p_rawAudR);

    // 그레이스풀 클로즈 확인
    void _checkGracefulClose();

    // 트리거 결과 처리
    void _handleTriggerResult(T2_Type::EM_DetectionResult_t p_res,
                              const T2_Type::ST_FeatureSlot_Aud_t& p_audSlot,
                              const T2_Type::ST_FeatureSlot_Vib_t& p_vibSlot);


public:
    // 싱글톤 접근
    static CL_T2_FsmManager& getInstance() {
        static CL_T2_FsmManager v_inst;
        s_pInstance = &v_inst;   // ISR이 접근할 수 있도록 주소 저장
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

    // 상태 설정 (뮤텍스 보호)
    void setState(T2_Type::EM_SystemState_t p_newState);

    // DSP 필터 재장전
    void reloadDspFilters();

    // 수동 복구 및 파이프라인 가드 연쇄 해제 오케스트레이션
    void processManualReset();

    // OTA 준비
    void prepareForOta();

    // OTA 실패 시 복구
    void resumeFromOtaFailure();

    // 텔레메트리 페이로드 브로드캐스트
    void broadcastTelemetryPayload(const T2_Type::ST_FeatureSlot_Vib_t& p_vib, const T2_Type::ST_FeatureSlot_Aud_t& p_aud);
};

