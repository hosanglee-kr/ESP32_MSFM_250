/* ============================================================================
 * File: T290_FsmMgr_245.hpp
 * Summary: 4-Tier 시스템 통합 오케스트레이터 (State Machine)
 * ============================================================================
 * [핵심 아키텍처]
 * - Core 0 (센서 데이터 수집: Capture Task) 및 Core 1 (신호 처리: Processing Task) 이중 태스크 구조
 * - PSRAM Tiling을 사용하여 캐시 미스를 방지하고 고속 파이프라인 보장
 * ========================================================================== */
#pragma once

#include "T210_Def_245.hpp"
#include "T215_Type_245.hpp"
#include "T220_CfgMgr_245.hpp"
#include "T230_Sensor_245.hpp"
#include "T240_DspEng_245.hpp"
#include "T245_FeatExtra_245.hpp"
#include "T248_SeqBuild_245.hpp"
#include "T250_TriggerEng_245.hpp"
#include "T260_Storage_245.hpp"
#include "T270_Commu_245.hpp"
#include "T280_Calibrator_245.hpp"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

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
    portMUX_TYPE            _stateMux = portMUX_INITIALIZER_UNLOCKED;
    volatile T2_Type::EM_SystemState_t _state = T2_Type::EM_SystemState_t::INIT;

    // 태스크 핸들 및 통신 큐
    TaskHandle_t            _hCaptureTask = nullptr;
    TaskHandle_t            _hProcessTask = nullptr;
    QueueHandle_t           _qReadyIdx = nullptr;
    QueueHandle_t           _qFreeIdx = nullptr;

    // PSRAM 데이터 풀 (Zero-copy 파이프라인 - v245 도메인 격리화)
    T2_Type::ST_UnifiedFeatureSlot_t* _featurePool = nullptr;
    T2_Type::ST_Raw_Accel_t*       _accelPool = nullptr;
    T2_Type::ST_Raw_Gyro_t*        _gyroPool = nullptr;
    T2_Type::ST_Raw_Audio_t*       _audioPool = nullptr;

    // 연산 핫패스용 Internal SRAM 고속 버퍼 (캐시 미스 방지 및 가속)
    float* _capBufL = nullptr;
    float* _capBufR = nullptr;
    float* _prcBufL = nullptr;
    float* _prcBufR = nullptr;
    float* _prcBeamformed = nullptr;

    // 텔레메트리 전송용 버퍼 (WebSocket)
    T2_Type::ST_PktTelemetry_t*       _pktTele = nullptr;
    T2_Type::ST_PktWaveformAudio_t*   _pktWaveAudio = nullptr;
    T2_Type::ST_PktWaveformAccel_t*   _pktWaveAccel = nullptr;
    T2_Type::ST_PktWaveformGyro_t*    _pktWaveGyro = nullptr;
    T2_Type::ST_PktSpectrum_t*        _pktSpec = nullptr;
    T2_Type::ST_PktSequence_t*        _pktSeq = nullptr;

    // 런타임 제어 및 스트리밍 누적기
    uint8_t  _currentTrial = 0;
    uint32_t _lastTick = 0;
    bool     _isManualRecording = false;

    // 스트리밍 주율 제어 (Hz)
    uint8_t  _telemetryHz = 10;
    uint8_t  _waveformHz = 0;
    uint8_t  _spectrumHz = 0;
    uint8_t  _sequenceHz = 0;
    uint8_t  _accTele = 0, _accWave = 0, _accSpec = 0, _accSeq = 0;

    // 비동기 세션 제어 및 Graceful Close용
    QueueHandle_t _qSessionCmd = nullptr;
    uint8_t       _stopReasonCmd = 0;

    // 내부 메서드
    CL_T2_FsmManager();
    ~CL_T2_FsmManager();

    static void _captureTask(void* p_param);
    static void _processTask(void* p_param);

    void _broadcastStreams(const T2_Type::ST_UnifiedFeatureSlot_t& p_slot,
                           const T2_Type::ST_Raw_Accel_t& p_rawAcc,
                           const T2_Type::ST_Raw_Gyro_t& p_rawGyr,
                           const T2_Type::ST_Raw_Audio_t& p_rawAud);
    void _checkGracefulClose();
    void _handleTriggerResult(T2_Type::EM_DetectionResult_t p_res,
                              const T2_Type::ST_UnifiedFeatureSlot_t& p_slot,
                              const T2_Type::ST_Raw_Accel_t* p_rawAcc,
                              const T2_Type::ST_Raw_Gyro_t* p_rawGyr,
                              const T2_Type::ST_Raw_Audio_t* p_rawAud);

public:
    static CL_T2_FsmManager& getInstance() {
        static CL_T2_FsmManager v_inst;
        return v_inst;
    }

    /**
     * @brief 시스템 초기화 및 하위 엔진 기동
     */
    bool init();

    /**
     * @brief 메인 루프에서 호출되는 관리 태스크 (NTP 체크, 지연 쓰기 등)
     */
    void runMaintenance();

    /**
     * @brief 전역 명령 분배기 (Web/MQTT/Key 등으로부터 유입)
     */
    void dispatchCommand(T2_Type::EM_SystemCommand_t p_cmd);

    // 상태 조회
    T2_Type::EM_SystemState_t getState() const { return _state; }
    void setState(T2_Type::EM_SystemState_t p_newState);
};
