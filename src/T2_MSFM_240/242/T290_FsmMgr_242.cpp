/* ============================================================================
 * File: T290_FsmMgr_242.cpp
 * Summary: T240 시스템 오케스트레이터 구현부 (Full Integration)
 * ============================================================================
 * [로직 상태 추적 (Logic State Tracker)]
 * - 유지: FSM 기반 상태 천이 및 10ms 단위 이중 큐 인덱스 통신.
 * - 갱신: [교정] Sequence Builder 및 Calibrator 누락 완벽 복원 및 FSM 결합.
 * - 신규: [원칙 준수] 하드웨어 Watchdog 명시적 리셋 및 I2S 데이터 드랍 방어벽 탑재.
 * ========================================================================== */

#include "T290_FsmMgr_242.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include <cstring>
#include <sys/time.h>

static const char* TAG = "T247_FSM";

void T240_DispatchCommand(T2_Type::SystemCommand p_cmd) { CL_T2_FsmManager::getInstance().dispatchCommand(p_cmd); }
uint8_t T240_GetCurrentState() { return (uint8_t)CL_T2_FsmManager::getInstance().getCurrentState(); }
void T240_SetStreamFrequencies(uint8_t t, uint8_t s, uint8_t w) { CL_T2_FsmManager::getInstance().setStreamFreq(t, s, w); }

CL_T2_FsmManager::CL_T2_FsmManager() : _sensor(SPI) {}

CL_T2_FsmManager::~CL_T2_FsmManager() {
    if (_featurePool) heap_caps_free(_featurePool);
    if (_rawPool) heap_caps_free(_rawPool);
    if (_prcBeamformed) heap_caps_free(_prcBeamformed);
    if (_txTelemetryBuf) heap_caps_free(_txTelemetryBuf);
    if (_txWaveformBuf) heap_caps_free(_txWaveformBuf);
    if (_txSpectrumBuf) heap_caps_free(_txSpectrumBuf);

	if (_tmpAudL) heap_caps_free(_tmpAudL);
    if (_tmpAudR) heap_caps_free(_tmpAudR);
}

void CL_T2_FsmManager::begin() {
    T2_Type::DynamicConfig v_cfg = CL_T2_ConfigManager::getInstance().getConfig();

    if (!_sensor.init(v_cfg.sensor) || !_dsp.init() || !_extractor.init() || !_storage.init()) {
        ESP_LOGE(TAG, "Critical: Sub-Engine Init Failed!");
        return;
    }

    // 하위 융합 엔진 초기화
    _calibrator.bindExtractor(&_extractor);
    _seqBuilder.init(T2_Def::System::SEQUENCE_FRAMES_MAX_CONST, T2_Def::System::MFCC_DIM_CONST);
    _comm.init();

    _featurePool = (T2_Type::UnifiedFeatureSlot*)heap_caps_aligned_alloc(16, sizeof(T2_Type::UnifiedFeatureSlot) * 100, MALLOC_CAP_SPIRAM);
    _rawPool     = (T2_Type::UnifiedRawChunk*)heap_caps_aligned_alloc(16, sizeof(T2_Type::UnifiedRawChunk) * 100, MALLOC_CAP_SPIRAM);
    _txTelemetryBuf = (T2_Type::PktTelemetry*)heap_caps_aligned_alloc(16, sizeof(T2_Type::PktTelemetry), MALLOC_CAP_SPIRAM);
    _txWaveformBuf  = (T2_Type::PktWaveform*)heap_caps_aligned_alloc(16, sizeof(T2_Type::PktWaveform), MALLOC_CAP_SPIRAM);
    _txSpectrumBuf  = (T2_Type::PktSpectrum*)heap_caps_aligned_alloc(16, sizeof(T2_Type::PktSpectrum), MALLOC_CAP_SPIRAM);
    _prcBeamformed  = (float*)heap_caps_aligned_alloc(16, T2_Def::System::FFT_SIZE_AUDIO_CONST * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

	// L/R 다운믹스용 스크래치 버퍼 할당
    _tmpAudL = (float*)heap_caps_aligned_alloc(16, T2_Def::System::FFT_SIZE_AUDIO_CONST * sizeof(float), MALLOC_CAP_SPIRAM);
    _tmpAudR = (float*)heap_caps_aligned_alloc(16, T2_Def::System::FFT_SIZE_AUDIO_CONST * sizeof(float), MALLOC_CAP_SPIRAM);

    if (!_featurePool || !_rawPool || !_prcBeamformed || !_txTelemetryBuf || !_txWaveformBuf || !_txSpectrumBuf || !_tmpAudL || !_tmpAudR) {
        ESP_LOGE(TAG, "Memory Allocation Failed! System Halted.");
        return;
    }

    _qFreeSlotIdx = xQueueCreate(100, sizeof(uint8_t));
    _qReadySlotIdx = xQueueCreate(100, sizeof(uint8_t));
    _qSessionCmd = xQueueCreate(20, sizeof(uint8_t));

    for (uint8_t i = 0; i < 100; i++) xQueueSend(_qFreeSlotIdx, &i, portMAX_DELAY);

    xTaskCreatePinnedToCore(_captureTask, "CapTask", 8192, this, 10, &_hCaptureTask, 0);
    xTaskCreatePinnedToCore(_processingTask, "PrcTask", 16384, this, 5, &_hProcessTask, 1);

    setSystemState(T2_Type::SystemState::READY);
    ESP_LOGI(TAG, "T240 Orchestrator Started Successfully.");
}

void CL_T2_FsmManager::setSystemState(T2_Type::SystemState p_next) {
    portENTER_CRITICAL(&_stateMux);
    if (_systemState == p_next) { portEXIT_CRITICAL(&_stateMux); return; }
    T2_Type::SystemState v_prev = _systemState;
    _systemState = p_next;
    portEXIT_CRITICAL(&_stateMux);

    if (p_next == T2_Type::SystemState::READY) {
        _readyStartTick = (uint32_t)(esp_timer_get_time() / 1000);
        _isManualRecording = false;
        _currentTrialNo = 0;
    } else if (p_next == T2_Type::SystemState::MAINTENANCE) {
        _storage.closeSession("maintenance_lock");
    } else if (v_prev == T2_Type::SystemState::READY && (p_next == T2_Type::SystemState::MONITORING || p_next == T2_Type::SystemState::RECORDING)) {
        _dsp.resetStates();
        _seqBuilder.reset();
    }
}

void CL_T2_FsmManager::handleExternalTrigger(bool p_isActive) {
    static uint32_t s_lastTrigMs = 0;
    uint32_t v_nowMs = (uint32_t)(esp_timer_get_time() / 1000);
    if (p_isActive != _isrTriggerActive && (v_nowMs - s_lastTrigMs > 50)) {
        _isrTriggerActive = p_isActive;
        s_lastTrigMs = v_nowMs;
    }
}

// ============================================================================
// Core 0: 데이터 수집 태스크
// ============================================================================
void CL_T2_FsmManager::_captureTask(void* p_param) {
    CL_T2_FsmManager* v_this = (CL_T2_FsmManager*)p_param;
    while(1) {
        v_this->_handleSensorStateSync();
        v_this->_handleTriggerLogic();

        if (v_this->_systemState >= T2_Type::SystemState::MONITORING && v_this->_systemState != T2_Type::SystemState::MAINTENANCE) {
            uint8_t v_idx;
            if (xQueueReceive(v_this->_qFreeSlotIdx, &v_idx, 0) == pdTRUE) {
                v_this->_acquireData(v_idx);
                if (xQueueSend(v_this->_qReadySlotIdx, &v_idx, 0) != pdTRUE) {
                    xQueueSend(v_this->_qFreeSlotIdx, &v_idx, 0);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void CL_T2_FsmManager::_handleSensorStateSync() {
    static T2_Type::SystemState s_prev = T2_Type::SystemState::INIT;
    if (s_prev != _systemState) {
        if (_systemState == T2_Type::SystemState::READY || _systemState == T2_Type::SystemState::MAINTENANCE) _sensor.pause();
        else {
			_sensor.resume();
			_dsp.resetStates();
		}
        s_prev = _systemState;
    }
}

void CL_T2_FsmManager::_handleTriggerLogic() {
    if (_systemState == T2_Type::SystemState::READY && _isrTriggerActive) {
        setSystemState(T2_Type::SystemState::RECORDING);
        _currentTrialNo = 1;
        uint8_t v_cmd = (uint8_t)AsyncSessionCmd::OPEN_AUTO;
        xQueueSend(_qSessionCmd, &v_cmd, 0);
        _recordStartTick = (uint32_t)(esp_timer_get_time() / 1000);
    } else if (_systemState == T2_Type::SystemState::RECORDING && !_isrTriggerActive && !_isManualRecording) {
        _checkGracefulClose();
    }
}

void CL_T2_FsmManager::_acquireData(uint8_t p_idx) {
    T2_Type::UnifiedFeatureSlot& v_slot = _featurePool[p_idx];
    T2_Type::UnifiedRawChunk& v_raw     = _rawPool[p_idx];

    // 지역 배열(Stack 8KB) 파괴 -> 미리 할당된 PSRAM 버퍼 사용
    uint32_t v_read = _sensor.readAudioChunk(_tmpAudL, _tmpAudR, T2_Def::System::FFT_SIZE_AUDIO_CONST);

    if (v_read == T2_Def::System::FFT_SIZE_AUDIO_CONST) {
        _sensor.readVibFifoBatch(v_raw.vib[0], v_raw.vib[1], v_raw.vib[2], T2_Def::System::FFT_SIZE_VIB_CONST);
        for(int i = 0; i < T2_Def::System::FFT_SIZE_AUDIO_CONST; i++) {
            v_raw.audio[i] = (_tmpAudL[i] + _tmpAudR[i]) * 0.5f; // Stereo to Mono Downmix
        }
        v_slot.timestamp = (uint64_t)time(NULL);
        v_slot.frame_id++;
    } else {
        _dropCount++;
        _reqSeqReset = true;
    }
}

// ============================================================================
// Core 1: DSP, 특징 추출 및 오케스트레이션 태스크
// ============================================================================
void CL_T2_FsmManager::_processingTask(void* p_param) {
    CL_T2_FsmManager* v_this = (CL_T2_FsmManager*)p_param;
    uint8_t v_idx;

    while(1) {
        if (xQueueReceive(v_this->_qReadySlotIdx, &v_idx, pdMS_TO_TICKS(10))) {
            T2_Type::UnifiedFeatureSlot& v_slot = v_this->_featurePool[v_idx];
            T2_Type::UnifiedRawChunk& v_raw     = v_this->_rawPool[v_idx];

            if (v_this->_systemState == T2_Type::SystemState::CALIBRATING) {
                v_this->_handleCalibratingSlot(v_slot, v_raw); // [복원] 순수 데이터 수집
            } else {
                v_this->_handleNormalSlot(v_idx, v_slot, v_raw); // 정상 로깅 및 판정
            }
            xQueueSend(v_this->_qFreeSlotIdx, &v_idx, 0);
        }
        v_this->_checkGracefulClose();

        vTaskDelay(pdMS_TO_TICKS(1));

        // [방어] 무거운 연산 루프 방어를 위한 명시적 하드웨어 WDT 리셋
        #ifdef ESP_IDF_VERSION
            esp_task_wdt_reset();
        #endif
    }
}

void CL_T2_FsmManager::_handleCalibratingSlot(T2_Type::UnifiedFeatureSlot& p_slot, T2_Type::UnifiedRawChunk& p_raw) {
    _storage.pushFrame(&p_slot, &p_raw);
    uint32_t v_currentTick = (uint32_t)(esp_timer_get_time() / 1000);
    // 10초 분량 수집 완료 시 자동 종료 예약
    if (v_currentTick - _recordStartTick > 10000) {
        requestStop((uint8_t)AsyncSessionCmd::CLOSE_CALIB_DONE);
    }
}

void CL_T2_FsmManager::_handleNormalSlot(uint8_t p_slotIdx, T2_Type::UnifiedFeatureSlot& p_slot, T2_Type::UnifiedRawChunk& p_raw) {
    if (_reqSeqReset) {
        _seqBuilder.reset();
        _reqSeqReset = false;
    }

    // DSP 전처리 및 특징 추출
    _dsp.processAcoustic(p_raw.audio, p_raw.audio, _prcBeamformed);
	_dsp.processVibration(p_raw.vib[0], p_raw.vib[1], p_raw.vib[2], p_raw.vib[0], p_raw.vib[1], p_raw.vib[2], T2_Def::System::FFT_SIZE_VIB_CONST);

    _extractor.extractAudio(_prcBeamformed, T2_Def::System::FFT_SIZE_AUDIO_CONST, p_slot);
    _extractor.extractVibration(p_raw.vib[0], p_raw.vib[1], p_raw.vib[2], T2_Def::System::FFT_SIZE_VIB_CONST, p_slot);

    // [복원] TinyML 추론용 텐서 시퀀스 조립
    _seqBuilder.pushVector(p_slot.mfcc);

    T2_Type::DetectionResult v_res = _runHybridDecision(p_slot);
    if (v_res != T2_Type::DetectionResult::PASS) {
        if (_systemState == T2_Type::SystemState::MONITORING) {
            setSystemState(T2_Type::SystemState::RECORDING);
            _storage.openSession("trg_detect");
        }
        _comm.publishResultMqtt(p_slot, v_res);
    }

    _storage.pushFrame(&p_slot, &p_raw);
    _broadcastStreams(p_slot, p_raw);

    // [복원] 웹에서 필터 튜닝 시 오염된 텐서 시퀀스 초기화
    if (CL_T2_ConfigManager::getInstance().isTuningActive()) {
        CL_T2_ConfigManager::getInstance().clearTuningFlag();
        _seqBuilder.reset();
        ESP_LOGI(TAG, "Hot-Swap Detected. ML Sequence Tensor Reset.");
    }
}

T2_Type::DetectionResult CL_T2_FsmManager::_runHybridDecision(const T2_Type::UnifiedFeatureSlot& p_slot) {
    T2_Type::DynamicConfig v_cfg = CL_T2_ConfigManager::getInstance().getConfig();

    // 1. [교정] 진동 RMS (X축 단일 검사의 치명적 사각지대 제거 -> 3축 모두 검사)
    for (uint8_t i = 0; i < T2_Def::System::VIB_AXIS_CONST; i++) {
        if (p_slot.vib_rms[i] > v_cfg.trigger.vib_rms_thresh) {
            ESP_LOGW(TAG, "VIB NG Detected on Axis %d: %.3f", i, p_slot.vib_rms[i]);
            return T2_Type::DetectionResult::RULE_VIB_NG;
        }
    }

    // 2. 진동 첨도 (베어링/기어 손상 검사)
    // (향후 5.0f 하드코딩을 Config 임계값으로 빼는 것을 권장)
    if (p_slot.kurtosis > 5.0f) {
        ESP_LOGW(TAG, "VIB KURTOSIS NG Detected: %.3f", p_slot.kurtosis);
        return T2_Type::DetectionResult::RULE_VIB_NG;
    }

    // 3. 오디오 전체 RMS
    if (p_slot.audio_rms > v_cfg.trigger.audio_rms_thresh) {
        ESP_LOGW(TAG, "AUDIO RMS NG Detected: %.3f", p_slot.audio_rms);
        return T2_Type::DetectionResult::RULE_AUDIO_NG;
    }

    // 4. [복원] 주파수 대역별(Band) 에너지 임계치 검사 (특정 노이즈/하모닉 대역 돌출 감지)
    for (uint8_t b = 0; b < T2_Def::FeatureLimit::MAX_BAND_RMS_CONST; b++) {
        if (v_cfg.trigger.band_enable[b] && (p_slot.band_energy[b] > v_cfg.trigger.band_thresh[b])) {
            ESP_LOGW(TAG, "AUDIO BAND[%d] NG Detected: %.3f", b, p_slot.band_energy[b]);
            return T2_Type::DetectionResult::RULE_AUDIO_NG;
        }
    }

    return T2_Type::DetectionResult::PASS;
}

void CL_T2_FsmManager::_broadcastStreams(const T2_Type::UnifiedFeatureSlot& p_slot, const T2_Type::UnifiedRawChunk& p_raw) {
    _accTele += _telemetryHz;
    if (_accTele >= 100) {
        _accTele -= 100;
        T2_Type::PktTelemetry* v_pkt = _txTelemetryBuf;
        v_pkt->header.magic = 0x54; v_pkt->header.type = 0x01;
        v_pkt->header.length = sizeof(T2_Type::PktTelemetry) - sizeof(T2_Type::WsHeader);
        v_pkt->sys_state = (uint8_t)_systemState;
        v_pkt->audio_rms = p_slot.audio_rms;
        v_pkt->kurtosis = p_slot.kurtosis;
        v_pkt->spectral_centroid = p_slot.spectral_centroid;

        v_pkt->crest_factor = p_slot.crest_factor;
        v_pkt->sta_lta_ratio = p_slot.sta_lta_ratio;

        memcpy(v_pkt->vib_rms, p_slot.vib_rms, sizeof(v_pkt->vib_rms));
        memcpy(v_pkt->mfcc, p_slot.mfcc, sizeof(v_pkt->mfcc));
        _comm.broadcastBinary(v_pkt, sizeof(T2_Type::PktTelemetry));
    }

    _accWave += _waveformHz;
    if (_accWave >= 100) {
        _accWave -= 100;
        T2_Type::PktWaveform* v_pktW = _txWaveformBuf;
        v_pktW->header.magic = 0x54; v_pktW->header.type = 0x03;
        v_pktW->header.length = sizeof(v_pktW->samples);
        memcpy(v_pktW->samples, _prcBeamformed, sizeof(v_pktW->samples));
        _comm.broadcastBinary(v_pktW, sizeof(T2_Type::PktWaveform));
    }
}

void CL_T2_FsmManager::_checkGracefulClose() {
    if (_stopReasonCmd != 0 && uxQueueMessagesWaiting(_qReadySlotIdx) == 0) {
        uint8_t v_cmd = _stopReasonCmd;
        if (xQueueSend(_qSessionCmd, &v_cmd, 0) == pdTRUE) {
            _stopReasonCmd = 0;
            switch ((AsyncSessionCmd)v_cmd) {
                case AsyncSessionCmd::CLOSE_CALIB_ABORT:
                    setSystemState(T2_Type::SystemState::READY);
                    _dsp.reloadFilters();
                    ESP_LOGI(TAG, "Calibration Aborted.");
                    break;
                default:
                    setSystemState(T2_Type::SystemState::READY);
                    break;
            }
        }
    }
}

void CL_T2_FsmManager::runMaintenanceTask() {
    uint8_t v_cmd;
    while(xQueueReceive(_qSessionCmd, &v_cmd, 0) == pdTRUE) {
        switch((AsyncSessionCmd)v_cmd) {
            case AsyncSessionCmd::OPEN_AUTO: _storage.openSession("trg_auto"); break;
            case AsyncSessionCmd::OPEN_MANUAL: _storage.openSession("man"); break;
            case AsyncSessionCmd::OPEN_CALIB_MAN: _storage.openSession("calib_man", "/sys"); break;
            case AsyncSessionCmd::CLOSE_NORMAL: _storage.closeSession("normal_end"); break;
            case AsyncSessionCmd::CLOSE_MANUAL: _storage.closeSession("man_end"); break;
            case AsyncSessionCmd::CLOSE_CALIB_DONE:
                _storage.closeSession("calib_done");
                _calibrator.startManualCalibration(_storage.getLastRawPath());
                break;
            default: break;
        }
    }
    _comm.runNetwork();
    _storage.checkRotation();

    CL_T2_ConfigManager::getInstance().checkLazyWrite();
}

void CL_T2_FsmManager::dispatchCommand(T2_Type::SystemCommand p_cmd) {
    if (_systemState == T2_Type::SystemState::MAINTENANCE && p_cmd != T2_Type::SystemCommand::CMD_REBOOT) return;

    switch(p_cmd) {
        case T2_Type::SystemCommand::CMD_START:
            _isManualRecording = true;
            setSystemState(T2_Type::SystemState::RECORDING);
            { uint8_t v_c = (uint8_t)AsyncSessionCmd::OPEN_MANUAL; xQueueSend(_qSessionCmd, &v_c, 0); }
            _recordStartTick = (uint32_t)(esp_timer_get_time() / 1000);
            break;
        case T2_Type::SystemCommand::CMD_STOP:
            _stopReasonCmd = (uint8_t)AsyncSessionCmd::CLOSE_MANUAL;
            break;
        case T2_Type::SystemCommand::CMD_CALIBRATE:
            if (_systemState == T2_Type::SystemState::READY) {
                setSystemState(T2_Type::SystemState::CALIBRATING);
                uint8_t v_c = (uint8_t)AsyncSessionCmd::OPEN_CALIB_MAN; xQueueSend(_qSessionCmd, &v_c, 0);
                _recordStartTick = (uint32_t)(esp_timer_get_time() / 1000);
            }
            break;
        case T2_Type::SystemCommand::CMD_REBOOT:
            ESP.restart();
            break;
        default: break;
    }
}
