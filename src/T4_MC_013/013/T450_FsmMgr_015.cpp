#include "T450_FsmMgr_015.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include <sys/time.h>
#include <cstring>

static const char* TAG = "T450_FSM";

void T450_FsmManager::begin() {
    if (!_micEngine.init() || !_dspEngine.init() || !_extractor.init() || !_storage.init()) {
        ESP_LOGE(TAG, "Engine Init Failed!");
        return;
    }

    _calibrator.bindExtractor(&_extractor);

    // 거대 구조체 스택 오버플로우 방지 (PSRAM 힙 동적 할당 및 Zero-copy)
    _featurePool    = (SmeaType::FeatureSlot*)heap_caps_aligned_alloc(16, sizeof(SmeaType::FeatureSlot) * SmeaConfig::System::FEATURE_POOL_SIZE_CONST, MALLOC_CAP_SPIRAM);
    _rawPool        = (SmeaType::RawDataSlot*)heap_caps_aligned_alloc(16, sizeof(SmeaType::RawDataSlot) * SmeaConfig::System::FEATURE_POOL_SIZE_CONST, MALLOC_CAP_SPIRAM);
    _txTelemetryBuf = (SmeaType::PktTelemetry*)heap_caps_aligned_alloc(16, sizeof(SmeaType::PktTelemetry), MALLOC_CAP_SPIRAM);
    _txWaveformBuf  = (SmeaType::PktWaveform*)heap_caps_aligned_alloc(16, sizeof(SmeaType::PktWaveform), MALLOC_CAP_SPIRAM);
    _txSpectrumBuf  = (SmeaType::PktSpectrum*)heap_caps_aligned_alloc(16, sizeof(SmeaType::PktSpectrum), MALLOC_CAP_SPIRAM);

    if (!_featurePool || !_rawPool || !_txTelemetryBuf || !_txWaveformBuf || !_txSpectrumBuf) {
        ESP_LOGE(TAG, "Critical: PSRAM Pool Allocation Failed!");
        return;
    }

    // [연산 최적화] 연산 핫패스용 Internal SRAM(내부 캐시) 고속 버퍼 정렬 할당 (SIMD 대응)
    size_t v_bufSize = sizeof(float) * SmeaConfig::System::FFT_SIZE_CONST;
    _capBufL = (float*)heap_caps_aligned_alloc(16, v_bufSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    _capBufR = (float*)heap_caps_aligned_alloc(16, v_bufSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    _prcBufL = (float*)heap_caps_aligned_alloc(16, v_bufSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    _prcBufR = (float*)heap_caps_aligned_alloc(16, v_bufSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    _prcBeamformed = (float*)heap_caps_aligned_alloc(16, v_bufSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    if (!_capBufL || !_capBufR || !_prcBufL || !_prcBufR || !_prcBeamformed) {
        ESP_LOGE(TAG, "Critical: Internal SRAM Allocation Failed! Reduce FFT size.");
        return;
    }

    _seqBuilder.init(SmeaConfig::MlLimit::MAX_SEQUENCE_FRAMES_CONST, SmeaConfig::System::MFCC_TOTAL_DIM_CONST);
    _communicator.init("", "", ""); 

    _qFreeSlotIdx = xQueueCreate(SmeaConfig::System::FEATURE_POOL_SIZE_CONST, sizeof(uint8_t));
    _qReadySlotIdx = xQueueCreate(SmeaConfig::System::FEATURE_POOL_SIZE_CONST, sizeof(uint8_t));
    _qSessionCmd = xQueueCreate(20, sizeof(uint8_t)); // 극한의 환경 오버플로우 방어 버퍼 20개 확보

    for (uint8_t i = 0; i < SmeaConfig::System::FEATURE_POOL_SIZE_CONST; i++) {
        xQueueSend(_qFreeSlotIdx, &i, portMAX_DELAY);
    }

    xTaskCreatePinnedToCore(_captureTask, "CapTask", SmeaConfig::Task::CAPTURE_STACK_SIZE_CONST, this, SmeaConfig::Task::CAPTURE_PRIORITY_CONST, &_hCaptureTask, SmeaConfig::Task::CORE_CAPTURE_CONST);
    xTaskCreatePinnedToCore(_processingTask, "PrcTask", SmeaConfig::Task::PROCESS_STACK_SIZE_CONST, this, SmeaConfig::Task::PROCESS_PRIORITY_CONST, &_hProcessTask, SmeaConfig::Task::CORE_PROCESS_CONST);

    setSystemState(SystemState::READY);
}

// 멀티코어 환경에서 100% 안전한 원자적(Atomic) 상태 천이
void T450_FsmManager::setSystemState(SystemState p_nextState) {
    portENTER_CRITICAL(&_stateMux);
    if (_systemState == p_nextState) {
        portEXIT_CRITICAL(&_stateMux);
        return;
    }
    SystemState v_prevState = _systemState;
    _systemState = p_nextState;
    portEXIT_CRITICAL(&_stateMux);

    // 무거운 초기화 연산은 락 바깥에서 수행하여 인터럽트 지연 차단
    if (p_nextState == SystemState::READY) {
        _readyStartSec = (uint32_t)(esp_timer_get_time() / 1000000ULL);
        _currentTrialNo = 0;
        _isManualRecording = false;
    }
    else if (p_nextState == SystemState::MAINTENANCE) {
        _storage.closeSession("ota");
        ESP_LOGI(TAG, "System locked into MAINTENANCE for OTA. Bus isolated.");
    }
    else if (p_nextState == SystemState::MONITORING || p_nextState == SystemState::RECORDING) {
        if (v_prevState == SystemState::READY) {
            _dspEngine.resetFilterStates();
            _seqBuilder.reset();
        }
    }
}

// 기계식 스위치의 Bouncing 노이즈 차단을 위한 50ms 소프트웨어 디바운싱
void T450_FsmManager::handleExternalTrigger(bool p_isActive) {
    static uint32_t s_lastTriggerMs = 0;
    uint32_t v_now = (uint32_t)(esp_timer_get_time() / 1000);

    if (p_isActive != _isrTriggerActive) {
        if (v_now - s_lastTriggerMs > 50) { 
            _isrTriggerActive = p_isActive;
            s_lastTriggerMs = v_now;
        }
    }
}

// ============================================================================
// 초경량화된 I2S 하드웨어 캡처 태스크 (Core 0)
// ============================================================================
void T450_FsmManager::_captureTask(void* p_param) {
    T450_FsmManager* v_this = (T450_FsmManager*)p_param;

    // [스택 최적화] 무거운 로컬 배열 대신 초고속 Internal SRAM 포인터 사용
    float* v_windowL = v_this->_capBufL;
    float* v_windowR = v_this->_capBufR;
    memset(v_windowL, 0, sizeof(float) * SmeaConfig::System::FFT_SIZE_CONST);
    memset(v_windowR, 0, sizeof(float) * SmeaConfig::System::FFT_SIZE_CONST);

    SystemState v_lastAcqState = SystemState::INIT;

    while(1) {
        v_this->_handleMicStateTransition();
        v_this->_handleTriggerLogic();
        v_this->_acquireAndQueueData(v_lastAcqState, v_windowL, v_windowR);

        vTaskDelay(pdMS_TO_TICKS(SmeaConfig::Task::WDG_YIELD_MS_CONST));
    }
}

void T450_FsmManager::_handleMicStateTransition() {
    static SystemState s_prevMicState = SystemState::INIT;
    SystemState v_currState = _systemState;
    
    if (s_prevMicState != v_currState) {
        if (v_currState == SystemState::READY || v_currState == SystemState::MAINTENANCE) {
            _micEngine.pause();
        } 
        else if (s_prevMicState == SystemState::READY || s_prevMicState == SystemState::MAINTENANCE) {
            _micEngine.resume();
            _dspEngine.resetFilterStates();
        }
        s_prevMicState = v_currState; 
    }
}

void T450_FsmManager::_handleTriggerLogic() {
    bool v_currentTrigger = _isrTriggerActive;
    SystemState v_currState = _systemState;

    if (v_currState != SystemState::MAINTENANCE && v_currState != SystemState::CALIBRATING) {
        if (!v_currentTrigger && v_currState != SystemState::READY && !_isManualRecording) {
            if (v_currState == SystemState::RECORDING) {
                requestStop((uint8_t)AsyncSessionCmd::CLOSE_FORCE_DROP);
            } else {
                setSystemState(SystemState::READY);
            }
        }
        else if (v_currentTrigger && v_currState == SystemState::READY) {
            setSystemState(SystemState::RECORDING); 
            _currentTrialNo = 1;
            _recordStartTick = (uint32_t)(esp_timer_get_time() / 1000); 
            
            uint8_t v_cmd = (uint8_t)AsyncSessionCmd::OPEN_AUTO;
            xQueueSend(_qSessionCmd, &v_cmd, 0); 
            _extractor.setNoiseLearning(true); 
        }
    }
}

void T450_FsmManager::_acquireAndQueueData(SystemState& p_lastAcqState, float* p_windowL, float* p_windowR) {
    SystemState v_currState = _systemState;
    
    if ((v_currState == SystemState::MONITORING || v_currState == SystemState::RECORDING || v_currState == SystemState::CALIBRATING) && (_stopReasonCmd == 0)) { 

        DynamicConfig v_cfg = T415_ConfigManager::getInstance().getConfig();
        const uint32_t v_fftSize = SmeaConfig::System::FFT_SIZE_CONST;
        const uint32_t v_hopSamples = (uint32_t)(SmeaConfig::System::SAMPLING_RATE_CONST * (v_cfg.dsp.hop_ms / SmeaConfig::System::MS_PER_SEC_CONST));
        const uint32_t v_overlapSamples = v_fftSize - v_hopSamples;

        uint32_t v_readCount = 0;
        uint32_t v_targetCount = (p_lastAcqState == SystemState::READY) ? v_fftSize : v_hopSamples;

        if (p_lastAcqState == SystemState::READY) {
            v_readCount = _micEngine.readData(p_windowL, p_windowR, v_fftSize);
        } else {
            memmove(&p_windowL[0], &p_windowL[v_hopSamples], v_overlapSamples * sizeof(float));
            memmove(&p_windowR[0], &p_windowR[v_hopSamples], v_overlapSamples * sizeof(float));
            v_readCount = _micEngine.readData(&p_windowL[v_overlapSamples], &p_windowR[v_overlapSamples], v_hopSamples);
        }

        p_lastAcqState = v_currState;

        if (v_readCount == v_targetCount) {
            uint8_t v_slotIdx;
            if (xQueueReceive(_qFreeSlotIdx, &v_slotIdx, 0) == pdTRUE) {
                SmeaType::FeatureSlot& v_slot = _featurePool[v_slotIdx];
                SmeaType::RawDataSlot& v_raw  = _rawPool[v_slotIdx];

                memcpy(v_raw.raw_L, p_windowL, v_fftSize * sizeof(float));
                memcpy(v_raw.raw_R, p_windowR, v_fftSize * sizeof(float));
                v_slot.timestamp = (uint64_t)time(NULL);

                if (xQueueSend(_qReadySlotIdx, &v_slotIdx, 0) != pdTRUE) {
                    xQueueSend(_qFreeSlotIdx, &v_slotIdx, 0);
                }
            }
        }
        else {
            _dropCount++;
            _reqSeqReset = true; // Core 1로 시퀀스 리셋 위임 (데이터 레이스 방지)
            ESP_LOGW(TAG, "I2S Drop Detected (Total: %u)! Tensor reset requested.", _dropCount);
        }
    } else {
        p_lastAcqState = v_currState; 
    }
}

// ============================================================================
// 초경량화된 중앙 연산 처리 태스크 (Core 1)
// ============================================================================
void T450_FsmManager::_processingTask(void* p_param) {
    T450_FsmManager* v_this = (T450_FsmManager*)p_param;
    uint8_t v_slotIdx;
    
    while(1) {
        if (xQueueReceive(v_this->_qReadySlotIdx, &v_slotIdx, pdMS_TO_TICKS(SmeaConfig::Task::QUEUE_BLOCK_MS_CONST))) {
            SmeaType::FeatureSlot& v_slot = v_this->_featurePool[v_slotIdx];
            SmeaType::RawDataSlot& v_raw  = v_this->_rawPool[v_slotIdx];

            if (v_this->_systemState == SystemState::CALIBRATING) {
                v_this->_handleCalibratingSlot(v_slot, v_raw);
            } else {
                v_this->_handleNormalSlot(v_slotIdx, v_slot, v_raw, v_this->_prcBeamformed);
            }

            xQueueSend(v_this->_qFreeSlotIdx, &v_slotIdx, 0);
        }

        v_this->_checkGracefulClose();

        vTaskDelay(pdMS_TO_TICKS(1));
        #ifdef ESP_IDF_VERSION
            esp_task_wdt_reset();
        #endif
    }
}

void T450_FsmManager::_handleCalibratingSlot(SmeaType::FeatureSlot& p_slot, SmeaType::RawDataSlot& p_raw) {
    _storage.pushFrame(&p_slot, &p_raw);

    uint32_t v_currentTick = (uint32_t)(esp_timer_get_time() / 1000);
    if (v_currentTick - _recordStartTick > (SmeaConfig::CalibLimit::MANUAL_RECORD_SEC_CONST * 1000)) {
        requestStop((uint8_t)AsyncSessionCmd::CLOSE_CALIB_DONE);
    }
}

void T450_FsmManager::_handleNormalSlot(uint8_t p_slotIdx, SmeaType::FeatureSlot& p_slot, SmeaType::RawDataSlot& p_raw, float* p_beamformed) {
    if (_reqSeqReset) {
        _seqBuilder.reset();
        _reqSeqReset = false;
    }
    
    DynamicConfig v_cfg = T415_ConfigManager::getInstance().getConfig();
    p_slot.trial_no = _currentTrialNo;

    // [연산 최적화: 제로 캐시 미스] 느린 PSRAM 데이터를 초고속 Internal SRAM으로 단숨에 블록 복사
    size_t v_copyBytes = sizeof(float) * SmeaConfig::System::FFT_SIZE_CONST;
    memcpy(_prcBufL, p_raw.raw_L, v_copyBytes);
    memcpy(_prcBufR, p_raw.raw_R, v_copyBytes);

    // 복사된 Internal SRAM 버퍼로 DSP/특징추출 연산 수행 (CPU 파이프라인 쓰래싱 0%)
    _dspEngine.process(_prcBufL, _prcBufR, p_beamformed, SmeaConfig::System::FFT_SIZE_CONST);
    _extractor.extract(p_beamformed, _prcBufL, _prcBufR, SmeaConfig::System::FFT_SIZE_CONST, p_slot);
    _seqBuilder.pushVector(p_slot.mfcc);

    DetectionResult v_result = _runHybridDecision(p_slotIdx);

    if (v_result != DetectionResult::PASS) {
        if (_systemState == SystemState::MONITORING) {
            setSystemState(SystemState::RECORDING);
            _storage.openSession("trg_ng_detect");
            _recordStartTick = (uint32_t)(esp_timer_get_time() / 1000);
        }
        _communicator.publishResultMqtt(p_slot, v_result);
    }

    _storage.pushFrame(&p_slot, &p_raw);
        
    if (_systemState == SystemState::RECORDING) {
        uint32_t v_currentTick = (uint32_t)(esp_timer_get_time() / 1000);
        if (!_isManualRecording && (v_currentTick - _recordStartTick > (v_cfg.decision.valid_end_sec * SmeaConfig::System::MS_PER_SEC_CONST))) {
            if (_currentTrialNo < SmeaConfig::DecisionLimit::MAX_TRIAL_COUNT_CONST) {
                _currentTrialNo++;
                _recordStartTick = v_currentTick;
                _extractor.setNoiseLearning(_currentTrialNo == 1);
            } else {
                requestStop((uint8_t)AsyncSessionCmd::CLOSE_NORMAL);
            }
        }
    }

    _broadcastStreams(p_slot, p_beamformed);

    if (T415_ConfigManager::getInstance().isTuningActive()) {
        T415_ConfigManager::getInstance().clearTuningFlag();
        _seqBuilder.reset();
        ESP_LOGI(TAG, "Tuning Applied. ML Sequence Tensor Reset.");
    }
}

void T450_FsmManager::_broadcastStreams(SmeaType::FeatureSlot& p_slot, float* p_beamformed) {
    _frameCount++;
    uint8_t v_safeTeleHz = (_telemetryHz > 100) ? 100 : _telemetryHz;
    uint8_t v_safeWaveHz = (_waveformHz > 100) ? 100 : _waveformHz;
    uint8_t v_safeSpecHz = (_spectrumHz > 100) ? 100 : _spectrumHz;

    _accTele += v_safeTeleHz;
    if (_accTele >= 100) {
        _accTele -= 100;
        SmeaType::PktTelemetry* v_pktTele = _txTelemetryBuf;
        v_pktTele->header.magic = 0xA5;
        v_pktTele->header.type = (uint8_t)SmeaType::StreamType::TELEMETRY;
        v_pktTele->header.length = sizeof(SmeaType::PktTelemetry) - sizeof(SmeaType::WsHeader);
        v_pktTele->header.stage = 0;
        v_pktTele->sys_state = (uint8_t)_systemState;
        v_pktTele->trial_no = _currentTrialNo;
        v_pktTele->rms = p_slot.rms;
        v_pktTele->sta_lta_ratio = p_slot.sta_lta_ratio;
        v_pktTele->kurtosis = p_slot.kurtosis;
        v_pktTele->spectral_centroid = p_slot.spectral_centroid;

        memcpy(v_pktTele->band_rms, p_slot.band_rms, sizeof(p_slot.band_rms));
        for (uint8_t i = 0; i < SmeaConfig::FeatureLimit::TOP_PEAKS_COUNT_CONST; i++) {
            v_pktTele->peak_freqs[i] = p_slot.top_peaks[i].frequency;
            v_pktTele->peak_amps[i]  = p_slot.top_peaks[i].amplitude;
        }
        memcpy(v_pktTele->mfcc, p_slot.mfcc, sizeof(p_slot.mfcc));

        _communicator.broadcastBinary(v_pktTele, sizeof(SmeaType::PktTelemetry));
    }

    _accWave += v_safeWaveHz;
    if (_accWave >= 100) {
        _accWave -= 100;
        SmeaType::PktWaveform* v_pktWave = _txWaveformBuf;
        v_pktWave->header.magic = 0xA5;
        v_pktWave->header.type = (uint8_t)SmeaType::StreamType::WAVEFORM;
        v_pktWave->header.length = sizeof(v_pktWave->samples);
        v_pktWave->header.stage = 2; 
        memcpy(v_pktWave->samples, p_beamformed, sizeof(v_pktWave->samples));
        _communicator.broadcastBinary(v_pktWave, sizeof(SmeaType::PktWaveform));
    }

    _accSpec += v_safeSpecHz;
    if (_accSpec >= 100) {
        _accSpec -= 100;
        SmeaType::PktSpectrum* v_pktSpec = _txSpectrumBuf;
        v_pktSpec->header.magic = 0xA5;
        v_pktSpec->header.type = (uint8_t)SmeaType::StreamType::SPECTRUM;
        v_pktSpec->header.length = sizeof(v_pktSpec->bins);
        v_pktSpec->header.stage = 2;
        memcpy(v_pktSpec->bins, _extractor.getPowerSpectrumBuf(), sizeof(v_pktSpec->bins));
        _communicator.broadcastBinary(v_pktSpec, sizeof(SmeaType::PktSpectrum));
    }
}

void T450_FsmManager::_checkGracefulClose() {
    if (_stopReasonCmd != 0 && uxQueueMessagesWaiting(_qReadySlotIdx) == 0) {
        uint8_t v_cmd = _stopReasonCmd; 
        
        // [교정 1] 큐가 꽉 차서 실패하면 상태 천이를 미루고 다음 루프에서 재시도 (스토리지 락업 원천 차단)
        if (xQueueSend(_qSessionCmd, &v_cmd, 0) == pdTRUE) {
            _stopReasonCmd = 0; // 성공 시에만 깃발 내림
            
            switch ((AsyncSessionCmd)v_cmd) {
                case AsyncSessionCmd::CLOSE_NORMAL:
                    setSystemState(_isrTriggerActive ? SystemState::MONITORING : SystemState::READY);
                    break;
                case AsyncSessionCmd::CLOSE_FORCE_DROP:
                case AsyncSessionCmd::CLOSE_MANUAL:
                    setSystemState(SystemState::READY);
                    break;
                case AsyncSessionCmd::CLOSE_CALIB_DONE:
                    break; // 유지
                case AsyncSessionCmd::CLOSE_CALIB_ABORT:
                    setSystemState(SystemState::READY);
                    _dspEngine.reloadCalibration();
                    _extractor.reloadConfigCache();
                    _extractor.setNoiseLearning(true);
                    ESP_LOGI(TAG, "Calibration Aborted safely.");
                    break;
                default:
                    setSystemState(SystemState::READY);
                    break;
            }
        } else {
            ESP_LOGW(TAG, "Session Command Queue is FULL! Retrying Graceful Close...");
        }
    }
}

DetectionResult T450_FsmManager::_runHybridDecision(uint8_t p_slotIdx) {
    SmeaType::FeatureSlot& v_slot = _featurePool[p_slotIdx];
    DynamicConfig v_cfg = T415_ConfigManager::getInstance().getConfig();

    if (v_slot.energy < v_cfg.decision.test_ng_min_energy) return DetectionResult::TEST_NG;
    if (v_slot.energy > v_cfg.decision.rule_enrg_threshold) return DetectionResult::RULE_NG;
    if (v_slot.pooling_stddev_min > v_cfg.decision.rule_stddev_threshold) return DetectionResult::RULE_NG;
    if (v_slot.sta_lta_ratio > v_cfg.decision.sta_lta_threshold) return DetectionResult::RULE_NG;

    return DetectionResult::PASS;
}

// ============================================================================
// Main Loop Maintenance & I/O Offloading 태스크
// ============================================================================
void T450_FsmManager::runMaintenanceTask() {
    uint8_t v_cmd;
    while (xQueueReceive(_qSessionCmd, &v_cmd, 0) == pdTRUE) {
        switch ((AsyncSessionCmd)v_cmd) {
            case AsyncSessionCmd::OPEN_AUTO:       _storage.openSession("trg_auto"); break;
            case AsyncSessionCmd::OPEN_MANUAL:     _storage.openSession("man"); break;
            case AsyncSessionCmd::OPEN_CALIB_MAN:  _storage.openSession("calib_man", SmeaConfig::Path::DIR_CALIB_DEF); break;
            case AsyncSessionCmd::OPEN_CALIB_AUTO: _storage.openSession("calib_auto_tmp", SmeaConfig::Path::DIR_CALIB_DEF); break;
            case AsyncSessionCmd::CLOSE_NORMAL:    _storage.closeSession("trg_end_trials"); break;
            case AsyncSessionCmd::CLOSE_FORCE_DROP:_storage.closeSession("trg_drop_force_end"); break;
            case AsyncSessionCmd::CLOSE_REBOOT:    _storage.closeSession("reboot"); break;
            case AsyncSessionCmd::CLOSE_MANUAL:    _storage.closeSession("man_end"); break;
            case AsyncSessionCmd::CLOSE_CALIB_ABORT: 
                _storage.closeSession("calib_abort"); 
                break; 
            case AsyncSessionCmd::CLOSE_CALIB_DONE:
                _storage.closeSession("calib_done"); 
                {
                    const char* v_lastPath = _storage.getLastRawPath();
                    if (v_lastPath != nullptr && strlen(v_lastPath) > 0) {
                        if (strstr(v_lastPath, "auto") != nullptr) {
                            _calibrator.startAutoCalibration(v_lastPath);
                        } else {
                            if(!_calibrator.startManualCalibration(v_lastPath)) {
                                ESP_LOGE(TAG, "Failed to start Calib Task (OOM). Aborting.");
                                dispatchCommand(SystemCommand::CMD_CALIB_STOP);
                            }
                        }
                    }
                }
                break;
        }
    }

    _communicator.runNetwork();
    _storage.checkRotation();
    T415_ConfigManager::getInstance().checkLazyWrite();

    if (_storage.hasIoError()) {
        _storage.attemptRecovery();
    }

    if (_systemState == SystemState::READY) {
        DynamicConfig v_cfg = T415_ConfigManager::getInstance().getConfig();
        uint32_t v_currentSec = (uint32_t)(esp_timer_get_time() / 1000000ULL);

        if (v_cfg.calib.auto_idle_min > 0) {
            if ((v_currentSec - _readyStartSec) > (v_cfg.calib.auto_idle_min * 60)) {
                dispatchCommand(SystemCommand::CMD_CALIB_AUTO_START);
                _readyStartSec = v_currentSec;
            }
        }
    }
}

void T450_FsmManager::dispatchCommand(SystemCommand p_cmd) {
    if (_systemState == SystemState::MAINTENANCE && p_cmd != SystemCommand::CMD_REBOOT && p_cmd != SystemCommand::CMD_OTA_END) {
        return;
    }

    switch (p_cmd) {
        case SystemCommand::CMD_MANUAL_RECORD_START:
            if (_systemState == SystemState::MONITORING || _systemState == SystemState::READY) {
                _isManualRecording = true;
                setSystemState(SystemState::RECORDING);
                uint8_t v_cmd = (uint8_t)AsyncSessionCmd::OPEN_MANUAL;
                xQueueSend(_qSessionCmd, &v_cmd, 0);
                _recordStartTick = (uint32_t)(esp_timer_get_time() / 1000);
            }
            break;
        case SystemCommand::CMD_MANUAL_RECORD_STOP:
            if (_systemState == SystemState::RECORDING && _isManualRecording) {
                _isManualRecording = false;
                // 웹 API의 명령은 최우선이므로 기존 예약 무시(forceOverride = true)
                requestStop((uint8_t)AsyncSessionCmd::CLOSE_MANUAL, true); 
            }
            break;
        case SystemCommand::CMD_LEARN_NOISE:
            _extractor.setNoiseLearning(true);
            break;
        case SystemCommand::CMD_REBOOT:
            _storage.closeSession("reboot");
            vTaskDelay(pdMS_TO_TICKS(SmeaConfig::Task::REBOOT_DELAY_MS_CONST));
            ESP.restart();
            break;
        case SystemCommand::CMD_OTA_START:
            setSystemState(SystemState::MAINTENANCE);
            break;
        case SystemCommand::CMD_OTA_END:
            if (_systemState == SystemState::MAINTENANCE) {
                setSystemState(SystemState::READY);
            }
            break;
        case SystemCommand::CMD_CALIB_MANUAL_START:
            if (_systemState == SystemState::READY) {
                setSystemState(SystemState::CALIBRATING);
                uint8_t v_cmd = (uint8_t)AsyncSessionCmd::OPEN_CALIB_MAN;
                xQueueSend(_qSessionCmd, &v_cmd, 0);
                ESP_LOGI(TAG, "Manual Calibration Session Started.");
            }
            break;
        case SystemCommand::CMD_CALIB_AUTO_START:
            if (_systemState == SystemState::READY) {
                setSystemState(SystemState::CALIBRATING);
                uint8_t v_cmd = (uint8_t)AsyncSessionCmd::OPEN_CALIB_AUTO;
                xQueueSend(_qSessionCmd, &v_cmd, 0);
                _recordStartTick = (uint32_t)(esp_timer_get_time() / 1000);
                ESP_LOGI(TAG, "Auto Calibration Session Started.");
            }
            break;
        // (캘리브레이션 정지)
        case SystemCommand::CMD_CALIB_STOP:
            if (_systemState == SystemState::CALIBRATING) {
                // 웹 API의 강제 취소명령 최우선 적용
                requestStop((uint8_t)AsyncSessionCmd::CLOSE_CALIB_ABORT, true); 
            }
            break;
        case SystemCommand::CMD_TUNING_PREVIEW:
        case SystemCommand::CMD_TUNING_SAVE:
        case SystemCommand::CMD_TUNING_CANCEL:
            break;
    }
}
