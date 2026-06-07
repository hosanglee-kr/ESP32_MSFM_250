/* ============================================================================
 * File: T290_FsmMgr_243.cpp
 * Summary: v243 4-Tier 시스템 통합 오케스트레이터 구현부
 * ========================================================================== */
#include <cstring>
#include <time.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "esp_heap_caps.h"
#include "esp_system.h"

#include "T290_FsmMgr_243.hpp"

static const char* TAG = "T243_FSM";

// 전역 인터페이스 함수들
void T240_DispatchCommand(T2_Type::SystemCommand p_cmd) { CL_T2_FsmManager::getInstance().dispatchCommand(p_cmd); }
uint8_t T240_GetCurrentState() { return (uint8_t)CL_T2_FsmManager::getInstance().getState(); }

CL_T2_FsmManager::CL_T2_FsmManager()
    : _sensor(SPI), _state(T2_Type::SystemState::INIT) {}

CL_T2_FsmManager::~CL_T2_FsmManager() {
    if (_featurePool) heap_caps_free(_featurePool);
    if (_rawPool) heap_caps_free(_rawPool);
    if (_pktTele) heap_caps_free(_pktTele);
    if (_pktWave) heap_caps_free(_pktWave);
    if (_pktWaveVib) heap_caps_free(_pktWaveVib);
    if (_pktSpec) heap_caps_free(_pktSpec);
    if (_pktSeq) heap_caps_free(_pktSeq);
    if (_capBufL) heap_caps_free(_capBufL);
    if (_capBufR) heap_caps_free(_capBufR);
    if (_prcBufL) heap_caps_free(_prcBufL);
    if (_prcBufR) heap_caps_free(_prcBufR);
    if (_prcBeamformed) heap_caps_free(_prcBeamformed);
}

bool CL_T2_FsmManager::init() {
    ESP_LOGI(TAG, "Initializing v243 Orchestrator...");

    // 1. 설정 매니저 초기화 및 로드
    if (!CL_T2_ConfigManager::getInstance().init()) {
        ESP_LOGE(TAG, "ConfigManager Init Failed!");
        return false;
    }
    const auto& v_cfg = CL_T2_ConfigManager::getInstance().getConfig();

    // 2. 하위 엔진 초기화
    if (!_sensor.init(v_cfg.system, v_cfg.vib_sensor, v_cfg.aud_sensor)) return false;
    if (!_dsp.init(v_cfg)) return false;
    if (!_extractor.init()) return false;
    if (!_storage.init()) return false;
    if (!_comm.init()) return false;

    _seqBuilder.init(T2_Def::Global::System::SEQUENCE_FRAMES_MAX, T2_Def::Audio::FeatureLimit::MFCC_DIM_MAX);
    _calibrator.bindExtractor(&_extractor);

    // 3. PSRAM 데이터 풀 (Zero-copy 파이프라인)
    _featurePool = (T2_Type::UnifiedFeatureSlot*)heap_caps_aligned_alloc(16, sizeof(T2_Type::UnifiedFeatureSlot) * 64, MALLOC_CAP_SPIRAM);
    _rawPool     = (T2_Type::UnifiedRawChunk*)heap_caps_aligned_alloc(16, sizeof(T2_Type::UnifiedRawChunk) * 64, MALLOC_CAP_SPIRAM);
    _pktTele     = (T2_Type::PktTelemetry*)heap_caps_aligned_alloc(16, sizeof(T2_Type::PktTelemetry), MALLOC_CAP_SPIRAM);
    _pktWave     = (T2_Type::PktWaveform*)heap_caps_aligned_alloc(16, sizeof(T2_Type::PktWaveform), MALLOC_CAP_SPIRAM);
    _pktWaveVib  = (T2_Type::PktWaveformVib*)heap_caps_aligned_alloc(16, sizeof(T2_Type::PktWaveformVib), MALLOC_CAP_SPIRAM);
    _pktSpec     = (T2_Type::PktSpectrum*)heap_caps_aligned_alloc(16, sizeof(T2_Type::PktSpectrum), MALLOC_CAP_SPIRAM);
    _pktSeq      = (T2_Type::PktSequence*)heap_caps_aligned_alloc(16, sizeof(T2_Type::PktSequence), MALLOC_CAP_SPIRAM);

    _telemetryHz = v_cfg.system.tele_hz;
    _waveformHz  = v_cfg.system.wave_hz;
    _sequenceHz  = 2; // AI 시퀀스는 0.5초(2Hz) 주기로 전송 (대역폭 방어)

    // [v015 이식] 연산 핫패스용 Internal SRAM 고속 버퍼 정렬 할당 (SIMD 가속용)
    size_t v_fSize = sizeof(float) * T2_Def::Vib::Sensor::FFT_SIZE_MAX;

    _capBufL = nullptr;
    _capBufR = nullptr;
    _prcBufL = nullptr;
    _prcBufR = nullptr;
    _prcBeamformed = nullptr;

    if (v_cfg.system.audio_enable) {
        _capBufL = (float*)heap_caps_aligned_alloc(16, v_fSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        _prcBufL = (float*)heap_caps_aligned_alloc(16, v_fSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        _prcBeamformed = (float*)heap_caps_aligned_alloc(16, v_fSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

        if (v_cfg.aud_sensor.channel_mode == T2_Type::AudioChannelMode::STEREO) {
            _capBufR = (float*)heap_caps_aligned_alloc(16, v_fSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
            _prcBufR = (float*)heap_caps_aligned_alloc(16, v_fSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        }
    }

    bool v_allocSuccess = _featurePool && _rawPool && _pktTele && _pktWave && _pktWaveVib && _pktSpec && _pktSeq;
    if (v_cfg.system.audio_enable) {
        v_allocSuccess &= (_capBufL != nullptr) && (_prcBufL != nullptr) && (_prcBeamformed != nullptr);
        if (v_cfg.aud_sensor.channel_mode == T2_Type::AudioChannelMode::STEREO) {
            v_allocSuccess &= (_capBufR != nullptr) && (_prcBufR != nullptr);
        }
    }

    if (!v_allocSuccess) {
        ESP_LOGE(TAG, "Critical Allocation Failed! Check SRAM/PSRAM Heap.");
        return false;
    }

    // 4. 통신 큐 생성
    _qFreeIdx  = xQueueCreate(64, sizeof(uint8_t));
    _qReadyIdx = xQueueCreate(64, sizeof(uint8_t));
    _qSessionCmd = xQueueCreate(16, sizeof(uint8_t)); // [v015 이식] 세션 비동기 제어 큐

    for (uint8_t i = 0; i < 64; i++) xQueueSend(_qFreeIdx, &i, 0);

    // 5. 태스크 생성
    xTaskCreatePinnedToCore(_captureTask, "CapTask", 8192, this, 10, &_hCaptureTask, 0);
    xTaskCreatePinnedToCore(_processTask, "PrcTask", 16384, this, 5, &_hProcessTask, 1);

    setState(T2_Type::SystemState::READY);
    ESP_LOGI(TAG, "v243 Orchestrator Ready.");
    return true;
}

void CL_T2_FsmManager::setState(T2_Type::SystemState p_newState) {
    portENTER_CRITICAL(&_stateMux);
    if (_state == p_newState) { portEXIT_CRITICAL(&_stateMux); return; }
    T2_Type::SystemState v_prevState = _state;
    _state = p_newState;
    portEXIT_CRITICAL(&_stateMux);

    ESP_LOGI(TAG, "System State Changed: %d -> %d", (int)v_prevState, (int)p_newState);

    // [v015 이식] 상태 천이별 중량 초기화 로직 (락 바깥에서 수행)
    if (p_newState == T2_Type::SystemState::READY) {
        _lastTick = (uint32_t)(esp_timer_get_time() / 1000000ULL);
        _currentTrial = 0;
        _isManualRecording = false;
    }
    else if (p_newState == T2_Type::SystemState::MAINTENANCE) {
        _storage.closeSession("ota_maintenance");
        ESP_LOGW(TAG, "System entering MAINTENANCE mode. Bus isolated.");
    }
    else if (p_newState == T2_Type::SystemState::MONITORING || p_newState == T2_Type::SystemState::RECORDING) {
        if (v_prevState == T2_Type::SystemState::READY) {
            _dsp.resetStates();
            _seqBuilder.reset();
        }
    }
}

void CL_T2_FsmManager::dispatchCommand(T2_Type::SystemCommand p_cmd) {
    if (_state == T2_Type::SystemState::MAINTENANCE &&
        p_cmd != T2_Type::SystemCommand::CMD_REBOOT) return;

    ESP_LOGI(TAG, "Command Dispatched: %d", (uint8_t)p_cmd);
    uint8_t v_sessCmd = 0;

    switch (p_cmd) {
        case T2_Type::SystemCommand::CMD_START:
            setState(T2_Type::SystemState::MONITORING);
            break;
        case T2_Type::SystemCommand::CMD_STOP:
            _stopReasonCmd = (uint8_t)T2_Type::AsyncSessionCmd::CLOSE_NORMAL;
            break;
        case T2_Type::SystemCommand::CMD_MANUAL_REC_START:
            if (_state == T2_Type::SystemState::READY || _state == T2_Type::SystemState::MONITORING) {
                _isManualRecording = true;
                setState(T2_Type::SystemState::RECORDING);
                v_sessCmd = (uint8_t)T2_Type::AsyncSessionCmd::OPEN_MANUAL;
                xQueueSend(_qSessionCmd, &v_sessCmd, 0);
            }
            break;
        case T2_Type::SystemCommand::CMD_MANUAL_REC_STOP:
            if (_state == T2_Type::SystemState::RECORDING && _isManualRecording) {
                _isManualRecording = false;
                _stopReasonCmd = (uint8_t)T2_Type::AsyncSessionCmd::CLOSE_MANUAL;
            }
            break;
        case T2_Type::SystemCommand::CMD_LEARN_NOISE:
            _extractor.setNoiseLearning(true);
            break;
        case T2_Type::SystemCommand::CMD_CALIBRATE:
            if (_state == T2_Type::SystemState::READY) {
                setState(T2_Type::SystemState::CALIBRATING);
                v_sessCmd = (uint8_t)T2_Type::AsyncSessionCmd::OPEN_CALIB_MAN;
                xQueueSend(_qSessionCmd, &v_sessCmd, 0);
            }
            break;
        case T2_Type::SystemCommand::CMD_REBOOT:
            _storage.closeSession("reboot");
            vTaskDelay(pdMS_TO_TICKS(500));
            esp_restart();
            break;
        default: break;
    }
}

void CL_T2_FsmManager::runMaintenance() {
    uint8_t v_cmd;
    while (xQueueReceive(_qSessionCmd, &v_cmd, 0) == pdTRUE) {
        switch ((T2_Type::AsyncSessionCmd)v_cmd) {
            case T2_Type::AsyncSessionCmd::OPEN_AUTO:       _storage.openSession("trg_auto"); break;
            case T2_Type::AsyncSessionCmd::OPEN_MANUAL:     _storage.openSession("man"); break;
            case T2_Type::AsyncSessionCmd::OPEN_CALIB_MAN:  _storage.openSession("calib_man"); break;
            case T2_Type::AsyncSessionCmd::CLOSE_NORMAL:    _storage.closeSession("auto_end"); break;
            case T2_Type::AsyncSessionCmd::CLOSE_MANUAL:    _storage.closeSession("man_end"); break;
            case T2_Type::AsyncSessionCmd::CLOSE_CALIB_DONE:
                _storage.closeSession("calib_done");
                _calibrator.startManualCalibration(_storage.getLastPath()); // bin 파일 경로 사용
                break;
            default: break;
        }
    }

    CL_T2_ConfigManager::getInstance().checkLazyWrite();
    _comm.runNetwork();

    if (_storage.hasIoError()) _storage.attemptRecovery();

    // [v015 이식] 유휴 상태 자동 교정 트리거
    if (_state == T2_Type::SystemState::READY) {
        uint32_t v_nowSec = (uint32_t)(esp_timer_get_time() / 1000000ULL);
        const auto& v_cfg = CL_T2_ConfigManager::getInstance().getConfig();
        if (v_cfg.aud_calib.auto_idle_min > 0 && (v_nowSec - _lastTick > v_cfg.aud_calib.auto_idle_min * 60)) {
            dispatchCommand(T2_Type::SystemCommand::CMD_CALIBRATE);
            _lastTick = v_nowSec;
        }
    }
}

void CL_T2_FsmManager::_captureTask(void* p_param) {
    CL_T2_FsmManager* v_this = (CL_T2_FsmManager*)p_param;
    if (!v_this || !v_this->_rawPool || !v_this->_featurePool) {
        ESP_LOGE(TAG, "CaptureTask Safety Guard: Null Pointer! Suspending.");
        vTaskSuspend(NULL);
    }

    while (1) {
        if (v_this->_state >= T2_Type::SystemState::READY) {
            uint8_t v_idx;
            if (xQueueReceive(v_this->_qFreeIdx, &v_idx, 0)) {
                T2_Type::UnifiedRawChunk& v_raw = v_this->_rawPool[v_idx];
                T2_Type::UnifiedFeatureSlot& v_slot = v_this->_featurePool[v_idx];

                // [중요] 센서 데이터 획득
                const auto& v_cfg = CL_T2_ConfigManager::getInstance().getConfig();
                if (v_cfg.system.vib_enable) {
                    if (v_cfg.vib_sensor.accel_enable || v_cfg.vib_sensor.gyro_enable) {
                        v_this->_sensor.readVibFifoBatch(v_raw.vib[0], v_raw.vib[1], v_raw.vib[2], T2_Def::Vib::Sensor::FFT_SIZE_MAX);
                    }
                }
                if (v_cfg.system.audio_enable) {
                    v_this->_sensor.readAudioChunk(v_raw.audio_l, v_raw.audio_r, T2_Def::Audio::Sensor::FFT_SIZE_MAX);
                }

                // 헤더 정보 기록
                v_slot.header.ts = (uint64_t)time(NULL) * 1000;
                v_slot.header.fid++;
                v_slot.header.uptime = millis();

                // 상태 플래그 취합 (StatusBit)
                uint8_t v_flags = 0;
                if (time(NULL) > 1700000000) v_flags |= (1 << (uint8_t)T2_Type::StatusBit::NTP_SYNCED);
                if (SD_MMC.cardSize() > 0) v_flags |= (1 << (uint8_t)T2_Type::StatusBit::SD_MOUNTED);
                if (v_this->_state == T2_Type::SystemState::RECORDING) v_flags |= (1 << (uint8_t)T2_Type::StatusBit::RECORDING_NOW);
                v_slot.header.flags = v_flags;

                xQueueSend(v_this->_qReadyIdx, &v_idx, 0);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void CL_T2_FsmManager::_processTask(void* p_param) {
    CL_T2_FsmManager* v_this = (CL_T2_FsmManager*)p_param;
    uint8_t v_idx;
    const auto& v_cfg = CL_T2_ConfigManager::getInstance().getConfig();

    while (1) {
        if (xQueueReceive(v_this->_qReadyIdx, &v_idx, pdMS_TO_TICKS(10))) {
            T2_Type::UnifiedFeatureSlot& v_slot = v_this->_featurePool[v_idx];
            T2_Type::UnifiedRawChunk& v_raw     = v_this->_rawPool[v_idx];

            // [v015 이식] PSRAM 데이터를 Internal SRAM 고속 버퍼로 복사하여 연산 가속
            if (v_cfg.system.audio_enable) {
                size_t v_rawBytes = sizeof(float) * T2_Def::Audio::Sensor::FFT_SIZE_MAX;
                if (v_this->_prcBufL && v_raw.audio_l) {
                    memcpy(v_this->_prcBufL, v_raw.audio_l, v_rawBytes);
                }
                if (v_cfg.aud_sensor.channel_mode == T2_Type::AudioChannelMode::STEREO && v_this->_prcBufR && v_raw.audio_r) {
                    memcpy(v_this->_prcBufR, v_raw.audio_r, v_rawBytes);
                }
            }

            // [Step 1] DSP 가공 (Internal SRAM 버퍼 사용)
            if (v_cfg.system.vib_enable) {
                if (v_cfg.vib_sensor.accel_enable) {
                    v_this->_dsp.processAccel(v_raw.vib[0], v_raw.vib[1], v_raw.vib[2], v_raw.vib[0], v_raw.vib[1], v_raw.vib[2], T2_Def::Vib::Sensor::FFT_SIZE_MAX, v_cfg.shared_dsp);
                }
                if (v_cfg.vib_sensor.gyro_enable) {
                    v_this->_dsp.processGyro(v_raw.vib[0], v_raw.vib[1], v_raw.vib[2], v_raw.vib[0], v_raw.vib[1], v_raw.vib[2], T2_Def::Vib::Sensor::FFT_SIZE_MAX, v_cfg.shared_dsp);
                }
            }
            if (v_cfg.system.audio_enable) {
                v_this->_dsp.processAudio(v_this->_prcBufL, v_this->_prcBufR, v_this->_prcBeamformed, T2_Def::Audio::Sensor::FFT_SIZE_MAX, v_cfg.shared_dsp, v_cfg.aud_dsp);
            }

            // [Step 2] 특징 추출
            if (v_cfg.system.vib_enable) {
                v_this->_extractor.extractVibration(v_raw.vib[0], v_raw.vib[1], v_raw.vib[2], T2_Def::Vib::Sensor::FFT_SIZE_MAX, v_cfg.vib_sensor.sample_rate, v_slot, v_cfg.vib_feat, v_cfg.vib_trig, v_cfg.vib_sensor.accel_axis_count);
            }
            if (v_cfg.system.audio_enable) {
                v_this->_extractor.extractAudio(v_this->_prcBeamformed, T2_Def::Audio::Sensor::FFT_SIZE_MAX, v_cfg.aud_sensor.sample_rate, v_slot, v_cfg.aud_feat, v_cfg.aud_trig);
            }

            // [Step 3] 시퀀스 빌딩 (ML용)
            if (v_cfg.system.audio_enable) {
                v_this->_seqBuilder.pushVector(v_slot.tensor.mfcc);
            }

            // [Step 4] 판정 및 저장 로직
            if (v_this->_state == T2_Type::SystemState::MONITORING || v_this->_state == T2_Type::SystemState::RECORDING) {
                T2_Type::DetectionResult v_res = v_this->_trigger.runDiagnostic(v_slot, v_cfg);
                v_this->_handleTriggerResult(v_res, v_slot, v_raw);
            } else if (v_this->_state == T2_Type::SystemState::CALIBRATING) {
                // 교정 모드일 때도 데이터 기록
                v_this->_storage.pushFrame(&v_slot, &v_raw);
            }

            // [Step 5] 실시간 스트리밍 (가변 주율 적용)
            v_this->_broadcastStreams(v_slot, v_raw);

            xQueueSend(v_this->_qFreeIdx, &v_idx, 0);
        }

        v_this->_checkGracefulClose();
        esp_task_wdt_reset();
    }
}

void CL_T2_FsmManager::_broadcastStreams(const T2_Type::UnifiedFeatureSlot& p_slot, const T2_Type::UnifiedRawChunk& p_raw) {
    // 1. Telemetry (상태 및 특징량 요약)
    _accTele += _telemetryHz;
    if (_accTele >= 100) {
        _accTele -= 100;
        _pktTele->header.magic = 0xAA;
        _pktTele->header.type = (uint8_t)T2_Type::StreamType::TELEMETRY;
        _pktTele->header.len  = sizeof(T2_Type::PktTelemetry) - sizeof(T2_Type::WsHeader);
        _pktTele->header.source = 0; // Global
        _pktTele->header.stage = (uint8_t)_state;

        _pktTele->sys_state     = (uint8_t)_state;
        _pktTele->detect_result = (uint8_t)p_slot.header.src; // FIXME: Actual result needed
        _pktTele->trial_no      = p_slot.header.trial;
        _pktTele->trigger_source= p_slot.header.src;
        _pktTele->active_axes   = p_slot.header.active_axes;
        _pktTele->payload_type  = 0; // Mixed

        for(int i=0; i<3; i++) {
            _pktTele->vib_rms[i] = p_slot.vib.rms[i];
            _pktTele->vib_centroid[i] = 0; // FIXME
        }
        _pktTele->vib_kurtosis = p_slot.vib.kurt;
        _pktTele->vib_crest_factor = p_slot.vib.crest;
        _pktTele->vib_sta_lta_ratio = 0;
        memcpy(_pktTele->vib_band_energy, p_slot.vib.band_energy, sizeof(_pktTele->vib_band_energy));

        _pktTele->audio_rms = p_slot.audio.rms;
        _pktTele->audio_energy = p_slot.audio.energy;
        _pktTele->audio_kurtosis = p_slot.audio.kurt;
        _pktTele->audio_crest_factor = p_slot.audio.crest;
        _pktTele->audio_sta_lta_ratio = p_slot.audio.sta_lta;
        _pktTele->audio_skewness = p_slot.audio.skew;
        _pktTele->audio_spectral_centroid = p_slot.audio.centroid;
        memcpy(_pktTele->audio_band_energy, p_slot.audio.band_energy, sizeof(_pktTele->audio_band_energy));
        memcpy(_pktTele->mfcc, p_slot.tensor.mfcc, sizeof(_pktTele->mfcc));

        _comm.broadcastBinary(_pktTele, sizeof(T2_Type::PktTelemetry));
    }

    // 2. Waveform (오디오 + 진동 순차 전송 - v234 정합성)
    _accWave += _waveformHz;
    if (_accWave >= 100) {
        _accWave -= 100;
        // Audio L
        _pktWave->header.magic = 0xAA;
        _pktWave->header.type = (uint8_t)T2_Type::StreamType::WAVEFORM;
        _pktWave->header.source = 1; // Audio
        _pktWave->header.len  = T2_Def::Audio::Sensor::FFT_SIZE_MAX * sizeof(float);
        memcpy(_pktWave->samples, p_raw.audio_l, _pktWave->header.len);
        _comm.broadcastBinary(_pktWave, sizeof(T2_Type::PktWaveform));

        // Vibration (X, Y, Z 순차 전송)
        for (int i = 0; i < 3; i++) {
            _pktWaveVib->header.magic = 0xAA;
            _pktWaveVib->header.type = (uint8_t)T2_Type::StreamType::WAVEFORM;
            _pktWaveVib->header.source = 2 + i; // Vib X, Y, Z
            _pktWaveVib->header.len = T2_Def::Vib::Sensor::FFT_SIZE_MAX * sizeof(float);
            const float* v_srcVib[3] = {p_raw.vib[0], p_raw.vib[1], p_raw.vib[2]};
            memcpy(_pktWaveVib->samples, v_srcVib[i], _pktWaveVib->header.len);
            _comm.broadcastBinary(_pktWaveVib, sizeof(T2_Type::PktWaveformVib));
        }
    }

    // 3. Spectrum (오디오 중심)
    _accSpec += _spectrumHz;
    if (_accSpec >= 100) {
        _accSpec -= 100;
        _pktSpec->header.magic = 0xAA;
        _pktSpec->header.type = (uint8_t)T2_Type::StreamType::SPECTRUM;
        _pktSpec->header.source = 1; // Audio
        _pktSpec->header.len  = ((T2_Def::Audio::Sensor::FFT_SIZE_MAX / 2) + 1) * sizeof(float);
        memcpy(_pktSpec->frequencies, _extractor.getAudioPowerSpectrum(), _pktSpec->header.len);
        _comm.broadcastBinary(_pktSpec, sizeof(T2_Type::PktSpectrum));
    }

    // 4. AI Sequence Tensor (v234 정합성 복구)
    _accSeq += _sequenceHz;
    if (_accSeq >= 100) {
        _accSeq -= 100;
        if (_seqBuilder.isReady()) {
            _pktSeq->header.magic = 0xAA;
            _pktSeq->header.type = (uint8_t)T2_Type::StreamType::SEQUENCE;
            _pktSeq->header.source = 0; // Multi-channel
            size_t v_seqBytes = T2_Def::Global::System::SEQUENCE_FRAMES_MAX * T2_Def::Audio::FeatureLimit::MFCC_DIM_MAX * sizeof(float);
            _pktSeq->header.len = v_seqBytes;
            _seqBuilder.getSequenceFlat(_pktSeq->data, v_seqBytes);
            _comm.broadcastBinary(_pktSeq, sizeof(T2_Type::WsHeader) + v_seqBytes);
        }
    }
}

void CL_T2_FsmManager::_checkGracefulClose() {
    // [v015 이식] Graceful Close: 모든 처리 대기 데이터가 빠질 때까지 대기 후 세션 종료
    if (_stopReasonCmd != 0 && uxQueueMessagesWaiting(_qReadyIdx) == 0) {
        uint8_t v_cmd = _stopReasonCmd;
        if (xQueueSend(_qSessionCmd, &v_cmd, 0) == pdTRUE) {
            _stopReasonCmd = 0;
            if (v_cmd == (uint8_t)T2_Type::AsyncSessionCmd::CLOSE_NORMAL ||
                v_cmd == (uint8_t)T2_Type::AsyncSessionCmd::CLOSE_MANUAL) {
                setState(T2_Type::SystemState::READY);
            }
        }
    }
}

void CL_T2_FsmManager::_handleTriggerResult(T2_Type::DetectionResult p_res, const T2_Type::UnifiedFeatureSlot& p_slot, const T2_Type::UnifiedRawChunk& p_raw) {
    if (p_res != T2_Type::DetectionResult::PASS) {
        // 고장 감지 시 자동 녹화 시작 또는 유지
        if (_state != T2_Type::SystemState::RECORDING) {
            setState(T2_Type::SystemState::RECORDING);
            _storage.openSession("auto_trigger");
        }
        _storage.pushFrame(&p_slot, &p_raw);
    }
    _comm.publishResultMqtt(p_slot, p_res);
}
