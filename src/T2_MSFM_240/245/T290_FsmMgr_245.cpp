/* ============================================================================
 * File: T290_FsmMgr_245.cpp
 * Summary: 4-Tier 시스템 통합 오케스트레이터 구현부
 * ========================================================================== */
#include <cstring>
#include <time.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include <Arduino.h>

#include "T290_FsmMgr_245.hpp"

static const char* TAG = "T245_FSM";

/**
 * @brief 전역 명령 분배기 Wrapper (C 스타일 라이브러리 연동용)
 * @param p_cmd 실행할 시스템 명령
 */
void T240_DispatchCommand(T2_Type::EM_SystemCommand_t p_cmd) { 
    CL_T2_FsmManager::getInstance().dispatchCommand(p_cmd); 
}

/**
 * @brief 전역 시스템 상태 조회 Wrapper
 * @return 현재 상태 정수형 값
 */
uint8_t T240_GetCurrentState() { 
    return (uint8_t)CL_T2_FsmManager::getInstance().getState(); 
}

/**
 * @brief CL_T2_FsmManager 생성자
 */
CL_T2_FsmManager::CL_T2_FsmManager()
    : _sensor(SPI), _state(T2_Type::EM_SystemState_t::INIT) {}

/**
 * @brief CL_T2_FsmManager 소멸자 (동적 메모리 자원 해제)
 */
CL_T2_FsmManager::~CL_T2_FsmManager() {
    // PSRAM 데이터 풀 메모리 해제
    if (_featurePool) heap_caps_free(_featurePool);
    if (_accelPool) heap_caps_free(_accelPool);
    if (_gyroPool) heap_caps_free(_gyroPool);
    if (_audioPool) heap_caps_free(_audioPool);

    // 웹소켓 전송 패킷 메모리 해제
    if (_pktTele) heap_caps_free(_pktTele);
    if (_pktWaveAudio) heap_caps_free(_pktWaveAudio);
    if (_pktWaveAccel) heap_caps_free(_pktWaveAccel);
    if (_pktWaveGyro) heap_caps_free(_pktWaveGyro);
    if (_pktSpec) heap_caps_free(_pktSpec);
    if (_pktSeq) heap_caps_free(_pktSeq);

    // 고속 핫패스 임시 버퍼 메모리 해제
    if (_capBufL) heap_caps_free(_capBufL);
    if (_capBufR) heap_caps_free(_capBufR);
    if (_prcBufL) heap_caps_free(_prcBufL);
    if (_prcBufR) heap_caps_free(_prcBufR);
    if (_prcBeamformed) heap_caps_free(_prcBeamformed);
}

/**
 * @brief 시스템 및 산하 모듈/엔진 초기화
 * @return 초기화 성공 여부
 */
bool CL_T2_FsmManager::init() {
    // RGB LED 초기 상태 노란색 점등
    if (T2_Def::Global::Hardware::PIN_RGB_LED_CONST != T2_Def::Global::Hardware::PIN_NOT_SET_CONST) {
        neopixelWrite(T2_Def::Global::Hardware::PIN_RGB_LED_CONST, 32, 32, 0);
    }
    ESP_LOGI(TAG, "Initializing Orchestrator...");

    // [처리 단위 1] 설정 매니저 및 하위 엔진 초기화
    if (!CL_T2_ConfigManager::getInstance().init()) {
        ESP_LOGE(TAG, "ConfigManager Init Failed!");
        return false;
    }
    const auto& v_cfg = CL_T2_ConfigManager::getInstance().getConfig();

    if (!_sensor.init(v_cfg.system, v_cfg.accel, v_cfg.gyro, v_cfg.audio)) return false;
    if (!_dsp.init(v_cfg)) return false;
    if (!_extractor.init(v_cfg.audio)) return false;
    if (!_storage.init()) return false;
    if (!_comm.init()) return false;

    _seqBuilder.init(T2_Def::Global::System::SEQUENCE_FRAMES_MAX, T2_Def::AI::Tensor::MFCC_DIM_MAX);
    _calibrator.bindExtractor(&_extractor);

    // [처리 단위 2] PSRAM 데이터 풀 동적 할당 (Zero-copy 파이프라인 보장)
    _featurePool = (T2_Type::ST_UnifiedFeatureSlot_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_UnifiedFeatureSlot_t) * 64, MALLOC_CAP_SPIRAM);
    _accelPool   = (T2_Type::ST_Raw_Accel_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_Raw_Accel_t) * 64, MALLOC_CAP_SPIRAM);
    _gyroPool    = (T2_Type::ST_Raw_Gyro_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_Raw_Gyro_t) * 64, MALLOC_CAP_SPIRAM);
    _audioPool   = (T2_Type::ST_Raw_Audio_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_Raw_Audio_t) * 64, MALLOC_CAP_SPIRAM);

    // [처리 단위 3] 실시간 방송을 위한 대용량 웹소켓 패킷 버퍼 PSRAM 할당
    _pktTele      = (T2_Type::ST_PktTelemetry_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_PktTelemetry_t), MALLOC_CAP_SPIRAM);
    _pktWaveAudio = (T2_Type::ST_PktWaveformAudio_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_PktWaveformAudio_t), MALLOC_CAP_SPIRAM);
    _pktWaveAccel = (T2_Type::ST_PktWaveformAccel_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_PktWaveformAccel_t), MALLOC_CAP_SPIRAM);
    _pktWaveGyro  = (T2_Type::ST_PktWaveformGyro_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_PktWaveformGyro_t), MALLOC_CAP_SPIRAM);
    _pktSpec      = (T2_Type::ST_PktSpectrum_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_PktSpectrum_t), MALLOC_CAP_SPIRAM);
    _pktSeq       = (T2_Type::ST_PktSequence_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_PktSequence_t), MALLOC_CAP_SPIRAM);

    _telemetryHz = v_cfg.system.tele_hz;
    _waveformHz  = v_cfg.system.wave_hz;
    _sequenceHz  = 2; // AI 시퀀스는 2Hz 주기로 전송 고정

    // [처리 단위 4] 연산 가속을 위한 Internal SRAM 고속 버퍼 할당
    size_t v_fSize = sizeof(float) * T2_Def::Audio::Sensor::FFT_SIZE_MAX;
    _capBufL = nullptr;
    _capBufR = nullptr;
    _prcBufL = nullptr;
    _prcBufR = nullptr;
    _prcBeamformed = nullptr;

    if (v_cfg.audio.enable) {
        _capBufL = (float*)heap_caps_aligned_alloc(16, v_fSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        _prcBufL = (float*)heap_caps_aligned_alloc(16, v_fSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        _capBufR = (float*)heap_caps_aligned_alloc(16, v_fSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        _prcBufR = (float*)heap_caps_aligned_alloc(16, v_fSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        _prcBeamformed = (float*)heap_caps_aligned_alloc(16, v_fSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }

    // 할당 확인 및 예외 처리
    bool v_allocSuccess = _featurePool && _accelPool && _gyroPool && _audioPool &&
                          _pktTele && _pktWaveAudio && _pktWaveAccel && _pktWaveGyro && _pktSpec && _pktSeq;
    if (v_cfg.audio.enable) {
        v_allocSuccess &= (_capBufL != nullptr) && (_prcBufL != nullptr) && (_capBufR != nullptr) && (_prcBufR != nullptr) && (_prcBeamformed != nullptr);
    }

    if (!v_allocSuccess) {
        ESP_LOGE(TAG, "Critical Allocation Failed! Check SRAM/PSRAM Heap.");
        return false;
    }

    // [처리 단위 5] 비동기 데이터 파이프라인을 위한 FreeRTOS 통신 큐 생성
    _qFreeIdx  = xQueueCreate(64, sizeof(uint8_t));
    _qReadyIdx = xQueueCreate(64, sizeof(uint8_t));
    _qSessionCmd = xQueueCreate(16, sizeof(uint8_t));

    // 사용 가능한 인덱스 자원을 Free 큐에 채움
    for (uint8_t i = 0; i < 64; i++) xQueueSend(_qFreeIdx, &i, 0);

    // [처리 단위 6] Core 0 (수집) 및 Core 1 (신호처리) 멀티코어 태스크 구동
    xTaskCreatePinnedToCore(_captureTask, "CapTask", 8192, this, 10, &_hCaptureTask, 0);
    xTaskCreatePinnedToCore(_processTask, "PrcTask", 16384, this, 5, &_hProcessTask, 1);

    setState(T2_Type::EM_SystemState_t::READY);
    ESP_LOGI(TAG, "Orchestrator Ready.");
    return true;
}

/**
 * @brief 시스템 상태(FSM) 안전 전이 및 이에 따른 RGB LED / 제어 변수 연동 설정
 * @param p_newState 신규 상태
 */
void CL_T2_FsmManager::setState(T2_Type::EM_SystemState_t p_newState) {
    portENTER_CRITICAL(&_stateMux);
    if (_state == p_newState) { portEXIT_CRITICAL(&_stateMux); return; }
    T2_Type::EM_SystemState_t v_prevState = _state;
    _state = p_newState;
    portEXIT_CRITICAL(&_stateMux);

    ESP_LOGI(TAG, "System State Changed: %d -> %d", (int)v_prevState, (int)p_newState);

    // [처리 단위 1] 신규 상태에 따른 NeoPixel RGB LED 디스플레이 연동
    if (T2_Def::Global::Hardware::PIN_RGB_LED_CONST != T2_Def::Global::Hardware::PIN_NOT_SET_CONST) {
        uint8_t r = 0, g = 0, b = 0;
        switch (p_newState) {
            case T2_Type::EM_SystemState_t::INIT:            r = 32;  g = 32;  b = 0;   break; // 노란색
            case T2_Type::EM_SystemState_t::READY:           r = 0;   g = 64;  b = 0;   break; // 초록색
            case T2_Type::EM_SystemState_t::MONITORING:      r = 0;   g = 64;  b = 64;  break; // 하늘색
            case T2_Type::EM_SystemState_t::RECORDING:       r = 64;  g = 0;   b = 0;   break; // 빨간색
            case T2_Type::EM_SystemState_t::NOISE_LEARNING:  r = 64;  g = 0;   b = 64;  break; // 보라색
            case T2_Type::EM_SystemState_t::MAINTENANCE:     r = 32;  g = 32;  b = 32;  break; // 흰색
            case T2_Type::EM_SystemState_t::ERROR:           r = 128; g = 0;   b = 0;   break; // 빨간색 (에러)
            case T2_Type::EM_SystemState_t::CALIBRATING:     r = 64;  g = 32;  b = 0;   break; // 주황색
            default: break;
        }
        neopixelWrite(T2_Def::Global::Hardware::PIN_RGB_LED_CONST, r, g, b);
    }

    // [처리 단위 2] 상태 전이에 따른 부가 변수 및 버퍼 리셋 처리
    if (p_newState == T2_Type::EM_SystemState_t::READY) {
        _lastTick = (uint32_t)(esp_timer_get_time() / 1000000ULL);
        _currentTrial = 0;
        _isManualRecording = false;
    }
    else if (p_newState == T2_Type::EM_SystemState_t::MAINTENANCE) {
        _storage.closeSession("ota_maintenance");
        ESP_LOGW(TAG, "System entering MAINTENANCE mode. Bus isolated.");
    }
    else if (p_newState == T2_Type::EM_SystemState_t::MONITORING || p_newState == T2_Type::EM_SystemState_t::RECORDING) {
        if (v_prevState == T2_Type::EM_SystemState_t::READY) {
            _dsp.resetStates();
            _seqBuilder.reset();
        }
    }
}

/**
 * @brief 전역 명령 수신 인터페이스 및 명령별 세션 실행
 * @param p_cmd 시스템 명령
 */
void CL_T2_FsmManager::dispatchCommand(T2_Type::EM_SystemCommand_t p_cmd) {
    if (_state == T2_Type::EM_SystemState_t::MAINTENANCE &&
        p_cmd != T2_Type::EM_SystemCommand_t::CMD_REBOOT) return;

    ESP_LOGI(TAG, "Command Dispatched: %d", (uint8_t)p_cmd);
    uint8_t v_sessCmd = 0;

    switch (p_cmd) {
        case T2_Type::EM_SystemCommand_t::CMD_START:
            setState(T2_Type::EM_SystemState_t::MONITORING);
            break;
        case T2_Type::EM_SystemCommand_t::CMD_STOP:
            _stopReasonCmd = (uint8_t)T2_Type::EM_AsyncSessionCmd_t::CLOSE_NORMAL;
            break;
        case T2_Type::EM_SystemCommand_t::CMD_MANUAL_REC_START:
            if (_state == T2_Type::EM_SystemState_t::READY || _state == T2_Type::EM_SystemState_t::MONITORING) {
                _isManualRecording = true;
                setState(T2_Type::EM_SystemState_t::RECORDING);
                v_sessCmd = (uint8_t)T2_Type::EM_AsyncSessionCmd_t::OPEN_MANUAL;
                xQueueSend(_qSessionCmd, &v_sessCmd, 0);
            }
            break;
        case T2_Type::EM_SystemCommand_t::CMD_MANUAL_REC_STOP:
            if (_state == T2_Type::EM_SystemState_t::RECORDING && _isManualRecording) {
                _isManualRecording = false;
                _stopReasonCmd = (uint8_t)T2_Type::EM_AsyncSessionCmd_t::CLOSE_MANUAL;
            }
            break;
        case T2_Type::EM_SystemCommand_t::CMD_LEARN_NOISE:
            _extractor.setNoiseLearning(true);
            break;
        case T2_Type::EM_SystemCommand_t::CMD_CALIBRATE:
            if (_state == T2_Type::EM_SystemState_t::READY) {
                setState(T2_Type::EM_SystemState_t::CALIBRATING);
                v_sessCmd = (uint8_t)T2_Type::EM_AsyncSessionCmd_t::OPEN_CALIB_MAN;
                xQueueSend(_qSessionCmd, &v_sessCmd, 0);
            }
            break;
        case T2_Type::EM_SystemCommand_t::CMD_REBOOT:
            _storage.closeSession("reboot");
            vTaskDelay(pdMS_TO_TICKS(500));
            esp_restart();
            break;
        default: break;
    }
}

/**
 * @brief 메인 루프 백그라운드 관리 함수 (NTP 및 슬립 감시, 지연 스토리지 기록)
 */
void CL_T2_FsmManager::runMaintenance() {
    uint8_t v_cmd;
    // [처리 단위 1] 비동기 세션 시작/종료 명령 큐 처리
    while (xQueueReceive(_qSessionCmd, &v_cmd, 0) == pdTRUE) {
        switch ((T2_Type::EM_AsyncSessionCmd_t)v_cmd) {
            case T2_Type::EM_AsyncSessionCmd_t::OPEN_AUTO:       _storage.openSession("trg_auto"); break;
            case T2_Type::EM_AsyncSessionCmd_t::OPEN_MANUAL:     _storage.openSession("man"); break;
            case T2_Type::EM_AsyncSessionCmd_t::OPEN_CALIB_MAN:  _storage.openSession("calib_man"); break;
            case T2_Type::EM_AsyncSessionCmd_t::CLOSE_NORMAL:    _storage.closeSession("auto_end"); break;
            case T2_Type::EM_AsyncSessionCmd_t::CLOSE_MANUAL:    _storage.closeSession("man_end"); break;
            case T2_Type::EM_AsyncSessionCmd_t::CLOSE_CALIB_DONE:
                _storage.closeSession("calib_done");
                _calibrator.startManualCalibration(_storage.getLastPath());
                break;
            default: break;
        }
    }

    // [처리 단위 2] 지연 파일 기록 및 네트워크 루프 수행, 스토리지 복구
    CL_T2_ConfigManager::getInstance().checkLazyWrite();
    _comm.runNetwork();

    if (_storage.hasIoError()) _storage.attemptRecovery();

    // [처리 단위 3] 유휴(Idle) 상태 자동 캘리브레이션 트리거 검사
    if (_state == T2_Type::EM_SystemState_t::READY) {
        uint32_t v_nowSec = (uint32_t)(esp_timer_get_time() / 1000000ULL);
        const auto& v_cfg = CL_T2_ConfigManager::getInstance().getConfig();
        if (v_cfg.audio.auto_idle_min > 0 && (v_nowSec - _lastTick > v_cfg.audio.auto_idle_min * 60)) {
            dispatchCommand(T2_Type::EM_SystemCommand_t::CMD_CALIBRATE);
            _lastTick = v_nowSec;
        }

        // [처리 단위 4] 저전력 딥슬립 전환 감시
        if (v_cfg.trig_use_sleep && v_cfg.trig_sleep_sec > 0 && (v_nowSec - _lastTick > v_cfg.trig_sleep_sec)) {
            ESP_LOGW(TAG, "Entering Deep Sleep... Idle for %u sec", (unsigned int)v_cfg.trig_sleep_sec);
            _storage.closeSession("deep_sleep");
            if (T2_Def::Global::Hardware::PIN_RGB_LED_CONST != T2_Def::Global::Hardware::PIN_NOT_SET_CONST) {
                neopixelWrite(T2_Def::Global::Hardware::PIN_RGB_LED_CONST, 0, 0, 0);
            }
            vTaskDelay(pdMS_TO_TICKS(100));

            // 제어 버튼(PIN_BTN_CONTROL_CONST) 눌림 시(LOW) 기상하도록 ext0 등록
            esp_sleep_enable_ext0_wakeup((gpio_num_t)T2_Def::Global::Hardware::PIN_BTN_CONTROL_CONST, 0);
            esp_deep_sleep_start();
        }
    }
}

/**
 * @brief 센서 원시 데이터 수집 전담 Capture Task (Core 0 실행)
 * @param p_param FsmManager 인스턴스 포인터
 */
void CL_T2_FsmManager::_captureTask(void* p_param) {
    CL_T2_FsmManager* v_this = (CL_T2_FsmManager*)p_param;
    if (!v_this || !v_this->_accelPool || !v_this->_gyroPool || !v_this->_audioPool || !v_this->_featurePool) {
        ESP_LOGE(TAG, "CaptureTask Safety Guard: Null Pointer! Suspending.");
        vTaskSuspend(NULL);
    }

    while (1) {
        if (v_this->_state >= T2_Type::EM_SystemState_t::READY) {
            const auto& v_cfg = CL_T2_ConfigManager::getInstance().getConfig();

            // 1. 센서 하드웨어 FIFO 버퍼 누적
            if (v_cfg.accel.enable || v_cfg.gyro.enable) {
                v_this->_sensor.accumulateFifo();
            }

            // 2. 수집 조건(진동/자이로 FFT 크기 이상 누적 여부) 검사
            bool v_triggerCapture = false;
            if (v_cfg.accel.enable || v_cfg.gyro.enable) {
                uint16_t v_accCount = v_this->_sensor.getAccumulatedAccelCount();
                uint16_t v_gyrCount = v_this->_sensor.getAccumulatedGyroCount();
                uint16_t v_targetReq = T2_Def::Accel::Sensor::FFT_SIZE_MAX;

                if (v_cfg.accel.enable && v_accCount >= v_targetReq) {
                    v_triggerCapture = true;
                } else if (v_cfg.gyro.enable && !v_cfg.accel.enable && v_gyrCount >= v_targetReq) {
                    v_triggerCapture = true;
                }
            } else if (v_cfg.audio.enable) {
                v_triggerCapture = true;
            }

            // [처리 단위 3] 데이터 버퍼 인출 및 패킷화하여 처리 Task로 전달
            if (v_triggerCapture) {
                uint8_t v_idx;
                if (xQueueReceive(v_this->_qFreeIdx, &v_idx, 0) == pdTRUE) {
                    T2_Type::ST_Raw_Accel_t& v_rawAcc = v_this->_accelPool[v_idx];
                    T2_Type::ST_Raw_Gyro_t&  v_rawGyr = v_this->_gyroPool[v_idx];
                    T2_Type::ST_Raw_Audio_t& v_rawAud = v_this->_audioPool[v_idx];
                    T2_Type::ST_UnifiedFeatureSlot_t& v_slot = v_this->_featurePool[v_idx];

                    // 가속도 데이터 추출 및 메타 입력
                    if (v_cfg.accel.enable || v_cfg.gyro.enable) {
                        uint16_t v_req = T2_Def::Accel::Sensor::FFT_SIZE_MAX;
                        if (v_cfg.accel.enable) {
                            v_this->_sensor.getAccumulatedAccel(v_rawAcc.data[0], v_rawAcc.data[1], v_rawAcc.data[2], v_req);
                            v_rawAcc.ts = (uint64_t)time(NULL) * 1000;
                            v_rawAcc.sample_rate = v_cfg.accel.sample_rate;
                            v_rawAcc.active_mask = v_cfg.accel.axis_mask;
                        } else {
                            memset(v_rawAcc.data, 0, sizeof(v_rawAcc.data));
                        }

                        if (v_cfg.gyro.enable) {
                            v_this->_sensor.getAccumulatedGyro(v_rawGyr.data[0], v_rawGyr.data[1], v_rawGyr.data[2], v_req);
                            v_rawGyr.ts = (uint64_t)time(NULL) * 1000;
                            v_rawGyr.sample_rate = v_cfg.gyro.sample_rate;
                            v_rawGyr.active_mask = v_cfg.gyro.axis_mask;
                        } else {
                            memset(v_rawGyr.data, 0, sizeof(v_rawGyr.data));
                        }
                    }

                    // 오디오 데이터 읽기
                    if (v_cfg.audio.enable) {
                        v_this->_sensor.readAudioChunk(v_rawAud.data[0], v_rawAud.data[1], T2_Def::Audio::Sensor::FFT_SIZE_MAX);
                        v_rawAud.ts = (uint64_t)time(NULL) * 1000;
                        v_rawAud.sample_rate = v_cfg.audio.sample_rate;
                        v_rawAud.active_mask = v_cfg.audio.channel_mask;
                    }

                    // 통합 슬롯 공통 헤더 기록
                    v_slot.header.ts = (uint64_t)time(NULL) * 1000;
                    v_slot.header.fid++;
                    v_slot.header.uptime = millis();
                    v_slot.header.accel_mask = v_cfg.accel.axis_mask;
                    v_slot.header.gyro_mask  = v_cfg.gyro.axis_mask;
                    v_slot.header.audio_mask = v_cfg.audio.channel_mask;
                    v_slot.header.payload_type = (uint8_t)T2_Type::EM_DataPayloadType_t::VIB_AUDIO_BOTH;
                    v_slot.header.temp = v_this->_sensor.readTemperatureSensor();

                    // 상태 비트 조합
                    uint8_t v_flags = 0;
                    if (time(NULL) > 1700000000) v_flags |= (1 << (uint8_t)T2_Type::EM_StatusBit_t::NTP_SYNCED);
                    if (SD_MMC.cardSize() > 0) v_flags |= (1 << (uint8_t)T2_Type::EM_StatusBit_t::SD_MOUNTED);
                    if (v_this->_state == T2_Type::EM_SystemState_t::RECORDING) v_flags |= (1 << (uint8_t)T2_Type::EM_StatusBit_t::RECORDING_NOW);
                    v_slot.header.flags = v_flags;

                    xQueueSend(v_this->_qReadyIdx, &v_idx, 0);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/**
 * @brief 수집 데이터 분석 및 특징 추출 전담 Processing Task (Core 1 실행)
 * @param p_param FsmManager 인스턴스 포인터
 */
void CL_T2_FsmManager::_processTask(void* p_param) {
    CL_T2_FsmManager* v_this = (CL_T2_FsmManager*)p_param;
    uint8_t v_idx;
    const auto& v_cfg = CL_T2_ConfigManager::getInstance().getConfig();

    while (1) {
        if (xQueueReceive(v_this->_qReadyIdx, &v_idx, pdMS_TO_TICKS(10))) {
            T2_Type::ST_UnifiedFeatureSlot_t& v_slot = v_this->_featurePool[v_idx];
            T2_Type::ST_Raw_Accel_t& v_rawAcc     = v_this->_accelPool[v_idx];
            T2_Type::ST_Raw_Gyro_t&  v_rawGyr     = v_this->_gyroPool[v_idx];
            T2_Type::ST_Raw_Audio_t& v_rawAud     = v_this->_audioPool[v_idx];

            // [처리 단위 1] PSRAM 데이터 -> 고속 SRAM 임시 버퍼 복사
            if (v_cfg.audio.enable) {
                size_t v_rawBytes = sizeof(float) * T2_Def::Audio::Sensor::FFT_SIZE_MAX;
                if (v_this->_prcBufL && v_rawAud.data[0]) {
                    memcpy(v_this->_prcBufL, v_rawAud.data[0], v_rawBytes);
                }
                if (v_this->_prcBufR && v_rawAud.data[1]) {
                    memcpy(v_this->_prcBufR, v_rawAud.data[1], v_rawBytes);
                }
            }

            // [처리 단위 2] DSP 필터링 파이프라인 수행 (SRAM 복사본 대상)
            if (v_cfg.accel.enable) {
                v_this->_dsp.processAccel(v_rawAcc.data[0], v_rawAcc.data[1], v_rawAcc.data[2],
                                          v_rawAcc.data[0], v_rawAcc.data[1], v_rawAcc.data[2],
                                          T2_Def::Accel::Sensor::FFT_SIZE_MAX, v_cfg.accel);
            }
            if (v_cfg.gyro.enable) {
                v_this->_dsp.processGyro(v_rawGyr.data[0], v_rawGyr.data[1], v_rawGyr.data[2],
                                         v_rawGyr.data[0], v_rawGyr.data[1], v_rawGyr.data[2],
                                         T2_Def::Gyro::Sensor::FFT_SIZE_MAX, v_cfg.gyro);
            }
            if (v_cfg.audio.enable) {
                v_this->_dsp.processAudio(v_this->_prcBufL, v_this->_prcBufR,
                                          v_this->_prcBufL, v_this->_prcBufR,
                                          T2_Def::Audio::Sensor::FFT_SIZE_MAX, v_cfg.audio);
            }

            // [처리 단위 3] 타깃 특징 추출 (RMS, 크레스트 팩터, MFCC 등)
            if (v_cfg.accel.enable) {
                v_this->_extractor.extractAccel(v_rawAcc.data[0], v_rawAcc.data[1], v_rawAcc.data[2],
                                                T2_Def::Accel::Sensor::FFT_SIZE_MAX, v_cfg.accel.sample_rate,
                                                v_slot, v_cfg.accel);
            }
            if (v_cfg.gyro.enable) {
                v_this->_extractor.extractGyro(v_rawGyr.data[0], v_rawGyr.data[1], v_rawGyr.data[2],
                                               T2_Def::Gyro::Sensor::FFT_SIZE_MAX, v_cfg.gyro.sample_rate,
                                                v_slot, v_cfg.gyro);
            }
            if (v_cfg.audio.enable) {
                v_this->_extractor.extractAudio(v_this->_prcBufL, v_this->_prcBufR,
                                                T2_Def::Audio::Sensor::FFT_SIZE_MAX, v_cfg.audio.sample_rate,
                                                v_slot, v_cfg.audio);
            }

            // [처리 단위 4] 특징 벡터 시퀀스 누적 push
            if (v_cfg.audio.enable) {
                v_this->_seqBuilder.pushVector(v_slot.tensor.mfcc);
            }

            // [처리 단위 5] 감지 임계 구역 판정 및 SD카드 파일 저장
            if (v_this->_state == T2_Type::EM_SystemState_t::MONITORING || v_this->_state == T2_Type::EM_SystemState_t::RECORDING) {
                T2_Type::EM_DetectionResult_t v_res = v_this->_trigger.runDiagnostic(v_slot, v_cfg);
                v_this->_handleTriggerResult(v_res, v_slot, &v_rawAcc, &v_rawGyr, &v_rawAud);
            } else if (v_this->_state == T2_Type::EM_SystemState_t::CALIBRATING) {
                v_this->_storage.pushFrame(&v_slot, &v_rawAcc, &v_rawGyr, &v_rawAud);
            }

            // [처리 단위 6] 실시간 웹소켓 방송 송신
            v_this->_broadcastStreams(v_slot, v_rawAcc, v_rawGyr, v_rawAud);

            xQueueSend(v_this->_qFreeIdx, &v_idx, 0);
        }

        v_this->_checkGracefulClose();
        esp_task_wdt_reset();
    }
}

/**
 * @brief 웹소켓용 스트림 데이터 패키징 및 송출
 * @param p_slot 통합 특성 데이터 슬롯 포인터
 * @param p_rawAcc 가속도 raw 데이터 참조
 * @param p_rawGyr 자이로 raw 데이터 참조
 * @param p_rawAud 오디오 raw 데이터 참조
 */
void CL_T2_FsmManager::_broadcastStreams(const T2_Type::ST_UnifiedFeatureSlot_t& p_slot,
                                         const T2_Type::ST_Raw_Accel_t& p_rawAcc,
                                         const T2_Type::ST_Raw_Gyro_t& p_rawGyr,
                                         const T2_Type::ST_Raw_Audio_t& p_rawAud) {
    const auto& v_cfg = CL_T2_ConfigManager::getInstance().getConfig();

    // 1. 텔레메트리 스트리밍 (Hz 스케줄링)
    _accTele += _telemetryHz;
    if (_accTele >= 100) {
        _accTele -= 100;
        _pktTele->header.magic = 0xAA;
        _pktTele->header.type = (uint8_t)T2_Type::EM_StreamType_t::TELEMETRY;
        _pktTele->header.len  = sizeof(T2_Type::ST_PktTelemetry_t) - sizeof(T2_Type::ST_WsHeader_t);
        _pktTele->header.source = 0;
        _pktTele->header.stage = (uint8_t)_state;

        _pktTele->sys_state     = (uint8_t)_state;
        _pktTele->detect_result = (uint8_t)p_slot.header.src;
        _pktTele->trial_no      = p_slot.header.trial;
        _pktTele->trigger_source= p_slot.header.src;
        _pktTele->accel_mask    = p_slot.header.accel_mask;
        _pktTele->gyro_mask     = p_slot.header.gyro_mask;
        _pktTele->audio_mask    = p_slot.header.audio_mask;
        _pktTele->payload_type  = p_slot.header.payload_type;

        // 가속도 RMS 및 스펙트럼 밴드 에너지 복사
        for (int i = 0; i < 3; i++) {
            _pktTele->accel_rms[i] = p_slot.accel.rms[i];
            _pktTele->accel_centroid[i] = p_slot.accel.centroid[i];
            _pktTele->accel_kurtosis[i] = p_slot.accel.kurt[i];
            _pktTele->accel_crest_factor[i] = p_slot.accel.crest[i];
            _pktTele->accel_sta_lta_ratio[i] = p_slot.accel.sta_lta[i];
            _pktTele->accel_skewness[i] = p_slot.accel.skew[i];
            _pktTele->accel_std[i] = p_slot.accel.std[i];
            memcpy(_pktTele->accel_band_energy[i], p_slot.accel.band_energy[i], sizeof(_pktTele->accel_band_energy[i]));
        }

        // 자이로 특징값 복사
        for (int i = 0; i < 3; i++) {
            _pktTele->gyro_rms[i] = p_slot.gyro.rms[i];
            _pktTele->gyro_centroid[i] = p_slot.gyro.centroid[i];
            _pktTele->gyro_kurtosis[i] = p_slot.gyro.kurt[i];
            _pktTele->gyro_crest_factor[i] = p_slot.gyro.crest[i];
            _pktTele->gyro_sta_lta_ratio[i] = p_slot.gyro.sta_lta[i];
            _pktTele->gyro_skewness[i] = p_slot.gyro.skew[i];
            _pktTele->gyro_std[i] = p_slot.gyro.std[i];
            _pktTele->gyro_drift_est[i] = p_slot.gyro.drift_est[i];
            memcpy(_pktTele->gyro_band_energy[i], p_slot.gyro.band_energy[i], sizeof(_pktTele->gyro_band_energy[i]));
        }

        // 오디오 특징 및 피크 데이터 복사
        for (int ch = 0; ch < 2; ch++) {
            _pktTele->audio_rms[ch] = p_slot.audio.ch[ch].rms;
            _pktTele->audio_energy[ch] = p_slot.audio.ch[ch].energy;
            _pktTele->audio_kurtosis[ch] = p_slot.audio.ch[ch].kurt;
            _pktTele->audio_crest_factor[ch] = p_slot.audio.ch[ch].crest;
            _pktTele->audio_sta_lta_ratio[ch] = p_slot.audio.ch[ch].sta_lta;
            _pktTele->audio_skewness[ch] = p_slot.audio.ch[ch].skew;
            _pktTele->audio_spectral_centroid[ch] = p_slot.audio.ch[ch].centroid;
            memcpy(_pktTele->audio_band_energy[ch], p_slot.audio.ch[ch].band_energy, sizeof(_pktTele->audio_band_energy[ch]));

            for (int p = 0; p < T2_Def::Audio::FeatureLimit::TOP_PEAKS_MAX; p++) {
                _pktTele->audio_peak_freqs[ch][p] = p_slot.audio.ch[ch].top_peaks[p].freq;
                _pktTele->audio_peak_amps[ch][p] = p_slot.audio.ch[ch].top_peaks[p].amp;
            }

            for (int c = 0; c < T2_Def::Audio::FeatureLimit::CEPS_TARGET_MAX; c++) {
                _pktTele->audio_cpsr_max[ch][c] = p_slot.audio.ch[ch].cpsr_max[c];
                _pktTele->audio_cpsr_mxrms[ch][c] = p_slot.audio.ch[ch].cpsr_mxr[c];
            }
        }

        _pktTele->audio_coh = p_slot.audio.coh;
        _pktTele->audio_ipd = p_slot.audio.ipd;
        _pktTele->accel_ts  = p_rawAcc.ts;
        _pktTele->gyro_ts   = p_rawGyr.ts;
        _pktTele->audio_ts  = p_rawAud.ts;

        memcpy(_pktTele->mfcc, p_slot.tensor.mfcc, sizeof(_pktTele->mfcc));
        _comm.broadcastBinary(_pktTele, sizeof(T2_Type::ST_PktTelemetry_t));
    }

    // 2. 파형(Waveform) 스트리밍 (Hz 스케줄링)
    _accWave += _waveformHz;
    if (_accWave >= 100) {
        _accWave -= 100;

        // 오디오 좌/우 채널 파형 브로드캐스트
        if (v_cfg.audio.enable) {
            _pktWaveAudio->header.magic = 0xAA;
            _pktWaveAudio->header.type = (uint8_t)T2_Type::EM_StreamType_t::WAVEFORM;
            _pktWaveAudio->header.source = 1; // Left
            _pktWaveAudio->header.len = T2_Def::Audio::Sensor::FFT_SIZE_MAX * sizeof(float);
            memcpy(_pktWaveAudio->samples, p_rawAud.data[0], _pktWaveAudio->header.len);
            _comm.broadcastBinary(_pktWaveAudio, sizeof(T2_Type::ST_PktWaveformAudio_t));

            if (v_cfg.audio.channel_mask & (uint8_t)T2_Type::EM_ChannelMask_t::CH_RIGHT) {
                _pktWaveAudio->header.magic = 0xAA;
                _pktWaveAudio->header.type = (uint8_t)T2_Type::EM_StreamType_t::WAVEFORM;
                _pktWaveAudio->header.source = 5; // Right
                _pktWaveAudio->header.len = T2_Def::Audio::Sensor::FFT_SIZE_MAX * sizeof(float);
                memcpy(_pktWaveAudio->samples, p_rawAud.data[1], _pktWaveAudio->header.len);
                _comm.broadcastBinary(_pktWaveAudio, sizeof(T2_Type::ST_PktWaveformAudio_t));
            }
        }

        // 가속도 3축 파형 브로드캐스트
        if (v_cfg.accel.enable) {
            for (int i = 0; i < 3; i++) {
                if (v_cfg.accel.axis_mask & (1 << i)) {
                    _pktWaveAccel->header.magic = 0xAA;
                    _pktWaveAccel->header.type = (uint8_t)T2_Type::EM_StreamType_t::WAVEFORM;
                    _pktWaveAccel->header.source = 2 + i;
                    _pktWaveAccel->header.len = T2_Def::Accel::Sensor::FFT_SIZE_MAX * sizeof(float);
                    memcpy(_pktWaveAccel->samples, p_rawAcc.data[i], _pktWaveAccel->header.len);
                    _comm.broadcastBinary(_pktWaveAccel, sizeof(T2_Type::ST_PktWaveformAccel_t));
                }
            }
        }

        // 자이로 3축 파형 브로드캐스트
        if (v_cfg.gyro.enable) {
            for (int i = 0; i < 3; i++) {
                if (v_cfg.gyro.axis_mask & (1 << i)) {
                    _pktWaveGyro->header.magic = 0xAA;
                    _pktWaveGyro->header.type = (uint8_t)T2_Type::EM_StreamType_t::WAVEFORM;
                    _pktWaveGyro->header.source = 6 + i;
                    _pktWaveGyro->header.len = T2_Def::Gyro::Sensor::FFT_SIZE_MAX * sizeof(float);
                    memcpy(_pktWaveGyro->samples, p_rawGyr.data[i], _pktWaveGyro->header.len);
                    _comm.broadcastBinary(_pktWaveGyro, sizeof(T2_Type::ST_PktWaveformGyro_t));
                }
            }
        }
    }

    // 3. 파워 스펙트럼(Spectrum) 스트리밍 (Hz 스케줄링)
    _accSpec += _spectrumHz;
    if (_accSpec >= 100) {
        _accSpec -= 100;
        _pktSpec->header.magic = 0xAA;
        _pktSpec->header.type = (uint8_t)T2_Type::EM_StreamType_t::SPECTRUM;
        _pktSpec->header.source = 1;
        _pktSpec->header.len  = ((T2_Def::Audio::Sensor::FFT_SIZE_MAX / 2) + 1) * sizeof(float);
        memcpy(_pktSpec->frequencies, _extractor.getAudioPowerSpectrum(), _pktSpec->header.len);
        _comm.broadcastBinary(_pktSpec, sizeof(T2_Type::ST_PktSpectrum_t));
    }

    // 4. AI 시퀀스 데이터 스트리밍 (Hz 스케줄링)
    _accSeq += _sequenceHz;
    if (_accSeq >= 100) {
        _accSeq -= 100;
        if (_seqBuilder.isReady()) {
            _pktSeq->header.magic = 0xAA;
            _pktSeq->header.type = (uint8_t)T2_Type::EM_StreamType_t::SEQUENCE;
            _pktSeq->header.source = 0;
            size_t v_seqBytes = T2_Def::Global::System::SEQUENCE_FRAMES_MAX * T2_Def::AI::Tensor::MFCC_DIM_MAX * sizeof(float);
            _pktSeq->header.len = v_seqBytes;
            _seqBuilder.getSequenceFlat(_pktSeq->data, v_seqBytes);
            _comm.broadcastBinary(_pktSeq, sizeof(T2_Type::ST_WsHeader_t) + v_seqBytes);
        }
    }
}

/**
 * @brief 정상 종료 조건에 따라 비동기 세션을 정상 종료하는 함수
 */
void CL_T2_FsmManager::_checkGracefulClose() {
    if (_stopReasonCmd != 0 && uxQueueMessagesWaiting(_qReadyIdx) == 0) {
        uint8_t v_cmd = _stopReasonCmd;
        if (xQueueSend(_qSessionCmd, &v_cmd, 0) == pdTRUE) {
            _stopReasonCmd = 0;
            if (v_cmd == (uint8_t)T2_Type::EM_AsyncSessionCmd_t::CLOSE_NORMAL ||
                v_cmd == (uint8_t)T2_Type::EM_AsyncSessionCmd_t::CLOSE_MANUAL) {
                setState(T2_Type::EM_SystemState_t::READY);
            }
        }
    }
}

/**
 * @brief 트리거 판정 결과에 따른 파일 기록 시작 또는 MQTT 결과 퍼블리싱 처리
 * @param p_res 판정 결과
 * @param p_slot 특징 데이터 슬롯
 * @param p_rawAcc 원시 가속도 포인터
 * @param p_rawGyr 원시 자이로 포인터
 * @param p_rawAud 원시 오디오 포인터
 */
void CL_T2_FsmManager::_handleTriggerResult(T2_Type::EM_DetectionResult_t p_res,
                                            const T2_Type::ST_UnifiedFeatureSlot_t& p_slot,
                                            const T2_Type::ST_Raw_Accel_t* p_rawAcc,
                                            const T2_Type::ST_Raw_Gyro_t* p_rawGyr,
                                            const T2_Type::ST_Raw_Audio_t* p_rawAud) {
    if (p_res != T2_Type::EM_DetectionResult_t::PASS) {
        if (_state != T2_Type::EM_SystemState_t::RECORDING) {
            setState(T2_Type::EM_SystemState_t::RECORDING);
            _storage.openSession("auto_trigger");
        }
        _storage.pushFrame(&p_slot, p_rawAcc, p_rawGyr, p_rawAud);
    }
    _comm.publishResultMqtt(p_slot, p_res);
}
