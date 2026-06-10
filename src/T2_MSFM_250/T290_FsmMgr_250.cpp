/* ============================================================================
 * File: T290_FsmMgr_250.cpp
 * Summary: 4-Tier 시스템 통합 오케스트레이터 구현부
 * ========================================================================== */
#include "T290_FsmMgr_250.hpp"
#include <cstring>
#include <time.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include <Arduino.h>

void SafetyLifecycleManager::processManualReset() {
    _is_alarm_latched = false;
    std::atomic_thread_fence(std::memory_order_release);
    asm volatile("memw"); // 멀티코어 메모리 배리어 동기화
}

static const char* TAG = "T290_FSM";

static volatile TaskHandle_t g_isr_imu_task = nullptr;

static void IRAM_ATTR T245_bmi_watermark_isr() {
    BaseType_t woken = pdFALSE;
    uint32_t ccount = esp_cpu_get_ccount();
    static volatile uint32_t last_ccount = 0;

    // 0.5ms 이하의 비정상 인터럽트 무시 (스로틀링)
    if (ccount - last_ccount > 120000) {
        if (g_isr_imu_task) {
            vTaskNotifyGiveFromISR(g_isr_imu_task, &woken);
            if (woken) portYIELD_FROM_ISR();
        }
        last_ccount = ccount;
    }
}


// 전역 인터페이스 함수들
void T240_DispatchCommand(T2_Type::EM_SystemCommand_t p_cmd) {
    CL_T2_FsmManager::getInstance().dispatchCommand(p_cmd);
}

uint8_t T240_GetCurrentState() {
    return (uint8_t)CL_T2_FsmManager::getInstance().getState();
}

void T240_ReloadDspFilters() {
    CL_T2_FsmManager::getInstance().reloadDspFilters();
}

void CL_T2_FsmManager::reloadDspFilters() {
    const auto& v_cfg = CL_T2_ConfigManager::getInstance().getConfig();
    _dsp.reloadFilters(v_cfg);
}

CL_T2_FsmManager::CL_T2_FsmManager()
    : _sensor(SPI), _state(T2_Type::EM_SystemState_t::INIT) {
    // T210_Def에 정의된 전역 안전 릴레이 핀 상수를 바인딩하여 하드코딩 제거
    _interlock = new PreemptiveSafetyInterlock(T2_Def::Global::Hardware::PIN_SAFETY_RELAY_CONST);
    _safetyManager = new SafetyLifecycleManager(*_interlock);
}

CL_T2_FsmManager::~CL_T2_FsmManager() {
    if (_pktTele)       heap_caps_free(_pktTele);
    if (_pktWaveAudio)  heap_caps_free(_pktWaveAudio);
    if (_pktWaveAccel)  heap_caps_free(_pktWaveAccel);
    if (_pktWaveGyro)   heap_caps_free(_pktWaveGyro);
    if (_pktSpec)       heap_caps_free(_pktSpec);
    if (_pktSeq)        heap_caps_free(_pktSeq);

    if (_prcBeamformed) heap_caps_free(_prcBeamformed);

    if (_interlock)     delete _interlock;
    if (_safetyManager) delete _safetyManager;
}

bool CL_T2_FsmManager::init() {
    if (T2_Def::Global::Hardware::PIN_RGB_LED_CONST != T2_Def::Global::Hardware::PIN_NOT_SET_CONST) {
        neopixelWrite(T2_Def::Global::Hardware::PIN_RGB_LED_CONST, 32, 32, 0);
    }
    ESP_LOGI(TAG, "Initializing v247 Orchestrator...");

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

    // 텔레메트리 패킷 등 통신 버퍼만 PSRAM 할당
    _pktTele      = (T2_Type::ST_PktTelemetry_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_PktTelemetry_t), MALLOC_CAP_SPIRAM);
    _pktWaveAudio = (T2_Type::ST_PktWaveformAudio_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_PktWaveformAudio_t), MALLOC_CAP_SPIRAM);
    _pktWaveAccel = (T2_Type::ST_PktWaveformAccel_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_PktWaveformAccel_t), MALLOC_CAP_SPIRAM);
    _pktWaveGyro  = (T2_Type::ST_PktWaveformGyro_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_PktWaveformGyro_t), MALLOC_CAP_SPIRAM);
    _pktSpec      = (T2_Type::ST_PktSpectrum_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_PktSpectrum_t), MALLOC_CAP_SPIRAM);
    _pktSeq       = (T2_Type::ST_PktSequence_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_PktSequence_t), MALLOC_CAP_SPIRAM);

    _telemetryHz = v_cfg.system.tele_hz;
    _waveformHz  = v_cfg.system.wave_hz;
    _sequenceHz  = 2;

    // 공유 메모리 컨텍스트 할당 및 초기화
    _sharedCtx = (T2_Type::ST_SharedContext_t*)heap_caps_aligned_alloc(16, sizeof(T2_Type::ST_SharedContext_t), MALLOC_CAP_SPIRAM);
    memset(_sharedCtx, 0, sizeof(T2_Type::ST_SharedContext_t));
    _sharedCtx->vib_idx.store(0);
    _sharedCtx->aud_idx.store(0);

    _qSessionCmd = xQueueCreate(16, sizeof(uint8_t));

    // 비동기 태스크 분리 생성
    xTaskCreatePinnedToCore(_imuAcqTask,       "ImuAcqTask",  T2_Def::Global::Task::IMU_ACQ_STACK_SIZE,  this, T2_Def::Global::Task::IMU_ACQ_PRIORITY,  &_hImuAcqTask, T2_Def::Global::Task::CORE_CAPTURE_DEF);
    xTaskCreatePinnedToCore(_audioProcessTask, "AudProcTask", T2_Def::Global::Task::AUD_PROC_STACK_SIZE, this, T2_Def::Global::Task::AUD_PROC_PRIORITY, &_hAudioTask,   T2_Def::Global::Task::CORE_PROCESS_DEF);
    xTaskCreatePinnedToCore(_vibProcessTask,   "VibProcTask", T2_Def::Global::Task::VIB_PROC_STACK_SIZE, this, T2_Def::Global::Task::VIB_PROC_PRIORITY, &_hVibTask,     T2_Def::Global::Task::CORE_PROCESS_DEF);

    setState(T2_Type::EM_SystemState_t::READY);
    ESP_LOGI(TAG, "v247 Orchestrator Ready with Async Pipeline.");
    return true;
}


void CL_T2_FsmManager::setState(T2_Type::EM_SystemState_t p_newState) {
    portENTER_CRITICAL(&_stateMux);
    if (_state == p_newState) { portEXIT_CRITICAL(&_stateMux); return; }
    T2_Type::EM_SystemState_t v_prevState = _state;
    _state = p_newState;
    portEXIT_CRITICAL(&_stateMux);

    ESP_LOGI(TAG, "System State Changed: %d -> %d", (int)v_prevState, (int)p_newState);

    // RGB LED 상태 표시 제어 (PIN_RGB_LED_CONST 핀 사용)
    if (T2_Def::Global::Hardware::PIN_RGB_LED_CONST != T2_Def::Global::Hardware::PIN_NOT_SET_CONST) {
        uint8_t r = 0, g = 0, b = 0;
        switch (p_newState) {
            case T2_Type::EM_SystemState_t::INIT:            r = 32;  g = 32; b = 0;  break; // 노란색
            case T2_Type::EM_SystemState_t::READY:           r = 0;   g = 64; b = 0;  break; // 초록색
            case T2_Type::EM_SystemState_t::MONITORING:      r = 0;   g = 64; b = 64; break; // 하늘색
            case T2_Type::EM_SystemState_t::RECORDING:       r = 64;  g = 0;  b = 0;  break; // 빨간색
            case T2_Type::EM_SystemState_t::NOISE_LEARNING:  r = 64;  g = 0;  b = 64; break; // 보라색
            case T2_Type::EM_SystemState_t::MAINTENANCE:     r = 32;  g = 32; b = 32; break; // 흰색
            case T2_Type::EM_SystemState_t::ERROR:           r = 128; g = 0;  b = 0;  break; // 빨간색
            case T2_Type::EM_SystemState_t::CALIBRATING:     r = 64;  g = 32; b = 0;  break; // 주황색
            default: break;
        }
        neopixelWrite(T2_Def::Global::Hardware::PIN_RGB_LED_CONST, r, g, b);
    }

    if (p_newState == T2_Type::EM_SystemState_t::READY) {
        _lastTick = (uint32_t)(esp_timer_get_time() / 1000000ULL);
        _currentTrial = 0;
        _isManualRecording = false;

        // Cascade Clear
        _dsp.resetStates();
        _seqBuilder.reset();
        _trigger.resetCounter();
    }
    else if (p_newState == T2_Type::EM_SystemState_t::MAINTENANCE) {
        _storage.closeSession("ota_maintenance");
        // 분리형 파이프라인 가공 태스크 일시 중단 처리 전환
        if (_hAudioTask) vTaskSuspend(_hAudioTask);
        if (_hVibTask)   vTaskSuspend(_hVibTask);
        ESP_LOGW(TAG, "System entering MAINTENANCE mode. Bus isolated. Tasks suspended.");
    }
    else if (p_newState == T2_Type::EM_SystemState_t::MONITORING || p_newState == T2_Type::EM_SystemState_t::RECORDING || p_newState == T2_Type::EM_SystemState_t::READY) {
        if (v_prevState == T2_Type::EM_SystemState_t::MAINTENANCE) {
            // 분리형 파이프라인 가공 태스크 재개 처리 전환
            if (_hAudioTask) vTaskResume(_hAudioTask);
            if (_hVibTask)   vTaskResume(_hVibTask);
            ESP_LOGI(TAG, "Tasks resumed from MAINTENANCE.");
        }
        if ((p_newState == T2_Type::EM_SystemState_t::MONITORING || p_newState == T2_Type::EM_SystemState_t::RECORDING) && v_prevState == T2_Type::EM_SystemState_t::READY) {
            _dsp.resetStates();
            _seqBuilder.reset();
            _trigger.resetCounter();
        }
    }
}


void CL_T2_FsmManager::dispatchCommand(T2_Type::EM_SystemCommand_t p_cmd) {
    if (_state == T2_Type::EM_SystemState_t::MAINTENANCE &&
        p_cmd != T2_Type::EM_SystemCommand_t::CMD_REBOOT &&
        p_cmd != T2_Type::EM_SystemCommand_t::CMD_OTA_END) return;

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
        // OTA 상태 전이 명령
        case T2_Type::EM_SystemCommand_t::CMD_OTA_START:
            ESP_LOGW(TAG, "OTA Update Started. Entering MAINTENANCE mode.");
            setState(T2_Type::EM_SystemState_t::MAINTENANCE);
            break;
        case T2_Type::EM_SystemCommand_t::CMD_OTA_END:
            ESP_LOGI(TAG, "OTA Update Ended.");
            if (_state == T2_Type::EM_SystemState_t::MAINTENANCE) {
                setState(T2_Type::EM_SystemState_t::READY);
            }
            break;

        default: break;
    }
}


void CL_T2_FsmManager::runMaintenance() {
    // 1. 비동기 종료 예약 명령 확인 및 처리 (추가된 부분)
    _checkGracefulClose();

    // 2. 세션 제어 큐 처리
    uint8_t v_cmd;
    while (xQueueReceive(_qSessionCmd, &v_cmd, 0) == pdTRUE) {
        switch ((T2_Type::EM_AsyncSessionCmd_t)v_cmd) {
            case T2_Type::EM_AsyncSessionCmd_t::OPEN_AUTO:       _storage.openSession("trg_auto"); break;
            case T2_Type::EM_AsyncSessionCmd_t::OPEN_MANUAL:     _storage.openSession("man"); break;
            case T2_Type::EM_AsyncSessionCmd_t::OPEN_CALIB_MAN:  _storage.openSession("calib_man"); break;
            case T2_Type::EM_AsyncSessionCmd_t::CLOSE_NORMAL:    _storage.closeSession("auto_end"); break;
            case T2_Type::EM_AsyncSessionCmd_t::CLOSE_MANUAL:    _storage.closeSession("man_end"); break;
            case T2_Type::EM_AsyncSessionCmd_t::CLOSE_CALIB_DONE:
                _storage.closeSession("calib_done");
                _calibrator.startManualCalibration(_storage.getLastPath()); // bin 파일 경로 사용
                break;
            default: break;
        }
    }

    CL_T2_ConfigManager::getInstance().checkLazyWrite();
    _comm.runNetwork();

    if (_storage.hasIoError()) _storage.attemptRecovery();

    // 유휴 상태 자동 교정 트리거 (audio.auto_idle_min 적용)
    if (_state == T2_Type::EM_SystemState_t::READY) {
        uint32_t v_nowSec = (uint32_t)(esp_timer_get_time() / 1000000ULL);
        const auto& v_cfg = CL_T2_ConfigManager::getInstance().getConfig();
        if (v_cfg.audio.auto_idle_min > 0 && (v_nowSec - _lastTick > v_cfg.audio.auto_idle_min * 60)) {
            dispatchCommand(T2_Type::EM_SystemCommand_t::CMD_CALIBRATE);
            _lastTick = v_nowSec;
        }

        // 유휴 상태 딥슬립 진입 트리거 (trig_use_sleep 및 trig_sleep_sec 적용)
        if (v_cfg.trig_use_sleep && v_cfg.trig_sleep_sec > 0 && (v_nowSec - _lastTick > v_cfg.trig_sleep_sec)) {
            ESP_LOGW(TAG, "Entering Deep Sleep... Idle for %u sec", (unsigned int)v_cfg.trig_sleep_sec);
            _storage.closeSession("deep_sleep");
            if (T2_Def::Global::Hardware::PIN_RGB_LED_CONST != T2_Def::Global::Hardware::PIN_NOT_SET_CONST) {
                neopixelWrite(T2_Def::Global::Hardware::PIN_RGB_LED_CONST, 0, 0, 0); // LED 끄기
            }
            vTaskDelay(pdMS_TO_TICKS(100));

            // 복귀 기상 트리거 소스 이원화 연동
            // 소스 1. 제어용 제어 버튼 (물리 핀 LOW 인식)
            esp_sleep_enable_ext0_wakeup((gpio_num_t)T2_Def::Global::Hardware::PIN_BTN_CONTROL_CONST, 0);

            // 소스 2. BMI270 물리 INT2 전용 핀 (Any-Motion 발생 시 RISING 하이 레벨 클럭 복귀)
            // 설비 충격 및 가동 시작 시 기기를 자동으로 잠에서 깨웁니다.
            esp_sleep_enable_ext1_wakeup(1ULL << T2_Def::Imu::Hardware::PIN_INT2_MOTION_CONST, ESP_EXT1_WAKEUP_ANY_HIGH);

            esp_deep_sleep_start();
        }
    }
}

// ============================================================================
// [Task 1] IMU 데이터 수집 태스크 (Core 0: Producer)
// ============================================================================
void CL_T2_FsmManager::_imuAcqTask(void* p_param) {
    CL_T2_FsmManager* v_this = (CL_T2_FsmManager*)p_param;
    g_isr_imu_task = xTaskGetCurrentTaskHandle();

    // 초기 인터럽트 연결 (BMI270 Watermark RISING 엣지)
    attachInterrupt(digitalPinToInterrupt(T2_Def::Imu::Hardware::PIN_INT1_WATERMARK_CONST), T245_bmi_watermark_isr, RISING);

    while (v_this->_state != T2_Type::EM_SystemState_t::INIT) {
        // 하드웨어 인터럽트 대기
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // 시스템이 MAINTENANCE(OTA 등) 상태가 아닐 때만 FIFO 수집
        if (v_this->_state != T2_Type::EM_SystemState_t::MAINTENANCE) {
            // INT1 인터럽트 전송 원인이 실제 FIFO Watermark 비트가 맞는지 레지스터 교차 검증 (오동작 방지)
            uint8_t v_int_stat_1 = v_this->_sensor._readRegSingle(T2_Def::Imu::Hardware::REG_INT_STATUS_1_ADDR);
            if (v_int_stat_1 & T2_Def::Imu::Hardware::REG_FIFO_WTM_STATUS_BIT) {
                v_this->_sensor.accumulateFifo();
            }
        }
        esp_task_wdt_reset();
    }

    // 종료 시 리소스 정리
    detachInterrupt(digitalPinToInterrupt(T2_Def::Imu::Hardware::PIN_INT1_WATERMARK_CONST));
    g_isr_imu_task = nullptr;
    v_this->_hImuAcqTask = nullptr;
    vTaskDelete(NULL);
}

// ============================================================================
// [Task 2] 진동 도메인 독립 가공 태스크 (Core 1: Consumer & Vib_Producer)
// ============================================================================
void CL_T2_FsmManager::_vibProcessTask(void* p_param) {
    CL_T2_FsmManager* v_this = (CL_T2_FsmManager*)p_param;
    const auto& 	  v_cfg  = CL_T2_ConfigManager::getInstance().getConfig();

    // 진동 도메인 전용 로컬 스크래치 버퍼 (Internal SRAM 16B 정렬 할당 사양 준수)
    float* v_acc_proc_x = (float*)heap_caps_malloc(T2_Def::Accel::Sensor::FFT_SIZE_MAX * sizeof(float), MALLOC_CAP_INTERNAL);
    float* v_acc_proc_y = (float*)heap_caps_malloc(T2_Def::Accel::Sensor::FFT_SIZE_MAX * sizeof(float), MALLOC_CAP_INTERNAL);
    float* v_acc_proc_z = (float*)heap_caps_malloc(T2_Def::Accel::Sensor::FFT_SIZE_MAX * sizeof(float), MALLOC_CAP_INTERNAL);
    float* v_gyr_proc_x = (float*)heap_caps_malloc(T2_Def::Gyro::Sensor::FFT_SIZE_MAX * sizeof(float), MALLOC_CAP_INTERNAL);
    float* v_gyr_proc_y = (float*)heap_caps_malloc(T2_Def::Gyro::Sensor::FFT_SIZE_MAX * sizeof(float), MALLOC_CAP_INTERNAL);
    float* v_gyr_proc_z = (float*)heap_caps_malloc(T2_Def::Gyro::Sensor::FFT_SIZE_MAX * sizeof(float), MALLOC_CAP_INTERNAL);

    while (v_this->_state != T2_Type::EM_SystemState_t::INIT) {
        if (v_this->_state == T2_Type::EM_SystemState_t::MONITORING ||
            v_this->_state == T2_Type::EM_SystemState_t::RECORDING ||
            v_this->_state == T2_Type::EM_SystemState_t::CALIBRATING) {

            // 1. 수집된 IMU 프레임이 FFT 크기(1024)에 도달했는지 확인 (약 640ms 주기 달성)
            if (v_this->_sensor.getAccumulatedAccelCount() >= T2_Def::Accel::Sensor::FFT_SIZE_MAX) {

                // 더블 버퍼링: 쓰기 인덱스 결정 (현재 인덱스 ^ 1)
                uint8_t v_write_idx = v_this->_sharedCtx->vib_idx.load(std::memory_order_relaxed) ^ 1;
                auto& v_slot = v_this->_sharedCtx->vib_slots[v_write_idx];
                memset(&v_slot, 0, sizeof(T2_Type::ST_FeatureSlot_Vib_t)); // 패딩 영역 0 초기화

                // 헤더 조립
                v_slot.header.ts = (uint64_t)(esp_timer_get_time() / 1000);
                v_slot.header.uptime = millis();
                v_slot.header.temp = v_this->_sensor.readTemperatureSensor();
                v_slot.header.payload_type = (uint8_t)T2_Type::EM_DataPayloadType_t::VIB_ONLY;
                v_slot.header.accel_mask = v_cfg.accel.axis_mask;
                v_slot.header.gyro_mask  = v_cfg.gyro.axis_mask;

                // 2. 가속도/자이로 원시 데이터 인출 (Zero-Copy 링버퍼 활용)
                v_this->_sensor.getAccumulatedAccel(v_acc_proc_x, v_acc_proc_y, v_acc_proc_z, T2_Def::Accel::Sensor::FFT_SIZE_MAX);
                v_this->_sensor.getAccumulatedGyro(v_gyr_proc_x, v_gyr_proc_y, v_gyr_proc_z, T2_Def::Gyro::Sensor::FFT_SIZE_MAX);

                // 3. DSP 가공 및 특징 추출
                if (v_cfg.accel.enable) {
                    v_this->_dsp.processAccel(v_acc_proc_x, v_acc_proc_y, v_acc_proc_z, v_acc_proc_x, v_acc_proc_y, v_acc_proc_z, T2_Def::Accel::Sensor::FFT_SIZE_MAX, v_cfg.accel);
                    v_this->_extractor.extractAccel(v_acc_proc_x, v_acc_proc_y, v_acc_proc_z, T2_Def::Accel::Sensor::FFT_SIZE_MAX, v_cfg.accel.sample_rate, v_slot, v_cfg.accel);
                }

                if (v_cfg.gyro.enable) {
                    v_this->_dsp.processGyro(v_gyr_proc_x, v_gyr_proc_y, v_gyr_proc_z, v_gyr_proc_x, v_gyr_proc_y, v_gyr_proc_z, T2_Def::Gyro::Sensor::FFT_SIZE_MAX, v_cfg.gyro);
                    v_this->_extractor.extractGyro(v_gyr_proc_x, v_gyr_proc_y, v_gyr_proc_z, T2_Def::Gyro::Sensor::FFT_SIZE_MAX, v_cfg.gyro.sample_rate, v_slot, v_cfg.gyro);
                }

                // 글로벌 스트리밍 버퍼에 복사 스냅샷 생성 (웹소켓 전송용 동기화 캐시 스페이스 정합)
                if (v_this->_dsp.getAccBufX()) memcpy(v_this->_dsp.getAccBufX(), v_acc_proc_x, T2_Def::Accel::Sensor::FFT_SIZE_MAX * sizeof(float));
                if (v_this->_dsp.getAccBufY()) memcpy(v_this->_dsp.getAccBufY(), v_acc_proc_y, T2_Def::Accel::Sensor::FFT_SIZE_MAX * sizeof(float));
                if (v_this->_dsp.getAccBufZ()) memcpy(v_this->_dsp.getAccBufZ(), v_acc_proc_z, T2_Def::Accel::Sensor::FFT_SIZE_MAX * sizeof(float));
                if (v_this->_dsp.getGyrBufX()) memcpy(v_this->_dsp.getGyrBufX(), v_gyr_proc_x, T2_Def::Gyro::Sensor::FFT_SIZE_MAX * sizeof(float));
                if (v_this->_dsp.getGyrBufY()) memcpy(v_this->_dsp.getGyrBufY(), v_gyr_proc_y, T2_Def::Gyro::Sensor::FFT_SIZE_MAX * sizeof(float));
                if (v_this->_dsp.getGyrBufZ()) memcpy(v_this->_dsp.getGyrBufZ(), v_gyr_proc_z, T2_Def::Gyro::Sensor::FFT_SIZE_MAX * sizeof(float));

                // 4. 원자적 스왑: 오디오 태스크가 이제 무등급으로 읽어갈 수 있게 배포 장벽 해제
                v_this->_sharedCtx->vib_idx.store(v_write_idx, std::memory_order_release);

                // [Phase 4 완료] 진동 특징량 구조체 및 복사된 원시 정적 버퍼를 스토리지 비동기 엔진에 인입
                T2_Type::ST_Raw_Accel_t v_tmpAcc;
                v_tmpAcc.ts = v_slot.header.ts;
                v_tmpAcc.sample_rate = v_cfg.accel.sample_rate;
                v_tmpAcc.active_mask = v_slot.header.accel_mask;
                memcpy(v_tmpAcc.data[0], v_acc_proc_x, T2_Def::Accel::Sensor::FFT_SIZE_MAX * sizeof(float));
                memcpy(v_tmpAcc.data[1], v_acc_proc_y, T2_Def::Accel::Sensor::FFT_SIZE_MAX * sizeof(float));
                memcpy(v_tmpAcc.data[2], v_acc_proc_z, T2_Def::Accel::Sensor::FFT_SIZE_MAX * sizeof(float));

                T2_Type::ST_Raw_Gyro_t v_tmpGyr;
                v_tmpGyr.ts = v_slot.header.ts;
                v_tmpGyr.sample_rate = v_cfg.gyro.sample_rate;
                v_tmpGyr.active_mask = v_slot.header.gyro_mask;
                memcpy(v_tmpGyr.data[0], v_gyr_proc_x, T2_Def::Gyro::Sensor::FFT_SIZE_MAX * sizeof(float));
                memcpy(v_tmpGyr.data[1], v_gyr_proc_y, T2_Def::Gyro::Sensor::FFT_SIZE_MAX * sizeof(float));
                memcpy(v_tmpGyr.data[2], v_gyr_proc_z, T2_Def::Gyro::Sensor::FFT_SIZE_MAX * sizeof(float));

                v_this->_storage.pushVibFrame(&v_slot, &v_tmpAcc, &v_tmpGyr);
            }
        }

        // 과도한 CPU 점유 방지 및 Task Scheduler 양보
        vTaskDelay(pdMS_TO_TICKS(10));
        esp_task_wdt_reset();
    }

    // 종료 시 리소스 반환
    if (v_acc_proc_x) heap_caps_free(v_acc_proc_x);
    if (v_acc_proc_y) heap_caps_free(v_acc_proc_y);
    if (v_acc_proc_z) heap_caps_free(v_acc_proc_z);
    if (v_gyr_proc_x) heap_caps_free(v_gyr_proc_x);
    if (v_gyr_proc_y) heap_caps_free(v_gyr_proc_y);
    if (v_gyr_proc_z) heap_caps_free(v_gyr_proc_z);

    v_this->_hVibTask = nullptr;
    vTaskDelete(NULL);
}



// ============================================================================
// [Task 3] 오디오 도메인 독립 가공 태스크 (Core 1: Consumer & Tensor Builder)
// ============================================================================
void CL_T2_FsmManager::_audioProcessTask(void* p_param) {
    CL_T2_FsmManager* v_this = (CL_T2_FsmManager*)p_param;
    const auto& v_cfg = CL_T2_ConfigManager::getInstance().getConfig();

	float* v_rawAudioL = v_this->_dsp.getCapBufL();
    float* v_rawAudioR = v_this->_dsp.getCapBufR();

    float* v_outAudioL = v_this->_dsp.getPrcBufL();
    float* v_outAudioR = v_this->_dsp.getPrcBufR();


    while (v_this->_state != T2_Type::EM_SystemState_t::INIT) {
        if (v_this->_state == T2_Type::EM_SystemState_t::MONITORING ||
            v_this->_state == T2_Type::EM_SystemState_t::RECORDING ||
            v_this->_state == T2_Type::EM_SystemState_t::NOISE_LEARNING ||
            v_this->_state == T2_Type::EM_SystemState_t::CALIBRATING) {

            // 1. 오디오 청크 수집 (Wait-free I2S DMA 인출)
            uint32_t v_samples = v_this->_sensor.readAudioChunk(v_rawAudioL, v_rawAudioR, T2_Def::Audio::Sensor::FFT_SIZE_MAX);

            if (v_samples > 0) {
                // 더블 버퍼링: 쓰기 인덱스 결정
                uint8_t v_write_idx = v_this->_sharedCtx->aud_idx.load(std::memory_order_relaxed) ^ 1;
                auto&   v_aud_slot = v_this->_sharedCtx->aud_slots[v_write_idx];
                memset(&v_aud_slot, 0, sizeof(T2_Type::ST_FeatureSlot_Aud_t));

                // 헤더 조립
                v_aud_slot.header.ts = (uint64_t)(esp_timer_get_time() / 1000);
                v_aud_slot.header.uptime = millis();
                v_aud_slot.header.payload_type = (uint8_t)T2_Type::EM_DataPayloadType_t::AUDIO_ONLY;
                v_aud_slot.header.audio_mask = v_cfg.audio.channel_mask;

                // 2. 오디오 DSP 가공 및 특징 추출
                v_this->_dsp.processAudio(v_rawAudioL, v_rawAudioR, v_outAudioL, v_outAudioR, v_samples, v_cfg.audio);
                v_this->_extractor.extractAudio(v_outAudioL, v_outAudioR, v_samples, v_cfg.audio.sample_rate, v_aud_slot, v_cfg.audio);

                // 3. 원자적 스왑 (오디오 슬롯 갱신)
                v_this->_sharedCtx->aud_idx.store(v_write_idx, std::memory_order_release);

                // 4. Late-Sync: 가장 최신 진동 슬롯을 Lock-free 인출
                uint8_t v_vib_idx = v_this->_sharedCtx->vib_idx.load(std::memory_order_acquire);
                const auto& v_vib_slot = v_this->_sharedCtx->vib_slots[v_vib_idx];

                // 텐서 조립 (진동 데이터 초기화가 완료된 시점부터)
                if (v_vib_slot.header.ts > 0) {
                    memcpy(v_this->_flatTensor, v_aud_slot.mfcc, 78 * sizeof(float));
                    memcpy(v_this->_flatTensor + 78, v_vib_slot.mfcc, 234 * sizeof(float));
                    v_this->_seqBuilder.pushVector(v_this->_flatTensor);
                }

                // [Phase 4 완료] 초고속 비동기 융합 판정 (Trigger) 연동 사양 정합
                if (v_this->_state == T2_Type::EM_SystemState_t::MONITORING || v_this->_state == T2_Type::EM_SystemState_t::RECORDING) {
                    T2_Type::EM_DetectionResult_t v_res = v_this->_trigger.runDiagnostic(v_aud_slot, v_vib_slot, v_cfg);
                    v_this->_handleTriggerResult(v_res, v_aud_slot, v_vib_slot);
                } else if (v_this->_state == T2_Type::EM_SystemState_t::CALIBRATING) {
                    // 캘리브레이션 세션 활성화 상태 시 무조건 밀어넣기 처리
                    T2_Type::ST_Raw_Audio_t v_tmpAud;
                    v_tmpAud.ts = v_aud_slot.header.ts;
                    v_tmpAud.sample_rate = v_cfg.audio.sample_rate;
                    v_tmpAud.active_mask = v_aud_slot.header.audio_mask;
                    memcpy(v_tmpAud.data[0], v_rawAudioL, T2_Def::Audio::Sensor::FFT_SIZE_MAX * sizeof(float));
                    memcpy(v_tmpAud.data[1], v_rawAudioR, T2_Def::Audio::Sensor::FFT_SIZE_MAX * sizeof(float));
                    v_this->_storage.pushAudioFrame(&v_aud_slot, &v_tmpAud);
                }

                // [Phase 4 완료] 실시간 스트리밍 다중 파형 매핑 호출 전달 정합 완료
                v_this->_broadcastStreams(v_aud_slot, v_vib_slot, v_rawAudioL, v_rawAudioR);
            }
        }

        // 24ms 루프 주기 준수 (I2S DMA 속도와 동기화)
        vTaskDelay(pdMS_TO_TICKS(24));
        esp_task_wdt_reset();
    }

    v_this->_hAudioTask = nullptr;
    vTaskDelete(NULL);
}

void CL_T2_FsmManager::_broadcastStreams(const T2_Type::ST_FeatureSlot_Aud_t& p_audSlot,
                                         const T2_Type::ST_FeatureSlot_Vib_t& p_vibSlot,
                                         const float* p_rawAudL, const float* p_rawAudR) {
    const auto& v_cfg = CL_T2_ConfigManager::getInstance().getConfig();

    // 1. Telemetry 패킷 매핑
    _accTele += _telemetryHz;
    if (_accTele >= 100) {
        _accTele -= 100;
        _pktTele->header.magic  = 0xAA;
        _pktTele->header.type   = (uint8_t)T2_Type::EM_StreamType_t::TELEMETRY;
        _pktTele->header.len    = sizeof(T2_Type::ST_PktTelemetry_t) - sizeof(T2_Type::ST_WsHeader_t);
        _pktTele->header.source = 0;
        _pktTele->header.stage  = (uint8_t)_state;

        _pktTele->sys_state      = (uint8_t)_state;
        _pktTele->detect_result  = (uint8_t)p_audSlot.header.src; // 융합 트리거 결과 연동
        _pktTele->trial_no       = p_audSlot.header.trial;
        _pktTele->trigger_source = p_audSlot.header.src;

        _pktTele->accel_mask     = p_vibSlot.header.accel_mask;
        _pktTele->gyro_mask      = p_vibSlot.header.gyro_mask;
        _pktTele->audio_mask     = p_audSlot.header.audio_mask;
        _pktTele->payload_type   = (uint8_t)T2_Type::EM_DataPayloadType_t::VIB_AUDIO_BOTH;

        _pktTele->accel_ts  = p_vibSlot.header.ts;
        _pktTele->gyro_ts   = p_vibSlot.header.ts;
        _pktTele->audio_ts  = p_audSlot.header.ts;
        _pktTele->temp      = p_audSlot.header.temp;

        // 가속도 16밴드 에너지 복사 (0번 축/대표 축 기준)
        memcpy(_pktTele->accel_band_energy, p_vibSlot.accel.band_energy[0], sizeof(_pktTele->accel_band_energy));

        // 자이로 하위 2개 저주파 밴드 복사 (0번 축/대표 축 기준)
        _pktTele->gyro_rms_energy[0] = p_vibSlot.gyro.band_energy[0][0];
        _pktTele->gyro_rms_energy[1] = p_vibSlot.gyro.band_energy[0][1];

        // 오디오 1/3 옥타브 대역 복사
        memcpy(_pktTele->audio_timbre_bands, p_audSlot.audio.timbre_bands, sizeof(_pktTele->audio_timbre_bands));

        // 오디오 MFCC 계수 복사 (13차원)
        memcpy(_pktTele->audio_mfcc, p_audSlot.mfcc, sizeof(_pktTele->audio_mfcc));

        _comm.broadcastBinary(_pktTele, sizeof(T2_Type::ST_PktTelemetry_t));
    }

    // 2. Waveform 파형 전송 전처리
    _accWave += _waveformHz;
    if (_accWave >= 100) {
        _accWave -= 100;

        if (v_cfg.audio.enable && p_rawAudL) {
            _pktWaveAudio->header.magic  = 0xAA;
            _pktWaveAudio->header.type   = (uint8_t)T2_Type::EM_StreamType_t::WAVEFORM;
            _pktWaveAudio->header.source = 1; // Left Channel
            _pktWaveAudio->header.len    = T2_Def::Audio::Sensor::FFT_SIZE_MAX * sizeof(float);
            memcpy(_pktWaveAudio->samples, p_rawAudL, _pktWaveAudio->header.len);
            _comm.broadcastBinary(_pktWaveAudio, sizeof(T2_Type::ST_PktWaveformAudio_t));

            if ((v_cfg.audio.channel_mask & (uint8_t)T2_Type::EM_ChannelMask_t::CH_RIGHT) && p_rawAudR) {
                _pktWaveAudio->header.source = 5; // Right Channel
                memcpy(_pktWaveAudio->samples, p_rawAudR, _pktWaveAudio->header.len);
                _comm.broadcastBinary(_pktWaveAudio, sizeof(T2_Type::ST_PktWaveformAudio_t));
            }
        }

        // 관성 센서(가속도/자이로) 전송용 고속 버퍼 파형 복사 스트리밍 바인딩
        if (v_cfg.accel.enable && _dsp.getAccBufX()) {
            for (int i = 0; i < 3; i++) {
                if (v_cfg.accel.axis_mask & (1 << i)) {
                    _pktWaveAccel->header.magic  = 0xAA;
                    _pktWaveAccel->header.type   = (uint8_t)T2_Type::EM_StreamType_t::WAVEFORM;
                    _pktWaveAccel->header.source = 2 + i;
                    _pktWaveAccel->header.len    = T2_Def::Accel::Sensor::FFT_SIZE_MAX * sizeof(float);
                    float* v_srcBuf = (i == 0) ? _dsp.getAccBufX() : (i == 1) ? _dsp.getAccBufY() : _dsp.getAccBufZ();
                    memcpy(_pktWaveAccel->samples, v_srcBuf, _pktWaveAccel->header.len);
                    _comm.broadcastBinary(_pktWaveAccel, sizeof(T2_Type::ST_PktWaveformAccel_t));
                }
            }
        }
    }

    // 3. Spectrum 전송
    _accSpec += _spectrumHz;
    if (_accSpec >= 100) {
        _accSpec -= 100;
        _pktSpec->header.magic  = 0xAA;
        _pktSpec->header.type   = (uint8_t)T2_Type::EM_StreamType_t::SPECTRUM;
        _pktSpec->header.source = 1;
        _pktSpec->header.len    = ((T2_Def::Audio::Sensor::FFT_SIZE_MAX / 2) + 1) * sizeof(float);
        memcpy(_pktSpec->frequencies, _extractor.getAudioPowerSpectrum(), _pktSpec->header.len);
        _comm.broadcastBinary(_pktSpec, sizeof(T2_Type::ST_PktSpectrum_t));
    }

    // 4. AI Sequence 텐서 전송
    _accSeq += _sequenceHz;
    if (_accSeq >= 100) {
        _accSeq -= 100;
        if (_seqBuilder.isReady()) {
            _pktSeq->header.magic  = 0xAA;
            _pktSeq->header.type   = (uint8_t)T2_Type::EM_StreamType_t::SEQUENCE;
            _pktSeq->header.source = 0;
            size_t v_seqBytes = T2_Def::Global::System::SEQUENCE_FRAMES_MAX * T2_Def::AI::Tensor::MFCC_DIM_MAX * sizeof(float);
            _pktSeq->header.len = v_seqBytes;
            _seqBuilder.getSequenceFlat(_pktSeq->data, v_seqBytes);
            _comm.broadcastBinary(_pktSeq, sizeof(T2_Type::ST_WsHeader_t) + v_seqBytes);
        }
    }
}

void CL_T2_FsmManager::_checkGracefulClose() {
    // [보완] 더 이상 FSM 단의 가공 대기 큐(_qReadyIdx)를 확인할 필요가 없습니다.
    // 비동기 파이프라인에서는 StorageManager가 자신의 큐를 100% 비운 뒤 알아서 파일을 닫습니다.
    if (_stopReasonCmd != 0) {
        uint8_t v_cmd = _stopReasonCmd;

        // 세션 제어 큐로 종료 명령을 즉각 푸시
        if (xQueueSend(_qSessionCmd, &v_cmd, 0) == pdTRUE) {
            _stopReasonCmd = 0;

            // FSM 상태는 즉시 READY로 전환하여 센서 수집 및 가공 태스크를 논리적으로 중단시킴
            if (v_cmd == (uint8_t)T2_Type::EM_AsyncSessionCmd_t::CLOSE_NORMAL ||
                v_cmd == (uint8_t)T2_Type::EM_AsyncSessionCmd_t::CLOSE_MANUAL) {
                setState(T2_Type::EM_SystemState_t::READY);
            }
        }
    }
}


void CL_T2_FsmManager::_handleTriggerResult(T2_Type::EM_DetectionResult_t p_res,
                                            const T2_Type::ST_FeatureSlot_Aud_t& p_audSlot,
                                            const T2_Type::ST_FeatureSlot_Vib_t& p_vibSlot) {
    if (p_res != T2_Type::EM_DetectionResult_t::PASS) {
        if (_state != T2_Type::EM_SystemState_t::RECORDING) {
            setState(T2_Type::EM_SystemState_t::RECORDING);
            _storage.openSession("auto_trigger");
        }

        // --------------------------------------------------------
        // 1. 오디오 원시 데이터 정적 구조체 메모리 복사 및 스토리지 비동기 인입
        // --------------------------------------------------------
        T2_Type::ST_Raw_Audio_t v_tmpAud;
        memset(&v_tmpAud, 0, sizeof(T2_Type::ST_Raw_Audio_t));

        v_tmpAud.ts = p_audSlot.header.ts;
        v_tmpAud.sample_rate = CL_T2_ConfigManager::getInstance().getConfig().audio.sample_rate;
        v_tmpAud.active_mask = p_audSlot.header.audio_mask;

        // 안전 가드: 소스 버퍼 유효성 검사 후 정적 배열에 값 복사
        size_t v_audBlockSize = T2_Def::Audio::Sensor::FFT_SIZE_MAX * sizeof(float);
        if (_dsp.getCapBufL()) memcpy(v_tmpAud.data[0], _dsp.getCapBufL(), v_audBlockSize);
        if (_dsp.getCapBufR()) memcpy(v_tmpAud.data[1], _dsp.getCapBufR(), v_audBlockSize);

        _storage.pushAudioFrame(&p_audSlot, &v_tmpAud);


        // --------------------------------------------------------
        // 2. 진동 원시 데이터(가속도/자이로) 정적 구조체 메모리 복사 및 인입
        // --------------------------------------------------------
        T2_Type::ST_Raw_Accel_t v_tmpAcc;
        memset(&v_tmpAcc, 0, sizeof(T2_Type::ST_Raw_Accel_t));

        v_tmpAcc.ts = p_vibSlot.header.ts;
        v_tmpAcc.sample_rate = CL_T2_ConfigManager::getInstance().getConfig().accel.sample_rate;
        v_tmpAcc.active_mask = p_vibSlot.header.accel_mask;

        size_t v_vibBlockSize = T2_Def::Accel::Sensor::FFT_SIZE_MAX * sizeof(float);
        if (_dsp.getAccBufX()) memcpy(v_tmpAcc.data[0], _dsp.getAccBufX(), v_vibBlockSize);
        if (_dsp.getAccBufY()) memcpy(v_tmpAcc.data[1], _dsp.getAccBufY(), v_vibBlockSize);
        if (_dsp.getAccBufZ()) memcpy(v_tmpAcc.data[2], _dsp.getAccBufZ(), v_vibBlockSize);

        T2_Type::ST_Raw_Gyro_t v_tmpGyr;
        memset(&v_tmpGyr, 0, sizeof(T2_Type::ST_Raw_Gyro_t));

        v_tmpGyr.ts = p_vibSlot.header.ts;
        v_tmpGyr.sample_rate = CL_T2_ConfigManager::getInstance().getConfig().gyro.sample_rate;
        v_tmpGyr.active_mask = p_vibSlot.header.gyro_mask;

        if (_dsp.getGyrBufX()) memcpy(v_tmpGyr.data[0], _dsp.getGyrBufX(), v_vibBlockSize);
        if (_dsp.getGyrBufY()) memcpy(v_tmpGyr.data[1], _dsp.getGyrBufY(), v_vibBlockSize);
        if (_dsp.getGyrBufZ()) memcpy(v_tmpGyr.data[2], _dsp.getGyrBufZ(), v_vibBlockSize);

        _storage.pushVibFrame(&p_vibSlot, &v_tmpAcc, &v_tmpGyr);
    }

    // 이원화된 특징량 슬롯을 개정된 MQTT 포맷 채널로 발행 처리
    _comm.publishResultMqtt(p_audSlot, p_vibSlot, p_res);
}

void CL_T2_FsmManager::processManualReset() {
    if (_safetyManager) {
        _safetyManager->processManualReset();
    }
    if (_interlock) {
        _interlock->clearEmergencyLatch();
    }
    setState(T2_Type::EM_SystemState_t::READY);
}

