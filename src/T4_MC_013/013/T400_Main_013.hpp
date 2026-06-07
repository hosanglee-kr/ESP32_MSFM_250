/* ============================================================================
 * File: T400_Main_013.hpp (main.cpp에서 include 및 호출)
 * Summary: SMEA-100 System Entry Point & Global Interrupt Handler
 * ============================================================================
 * * [AI 메모: 보완 및 방어 사항 적용]
 * 1. [링커 방어] 헤더 파일 내 구현체로 인한 다중 정의(Multiple Definition) 에러 차단.
 * 2. [블라인드 스팟 차단] 부팅 시 이미 트리거가 HIGH인 상태 감지 동기화.
 * 3. [초기화 순서 보장] 동적 설정(T415)을 가장 먼저 마운트.
 * 4. [v012 고도화 적용 - 채터링 방어]: 불안정한 전기적 노이즈로 인한
 * FSM 세션 충돌을 막기 위해 esp_timer_get_time() 기반 200ms 디바운스 로직 주입.
 * ========================================================================== */
#pragma once

#include <Arduino.h>
#include "esp_timer.h" // 디바운스용 타이머
#include "T410_Def_013.hpp"
#include "T415_ConfigMgr_013.hpp" 
#include "T450_FsmMgr_015.hpp"

static T450_FsmManager& v_fsm = T450_FsmManager::getInstance();

// 외부 트리거(DC High/Low) 인터럽트 서비스 루틴
void IRAM_ATTR handleTriggerISR() {
    static uint64_t v_lastTriggerUs = 0;
    uint64_t v_currentUs = esp_timer_get_time();

    // 200ms 소프트웨어 디바운싱 (Chattering 회피)
    if (v_currentUs - v_lastTriggerUs > 200000ULL) {
        v_lastTriggerUs = v_currentUs;
        bool v_isActive = digitalRead(SmeaConfig::Hardware::PIN_TRIGGER_CONST);
        v_fsm.handleExternalTrigger(v_isActive);
    }
}

inline void T4_init() {
    Serial.begin(SmeaConfig::Hardware::SERIAL_BAUD_CONST);
    delay(SmeaConfig::Task::BOOT_DELAY_MS_CONST);
    Serial.println("[SMEA-100] Starting System ...");

    // 0. JSON 동적 설정 매니저 마운트 (최우선)
    T415_ConfigManager::getInstance().init();

    // 하드웨어 핀 모드 설정 및 ISR 할당
    pinMode(SmeaConfig::Hardware::PIN_TRIGGER_CONST, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(SmeaConfig::Hardware::PIN_TRIGGER_CONST), handleTriggerISR, CHANGE);

    bool v_initialState = digitalRead(SmeaConfig::Hardware::PIN_TRIGGER_CONST);
    v_fsm.handleExternalTrigger(v_initialState);

    v_fsm.begin();

    Serial.println("[SMEA-100] Setup Completed. Waiting for Trigger...");
}

// 메인 루프 (Core 양보 기반 백그라운드 관리)
inline void T4_run() {
    static uint32_t v_lastAliveCheck = 0;
    if (millis() - v_lastAliveCheck > SmeaConfig::Task::ALIVE_CHECK_MS_CONST) {
        v_lastAliveCheck = millis();
    }

    // 강력한 방어 로직(네트워크 재연결, OOM 킥, 설정 플래시 지연 쓰기 등) 실행
    v_fsm.runMaintenanceTask();

    vTaskDelay(pdMS_TO_TICKS(SmeaConfig::Task::MAIN_LOOP_DELAY_MS_CONST));
}

