/* ============================================================================
 * File: T200_Main_245.h
 * Summary: v245 4-Tier 시스템 통합 진입점 및 전역 관리 (main.cpp 연동용)
 * ============================================================================
 * [로직 상태 추적 (Logic State Tracker)]
 * - 유지: esp_timer 기반 ISR 디바운싱 및 핵심 부팅 시퀀스.
 * - 갱신: v245 4-Tier 설정 체계에 따른 하드웨어 핀 맵핑 적용.
 * - 신규: [원칙 준수] 링커 오류 방지를 위한 inline 선언 및 SSOT 초기화 보장.
 * ========================================================================== */
#pragma once

#include <Arduino.h>
#include "esp_timer.h"
#include "T210_Def_245.hpp"
#include "T220_CfgMgr_245.hpp"
#include "T290_FsmMgr_245.hpp"

// FSM 매니저 싱글톤 참조
static CL_T2_FsmManager& v_fsm_ref = CL_T2_FsmManager::getInstance();

/**
 * @brief 외부 물리 버튼/신호 인터럽트 서비스 루틴
 */
void IRAM_ATTR T245_handleTriggerISR() {
    static uint64_t v_last_us = 0;
    uint64_t v_now_us = esp_timer_get_time();

    // 200ms 소프트웨어 디바운싱
    if (v_now_us - v_last_us > 200000ULL) {
        v_last_us = v_now_us;
        bool v_active = digitalRead(T2_Def::Global::Hardware::PIN_BTN_CONTROL_CONST);
        // FSM에 상태 전달 (필요 시 CMD_START/STOP으로 변환 가능)
        if (v_active) v_fsm_ref.dispatchCommand(T2_Type::EM_SystemCommand_t::CMD_START);
        else v_fsm_ref.dispatchCommand(T2_Type::EM_SystemCommand_t::CMD_STOP);
    }
}

/**
 * @brief 시스템 초기화 (main.cpp의 setup()에서 호출)
 */
inline void T2_init() {
    // 1. 디버그 시리얼 기동
    Serial.begin(115200);
    vTaskDelay(pdMS_TO_TICKS(100));
    Serial.println("\n[SMEA-T240] v245 4-Tier Diagnostic System Booting...");

    // 2. 하드웨어 핀 설정 및 초기 인터럽트 동기화
    pinMode(T2_Def::Global::Hardware::PIN_BTN_CONTROL_CONST, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(T2_Def::Global::Hardware::PIN_BTN_CONTROL_CONST), T245_handleTriggerISR, CHANGE);

    // 3. 시스템 오케스트레이터 기동 (내부적으로 CfgMgr, Sensor, Dsp, Storage 등 순차 초기화)
    if (!v_fsm_ref.init()) {
        Serial.println("[CRITICAL] System Orchestrator Initialization Failed!");
    }

    Serial.println("[SMEA-T240] System Bootstrap Completed. Ready for Monitoring.");
}

/**
 * @brief 메인 루프 실행 (main.cpp의 loop()에서 호출)
 */
inline void T2_run() {
    // 네트워크 서비스, 지연 쓰기, 자동 회전 등 관리 업무 수행
    v_fsm_ref.runMaintenance();

    // 루프 부하 분산 및 워치독 방어
    vTaskDelay(pdMS_TO_TICKS(10));
}
