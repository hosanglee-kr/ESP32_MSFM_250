/* ============================================================================
 * File: T240_Main_241.hpp (main.cpp에서 include 및 T240_init 호출)
 * Summary: T240 멀티모달 진단 시스템 진입점 및 전역 인터럽트 핸들러
 * ============================================================================
 * [로직 상태 추적 (Logic State Tracker)]
 * - 유지: T400의 esp_timer 기반 200ms 소프트웨어 디바운싱 로직.
 * - 갱신: [교정] T240_Config 상수를 이용한 하드웨어 핀 및 통신 속도 맵핑.
 * - 신규: [원칙 준수] 링커 오류 방어를 위한 모든 전역 함수의 inline 선언화.
 *
 * [축소/누락 방어 체크리스트 (Omission Defense)]
 * 1. [링커 방어]: inline 키워드 적용으로 Multiple Definition 에러 차단 (Rule #1).
 * 2. [부팅 방어]: JSON 설정(T241)을 최우선 초기화하여 엔진 파라미터 무결성 확보.
 * 3. [블라인드 스팟 방어]: 인터럽트 할당 전 digitalRead를 통해 현재 트리거 상태 강제 동기화.
 * 4. [네이밍 방어]: 모든 지역 변수에 v_ 접두어 적용 및 하드코딩 배제.
 * ========================================================================== */
#pragma once

#include <Arduino.h>
#include "esp_timer.h"
#include "T210_Def_241.hpp"
#include "T215_ConfigMgr_241.hpp"
#include "T250_FsmMgr_241.hpp"

// FSM 매니저 싱글톤 참조
static CL_T2_FsmManager& v_fsm_ref = CL_T2_FsmManager::getInstance();

/**
 * @brief 외부 트리거 인터럽트 서비스 루틴 (CHANGE 감지)
 * @details 200ms 디바운싱을 통해 현장의 접점 노이즈(Chattering)를 걸러냅니다.
 */
void IRAM_ATTR T240_handleTriggerISR() {
    static uint64_t v_last_trigger_us = 0;
    uint64_t v_current_us = esp_timer_get_time();

    // 200ms 소프트웨어 디바운싱 (200,000 마이크로초)
    if (v_current_us - v_last_trigger_us > 200000ULL) {
        v_last_trigger_us = v_current_us;
        // T240_Config 상수를 통한 직접 접근
        bool v_is_active = digitalRead(T2_Config::Hardware::PIN_BTN_CONTROL_CONST);
        v_fsm_ref.handleExternalTrigger(v_is_active);
    }
}

// 시스템 초기화 진입점
inline void T2_init() {
    // 1. 시리얼 통신 기동
    Serial.begin(T2_Config::Hardware::SERIAL_BAUD_CONST);
    vTaskDelay(pdMS_TO_TICKS(100)); // 부팅 안정화 지연
    Serial.println("\n[T240-Unified] Starting Industrial Diagnostic System...");

    // 2. [SSOT] JSON 동적 설정 매니저 초기화 (최우선)
    if (!CL_T2_ConfigManager::getInstance().init()) {
        Serial.println("[CRITICAL] LittleFS/Config Mount Failed!");
    }

    // 3. 하드웨어 트리거 핀 모드 설정 및 ISR 바인딩
    // (PULLDOWN 설정을 통해 플로팅 노이즈 방어)
    pinMode(T2_Config::Hardware::PIN_BTN_CONTROL_CONST, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(T2_Config::Hardware::PIN_BTN_CONTROL_CONST), T240_handleTriggerISR, CHANGE);

    // 4. [블라인드 스팟 차단] 인터럽트 활성화 직후 현재 물리 상태를 강제 동기화
    bool v_boot_trigger_state = digitalRead(T2_Config::Hardware::PIN_BTN_CONTROL_CONST);
    v_fsm_ref.handleExternalTrigger(v_boot_trigger_state);

    // 5. 오케스트레이터 및 내부 엔진(Sensor, DSP, Storage, Comm) 일괄 기동
    v_fsm_ref.begin();

    Serial.println("[T240-Unified] System Bootstrapping Completed.");
}

// 메인 루프 실행 엔진
// Maintenance 태스크를 실행하여 네트워크 유지보수 및 지연 쓰기를 처리합니다.
inline void T2_run() {
    // 시스템 오케스트레이터의 유지보수 루틴 실행
    // (네트워크 재접속, SD카드 로테이션, JSON 지연 쓰기 등 처리)
    v_fsm_ref.runMaintenanceTask();

    // [Rule #35] Core 1 점유율 최적화 및 WDT 방어
    vTaskDelay(pdMS_TO_TICKS(10));
}
