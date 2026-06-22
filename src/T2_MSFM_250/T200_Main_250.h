/* ============================================================================
 * File: T200_Main_250.h
 * Summary: 4-Tier 시스템 통합 진입점 및 전역 관리 (main.cpp 연동용)
 * ============================================================================
 * [로직 상태 추적 (Logic State Tracker)]
 * - 유지: esp_timer 기반 ISR 디바운싱 및 핵심 부팅 시퀀스.
 * - 갱신: 4-Tier 설정 체계에 따른 하드웨어 핀 맵핑 적용.
 * - 신규: [원칙 준수] 링커 오류 방지를 위한 inline 선언 및 SSOT 초기화 보장.
 * ========================================================================== */
#pragma once

#include <Arduino.h>
#include "esp_timer.h"
#include "T210_Def_250.hpp"
#include "T220_CfgMgr_250.hpp"
#include "T290_FsmMgr_250.hpp"

// FSM 매니저 싱글톤 참조
static CL_T2_FsmManager& g_T2_00_Main_FsmInst_ref = CL_T2_FsmManager::getInstance();

/**
 * @brief 외부 물리 버튼/신호 인터럽트 서비스 루틴
 * - [이슈 해결] ISR 컨텍스트 내에서 느린 digitalRead() API 호출을 원천 차단하여 IRAM_ATTR 실행 시간 및 지연 마진 최적화.
 * - FALLING 엣지 단방향 트리거를 활용해 즉각 CMD_START 명령을 안전하게 디스패치합니다.
 */
RTC_DATA_ATTR struct ST_CrashDiagnostics_t {
    uint8_t  last_detection_result;
    float    last_rms;
    uint32_t trial_count;
} g_CrashDiag;

// T2_00_BtnHandle_TriggerISR 전방 선언 필요성 해소
void IRAM_ATTR T2_00_BtnHandle_TriggerISR();

/**
 * @brief 외부 물리 버튼/신호 인터럽트 서비스 루틴
 * - [이슈 해결] ISR 컨텍스트 내에서 느린 digitalRead() API 호출을 원천 차단하여 IRAM_ATTR 실행 시간 및 지연 마진 최적화.
 * - FALLING 엣지 단방향 트리거를 활용해 즉각 CMD_START 명령을 안전하게 디스패치합니다.
 */
void IRAM_ATTR T2_00_BtnHandle_TriggerISR() {
    static uint64_t v_last_us = 0;
    uint64_t v_now_us = esp_timer_get_time();

    // 200ms 소프트웨어 디바운싱
    if (v_now_us - v_last_us > 200000ULL) {
        v_last_us = v_now_us;
        // 핀 상태를 digitalRead()로 읽지 않고, 인터럽트 발생 즉시 START 명령 송출
        g_T2_00_Main_FsmInst_ref.dispatchCommand(T2_Type::EM_SystemCommand_t::CMD_START);
    }
}

/**
 * @brief 시스템 초기화 (main.cpp의 setup()에서 호출)
 */
inline void T2_init() {
    // 1. 디버그 시리얼 기동
    Serial.begin(115200);

    vTaskDelay(pdMS_TO_TICKS(100));
    Serial.println("\n[MSFM_T2] 4-Tier Diagnostic System Booting...");

    // [신규] esp_reset_reason() 검사를 통한 RTC 메모리 크래시 레포트 안전 가드 초기화
    esp_reset_reason_t v_rstReason = esp_reset_reason();
    if (v_rstReason == ESP_RST_POWERON || v_rstReason == ESP_RST_EXT || v_rstReason == ESP_RST_BROWNOUT) {
        Serial.println("[RTC] Cold boot detected. Purging crash diagnostics...");
        memset(&g_CrashDiag, 0, sizeof(ST_CrashDiagnostics_t));
        std::atomic_thread_fence(std::memory_order_seq_cst);
        asm volatile("memw");
    } else {
        Serial.printf("[RTC] Warm reboot detected (Reason: %d). Preserving diagnostic state. Last RMS: %.4f\n",
                      (int)v_rstReason, g_CrashDiag.last_rms);
    }

    // 2. 하드웨어 핀 설정 및 초기 인터럽트 동기화
    pinMode(T2_Def::Global::Hardware::PIN_BTN_CONTROL_CONST, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(T2_Def::Global::Hardware::PIN_BTN_CONTROL_CONST), T2_00_BtnHandle_TriggerISR, FALLING);

    // 3. 시스템 오케스트레이터 기동 (내부적으로 CfgMgr, Sensor, Dsp, Storage 등 순차 초기화)
    if (!g_T2_00_Main_FsmInst_ref.init()) {
        Serial.println("[CRITICAL] System Orchestrator Initialization Failed!");
    }

    Serial.println("[MSFM-T2] System Bootstrap Completed. Ready for Monitoring.");
}

/**
 * @brief 메인 루프 실행 (main.cpp의 loop()에서 호출)
 */
inline void T2_run() {
    // 네트워크 서비스, 지연 쓰기, 자동 회전 등 관리 업무 수행
    g_T2_00_Main_FsmInst_ref.runMaintenance();

    // 루프 부하 분산 및 워치독 방어
    vTaskDelay(pdMS_TO_TICKS(10));
}
