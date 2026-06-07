/* ============================================================================
 * File: T221_Mfcc_Int_235.h
 * Summary: 통합 내부 구현체 (Pimpl) 및 FSM 상태 관리
 * ========================================================================== */
#pragma once
#include "T220_Mfcc_234.h"
#include "T231_Dsp_234.h"
#include "T232_Sensor_234.h"
#include "T233_Seq_234.h"
#include "T234_Storage_234.h"
#include "T240_Comm_234.h"

struct CL_T20_Mfcc::ST_Impl {
    CL_T20_SensorEngine    sensor;
    CL_T20_DspPipeline     dsp;
    CL_T20_SequenceBuilder seq_builder;
    CL_T20_StorageService  storage;
    CL_T20_CommService     comm;

    // RTOS 자원
    TaskHandle_t           sensor_task    = nullptr;
    TaskHandle_t           process_task   = nullptr;
    TaskHandle_t           recorder_task  = nullptr;

    QueueHandle_t          frame_queue    = nullptr;
    QueueHandle_t          recorder_queue = nullptr;
    QueueHandle_t          cmd_queue      = nullptr; // [신규] FSM 제어 명령 큐

    SemaphoreHandle_t      mutex          = nullptr;

    // [신규] FSM 전역 상태 변수
    EM_T20_SysState_t      current_state  = EN_T20_STATE_INIT;

    // 핑퐁 버퍼 (PSRAM 적재)
    alignas(16) float raw_buffer[3][T20::C10_Sys::RAW_FRAME_BUFFERS][4096];

    uint8_t   active_fill_buffer  = 0;
    uint16_t  active_sample_index = 0;
    
    // 1.2KB Feature 전송 복사 낭비를 막고 Race Condition을 방어하기 위한 정적 메모리 풀
    // 큐 사이즈(8)보다 여유있게 10칸으로 할당하여 오버랩 덮어쓰기 원천 차단
    alignas(16) ST_T20_FeatureVector_t feature_pool[10];
    uint8_t active_feature_slot = 0;

    
    

    // --- 이벤트 감시 자원 ---
    uint32_t        last_trigger_ms     = 0;
    uint32_t        sample_counter      = 0;
    uint32_t        last_btn_ms         = 0;

    ST_T20_Config_t cfg;
    bool            running = false;

    ST_Impl() : sensor(SPI), comm() {}
};


