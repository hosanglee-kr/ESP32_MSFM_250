/* ============================================================================
 * File: T220_Mfcc_234.h
 * Summary: Master Controller Public Interface (FSM Enabled)
 * ========================================================================== */
#pragma once
#include "T210_Def_234.h"

class CL_T20_Mfcc {
public:
    struct ST_Impl;
    CL_T20_Mfcc();
    ~CL_T20_Mfcc();

    bool begin(const ST_T20_Config_t* p_cfg = nullptr);
    bool start();
    void stop();
    void run(); // Watchdog, Deep Sleep 및 물리 버튼 루프

    // [신규] FSM 상태 제어를 위한 비동기 커맨드 전달 인터페이스
    void postCommand(EM_T20_Command_t cmd);

    void printStatus(Stream& out) const;

private:
    CL_T20_Mfcc(const CL_T20_Mfcc&) = delete;
    CL_T20_Mfcc& operator=(const CL_T20_Mfcc&) = delete;
    ST_Impl* _impl;

    friend void T20_sensorTask(void* p_arg);
    friend void T20_processTask(void* p_arg);
    friend void T20_recorderTask(void* p_arg);
};

extern CL_T20_Mfcc* g_t20;

