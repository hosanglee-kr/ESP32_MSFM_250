/* ============================================================================
 * File: T250_Trigger_250.hpp
 * Summary: 멀티모달(가속도/자이로/소음) 결함 판정 엔진
 * ============================================================================ */

#pragma once

#include "T210_Def_250.hpp"
#include "T215_Type_250.hpp"

class CL_T2_TriggerEngine {
private:
    uint8_t 	_trialCounter;		// 시도 횟수

    // 가속도 결함 판정 함수
    T2_Type::EM_DetectionResult_t _checkAccelRules(const T2_Type::ST_Slot_Accel_t& p_acc, const T2_Type::ST_DynamicConfig_t& p_cfg);
	// 자이로 결함 판정 함수
	T2_Type::EM_DetectionResult_t _checkGyroRules(const T2_Type::ST_Slot_Gyro_t& p_gyr, const T2_Type::ST_DynamicConfig_t& p_cfg);
    // 오디오 결함 판정 함수
	T2_Type::EM_DetectionResult_t _checkAudioRules(const T2_Type::ST_Slot_Audio_t& p_aud, const T2_Type::ST_DynamicConfig_t& p_cfg);

public:
    CL_T2_TriggerEngine();
    ~CL_T2_TriggerEngine() = default;

	// 멀티모달 결함 판정 실행
    T2_Type::EM_DetectionResult_t runDiagnostic(T2_Type::ST_FeatureSlot_Aud_t& p_audSlot,
                                                const T2_Type::ST_FeatureSlot_Vib_t& p_vibSlot,
                                                const T2_Type::ST_DynamicConfig_t& p_cfg);

	// 시도 횟수 초기화
    void resetCounter();
};

