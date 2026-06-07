/* ============================================================================
 * File: T250_TriggerEng_245.hpp
 * Summary: 멀티모달(가속도/자이로/소음) 결함 판정 엔진
 * ============================================================================ */

#pragma once

#include "T210_Def_245.hpp"
#include "T215_Type_245.hpp"

/**
 * @class CL_T2_TriggerEngine
 * @brief 추출된 가속도, 자이로, 오디오의 다차원 특징들을 사전에 규정된 룰(임계치)에 대입하여 장비 결함 여부를 실시간 판정하는 감지 엔진
 */
class CL_T2_TriggerEngine {
private:
    uint8_t _trialCounter; ///< 연속 NG 판정의 유효 횟수 카운터

    // 내부 판정 헬퍼 함수
    T2_Type::EM_DetectionResult_t _checkAccelRules(const T2_Type::ST_Slot_Accel_t& p_acc, const T2_Type::ST_DynamicConfig_t& p_cfg);
    T2_Type::EM_DetectionResult_t _checkGyroRules(const T2_Type::ST_Slot_Gyro_t& p_gyr, const T2_Type::ST_DynamicConfig_t& p_cfg);
    T2_Type::EM_DetectionResult_t _checkAudioRules(const T2_Type::ST_Slot_Audio_t& p_aud, const T2_Type::ST_DynamicConfig_t& p_cfg);

public:
    CL_T2_TriggerEngine();
    ~CL_T2_TriggerEngine() = default;

    /**
     * @brief 추출 지표들을 종합하여 장비 이상 판정을 진단
     * @param p_slot 진단 특징 데이터가 담긴 연산 슬롯
     * @param p_cfg 이상 룰 임계치가 기술된 설정 정보
     * @return 진단 판정 결과 코드 (PASS 또는 에러 룰 종류)
     */
    T2_Type::EM_DetectionResult_t runDiagnostic(T2_Type::ST_UnifiedFeatureSlot_t& p_slot, const T2_Type::ST_DynamicConfig_t& p_cfg);

    /**
     * @brief 기동 초기 또는 오경보 방지 카운터 상태 소거
     */
    void resetCounter();
};
