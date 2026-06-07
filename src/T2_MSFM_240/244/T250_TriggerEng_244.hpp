/* ============================================================================
 * File: T250_TriggerEng_244.hpp
 * Summary: 멀티모달(진동/소음) 하이브리드 결함 판정 엔진 (v243)
 * ============================================================================
 * [로직 상태 추적 (Logic State Tracker)]
 * - 유지: v242의 RMS 및 밴드 에너지 기반 룰베이스 판정 로직.
 * - 갱신: v243 4-Tier 설정을 통한 첨도(Kurtosis), 파고율(Crest) 임계값 동적화.
 * - 신규: [신뢰성] Trial Counter 연동을 통한 일시적 노이즈(False-Positive) 방어.
 * ========================================================================== */
#pragma once

#include "T210_Def_244.hpp"
#include "T215_Type_244.hpp"

class CL_T2_TriggerEngine {
private:
    uint8_t _trialCounter;

    // 내부 판정 헬퍼
    T2_Type::DetectionResult _checkVibRules(const T2_Type::ST_Slot_Vib& p_vib, const T2_Type::DynamicConfig& p_cfg);
    T2_Type::DetectionResult _checkAudioRules(const T2_Type::ST_Slot_Audio& p_aud, const T2_Type::DynamicConfig& p_cfg);

public:
    CL_T2_TriggerEngine();
    ~CL_T2_TriggerEngine() = default;

    /**
     * @brief 특징량 슬롯 데이터를 기반으로 종합 결함 판정 수행
     * @param p_slot 추출된 특징량 데이터 (Header, Vib, Audio, Tensor)
     * @param p_cfg 현재 시스템 설정 (임계값 참조)
     * @return 최종 판정 결과 (PASS 또는 NG 사유)
     */
    T2_Type::DetectionResult runDiagnostic(T2_Type::UnifiedFeatureSlot& p_slot, const T2_Type::DynamicConfig& p_cfg);

    /**
     * @brief 트리거 발생 회차 카운터 리셋
     */
    void resetCounter();
};
