/* ============================================================================
 * File: T250_TriggerEng_244.hpp
 * Summary: 멀티모달 하이브리드 결함 판정 엔진 구현부 (v243)
 * ========================================================================== */
#include "T250_TriggerEng_244.hpp"
#include "esp_log.h"

static const char* TAG = "T243_TRIG";

CL_T2_TriggerEngine::CL_T2_TriggerEngine() : _trialCounter(0) {}

T2_Type::DetectionResult CL_T2_TriggerEngine::runDiagnostic(T2_Type::UnifiedFeatureSlot& p_slot, const T2_Type::DynamicConfig& p_cfg) {
    T2_Type::DetectionResult v_finalRes = T2_Type::DetectionResult::PASS;

    // [Step 1] 진동 룰베이스 판정
    if (p_cfg.system.vib_enable) {
        v_finalRes = _checkVibRules(p_slot.vib, p_cfg);
        if (v_finalRes != T2_Type::DetectionResult::PASS) {
            p_slot.header.src = (uint8_t)T2_Type::TriggerSource::SW_RMS; // 단순화를 위해 RMS 우선 매핑 (필요 시 분화 가능)
        }
    }

    // [Step 2] 소음 룰베이스 판정
    if (v_finalRes == T2_Type::DetectionResult::PASS && p_cfg.system.audio_enable) {
        v_finalRes = _checkAudioRules(p_slot.audio, p_cfg);
        if (v_finalRes != T2_Type::DetectionResult::PASS) {
            p_slot.header.src = (uint8_t)T2_Type::TriggerSource::SW_RMS;
        }
    }

    // [Step 3] AI/ML 판정 (생략)

    // [Step 4] 신뢰성 검증 (Trial Counter)
    if (v_finalRes != T2_Type::DetectionResult::PASS) {
        _trialCounter++;
        p_slot.header.trial = _trialCounter;

        if (_trialCounter < 2) return T2_Type::DetectionResult::PASS;
    } else {
        _trialCounter = 0;
        p_slot.header.trial = 0;
        p_slot.header.src = (uint8_t)T2_Type::TriggerSource::NONE;
    }

    return v_finalRes;
}

T2_Type::DetectionResult CL_T2_TriggerEngine::_checkVibRules(const T2_Type::ST_Slot_Vib& p_vib, const T2_Type::DynamicConfig& p_cfg) {
    int v_axes = (p_cfg.vib_sensor.accel_axis_count > 3) ? 3 : p_cfg.vib_sensor.accel_axis_count;

    // 1. [3축 개정] 축별 RMS 및 통계 특징량(첨도, 파고율, 왜도) 격리 검사
    for (int i = 0; i < v_axes; i++) {
        if (p_vib.rms[i] > p_cfg.vib_trig.rms_thresh[i]) return T2_Type::DetectionResult::RULE_VIB_NG;
        if (p_vib.kurt[i] > p_cfg.vib_trig.kurt_ng_thresh[i]) return T2_Type::DetectionResult::RULE_VIB_NG;
        if (p_vib.crest[i] > p_cfg.vib_trig.crest_ng_thresh[i]) return T2_Type::DetectionResult::RULE_VIB_NG;
        if (p_vib.skew[i] > p_cfg.vib_trig.skew_ng_thresh[i]) return T2_Type::DetectionResult::RULE_VIB_NG;
    }

    // 2. [3축 개정] 축별 독립 밴드 에너지 대조 판정
    for (int i = 0; i < v_axes; i++) {
        for (int b = 0; b < p_cfg.vib_trig.active_band_count; b++) {
            if (p_cfg.vib_trig.band_en[b] && (p_vib.band_energy[i][b] > p_cfg.vib_trig.band_thresh[i][b])) {
                return T2_Type::DetectionResult::RULE_VIB_NG;
            }
        }
    }

    return T2_Type::DetectionResult::PASS;
}

T2_Type::DetectionResult CL_T2_TriggerEngine::_checkAudioRules(const T2_Type::ST_Slot_Audio& p_aud, const T2_Type::DynamicConfig& p_cfg) {
    // [듀얼 개정] Left/Right 독립 2채널 개별 검사
    for (int ch = 0; ch < 2; ch++) {
        // 1. RMS 검사
        if (p_aud.ch[ch].rms > p_cfg.aud_trig.rms_thresh[ch]) return T2_Type::DetectionResult::RULE_AUDIO_NG;

        // 2. 통계 특징량 검사 (첨도, 파고율, 왜도)
        if (p_aud.ch[ch].kurt > p_cfg.aud_trig.kurt_ng_thresh[ch]) return T2_Type::DetectionResult::RULE_AUDIO_NG;
        if (p_aud.ch[ch].crest > p_cfg.aud_trig.crest_ng_thresh[ch]) return T2_Type::DetectionResult::RULE_AUDIO_NG;
        if (p_aud.ch[ch].skew > p_cfg.aud_trig.skew_ng_thresh[ch]) return T2_Type::DetectionResult::RULE_AUDIO_NG;

        // 3. 밴드 에너지 검사
        for (int b = 0; b < p_cfg.aud_trig.active_band_count; b++) {
            if (p_cfg.aud_trig.band_en[b] && (p_aud.ch[ch].band_energy[b] > p_cfg.aud_trig.band_thresh[ch][b])) {
                return T2_Type::DetectionResult::RULE_AUDIO_NG;
            }
        }
    }

    return T2_Type::DetectionResult::PASS;
}

void CL_T2_TriggerEngine::resetCounter() {
    _trialCounter = 0;
}
