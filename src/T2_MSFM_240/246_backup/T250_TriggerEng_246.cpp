/* ============================================================================
 * File: T250_TriggerEng_246.cpp
 * Summary: 멀티모달 결함 판정 엔진 구현부 (v245 개정판)
 * ============================================================================ */

#include "T250_TriggerEng_246.hpp"
#include "esp_log.h"

static const char* TAG = "T245_TRIG";

CL_T2_TriggerEngine::CL_T2_TriggerEngine() : _trialCounter(0) {}

T2_Type::EM_DetectionResult_t CL_T2_TriggerEngine::runDiagnostic(T2_Type::ST_UnifiedFeatureSlot_t& p_slot, const T2_Type::ST_DynamicConfig_t& p_cfg) {
    // [Step 0] 과도 응답 배제 조건 검사 (기동 초기 및 지정 구간 스킵)
    uint32_t uptimeSec = p_slot.header.uptime / 1000;
    if (uptimeSec < p_cfg.decision.valid_start_sec || (p_cfg.decision.valid_end_sec > 0.001f && uptimeSec > p_cfg.decision.valid_end_sec)) {
        return T2_Type::EM_DetectionResult_t::PASS;
    }

    T2_Type::EM_DetectionResult_t v_accelRes = T2_Type::EM_DetectionResult_t::PASS;
    T2_Type::EM_DetectionResult_t v_gyroRes  = T2_Type::EM_DetectionResult_t::PASS;
    T2_Type::EM_DetectionResult_t v_audioRes = T2_Type::EM_DetectionResult_t::PASS;

    // [Step 1] 가속도 룰베이스 판정
    if (p_cfg.accel.enable) {
        v_accelRes = _checkAccelRules(p_slot.accel, p_cfg);
    }

    // [Step 2] 자이로 룰베이스 판정
    if (p_cfg.gyro.enable) {
        v_gyroRes = _checkGyroRules(p_slot.gyro, p_cfg);
    }

    // [Step 3] 소음 룰베이스 판정
    if (p_cfg.audio.enable) {
        v_audioRes = _checkAudioRules(p_slot.audio, p_cfg);
    }

    // 최종 결과 통합 (마스킹 없는 종합 결함 판정)
    T2_Type::EM_DetectionResult_t v_finalRes = T2_Type::EM_DetectionResult_t::PASS;
    uint8_t v_triggerSrc = (uint8_t)T2_Type::EM_TriggerSource_t::NONE;

    if (v_accelRes != T2_Type::EM_DetectionResult_t::PASS) {
        v_finalRes = v_accelRes;
        v_triggerSrc = (uint8_t)T2_Type::EM_TriggerSource_t::SW_RMS;
    }
    if (v_gyroRes != T2_Type::EM_DetectionResult_t::PASS) {
        v_finalRes = v_gyroRes;
        v_triggerSrc = (uint8_t)T2_Type::EM_TriggerSource_t::SW_RMS;
    }
    if (v_audioRes != T2_Type::EM_DetectionResult_t::PASS) {
        v_finalRes = v_audioRes;
        v_triggerSrc = (uint8_t)T2_Type::EM_TriggerSource_t::SW_RMS;
    }

    // [Step 4] 신뢰성 검증 (Trial Counter)
    if (v_finalRes != T2_Type::EM_DetectionResult_t::PASS) {
        _trialCounter++;
        p_slot.header.trial = _trialCounter;
        p_slot.header.src = v_triggerSrc;

        if (_trialCounter < T2_Def::Global::Decision::MAX_TRIAL_COUNT_DEF) return T2_Type::EM_DetectionResult_t::PASS;
    } else {
        _trialCounter = 0;
        p_slot.header.trial = 0;
        p_slot.header.src = (uint8_t)T2_Type::EM_TriggerSource_t::NONE;
    }

    return v_finalRes;
}

T2_Type::EM_DetectionResult_t CL_T2_TriggerEngine::_checkAccelRules(const T2_Type::ST_Slot_Accel_t& p_acc, const T2_Type::ST_DynamicConfig_t& p_cfg) {
    for (int i = 0; i < T2_Def::Accel::Sensor::AXIS_MAX; i++) {
        if (!(p_cfg.accel.axis_mask & (1 << i))) continue;

        if (p_acc.rms[i] > p_cfg.accel.rms_thresh[i]) return T2_Type::EM_DetectionResult_t::RULE_VIB_NG;
        if (p_acc.kurt[i] > p_cfg.accel.kurt_ng_thresh[i]) return T2_Type::EM_DetectionResult_t::RULE_VIB_NG;
        if (p_acc.crest[i] > p_cfg.accel.crest_ng_thresh[i]) return T2_Type::EM_DetectionResult_t::RULE_VIB_NG;
        if (p_acc.skew[i] > p_cfg.accel.skew_ng_thresh[i]) return T2_Type::EM_DetectionResult_t::RULE_VIB_NG;
    }

    // 밴드 에너지 판정
    for (int i = 0; i < T2_Def::Accel::Sensor::AXIS_MAX; i++) {
        if (!(p_cfg.accel.axis_mask & (1 << i))) continue;

        for (int b = 0; b < p_cfg.accel.active_band_count; b++) {
            if (p_cfg.accel.band_en[b] && (p_acc.band_energy[i][b] > p_cfg.accel.band_thresh[i][b])) {
                return T2_Type::EM_DetectionResult_t::RULE_VIB_NG;
            }
        }
    }

    return T2_Type::EM_DetectionResult_t::PASS;
}

T2_Type::EM_DetectionResult_t CL_T2_TriggerEngine::_checkGyroRules(const T2_Type::ST_Slot_Gyro_t& p_gyr, const T2_Type::ST_DynamicConfig_t& p_cfg) {
    for (int i = 0; i < T2_Def::Gyro::Sensor::AXIS_MAX; i++) {
        if (!(p_cfg.gyro.axis_mask & (1 << i))) continue;

        if (p_gyr.rms[i] > p_cfg.gyro.rms_thresh[i]) return T2_Type::EM_DetectionResult_t::RULE_VIB_NG;
        if (p_gyr.kurt[i] > p_cfg.gyro.kurt_ng_thresh[i]) return T2_Type::EM_DetectionResult_t::RULE_VIB_NG;
        if (p_gyr.crest[i] > p_cfg.gyro.crest_ng_thresh[i]) return T2_Type::EM_DetectionResult_t::RULE_VIB_NG;
        if (p_gyr.skew[i] > p_cfg.gyro.skew_ng_thresh[i]) return T2_Type::EM_DetectionResult_t::RULE_VIB_NG;
    }

    // 밴드 에너지 판정
    for (int i = 0; i < T2_Def::Gyro::Sensor::AXIS_MAX; i++) {
        if (!(p_cfg.gyro.axis_mask & (1 << i))) continue;

        for (int b = 0; b < p_cfg.gyro.active_band_count; b++) {
            if (p_cfg.gyro.band_en[b] && (p_gyr.band_energy[i][b] > p_cfg.gyro.band_thresh[i][b])) {
                return T2_Type::EM_DetectionResult_t::RULE_VIB_NG;
            }
        }
    }

    return T2_Type::EM_DetectionResult_t::PASS;
}

T2_Type::EM_DetectionResult_t CL_T2_TriggerEngine::_checkAudioRules(const T2_Type::ST_Slot_Audio_t& p_aud, const T2_Type::ST_DynamicConfig_t& p_cfg) {
    for (int ch = 0; ch < T2_Def::Audio::Sensor::CHANNELS_MAX; ch++) {
        if (!(p_cfg.audio.channel_mask & (1 << ch))) continue;

        if (p_aud.ch[ch].rms > p_cfg.audio.rms_thresh[ch]) return T2_Type::EM_DetectionResult_t::RULE_AUDIO_NG;
        if (p_aud.ch[ch].kurt > p_cfg.audio.kurt_ng_thresh[ch]) return T2_Type::EM_DetectionResult_t::RULE_AUDIO_NG;
        if (p_aud.ch[ch].crest > p_cfg.audio.crest_ng_thresh[ch]) return T2_Type::EM_DetectionResult_t::RULE_AUDIO_NG;
        if (p_aud.ch[ch].skew > p_cfg.audio.skew_ng_thresh[ch]) return T2_Type::EM_DetectionResult_t::RULE_AUDIO_NG;

        for (int b = 0; b < p_cfg.audio.active_band_count; b++) {
            if (p_cfg.audio.band_en[b] && (p_aud.ch[ch].band_energy[b] > p_cfg.audio.band_thresh[ch][b])) {
                return T2_Type::EM_DetectionResult_t::RULE_AUDIO_NG;
            }
        }
    }

    return T2_Type::EM_DetectionResult_t::PASS;
}

void CL_T2_TriggerEngine::resetCounter() {
    _trialCounter = 0;
}
