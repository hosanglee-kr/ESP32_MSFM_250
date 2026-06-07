/* ============================================================================
 * File: T250_TriggerEng_245.cpp
 * Summary: 멀티모달 결함 판정 엔진 구현부
 * ============================================================================ */

#include "T250_TriggerEng_245.hpp"
#include "esp_log.h"

static const char* TAG = "T245_TRIG";

/**
 * @brief CL_T2_TriggerEngine 생성자
 */
CL_T2_TriggerEngine::CL_T2_TriggerEngine() : _trialCounter(0) {}

/**
 * @brief 융합 센서 특징량 슬롯 정보를 토대로 가속도, 자이로, 소음 임계치를 평가하여 결함 등급을 진단
 * @param p_slot 진단 대상 특징량 데이터가 담긴 통합 슬롯
 * @param p_cfg 이상 탐지 임계치 설정 객체
 * @return 진단 결과 등급 코드
 */
T2_Type::EM_DetectionResult_t CL_T2_TriggerEngine::runDiagnostic(T2_Type::ST_UnifiedFeatureSlot_t& p_slot, const T2_Type::ST_DynamicConfig_t& p_cfg) {
    T2_Type::EM_DetectionResult_t v_finalRes = T2_Type::EM_DetectionResult_t::PASS;

    // [처리 단위 1] 가속도 룰 임계 판정
    if (p_cfg.accel.enable) {
        v_finalRes = _checkAccelRules(p_slot.accel, p_cfg);
        if (v_finalRes != T2_Type::EM_DetectionResult_t::PASS) {
            p_slot.header.src = (uint8_t)T2_Type::EM_TriggerSource_t::SW_RMS;
        }
    }

    // [처리 단위 2] 자이로 룰 임계 판정
    if (v_finalRes == T2_Type::EM_DetectionResult_t::PASS && p_cfg.gyro.enable) {
        v_finalRes = _checkGyroRules(p_slot.gyro, p_cfg);
        if (v_finalRes != T2_Type::EM_DetectionResult_t::PASS) {
            p_slot.header.src = (uint8_t)T2_Type::EM_TriggerSource_t::SW_RMS;
        }
    }

    // [처리 단위 3] 오디오 룰 임계 판정
    if (v_finalRes == T2_Type::EM_DetectionResult_t::PASS && p_cfg.audio.enable) {
        v_finalRes = _checkAudioRules(p_slot.audio, p_cfg);
        if (v_finalRes != T2_Type::EM_DetectionResult_t::PASS) {
            p_slot.header.src = (uint8_t)T2_Type::EM_TriggerSource_t::SW_RMS;
        }
    }

    // [처리 단위 4] 연속 에러 신뢰도 필터링 검사 (최소 감지 프레임 제한)
    if (v_finalRes != T2_Type::EM_DetectionResult_t::PASS) {
        _trialCounter++;
        p_slot.header.trial = _trialCounter;

        if (_trialCounter < T2_Def::Global::Decision::MAX_TRIAL_COUNT_DEF) {
            return T2_Type::EM_DetectionResult_t::PASS;
        }
    } else {
        _trialCounter = 0;
        p_slot.header.trial = 0;
        p_slot.header.src = (uint8_t)T2_Type::EM_TriggerSource_t::NONE;
    }

    return v_finalRes;
}

/**
 * @brief 가속도 특징들에 대해 RMS, 첨도, 왜도, 밴드 에너지를 평가
 * @param p_acc 가속도 특징 슬롯
 * @param p_cfg 이상 탐지 임계치 설정
 * @return 진단 결과 코드
 */
T2_Type::EM_DetectionResult_t CL_T2_TriggerEngine::_checkAccelRules(const T2_Type::ST_Slot_Accel_t& p_acc, const T2_Type::ST_DynamicConfig_t& p_cfg) {
    // 활성 축 통계값 임계 평가
    for (int i = 0; i < T2_Def::Accel::Sensor::AXIS_MAX; i++) {
        if (!(p_cfg.accel.axis_mask & (1 << i))) continue;

        if (p_acc.rms[i] > p_cfg.accel.rms_thresh[i])     return T2_Type::EM_DetectionResult_t::RULE_VIB_NG;
        if (p_acc.kurt[i] > p_cfg.accel.kurt_ng_thresh[i])   return T2_Type::EM_DetectionResult_t::RULE_VIB_NG;
        if (p_acc.crest[i] > p_cfg.accel.crest_ng_thresh[i]) return T2_Type::EM_DetectionResult_t::RULE_VIB_NG;
        if (p_acc.skew[i] > p_cfg.accel.skew_ng_thresh[i])   return T2_Type::EM_DetectionResult_t::RULE_VIB_NG;
    }

    // 주파수 밴드 대역별 통계 에너지 임계 평가
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

/**
 * @brief 자이로 특징들에 대해 RMS, 첨도, 왜도, 밴드 에너지를 평가
 * @param p_gyr 자이로 특징 슬롯
 * @param p_cfg 이상 탐지 임계치 설정
 * @return 진단 결과 코드
 */
T2_Type::EM_DetectionResult_t CL_T2_TriggerEngine::_checkGyroRules(const T2_Type::ST_Slot_Gyro_t& p_gyr, const T2_Type::ST_DynamicConfig_t& p_cfg) {
    // 활성 축 통계값 임계 평가
    for (int i = 0; i < T2_Def::Gyro::Sensor::AXIS_MAX; i++) {
        if (!(p_cfg.gyro.axis_mask & (1 << i))) continue;

        if (p_gyr.rms[i] > p_cfg.gyro.rms_thresh[i])     return T2_Type::EM_DetectionResult_t::RULE_VIB_NG;
        if (p_gyr.kurt[i] > p_cfg.gyro.kurt_ng_thresh[i])   return T2_Type::EM_DetectionResult_t::RULE_VIB_NG;
        if (p_gyr.crest[i] > p_cfg.gyro.crest_ng_thresh[i]) return T2_Type::EM_DetectionResult_t::RULE_VIB_NG;
        if (p_gyr.skew[i] > p_cfg.gyro.skew_ng_thresh[i])   return T2_Type::EM_DetectionResult_t::RULE_VIB_NG;
    }

    // 주파수 밴드 대역별 통계 에너지 임계 평가
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

/**
 * @brief 오디오 채널 특징들에 대해 RMS, 첨도, 왜도, 밴드 에너지를 평가
 * @param p_aud 오디오 특징 슬롯
 * @param p_cfg 이상 탐지 임계치 설정
 * @return 진단 결과 코드
 */
T2_Type::EM_DetectionResult_t CL_T2_TriggerEngine::_checkAudioRules(const T2_Type::ST_Slot_Audio_t& p_aud, const T2_Type::ST_DynamicConfig_t& p_cfg) {
    // 활성 오디오 채널 통계값 및 밴드 에너지 임계 평가
    for (int ch = 0; ch < T2_Def::Audio::Sensor::CHANNELS_MAX; ch++) {
        if (!(p_cfg.audio.channel_mask & (1 << ch))) continue;

        if (p_aud.ch[ch].rms > p_cfg.audio.rms_thresh[ch])     return T2_Type::EM_DetectionResult_t::RULE_AUDIO_NG;
        if (p_aud.ch[ch].kurt > p_cfg.audio.kurt_ng_thresh[ch])   return T2_Type::EM_DetectionResult_t::RULE_AUDIO_NG;
        if (p_aud.ch[ch].crest > p_cfg.audio.crest_ng_thresh[ch]) return T2_Type::EM_DetectionResult_t::RULE_AUDIO_NG;
        if (p_aud.ch[ch].skew > p_cfg.audio.skew_ng_thresh[ch])   return T2_Type::EM_DetectionResult_t::RULE_AUDIO_NG;

        for (int b = 0; b < p_cfg.audio.active_band_count; b++) {
            if (p_cfg.audio.band_en[b] && (p_aud.ch[ch].band_energy[b] > p_cfg.audio.band_thresh[ch][b])) {
                return T2_Type::EM_DetectionResult_t::RULE_AUDIO_NG;
            }
        }
    }

    return T2_Type::EM_DetectionResult_t::PASS;
}

/**
 * @brief 카운터 리셋
 */
void CL_T2_TriggerEngine::resetCounter() {
    _trialCounter = 0;
}
