/* ============================================================================
 * File: T248_SeqBuild_250.cpp
 * Summary: TinyML 시퀀스 텐서 빌더 구현부
 * ============================================================================ */

#include "T248_SeqBuild_250.hpp"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "T248_SEQ";

CL_T2_SequenceBuilder::CL_T2_SequenceBuilder()
    : _dataFlat(nullptr), _frames(0), _dim(0), _strideDim(0), _head(0), _isFull(false) {}

CL_T2_SequenceBuilder::~CL_T2_SequenceBuilder() {
    if (_dataFlat) {
        heap_caps_free(_dataFlat);
        _dataFlat = nullptr;
    }
}

bool CL_T2_SequenceBuilder::init(uint16_t p_sequenceFrames, uint16_t p_featureDim) {
    _frames = (p_sequenceFrames > T2_Def::Global::System::SEQUENCE_FRAMES_MAX) ?
               T2_Def::Global::System::SEQUENCE_FRAMES_MAX : p_sequenceFrames;

    _dim = (p_featureDim > T2_Def::AI::Tensor::MFCC_DIM_MAX) ?
            T2_Def::AI::Tensor::MFCC_DIM_MAX : p_featureDim;

    _strideDim = (p_featureDim + 3) & ~3;

    if (!_dataFlat) {
        size_t v_allocSize = _frames * _strideDim * sizeof(float);
        _dataFlat = (float*)heap_caps_aligned_alloc(16, v_allocSize, MALLOC_CAP_SPIRAM);

        if (!_dataFlat) {
            ESP_LOGE(TAG, "PSRAM Allocation Failed! Req: %zu bytes", v_allocSize);
            return false;
        }
        memset(_dataFlat, 0, v_allocSize);
    }

    reset();
    ESP_LOGI(TAG, "SeqBuilder Init: %d x %d (Stride: %d)", _frames, _dim, _strideDim);
    return true;
}

void CL_T2_SequenceBuilder::pushVector(const float* p_vector) {
    if (!p_vector || !_dataFlat || _frames == 0) return;

    float* v_targetSlot = &_dataFlat[_head * _strideDim];
    memcpy(v_targetSlot, p_vector, sizeof(float) * _dim);

    if (_strideDim > _dim) {
        for (uint16_t j = _dim; j < _strideDim; j++) {
            v_targetSlot[j] = 0.0f;
        }
    }

    _head++;
    if (_head >= _frames) {
        _head = 0;
        _isFull = true;
    }
}

void CL_T2_SequenceBuilder::reset() {
    _head = 0;
    _isFull = false;
}

void CL_T2_SequenceBuilder::getSequenceFlat(float* p_outBuffer, size_t p_maxOutSize) const {
    if (!p_outBuffer || !_dataFlat) return;

    if (((uintptr_t)p_outBuffer & 15) != 0) {
        ESP_LOGW(TAG, "Performance Warning: Output buffer alignment check failed.");
    }

    size_t v_validCount = _isFull ? _frames : _head;
    if (v_validCount == 0) return;

    size_t v_totalPackedSize = _frames * _dim * sizeof(float);
    if (p_maxOutSize < v_totalPackedSize) {
        ESP_LOGE(TAG, "Output buffer too small! Req: %zu", v_totalPackedSize);
        return;
    }

    uint16_t v_oldestIdx = _isFull ? _head : 0;
    size_t v_currentOffset = 0;

    for (uint16_t i = 0; i < v_validCount; i++) {
        uint16_t v_readIdx = (v_oldestIdx + i) % _frames;
        memcpy(p_outBuffer + v_currentOffset, &_dataFlat[v_readIdx * _strideDim], sizeof(float) * _dim);
        v_currentOffset += _dim;
    }

    if (v_validCount < _frames) {
        size_t v_paddingElements = (_frames - v_validCount) * _dim;
        memset(p_outBuffer + v_currentOffset, 0, v_paddingElements * sizeof(float));
    }
}

// ========================================================================
// MultiRateTimeAligner 구현
// ========================================================================
MultiRateTimeAligner::MultiRateTimeAligner(uint64_t p_maxSkewUs)
    : _max_allowed_skew_us(p_maxSkewUs) {
    _prev_vib.is_valid = false;
    _curr_vib.is_valid = false;
}

void MultiRateTimeAligner::injectNewVibSample(const float* p_rawFeats, uint64_t p_ts) {
    _prev_vib = _curr_vib; // 이전 스냅샷 백업

    constexpr size_t VIB_FEAT_CNT = T2_Def::AI::Tensor::MFCC_COEFFS_DEF * T2_Def::AI::Tensor::MFCC_COMPONENTS_DEF * (T2_Def::Accel::Sensor::AXIS_MAX + T2_Def::Gyro::Sensor::AXIS_MAX);
    for (size_t i = 0; i < VIB_FEAT_CNT; i++) {
        _curr_vib.features[i] = G_T2_10_Def_FPU_SAN_FLOAT(p_rawFeats[i]);
    }
    _curr_vib.timestamp_us = p_ts;
    _curr_vib.is_valid = true;

    if (!_prev_vib.is_valid) {
        _prev_vib = _curr_vib; // 최초 기동 시 초기화 가드
    }
}

bool MultiRateTimeAligner::getAlignedVibration(uint64_t p_audioTs, float* p_outFeats) {
    if (!_curr_vib.is_valid) return false;

    int64_t v_skew = static_cast<int64_t>(p_audioTs) - static_cast<int64_t>(_curr_vib.timestamp_us);

    // 임계값 초과 시 강제 Drop & Shift (회복 모듈)
    if (std::abs(v_skew) > static_cast<int64_t>(_max_allowed_skew_us)) {
        ESP_LOGW("T248_TIME", "Clock drift skew detected: %lld us. Soft drop and align.", v_skew);
        _prev_vib = _curr_vib;
        _prev_vib.timestamp_us = p_audioTs - 20000; // 가상 오차 보정 시간 임의 주입
        _curr_vib.timestamp_us = p_audioTs;
        v_skew = 0;
    }

    constexpr size_t VIB_FEAT_CNT = T2_Def::AI::Tensor::MFCC_COEFFS_DEF * T2_Def::AI::Tensor::MFCC_COMPONENTS_DEF * (T2_Def::Accel::Sensor::AXIS_MAX + T2_Def::Gyro::Sensor::AXIS_MAX);

    // 보간 연산 및 uint64_t 언더플로우 방어 가드
    if (_curr_vib.timestamp_us <= _prev_vib.timestamp_us || p_audioTs <= _prev_vib.timestamp_us) {
        // 시간축 역전 또는 비정상 흐름 시 보간을 생략하고 최신 특징량 복제 전달
        std::copy(_curr_vib.features, _curr_vib.features + VIB_FEAT_CNT, p_outFeats);
        return true;
    }

    uint64_t v_totalDelta = _curr_vib.timestamp_us - _prev_vib.timestamp_us;
    float v_alpha = static_cast<float>(p_audioTs - _prev_vib.timestamp_us) / static_cast<float>(v_totalDelta);

    // 외삽(Extrapolation) 방지를 위한 계수 클램핑
    if (v_alpha < 0.0f) v_alpha = 0.0f;
    if (v_alpha > 1.0f) v_alpha = 1.0f;

    for (size_t i = 0; i < VIB_FEAT_CNT; i++) {
        p_outFeats[i] = _prev_vib.features[i] + v_alpha * (_curr_vib.features[i] - _prev_vib.features[i]);
    }
    return true;
}

// ========================================================================
// DynamicTensorBinder 구현
// ========================================================================
DynamicTensorBinder::DynamicTensorBinder(size_t p_nnInputSize)
    : _audio_offset(INVALID_OFFSET), _vib_offset(INVALID_OFFSET),
      _total_active_elements(0), _nn_input_boundary(p_nnInputSize) {}

void DynamicTensorBinder::updateTensorOffsets(uint16_t p_activeMaskFlags, size_t p_audioFeatCnt, size_t p_vibFeatCnt) {
    size_t v_currentPtr = 0;

    // active_mask_flags는 EM_DataPayloadType_t 열거형에 정의된 비트 플래그를 정합 참조합니다.
    // VIB_ONLY (1) = 0x01, AUDIO_ONLY (2) = 0x02, VIB_AUDIO_BOTH (3) = 0x03
    bool v_hasAudio = (p_activeMaskFlags & static_cast<uint16_t>(T2_Type::EM_DataPayloadType_t::AUDIO_ONLY)) != 0;
    bool v_hasVib   = (p_activeMaskFlags & static_cast<uint16_t>(T2_Type::EM_DataPayloadType_t::VIB_ONLY)) != 0;

    if (v_hasAudio) {
        _audio_offset = v_currentPtr;
        v_currentPtr += p_audioFeatCnt;
    } else {
        _audio_offset = INVALID_OFFSET;
    }

    if (v_hasVib) {
        _vib_offset = v_currentPtr;
        v_currentPtr += p_vibFeatCnt;
    } else {
        _vib_offset = INVALID_OFFSET;
    }

    _total_active_elements = v_currentPtr;
}

bool DynamicTensorBinder::buildFlattenTensor(float* p_targetTensor, const float* p_audioSrc, const float* p_vibSrc, uint16_t p_mask) {
    if (_total_active_elements > _nn_input_boundary) {
        return false;
    }

    // 전체 영역 안전 소거 (Fixed Shape 가정을 위해 0.0f 패딩 강제 적용)
    std::fill_n(p_targetTensor, _nn_input_boundary, 0.0f);

    constexpr size_t AUD_FEAT_CNT = T2_Def::AI::Tensor::MFCC_COEFFS_DEF * T2_Def::AI::Tensor::MFCC_COMPONENTS_DEF * T2_Def::Audio::Sensor::CHANNELS_MAX;
    constexpr size_t VIB_FEAT_CNT = T2_Def::AI::Tensor::MFCC_COEFFS_DEF * T2_Def::AI::Tensor::MFCC_COMPONENTS_DEF * (T2_Def::Accel::Sensor::AXIS_MAX + T2_Def::Gyro::Sensor::AXIS_MAX);

    bool v_hasAudio = (p_mask & static_cast<uint16_t>(T2_Type::EM_DataPayloadType_t::AUDIO_ONLY)) != 0;
    bool v_hasVib   = (p_mask & static_cast<uint16_t>(T2_Type::EM_DataPayloadType_t::VIB_ONLY)) != 0;

    // 오디오 복사 (INVALID_OFFSET이 아닌 경우 복사)
    if (v_hasAudio && _audio_offset != INVALID_OFFSET) {
        std::copy(p_audioSrc, p_audioSrc + AUD_FEAT_CNT, p_targetTensor + _audio_offset);
    }

    // 진동 복사
    if (v_hasVib && _vib_offset != INVALID_OFFSET) {
        std::copy(p_vibSrc, p_vibSrc + VIB_FEAT_CNT, p_targetTensor + _vib_offset);
    }

    return true;
}
