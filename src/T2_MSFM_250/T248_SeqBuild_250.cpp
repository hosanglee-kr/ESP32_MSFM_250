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
MultiRateTimeAligner::MultiRateTimeAligner(uint64_t max_skew_us)
    : _max_allowed_skew_us(max_skew_us) {
    _prev_vib.is_valid = false;
    _curr_vib.is_valid = false;
}

void MultiRateTimeAligner::injectNewVibSample(const float* raw_feats, uint64_t ts) {
    _prev_vib = _curr_vib; // 이전 스냅샷 백업

    constexpr size_t VIB_FEAT_CNT = T2_Def::AI::Tensor::MFCC_COEFFS_DEF * T2_Def::AI::Tensor::MFCC_COMPONENTS_DEF * (T2_Def::Accel::Sensor::AXIS_MAX + T2_Def::Gyro::Sensor::AXIS_MAX);
    for (size_t i = 0; i < VIB_FEAT_CNT; i++) {
        _curr_vib.features[i] = SMEA_SAN_FLOAT(raw_feats[i]);
    }
    _curr_vib.timestamp_us = ts;
    _curr_vib.is_valid = true;

    if (!_prev_vib.is_valid) {
        _prev_vib = _curr_vib; // 최초 기동 시 초기화 가드
    }
}

bool MultiRateTimeAligner::getAlignedVibration(uint64_t audio_ts, float* out_feats) {
    if (!_curr_vib.is_valid) return false;

    // 양방향 타임스탬프 스큐 절댓값 검증 가드 (언더런 및 오버런 동시 차단)
    int64_t skew = static_cast<int64_t>(audio_ts) - static_cast<int64_t>(_curr_vib.timestamp_us);
    if (std::abs(skew) > static_cast<int64_t>(_max_allowed_skew_us)) {
        return false; // 정합성 붕괴로 판정하여 안전하게 프레임 드롭
    }

    constexpr size_t VIB_FEAT_CNT = T2_Def::AI::Tensor::MFCC_COEFFS_DEF * T2_Def::AI::Tensor::MFCC_COMPONENTS_DEF * (T2_Def::Accel::Sensor::AXIS_MAX + T2_Def::Gyro::Sensor::AXIS_MAX);

    // 지연 추론(해석 A) 전제: 오디오 프레임이 버퍼링된 상태로 진동 스냅샷 시간축 사이를 순회
    if (audio_ts >= _prev_vib.timestamp_us && audio_ts <= _curr_vib.timestamp_us) {
        uint64_t total_delta = _curr_vib.timestamp_us - _prev_vib.timestamp_us;
        if (total_delta == 0) {
            std::copy(_curr_vib.features, _curr_vib.features + VIB_FEAT_CNT, out_feats);
            return true;
        }
        float alpha = static_cast<float>(audio_ts - _prev_vib.timestamp_us) / static_cast<float>(total_delta);

        for (size_t i = 0; i < VIB_FEAT_CNT; i++) {
            // NaN 오염 방지 가드가 선행 완료된 데이터를 보간
            out_feats[i] = _prev_vib.features[i] + alpha * (_curr_vib.features[i] - _prev_vib.features[i]);
        }
    } else {
        // 경계를 벗어난 과도기/예외 구간은 가장 안전하게 최신 스냅샷 값 유지 (ZOH 폴백)
        std::copy(_curr_vib.features, _curr_vib.features + VIB_FEAT_CNT, out_feats);
    }
    return true;
}

// ========================================================================
// DynamicTensorBinder 구현
// ========================================================================
DynamicTensorBinder::DynamicTensorBinder(size_t nn_input_size)
    : _audio_offset(INVALID_OFFSET), _vib_offset(INVALID_OFFSET), 
      _total_active_elements(0), _nn_input_boundary(nn_input_size) {}

void DynamicTensorBinder::updateTensorOffsets(uint16_t active_mask_flags, size_t audio_feat_cnt, size_t vib_feat_cnt) {
    size_t current_ptr = 0;

    // active_mask_flags는 EM_DataPayloadType_t 열거형에 정의된 비트 플래그를 정합 참조합니다.
    // VIB_ONLY (1) = 0x01, AUDIO_ONLY (2) = 0x02, VIB_AUDIO_BOTH (3) = 0x03
    bool has_audio = (active_mask_flags & static_cast<uint16_t>(T2_Type::EM_DataPayloadType_t::AUDIO_ONLY)) != 0;
    bool has_vib   = (active_mask_flags & static_cast<uint16_t>(T2_Type::EM_DataPayloadType_t::VIB_ONLY)) != 0;

    if (has_audio) {
        _audio_offset = current_ptr;
        current_ptr += audio_feat_cnt;
    } else {
        _audio_offset = INVALID_OFFSET;
    }

    if (has_vib) {
        _vib_offset = current_ptr;
        current_ptr += vib_feat_cnt;
    } else {
        _vib_offset = INVALID_OFFSET;
    }

    _total_active_elements = current_ptr;
}

bool DynamicTensorBinder::buildFlattenTensor(float* p_target_tensor, const float* audio_src, const float* vib_src, uint16_t mask) {
    if (_total_active_elements > _nn_input_boundary) {
        return false;
    }

    // 전체 영역 안전 소거 (Fixed Shape 가정을 위해 0.0f 패딩 강제 적용)
    std::fill_n(p_target_tensor, _nn_input_boundary, 0.0f);

    constexpr size_t AUD_FEAT_CNT = T2_Def::AI::Tensor::MFCC_COEFFS_DEF * T2_Def::AI::Tensor::MFCC_COMPONENTS_DEF * T2_Def::Audio::Sensor::CHANNELS_MAX;
    constexpr size_t VIB_FEAT_CNT = T2_Def::AI::Tensor::MFCC_COEFFS_DEF * T2_Def::AI::Tensor::MFCC_COMPONENTS_DEF * (T2_Def::Accel::Sensor::AXIS_MAX + T2_Def::Gyro::Sensor::AXIS_MAX);

    bool has_audio = (mask & static_cast<uint16_t>(T2_Type::EM_DataPayloadType_t::AUDIO_ONLY)) != 0;
    bool has_vib   = (mask & static_cast<uint16_t>(T2_Type::EM_DataPayloadType_t::VIB_ONLY)) != 0;

    // 오디오 복사 (INVALID_OFFSET이 아닌 경우 복사)
    if (has_audio && _audio_offset != INVALID_OFFSET) {
        std::copy(audio_src, audio_src + AUD_FEAT_CNT, p_target_tensor + _audio_offset);
    }

    // 진동 복사
    if (has_vib && _vib_offset != INVALID_OFFSET) {
        std::copy(vib_src, vib_src + VIB_FEAT_CNT, p_target_tensor + _vib_offset);
    }

    return true;
}
