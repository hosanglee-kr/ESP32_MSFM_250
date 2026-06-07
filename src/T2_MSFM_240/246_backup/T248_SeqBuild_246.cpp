/* ============================================================================
 * File: T248_SeqBuild_246.cpp
 * Summary: TinyML 시퀀스 텐서 빌더 구현부 (v245 개정판)
 * ============================================================================ */

#include "T248_SeqBuild_246.hpp"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "T245_SEQ";

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
