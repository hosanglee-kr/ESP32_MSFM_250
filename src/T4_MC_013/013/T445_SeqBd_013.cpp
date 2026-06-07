/* ============================================================================
 * File: T445_SeqBd_013.cpp
 * Summary: Sequence Tensor Builder Implementation (PSRAM & SIMD Optimized)
 * ============================================================================
 * * [AI 메모: 마이그레이션 적용 완료 사항]
 * - 2D 정적 배열을 PSRAM 1D Flat 배열로 포인터 연산 전환 완료 (SRAM 확보).
 * - SIMD 처리용 패딩 영역(Garbage 값)에 대한 NaN 독성 차단(memset 0.0f) 유지.
 * - TFLite 등 직렬화 추출 시 Zero-padding (시퀀스 미달 시 0으로 채움) 로직 유지.
 * - [v012 유지보수] v012 헤더 버전 맵핑 유지.
 * ========================================================================== */

#include "T445_SeqBd_013.hpp"
#include "esp_log.h"

static const char* TAG = "T445_SEQ";

T445_SequenceBuilder::T445_SequenceBuilder() {
    _dataFlat = nullptr;
    _frames = SmeaConfig::MlLimit::MAX_SEQUENCE_FRAMES_CONST;
    _dim = SmeaConfig::System::MFCC_TOTAL_DIM_CONST;
    _strideDim = (_dim + 3) & ~3;
    _head = 0;
    _isFull = false;
}

T445_SequenceBuilder::~T445_SequenceBuilder() {
    if (_dataFlat) {
        heap_caps_free(_dataFlat);
        _dataFlat = nullptr;
    }
}

bool T445_SequenceBuilder::init(uint16_t p_sequenceFrames, uint16_t p_featureDim) {
    _frames = p_sequenceFrames;
    if (_frames < 1) _frames = 1;
    if (_frames > SmeaConfig::MlLimit::MAX_SEQUENCE_FRAMES_CONST) _frames = SmeaConfig::MlLimit::MAX_SEQUENCE_FRAMES_CONST;

    _dim = p_featureDim;
    if (_dim < 1) _dim = 1;
    if (_dim > SmeaConfig::System::MFCC_TOTAL_DIM_CONST) _dim = SmeaConfig::System::MFCC_TOTAL_DIM_CONST;

    _strideDim = (_dim + 3) & ~3;

    if (!_dataFlat) {
        size_t v_allocSize = SmeaConfig::MlLimit::MAX_SEQUENCE_FRAMES_CONST * MAX_DIM_PADDED * sizeof(float);
        _dataFlat = (float*)heap_caps_aligned_alloc(16, v_allocSize, MALLOC_CAP_SPIRAM);
        
        if (!_dataFlat) {
            ESP_LOGE(TAG, "[CRITICAL] PSRAM Allocation Failed for SeqBuilder! (Req: %zu Bytes)", v_allocSize);
            return false;
        }
        memset(_dataFlat, 0, v_allocSize);
    }

    _head = 0;
    _isFull = false;
    return true;
}

void T445_SequenceBuilder::pushVector(const float* p_vector) {
    if (!p_vector || !_dataFlat || _frames == 0) return;

    float* v_targetPtr = &_dataFlat[_head * MAX_DIM_PADDED];

    memcpy(v_targetPtr, p_vector, sizeof(float) * _dim);

    if (_strideDim > _dim) {
        memset(v_targetPtr + _dim, 0, (_strideDim - _dim) * sizeof(float));
    }

    _head++;
    if (_head >= _frames) {
        _head = 0;
        _isFull = true;
    }
}

void T445_SequenceBuilder::getSequenceFlat(float* p_outBuffer, size_t p_maxOutSize) const {
    if (!p_outBuffer || !_dataFlat || ((uintptr_t)p_outBuffer & 15) != 0) return;

    size_t v_validFrames = _isFull ? _frames : _head;
    if (v_validFrames == 0) return;

    size_t v_requiredBytes = v_validFrames * _dim * sizeof(float);

    if (p_maxOutSize < v_requiredBytes) return;

    uint16_t v_oldestIdx = _isFull ? _head : 0;
    size_t v_offset = 0;

    for (uint16_t i = 0; i < v_validFrames; i++) {
        uint16_t v_currentIdx = v_oldestIdx + i;
        if (v_currentIdx >= _frames) v_currentIdx -= _frames;

        memcpy(p_outBuffer + v_offset, &_dataFlat[v_currentIdx * MAX_DIM_PADDED], sizeof(float) * _dim);
        v_offset += _dim;
    }

    size_t v_totalTensorBytes = _frames * _dim * sizeof(float);
    if (v_validFrames < _frames && p_maxOutSize >= v_totalTensorBytes) {
        memset(p_outBuffer + v_offset, 0, v_totalTensorBytes - v_requiredBytes);
    }
}

void T445_SequenceBuilder::reset() {
    _head = 0;
    _isFull = false;
}
