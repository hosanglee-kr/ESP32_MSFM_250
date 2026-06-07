/* ============================================================================
 * File: T248_SeqBuild_243.cpp
 * Summary: TinyML 시퀀스 텐서 빌더 구현부 (v243)
 * ========================================================================== */
#include "T248_SeqBuild_243.hpp"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "T243_SEQ";

CL_T2_SequenceBuilder::CL_T2_SequenceBuilder()
    : _dataFlat(nullptr), _frames(0), _dim(0), _strideDim(0), _head(0), _isFull(false) {}

CL_T2_SequenceBuilder::~CL_T2_SequenceBuilder() {
    if (_dataFlat) {
        heap_caps_free(_dataFlat);
        _dataFlat = nullptr;
    }
}

bool CL_T2_SequenceBuilder::init(uint16_t p_sequenceFrames, uint16_t p_featureDim) {
    // [SSOT 연동] 정적 상한선(MAX) 내에서 유효성 검사
    _frames = (p_sequenceFrames > T2_Def::Global::System::SEQUENCE_FRAMES_MAX) ?
               T2_Def::Global::System::SEQUENCE_FRAMES_MAX : p_sequenceFrames;

    _dim = (p_featureDim > T2_Def::Audio::FeatureLimit::MFCC_DIM_MAX) ?
            T2_Def::Audio::FeatureLimit::MFCC_DIM_MAX : p_featureDim;

    // SIMD(16B) 가속을 위해 4-float 단위로 스트라이드 정렬
    _strideDim = (p_featureDim + 3) & ~3; 

    if (!_dataFlat) {
        // [Rule #21] 16바이트 정렬된 PSRAM 힙 할당 (가변 길이에 대응)
        size_t v_allocSize = _frames * _strideDim * sizeof(float);
        _dataFlat = (float*)heap_caps_aligned_alloc(16, v_allocSize, MALLOC_CAP_SPIRAM);

        if (!_dataFlat) {
            ESP_LOGE(TAG, "PSRAM Allocation Failed! Req: %zu bytes", v_allocSize);
            return false;
        }
        memset(_dataFlat, 0, v_allocSize);
    }

    reset();
    ESP_LOGI(TAG, "SeqBuilder v243 Init: %d x %d (Stride: %d)", _frames, _dim, _strideDim);
    return true;
}

void CL_T2_SequenceBuilder::pushVector(const float* p_vector) {
    if (!p_vector || !_dataFlat || _frames == 0) return;

    // 현재 링버퍼 쓰기 헤드 위치 계산
    float* v_targetSlot = &_dataFlat[_head * _strideDim];

    // 1. 유효 데이터 복사
    memcpy(v_targetSlot, p_vector, sizeof(float) * _dim);

    // 2. 패딩 영역 초기화 (SIMD 독성 방지)
    if (_strideDim > _dim) {
        for (uint16_t j = _dim; j < _strideDim; j++) {
            v_targetSlot[j] = 0.0f;
        }
    }

    // 3. 인덱스 순환 처리
    _head++;
    if (_head >= _frames) {
        _head = 0;
        _isFull = true;
    }
}

void CL_T2_SequenceBuilder::reset() {
    // O(1) 초기화: 메모리 삭제 없이 포인터만 리셋하여 실시간성 유지
    _head = 0;
    _isFull = false;
}

void CL_T2_SequenceBuilder::getSequenceFlat(float* p_outBuffer, size_t p_maxOutSize) const {
    if (!p_outBuffer || !_dataFlat) return;

    // 출력 버퍼 정렬 상태 확인 (S3 SIMD 가속 최적화 여부)
    if (((uintptr_t)p_outBuffer & 15) != 0) {
        ESP_LOGW(TAG, "Performance Warning: Output buffer alignment check failed.");
    }

    size_t v_validCount = _isFull ? _frames : _head;
    if (v_validCount == 0) return;

    // Packed(밀착) 데이터 크기 계산 (패딩 제외)
    size_t v_totalPackedSize = _frames * _dim * sizeof(float);
    if (p_maxOutSize < v_totalPackedSize) {
        ESP_LOGE(TAG, "Output buffer too small! Req: %zu", v_totalPackedSize);
        return;
    }

    // 시계열 순서 재조합: 가장 오래된 데이터부터 차례대로 추출
    uint16_t v_oldestIdx = _isFull ? _head : 0;
    size_t v_currentOffset = 0;

    for (uint16_t i = 0; i < v_validCount; i++) {
        uint16_t v_readIdx = (v_oldestIdx + i) % _frames;
        
        // Padded 저장 구조에서 순수 데이터만 추출하여 출력 버퍼에 Packed 복사
        memcpy(p_outBuffer + v_currentOffset, &_dataFlat[v_readIdx * _strideDim], sizeof(float) * _dim);
        v_currentOffset += _dim;
    }

    // [Omission Defense] 아직 채워지지 않은 프레임은 Zero-padding 처리
    if (v_validCount < _frames) {
        size_t v_paddingElements = (_frames - v_validCount) * _dim;
        memset(p_outBuffer + v_currentOffset, 0, v_paddingElements * sizeof(float));
    }
}
