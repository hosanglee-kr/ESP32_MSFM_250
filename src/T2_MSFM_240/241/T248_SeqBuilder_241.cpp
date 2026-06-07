/* ============================================================================
 * File: T248_SeqBuilder_241.cpp
 * Summary: TinyML 시퀀스 텐서 빌더 구현부 (Zero-Defect Version)
 * ============================================================================
 * [로직 상태 추적 (Logic State Tracker)]
 * - 유지: 링버퍼 기반 FIFO 출력 및 데이터 무결성 보존.
 * - 갱신: [교정] pushVector 시점에 해당 슬롯의 패딩 영역만 선별적 초기화(Rule #2).
 * - 신규: [원칙 준수] 모든 로컬 변수 v_ 접두어 적용 및 heap_caps_aligned_alloc 사용.
 * ========================================================================== */

#include "T248_SeqBuilder_241.hpp"
#include "esp_heap_caps.h" // [교정] 필수 헤더 추가
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
    // [Rule #5] 정적 상수를 이용한 범위 제약 (OOM 방어)
    _frames = (p_sequenceFrames > T2_Config::System::SEQUENCE_FRAMES_MAX_CONST) ?
               T2_Config::System::SEQUENCE_FRAMES_MAX_CONST : p_sequenceFrames;

    _dim = (p_featureDim > T2_Config::System::MFCC_DIM_CONST) ?
            T2_Config::System::MFCC_DIM_CONST : p_featureDim;

    _strideDim = MAX_DIM_PADDED; // 39 -> 40 (SIMD 16B 정렬용)

    if (!_dataFlat) {
        // [Rule #21, #23] 16바이트 정렬된 PSRAM 힙 할당
        size_t v_allocSize = _frames * _strideDim * sizeof(float);
        _dataFlat = (float*)heap_caps_aligned_alloc(16, v_allocSize, MALLOC_CAP_SPIRAM);

        if (!_dataFlat) {
            ESP_LOGE(TAG, "PSRAM Allocation Failed! Req: %zu bytes", v_allocSize);
            return false;
        }
        // 최초 1회 전체 초기화 (NaN 독성 방지)
        memset(_dataFlat, 0, v_allocSize);
    }

    reset();
    ESP_LOGI(TAG, "SeqBuilder Init: %d x %d (Stride: %d)", _frames, _dim, _strideDim);
    return true;
}

void CL_T2_SequenceBuilder::pushVector(const float* p_vector) {
    if (!p_vector || !_dataFlat || _frames == 0) return;

    // 현재 쓰기 헤드 포인터 계산
    float* v_targetSlot = &_dataFlat[_head * _strideDim];

    // 1. 핵심 데이터 복사 (39D)
    memcpy(v_targetSlot, p_vector, sizeof(float) * _dim);

    // 2. [Rule #2] 런타임 전체 memset 금지 준수
    // 현재 슬롯의 패딩 영역(40번째 차원)만 선택적으로 0.0f 초기화하여 SIMD 오염 방어
    if (_strideDim > _dim) {
        v_targetSlot[_dim] = 0.0f;
    }

    // 3. 헤드 전진 및 충만 상태 갱신
    _head++;
    if (_head >= _frames) {
        _head = 0;
        _isFull = true;
    }
}

void CL_T2_SequenceBuilder::getSequenceFlat(float* p_outBuffer, size_t p_maxOutSize) const {
    // [Rule #23] 하드웨어 가속을 위한 출력 버퍼 정렬 검사
    if (!p_outBuffer || !_dataFlat) return;

    // 출력 버퍼가 16바이트 정렬되지 않았을 경우 경고 출력 (S3 SIMD 최적화 저해 요소)
    if (((uintptr_t)p_outBuffer & 15) != 0) {
        ESP_LOGW(TAG, "Performance Warning: Output buffer is not 16-byte aligned!");
    }

    size_t v_validCount = _isFull ? _frames : _head;
    if (v_validCount == 0) return;

    // 모델 요구 Packed 데이터 크기 (예: 16프레임 x 39차원)
    size_t v_totalPackedSize = _frames * _dim * sizeof(float);
    if (p_maxOutSize < v_totalPackedSize) {
        ESP_LOGE(TAG, "Output buffer too small! Req: %zu", v_totalPackedSize);
        return;
    }

    // 시계열 순서 계산: 가장 오래된 데이터의 위치
    uint16_t v_oldestIdx = _isFull ? _head : 0;
    size_t v_currentOffset = 0;

    // [Temporal Order Reconstruction] 링버퍼 해제 및 Packed 복사
    for (uint16_t i = 0; i < v_validCount; i++) {
        uint16_t v_readIdx = (v_oldestIdx + i) % _frames;

        // 내부 Padded 데이터(40D)에서 순수 요구 차원(39D)만 추출하여 밀착 복사
        memcpy(p_outBuffer + v_currentOffset, &_dataFlat[v_readIdx * _strideDim], sizeof(float) * _dim);
        v_currentOffset += _dim;
    }

    // [Omission Defense] 시퀀스가 아직 덜 찼다면 앞부분 또는 뒷부분을 0.0f로 제로패딩
    if (v_validCount < _frames) {
        size_t v_paddingElements = (_frames - v_validCount) * _dim;
        memset(p_outBuffer + v_currentOffset, 0, v_paddingElements * sizeof(float));
    }
}

void CL_T2_SequenceBuilder::reset() {
    // [Rule #2] $O(1)$ 리셋: 포인터와 플래그만 초기화하여 CPU Spike 방지
    _head = 0;
    _isFull = false;
}
