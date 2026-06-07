/* ============================================================================
 * File: T248_SeqBuild_245.cpp
 * Summary: TinyML 추론용 시퀀스 텐서 조립기 구현부
 * ============================================================================ */

#include "T248_SeqBuild_245.hpp"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "T245_SEQ";

/**
 * @brief CL_T2_SequenceBuilder 생성자
 */
CL_T2_SequenceBuilder::CL_T2_SequenceBuilder()
    : _dataFlat(nullptr), _frames(0), _dim(0), _strideDim(0), _head(0), _isFull(false) {}

/**
 * @brief CL_T2_SequenceBuilder 소멸자 (PSRAM 메모리 해제)
 */
CL_T2_SequenceBuilder::~CL_T2_SequenceBuilder() {
    if (_dataFlat) {
        heap_caps_free(_dataFlat);
        _dataFlat = nullptr;
    }
}

/**
 * @brief 시퀀스 빌더 파라미터 초기 설정 및 메모리 할당
 * @param p_sequenceFrames 시퀀스에 보관될 최대 시간축 프레임 수
 * @param p_featureDim 1프레임당 특징량 차원 크기
 * @return 초기화 완료 성공 여부
 */
bool CL_T2_SequenceBuilder::init(uint16_t p_sequenceFrames, uint16_t p_featureDim) {
    // 프레임 범위 제한 검사
    _frames = (p_sequenceFrames > T2_Def::Global::System::SEQUENCE_FRAMES_MAX) ?
               T2_Def::Global::System::SEQUENCE_FRAMES_MAX : p_sequenceFrames;

    _dim = (p_featureDim > T2_Def::AI::Tensor::MFCC_DIM_MAX) ?
            T2_Def::AI::Tensor::MFCC_DIM_MAX : p_featureDim;

    // SIMD 정렬(16바이트) 보장을 위해 4배수로 스트라이드 올림
    _strideDim = (p_featureDim + 3) & ~3;

    // [처리 단위 1] PSRAM 메모리에 플랫 시퀀스 텐서용 공간 할당
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

/**
 * @brief 신규 특징 프레임(벡터)을 링 버퍼 헤드 위치에 복사 삽입
 * @param p_vector 추가할 1개 프레임 분량의 float 특징 벡터 포인터
 */
void CL_T2_SequenceBuilder::pushVector(const float* p_vector) {
    if (!p_vector || !_dataFlat || _frames == 0) return;

    // [처리 단위 1] 현재 링 헤드 버퍼 위치에 복사 기입 및 패딩 소거
    float* v_targetSlot = &_dataFlat[_head * _strideDim];
    memcpy(v_targetSlot, p_vector, sizeof(float) * _dim);

    if (_strideDim > _dim) {
        for (uint16_t j = _dim; j < _strideDim; j++) {
            v_targetSlot[j] = 0.0f;
        }
    }

    // [처리 단위 2] 링 버퍼 헤드 인덱스 전진 및 가득 참 감시
    _head++;
    if (_head >= _frames) {
        _head = 0;
        _isFull = true;
    }
}

/**
 * @brief 인덱스 지점 초기화
 */
void CL_T2_SequenceBuilder::reset() {
    _head = 0;
    _isFull = false;
}

/**
 * @brief 링 버퍼에 적재된 시퀀스를 정렬 상태로 1D 플랫 버퍼에 인출
 * @param p_outBuffer 출력을 수신할 버퍼 포인터
 * @param p_maxOutSize 버퍼 최대 바이트 용량
 */
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

    // 가장 오래된 프레임 위치 설정 (Full 상태면 _head가 가장 먼저 기입된 곳)
    uint16_t v_oldestIdx = _isFull ? _head : 0;
    size_t v_currentOffset = 0;

    // [처리 단위 1] 링 버퍼를 순회하여 시간축 정렬 복사 수행
    for (uint16_t i = 0; i < v_validCount; i++) {
        uint16_t v_readIdx = (v_oldestIdx + i) % _frames;
        memcpy(p_outBuffer + v_currentOffset, &_dataFlat[v_readIdx * _strideDim], sizeof(float) * _dim);
        v_currentOffset += _dim;
    }

    // [처리 단위 2] 링 버퍼가 덜 채워진 상태인 경우, 나머지 공간에 대해 제로 패딩 삽입
    if (v_validCount < _frames) {
        size_t v_paddingElements = (_frames - v_validCount) * _dim;
        memset(p_outBuffer + v_currentOffset, 0, v_paddingElements * sizeof(float));
    }
}
