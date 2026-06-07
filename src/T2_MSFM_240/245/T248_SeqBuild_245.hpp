/* ============================================================================
 * File: T248_SeqBuild_245.hpp
 * Summary: TinyML 추론용 시퀀스 텐서 조립기 (v245 개정판)
 * ============================================================================ */

#pragma once

#include "T210_Def_245.hpp"
#include <cstddef>
#include <cstdint>

class CL_T2_SequenceBuilder {
private:
    static constexpr uint16_t MAX_DIM_PADDED = (T2_Def::AI::Tensor::MFCC_DIM_MAX + 3) & ~3;

    float* _dataFlat;        // PSRAM 1D Flat 버퍼 포인터
    uint16_t _frames;        // 현재 설정된 시퀀스 길이
    uint16_t _dim;           // 현재 설정된 특징량 차원 (예: 312)
    uint16_t _strideDim;     // 패딩된 내부 스트라이드 (SIMD 정렬용)
    uint16_t _head;          // 현재 링버퍼 쓰기 헤드 인덱스
    bool     _isFull;        // 링버퍼가 전체 프레임을 채웠는지 여부

public:
    CL_T2_SequenceBuilder();
    ~CL_T2_SequenceBuilder();

    bool init(uint16_t p_sequenceFrames, uint16_t p_featureDim);
    void pushVector(const float* p_vector);
    void reset();
    bool isReady() const { return _isFull; }
    void getSequenceFlat(float* p_outBuffer, size_t p_maxOutSize) const;
};
