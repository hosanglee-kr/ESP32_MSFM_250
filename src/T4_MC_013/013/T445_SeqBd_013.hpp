/* ============================================================================
 * [SMEA-100 핵심 구현 원칙 및 AI 셀프 회고 바이블]
 * 1. SIMD 메모리 정렬을 위해 내부는 MAX_DIM_PADDED로 저장하되,
 * 모델 추론(추출) 시에는 순수 요구 차원(_dim)으로 Tightly Packed 직렬화 수행.
 * 2. 런타임 초기화 병목(CPU Spike) 제거를 위한 memset 없는 O(1) 리셋 적용.
 * 3. [네이밍 컨벤션 엄수]: private(_), 매개변수(p_), 로컬변수(v_)
 * 4. [방어/교정] Internal SRAM 고갈 방어: 20KB에 달하는 2D 시퀀스 텐서 배열을 
 * 정적으로 선언하지 않고, init() 시점에 MALLOC_CAP_SPIRAM으로 PSRAM에 1D로 동적 할당한다.
 * 5. [동적 할당 방어]: 텐서 버퍼는 런타임 설정(_DEF)을 배제하고 오직
 * 최대 정적 상수(_CONST)만 사용하여 OOM을 원천 차단한다.
 * ============================================================================
 * File: T445_SeqBd_013.hpp
 * Summary: TinyML Inference Sequence Tensor Builder (PSRAM Optimized)
 * ========================================================================== */
#pragma once

#include "T410_Def_013.hpp"
#include <cstddef>
#include <cstring>
#include <cstdint>

class T445_SequenceBuilder {
private:
    static constexpr uint16_t MAX_DIM_PADDED = (SmeaConfig::System::MFCC_TOTAL_DIM_CONST + 3) & ~3;

    float* _dataFlat;

    uint16_t _frames;       
    uint16_t _dim;          
    uint16_t _strideDim;    
    uint16_t _head;         
    bool     _isFull;       

public:
    T445_SequenceBuilder();
    ~T445_SequenceBuilder();

    bool init(uint16_t p_sequenceFrames, uint16_t p_featureDim);
    void pushVector(const float* p_vector);
    void reset();

    bool isReady() const { return _isFull; }
    void getSequenceFlat(float* p_outBuffer, size_t p_maxOutSize) const;
};

