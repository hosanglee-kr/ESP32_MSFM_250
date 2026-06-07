/* ============================================================================
 * File: T248_SeqBuilder_241.hpp
 * Summary: TinyML 추론용 시퀀스 텐서 조립기 (PSRAM & SIMD 최적화)
 * ============================================================================
 * [로직 상태 추적 (Logic State Tracker)]
 * - 유지: 1D Flat 배열 기반 링버퍼 포인터 연산 및 temporal 정렬 로직.
 * - 갱신: [교정] 39D MFCC(156B)와 16바이트 정렬(160B) 간의 스트라이드 연산 정밀화.
 * - 신규: [원칙 준수] 런타임 memset을 배제한 O(1) 상태 리셋 및 추출 시 Zero-padding.
 *
 * [축소/누락 방어 체크리스트 (Omission Defense)]
 * 1. [SRAM 보호]: 20KB 이상의 텐서는 반드시 MALLOC_CAP_SPIRAM에 할당 (Rule #21).
 * 2. [SIMD 정렬]: 내부 저장 스트라이드를 16바이트(4-float) 배수로 자동 패딩 (Rule #23).
 * 3. [NaN 독성 차단]: pushVector 시점에 패딩 영역(40번째 차원) 강제 0.0f 초기화.
 * 4. [포인터 방어]: getSequenceFlat 호출 시 출력 버퍼의 16바이트 정렬 여부 검사.
 * ========================================================================== */
#pragma once

#include "T210_Def_241.hpp"
#include <cstddef>
#include <cstdint>

class CL_T2_SequenceBuilder {
private:
    // SIMD 가속을 위해 4의 배수(16바이트)로 정렬된 내부 차원 패딩 (39D -> 40D)
    static constexpr uint16_t MAX_DIM_PADDED = (T2_Config::System::MFCC_DIM_CONST + 3) & ~3;

    float* _dataFlat;        // PSRAM 1D Flat 버퍼 포인터
    uint16_t _frames;        // 요구 시퀀스 길이 (Static Constant 기반)
    uint16_t _dim;           // 순수 특징량 차원 (예: 39)
    uint16_t _strideDim;     // 패딩된 내부 차원 (예: 40)
    uint16_t _head;          // 현재 쓰기 헤드 위치
    bool     _isFull;        // 링버퍼 충만 여부

public:
    CL_T2_SequenceBuilder();
    ~CL_T2_SequenceBuilder();

    // 1. 초기화 및 PSRAM 메모리 할당 (Rule #21)
    bool init(uint16_t p_sequenceFrames, uint16_t p_featureDim);

    // 2. 신규 특징량 벡터 추가 ($O(1)$ 연산, SIMD 패딩 보호 포함)
    void pushVector(const float* p_vector);

    // 3. 내부 상태값 리셋 ($O(1)$, 런타임 지연 0ms 준수)
    void reset();

    // 4. 추론 엔진 전용 Packed 데이터 추출 (Oldest -> Newest 정렬)
    // p_outBuffer: 결과를 담을 외부 버퍼 (반드시 16바이트 정렬 권장)
    void getSequenceFlat(float* p_outBuffer, size_t p_maxOutSize) const;

    bool isReady() const { return _isFull; }
};
