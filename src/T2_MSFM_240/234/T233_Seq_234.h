/* ============================================================================
 * File: T233_Seq_234.h
 * Summary: TinyML 추론을 위한 2D 텐서(Sequence) 조립 엔진
 * * * * [AI 셀프 회고 및 구현 원칙 - 실수 방지 유형화 목록] * * *
 * * [유형 1: 배열/메모리 경계(Boundary) 맹점]
 * - 실수: 외부 주입 변수(dim, frames)에 대한 하한선(0값) 및 상한선 검증 누락.
 * - 원칙: 외부에서 들어오는 모든 차원 변수는 1 이상, MAX 이하로 엄격히 클램핑(Clamping)할 것.
 * - 실수: 출력 버퍼의 크기를 확인하지 않고 memcpy를 수행하여 힙 파괴 유발.
 * - 원칙: 외부로 데이터를 내보낼 때는 반드시 max_out_size를 인자로 받아 안전 경계를 확인할 것.
 * * [유형 2: 하드웨어(SIMD) 정렬 및 AI 텐서 규격 충돌]
 * - 실수: 내부 SIMD 안전을 위해 배열을 패딩했으나, AI 모델 추출 시에도 패딩된 채로 보내 텐서 Shape을 파괴함.
 * - 원칙: 2D 배열 선언 시 열(Column)은 패딩 상수(MAX_DIM_PADDED)를 사용하되, getSequenceFlat 추출 시에는 AI 모델이 요구하는 순수 차원(_dim)으로 조밀하게(Tightly packed) 직렬화할 것.
 * * [유형 3: 링 버퍼 논리 및 상태 오염]
 * - 실수: 부분 플러시(!_is_full) 상태일 때 읽기 시작점(_head)과 유효 프레임 수를 잘못 계산함.
 * - 원칙: 버퍼가 다 차지 않았을 때의 시작점은 무조건 0이며, 추출 길이는 _head까지만 제한할 것.
 * * [유형 4: 불필요한 연산 낭비 (RTOS 실시간성)]
 * - 실수: 초기화 시마다 거대한 배열을 memset하여 CPU 스파이크 발생 및 루프 내 % 연산 남용.
 * - 원칙: 데이터 덮어쓰기 논리가 완벽하다면 대규모 memset은 삭제하여 O(1) 초기화를 달성하고, 링 버퍼 순회는 단순 덧셈과 분기문(-= _frames)으로 최적화할 것.
 * ========================================================================== */
#pragma once

#include "T210_Def_234.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>

class CL_T20_SequenceBuilder {
public:
    CL_T20_SequenceBuilder();
    ~CL_T20_SequenceBuilder() = default;

    void begin(uint16_t sequence_frames, uint16_t feature_dim);
    void pushVector(const float* p_vector);
    void reset();

    bool isReady() const { return _is_full; }

    // [보안/안정성] OOM 및 힙 파괴를 막기 위해 max_out_size 인자 추가 적용
    void getSequenceFlat(float* p_out_buffer, size_t max_out_size) const;

    uint16_t getSequenceFrames() const { return _frames; }
    uint16_t getFeatureDim() const { return _dim; }

private:
    // [SIMD 방어] MAX_FEATURE_DIM 자체가 16의 배수가 아닐 경우 발생하는 
    // 행(Row) 침범 패닉을 막기 위해 패딩된 컴파일 타임 상수 적용
    static constexpr uint16_t MAX_DIM_PADDED = (T20::C10_DSP::MAX_FEATURE_DIM + 3) & ~3;
    
    alignas(16) float _data[T20::C10_Sys::SEQUENCE_FRAMES_MAX][MAX_DIM_PADDED];

    uint16_t _frames;
    uint16_t _dim;
    uint16_t _stride_dim; // 연산 낭비를 막기 위한 패딩 차원 캐싱
    uint16_t _head;
    bool     _is_full;
};
