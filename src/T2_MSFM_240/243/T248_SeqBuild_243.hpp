/* ============================================================================
 * File: T248_SeqBuild_243.hpp
 * Summary: TinyML 추론용 시퀀스 텐서 조립기 (v243)
 * ============================================================================
 * [로직 상태 추적 (Logic State Tracker)]
 * - 유지: 링버퍼 기반 temporal 정렬 및 Packed 데이터 추출 로직.
 * - 갱신: v243 4-Tier 계층 구조 및 SSOT 상수(MFCC_DIM_MAX 등) 연동.
 * - 신규: [원칙 준수] alignas(16) 연산 호환성 보장 및 PSRAM 명시적 할당 (Rule #21).
 * ========================================================================== */
#pragma once

#include "T210_Def_243_9.hpp"
#include <cstddef>
#include <cstdint>

class CL_T2_SequenceBuilder {
private:
    // SIMD 가속을 위해 4의 배수(16바이트)로 정렬된 내부 차원 패딩 (최대 차원 기준 정적 계산)
    static constexpr uint16_t MAX_DIM_PADDED = (T2_Def::Audio::FeatureLimit::MFCC_DIM_MAX + 3) & ~3;

    float* _dataFlat;        // PSRAM 1D Flat 버퍼 포인터
    uint16_t _frames;        // 현재 설정된 시퀀스 길이
    uint16_t _dim;           // 현재 설정된 특징량 차원 (예: 39)
    uint16_t _strideDim;     // 패딩된 내부 스트라이드 (SIMD 정렬용)
    uint16_t _head;          // 현재 링버퍼 쓰기 헤드 인덱스
    bool     _isFull;        // 링버퍼가 전체 프레임을 채웠는지 여부

public:
    CL_T2_SequenceBuilder();
    ~CL_T2_SequenceBuilder();

    /**
     * @brief 시퀀스 빌더 초기화 및 PSRAM 버퍼 할당
     * @param p_sequenceFrames 타겟 시퀀스 길이 (예: 16)
     * @param p_featureDim 단일 벡터 차원 (예: 39)
     * @return 성공 여부
     */
    bool init(uint16_t p_sequenceFrames, uint16_t p_featureDim);

    /**
     * @brief 신규 특징량 벡터를 링버퍼에 추가
     * @param p_vector 추가할 특징량 배열 포인터
     */
    void pushVector(const float* p_vector);

    /**
     * @brief 내부 상태(헤드, 플래그) 리셋 (O(1) 연산)
     */
    void reset();

    /**
     * @brief 시퀀스 데이터가 가득 찼는지 여부 확인
     */
    bool isReady() const { return _isFull; }

    /**
     * @brief 추론 엔진 전용 Packed 데이터 추출 (Oldest -> Newest 시계열 정렬)
     * @param p_outBuffer 결과를 담을 평탄화된 출력 버퍼
     * @param p_maxOutSize 출력 버퍼의 최대 크기 (바이트 단위)
     */
    void getSequenceFlat(float* p_outBuffer, size_t p_maxOutSize) const;
};
