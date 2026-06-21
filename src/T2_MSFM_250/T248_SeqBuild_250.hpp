/* ============================================================================
 * File: T248_SeqBuild_250.hpp
 * Summary: TinyML 추론용 시퀀스 텐서 조립기
 * ============================================================================ */

#pragma once

#include "T210_Def_250.hpp"
#include "T215_Type_250.hpp"
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

// ------------------------------------------------------------------------
// [신규] MultiRateTimeAligner 및 DynamicTensorBinder 선언 (Tier 3)
// ------------------------------------------------------------------------

typedef struct {
    float features[T2_Def::AI::Tensor::MFCC_COEFFS_DEF * T2_Def::AI::Tensor::MFCC_COMPONENTS_DEF * (T2_Def::Accel::Sensor::AXIS_MAX + T2_Def::Gyro::Sensor::AXIS_MAX)];
    uint64_t timestamp_us;
    bool is_valid;
} ST_Vib_Snapshot_t;

class MultiRateTimeAligner {
private:
    ST_Vib_Snapshot_t _prev_vib;
    ST_Vib_Snapshot_t _curr_vib;
    uint64_t _max_allowed_skew_us;

public:
    MultiRateTimeAligner(uint64_t p_maxSkewUs);
    void injectNewVibSample(const float* p_rawFeats, uint64_t p_ts);
    bool getAlignedVibration(uint64_t p_audioTs, float* p_outFeats);
};

class DynamicTensorBinder {
private:
    static constexpr size_t INVALID_OFFSET = static_cast<size_t>(-1);
    size_t _audio_offset;
    size_t _vib_offset;
    size_t _total_active_elements;
    size_t _nn_input_boundary;

public:
    DynamicTensorBinder(size_t p_nnInputSize);
    void updateTensorOffsets(uint16_t p_activeMaskFlags, size_t p_audioFeatCnt, size_t p_vibFeatCnt);
    bool buildFlattenTensor(float* p_targetTensor, const float* p_audioSrc, const float* p_vibSrc, uint16_t p_mask);
};

