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

	// TODO : 필요 여부 검토 필요
    // static constexpr uint16_t MAX_DIM_PADDED = (T2_Def::AI::Tensor::MFCC_DIM_MAX + 3) & ~3; // 4의 배수로 패딩

    float*   _dataFlat;        // PSRAM 1D Flat 버퍼 포인터
    uint16_t _frames;          // 현재 설정된 시퀀스 길이
    uint16_t _dim;             // 현재 설정된 특징량 차원 (예: 312)
    uint16_t _strideDim;       // 패딩된 내부 스트라이드 (SIMD 정렬용)
    uint16_t _head;            // 현재 링버퍼 쓰기 헤드 인덱스
    bool     _isFull;          // 링버퍼가 전체 프레임을 채웠는지 여부

public:
    CL_T2_SequenceBuilder();
    ~CL_T2_SequenceBuilder();

	// 시퀀스 초기화
    bool init(uint16_t p_sequenceFrames, uint16_t p_featureDim);
	// 벡터 푸시
    void pushVector(const float* p_vector);
	// 리셋
    void reset();
	// 준비 여부 확인
    bool isReady() const { return _isFull; }
	// 플랫 시퀀스 가져오기
    void getSequenceFlat(float* p_outBuffer, size_t p_maxOutSize) const;
};

// ------------------------------------------------------------------------
// MultiRateTimeAligner 및 DynamicTensorBinder 선언 (Tier 3)
// ------------------------------------------------------------------------

// 진동 특징량 스냅샷 구조체
typedef struct {
    float    features[T2_Def::AI::Tensor::MFCC_COEFFS_DEF * T2_Def::AI::Tensor::MFCC_COMPONENTS_DEF * (T2_Def::Accel::Sensor::AXIS_MAX + T2_Def::Gyro::Sensor::AXIS_MAX)]; // 가속도/자이로 특징량
    uint64_t timestamp_us;	  // MicroSecond
    bool     is_valid;		  // 유효 여부
} ST_Vib_Snapshot_t;

// 다중 샘플 동기화 클래스
class MultiRateTimeAligner {
private:
    ST_Vib_Snapshot_t _prev_vib;            // 이전 스냅샷
    ST_Vib_Snapshot_t _curr_vib;            // 현재 스냅샷
    uint64_t          _max_allowed_skew_us; // 허용 오차

public:
    // 생성자
    MultiRateTimeAligner(uint64_t p_maxSkewUs);
	// 특징량 주입
    void injectNewVibSample(const float* p_rawFeats, uint64_t p_ts);
	// 특징량 동기화
    bool getAlignedVibration(uint64_t p_audioTs, float* p_outFeats);
};

// 동적 텐서 바인더 클래스
class DynamicTensorBinder {
private:
    static constexpr size_t INVALID_OFFSET = static_cast<size_t>(-1); // 유효하지 않은 오프셋
    size_t                 _audio_offset;                             // 오디오 오프셋
    size_t                 _vib_offset;                               // 진동 오프셋
    size_t                 _total_active_elements;                    // 활성 요소 수
    size_t                 _nn_input_boundary;                        // NN 입력 경계

public:
    // 생성자
    DynamicTensorBinder(size_t p_nnInputSize);
	// 텐서 오프셋 업데이트
    void updateTensorOffsets(uint16_t p_activeMaskFlags, size_t p_audioFeatCnt, size_t p_vibFeatCnt);
	// 플랫 텐서 빌드
    bool buildFlattenTensor(float* p_targetTensor, const float* p_audioSrc, const float* p_vibSrc, uint16_t p_mask);
};

