/* ============================================================================
 * File: T233_Seq_234.cpp
 * Summary: Sequence Builder Engine Implementation (Extreme Stress Handled)
 * ========================================================================== */
#include "T233_Seq_234.h"

CL_T20_SequenceBuilder::CL_T20_SequenceBuilder() {
    _frames = T20::C10_Sys::SEQUENCE_FRAMES_MAX;
    _dim = T20::C10_DSP::MFCC_COEFFS_DEF * 3U; 
    _stride_dim = (_dim + 3) & ~3;
    _head = 0;
    _is_full = false;
    // [최적화] 인스턴스 부팅 지연(CPU Spike) 원천 차단을 위해 memset 삭제
}

void CL_T20_SequenceBuilder::begin(uint16_t sequence_frames, uint16_t feature_dim) {
    // [메모리 방어] 외부 주입 변수 0값 및 한계 초과 클램핑
    _frames = sequence_frames;
    if (_frames < 1) _frames = 1;
    if (_frames > T20::C10_Sys::SEQUENCE_FRAMES_MAX) _frames = T20::C10_Sys::SEQUENCE_FRAMES_MAX;

    _dim = feature_dim;
    if (_dim < 1) _dim = 1;
    if (_dim > T20::C10_DSP::MAX_FEATURE_DIM) _dim = T20::C10_DSP::MAX_FEATURE_DIM;

    // [최적화] 16바이트 정렬 스트라이드 캐싱 (매 프레임 연산 낭비 제거)
    _stride_dim = (_dim + 3) & ~3;
    
    _head = 0;
    _is_full = false;
    // [최적화] 런타임 텐서 리셋 지연 방지를 위해 memset 삭제 (O(1) 속도)
}

void CL_T20_SequenceBuilder::pushVector(const float* p_vector) {
    if (!p_vector || _frames == 0) return;

    // [데이터 정합성] 추출 시 Tightly packed로 내보내더라도, 내부는 안전하게 스트라이드 보장
    memcpy(_data[_head], p_vector, sizeof(float) * _dim);
    
    // [오염 방어] SIMD를 위해 확장된 꼬리(Tail) 영역에 쓰레기값이 남지 않도록 명시적 제로 패딩
    if (_stride_dim > _dim) {
        memset(&_data[_head][_dim], 0, (_stride_dim - _dim) * sizeof(float));
    }
    
    _head++;
    if (_head >= _frames) {
        _head = 0;
        _is_full = true; 
    }
}

void CL_T20_SequenceBuilder::getSequenceFlat(float* p_out_buffer, size_t max_out_size) const {
    // [HW 패닉 방어] 포인터 유효성 및 16바이트 정렬 검증
    if (!p_out_buffer || ((uintptr_t)p_out_buffer & 15) != 0) return;

    // [데이터 무결성] 부분 플러시(가동 중단 등) 시 유효 프레임 수치만 산정
    size_t valid_frames = _is_full ? _frames : _head;
    if (valid_frames == 0) return;

    size_t required_bytes = valid_frames * _dim * sizeof(float);
    
    // [힙 파괴 방어] 외부 버퍼의 한계 크기 엄격 교차 검증
    if (max_out_size < required_bytes) return;

    // [논리 결함 복구] 덜 찬 버퍼에서 낡은 데이터를 읽어오는 AI 환각 방지
    uint16_t oldest_idx = _is_full ? _head : 0; 
    size_t offset = 0; // [오버플로우 방어] uint16_t -> size_t 승급

    for (uint16_t i = 0; i < valid_frames; i++) {
        uint16_t current_idx = oldest_idx + i;
        
        // [최적화] 고비용 Modulo(%) 연산을 분기문으로 대체
        if (current_idx >= _frames) current_idx -= _frames; 
        
        // [AI 텐서 정합성] 내부 패딩(_stride_dim)을 뺀 순수 차원(_dim)으로 조밀하게 직렬화
        memcpy(p_out_buffer + offset, _data[current_idx], sizeof(float) * _dim);
        offset += _dim;
    }

    // [오염 방어] 부분 플러시의 경우, 외부 버퍼의 남은 잉여 공간을 제로 패딩하여 쓰레기값 노이즈 제거
    size_t total_tensor_bytes = _frames * _dim * sizeof(float);
    if (valid_frames < _frames && max_out_size >= total_tensor_bytes) {
        memset(p_out_buffer + offset, 0, total_tensor_bytes - required_bytes);
    }
}

void CL_T20_SequenceBuilder::reset() {
    _head = 0;
    _is_full = false;
    // O(1) 고속 초기화 완료
}
