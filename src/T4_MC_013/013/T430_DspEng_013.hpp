/* ============================================================================
 * [SMEA-100 핵심 구현 원칙 및 AI 셀프 회고 바이블]
 * 1. [방어] LTI 법칙 최적화: FIR/IIR 필터는 선형 시불변(LTI)이므로, L/R 채널을
 * 각각 연산하지 않고 Beamforming(L+R)을 먼저 수행하여 1채널로 만든 뒤 필터링을
 * 1회만 수행한다. (CPU 및 필터 상태 메모리 50% 절감)
 * 2. [방어] 프레임 단절(Tearing) 방어: 블록 단위 스트리밍의 한계를 극복하기 위해,
 * Pre-emphasis 및 Median 필터는 반드시 이전 프레임의 꼬리(History) 데이터를
 * 보존하는 상태 변수를 유지하여 팝핑(Popping) 노이즈를 원천 차단한다.
 * 3. [방어] FIR 짝수 탭 붕괴 트랩: 스펙트럼 반전 기반 HPF 생성 시 탭 수가 짝수면
 * 대칭성이 파괴되므로, static_assert를 통해 컴파일 타임에 이를 원천 차단한다.
 * 4. [방어] 포인터 오염 방지: 입력 p_micL, p_micR은 반드시 const로 선언하여,
 * T440이 위상(IPD) 분석 시 사용할 원본 파형이 훼손(In-place 덮어쓰기)되지 않도록 한다.
 * 5. [네이밍 및 자료형 엄수]: private(_), 매개변수(p_), 로컬변수(v_) / <cstdint> 사용
 * 6. [v013 핫스왑 방어]: process() 핫패스 내에서 Mutex 호출을 완전히 제거.
 * T415의 isTuningActive() 플래그를 감지할 때만 reloadCalibration()을 수행하여
 * 프레임 유실 없는 실시간 Web UI/UX 차트 피드백을 완성한다.
 * ============================================================================
 * File: T430_DspEng_013.hpp
 * Summary: SIMD Optimized Signal Processing Pipeline (Mono-Reduced Hybrid Mode)
 * ========================================================================== */
#pragma once

#include "T410_Def_013.hpp"     
#include "T420_Types_013.hpp"      
#include "T415_ConfigMgr_013.hpp"  
#include "dsps_fft2r.h"
#include "dsps_biquad.h"
#include "dsps_fir.h"
#include <cstdint>                 

static_assert(SmeaConfig::FeatureLimit::FIR_TAPS_CONST % 2 == 1, "[CRITICAL ERROR] FIR_TAPS_CONST MUST be an odd number for HPF Spectral Inversion!");

class T430_DspEngine {
private:
    alignas(16) float _notchCoeffs[5];                                              
    alignas(16) float _notch2Coeffs[5];                                             
    alignas(16) float _firLpfCoeffs[SmeaConfig::FeatureLimit::FIR_TAPS_CONST + 3];  
    alignas(16) float _firHpfCoeffs[SmeaConfig::FeatureLimit::FIR_TAPS_CONST + 3];  

    alignas(16) float _wNotch[2];                                                   
    alignas(16) float _wNotch2[2];                                                  
    alignas(16) float _firStateLpf[SmeaConfig::FeatureLimit::FIR_TAPS_CONST + 3];   
    alignas(16) float _firStateHpf[SmeaConfig::FeatureLimit::FIR_TAPS_CONST + 3];   

    fir_f32_t _firInstHpf;                                                          
    fir_f32_t _firInstLpf;                                                          

    float _prevPreEmpSample;                                                        
    alignas(16) float _medianHistory[SmeaConfig::DspLimit::MAX_MEDIAN_WINDOW_CONST / 2];

    alignas(16) float _workBufA[SmeaConfig::System::FFT_SIZE_CONST];
    alignas(16) float _workBufB[SmeaConfig::System::FFT_SIZE_CONST];
    
    fir_f32_t         _firInstEq;
    alignas(16) float _firEqCoeffs[SmeaConfig::CalibLimit::EQ_FIR_TAPS_CONST + 3];
    alignas(16) float _firStateEq[SmeaConfig::CalibLimit::EQ_FIR_TAPS_CONST + 3];
    
    ST_Config_Dsp _dspCfg; // Mutex Lock 우회 및 전체 DSP 파라미터 캐싱용(calib 보정계수 등)

public:
    T430_DspEngine();
    ~T430_DspEngine();

    bool init();
    void resetFilterStates();
    void reloadCalibration(); // 보정 또는 핫스왑 완료 시 안전한 계수/상태 리셋

    void process(const float* p_micL, const float* p_micR, float* p_output, uint32_t p_len);

private:
    void _applyBeamforming(const float* p_L, const float* p_R, float* p_out, uint32_t p_len, float p_gain);
    void _applyMedianFilter(float* p_data, uint8_t p_windowSize, uint32_t p_len);   
    void _applyPreEmphasis(float* p_data, uint32_t p_len, float p_alpha);
    void _applyNoiseGate(float* p_data, uint32_t p_len, float p_gateThresh);

    void _generateFirLpfWindowedSinc(float* p_coeffs, uint16_t p_taps, float p_cutoffHz);
    void _generateFirHpfWindowedSinc(float* p_coeffs, uint16_t p_taps, float p_cutoffHz);
    
    // [v013 추가] 동적 필터 재계산을 위한 헬퍼 (핫스왑 지원)
    void _recalcDynamicFilters();
};

