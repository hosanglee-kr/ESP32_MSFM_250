/* ============================================================================
 * [SMEA-100 핵심 구현 원칙 및 AI 셀프 회고 바이블]
 * 1. 매직넘버 철폐: 모든 하드웨어 및 버퍼 크기는 SmeaConfig 의존.
 * 2. 16-Byte 정렬(SIMD 최적화): esp-dsp 가속기의 128-bit 처리 특성을 극대화하기 위해
 * 모든 연산 버퍼 및 필터 계수는 반드시 alignas(16)으로 메모리를 정렬한다.
 * 3. [방어] DMA 버퍼 크기 보장: 수집 버퍼 크기 초과로 인한 메모리 오염(Overrun)을 
 * 막기 위해 읽기 요청 길이를 FFT_SIZE_CONST로 엄격히 제한(Clamp)한다.
 * 4. [네이밍 컨벤션 엄수]: private(_), 매개변수(p_), 로컬변수(v_) / <cstdint> 사용
 * ============================================================================
 * File: T480_MicEng_013.hpp
 * Summary: I2S DMA Microphone Acquisition Engine (ICS43434)
 * * [AI 메모: 제공 기능 요약]
 * 1. ESP32-S3 I2S 하드웨어를 제어하여 2채널 42kHz 32bit 데이터를 DMA로 수집.
 * 2. 수집된 교차(Interleaved) 데이터를 L/R 독립 버퍼로 분리(De-interleave).
 * 3. FSM 상태 동기화 전력 제어 (Pause/Resume) 및 DMA 버퍼 클리어 지원.
 *
 * * [AI 메모: 구현 및 유지보수 주의사항]
 * 1. FSM 상태가 READY(대기) 모드일 때 CPU와 I2S 버스가 낭비되는 것을 막기 위해
 * pause() 메서드를 호출하여 I2S 클럭을 정지하고 마이크를 절전 모드로 전환합니다.
 * 2. resume() 호출 시 반드시 clearBuffer()가 선행되어야 과거의 쓰레기 데이터로 인한
 * 오탐지를 방지할 수 있습니다.
 * 3. [v012 연계 보완]: 향후 OTA(MAINTENANCE 상태) 진입 시 시스템 버스 락을
 * 해제하기 위해 pause()와 clearBuffer()가 완벽히 동기화되어야 합니다.
 * ========================================================================== */
#pragma once

#include "T410_Def_013.hpp"
#include <driver/i2s.h>
#include <cstring>
#include <cstdint>

class T480_MicEngine {
private:
    bool _isInitialized = false;
    bool _isPaused = false;
    char _statusText[32];

    // [방어/최적화] DMA에서 읽어올 임시 교차(Interleaved) 버퍼 (SIMD 가속용 16바이트 정렬)
    // DMA 컨트롤러는 내부 SRAM 할당이 필요하므로 클래스 멤버로 선언
    alignas(16) int32_t _dmaBuffer[SmeaConfig::System::FFT_SIZE_CONST * 2];

public:
    T480_MicEngine();
    ~T480_MicEngine() = default;

    bool init();
    void pause();
    void resume();
    void clearBuffer();

    /**
     * @brief DMA로부터 데이터를 읽어 L/R 채널을 분리 후 반환
     * @param p_outL 마이크 L 출력 버퍼
     * @param p_outR 마이크 R 출력 버퍼
     * @param p_reqSamples 요청 샘플 수 (내부에서 FFT_SIZE_CONST로 클램프됨)
     * @return 실제 읽어들인 샘플 수
     */
    uint32_t readData(float* p_outL, float* p_outR, uint32_t p_reqSamples);

    const char* getStatusText() const { return _statusText; }
};

