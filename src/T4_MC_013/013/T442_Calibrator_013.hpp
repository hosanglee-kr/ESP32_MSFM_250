/* ============================================================================
 * File: T442_Calibrator_013.hpp
 * Summary: Offline Calibration & Hardware Profiling Engine
 * * [AI 메모: v013 고도화 핵심 사항]
 * 1. OOM 방어: 3.3MB Raw 파일을 한 번에 PSRAM에 올리지 않고, Chunk 단위로 
 * 읽어 Welch's Method로 파워 스펙트럼 평균을 안전하게 추출.
 * 2. L/R 밸런싱: 하드 리미트(0.7 ~ 1.4배) 방어로 무한 증폭(화이트 노이즈) 방지.
 * 3. 스레드 독립성: CALIBRATING 상태에서 독립된 FreeRTOS 태스크로 구동.
 * 4. [v013 추가] Pure Math FIR 역산: IFFT 최적화와 Blackman 윈도윙을 통한 
 * 고정밀 주파수 평탄화 알고리즘 탑재 및 Web JSON 튜닝 파라미터 연동 적용.
 * ========================================================================== */
#pragma once

#include "T410_Def_013.hpp"
#include "T415_ConfigMgr_013.hpp"
#include "T440_FeatExtra_013.hpp"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <FS.h>

class T442_Calibrator {
private:
    TaskHandle_t _hCalibTask = nullptr;
    T440_FeatureExtractor* _refExtractor; // 노이즈 프로필 리셋 접근용

    // FFT 연산용 16바이트 정렬 내부 스크래치 버퍼
    alignas(16) float _fftWorkBuf[SmeaConfig::System::FFT_SIZE_CONST * 2];
    alignas(16) float _powerAccL[(SmeaConfig::System::FFT_SIZE_CONST / 2) + 1];
    alignas(16) float _powerAccR[(SmeaConfig::System::FFT_SIZE_CONST / 2) + 1];

public:
    T442_Calibrator();
    ~T442_Calibrator();

    void bindExtractor(T440_FeatureExtractor* p_extractor) { _refExtractor = p_extractor; }

    // 비동기 태스크 구동
    bool startManualCalibration(const char* p_pcmFilePath);
    
    bool startAutoCalibration(const char* p_pcmFilePath);

private:
    static void _calibTaskProc(void* p_param);
    void _processManual(const char* p_path);
    void _processAuto(const char* p_path);
};

