/* ============================================================================
 * File: T280_Calibrator_243.hpp
 * Summary: 멀티모달 오프라인 캘리브레이션 및 하드웨어 프로파일링 엔진 - v243
 * ============================================================================
 * [로직 상태 추적 (Logic State Tracker)]
 * - 유지: Welch's Method 기반 파워 스펙트럼 평균 추출 및 IFFT 역산 알고리즘.
 * - 갱신: v243 UnifiedRawChunk 기반 블록 파싱 및 4-Tier 설정 자동 반영.
 * - 신규: [원칙 준수] 16-byte 정렬 PSRAM 버퍼 사용 및 WDT 리셋 강제화.
 * ========================================================================== */
#pragma once

#include "T210_Def_243_9.hpp"
#include "T215_Type_243_8.hpp"
#include "T245_FeatExtra_243.hpp"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <FS.h>

class CL_T2_Calibrator {
private:
    TaskHandle_t _hCalibTask = nullptr;
    CL_T2_FeatureExtractor* _refExtractor = nullptr;

public:
    CL_T2_Calibrator();
    ~CL_T2_Calibrator();

    // 노이즈 프로필 리셋 연동을 위한 추출기 바인딩
    void bindExtractor(CL_T2_FeatureExtractor* p_extractor) { _refExtractor = p_extractor; }

    // 비동기 캘리브레이션 태스크 구동 (경로 미지정 시 최근 파일 자동 탐색)
    bool startManualCalibration(const char* p_rawFilePath = nullptr);
    bool startAutoCalibration(const char* p_rawFilePath = nullptr);

    bool isRunning() const { return _hCalibTask != nullptr; }

private:
    struct CalibTaskParam {
        CL_T2_Calibrator* instance;
        char filePath[T2_Def::Global::StorageLimit::MAX_PATH_LEN_CONST];
        bool isManual;
    };

    static void _calibTaskProc(void* p_param);
    void _processManual(const char* p_path);
    void _processAuto(const char* p_path);

    // 유틸리티: SD카드에서 가장 최근에 생성된 캘리브레이션 바이너리 탐색
    bool _findLatestCalibFile(char* p_outPath, size_t p_maxLen, bool p_isManual);
};
