/* ============================================================================
 * File: T280_Calibrator_242.hpp
 * Summary: 멀티모달 오프라인 캘리브레이션 및 하드웨어 프로파일링 엔진
 * ============================================================================
 * [로직 상태 추적 (Logic State Tracker)]
 * - 유지: Welch's Method 기반 파워 스펙트럼 평균 추출 및 IFFT 역산 알고리즘.
 * - 갱신: [교정] 32bit PCM 스트림을 T240 UnifiedRawChunk 블록 단위로 전면 교체.
 * - 신규: [원칙 준수] 예지보전(Predictive Maintenance)을 위한 auto_history.csv 로깅 복원.
 *
 * [축소/누락 방어 체크리스트 (Omission Defense)]
 * 1. [포맷 파괴 방어]: SD카드 읽기 시 FileHeader를 명시적으로 건너뛰는(seek) 로직 필수.
 * 2. [OOM 및 SIMD 방어]: v_chunk 할당 시 반드시 heap_caps_aligned_alloc 적용 (Rule #21, #23).
 * 3. [데이터 유실 방어]: 진동 3축(X, Y, Z)의 파워를 모두 검사하여 최악(Worst) 축을 기준 삼음.
 * 4. [인터페이스 동기화]: FSM 매니저 연동을 위한 인자 없는 메서드(최신 파일 탐색) 오버로딩 추가.
 * ========================================================================== */
#pragma once

#include "T210_Def_242.hpp"
#include "T215_CfgMgr_242.hpp"
#include "T245_FeatExtra_242.hpp"
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

    // 노이즈 프로필 하드 리셋 연동을 위한 추출기 바인딩
    void bindExtractor(CL_T2_FeatureExtractor* p_extractor) { _refExtractor = p_extractor; }

    // 비동기 캘리브레이션 태스크 구동 (경로 미지정 시 최근 파일 자동 탐색)
    bool startManualCalibration(const char* p_pcmFilePath = nullptr);
    bool startAutoCalibration(const char* p_pcmFilePath = nullptr);

private:
    struct CalibTaskParam {
        CL_T2_Calibrator* instance;
        char filePath[T2_Def::StorageLimit::MAX_PATH_LEN_CONST];
        bool isManual;
    };

    static void _calibTaskProc(void* p_param);
    void _processManual(const char* p_path);
    void _processAuto(const char* p_path);

    // 유틸리티: SD카드에서 가장 최근에 생성된 캘리브레이션 바이너리 탐색
    bool _findLatestCalibFile(char* p_outPath, size_t p_maxLen, bool p_isManual);
};

