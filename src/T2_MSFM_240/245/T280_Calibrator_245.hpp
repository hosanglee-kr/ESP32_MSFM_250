/* ============================================================================
 * File: T280_Calibrator_245.hpp
 * Summary: 멀티모달 오프라인 캘리브레이션 및 하드웨어 프로파일링 엔진
 * ============================================================================
 * [로직 상태 추적 (Logic State Tracker)]
 * - 유지: Welch's Method 기반 파워 스펙트럼 평균 추출 및 IFFT 역산 알고리즘.
 * - 갱신: 개별 .acc, .gyr, .wav 파일 분리 로딩 및 파싱.
 * - 신규: 16-byte 정렬 PSRAM 버퍼 사용 및 WDT 리셋 강제화.
 * ========================================================================== */
#pragma once

#include "T210_Def_245.hpp"
#include "T215_Type_245.hpp"
#include "T245_FeatExtra_245.hpp"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <FS.h>

/**
 * @class CL_T2_Calibrator
 * @brief 멀티모달 센서 오프라인 캘리브레이션 및 하드웨어 성능 프로파일링 엔진 클래스
 */
class CL_T2_Calibrator {
private:
    TaskHandle_t            _hCalibTask = nullptr;     ///< 캘리브레이션 비동기 태스크 핸들
    CL_T2_FeatureExtractor* _refExtractor = nullptr;   ///< 노이즈 프로필 리셋 연동을 위한 특징 추출기 참조

public:
    CL_T2_Calibrator();
    ~CL_T2_Calibrator();

    /**
     * @brief 특징 추출기 바인딩
     * @param p_extractor 특징 추출기 인스턴스 포인터
     */
    void bindExtractor(CL_T2_FeatureExtractor* p_extractor) { _refExtractor = p_extractor; }

    /**
     * @brief 비동기 수동(정밀) 캘리브레이션 태스크 구동
     * @param p_rawFilePath 대상 파일 경로 (생략 시 최신 파일 자동 탐색)
     * @return 구동 성공 여부
     */
    bool startManualCalibration(const char* p_rawFilePath = nullptr);

    /**
     * @brief 비동기 자동(신속) 캘리브레이션 태스크 구동
     * @param p_rawFilePath 대상 파일 경로 (생략 시 최신 파일 자동 탐색)
     * @return 구동 성공 여부
     */
    bool startAutoCalibration(const char* p_rawFilePath = nullptr);

    /**
     * @brief 캘리브레이션 태스크 실행 여부 확인
     * @return 실행 중이면 true
     */
    bool isRunning() const { return _hCalibTask != nullptr; }

private:
    /**
     * @struct CalibTaskParam
     * @brief 비동기 태스크 실행용 매개변수 구조체
     */
    struct CalibTaskParam {
        CL_T2_Calibrator* instance;                                            ///< 캘리브레이션 인스턴스 포인터
        char              filePath[T2_Def::Global::StorageLimit::PATH_LEN_MAX]; ///< 대상 파일 경로
        bool              isManual;                                            ///< 수동(Advanced FIR) 여부
    };

    static void _calibTaskProc(void* p_param);
    void _processManual(const char* p_path);
    void _processAuto(const char* p_path);

    // 유틸리티: SD카드에서 가장 최근에 생성된 캘리브레이션 바이너리 탐색
    bool _findLatestCalibFile(char* p_outPath, size_t p_maxLen, bool p_isManual);
};

