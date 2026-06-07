# [MSFM_T2_v250] T280_Calibrator 기능 규격서

본 문서는 **MSFM_T2_v250** 임베디드 펌웨어의 멀티모달 오프라인 보정 및 하드웨어 튜닝 모듈인 `T280_Calibrator` (Calibrator Engine)에 대한 기능 규격서입니다.

---

## 1. 모듈 개요

`T280_Calibrator` 모듈은 공정 설치 시 마이크의 조립 오차나 장비 고유 진동 등으로 인해 입력 감도가 평탄하지 않을 때, 표준 입력원에 맞춰 이득 곡선을 보정하는 **오프라인 캘리브레이션 연산기**입니다. Welch's Method 기반의 파워 스펙트럼 밀도(PSD) 평균 추출 및 역필터 IFFT 계산을 백그라운드 FreeRTOS 태스크로 실행하여 16비트 FIR 이퀄라이저 계수(`eq_coeffs`)를 역산해 냅니다.

---

## 2. API 명세 (인터페이스 선언)

[T280_Calibrator_250.hpp](../T280_Calibrator_250.hpp) 클래스의 공개 API 규격은 다음과 같습니다.

### `CL_T2_Calibrator(void)`
*   **기능설명**: 생성자로, 백그라운드 비동기 연산용 태스크 핸들(`_hCalibTask`)을 `nullptr`로 비우고 대기 상태를 점유합니다.

### `void bindExtractor(CL_T2_FeatureExtractor* p_extractor)`
*   **기능설명**: 캘리브레이션 계산 중 노이즈 스펙트럼 추출 결과 갱신을 주입하기 위해 특징량 추출기 인스턴스의 주소 포인터를 등록 바인딩합니다.

### `bool startManualCalibration(const char* p_rawFilePath = nullptr)`
*   **기능설명**: 수동 캘리브레이션 프로세스 태스크를 비동기로 구동합니다.
*   **동작규격**:
    1.  `p_rawFilePath`가 지정되지 않았을 경우, 내부 함수 `_findLatestCalibFile`을 호출하여 SD카드에서 가장 최근에 녹음되어 누적된 원시 오디오 파형 파일(`.wav`)을 자동 서치합니다.
    2.  동작 스택 `8192` 바이트, 코어 1 할당 조건 하에 `_calibTaskProc` 스레드를 비동기로 시작하고 인수를 넘깁니다.
*   **반환값**: 비동기 연산 스레드 태스크 기동 성공 여부.

### `bool startAutoCalibration(const char* p_rawFilePath = nullptr)`
*   **기능설명**: 자동 환경 보정 캘리브레이션을 구동합니다. 수동 캘리브레이션과 달리 공조 환경 소음 프로파일을 스캔하여 백그라운드 차감 필터 계수 보정을 주로 수행합니다.
*   **반환값**: 태스크 시작 성공 여부.

### `bool isRunning(void) const`
*   **기능설명**: 현재 백그라운드에서 캘리브레이션 수치 해석 연산이 진행 중인지 여부를 질의합니다.
*   **반환값**: `true` (연산 진행 중) / `false` (대기/완료 상태).

---

## 3. 핵심 역산 알고리즘 규격

1.  **PSD 계산 및 주파수 응답 평균화 (Welch's Method)**:
    수집된 캘리브레이션 오디오 파일 전체에서 윈도우 크기 `1024` 샘플 단위로 STFT 파워 스펙트럼을 누적 추출하고 평균을 계산하여 신호의 주파수 특성 곡선 $P_{xx}(f)$을 결정합니다.
2.  **이퀄라이제이션(EQ) 이득 보정 곡선 추출**:
    기준 주파수 대역($1000\text{Hz}$) 또는 사전에 설정된 평탄 감도 기준값에 대입하여 각 FFT 빈별 보정 대상 주파수 반응 계수 $H_{\text{cal}}(f)$를 역산합니다.
    $$H_{\text{cal}}(f) = \frac{\text{TargetResponse}(f)}{\sqrt{P_{xx}(f)}}$$
    *   보정치가 특정 상/하한선(`gain_min` = 0.3, `gain_max` = 3.0)을 넘는 극단적 왜곡 구간의 경우 하드웨어 마이크 불량으로 차단하고 안전 마진 값(`norm_safe`)으로 클램핑하여 발산을 억제합니다.
3.  **IFFT 연산을 통한 FIR 계수 생성**:
    보정 주파수 응답 곡선에 역 퓨리에 변환(IFFT)을 적용하여 시간 도메인의 FIR 필터 탭 계수로 변환합니다. 변환 후 Hann 윈도우를 다시 컨볼루션하여 잔여 사이드로브 노이즈를 억제한 최종 `63차` (`FIR_TAPS_DEF`) FIR 필터 계수를 완성하여 [T220_CfgMgr](../T220_CfgMgr_250.hpp)의 `eq_coeffs` 설정 파일 영역에 최종 덮어쓰기 기록합니다.
