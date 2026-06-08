# [MSFM_T2_250] T250_Trigger 기능 규격서

본 문서는 **MSFM_T2_250** 임베디드 펌웨어의 다차원 물리 지표 진단 엔진인 `T250_Trigger` (Trigger Engine) 모듈에 대한 기능 규격서입니다.

---

## 1. 모듈 개요

`T250_Trigger` 모듈은 가속도, 자이로, 소음(Audio) 3대 입력 도메인에서 실시간 추출된 통계 특징값들을 시스템에 로드된 동적 설정 임계치와 비교 분석하는 **진단 룰 엔진 (Rule Engine)**입니다. RMS, 첨도, 파고율, 왜도 및 주파수 밴드 에너지를 다층 분석하여 장비 상태 결함(NG) 여부를 신속하게 판정해 줍니다.

---

## 2. API 명세 (인터페이스 선언)

[T250_Trigger_250.hpp](../T250_Trigger_250.hpp) 클래스의 주요 공개 API 및 구조 정보는 다음과 같습니다.

### `CL_T2_TriggerEngine(void)`
*   **기능설명**: 생성자로, 연속 오류 감지 시 시퀀스 래치 트리거를 확인하기 위한 연속 이상 카운터(`_trialCounter`)를 `0`으로 초기화합니다.

### `T2_Type::EM_DetectionResult_t runDiagnostic(T2_Type::ST_FeatureSlot_Aud_t& p_audSlot, const T2_Type::ST_FeatureSlot_Vib_t& p_vibSlot, const T2_Type::ST_DynamicConfig_t& p_cfg)`
*   **기능설명**: 가속도/자이로 특징량이 담긴 진동 슬롯과 마이크 특징량이 담긴 오디오 슬롯 정보를 참조하여 런타임 룰 검사를 수행하고 최종 진단 상태를 결정해 반환합니다.
*   **동작규격**:
    1.  `_checkAccelRules`를 호출하여 가속도 센서의 3축(X/Y/Z) RMS, 왜도, 첨도, 파고율 및 밴드 에너지 임계치를 스캔합니다.
    2.  `_checkGyroRules`를 호출하여 자이로 센서의 3축 RMS, 왜도, 첨도, 파고율 및 자이로 밴드 에너지를 점검합니다.
    3.  `_checkAudioRules`를 호출하여 L/R 개별 RMS, 첨도, 파고율, 왜도 및 켑스트럼 피크 에너지 한계와 밴드 에너지 임계치를 체크합니다.
    4.  각 서브 규칙 중 하나라도 초과 임계치를 기록할 시 비정상 상태(`RULE_VIB_NG` 또는 `RULE_AUDIO_NG`) 판정을 내리고 연속 감지 카운터를 조정합니다.
*   **반환값**: 진단 결과 열거형 (`EM_DetectionResult_t` : `PASS`, `RULE_VIB_NG`, `RULE_AUDIO_NG` 등).

### `void resetCounter(void)`
*   **기능설명**: 임시 노이즈나 일시적 충격에 의한 트리거 복구를 위해 내부 연속 에러 시험 시도 횟수 카운터(`_trialCounter`)를 제로(`0`)로 리셋합니다.

---

## 3. 세부 진단 규칙 스펙

1.  **가속도 (Accel) 임계치 판정 규칙**:
    가속도 설정 정보 `ST_Accel_Config_t` 상에 정의된 축별 기준을 초과하면 에러로 판단합니다.
    *   **RMS 판정**: 축별 RMS $\ge$ `rms_thresh[axis]`
    *   **첨도 판정**: 축별 Kurtosis $\ge$ `kurt_ng_thresh[axis]`
    *   **파고율 판정**: 축별 Crest Factor $\ge$ `crest_ng_thresh[axis]`
    *   **왜도 판정**: 축별 Skewness $\ge$ `skew_ng_thresh[axis]`
    *   **대역 에너지 판정**: 각 활성 주파수 대역 에너지 $\ge$ `band_thresh[axis][band]`

2.  **오디오 (Audio) 임계치 판정 규칙**:
    오디오 설정 정보 `ST_Audio_Config_t` 상에 정의된 채널별 기준을 초과하면 에러로 판단합니다.
    *   **RMS 판정**: L/R 채널 RMS $\ge$ `rms_thresh[ch]`
    *   **첨도 판정**: L/R 채널 Kurtosis $\ge$ `kurt_ng_thresh[ch]`
    *   **파고율 판정**: L/R 채널 Crest Factor $\ge$ `crest_ng_thresh[ch]`
    *   **왜도 판정**: L/R 채널 Skewness $\ge$ `skew_ng_thresh[ch]`
    *   **대역 에너지 판정**: L/R 채널 대역 에너지 $\ge$ `band_thresh[ch][band]`
    *   **켑스트럼 이상 유무**: 타겟 대역 켑스트럼 최고 피크 오차 범주 미달 여부.
