# [MSFM_T2_v250] T290_FsmMgr 기능 규격서

본 문서는 **MSFM_T2_v250** 임베디드 펌웨어의 전체 동작 상태 및 멀티코어 태스크 연산을 총괄 지휘하는 오케스트레이터인 `T290_FsmMgr` (FSM Manager) 모듈에 대한 기능 규격서입니다.

---

## 1. 모듈 개요

`T290_FsmMgr` 모듈은 본 4-Tier 진단 시스템의 두뇌 역할을 담당합니다. 시스템 유한 상태 머신(Finite State Machine)의 상태 전이를 통제하고, Core 0과 Core 1에 분산 배치된 실시간 데이터 수집/가공 태스크의 실행 동기화를 공유 컨텍스트(`ST_SharedContext_t`) 더블 버퍼링 구조를 통해 중재합니다. 또한, 하드웨어 릴레이를 preemptive하게 즉각 오프(비활성화)시키는 **PreemptiveSafetyInterlock** 장치를 보유하고 있습니다.

---

## 2. API 명세 (인터페이스 선언)

[T290_FsmMgr_250.hpp](../T290_FsmMgr_250.hpp) 파일에 정의된 클래스들의 공개 API 규격은 다음과 같습니다.

### A. `CL_T2_FsmManager` 클래스

#### `static CL_T2_FsmManager& getInstance(void)`
*   **기능설명**: FSM 매니저 싱글톤 인스턴스에 대한 참조를 리턴합니다.

#### `bool init(void)`
*   **기능설명**: 시스템 부트스트랩 핵심 시퀀스를 조율합니다.
*   **부팅 시퀀스 흐름**:
    1.  [T220_CfgMgr](../T220_CfgMgr_250.hpp)을 가동하여 플래시 메모리에서 최종 저장된 JSON 설정을 복구 로드합니다.
    2.  [T230_Sensor](../T230_Sensor_250.hpp) 및 [T240_DspEng](../T240_DspEng_250.hpp), [T245_FeatExtra](../T245_FeatExtra_250.hpp) 모듈들을 구동 초기화합니다.
    3.  하드웨어 프리엠프티브 릴레이 제어 핀(`PIN_SAFETY_RELAY_CONST` = 25번) 모드를 `OUTPUT` 지정하고 안전 하이(`HIGH` = 정상 상태)를 출력합니다.
    4.  공유 컨텍스트 락-프리 메모리(`_sharedCtx`) 및 세션 명령 큐(`_qSessionCmd`)를 생성합니다.
    5.  FreeRTOS 스레드 3종을 생성 및 바인딩합니다:
        *   `t2_imu_acq` (Core 0, 우선순위 12)
        *   `t2_aud_proc` (Core 1, 우선순위 6)
        *   `t2_vib_proc` (Core 1, 우선순위 5)
    6.  [T270_Commu](../T270_Commu_250.hpp) 통신 레이어를 초기화하여 부팅 완료를 알립니다.
*   **반환값**: 서브 모듈 및 스레드 생성 전체 정상 통과 여부.

#### `void runMaintenance(void)`
*   **기능설명**: 네트워크 생존 관리 루틴 점검, NVS 카운터 영구 저장 기록 및 CfgMgr 지연 쓰기 만료 여부(`checkLazyWrite`)를 주기 연동 처리합니다.

#### `void dispatchCommand(T2_Type::EM_SystemCommand_t p_cmd)`
*   **기능설명**: 인터럽트 키 입력, Web UI, MQTT 브로커로부터 유입되는 외부 런타임 제어 명령어를 분배 및 즉시 처리합니다.
*   **명령별 제어 반응**:
    *   `CMD_START`: 상태를 `MONITORING`으로 전이하고 비동기 수집 태스크를 가동합니다.
    *   `CMD_STOP`: 진행 중인 모니터링 및 저장을 정지하고 대기(`READY`) 상태로 회귀합니다.
    *   `CMD_REBOOT`: 현재 오픈 중인 파일 세션을 긴급 수동 Close하고 500ms 지연 후 칩을 강제 리셋(`esp_restart()`)시킵니다.
    *   `CMD_CALIBRATE`: 오프라인 캘리브레이션 모드로 상태를 즉시 강제 전이시킵니다.

#### `void setState(T2_Type::EM_SystemState_t p_newState)`
*   **기능설명**: 스핀락 임계 영역(`_stateMux`) 진입 하에 원자적(Atomic) 상태 갱신을 수행하고 터미널에 상태 전이 이력을 디버그 로깅합니다.

#### `_broadcastStreams(const T2_Type::ST_FeatureSlot_Aud_t& p_audSlot, const T2_Type::ST_FeatureSlot_Vib_t& p_vibSlot, const float* p_rawAudL, const float* p_rawAudR)`
*   **기능설명**: 수집 완료된 특징량 슬롯(진동/소음)과 원시 음원 데이터를 활용하여 웹소켓 전송용 텔레메트리, 오디오/가속도/자이로 파형, 오디오 파워 스펙트럼 및 시퀀스 텐서 등의 패킷을 구성하고 브로드캐스트합니다.
*   **패킷 매핑 최적화**:
    개정되어 평탄화된 고밀도 패킷 구조체인 `ST_PktTelemetry_t`에 맞춰, 대다수의 불필요한 구버전 필드 할당을 생략하고 `accel_band_energy[16]`, `gyro_rms_energy[2]`, `audio_timbre_bands[32]`, `audio_mfcc[13]`의 4대 FPU-aligned 평탄 메트릭 영역 위주로 데이터를 다이렉트 복사(`memcpy`)하여 복사 대역폭 낭비를 차단합니다.

---

### B. `PreemptiveSafetyInterlock` 및 `SafetyLifecycleManager` 클래스
*   **역할**: 특징량 판단 결과에 따른 비상 상황 발생 시, 프로세스 지연 없이 물리 하드웨어 핀 출력 제어를 통해 프리엠프티브하게 회로 차단(Relay Low)을 수행하고 이를 사용자 수동 리셋 전까지 래치(Latch) 유지합니다.

#### `triggerEmergencyFault(uint16_t fault_bit)`
*   **기능설명**: 결함 검출 즉시 하드웨어 차단 릴레이 제어 핀을 즉각 `LOW`로 떨어뜨리고 비상 오류 비트 마스크에 해당 플래그를 저장합니다.

#### `void evaluateRuleEngine(bool is_density_ng, uint16_t fault_type)`
*   **기능설명**: 상태 분석 스캔 루프에서 지속 호출되어 이상 임계 검출이 참일 경우 `triggerEmergencyFault`를 유발하고 알람 래치 상태를 켭니다.

---

## 3. 핵심 FSM 상태 정의

FSM 매니저가 관리하는 시스템 8대 상태 국면은 다음과 같습니다.

| 상태 (System State) | 설명 |
| :--- | :--- |
| `INIT` | 부팅 직후 각 장치 마운트 및 설정 적재 단계를 나타내는 하드웨어 준비 단계. |
| `READY` | 하드웨어 구동 준비가 완료되었으며, 외부 시작 명령(`CMD_START`) 입력을 대기하는 루프 상태. |
| `MONITORING` | 센서 데이터가 정상 수집/가공되며 결함 규칙 검사를 실시간 수행 중인 상태. |
| `RECORDING` | 트리거 룰 검출에 의해 SD카드에 원시 파형 및 텔레메트리 저장이 진행 중인 임계 국면. |
| `NOISE_LEARNING` | 가속도 및 소음 환경의 배경 노이즈 평균 스펙트럼 밀도를 수집 및 업데이트 중인 상태. |
| `CALIBRATING` | [T280_Calibrator](../T280_Calibrator_250.hpp)에 의해 마이크 FIR 이퀄라이제이션 보정 곡선 계산을 실행 중인 오프라인 상태. |
| `MAINTENANCE` | OTA 펌웨어 무선 업데이트 혹은 외부 수동 캘리브레이션 테스트를 제어 중인 유지 보수 국면. |
| `ERROR` | SD카드 이탈, 센서 I/O 오류, WDT 임계 도달 등 물리 장애로 시스템이 정지된 긴급 국면. |
