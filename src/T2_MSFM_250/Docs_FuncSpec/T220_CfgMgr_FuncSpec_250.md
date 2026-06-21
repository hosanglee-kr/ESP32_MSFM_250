# [MSFM_T2_250] T220_CfgMgr 기능 규격서

본 문서는 **MSFM_T2_250** 임베디드 펌웨어의 전체 동작 설정을 중앙 관리하는 `T220_CfgMgr` (Config Manager) 모듈에 대한 기능 규격서입니다.

---

## 1. 모듈 개요

`T220_CfgMgr`는 전역 시스템, 와이파이, MQTT, NTP, 저장공간 및 개별 센서(가속도, 자이로, 마이크)의 모든 설정 정보를 보유하는 **단일 진실 공급원 (SSOT, Single Source of Truth)** 역할을 수행합니다. 내부적으로 LittleFS 파일 시스템과 `ArduinoJson` 라이브러리를 이용하여 설정을 저장/로드하고, 플래시 메모리 수명을 보호하기 위한 지연 쓰기(Lazy Write)와 웹 기반 실시간 설정 변경을 위한 튜닝 프리뷰 기능을 제공합니다.

기존의 복잡한 WAL(Write-Ahead Logging) 드라이버 설계를 전면 폐기하고 표준 NVS 및 캐시 구조로 변경하여 단순성 및 신뢰성을 확보하였습니다.

또한 LittleFS 파일 시스템에 병렬 스레드가 접근하여 VFS 충돌이 일어나는 레이스 컨디션을 예방하기 위해, 모든 파일 입출력 API 진입로(환경 노이즈 프로필 파일 저장/로드인 `saveNoiseProfile`, `loadNoiseProfile` 포함)에 `_fsLock` 세마포어(Mutex)를 정의하여 상호 배제 동기화를 의무화하였습니다.

---

## 2. API 명세 (인터페이스 선언)

[T220_CfgMgr_250.hpp](../T220_CfgMgr_250.hpp) 클래스의 공개 API 규격은 다음과 같습니다.

### `static CL_T2_ConfigManager& getInstance(void)`
*   **기능설명**: 설정 관리자 클래스의 싱글톤 인스턴스 참조를 획득합니다.

### `bool init(void)`
*   **기능설명**: 내부 플래시의 LittleFS 파일 시스템을 초기화 및 마운트하고 락 제어용 뮤텍스(`_lock` 및 `_fsLock`)를 생성합니다.
*   **반환값**: 마운트 성공 여부 (`true` / `false`).

### `bool load(void)`
*   **기능설명**: `/sys/cfg_250.json` 파일에서 설정을 읽어 메모리 구조체(`_dynConfig`)에 로드합니다. 파일이 없거나 파싱 실패 시 기본값(`_loadDefaults()`)을 구성 후 재생성합니다.
*   **반환값**: 로드 및 메모리 적재 성공 여부.

### `bool save(void)`
*   **기능설명**: 현재 설정 메모리 데이터를 JSON 문자열로 직렬화하여 플래시에 저장합니다.
*   **보안설계 (원자적 쓰기 & Mutex Lock)**:
    1.  `_fsLock`을 선취하여 LittleFS VFS 영역 진입을 보장합니다.
    2.  먼저 임시 파일 `/sys/cfg_250.json.tmp`에 온전한 JSON 문자열을 생성하여 저장합니다.
    3.  임시 파일 저장이 완료되면 기존 설정 파일을 삭제하고 임시 파일 이름을 `/sys/cfg_250.json`으로 리네임하여 저장 중 전원 불능 시에도 기존 설정이 깨지지 않도록 방어합니다.
*   **반환값**: 플래시 저장 성공 여부.

### `void resetToDefault(void)`
*   **기능설명**: 4-Tier 시스템 전역 설정을 컴파일 타임 기본값으로 롤백하고 플래시에 동기화 저장합니다.

### `T2_Type::ST_DynamicConfig_t getConfig(void)`
*   **기능설명**: 현재 메모리에 유지되고 있는 스냅샷 설정 데이터를 Mutex Lock 보호 조건 아래 안전하게 리턴합니다.
*   **반환값**: `ST_DynamicConfig_t` 복사본.

### `bool updateConfig(const T2_Type::ST_DynamicConfig_t& p_newConfig)`
*   **기능설명**: 전달받은 신규 설정 데이터로 메모리를 갱신하고 플래시에 동기(즉시) 저장합니다.
*   **반환값**: 저장 결과.

### `bool updateConfigLazy(const T2_Type::ST_DynamicConfig_t& p_newConfig)`
*   **기능설명**: 신규 설정 데이터를 메모리에 기록하되, 정규 LittleFS 플래시 저장을 뒤로 늦춥니다. 더티 플래그(`_isDirty`)를 설정하여 백그라운드 태스크에서 플러시되도록 유도합니다.
*   **반환값**: `true` 고정.

### `bool updateFromJson(const char* p_jsonString)`
*   **기능설명**: Web 브라우저나 MQTT 명령으로 전달받은 부분 JSON 텍스트를 분석하여 해당하는 메모리 영역만 갱신 적용하고 지연 쓰기(Lazy Write)를 자동 예약합니다.
*   **반환값**: JSON 파싱 및 병합 성공 여부.
*   **메모리 최적화 (ArduinoJson V7 적용)**:
    *   ArduinoJson V7 표준에 의거하여 클래스 멤버 영역에 공용 `JsonDocument`인 `_docPool`을 사용하여 힙 단편화와 OOM 리스크를 소멸시킵니다.
    *   파싱 완료 후 `_docPool.shrinkToFit()`을 호출하여 메모리를 최적화 평탄화합니다.

### `void checkLazyWrite(EM_SystemState_t currState)`
*   **기능설명**: 백그라운드 루틴(`T2_run` 또는 `runMaintenance`)에서 1초 주기로 호출되는 관리 함수입니다.
*   **동작 제약 (WAV 녹화 중 플래시 I/O 유예)**:
    *   시스템 상태가 실시간 녹화 수집 모드(`SYS_STATE_WAV_RECORDING`) 시에는 물리 플래시 쓰기 작업을 유예하고 IDLE 상태로 전이되었을 때 플러시하도록 제어하여 실시간 스트림 수집 간섭을 배제합니다.
    *   더티 상태이고 마지막 수정 시점으로부터 `10초` (`LAZY_WRITE_MS_DEF = 10000`)가 경과했을 때 LittleFS 플래시 쓰기(`save()`)를 기입하고 NVS 변경 내역을 반영합니다.

### `void saveCriticalConfig(const ST_DynamicConfig_t& cfg)`
*   **기능설명**: 사용자 캘리브레이션 락 설정, 크래시 진단 데이터 등 치명적 변경 정보를 지연 처리 없이 플래시 NVS 영역에 즉각 커밋(`nvs_commit`)하는 Immediate Commit API입니다.

### `bool updatePreview(const char* p_jsonString)`
*   **기능설명**: 튜닝 프리뷰 모드를 개시합니다. 플래시에 파일로 백업하지 않고, 순수 메모리 영역(`_dynConfig`) 데이터만 파싱 적용하여 장비가 즉각 실시간 갱신된 필터 계수로 동작하도록 가이드합니다. 튜닝 모드 활성 플래그 `_isTuningActive`를 `true`로 세팅합니다.
*   **반환값**: 파싱 및 메모리 업데이트 성공 여부.

### `bool commitSave(void)`
*   **기능설명**: 현재 프리뷰 적용 중인 튜닝 메모리 데이터를 최종 승인하여 플래시 파일에 영구 기록합니다. 성공 시 튜닝 플래그를 클리어합니다.
*   **반환값**: 영구 저장 성공 여부.

### `bool revertCancel(void)`
*   **기능설명**: 튜닝 적용 과정을 중간 취소합니다. 메모리 설정을 폐기하고 기존 플래시에 저장된 최종 설정 파일에서 복구 로드합니다.
*   **반환값**: 복구 성공 여부.

### `bool saveNoiseProfile(const float* profile, size_t size)` / `bool loadNoiseProfile(float* profile, size_t size)`
*   **기능설명**: 노이즈 제거용 배경 소음 프로필을 LittleFS 파일에 안전하게 쓰고 읽어옵니다. 다중 코어 접근 방지를 위해 내부적으로 `_fsLock`을 의무 획득 및 해제합니다.

---

## 3. 핵심 설계 데이터 및 시퀀스

1.  **동적 설정 구조체 (`ST_DynamicConfig_t`)**:
    *   마스터 설정형 구조체로서 `system`, `wifi`, `mqtt`, `ntp`, `storage`, `accel`, `gyro`, `audio` 등의 컴포넌트 단위 내부 상세 구조체를 일괄 포함합니다.
2.  **LittleFS 동시 접근 제어 (`_fsLock`)**:
    *   두 코어에서 파일 시스템에 접근하여 오염이 유발되는 것을 방어하기 위해 모든 쓰기/지우기 루틴은 반드시 `_fsLock` 세마포어로 보호됩니다.
3.  **WAV 녹화 시 Lazy Write 홀딩 시퀀스**:
    ```
    1초 주기 틱 수신 -> checkLazyWrite(currState) 호출
                              |
                     currState == SYS_STATE_WAV_RECORDING ?
                              |
                    [예] ----> 플래시 기입 유예 (즉시 리턴)
                    [아니오] -> _isDirty && 10초 경과? -> save() 기입 및 NVS 커밋
    ```

---

## 4. 변경 및 갱신 이력 (Revision History)

*   **v2.50 (2026-06-21)**:
    *   WAL 드라이버 구조체 및 파티션 명세 완전 삭제 및 NVS 표준 대체.
    *   ArduinoJson V7 규격 `JsonDocument` 적용 및 `shrinkToFit()` 평탄화 메모리 최적화 반영.
    *   `checkLazyWrite`에서 실시간 녹화(`SYS_STATE_WAV_RECORDING`) 시 저장 지연 가드 및 10초 주기 병합 적용.
    *   NVS 즉시 영속화를 위한 `saveCriticalConfig` Immediate Commit API 추가.
    *   배경 소음 프로필 입출력(`saveNoiseProfile`, `loadNoiseProfile`) 시 LittleFS `_fsLock` Mutex 적용 설계 보완.
