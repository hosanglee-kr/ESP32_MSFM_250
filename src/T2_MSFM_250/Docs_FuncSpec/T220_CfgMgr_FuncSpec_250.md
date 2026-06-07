# [MSFM_T2_v250] T220_CfgMgr 기능 규격서

본 문서는 **MSFM_T2_v250** 임베디드 펌웨어의 전체 동작 설정을 중앙 관리하는 `T220_CfgMgr` (Config Manager) 모듈에 대한 기능 규격서입니다.

---

## 1. 모듈 개요

`T220_CfgMgr`는 전역 시스템, 와이파이, MQTT, NTP, 저장공간 및 개별 센서(가속도, 자이로, 마이크)의 모든 설정 정보를 보유하는 **단일 진실 공급원 (SSOT, Single Source of Truth)** 역할을 수행합니다. 내부적으로 LittleFS 파일 시스템과 `ArduinoJson` 라이브러리를 이용하여 설정을 저장/로드하고, 플래시 메모리 수명을 보호하기 위한 지연 쓰기(Lazy Write)와 웹 기반 실시간 설정 변경을 위한 튜닝 프리뷰 기능을 제공합니다.

---

## 2. API 명세 (인터페이스 선언)

[T220_CfgMgr_250.hpp](../T220_CfgMgr_250.hpp) 클래스의 공개 API 규격은 다음과 같습니다.

### `static CL_T2_ConfigManager& getInstance(void)`
*   **기능설명**: 설정 관리자 클래스의 싱글톤 인스턴스 참조를 획득합니다.

### `bool init(void)`
*   **기능설명**: 내부 플래시의 LittleFS 파일 시스템을 초기화 및 마운트하고 락 제어용 뮤텍스(`_lock`)를 생성합니다.
*   **반환값**: 마운트 성공 여부 (`true` / `false`).

### `bool load(void)`
*   **기능설명**: `/sys/cfg_250.json` 파일에서 설정을 읽어 메모리 구조체(`_dynConfig`)에 로드합니다. 파일이 없거나 파싱 실패 시 기본값(`_loadDefaults()`)을 구성 후 재생성합니다.
*   **반환값**: 로드 및 메모리 적재 성공 여부.

### `bool save(void)`
*   **기능설명**: 현재 설정 메모리 데이터를 JSON 문자열로 직렬화하여 플래시에 저장합니다.
*   **보안설계 (원자적 쓰기)**:
    1.  먼저 임시 파일 `/sys/cfg_250.json.tmp`에 온전한 JSON 문자열을 생성하여 저장합니다.
    2.  임시 파일 저장이 완료되면 기존 설정 파일을 삭제하고 임시 파일 이름을 `/sys/cfg_250.json`으로 리네임하여 저장 중 전원 불능 시에도 기존 설정이 깨지지 않도록 방어합니다.
*   **반환값**: 플래시 저장 성공 여부.

### `void resetToDefault(void)`
*   **기능설명**: 4-Tier 시스템 전역 설정을 컴파일 타임 기본값으로 롤백하고 플래시에 동기화 저장합니다.

### `T2_Type::ST_DynamicConfig_t getConfig(void)`
*   **기능설명**: 현재 메모리에 유지되고 있는 스냅샷 설정 데이터를 Mutex Lock 보호 조건 아래 안전하게 리턴합니다.
*   **반환값**: `ST_DynamicConfig_t` 복사본.

### `bool updateConfig(const T2_Type::ST_DynamicConfig_t& p_newConfig)`
*   **기능설명**: 전달받은 신규 설정 데이터를 메모리에 기록하고 플래시에 동기(즉시) 저장합니다.
*   **반환값**: 저장 결과.

### `bool updateConfigLazy(const T2_Type::ST_DynamicConfig_t& p_newConfig)`
*   **기능설명**: 신규 설정 데이터를 메모리에 기록하되, 정규 LittleFS 플래시 저장을 뒤로 늦춥니다. 단, 전원 무손실 정합성 확보를 위해 **즉각적인 WAL 커밋(`_walDriver.commitConfigFast(_dynConfig)`)**을 수행하여 초동 전원 무결성을 확보하고, 더티 플래그(`_isDirty`)를 설정하여 백그라운드 태스크에서 플러시되도록 유도합니다.
*   **반환값**: `true` 고정.

### `bool updateFromJson(const char* p_jsonString)`
*   **기능설명**: Web 브라우저나 MQTT 명령으로 전달받은 부분 JSON 텍스트를 분석하여 해당하는 메모리 영역만 갱신 적용하고 지연 쓰기(Lazy Write)를 자동 예약합니다.
*   **반환값**: JSON 파싱 및 병합 성공 여부.

### `void checkLazyWrite(void)`
*   **기능설명**: 백그라운드 루틴(`T2_run` 또는 `runMaintenance`)에서 지속 호출되는 주기 함수입니다. 더티 상태이고 마지막 수정 시점으로부터 `3초` (`LAZY_WRITE_MS_DEF`)가 경과했을 때 실제 LittleFS 플래시 쓰기(`save()`)를 시작하여 파일 IO 쓰기 횟수를 최소화합니다.

### `bool updatePreview(const char* p_jsonString)`
*   **기능설명**: 튜닝 프리뷰 모드를 개시합니다. 플래시에 파일로 백업하지 않고, 순수 메모리 영역(`_dynConfig`) 데이터만 파싱 적용하여 장비가 즉각 실시간 갱신된 필터 계수로 동작하도록 가이드합니다. 튜닝 모드 활성 플래그 `_isTuningActive`를 `true`로 세팅합니다.
*   **반환값**: 파싱 및 메모리 업데이트 성공 여부.

### `bool commitSave(void)`
*   **기능설명**: 현재 프리뷰 적용 중인 튜닝 메모리 데이터를 최종 승인하여 플래시 파일에 영구 기록합니다. 성공 시 튜닝 플래그를 클리어합니다.
*   **반환값**: 영구 저장 성공 여부.

### `bool revertCancel(void)`
*   **기능설명**: 튜닝 적용 과정을 중간 취소합니다. 메모리 설정을 폐기하고 기존 플래시에 저장된 최종 설정 파일에서 복구 로드합니다.
*   **반환값**: 복구 성공 여부.

---

## 2.2 WAL (Write-Ahead Logging) 드라이버 명세

`CL_T2_WalDriver` 클래스의 공개 API 규격은 다음과 같습니다.

### `bool init(const esp_partition_t* p_part)`
*   **기능설명**: 지정된 파티션(`wal`) 정보를 가져와 초기화하고 쓰기 헤더 주소를 검증합니다.
*   **반환값**: 성공 여부.

### `bool loadLatestConfig(T2_Type::ST_DynamicConfig_t& p_cfg)`
*   **기능설명**: `wal` 파티션 내의 16개 섹터를 순회하여 가장 최신의 유효한 시퀀스 번호를 가진 설정 스냅샷(`ST_DynamicConfig_t`)을 역산 로드합니다.
*   **반환값**: 최신 설정 로드 성공 여부.

### `bool commitConfigFast(const T2_Type::ST_DynamicConfig_t& p_cfg)`
*   **기능설명**: 섹터 소거 없이 즉시 새로운 페이지 공간에 바이트 단위로 순차 기록(Erase 없이 Append)하는 초고속 저장을 실행합니다. 수십 µs 이내에 완료됩니다.
*   **반환값**: 커밋 성공 여부.

### `void prepareNextSlot(void)`
*   **기능설명**: WAL 섹터 공간이 포화되어 다음 섹터로 전진해야 할 때, 백그라운드 태스크에 의해 미리 해당 공간을 `0xFF`로 선제 소거(Erase)하여 대기 시간을 예방합니다.

### `void clearAll(void)`
*   **기능설명**: WAL 파티션의 모든 영역을 초기화(소거)합니다.

---

## 3. 핵심 설계 데이터 및 시퀀스

1.  **동적 설정 구조체 (`ST_DynamicConfig_t`)**:
    *   [T215_Type_250.hpp](../T215_Type_250.hpp#L396-L413)에 기술된 마스터 설정형 구조체로서 `system`, `wifi`, `mqtt`, `ntp`, `storage`, `output`, `decision`, `accel`, `gyro`, `audio` 등의 컴포넌트 단위 내부 상세 구조체를 일괄 포함합니다.
2.  **독립 Raw Flash WAL 파티션 (`wal`)**:
    *   `partitions_16MB.csv` 에 선언된 사용자 정의 서브타입(`0x99`), 크기 `0x10000` (64KB, 16개 섹터)의 독립 영역.
3.  **2-Phase Commit 지연 저장 메커니즘 (Lazy Write Sequence with WAL)**:
    ```
    웹/API 요청 -> updateFromJson() -> 메모리(RAM) 값 수정 -> _isDirty=true & _lastModifiedMs=현재시간
                                                                 |
                                              [1단계: 초고속 WAL 동기화 커밋]
                                                                 |
                                          _walDriver.commitConfigFast() 호출 (Erase 없이 Append, 수십 us)
                                                                 |
                                            [2단계: 백그라운드 LittleFS 지연 쓰기]
                                                                 |
                                       checkLazyWrite() 호출 -> 현재시간 - _lastModifiedMs > 3000ms?
                                                                 |
                                            예 -> save() 실행 (cfg_250.json 정규 플러시) 
                                                 및 차기 WAL 영역 0xFF 선제 소거 (Erase 지연 격리)
                                                 및 _isDirty=false
    ```
