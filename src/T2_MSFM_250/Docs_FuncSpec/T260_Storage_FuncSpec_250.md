# [MSFM_T2_250] T260_Storage 기능 규격서

본 문서는 **MSFM_T2_250** 임베디드 펌웨어의 멀티모달 비동기 대용량 스토리지 모듈인 `T260_Storage` (Storage Manager)에 대한 기능 규격서입니다.

---

## 1. 모듈 개요

`T260_Storage` 모듈은 고속 센서 데이터 수집 흐름에 영향을 주지 않으면서 **SD카드(SD_MMC 인터페이스)**에 특징량 바이너리 파일과 대용량 원시(Raw) 파형 파일을 비동기로 기록하는 저널링 스토리지 엔진입니다. 실시간 쓰기 지연을 예방하기 위해 **PSRAM 링버퍼(Ring Buffer)와 비동기 태스크**를 운용하며, 이벤트 트리거 발생 이전 수초의 기록을 소급 보존하는 **프리트리거(Pre-Trigger) 캐시 버퍼링**을 내장하고 있습니다.

---

## 2. API 명세 (인터페이스 선언)

[T260_Storage_250.hpp](../T260_Storage_250.hpp) 클래스의 공개 API 및 구조 정보는 다음과 같습니다.

### `static CL_StorageTaskManager& getInstance(void)`
*   **기능설명**: 비동기 스토리지 매니저의 싱글톤 인스턴스 참조를 획득합니다.

### `bool init(void)`
*   **기능설명**: SD_MMC 물리 버스를 마운트(`/sdcard`)하고 비동기 제어용 FreeRTOS 메시지 큐(`_qAudStorage`, `_qVibStorage`) 및 뮤텍스(`_lock` 및 `_fsLock`)를 생성합니다. 또한 비동기 파일 입출력을 전담할 백그라운드 태스크 `_storageTaskProc` (우선순위 2, 코어 0)를 기동합니다.
*   **반환값**: SD카드 물리 마운트 및 버퍼 준비 성공 여부.

### `bool openSession(const char* p_prefix, uint64_t p_triggerTimestamp, const T2_Type::ST_TriggerReason_t& p_reason, const char* p_overrideDir = nullptr)`
*   **기능설명**: 새로운 데이터 수집 세션을 개시합니다.
*   **동작규격**:
    1.  전역 NVS 파티션에서 현재 파일 시퀀스 번호를 읽어와 로테이션 서브 시퀀스 번호(`_rotationSubSeq`)를 세팅합니다.
    2.  오늘 날짜 시간 스탬프 및 시퀀스 정보를 조합하여 5대 데이터 채널에 대한 쓰기 대상 파일을 물리 오픈합니다.
    3.  오픈된 바이너리 파일 헤더에 `trigger_t0` 절대 마커와 `ST_TriggerReason_t` 메타데이터 구조체를 바인딩하여 기록합니다.
    4.  SD카드 물리 세그먼트 단편화 방지를 위해 예상 최대 영역(10MB 등)을 선할당(`_preAllocateFile`)합니다.
*   **반환값**: 모든 파일 세션 정상 오픈 여부.

### `void closeSession(const char* p_reason)`
*   **기능설명**: 비동기 링버퍼와 메시지 큐가 완전히 드레인될 때까지 최대 1초간 대기(Graceful Shutdown)한 후, 오픈 중인 모든 파일의 헤더를 최종 업데이트하고 안전하게 Close 처리합니다.

### `bool pushAudioFrame(const T2_Type::ST_FeatureSlot_Aud_t* p_audFeat, const T2_Type::ST_Raw_Audio_t* p_rawAud)`
*   **기능설명**: 실시간 처리 코어로부터 산출된 오디오 특징 정보 및 원시 샘플 파형 데이터를 프리트리거 링버퍼(`_preAudBuf`)에 적재합니다. 이벤트 트리거 중일 경우 즉시 비동기 쓰기 링버퍼(`_asyncAudRing`)로 데이터를 이송합니다.

### `bool pushVibFrame(const T2_Type::ST_FeatureSlot_Vib_t* p_vibFeat, const T2_Type::ST_Raw_Accel_t* p_rawAcc, const T2_Type::ST_Raw_Gyro_t* p_rawGyr)`
*   **기능설명**: 가속도/자이로 특징량 및 물리 파형 데이터를 가속도 프리트리거 버퍼에 적재하고, 트리거 상황 시 비동기 링버퍼(`_asyncVibRing`)로 고속 인출 이송합니다.

### `bool flush(void)`
*   **기능설명**: 비동기 링버퍼에 적체되어 있는 잔여 바이트를 강제로 백그라운드 태스크에 이송하여 SD카드 파일에 완전히 밀어냅니다(Sync).

### `void checkRotation(void)`
*   **기능설명**: 세션 동작 중 주기적으로 호출되어 기록 바이트 크기 혹은 파일 오픈 지속 시간이 설정된 임계치(예: 10MB 또는 60분)를 초과하는지 스캔하여, 초과 시 기존 세션을 안전히 닫고 후속 파일 번호로 신규 세션을 자동 오픈(File Rotation)합니다.

### `bool attemptRecovery(void)`
*   **기능설명**: 쓰기 오류나 SD카드 임시 이탈 감지 시, VFS 마운트를 해제하고 재접속을 시도하여 쓰기 세션을 복구합니다.

### `void dumpPreTriggerToSession()`
*   **기능설명**: 세션 기동 시점에 수집되어 있던 과거 중요 전조 파형(Pre-Trigger) 데이터를 신규 파일의 최상단에 우선 기록합니다.
*   **스냅샷 바운스 버퍼링**: SD 카드 파일 I/O 블로킹 시간 동안 `pushAudioFrame`과의 Lock 경합을 차단하기 위해, `_lock`을 획득하여 프리트리거 데이터를 임시 로컬 메모리 버퍼로 빠르게 일괄 복사한 후 `_lock`을 즉시 해제(Release)하여 실시간 수집 지연을 수 마이크로초 수준으로 줄입니다.

### `bool saveNoiseProfile(const float* p_profile, size_t p_size)` / `loadNoiseProfile(...)`
*   **기능설명**: 엣지 단에서 자가 학습된 배경 소음 노이즈 프로필(`_noiseProfile`)을 LittleFS 파일 시스템의 `/sys/noise_profile.bin` 경로에 저장하고 부팅 시 자동으로 복원하는 기능을 제공합니다. LittleFS 동시 접근 방지를 위해 `_fsLock` 세마포어(Mutex) 동기화를 거칩니다.

---

## 3. 프리트리거 및 바운스 버퍼 매커니즘

1.  **프리트리거 (Pre-Trigger) 링버퍼 구조 (PSRAM 할당)**:
    이벤트 트리거가 당장 들어오지 않더라도 평상시 감시 중일 때 수집된 특징량과 파형을 `3초` (`pre_trig_sec`) 용량으로 순환 버퍼에 항상 보존합니다. 이후 트리거 이벤트가 걸리는 시점에, 해당 3초 분량의 과거 데이터를 비동기 쓰기 링버퍼에 선 복사하여 사고 발생 전조 데이터 유실을 완전 방지합니다.

2.  **바운스 버퍼 (Bounce Buffer) 운용 정책**:
    ESP32-S3 SD/MMC DMA 컨트롤러가 PSRAM 물리 캐시 라인 미스에 의해 오작동을 유발하지 않도록, 쓰기 직전 SRAM 영역에 `alignas(16)` 정렬 선언된 중간 바운스 버퍼(`_bounceAudFeat` 등)에 데이터를 먼저 복사하고 물리 저장을 개시하여 안정성을 100% 확보합니다.
