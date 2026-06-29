# [MSFM_T2_250] 시스템 기능 상세 흐름도 (Detailed System Function Flowchart)

본 문서는 **MSFM_T2_250** 임베디드 펌웨어의 핵심 동작 시퀀스, 코어/태스크 구성, 실시간 데이터 파이프라인, 비동기 스토리지 기입 모델, 프리엠프티브 인터록 메커니즘을 시각적 다이어그램(Mermaid) 및 단계별 설명으로 구체화한 개발 가이드라인 문서입니다.

---

## 1. 하드웨어 리소스 및 태스크 코어 바인딩 모델

ESP32-S3 Dual-Core MCU 제약을 극복하고, 고주파수 오디오 수집(42kHz) 및 진동 수집(1.6kHz) 환경에서 데이터 유실 없는 처리를 수행하기 위해 다음과 같이 코어 및 우선순위를 배타 배정합니다.

| 태스크명              | 실행 함수               |   할당 코어    |   우선순위   |              주기 및 트리거              | 목적 및 설명                                                   |
| :---------------- | :------------------ | :--------: | :------: | :--------------------------------: | :-------------------------------------------------------- |
| **`ImuAcqTask`**  | `_imuAcqTask`       | **Core 0** | 12 (최상위) | BMI270 FIFO Watermark ISR (약 25ms) | SPI 버스 점유, FIFO 원시 데이터 고속 인출 및 링버퍼 적재                     |
| **`AudProcTask`** | `_audioProcessTask` | **Core 1** |    6     |     I2S DMA 수신 이벤트 (약 12.2ms)      | 오디오 핑퐁 버퍼 수신 시 기동하여 DSP 필터링, 특징 추출, 켑스트럼 분석 및 융합 판정/저장 지시 |
| **`VibProcTask`** | `_vibProcessTask`   | **Core 1** |    5     |     1024 샘플 수집 완료 시 (약 640ms)      | 가속도/자이로 축별 누적 링버퍼 데이터를 가져와 FIR/IIR 처리, 특징 연산 및 비동기 저장 지시  |
| **`StorageTask`**  | `_storageTaskProc`  | **Core 0** |  2 (하위)  |        비동기 스토리지 큐 메시지 수신 시         | SD 카드 파일 쓰기 및 용량/시간 한계 도달 시 파일 로테이션 관리 (락 디커플링 적용)                    |

> [!IMPORTANT]
> **SPI 트랜잭션 스케줄러 기반 상호 배제**:
> 수집 태스크와 설정/교정 태스크가 동시에 SPI 버스에 접근하지 않도록 모든 SPI 제어는 `ST_SpiTransaction_t` 스케줄러 큐를 경유해야 합니다.
>
> **VFS 파일시스템 보호를 위한 fsLock**:
> Lazy Write 데몬, 노이즈 프로필 파일 쓰기 등이 다중 코어에서 LittleFS 파일 시스템에 동시에 진입하여 발생하는 붕괴를 방지하기 위해 `_fsLock` 세마포어로 상호 배제를 적용합니다.

---

## 2. 시스템 초기화 및 부팅 흐름 (System Bootstrap)

시스템 전원이 켜진 후, 4-Tier 시스템 프레임워크가 구동되고 각 서브시스템과 태스크가 가동되는 흐름입니다.

```mermaid
sequenceDiagram
    autonumber
    participant Main as main.cpp (setup)
    participant T200 as T200_Main (T2_init)
    participant FsmMgr as CL_T2_FsmManager
    participant CfgMgr as CL_T2_ConfigManager
    participant Sensor as CL_T2_SensorEngine
    participant DspEng as CL_T2_DspEngine
    participant FeatExt as CL_T2_FeatureExtractor
    participant Storage as CL_T2_StorageManager
    participant Commu as CL_T2_Communicator

    Main->>T200: T2_init() 호출
    activate T200
    Note over T200: 1. Serial (115200) 설정 <br/>2. RTC 메모리 g_CrashDiag 검증 (magic=0x43524153 및 reset_reason 확인)<br/>3. NVS 미존재 컴파일 타임 검증 가드 체크
    T200->>FsmMgr: init() 호출
    activate FsmMgr
    
    FsmMgr->>CfgMgr: init() (JSON 및 NVS 설정 로드, _fsLock 초기화)
    FsmMgr->>Sensor: init() (SPI-BMI270, I2S-ICS43434 핀, 클럭 맵핑, DMA 디스크립터 계산)
    FsmMgr->>DspEng: init() (DC 컷, notch, IIR/FIR 필터 계수 설정, SMEA_FLASH_RODATA 속성 지정)
    FsmMgr->>FeatExt: init() (FFT 버퍼 및 Mel-Filterbank 초기화)
    
    FsmMgr->>Storage: init() 호출
    activate Storage
    Note over Storage: PSRAM 프리트리거/비동기 링버퍼 할당<br/>SD 카드 마운트 및 LittleFS 인덱스 파일 복구
    Storage-->>FsmMgr: StorageTask 생성 (Core 0, Prio 2)
    deactivate Storage

    FsmMgr->>Commu: init() (WebSocket & MQTT 기동)
    
    Note over FsmMgr: 3. 공유 메모리 컨텍스트 (_sharedCtx) PSRAM 할당<br/>4. 세션 명령 큐 (_qSessionCmd) 생성
    
    FsmMgr-->>FsmMgr: ImuAcq Task 생성 (Core 0, Prio 12)
    FsmMgr-->>FsmMgr: AudProc Task 생성 (Core 1, Prio 6)
    FsmMgr-->>FsmMgr: VibProc Task 생성 (Core 1, Prio 5)
    
    FsmMgr->>FsmMgr: setState(READY) 전이
    Note over FsmMgr: LED -> 초록색 (Green) 변경<br/>DSP/Sequence/Trigger 카운터 리셋
    
    FsmMgr-->>T200: 부팅 완료 리턴
    deactivate FsmMgr
    deactivate T200
```

---

## 3. FSM 상태 전이 흐름 (System FSM State Transitions)

`CL_T2_FsmManager`가 주도하는 시스템의 상태와 상태별 전환 트리거 및 LED 인디케이터 맵핑입니다.

```mermaid
stateDiagram-v2
    [*] --> INIT : Power On / Reset
    INIT --> READY : init() 성공 및 초기화 완료
    
    READY --> WARM_UP : CMD_START 디스패치 (물리 버튼 / 웹 CLI / MQTT)
    WARM_UP --> MONITORING : 30초 경과 시 (is_warmup_completed) 자동 전이
    
    MONITORING --> READY : CMD_STOP 디스패치
    WARM_UP --> READY : CMD_STOP 디스패치
    
    MONITORING --> RECORDING : 결함 감지 (Diagnostic Result != PASS) / CMD_MANUAL_REC_START
    RECORDING --> READY : CMD_STOP / CMD_MANUAL_REC_STOP / 저장 기간 완료 후 자동 복귀 (Graceful Shutdown)
    
    READY --> CALIBRATING : CMD_CALIBRATE / Auto-Idle 시간 초과 (배경 소음 차단 보정, State Lock 활성화)
    CALIBRATING --> READY : 캘리브레이션 연산 완료 후 자동 복귀 / CMD_STOP
    
    READY --> NOISE_LEARNING : CMD_LEARN_NOISE 디스패치
    NOISE_LEARNING --> READY : 노이즈 평균 스펙트럼 밀도 학습 완료 후 자동 복귀
    
    MONITORING --> MAINTENANCE : SYS_STATE_OTA 전이 (prepareForOta 실행)
    MAINTENANCE --> MONITORING : OTA 완료 또는 OTA 실패 복구 (resumeFromOtaFailure 실행)
    
    ANY_STATE --> ERROR : LittleFS/SD카드 물리 I/O 쓰기 장애 발생 시 전이
    ERROR --> READY : processManualReset() (사용자 버튼 / 긴급 복구 명령)
```

---

## 4. 실시간 멀티태스킹 데이터 가공 파이프라인 (Real-time Pipeline)

실시간 데이터 파이프라인은 코어 0에서 고속 수집된 원시 데이터를 코어 1로 안전하게 넘기기 위해 **SPSC (Single-Producer Single-Consumer) 락프리 순환 링버퍼 구조**를 채택하고 있습니다. 

```mermaid
graph TD
    %% 코어 배정
    subgraph Core0 [Core 0 : 수집 및 물리 디스크 I/O]
        style Core0 fill:#f0f4f8,stroke:#3b5998,stroke-width:2px
        ISR_Vib[BMI270 Watermark ISR] -->|vTaskNotifyGive| Task_Acq[ImuAcqTask 태스크]
        Task_Acq -->|SPI 스케줄러 큐 경유| Sensor_Vib[BMI270 Sensor]
        Sensor_Vib -->|LSB to G/Dps 변환 및 캘리브레이션| Ring_Sensor[Sensor Circular Buffers]
        
        Task_Storage[StorageTask 태스크] -->|SD MMC Write| SD_Card[(SD Card)]
    end

    subgraph Core1 [Core 1 : DSP 처리 및 추론/판정 엔진]
        style Core1 fill:#f5f6eb,stroke:#8a9a86,stroke-width:2px
        Task_VibProc[VibProcTask 태스크]
        Task_AudProc[AudProcTask 태스크]
        
        %% 진동 가공 흐름
        Ring_Sensor -->|SPSC getAccumulatedAccel/Gyro| Task_VibProc
        Task_VibProc -->|processAccel/Gyro| DSP_Vib[DC 컷, 노치, IIR/FIR 필터]
        DSP_Vib -->|extractAccel/Gyro| Feat_Vib[진동 특징량 추출 <br/> RMS, 첨도, 16-band 에너지, MFCC]
        Feat_Vib -->|std::memory_order_release| Shared_Ctx[공유 메모리 Context _sharedCtx]

        %% 오디오 가공 흐름
        I2S_DMA[I2S DMA 수신 이벤트] -->|readAudioChunk| Task_AudProc
        Task_AudProc -->|processAudio| DSP_Aud[DC 컷, 노치, Hann Window]
        DSP_Aud -->|extractAudio| Feat_Aud[오디오 특징량 추출 <br/> FFT, 1/3 Octave Timbre, MFCC]
        
        %% 특징량 병합 및 Late-Sync
        Feat_Aud -->|"std::memory_order_acquire 읽기"| Shared_Ctx
        Shared_Ctx & Feat_Aud -->|Late-Sync 정렬| Tensor_Binder[Dynamic Tensor Binder]
        Tensor_Binder -->|"MFCC (312차원) 조립"| Seq_Builder[Sequence Builder]
        Seq_Builder -->|TinyML 입력 융합 텐서| TinyML[TinyML 추론 모델]

        %% 진단 및 트리거
        Shared_Ctx & Feat_Aud -->|runDiagnostic| Trig_Eng[Trigger Engine]
        Trig_Eng -->|Fault 감지 시 즉각 릴레이 컷| Safety_Interlock[Preemptive Safety Interlock]
        Safety_Interlock -->|GPIO 25 LOW| Relay[물리 안전 차단 릴레이]

        %% 웹 및 스토리지 중계
        Trig_Eng -->|Fault / Calib 데이터 푸시| Storage_Push[pushAudio / pushVib Frame]
        Storage_Push -->|xQueueSend| Q_Storage[Storage Queues]
        Q_Storage --> Task_Storage

        Feat_Aud & Shared_Ctx -->|broadcastStreams| WS_Commu[CL_T2_Communicator]
        WS_Commu -->|WebSocket Binary Broadcast| External[Web UI / Dashboard]
    end
```

---

## 5. 트리거 및 실시간 안전 차단 인터록 흐름

특징량의 특정 한계치(RMS 및 STA/LTA 비율)가 무너졌을 때 CPU 점유 및 OS 지연을 우회하여 비상 정지를 수행하기 위한 안전 장치입니다.

```mermaid
sequenceDiagram
    autonumber
    participant AudProc as AudProcTask 태스크
    participant TrigEng as CL_T2_TriggerEngine
    participant SafetyMgr as SafetyLifecycleManager
    participant Interlock as PreemptiveSafetyInterlock
    participant Relay as 물리 안전 릴레이 핀

    AudProc->>TrigEng: runDiagnostic(audSlot, vibSlot)
    activate TrigEng
    Note over TrigEng: RMS 레벨 및 임계 비율 검사 (WARM_UP 동안 판정 패스)
    TrigEng-->>AudProc: EM_DetectionResult_t 리턴 (예: RULE_AUDIO_NG)
    deactivate TrigEng

    alt 진단 결과가 PASS가 아님 (이상 검출)
        AudProc->>SafetyMgr: evaluateRuleEngine(is_density_ng = true)
        activate SafetyMgr
        Note over SafetyMgr: 1. 알람 래치 상태 검사 (이미 래치되어 있다면 리턴)<br/>2. 알람 플래그 설정 (_is_alarm_latched = true)
        
        SafetyMgr->>Interlock: triggerEmergencyFault(fault_type)
        activate Interlock
        Note over Interlock: 3. 비상 리페어 맵 비트 설정<br/>4. _is_emergency_fault = true 저장
        
        Interlock->>Relay: digitalWrite(GPIO 25, LOW)
        Note over Relay: 물리 회로 차단 (지연 없이 즉각 차단)
        
        Interlock-->>SafetyMgr: 완료
        deactivate Interlock
        SafetyMgr-->>AudProc: 완료
        deactivate SafetyMgr
        
        Note over AudProc: 5. FSM 상태를 RECORDING으로 전환<br/>6. Storage 세션 기동 및 원시 파형 SD 카드 비동기 저장 시작
    end
```

### 알람 래치 복구 시퀀스
물리 안전 차단은 비상 상황 이후에도 래치(Latch)되어 풀리지 않습니다. 복구를 위해서는 사용자 명령어가 직접 유입되어야 합니다.
1. 사용자가 물리 버튼을 누르거나 웹 대시보드에서 **`CMD_RESET`** 또는 **`processManualReset`** 요청을 입력합니다.
2. `CL_T2_FsmManager::processManualReset()`이 기동됩니다.
3. `SafetyLifecycleManager::processManualReset()` 호출 ➔ `_is_alarm_latched = false`로 변경하고 멀티코어 메모리 배리어(`std::atomic_thread_fence`, `memw`)를 사용하여 모든 코어의 캐시 상태를 동기화합니다.
4. `PreemptiveSafetyInterlock::clearEmergencyLatch()` 호출 ➔ 결함 마스크 비트를 리셋하고 릴레이 제어 핀을 다시 **`HIGH`**로 변경하여 정상 전력을 복구합니다.
5. FSM 상태를 다시 **`READY`** 상태로 복귀시킵니다.

---

## 6. 비동기 이원화 저장소 파일 기입 흐름

SD 카드의 고질적인 물리 쓰기 지연(최대 수백 ms)으로 인해 실시간 데이터 수집 태스크가 영향을 받지 않도록, `StorageTask` 태스크가 비동기 링버퍼 및 대기 전용 프리트리거 메모리를 제어하여 병렬 기입을 처리합니다.

```mermaid
graph TD
    %% 데이터 소스 입력
    Slot_Aud[오디오 특징/Raw 패킷] --> Push_Aud{FSM 세션 상태?}
    Slot_Vib[진동 특징/Raw 패킷] --> Push_Vib{FSM 세션 상태?}

    %% 세션 OFF 상태 (대기 중) -> 프리트리거 링버퍼 적재
    Push_Aud -->|세션 닫힘: READY| Pre_AudBuf[PSRAM 프리트리거 오디오 버퍼 <br/> pre_trig_sec 만큼 순환 적재]
    Push_Vib -->|세션 닫힘: READY| Pre_VibBuf[PSRAM 프리트리거 진동 버퍼 <br/> pre_trig_sec 만큼 순환 적재]

    %% 세션 ON 상태 (트리거) -> 즉시 링버퍼 저장 및 큐 전송
    Push_Aud -->|세션 열림: RECORDING| Ring_Aud[PSRAM 비동기 오디오 링버퍼]
    Push_Vib -->|세션 열림: RECORDING| Ring_Vib[PSRAM 비동기 진동 링버퍼]

    %% 프리트리거 플러시 (스냅샷 바운스 버퍼 활용)
    Trig_Open[세션 오픈 트리거 발생] -->|dumpPreTriggerToSession| SnapshotBuf[정적 할당된 PSRAM 스냅샷 버퍼 복사]
    SnapshotBuf -->|락 즉시 해제| Disk_Write[락 외부에서 물리 f_write 실행]
    Disk_Write --> Ring_Aud
    Disk_Write --> Ring_Vib

    %% 큐 송출
    Ring_Aud -->|Write Index 푸시| Q_AudStorage[xQueue_qAudStorage]
    Ring_Vib -->|Write Index 푸시| Q_VibStorage[xQueue_qVibStorage]

    %% Storage Task 드레인
    subgraph StorageTaskProc [Core 0: _processRingIO]
        Q_AudStorage -->|xQueueReceive| Bounce_Aud[Internal SRAM Bounce Buffer 복사]
        Bounce_Aud -->|Burst Max 8| Disk_Aud[SD_MMC 기입]
        Disk_Aud --> File_AudBin[.aud.bin 특징량 파일]
        Disk_Aud --> File_Wav[.wav 원시 핑퐁 오디오 파일]

        Q_VibStorage -->|xQueueReceive| Bounce_Vib[Internal SRAM Bounce Buffer 복사]
        Bounce_Vib -->|Burst Max 2| Disk_Vib[SD_MMC 기입]
        Disk_Vib --> File_VibBin[.vib.bin 특징량 파일]
        Disk_Vib --> File_Acc[.acc 가속도 원시 파일]
        Disk_Vib --> File_Gyr[.gyr 자이로 원시 파일]
    end
```

---

## 7. 변경 및 갱신 이력 (Revision History)

*   **v2.50 (2026-06-21)**:
    *   구현설계서 v2에 의거하여 부팅 시퀀스(NVS 검증, Crash Diag 검증, fsLock 추가) 현행화.
    *   FSM 상태 전이에 `WARM_UP` 단계(30초) 및 SD I/O 에러 정책 추가.
    *   OTA Graceful Pause/Resume 프로세스 시퀀스 추가.
    *   프리트리거 덤프 시 스택 오버플로우 및 락 경합 방지를 위한 PSRAM 스냅샷 바운스 버퍼 플러시 흐름 반영.
