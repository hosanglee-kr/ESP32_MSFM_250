# [T215_Type] 4-Tier 데이터 타입 및 패킷

본 문서는 **MSFM_T2_250** 시스템의 핵심 자료형, 열거형(Enum), 런타임 제어 설정 및 네트워크 전송 패킷 모델을 기술한 `T215_Type_250.hpp` 모듈의 기능 규격서입니다.

---

## 1. 개요 및 설계 원칙

1.  **도메인별 물리 격리**:
    *   기존의 `UnifiedRawChunk` 구조를 완전 해체하고 Accel, Gyro, Audio의 고유 샘플 수율(Hz) 및 윈도우 크기에 독립적으로 대응하는 3개의 `ST_Raw_*_t` 구조체로 분리하여 버퍼 낭비를 없앴습니다.
2.  **비트 마스크 최적화**:
    *   `EM_AxisMask_t` (가속도/자이로 활성 축), `EM_ChannelMask_t` (오디오 활성 채널) 비트 마스크를 도입하여 사용하지 않는 채널의 연산을 런타임에 즉각 건너뛸 수 있도록 지원합니다.
3.  **SIMD 및 네트워크 정렬**:
    *   연산의 실시간 연쇄 수행 속도를 보장하기 위해 주요 원시 데이터 구조체 및 특징량 슬롯은 **`alignas(16)`** 정렬을 의무화합니다.
    *   네트워크 전송 오버헤드 최소화를 위해 바이너리 패킷은 **`#pragma pack(push, 1)`** 지시어로 1바이트 팩킹을 보장합니다.

---

## 2. 시스템 열거형 (Enumerations)

*   **`EM_SystemState_t`**: 시스템의 수명 주기를 제어합니다.
    *   `INIT`(초기화), `READY`(준비), `MONITORING`(실시간 모니터링), `RECORDING`(이벤트 기록), `NOISE_LEARNING`(배경 소음 학습), `MAINTENANCE`(점검), `ERROR`(장애 상태), `CALIBRATING`(이퀄라이제이션 보정)
*   **`EM_SystemCommand_t`**: 외부/내부로부터 전달되는 FSM 제어 명령입니다.
    *   `CMD_START`, `CMD_STOP`, `CMD_LEARN_NOISE`, `CMD_CALIBRATE`, `CMD_REBOOT`, `CMD_MANUAL_REC_START` 등
*   **`EM_TriggerSource_t`**: 결함 판정 및 기록을 시작하도록 한 트리거의 근원입니다.
    *   `NONE`, `HW_WAKE` (BMI270 모션 감지), `SW_RMS` (특징 RMS 임계치 초과), `SW_BAND` (특정 대역 에너지 초과), `MANUAL` (사용자 강제 명령)
*   **`EM_WiFiMode_t`**: `STA_ONLY`, `AP_ONLY`, `AP_STA`, `AUTO_FALLBACK` (AP 연결 실패 시 자동 백업 복구)
*   **`EM_AsyncSessionCmd_t`**: 비동기 스토리지 파일 쓰기 제어 이벤트 종류.
    *   `OPEN_AUTO`, `OPEN_MANUAL`, `OPEN_CALIB_MAN`, `CLOSE_NORMAL`, `CLOSE_MANUAL`, `CLOSE_CALIB_DONE`

---

## 3. 동적 설정 구조체 (Dynamic Config)

런타임에 JSON 및 NVS 설정값과 1:1로 매핑되는 제어 변수 구조체들입니다.

*   **`ST_Global_System_t`**: 현장 및 설비 식별자(`site_id`), 원시 파형 및 텔레메트리 송출 속도(Hz), 운용 모드(`EM_OpMode_t`) 등.
*   **`ST_Global_Decision_t`**: MLOps를 위해 최적화된 동적 판정 상수로 `max_trial_count`, `sta_lta_threshold`, `min_trigger_count` 등을 런타임에 개별 변경할 수 있도록 독립 구조체로 구성했습니다.
*   **`ST_Dsp_Config_t`**: DC 제거 필터 활성화 여부, 메디안 윈도우 크기, HPF/LPF/Notch 필터 활성 및 컷오프 주파수 등을 관리합니다.
*   **`ST_Accel_Config_t` / `ST_Gyro_Config_t` / `ST_Audio_Config_t`**:
    *   각 센서 채널 활성화 여부, ODR 설정, FFT 크기, 축/채널별 RMS 및 대역(Band) 임계치, FIR 이퀄라이제이션 필터 탭 계수(`eq_coeffs`) 등을 개별 관리합니다.
*   **`ST_DynamicConfig_t`**: 위의 세부 설정 구조체를 일괄 포함하는 **마스터 동적 설정 구조체**입니다.

---

## 4. 데이터 모델 및 특징량 메모리 구조 (Aligned Structures)

실시간 SIMD 및 FPU 연산 마진을 극대화하기 위해 16바이트 정렬이 물리적으로 적용된 데이터 슬롯들입니다.

*   **`ST_Raw_Accel_t` / `ST_Raw_Gyro_t` / `ST_Raw_Audio_t`**:
    *   실시간 수집된 원시 파형 및 타임스탬프(`ts`), 주율을 보존하는 16바이트 정렬 정적 2D 배열입니다.
*   **`ST_Slot_Header_t`**:
    *   특징량 패킷의 식별을 위한 공통 32바이트 정렬 헤더로 타임스탬프(`ts`), 프레임 ID(`fid`), 가동 시간(`uptime`), 센서별 채널 마스크 등을 기록합니다.
*   **`ST_Slot_Accel_t` / `ST_Slot_Gyro_t`**:
    *   RMS, Peak Freq, Spectral Centroid, Kurtosis, Crest Factor, 왜도(Skewness), 표준편차, 16개 주파수 대역 에너지(`band_energy`)를 포함하며, 전체 크기가 16의 배수로 완벽히 정렬(`_pad` 추가)되어 있습니다.
*   **`ST_Slot_Audio_t`**:
    *   스테레오 각 채널의 RMS, 에너지, 왜도, 첨도, 켑스트럼 피크 에너지(`cpsr_max`/`cpsr_mxr`), 16개 대역 에너지 및 1/3 옥타브 대역 상대 비율 에너지인 Timbre 벡터(`timbre_bands[32]`)를 결합한 576바이트 정렬 구조체입니다.
*   **`ST_FeatureSlot_Vib_t` / `ST_FeatureSlot_Aud_t`**:
    *   도메인별로 팩킹된 특징량 데이터 전송 슬롯으로, 스토리지 및 네트워크 발행 큐에서 더블 버퍼링 구조(`ST_SharedContext_t`)에 실시간 적재됩니다.

---

## 5. 통신용 팩킹 패킷 명세 (1-Byte Packed Packets)

웹소켓 및 이진 파일의 직렬화 연산을 고속 처리하기 위한 바이너리 팩킹 구조체들입니다.

*   **`ST_WsHeader_t`**: 웹소켓 전송 패킷 헤더 (8 Bytes).
*   **`ST_TriggerReason_t`**: 트리거 원인을 상세히 담고 있는 MLOps 관제 전용 메타데이터 구조체.
*   **`ST_FileHeader_t`**: 스토리지 바이너리 파일 저장 시, 이벤트 시점을 지목하는 T0 마커 타임스탬프(`trigger_t0`) 및 트리거 사유(`reason`)를 저장하는 헤더 구조체.
*   **`ST_PktTelemetry_t` (텔레메트리 패킷 - 320 Bytes)**:
    *   헤더, 시스템 상태, 트리거 원인, 타임스탬프, 온도 정보 등 메타데이터와 **가장 연산이 빈번한 4대 평탄화 지표**가 최후미 16바이트 정렬을 준수하여 배치되어 있습니다.
    *   배치: `accel_band_energy[16]` (64B) -> `gyro_rms_energy[2]` + `_pad_gyro` (16B) -> `audio_timbre_bands[32]` (128B) -> `audio_mfcc[13]` + `_pad_end` (64B)
    *   *컴파일 타임에 `offsetof` 검증 통과 완료.*
*   **`ST_PktWaveform*`**: 오디오/가속도/자이로 고유 FFT 해상도 크기만큼의 원시 부동소수점 데이터가 담긴 독립 전송 패킷.
*   **`ST_PktCalibration_t`**: Welch's PSD 캘리브레이션 시 분석 곡선 및 FIR 계수를 웹 인터페이스에 전송하는 패킷.
*   **`ST_PktSequence_t`**: AI 추론을 위한 시계열 프레임 데이터 패킷.

---

## 6. 오프셋 정렬 무결성 검증 (Static Assertions)

`T215_Type_250.hpp`는 실시간 정렬 충돌로 인한 덤프를 방지하기 위해 빌드 시점에 아래 코드를 강제 실행하여 검증합니다.

```cpp
static_assert(offsetof(ST_PktTelemetry_t, accel_band_energy) % 16 == 0, "Unaligned tensor offset!");
static_assert(offsetof(ST_PktTelemetry_t, gyro_rms_energy) % 16 == 0, "Unaligned tensor offset!");
static_assert(offsetof(ST_PktTelemetry_t, audio_timbre_bands) % 16 == 0, "Unaligned tensor offset!");
static_assert(offsetof(ST_PktTelemetry_t, audio_mfcc) % 16 == 0, "Unaligned tensor offset!");
static_assert(sizeof(ST_PktTelemetry_t) % 16 == 0, "Unaligned structure size!");
```
