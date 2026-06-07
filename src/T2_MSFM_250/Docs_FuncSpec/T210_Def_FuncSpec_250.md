# [T210_Def] 4-Tier 시스템 상수 정의 및 전역 파라미터 SSOT

본 문서는 **MSFM_T2_v250** 시스템의 전역 상수, 하드웨어 핀 맵, RTOS 태스크 바인딩 속성 및 컴파일 타임 계산식을 집중 기술한 `T210_Def_250.hpp` 모듈의 기능 규격서입니다. 본 모듈은 시스템 전체의 **Single Source of Truth (SSOT)** 역할을 수행합니다.

---

## 1. 구현 원칙 및 설계 근거 (Architecture Guide)

1.  **도메인 완벽 격리 (Domain Isolation)**:
    *   가속도(Accel), 자이로(Gyro), 소음(Audio) 데이터의 고유 샘플링 주율(Hz) 및 FFT 윈도우 크기를 독립 상수로 완전 분리하여, 상호 영향 없이 개별 튜닝이 가능하도록 설계되었습니다.
2.  **IMU 공통 인프라 통합**:
    *   BMI270 센서의 통신(SPI), FIFO 제어 등 하드웨어 의존성이 높은 상수들은 `Imu` 네임스페이스로 통합하여 하드웨어 제어 가독성을 높였습니다.
3.  **개별 필터 사양 할당**:
    *   각 도메인 및 채널별로 특화된 FIR 차수, 대역 수(Bands), 임계값을 별도 상수로 정의하여 공통 Shared 모듈 오작동을 근본적으로 제거했습니다.
4.  **AI 텐서 차원 동적화**:
    *   전체 물리 채널 수(가속도 3축 + 자이로 3축 + 오디오 2채널 = 8채널)를 바탕으로 켑스트럼 특징 텐서 차원(`MFCC_DIM_DEF = 13 * 3 * 8 = 312`)을 컴파일 타임에 자동 계산하도록 수식화하였습니다.
5.  **매직넘버 제로 (Magic Number Zero)**:
    *   코드 내의 하드코딩을 완벽히 금지하며, 디폴트 값 및 한계값은 오직 [T210_Def_250.hpp](../T210_Def_250.hpp) 파일만을 단일 통제소로 삼습니다.

---

## 2. 하드웨어 의존 및 컴파일 타임 매크로 (Compiler Directives)

*   **`SMEA_FLASH_RODATA`**: 필터 계수 및 윈도우 계수 테이블 등 대용량 상수를 Flash ROM 영역(`.rodata` 섹션)에 할당하여 내부 SRAM 소모를 차단합니다.
*   **`SMEA_SRAM_ATTR`**: FSM 큐 및 실시간 상태 레지스터 캐시 등을 내부 Fast SRAM(`DRAM_ATTR`)에 상주시킵니다.
*   **`SMEA_ALIGN_16`**: ESP32-S3 FPU 가속 명령(SIMD)이 16바이트 정렬된 메모리 어레이에서 지연 없이 수행될 수 있도록 강제합니다.
*   **FPU 예외 차단 매크로**:
    *   `SMEA_IS_NAN(v)`, `SMEA_IS_INF(v)`를 사용하여 하드웨어 벡터 연산 중 발생할 수 있는 `NaN`(Not a Number) 및 `Infinity`를 조기에 검출합니다.
    *   `SMEA_SAN_FLOAT(v)`: 비정상 데이터 검출 시 `0.0f`로 변환하여 시스템 연쇄 붕괴를 예방합니다.
    *   `SMEA_SAFE_DIV(num, denom, eps)`: 분모가 최소 한계치 `eps` 미만인 경우 강제로 분모를 `eps`로 상향하여 0 나누기 오류를 방지합니다.

---

## 3. 세부 네임스페이스별 명세 (Namespace Specs)

### 3.1 Global (전역 디바이스 및 통신 인프라)
*   **System**:
    *   `VERSION_STR` = `"T240_v250"`
    *   `SEQUENCE_FRAMES_MAX` = 32, `SEQUENCE_FRAMES_DEF` = 16 (시계열 텐서 결합을 위한 최대/기본 프레임)
    *   `MATH_EPSILON_CONST` = `1e-6f` (0 나누기 임계치)
*   **Hardware (GPIO 바인딩)**:
    *   물리 버튼: GPIO 0, RGB LED: GPIO 21
    *   SD 카드 SPI 핀: CLK (GPIO 39), CMD (GPIO 38), D0 (GPIO 40)
    *   프리엠프티브 릴레이: GPIO 25 (안전 차단 인터록 핀)
*   **Task (FreeRTOS 바인딩)**:
    *   `STORAGE_STACK_DEF` = 8192 Bytes, 우선순위 = 2
    *   `IMU_ACQ_STACK_SIZE` = 4096 Bytes, 우선순위 = 12 (Core 0 고정 바인딩)
    *   `AUD_PROC_STACK_SIZE` = 8192 Bytes, 우선순위 = 6 (Core 1 고정 바인딩)
    *   `VIB_PROC_STACK_SIZE` = 8192 Bytes, 우선순위 = 5 (Core 1 고정 바인딩)
    *   `LAZY_WRITE_MS_DEF` = 3000ms (SD카드 Lazy Write 쓰기 주기)
*   **Path**:
    *   설정 파일: `/sys/cfg_250.json` (임시 `/sys/cfg_250.json.tmp`)
    *   인덱스 파일: `/sys/idx_250.json`
*   **NetLimit & Net (Wi-Fi, NTP, MQTT)**:
    *   Wi-Fi SSID/PW 크기 제한 (32/64자), 오토폴백 예비 AP 개수 = 3
    *   NTP 타임존 = `"KST-9"` (한국 표준시), 서버: `pool.ntp.org`, `time.google.com`
    *   MQTT 기본 포트 = 1883, Topic Root = `"smea/T2_MSFM_250"`
*   **Storage**:
    *   `PRE_TRIGGER_SEC_DEF` = 3초 (트리거 감지 전 과거 3초 시점부터 기록)
    *   `ROTATE_MB_DEF` = 10MB (파일당 10MB 도달 시 로테이션)
    *   `ROTATE_KEEP_MAX_DEF` = 8개 (최대 8개 파일만 보존 후 순환 삭제)
*   **Decision (STA/LTA 및 룰 엔진)**:
    *   `STA_LTA_THRESHOLD_DEF` = 3.0f (단기/장기 에너지 비율 임계치)
    *   `MIN_TRIGGER_COUNT_DEF` = 1 (최소 연속 1회 이상 트리거 시 최종 결함 판정)

### 3.2 Imu (BMI270 공통 인프라)
*   `PIN_CS_CONST` = GPIO 10 (SPI CS 핀)
*   `PIN_INT1_WATERMARK_CONST` = GPIO 11 (FIFO Watermark 인터럽트 수신 핀 - Core 0 전용)
*   `PIN_INT2_MOTION_CONST` = GPIO 12 (Any-Motion 인터럽트 수신 핀 - Core 1 및 딥슬립 웨이크업 용)
*   `FIFO_WATERMARK_LIMIT` = 40 (40프레임 축적 시 워터마크 인터럽트 발행 - 약 25ms 간격)
*   `FIFO_FRAME_SIZE_CONST` = 7 Bytes (BMI270 FIFO 프레임 고정 크기)

### 3.3 Accel (가속도 파이프라인)
*   `RATE_DEF` = 1600Hz, `RANGE_DEF` = ±8G (`BMI2_ACC_RANGE_8G`)
*   `FFT_SIZE_DEF` = 1024 (자이로 ODR과 정렬하여 시간축 정합성 1024 유지)
*   `FIR_TAPS_DEF` = 63 (63차 FIR 필터링)
*   `HILBERT_FIR_TAPS` = 31 (힐버트 엔벨로프 추출용 31차 필터, 군지연 보정 15샘플 잠금)
*   **동적 STA/LTA 샘플수 역산**:
    *   `STA_DURATION_MS` = 1ms, `LTA_DURATION_MS` = 10ms
    *   `STA_SAMPLES_DEF` = `(1600 * 1 + 500) / 1000 = 2` 샘플
    *   `LTA_SAMPLES_DEF` = `(1600 * 10 + 500) / 1000 = 16` 샘플

### 3.4 Gyro (자이로 파이프라인)
*   `ENABLE_DEF` = false (기본 비활성화를 통한 전원 및 대역폭 절감)
*   `RATE_DEF` = 1600Hz (가속도 ODR과 1:1 대칭 정합)
*   `FFT_SIZE_DEF` = 1024 (가속도 해상도와 동기화)
*   `FIR_TAPS_DEF` = 31 (자이로 특화 31차 필터링)
*   `STA_SAMPLES_DEF` = 2 샘플, `LTA_SAMPLES_DEF` = 16 샘플 (가속도와 동등 연산)

### 3.5 Audio (소음 파이프라인)
*   `RATE_DEF` = 42000Hz (마이크 기본 샘플링 레이트, APLL 사용 강제)
*   `FFT_SIZE_DEF` = 1024 (오디오 기본 FFT 해상도)
*   `DMA_BUF_LEN_OPT_CONST` = 512, `DMA_BUF_COUNT_OPT_CONST` = 8 (512샘플 핑퐁 버퍼 8개 사용, 언더런 안전 마진 약 97ms 확보)
*   `FIR_TAPS_DEF` = 63 (마이크 하드웨어 보정용 63차 이퀄라이제이션 필터)
*   `MEL_BANDS_DEF` = 26 (켑스트럼 연산용 26개 Mel-Scale filterbank)
*   **동적 STA/LTA 샘플수 역산**:
    *   `STA_DURATION_MS` = 1ms, `LTA_DURATION_MS` = 10ms
    *   `STA_SAMPLES_DEF` = `(42000 * 1 + 500) / 1000 = 42` 샘플
    *   `LTA_SAMPLES_DEF` = `(42000 * 10 + 500) / 1000 = 420` 샘플

### 3.6 AI (시계열 텐서 조합 및 추론)
*   `CHANNELS_MAX` = 8 (가속도 3축 + 자이로 3축 + 오디오 2채널 = 8채널 융합)
*   `MFCC_COEFFS_DEF` = 13 (켑스트럼 13차 특징)
*   `MFCC_COMPONENTS_DEF` = 3 (정적 Static, 1차 변위 Delta, 2차 변위 Delta-Delta)
*   `DELTA_HISTORY_DEF` = 5 (델타 연산 프레임 간격, 타임 갭 `DELTA_GAP_CONST` = 4)
*   `MFCC_DIM_DEF` = 312 (`13 * 3 * 8` 융합 특징 벡터 총합 차원)

---

## 4. 모듈 정책 및 Traits (`T2_General`)

지식 그래프 및 추론 파이프라인에서 FPU/SIMD 연산을 마스킹하거나 활성화하기 위한 Compile-time Policy Traits 구조체입니다.

*   `AccelPolicy`: `enable_mfcc` = false, `enable_fft` = true
*   `GyroPolicy`: `enable_mfcc` = false, `enable_fft` = true
*   `AudioPolicy`: `enable_mfcc` = true, `enable_fft` = true

이를 통해 가속도 및 자이로 도메인에서는 MFCC 특징 추출 연산을 전면 생략하고, 오디오 도메인에서만 MFCC 연산을 전담하도록 보장합니다.

---

## 5. 변경 및 갱신 이력 (Revision History)

*   `v248` -> `v250` 변경 사항:
    *   가속도/자이로의 STA/LTA 샘플 역산식 버그(1샘플 고착 문제)를 해결하기 위해 `(ODR * duration_ms + 500) / 1000` 공식을 전면 도입하고 반올림을 보장했습니다.
    *   프리엠프티브 차단 인터록용 물리 릴레이 GPIO 25 상수를 `Hardware::PIN_SAFETY_RELAY_CONST` 로 새롭게 선언하여 제어 무결성을 확보했습니다.
    *   자이로 ODR을 가속도 ODR과 1:1 대칭 정렬(1600Hz)하고 FFT 크기를 1024로 고정하여 물리적 비대칭성으로 인한 시퀀스 바인딩 오류를 원천 차단했습니다.
