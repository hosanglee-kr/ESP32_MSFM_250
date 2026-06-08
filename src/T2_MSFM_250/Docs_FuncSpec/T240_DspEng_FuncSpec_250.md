# [MSFM_T2_250] T240_DspEng 기능 규격서

본 문서는 **MSFM_T2_250** 임베디드 펌웨어의 고속 신호처리 코어인 `T240_DspEng` (DSP Engine) 모듈에 대한 기능 규격서입니다.

---

## 1. 모듈 개요

`T240_DspEng` 모듈은 수집된 아날로그 원시 전압/가속도 정보에서 물리 노이즈를 제어하기 위한 고속 선형/비선형 필터들의 계산 집합체입니다. ESP32-S3의 하드웨어 FPU 및 **ESP-DSP (SIMD 벡터 가속)** 라이브러리를 직접 호출하여 연산 지연(Latency)을 최소화하도록 설계되었습니다.

---

## 2. API 명세 (인터페이스 선언)

[T240_DspEng_250.hpp](../T240_DspEng_250.hpp) 클래스의 주요 공개 API 및 구조 정보는 다음과 같습니다.

### `CL_T2_DspEngine(void)`
*   **기능설명**: 생성자로, 초기 상태 플래그를 미초기화 상태(`false`)로 세팅하고 런타임 메모리 포인터들을 `nullptr`로 초기화합니다.

### `bool init(const T2_Type::ST_DynamicConfig_t& p_cfg)`
*   **기능설명**: 설정 구조체 정보를 입력받아, 오디오/가속도/자이로 분석용 내부 런타임 구조체(`ST_AudioDspRuntime`, `ST_AccelDspRuntime`, `ST_GyroDspRuntime`) 및 연산 가공용 임시 중간 버퍼들을 힙 영역(PSRAM)에 동적으로 할당하고 초기화합니다.
*   **반환값**: 메모리 할당 및 초기 필터 계수 산출 성공 여부.

### `void reloadFilters(const T2_Type::ST_DynamicConfig_t& p_cfg)`
*   **기능설명**: 런타임 동작 중에 설정 임계치가 변경(예: Notch 주파수, HPF/LPF 차단 주파수 변경)되었을 때 호출됩니다. 새로운 컷오프 정보를 바탕으로 IIR Biquad 및 FIR 계수를 즉시 재연산하여 메모리에 업데이트합니다.

### `void resetStates(void)`
*   **기능설명**: 모든 필터 채널(X, Y, Z, L, R)에 대한 과거 상태 딜레이 라인(Delay Stage) 이력 버퍼 값을 전부 `0.0f`로 완전 소거하여 이전 연산 이력의 간섭을 차단합니다.

### `void processAudio(const float* p_audL, const float* p_audR, float* p_outL, float* p_outR, uint32_t p_len, const T2_Type::ST_Audio_Config_t& p_audCfg)`
*   **기능설명**: 좌우 스테레오 원시 입력 파형에 대해 아래의 전처리 파이프라인을 연속 수행합니다.
*   **처리 파이프라인 시퀀스**:
    1.  **DC Offset 제거**: 신호 평균값 차감을 통한 마이크 DC 편차 제어.
    2.  **Noise Gate**: `pre_alpha` 임계치 미만 무음 묵음 처리.
    3.  **Pre-Emphasis (고주파 강조)**: $y[n] = x[n] - \alpha \cdot x[n-1]$ 필터 적용 (MFCC 성능 향상 유도).
    4.  **Notch 필터**: 60Hz/120Hz 고조파 전원 노이즈 제거 (ESP-DSP Biquad SIMD).
    5.  **대역 통과 필터**: 설정에 맞춰 IIR 또는 FIR HPF/LPF 연산 순차 적용.

### `void processAccel(const float* p_inX, const float* p_inY, const float* p_inZ, float* p_outX, float* p_outY, float* p_outZ, uint32_t p_len, const T2_Type::ST_Accel_Config_t& p_accCfg)`
*   **기능설명**: 가속도 3축 센서 수집 신호에 대해 아래 전처리 파이프라인을 수행합니다.
*   **처리 파이프라인 시퀀스**:
    1.  **적응형 2단계 메디안 필터링**: 각 원시 샘플의 절대값이 센서의 하드웨어 풀스케일 감도(`p_accCfg.range`)의 95%를 초과할 때만 국부적 메디안 처리를 수행하고, 일반 진폭(20~80%) 구간의 물리적 충격파 성분은 Bypass하도록 설계하여 첨도 오염 방지와 물리적 과도 신호 보존을 동시에 달성합니다.
    2.  **DC 제거**: 중력 가속도 및 고정 바이어스 편차 소거.
    3.  **가속도 힐버트 변환 & 군지연 보정**: 가속도 원시 신호에 SIMD 가속 힐버트 변환을 가해 진폭 포락선을 안정적으로 추출하고 이 기반 하에서 차기 지표(STA/LTA 등)를 산출합니다. `esp-dsp` 버전 매크로 검증을 통해 컴파일 타임 `static_assert`로 내부 필터 차수에 따른 군지연 보정 상수의 무결성을 검증합니다.
    4.  **Notch 필터 및 대역 필터**: 60Hz/120Hz Notch 및 IIR/FIR 밴드 필터링 적용.

### `void processGyro(const float* p_inX, const float* p_inY, const float* p_inZ, float* p_outX, float* p_outY, float* p_outZ, uint32_t p_len, const T2_Type::ST_Gyro_Config_t& p_gyrCfg)`
*   **기능설명**: 자이로 3축 센서 수집 신호에 대해 1차 차분 신호 전처리 및 IIR/FIR 밴드 필터링을 수행합니다.

---

## 2.2 PSRAM 및 캐시 일관성 (Cache Coherency) 설계

대형 FFT 누적 윈도우 버퍼를 외부 PSRAM에 배치하고 GDMA를 이용해 내부 SRAM 임시 버퍼로 고속 전송할 때, L1 캐시 불일치로 인한 데이터 오염을 방지하기 위해 ESP32-S3 저수준 ROM 캐시 API 헤더 `<esp32s3/rom/cache.h>`를 이용하여 캐시 제어를 수행합니다.

- **FFT 연산 개시 전**: CPU L1 캐시를 강제 무효화하여 이전 수집 주기의 낡은 데이터가 잔류하는 것을 방지합니다.
  ```cpp
  Cache_Invalidate_Addr((uint32_t)_sramFftPing, size);
  ```
- **FFT 연산 완료 후**: 물리 메모리와 캐시 정합성을 동기화하기 위해 Dirty 플래시 플래그에 기인한 Write-back 캐시 플러시를 즉시 실행합니다.
  ```cpp
  Cache_WriteBack_Addr((uint32_t)_sramFftPing, size);
  ```

---

## 3. 핵심 필터 연산 규격

1.  **Notch 필터 계수 산출 (Biquad IIR)**:
    아날로그 전원 노이즈 제거를 위한 Biquad 전달함수 $H(s)$의 계수를 산출합니다.
    *   $\omega_0 = \frac{2\pi \cdot f_{\text{notch}}}{f_s}$
    *   $\alpha = \frac{\sin(\omega_0)}{2Q}$
    *   계수 구성:
        *   $b_0 = 1, \quad b_1 = -2\cos(\omega_0), \quad b_2 = 1$
        *   $a_0 = 1 + \alpha, \quad a_1 = -2\cos(\omega_0), \quad a_2 = 1 - \alpha$
    *   계수 정규화: $a_0$로 모든 $a, b$ 계수를 나누어 최종 Biquad 계수 집합을 생성하고 `dsps_biquad_f32_ae32` SIMD 어셈블리 함수로 고속 수행합니다.

2.  **FIR 필터 계수 산출 (Windowed Sinc)**:
    차단 주파수 $f_c$에 대해 아래식으로 LPF Sinc 함수 계수를 구하고 Hann Window를 취합니다.
    *   Sinc 함수 계수 ($n = -\frac{N-1}{2} \dots \frac{N-1}{2}$):
        $$h[n] = \frac{\sin(2\pi \cdot f_c \cdot n / f_s)}{\pi \cdot n}$$
    *   $h[0] = \frac{2f_c}{f_s}$ 로 정의한 후 Hann Window $w[n]$을 곱해 계수 차단 마진을 정립합니다.
