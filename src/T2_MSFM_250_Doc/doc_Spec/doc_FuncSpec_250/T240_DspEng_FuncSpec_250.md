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
*   **기능설명**: 설정 구조체 정보를 입력받아, 오디오/가속도/자이로 분석용 내부 런타임 구조체 및 연산 가공용 임시 중간 버퍼들을 힙 영역(PSRAM)에 동적으로 할당하고 초기화합니다.
*   **반환값**: 메모리 할당 및 초기 필터 계수 산출 성공 여부.

### `void reloadFilters(const T2_Type::ST_DynamicConfig_t& p_cfg)`
*   **기능설명**: 런타임 동작 중에 설정 임계치가 변경(예: Notch 주파수, HPF/LPF 차단 주파수 변경)되었을 때 호출됩니다. 새로운 컷오프 정보를 바탕으로 IIR Biquad 및 FIR 계수를 즉시 재연산하여 메모리에 업데이트합니다.

### `void recalculateFilters(const T2_Type::ST_DynamicConfig_t& p_cfg)`
*   **기능설명**: 런타임 설정 주파수(Notch, HPF 등) 변경 즉시 DSP 필터 런타임 계수를 재계산 및 갱신합니다. 
*   **동적 핫스왑 격리**: 노치 필터 계수 `_notch_coeffs` 등 실시간 주파수 변경이 잦은 영역은 플래시 ROM이 아닌 RAM 영역에 상주시켜 런타임 쓰기 예외 발생을 사전에 방어하고, 컴파일 타임 고정 필터 계수(`s_iir_hpf_coeffs` 등)에만 `SMEA_FLASH_RODATA` 플래시 ROM direct 페치 속성을 한정 적용합니다.

### `void resetStates(void)`
*   **기능설명**: 모든 필터 채널(X, Y, Z, L, R)에 대한 과거 상태 딜레이 라인(Delay Stage) 이력 버퍼 값을 전부 `0.0f`로 완전 소거하여 이전 연산 이력의 간섭을 차단합니다.

### `void prepare_complex_fft_buffer(const float* raw_real_data, float* complex_buffer, size_t n)`
*   **기능설명**: 복소수 FFT 연산 수행 시 발생하는 인덱스 오버플로우 메모리 붕괴(Memory Corruption)를 차단하기 위해 실수 데이터를 복소수 구조에 복사할 때 짝수=실수(Real Part), 홀수=0.0f(Imaginary Part)로 교차 배치하여 `2 * n` 크기를 강제 동기화합니다.

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
*   **기능설명**: 자이로 3축 센서 수집 신호에 대해 1차 차분 신호 전처리 및 경량 IIR 밴드 필터링을 수행합니다.
*   **SRAM 자원 절약 및 FIR 제거**: 자이로의 고차 FIR 상태 버퍼 및 FIR 계수 배열을 완전히 배제하고, 초경량화된 IIR 바이쿼드 필터 구조인 `ST_GyroDspRuntime_t`로 전환하여 SRAM 소비를 방지합니다.

---

## 3. 핵심 필터 연산 및 메모리 규격

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
3.  **자이로 전용 초경량 IIR 구조체 (`ST_GyroDspRuntime_t`)**:
    ```cpp
    struct ST_GyroDspRuntime_t {
        float iir_hpf_state[3][4]; // 3축 x 4차 IIR 바이쿼드 계수/상태 버퍼
        float iir_lpf_state[3][4];
    };
    ```
4.  **필터 계수 ROM direct 페치 최적화 (`SMEA_FLASH_RODATA`)**:
    *   고정 필터 계수 배열은 플래시 ROM 상주(`.rodata` 섹션)를 강제하고 ESP32-S3 SIMD 연산을 위해 16바이트 정렬을 적용하여 힙/SRAM 복사에 따른 자원 낭비를 차단합니다.
    *   `#define SMEA_FLASH_RODATA __attribute__((section(".rodata"), aligned(16)))`

---

## 4. 변경 및 갱신 이력 (Revision History)

*   **v2.50 (2026-06-21)**:
    *   `prepare_complex_fft_buffer` API 및 2*N 복소 버퍼 정렬 무결성 사양 도입.
    *   자이로 `ST_GyroDspRuntime_t` 경량 HPF/LPF IIR 상태 전이로 메모리 최적화 규격 갱신 (고차 FIR 제거).
    *   동적 notch 계수(RAM)와 고정 IIR/FIR 계수(`SMEA_FLASH_RODATA`, ROM) 격리 설계 보완 반영.
