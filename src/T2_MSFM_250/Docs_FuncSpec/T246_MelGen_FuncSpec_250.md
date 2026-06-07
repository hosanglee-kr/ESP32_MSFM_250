# [T246_MelGen] Mel 주파수 필터뱅크 생성 엔진

본 문서는 **MSFM_T2_v250** 임베디드 펌웨어에서 소음(Audio) 및 진동(IMU) 특징량 추출 시, 주파수 도메인(Hz)의 스펙트럼 강도를 인간의 청각 및 물리적 구조 인지 특성에 맞춘 Mel 스케일 특징으로 매핑해주는 `MelFilterbankGenerator` 클래스의 기능 규격서입니다.

---

## 1. 개요 및 설계 원칙

1.  **동적 주파수 대역 보간**:
    *   오디오 도메인과 진동(가속도/자이로) 도메인의 Nyquist 주파수가 완전히 다르기 때문에(오디오: 21kHz, IMU: 800Hz), 각각의 특징에 맞추어 보간 주파수 경계를 다르게 적용하여 최적화된 Mel-scale 특징 융합을 수행합니다.
2.  **행렬의 평탄화(Flatten)와 패딩 정렬**:
    *   2차원의 `[FFT 빈 x Mel 밴드]` 삼각 가중치 행렬을 부동소수점 1차원 플랫 어레이(`p_melBankFlat`)로 평탄화하여 메모리 오버헤드를 줄입니다.
    *   FPU/SIMD 고속 벡터 곱셈 루프 연산 시 정렬 에러가 나지 않도록 열(Column) 방향에 패딩 정렬을 보장합니다.

---

## 2. Mel 스케일 수식 모델 (Mathematical Model)

주파수 $f$ (Hz)와 Mel 스케일 간의 상호 변환 공식은 다음과 같으며, C++ 내에서는 `log1p` 및 `expm1` 고속 수학 라이브러리를 사용하여 연산 오차와 언더플로우를 방지합니다.

*   **주파수(Hz) $\rightarrow$ Mel 변환**:
    $$Mel = 1127.0 \times \ln\left(1 + \frac{f}{700.0}\right)$$
    *   *C++ 코드 적용*: `1127.0f * log1p(freq / 700.0f)`
*   **Mel $\rightarrow$ 주파수(Hz) 역변환**:
    $$f = 700.0 \times \left(e^{\frac{Mel}{1127.0}} - 1\right)$$
    *   *C++ 코드 적용*: `700.0f * (expm1(mel / 1127.0f))`

---

## 3. 핵심 API 및 삼각 필터 보간 알고리즘 (Algorithms)

### 3.1 generateAudioMelFilterbank (오디오 Mel 필터 가중치 행렬 생성)
*   **시그니처**:
    ```cpp
    static void generateAudioMelFilterbank(float* p_melBankFlat, float sampleRate, uint8_t activeMelBands, uint16_t binsMax, uint16_t melPadded, uint16_t melBandsMax)
    ```
*   **동작 파라미터**:
    *   `v_lowFreq`: 100.0Hz (오디오의 저주파 노이즈 영역 제거 하한)
    *   `v_highFreq`: $8000.0$Hz 와 $Nyquist \times 0.95$ 중 최소값 (안티에일리어싱 영역 제거 상한)
*   **삼각 필터 보간 메커니즘**:
    1.  최대/최소 주파수 경계를 Mel 값(`v_lowMel`, `v_highMel`)으로 변환합니다.
    2.  Mel 축상에서 활성 밴드 수(`activeMelBands`)에 맞추어 균등한 간격으로 각 삼각 필터의 **좌측 경계($v\_hzL$)**, **중심($v\_hzC$)**, **우측 경계($v\_hzR$)** 주파수 포인트를 Hz 단위로 계산 및 역산출합니다.
    3.  각 FFT 주파수 빈(b)에 대응하는 물리적 주파수 $v\_hzB = b \times \frac{Nyquist}{binsMax}$ 를 기반으로 삼각 필터 내 가중치를 계산합니다.
        *   **상승 구간** ($v\_hzL \le v\_hzB \le v\_hzC$):
            $$weight = \frac{v\_hzB - v\_hzL}{v\_hzC - v\_hzL}$$
        *   **하강 구간** ($v\_hzC < v\_hzB \le v\_hzR$):
            $$weight = \frac{v\_hzR - v\_hzB}{v\_hzR - v\_hzC}$$
        *   **그 외 구간**: $weight = 0.0$

### 3.2 generateImuMelFilterbank (IMU Mel 필터 가중치 행렬 생성)
*   **시그니처**:
    ```cpp
    static void generateImuMelFilterbank(float* p_melBankIMU, uint16_t binsImuMax, uint16_t melPadded, uint16_t melBandsMax)
    ```
*   **동작 파라미터**:
    *   `v_lowFreq_imu`: 10.0Hz (진동의 저주파 오프셋/중력 성분 배제 하한)
    *   `v_highFreq_imu`: 800.0Hz (1600Hz ODR 기준 Nyquist 주파수로 고정 적용)
*   **삼각 필터 보간 메커니즘**:
    *   오디오와 동일한 삼각 필터 상승/하강 보간 메커니즘을 따르되, 나이퀴스트 고정값 800Hz를 분모로 사용하여 10Hz ~ 800Hz 사이 영역에서 삼각 필터의 선형 보간 계수를 생성합니다.

---

## 4. 평탄화 1D 배열 매핑 및 정렬 배치 구조

2차원의 필터 가중치 행렬을 1차원 부동소수점 배열에 저장할 때는 고속 인덱싱 계산을 수행합니다.

*   **1D 메모리 인덱스 수식**:
    $$\text{Index} = b \times melPadded + m$$
    *   $b$: 현재 FFT 빈 인덱스 (`0 ~ binsMax - 1`)
    *   $m$: 현재 Mel 필터 밴드 인덱스 (`0 ~ activeMelBands - 1`)
    *   `melPadded`: 16바이트 정렬을 위해 밴드 크기 뒤에 여유 공간(Padding)을 둔 물리적 한계 인자.

이와 같은 플랫 배치 구조를 통해, 특징 추출 엔진(`T245_FeatExtra`)은 대용량 가중치 곱셈 연산 시 `for` 루프 내에서 간결하게 포인터 연산으로 삼각 필터 밴드별 적합 가중치를 참조하여 Mel 스펙트럼 에너지 특징으로의 압축 처리를 단 몇 마이크로초 이내에 완료할 수 있습니다.
