# [MSFM_T2_250] T245_FeatExtra 기능 규격서

본 문서는 **MSFM_T2_250** 임베디드 펌웨어의 멀티모달(진동/소음) 융합 특징 추출엔진인 `T245_FeatExtra` (Feature Extractor) 모듈에 대한 기능 규격서입니다.

---

## 1. 모듈 개요

`T245_FeatExtra` 모듈은 전처리(DSP) 과정을 거친 파형(Waveform) 데이터로부터 시간 영역(Time Domain) 통계 지표와 주파수 영역(Frequency Domain) 스펙트럼 지표, 그리고 음향 기계 학습 분야에서 널리 쓰이는 **MFCC (Mel-Frequency Cepstral Coefficients)** 다차원 벡터를 실시간 추출하는 수학적 코어 엔진입니다.

---

## 2. API 명세 (인터페이스 선언)

[T245_FeatExtra_250.hpp](../T245_FeatExtra_250.hpp) 클래스의 공개 API 규격은 다음과 같습니다.

### `CL_T2_FeatureExtractor(void)`
*   **기능설명**: 생성자로, 모든 PSRAM 버퍼 포인터들을 `nullptr`로 초기화하고 8채널 특징 이력 상태 인덱스를 초기화합니다.

### `bool init(const T2_Type::ST_Audio_Config_t& audioCfg)`
*   **기능설명**: 오디오/가속도/자이로 고속 변환 연산을 위한 FFT Complex Work 버퍼, Mel 필터뱅크 튜닝 평탄 행렬, DCT(이산 코사인 변환) 커널 계수 행렬 및 8개 채널의 MFCC 히스토리 저장 버퍼를 힙(PSRAM)에 할당합니다.
*   **반환값**: 메모리 버퍼 생성 완료 및 초기 DCT 커널 연산 통과 여부 (`true` / `false`).

### `void extractAccel(const float* p_inX, const float* p_inY, const float* p_inZ, uint32_t p_len, uint32_t p_sampleRate, T2_Type::ST_FeatureSlot_Vib_t& p_vibSlot, const T2_Type::ST_Accel_Config_t& p_accCfg)`
*   **기능설명**: 3축 가속도 신호로부터 RMS, Crest Factor, Skewness, Kurtosis, 대역별 에너지와 푸리에 파워 스펙트럼 및 진동 특징 벡터들을 연쇄 계산하여 진동 지표 슬롯에 저장합니다. (가속도 정책인 `AccelPolicy`에 의해 가속도 MFCC 계산은 생략되고 `0.0f`로 우회 마스킹됩니다.)

### `void extractGyro(const float* p_inX, const float* p_inY, const float* p_inZ, uint32_t p_len, uint32_t p_sampleRate, T2_Type::ST_FeatureSlot_Vib_t& p_vibSlot, const T2_Type::ST_Gyro_Config_t& p_gyrCfg)`
*   **기능설명**: 자이로 3축 원시 각속도 신호를 1차 차분(각가속도) 도메인으로 변환한 뒤의 단구간 RMS 에너지 및 왜도, 첨도, 대역별 에너지와 드리프트 추정값(자이로 특화)을 산출하여 슬롯을 채웁니다. (위상 왜곡을 초래하는 Zero-Crossing Rate 연산은 전면 폐기되었으며, `GyroPolicy`에 의해 자이로 MFCC 연산은 `0.0f` 마스킹 처리됩니다.)

### `void extractAudio(const float* p_audL, const float* p_audR, uint32_t p_len, uint32_t p_sampleRate, T2_Type::ST_FeatureSlot_Aud_t& p_audSlot, const T2_Type::ST_Audio_Config_t& p_audCfg)`
*   **기능설명**: 좌우 마이크 음원 파형 데이터에 대해 단시간 푸리에 변환(STFT)을 수행하고, 파워 스펙트럼, 켑스트럼 피크 오차 비율 및 2채널 오디오 MFCC와 델타/더블델타 시계열 계수를 연산하고, 32개 1/3 옥타브 밴드 상대 에너지 비율 벡터(Timbre 지표)를 산출해 내어 MFCC와 융합합니다. (다중 반사 왜곡을 일으키던 공간 위상 지표 Coherence 및 IPD는 전면 삭제되었습니다.)

### `void setNoiseLearning(bool p_enable)` / `void resetNoiseProfile(void)`
*   **기능설명**: 배경 소음 감산을 위한 노이즈 스펙트럼 적응형 학습의 기동 및 메모리 리셋을 제어합니다.

### `void reloadAudioMelFilter(float sampleRate)`
*   **기능설명**: 변경된 주파수 샘플율에 맞춰 내부 오디오 Mel 필터 계수 평탄화 행렬을 재구성합니다.

### `void resetHistory(void)`
*   **기능설명**: 델타(변위 속도), 더블-델타(변위 가속도) 연산용 MFCC 히스토리 링버퍼 데이터를 제로 클리어합니다.

---

## 3. 핵심 수학적 연산 수식 및 규격

1.  **시간 영역(Time Domain) 통계 추출**:
    *   **평균 제곱근 (RMS)**:
        $$\text{RMS} = \sqrt{\frac{1}{N}\sum_{n=0}^{N-1} x[n]^2}$$
    *   **첨도 (Kurtosis)**:
        $$\text{Kurtosis} = \frac{\frac{1}{N}\sum (x[n] - \mu)^4}{\sigma^4}$$
    *   **왜도 (Skewness)**:
        $$\text{Skewness} = \frac{\frac{1}{N}\sum (x[n] - \mu)^3}{\sigma^3}$$
    *   **파고율 (Crest Factor)**:
        $$\text{Crest Factor} = \frac{\max(|x[n]|)}{\text{RMS}}$$

2.  **주파수 영역(Frequency Domain) MFCC 추출**:
    *   **Mel 필터 가중치 변환**:
        $$\text{Mel}(f) = 2595 \cdot \log_{10}\left(1 + \frac{f}{700}\right)$$
    *   Sinc 필터 응답을 주파수 도메인 상의 삼각형 밴드 패스 응답(총 26개 밴드 기본)과 컨볼루션하여 각 Mel 대역 에너지를 산출합니다.
    *   **이산 코사인 변환 (DCT-II)**:
        $$X_{\text{MFCC}}[k] = \sum_{m=0}^{M-1} \log(S_{\text{Mel}}[m]) \cdot \cos\left(\frac{\pi k (2m+1)}{2M}\right)$$
    *   산출된 13차 계수에 대해 시계열 링버퍼 이력을 추적하여 1차 차분(Delta) 및 2차 차분(Double-Delta)을 산출해 총 `39차원` (13*3) 특징량을 8채널 통합 신경망 인풋 텐서 규격에 공급합니다.

3.  **템플릿 Traits 기반 연산 마스킹 규칙 (FPU 부하 최적화)**:
    인터페이스의 단일성을 유지하면서 도메인별 불필요한 FPU 연산(예: 진동 신호의 MFCC 연산 등)을 전면 생략하고 마스킹하기 위해 정책 Traits를 도입합니다.
    *   **가속도 정책 (`AccelPolicy`)**:
        - 16밴드 에너지만 연산 (`ENABLE_BAND_ENERGY = true`)
        - MFCC 연산 생략 및 `0.0f`로 우회 마스킹 (`ENABLE_MFCC = false`)
    *   **자이로 정책 (`GyroPolicy`)**:
        - 하위 2개 저주파 밴드만 누적 연산 (`ENABLE_BAND_ENERGY = true`, `BAND_COUNT = 2`)
        - MFCC 연산 생략 및 `0.0f` 마스킹 (`ENABLE_MFCC = false`)
    *   **오디오 정책 (`AudioPolicy`)**:
        - MFCC 연산 활성화 (`ENABLE_MFCC = true`)
        - 1/3 옥타브 밴드 상대 에너지(Timbre) 융합 활성화 (`ENABLE_TIMBRE = true`, `BAND_COUNT = 32`)
