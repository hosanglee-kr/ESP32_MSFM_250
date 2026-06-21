# [MSFM_T2_250] T248_SeqBuild 기능 규격서

본 문서는 **MSFM_T2_250** 임베디드 펌웨어의 AI 추론용 입력 데이터 정합 파이프라인 모듈인 `T248_SeqBuild` (Sequence Builder & Time Aligner)에 대한 기능 규격서입니다.

---

## 1. 모듈 개요

`T248_SeqBuild` 모듈은 추출된 다채널 특징 벡터 스트림을 시계열 신경망 모델(예: LSTM, GRU 등)의 입력 차원 규격에 맞게 2차원 평탄화(Flatten 1D Array)하여 조립하는 핵심적인 머신러닝 전처리 엔진입니다. 
서로 다른 물리 주기와 시간 왜곡(Skew)이 발생할 수 있는 진동 신호(가속도/자이로)와 소음 신호의 동기화를 수행하는 **MultiRateTimeAligner** 및 채널 스위치 온오프에 따라 텐서 오프셋을 조절해 주는 **DynamicTensorBinder**를 핵심 컴포넌트로 내장하고 있습니다.

---

## 2. API 명세 (인터페이스 선언)

[T248_SeqBuild_250.hpp](../T248_SeqBuild_250.hpp) 클래스들의 공개 API 규격은 다음과 같습니다.

### A. `CL_T2_SequenceBuilder` 클래스
*   **역할**: 시계열 특징량의 프레임 이력을 순환 저장(Ring Buffer)하고 모델 추론 시점에 맞춰 플랫(1D Float) 형태로 정렬 조립해 추출해 줍니다.

#### `bool init(uint16_t p_sequenceFrames, uint16_t p_featureDim)`
*   **기능설명**: 요구 시퀀스 길이(기본 16)와 차원수(예: 312)를 기반으로 SIMD 가속 정렬 오프셋을 맞추어 PSRAM 내에 평탄 버퍼(`_dataFlat`)를 16바이트 경계 정렬 할당(`heap_caps_aligned_alloc`)하여 초기화합니다.
*   **재기동 누수 방지**: 핫스왑 재기동 및 설정 변경 시 메모리 누수를 막기 위해, 이전 버퍼 존재 유무를 확인하여 사전 할당 해제(`heap_caps_free`)하는 가드를 보유하고 있습니다.
*   **반환값**: 메모리 할당 및 버퍼 구성 성공 여부.

#### `~CL_T2_SequenceBuilder()` (소멸자)
*   **기능설명**: 객체 소멸 및 할당 해제 시 `heap_caps_free(_dataFlat)`를 호출하여 힙 단편화와 메모리 누수를 완벽 차단합니다.

#### `void pushVector(const float* p_vector)`
*   **기능설명**: 새로운 1프레임 특징량 벡터를 링버퍼 구조의 현재 쓰기 헤더(`_head`)에 복사 적재하고 인덱스를 이동합니다. 버퍼 수량이 꽉 차면 `_isFull`을 참으로 마크합니다.

#### `void getSequenceFlat(float* p_outBuffer, size_t p_maxOutSize) const`
*   **기능설명**: 현재 쓰기 헤드부터 가장 오래된 프레임부터 최신 프레임 순으로 시간 순서대로 정렬하여 평탄하게 배열(`p_outBuffer`)에 정렬 복사합니다.

---

### B. `MultiRateTimeAligner` 클래스
*   **역할**: 샘플링 레이트 비대칭에 따른 시간 왜곡을 극복하기 위해, 최근 진동 스냅샷 정보를 보존하여 오디오 이벤트 타임스탬프와 시간 정합이 맞물리도록 매핑합니다.

#### `MultiRateTimeAligner(uint64_t max_skew_us)`
*   **기능설명**: 허용 가능한 타임스탬프 스큐(Skew) 임계치 한계를 마이크로초 단위로 설정해 인스턴스를 생성합니다.

#### `void notifyClockSourceSwitch()`
*   **기능설명**: 단조 시계 백업 스위칭 발생 시 과도기 위상 급변을 차단하기 위해 이전/현재 타임스탬프를 0으로 리셋하고 `_mutingRemainingTicks = 25` (약 500ms에 해당)로 설정하여 드롭 제어를 일시 동결합니다.

#### `void injectNewVibSample(const float* raw_feats, uint64_t ts)`
*   **기능설명**: 진동 처리 태스크에서 획득된 신규 3축 가속도/자이로 특징량 스냅샷 정보와 타임스탬프(`ts`)를 수집 주입하여 버퍼를 갱신합니다.

#### `bool getAlignedVibration(uint64_t audio_ts, float* out_feats)`
*   **기능설명**: 인가받은 오디오 타임스탬프 `audio_ts` 기준과 직전 수입된 진동 데이터의 타임스탬프 스큐를 비교 분석하여 최적 매핑된 진동 특징 벡터 데이터를 `out_feats`에 복사해 줍니다.
*   **동기화 정합 세부 정책**:
    *   **Muting 가드**: 단조 시계 백업 스위칭 발생 시 `_mutingRemainingTicks`가 0이 될 때까지 드롭 결정을 일시 동결하고 최신 특징량을 복사 전달하여 과도기 위상 급변을 완화합니다.
    *   오디오 타임스탬프가 진동 스냅샷 사이(prev ~ curr)에 존재할 경우 선형 보간하여 정합합니다.
    *   외삽(Extrapolation) 방지를 위해 보간 계수 `alpha`를 `0.0f ~ 1.0f` 범위로 클램핑합니다.
    *   시간축 역전이나 비정상 상태(감산 언더플로우 가능성) 발생 시 선형 보간을 패스하고 최신 스냅샷 값을 그대로 유지하는 **ZOH(Zero Order Hold) 폴백** 방식을 수행합니다.
    *   스큐 허용 범위(`_max_allowed_skew_us`)를 초과하는 과도한 클럭 드리프트 감지 시 강제 Drop & Shift하여 시간축을 정렬 상태로 복원합니다.
*   **반환값**: 스큐 허용치 조건 충족으로 시간 축 정합에 성공했는지 여부.

---

### C. `DynamicTensorBinder` 클래스
*   **역할**: 런타임에 어떤 물리 센서 채널이 켜지고 꺼졌는지 비트 마스크 지시어를 확인하여, 신경망 모델 입력 레이어로 전송할 단일 플랫 텐서 내 각 채널의 시작 인덱스(Offset)를 유동적으로 조정 공급합니다.

#### `DynamicTensorBinder(size_t nn_input_size)`
*   **기능설명**: 신경망 모델의 최종 1D 인풋 텐서 전체 하드웨어 영역 크기를 지정해 바인딩 관리자를 구동합니다.

#### `void updateTensorOffsets(uint16_t active_mask_flags, size_t audio_feat_cnt, size_t vib_feat_cnt)`
*   **기능설명**: 현재 활성화된 센서 채널 플래그 마스크에 기초하여 텐서 빌드 시 각각 오디오 및 진동 데이터의 시작 복사 위치(오프셋)를 산출 저장합니다.

#### `bool buildFlattenTensor(float* p_target_tensor, const float* audio_src, const float* vib_src, uint16_t mask)`
*   **기능설명**: 각 도메인의 특징 벡터 데이터를 사전에 연산한 오프셋 주소 공간으로 정확히 평탄 복사하여 신경망 입력 텐서(`p_target_tensor`)를 원자적 조립해 줍니다. 이때 템플릿 Traits 정책 마스킹에 의해 연산 생략된 영역(가속도/자이로 MFCC 등)이나 마스크가 오프된 비활성 도메인은 모델의 입력 구조 정합성을 위해 전체를 **`0.0f`로 강제 안전 패딩** 처리합니다.
*   **반환값**: 입력 바운더리 크기 규격 충족 여부 및 조립 완성 성공 여부.

---

## 3. 변경 및 갱신 이력 (Revision History)

*   **v2.50 (2026-06-21)**:
    *   `MultiRateTimeAligner` 내 단조 시계 백업 스위칭 시 `notifyClockSourceSwitch` 및 25틱 `_mutingRemainingTicks` 과도기 위상 Muting 제어 명세 반영.
    *   `CL_T2_SequenceBuilder` PSRAM 16바이트 정렬 할당 및 소멸자 `heap_caps_free` 해제 정책 명기.
