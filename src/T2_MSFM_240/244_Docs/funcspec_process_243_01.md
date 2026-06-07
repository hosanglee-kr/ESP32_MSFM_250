수석 아키텍트의 분석에 따라, T2_MSFM_240 v243 시스템의 부팅부터 데이터 처리 완료까지의 전 과정을 **상세 처리 Step**, **신호 처리 단위**, 그리고 **센서별 특화 로직**으로 나누어 체계적으로 정리해 드립니다.
### **[Step 1] 시스템 부팅 및 리소스 최적화 (Boot & Resource Setup)**
 1. **진입점 및 설정 로드:** T2_init()이 호출되면 ConfigManager가 LittleFS에서 JSON 설정(SSOT)을 로드하여 시스템 전역의 파라미터를 확정합니다.
 2. **메모리 풀(Pool) 할당:**
   * **PSRAM:** 대용량 데이터 전송을 위한 64개 슬롯의 UnifiedFeatureSlot(800B) 및 UnifiedRawChunk 풀을 할당합니다.
   * **Internal SRAM:** 연산 속도가 중요한 신호 처리 핫패스(Hot-path)용 버퍼(_capBufL, _prcBufL 등)를 할당하여 캐시 미스를 방지하고 SIMD 가속 효율을 극대화합니다.
 3. **이중 코어 태스크 기동:** Core 0은 데이터 수집(Capture), Core 1은 데이터 가공(Processing)으로 역할을 분리하여 병렬 처리를 시작합니다.
### **[Step 2] 멀티모달 데이터 수집 (Core 0: Capture Phase)**
Core 0에서는 센서의 물리적 특성에 따라 최적화된 방식으로 데이터를 퍼올립니다.
#### **1. 소음(Audio) 수집 로직 (I2S DMA)**
 * **처리 단위:** Audio::Sensor::FFT_SIZE_MAX (기본 1024 샘플) 단위로 수집합니다.
 * **채널 모드별 동작:**
   * **Stereo:** 좌(L)/우(R) 마이크의 32bit PCM 데이터를 각각 독립된 버퍼에 채웁니다.
   * **Mono L:** 좌측 마이크 데이터만 수집하며, 우측 버퍼는 0으로 채웁니다.
   * **Mono R:** 하드웨어 우측 채널 데이터를 수집하되, 처리 엔진과의 호환성을 위해 소프트웨어 버퍼의 '좌측' 위치로 매핑합니다.
 * **정규화:** 32bit 정수형 PCM 데이터를 PCM_32BIT_SCALE_CONST를 곱해 -1.0 ~ 1.0 사이의 부동소수점(float)으로 변환합니다.
#### **2. 진동(Vibration) 수집 로직 (SPI FIFO)**
 * **처리 단위:** BMI270의 하드웨어 FIFO에서 배치(Batch) 단위로 데이터를 읽어옵니다.
 * **축(Axis) 구분 및 보정:**
   * **가속도(Accel) 3축:** X, Y, Z축 데이터를 동시에 수집합니다. 각 샘플은 수집 즉시 (Raw * LSB - Offset) * Gain 수식을 통해 물리적 단위(G)로 보정됩니다.
   * **자이로(Gyro) 3축:** 필요 시(gyro_enable) 활성화되며, 가속도와 동일한 FIFO 프레임 구조에서 분리하여 수집합니다.
   * **단축(1-Axis) 모드:** 특정 targetAxis가 설정된 경우(예: Z축만 분석), 해당 축 데이터를 메인 분석 버퍼로 집중시킵니다.
### **[Step 3] 고속 신호 처리 (Core 1: DSP Processing Phase)**
수집된 데이터는 PSRAM에서 Internal SRAM 고속 버퍼로 복사된 후, 처리 단위별로 가공됩니다.
#### **1. Shared(공통) 및 Audio 처리 단위**
 1. **Beamforming:** 스테레오 입력을 beam_gain 가중치로 합성하여 타겟 방향의 신호를 강화합니다.
 2. **DC Removal:** 신호의 평균값을 빼서 영점(0V)을 물리적으로 정렬합니다.
 3. **Median Filter:** 윈도우 내 중간값을 취해 순간적인 전기적 스파이크 노이즈를 제거합니다.
 4. **Pre-emphasis:** 고주파 성분의 감쇠를 보상하기 위해 이전 샘플과의 차분 연산을 수행합니다.
 5. **Filter Bank:** IIR Notch(60/120Hz 제거) → IIR HPF/LPF → FIR HPF/LPF(정밀 대역 제한) 순으로 통과시켜 순수 신호만 남깁니다.
#### **2. Vibration 처리 단위**
 * **3축 독립 처리:** X, Y, Z 각 축에 대해 독립적인 FIR 필터 인스턴스를 적용합니다.
 * **데이터 정합성:** 각 축의 위상차를 보존하기 위해 동일한 필터 계수와 처리 타이밍을 유지합니다.
### **[Step 4] 특징량 추출 및 융합 (Feature Extraction Phase)**
가공된 파형에서 진단을 위한 핵심 지표를 뽑아냅니다.
 * **통계 특징량:** RMS(에너지), Kurtosis(충격성), Crest Factor(피크 강도), Skewness(비대칭성)를 도메인별로 산출합니다.
 * **주파수 특징량:**
   * **Audio:** FFT 후 Noise Learning 결과값을 빼서 배경 소음을 제거하고, Spectral Centroid와 상위 피크를 추출합니다.
   * **Vibration:** 8개 밴드별 에너지 밀도를 계산하여 베어링/기어 결함 주파수를 추적합니다.
 * **AI 텐서(MFCC):** 4채널(Vib 3 + Aud 1) 각각에 대해 39차원 MFCC(Static, Delta, Delta-Delta)를 생성하여 최종 UnifiedFeatureSlot에 평탄화(Flattening)하여 저장합니다.
### **[Step 5] 판정 및 데이터 관리 (Decision & Storage Phase)**
 1. **결함 판정:** TriggerEngine이 현재 추출된 특징량과 설정된 임계값을 비교합니다. trialCounter를 통해 2회 연속 NG 발생 시에만 실제 트리거로 인정하여 오작동을 방지합니다.
 2. **시퀀스 빌딩:** 추출된 MFCC 벡터를 링버퍼 형태의 SequenceBuilder에 밀어 넣어 TinyML 모델의 입력 텐서(16프레임 시퀀스 등)를 상시 준비합니다.
 3. **비동기 저장:** 트리거 발생 시 StorageManager가 프리트리거(과거 3초 등) 데이터를 포함하여 SD카드에 .bin 및 .raw를 기록합니다. 이때 파일 헤더에 현재 시스템 설정을 JSON으로 덤프하여 완벽한 재현성을 보장합니다.
 4. **실시간 스트리밍:** 설정된 Hz 주기에 맞춰 텔레메트리, 파형, 스펙트럼, AI 시퀀스 데이터를 WebSocket으로 브로드캐스팅합니다.

---

**[Thought #1]**
 * **Thought:** 제안한 로드맵에 따라 **Stage 1(인프라 및 메모리 할당)**과 **Stage 2(데이터 수집 및 동기화)**의 심층 아키텍처를 분석한다.
 * **MECE Analysis:**
   1. 메모리 아키텍처: PSRAM(대용량 보관) vs Internal SRAM(고속 연산 핫패스). 누락 없이 할당 전략 분해.
   2. 오디오 수집(I2S DMA): 채널 분리(De-interleaving), 32bit PCM 정규화.
   3. 진동 수집(SPI FIFO): 배치(Batch) 파싱, 1축/3축 동적 매핑, 런타임 캘리브레이션 연산 병합.
 * **Reasoning:** v243의 가장 큰 아키텍처적 도약은 alignas(16) 기반의 SIMD 최적화와 메모리 파편화 방지를 위한 _MAX 기반 정적 할당이다. 이 물리적 한계가 센서 수집 단계에 어떻게 반영되었는지 수식과 코드를 통해 증명해야 한다.
 * **Next Step:** Stage 1, 2에 대한 심층 기술 문서 전개.
### **Stage 1. 인프라 설계 및 메모리 아키텍처 (Memory & Booting)**
v243 시스템은 OOM(Out of Memory) 방지와 esp-dsp SIMD 가속기의 성능을 100% 끌어내기 위해, 철저하게 분리된 메모리 풀링(Pooling) 전략을 사용합니다.
#### **1.1. 하이브리드 메모리 할당 전략 (SRAM vs PSRAM)**
CL_T2_FsmManager::init() 단계에서 두 가지 힙(Heap) 영역을 명확히 구분하여 사용합니다.
 * **PSRAM (Zero-Copy 데이터 보관용):**
   * **할당 대상:** _featurePool(특징량 슬롯 64개), _rawPool(원시 파형 슬롯 64개), 웹소켓 스트리밍 패킷(_pktTele, _pktWave 등).
   * **목적:** 대용량 데이터를 시스템 크래시 없이 보존하고, 통신 및 저장 태스크(Task)에 포인터만 넘기는 Zero-Copy 파이프라인 구축.
 * **Internal SRAM (DSP 연산 핫패스용):**
   * **할당 대상:** _capBufL, _capBufR, _prcBufL, _prcBufR, _prcBeamformed.
   * **목적:** ESP32-S3의 캐시 미스(Cache Miss)를 방지하기 위해, SPI/I2S로 들어온 데이터를 복사하여 FIR 필터, FFT 등 연산 집약적인 로직을 수행하는 초고속 워크스페이스. MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT 플래그를 사용합니다.
#### **1.2. SIMD 16바이트 강제 정렬 (Memory Alignment)**
모든 힙 할당은 heap_caps_aligned_alloc(16, ...)을 통해 메모리 주소가 16의 배수로 시작하도록 강제합니다. 이는 벡터 연산(128-bit 레지스터) 시 Alignment Fault를 원천 차단하여 연산 속도를 극대화합니다.
### **Stage 2. 데이터 수집 및 동기화 (Data Capture & Sync)**
Core 0에 할당된 _captureTask는 하드웨어 인터페이스(I2S, SPI)로부터 데이터를 블로킹/폴링 방식으로 퍼올립니다.
#### **2.1. Audio 도메인: I2S DMA 수집 및 De-interleaving**
CL_T2_SensorEngine::readAudioChunk()는 I2S DMA 버퍼에서 32bit PCM 데이터를 읽어옵니다.
 * **정규화 (Normalization):** 32bit 정수를 부동소수점으로 변환하기 위해 나눗셈 대신 사전에 정의된 곱셈 계수를 사용합니다. 연산식을 코어 레벨로 최적화했습니다.
   * Value = Raw_{int32} \times 4.656612873 \times 10^{-10} (PCM_32BIT_SCALE_CONST 활용).
 * **채널 모드 매핑 (AudioChannelMode):**
   * STEREO: 좌/우 버퍼(p_outL, p_outR)에 번갈아가며(De-interleave) 데이터를 채웁니다.
   * MONO_R: 우측 마이크 데이터만 추출하되, 후단 DSP 엔진의 호환성을 위해 **좌측 버퍼(p_outL)에 데이터를 덮어씌웁니다**. (메모리 낭비 차단)
#### **2.2. Vib 도메인: 고속 SPI FIFO 배치(Batch) 파싱**
CL_T2_SensorEngine::readVibFifoBatch()는 BMI270의 하드웨어 FIFO 레지스터를 직접 타격하여 오버헤드를 제거합니다.
 * **배치 처리:** FIFO 길이를 먼저 읽고, 한 번의 SPI 트랜잭션으로 최대 64 프레임(FIFO_BATCH_SIZE_MAX)을 긁어옵니다.
 * **헤더 헌팅 (Header Hunting):** 데이터 스트림 내에서 가속도 헤더(0x84)와 자이로 헤더(0x88)를 실시간으로 스캐닝하여 프레임을 분리합니다.
 * **런타임 캘리브레이션 즉시 적용:** 루프를 돌면서 배열을 순회하는 비용을 없애기 위해, FIFO 파싱과 동시에 오프셋 및 게인(Gain) 보정을 수행합니다.
   *  * **동적 축 매핑 (Axis Counting):** _axisCount가 1인 경우, 불필요한 배열 메모리 복사를 생략하고 _targetAxis(예: Z축)의 데이터만 p_outX 배열로 몰아넣어 1축 모드 최적화를 달성합니다.
#### **2.3. 동기화 및 타임스탬핑**
데이터 수집 직후, UnifiedFeatureSlot 헤더에 (uint64_t)time(NULL) * 1000을 통해 절대 시간(NTP 기준 밀리초)을 찍고, fid(Frame ID)를 1씩 증가시킵니다. 이후 _qReadyIdx 큐를 통해 Core 1의 DSP 엔진으로 처리 권한을 즉시 이관합니다.
위 내용은 시스템의 뼈대와 혈관(데이터 수집)에 해당합니다. 확인이 완료되시면, 다음 단계인 **Stage 3: 멀티모달 신호 처리 엔진(DSP 및 특징량 추출 수학적 공식 등)**에 대한 브리핑을 진행하겠습니다.


---

**[Thought #3] 멀티모달 신호 처리 및 특징량 추출 (Stage 3)**
 * **MECE Analysis:** 신호 처리 파이프라인을 시간 영역 전처리(DSP) → 주파수 변환(FFT) → 특징량 추출(Feature Extraction)의 3단계로 분리하고, 이를 다시 Audio와 Vibration 도메인으로 완벽히 교차 분해.
 * **Reasoning:** v243의 핵심은 OOM 방지를 위해 FFT_SIZE_MAX 크기의 고정된 정적 버퍼를 사용하면서도, SIMD(esp-dsp) 가속을 통해 연산 병목을 제거한 것입니다. 특히 Audio의 채널 융합(Beamforming)과 Vib의 3축 독립 병렬 처리 방식의 차이를 명확히 대조해야 합니다.
 * **Next Step:** DSP 엔진(T240_DspEng_243)과 특징량 추출 엔진(T245_FeatExtra_243)의 동작 명세 구체화.
### **Stage 3. 멀티모달 신호 처리 엔진 및 특징 추출 (DSP & Feature Extraction)**
수집된 원시 데이터(Raw Data)는 Internal SRAM으로 복사된 후, CL_T2_DspEngine과 CL_T2_FeatureExtractor를 거치며 머신러닝 및 룰베이스 판정을 위한 지표로 압축됩니다.
#### **3.1. 소음(Audio) 도메인 처리 상세**
 * **처리 단위 (Processing Unit):** T2_Def::Audio::Sensor::FFT_SIZE_MAX (최대 4096 샘플) 단위의 프레임 배열.
**① 채널 융합 및 시간 영역 전처리 (DSP Pipeline)**
 1. **Beamforming (Stereo/Mono 분기):** * **Stereo:** 좌(L)와 우(R) 채널 데이터를 (L + R) * beam_gain 수식으로 합성하여 단일 채널로 만듭니다.
   * **Mono L/R:** 활성화된 단일 채널 데이터에 동일하게 beam_gain을 곱하여 통과시킵니다.
 2. **DC Removal:** 신호 배열의 평균값을 계산한 뒤 전체 데이터에서 차감하여 영점(0V)을 맞춥니다.
 3. **Median Filter:** med_win(예: 3) 크기의 윈도우를 슬라이딩하며 중앙값을 추출하여 스파이크 노이즈를 제거합니다.
 4. **Pre-emphasis:** x[n] = x[n] - \alpha \cdot x[n-1] 공식을 적용하여 고주파수 대역의 에너지를 강조합니다.
 5. **IIR/FIR Filtering:** 60Hz/120Hz 대역을 파내는 Notch 필터를 거친 후, 설정에 따라 HPF/LPF 필터를 통과시킵니다.
 6. **Noise Gate:** 지정된 진폭 임계치(gate_thresh) 이하의 미세한 백색 소음을 강제로 0.0f로 묵음 처리합니다.
**② 주파수 변환 및 특징 추출 (Feature Extraction)**
 1. **통계 지표:** 전처리된 시간 영역 파형에서 RMS, 첨도(Kurtosis), 파고율(Crest Factor), 왜도(Skewness)를 계산합니다.
 2. **FFT 및 Spectral Subtraction:** Hann 윈도우 적용 후 복소 FFT 연산을 수행하여 파워 스펙트럼(_powerAudio)을 획득합니다. 학습 모드(_isLearning)일 경우 배경 소음 프로필을 누적 학습하고, 일반 모드일 경우 스펙트럼 감산법(v_subStr)을 통해 배경 소음을 차감합니다.
 3. **주파수 지표 추출:** * **Spectral Centroid:** 주파수 스펙트럼의 무게 중심을 계산합니다.
   * **Top Peaks:** NMS(Non-Maximum Suppression) 방식을 차용하여, 최소 이격 거리(peak_freq_gap_min) 조건을 만족하는 상위 10개의 강한 피크(Hz, Amp)를 추출합니다.
   * **Band Energy:** 에어 리크(Air Leak) 등 특정 결함 대역 파워의 합을 구합니다.
 4. **Audio MFCC:** 파워 스펙트럼을 Mel-Filterbank에 통과시키고 Log를 취한 뒤, DCT 매트릭스 곱셈을 수행하여 MFCC를 추출합니다. _historyCount 링버퍼를 통해 과거 5프레임을 추적하여 Delta 및 Delta-Delta까지 총 39차원(채널 인덱스 3)을 완성합니다.
#### **3.2. 진동(Vibration) 도메인 처리 상세**
 * **처리 단위 (Processing Unit):** T2_Def::Vib::Sensor::FFT_SIZE_MAX (최대 1024 샘플) 단위의 프레임 배열. 오디오와 달리 **물리적 축 단위로 완전히 분리되어 병렬 처리**됩니다.
**① 가속도 및 자이로 3축 독립 전처리 (DSP Pipeline)**
 1. **Axis Separation:** 가속도 X, Y, Z축 데이터는 v_out[3] 포인터 배열을 통해 각각 독립적인 루프를 탑니다. 자이로(Gyro) 데이터 역시 설정(gyro_enable) 시 1축 타겟팅을 통해 이 파이프라인에 편승할 수 있습니다.
 2. **Median Filter:** 각 축마다 독립적인 히스토리 버퍼(_medianHistVib[3])를 유지하며 스파이크 충격을 제거합니다.
 3. **FIR Filtering (SIMD):** 각 축별로 할당된 필터 인스턴스(_firInstHpfVib[3])를 거칩니다. esp-dsp 가속기를 사용하여 연산 속도를 극대화합니다.
**② 3축 독립 주파수 변환 및 특징 추출 (Feature Extraction)**
루프를 돌며 각 축(i = 0, 1, 2)에 대해 다음 과정을 3회 반복합니다.
 1. **통계 지표 산출:** 축별 RMS, Kurtosis, Crest Factor, Skewness 외에도 추가적으로 표준편차(STD)를 구합니다.
 2. **FFT 연산:** 각 축의 파형을 FFT 변환하여 축별 파워 스펙트럼(_powerVib)을 도출합니다.
 3. **결함 밴드 에너지 (Max-Pooling):** 회전체 결함 주파수(BPFO, BPFI 등)를 추적하기 위해 설정된 대역의 에너지를 구합니다. 3축 중 **가장 에너지가 강한 축의 밴드 에너지(v_maxBandEnergy)를 대표값으로 채택**하여 최종 슬롯에 저장합니다.
 4. **Vib MFCC 텐서:** 진동 파형의 미세한 질감(Texture) 변화를 AI가 인지할 수 있도록, 각 축에 대해 Audio와 동일한 방식으로 MFCC를 추출합니다.
   * 채널 0 (X축 MFCC 39D)
   * 채널 1 (Y축 MFCC 39D)
   * 채널 2 (Z축 MFCC 39D)
#### **3.3. 시퀀스 텐서 조립 (Sequence Building)**
추출된 진동 3축(117D) + 오디오 1축(39D) = 총 156차원(SIMD 정렬을 위해 384D 배열 내 적재)의 단일 프레임 특징량은 CL_T2_SequenceBuilder로 넘겨집니다.
 * _strideDim 패딩 정책에 의해 메모리가 정렬되며, 16 프레임(SEQUENCE_FRAMES_MAX)이 링버퍼에 차오르면 머신러닝 추론기(Edge AI)에 통째로 밀어넣을 수 있는 Packed Sequence Tensor로 재조립(getSequenceFlat)됩니다.
위 과정은 멀티모달 데이터가 UnifiedFeatureSlot이라는 800바이트 단일 데이터베이스(SSOT)로 압축되는 핵심 연산 과정입니다.
다음 단계인 **Stage 4: 이벤트 제어 및 출력 인프라 (결함 판정 룰베이스, 스토리지 비동기 저장, 스트리밍 송신 로직)**에 대한 전개를 진행할지 지시해 주십시오.


---

**[Thought #4] 이벤트 제어 및 출력 인프라 (Stage 4)**
 * **Thought:** 추출된 특징량과 AI 시퀀스 텐서가 최종적으로 어떻게 평가(Trigger)되고, 보존(Storage)되며, 외부로 송출(Communication)되는지 분석한다.
 * **MECE Analysis:**
   1. 결함 판정 (Decision): 룰베이스(Rule-base) 임계치 검사 및 신뢰성(Trial) 검증.
   2. 비동기 저장 (Storage): 프리트리거(Pre-trigger) 버퍼링 및 Non-blocking 파일 I/O.
   3. 실시간 통신 (Network): WebSocket 스트리밍 (정렬된 바이너리 패킷) 및 MQTT 클라우드 연동.
 * **Reasoning:** v243 아키텍처의 안정성을 결정짓는 핵심은 디스크 I/O나 네트워크 지연이 Core 1의 신호 처리 핫패스(Hot-path)를 블로킹하지 않도록 격리하는 것이다. 이를 위해 FreeRTOS 큐와 링버퍼(Ring Buffer)가 어떻게 활용되었는지 입증해야 한다.
 * **Next Step:** Stage 4 상세 로직 전개 및 전체 작동 프로세스 결론 도출.
### **Stage 4. 이벤트 제어 및 출력 인프라 (Decision & Output)**
Core 1에서 추출이 완료된 UnifiedFeatureSlot 데이터는 시스템의 최종 판단을 거쳐 저장 매체와 네트워크로 분배됩니다.
#### **4.1. 결함 판정 엔진 (Trigger Engine)**
CL_T2_TriggerEngine은 매 프레임마다 추출된 특징량을 설정값(SSOT)의 임계치와 비교하여 설비의 결함 여부(NG)를 판정합니다.
 1. **진동(Vib) 룰베이스 판정:**
   * 활성화된 3축 전체에 대해 RMS 임계치 초과 여부를 검사합니다.
   * 첨도(Kurtosis)와 파고율(Crest Factor) 임계치를 검사하여 베어링/기어의 기계적 충격을 감지합니다.
   * 활성화된 밴드(Band) 에너지 임계치를 검사하여 특정 결함 주파수 성분의 돌출을 확인합니다.
 2. **소음(Audio) 룰베이스 판정:**
   * 진동 판정 통과 시, 소음 도메인에 대해서도 RMS, 첨도, 파고율, 왜도(Skewness), 밴드 에너지를 동일하게 검사합니다 (에어 리크, 마찰음 감지).
 3. **신뢰성 검증 (Trial Counter):**
   * 일시적인 노이즈(False-Positive)로 인한 오작동을 막기 위해 _trialCounter를 운용합니다.
   * NG 조건이 감지되더라도 설정된 횟수(최소 2회) 연속으로 발생해야만 최종 트리거(이벤트 발생)로 인정하고 상태를 천이시킵니다.
#### **4.2. 비동기 스토리지 엔진 (Storage Manager)**
느린 SD 카드 I/O로 인해 전체 시스템이 멈추는 것을 막기 위해 철저한 비동기(Asynchronous) 아키텍처를 채택했습니다.
 1. **프리트리거 (Pre-Trigger) 버퍼링:**
   * **평상시 (Monitoring):** 트리거가 발생하지 않은 상태에서는 PSRAM에 할당된 프리트리거 링버퍼(_preFeatBuf, _preRawBuf)에 데이터를 순환 저장합니다.
   * 이를 통해 결함 이벤트 발생 시점 기준 과거 수 초간의 데이터를 잃지 않고 소급하여 저장할 수 있습니다.
 2. **비동기 링버퍼 및 전용 Task:**
   * **기록 시 (Recording):** 데이터는 _asyncFeatRing과 _asyncRawRing에 먼저 푸시(Push)됩니다.
   * 낮은 우선순위(STORAGE_PRIO_DEF)로 백그라운드에서 도는 별도의 FreeRTOS Task(_storageTaskProc)가 링버퍼의 데이터를 꺼내어 실제 파일에 기록(write)합니다.
 3. **파일 규격 및 MLOps 지원:**
   * 특징량은 .bin 포맷, 원시 파형은 WAV 규격(32bit Float, Stereo)을 준수하는 .raw 포맷으로 저장됩니다.
   * .bin 파일의 FileHeader에는 데이터 수집 당시의 모든 센서 설정, 필터 계수, 임계값 등 전체 시스템 상태가 8KB 크기의 **JSON 문자열(config_dump)로 함께 기록**되어 완벽한 MLOps 데이터 추적성을 제공합니다.
#### **4.3. 실시간 통신 및 스트리밍 (Communicator)**
가공된 데이터는 외부 관제 시스템이나 웹 UI로 실시간 송출됩니다.
 1. **WebSocket 바이너리 스트리밍:**
   * FSM 매니저 내부의 누적기 카운터에 따라 설정된 전송 주율(tele_hz, wave_hz 등)에 맞춰 스트리밍을 트리거합니다.
   * PktTelemetry, PktWaveform, PktSpectrum, PktSequence 등 각각의 목적에 맞는 패킷을 조립합니다.
   * **정렬 방어:** 자바스크립트 프론트엔드에서 Float32Array 파싱 시 메모리 정렬 오류가 나지 않도록, 구조체 내부에 _pad[2] 등을 삽입하여 16바이트 정렬을 강제한 평탄화(Flattening) 패킷을 브로드캐스팅합니다.
 2. **MQTT 클라우드 연동:**
   * 네트워크 유실 방지를 위해 QoS(기본 레벨 1) 설정과 LWT(Last Will and Testament)를 지원합니다.
   * 특징량 슬롯(UnifiedFeatureSlot)에서 결함 판정 결과와 대표 RMS 값 등 최소한의 필수 정보만 JSON 형태로 압축하여 지정된 토픽으로 Publish 합니다.
### **Conclusion: 수석 아키텍트 종합 제언**
T2_MSFM_240 v243의 설계는 엣지 디바이스(ESP32-S3)의 물리적 한계를 극복하기 위해 **메모리 할당의 정적화(_MAX 기반 OOM 원천 차단)**, **데이터 흐름의 비차단화(Zero-Copy & Async Queue)**, 그리고 **연산의 벡터화(alignas(16) 기반 SIMD)**를 극한으로 끌어올린 엔지니어링의 결정체입니다.
 * **확장성 (Scalability):** 4-Tier SSOT(DynamicConfig)가 중앙에서 모든 스위치를 통제하므로, 향후 새로운 센서 채널이나 AI 모델이 추가되더라도 UnifiedFeatureSlot의 패딩 영역만 조정하면 코어 파이프라인의 수정 없이 즉시 확장이 가능합니다.
 * **유지보수 (Maintainability):** 바이너리 파일 헤더에 JSON 설정값을 덤프(Dump)하는 설계는 추후 클라우드에서 불량 데이터를 학습시킬 때, 당시 디바이스가 어떤 필터 주파수와 임계치로 데이터를 바라보았는지 정확히 재현(Reproduce)할 수 있게 하여 MLOps 파이프라인의 신뢰성을 보장합니다.
지금까지 4단계에 걸쳐 v243의 전체 작동 상태 및 처리 내용을 체계적으로 분석 완료했습니다. 특정 알고리즘(예: Welch's Method 캘리브레이션 역산)이나 통신 프로토콜 규격에 대한 코드 레벨의 구현/수정이 필요하시다면 지시해 주십시오.


