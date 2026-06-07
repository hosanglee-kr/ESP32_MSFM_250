ICS-43434 추가

매뉴얼은 이제 그만
- 임계값은 현장의 가혹한 노이즈 환경에서 오동작을 줄이는 기능 추가 검토중
- 기존 대이터로 설비의 idle, run(속도 변화), off상태 반영
- 내용 검토 후 추가/보완해서 정리해줘

현장의 가혹한 노이즈 환경(공장 진동, 전원 노이즈, 물리적 충격)은 연구실 데이터와는 완전히 다른 차원의 난제입니다. 제시해주신 6가지 기능은 T20 시스템을 단순한 분석기를 넘어 **'현장 투입형 산업용 장비'**로 격상시키는 핵심 로직들입니다.
특히 **설비의 상태(OFF, IDLE, RUN)**를 반영하여 오동작을 줄이는 관점에서 기존 내용을 검토하고, 구현 시 반드시 고려해야 할 **엣지 케이스 방어책**을 추가하여 정리해 드립니다.
### 🛡️ T20 현장 신뢰성 강화 및 상태 자각(Awareness) 설계
#### 1. 트리거 히스테리시스 (Hysteresis) 및 디바운스 강화
 * **보완점:** 단순 비율(\times 0.8)뿐만 아니라 **'지속 시간(Debounce)'** 개념을 병행해야 합니다.
 * **로직 고도화:**
   * **Latch-ON:** Energy > Threshold_{High} 가 **N개 프레임** 연속될 때 활성화.
   * **Latch-OFF:** Energy < Threshold_{Low} ($Threshold_{High}$의 80%~90%)가 **M개 프레임** 연속될 때 + Hold\_Time 만료 시 해제.
 * **효과:** 대형 모터 기동 시 발생하는 일시적 서지 진동에 의한 중복 파일 생성을 원천 차단합니다.
#### 2. 적응형 임계값 (Adaptive Threshold) 및 노이즈 플로어 추적
 * **보완점:** 설비가 가동 중(RUN)일 때와 유휴 상태(IDLE)일 때의 베이스 노이즈는 다릅니다.
 * **구현전략:**
   * **상태별 \sigma 관리:** NOISE_LEARN 시 설비를 IDLE 상태로 두어 $Threshold_{Base} = \mu_{idle} + k\sigma_{idle}$를 산출합니다.
   * **동적 가중치:** 설비가 RUN 상태(가동 중)로 판명되면 민감도 k를 자동으로 상향 조정하여, 정상 가동 진동을 이상 징후로 오판하지 않도록 방어합니다.
   * **수식:** Threshold_{final} = \mu_{base} + (k \times \sigma_{base} \times State\_Multiplier)
#### 3. 설비 상태 인식 (Machine State Awareness)
 * **검토:** 기존 데이터를 분석하여 현재 설비가 어떤 상태인지 FSM에 반영합니다.
 * **상태 판별 기준:**
   * **OFF (전원 꺼짐):** RMS \approx 0, 주파수 성분 전무. (이때는 트리거 감시 중단 가능)
   * **IDLE (유휴):** 낮은 수준의 정적 진동(냉각 팬 등). 특정 주파수(예: 60Hz) 대역에 에너지가 집중됨.
   * **RUN (가동):** 전체 에너지 급증 및 지배적 주파수(Dominant Freq)의 이동.
 * **속도 변화 대응:** 가속/감속 시 지배적 주파수가 변하면 이를 '이상'이 아닌 '상태 변화'로 인식하도록 FFT Peak 추적 로직을 연계합니다.
#### 4. 신호 품질 지수 (SQI) 및 자가 진단
 * **세부 보완:**
   * **Saturation Check:** 가속도계 데이터가 설정 범위(Full-scale)의 98%를 넘어서는 샘플을 카운트합니다.
   * **Flat-line Detection:** 센서 통신은 정상이지만 값이 일정 시간 고정(stuck)되면 ADC 불량으로 간주합니다.
   * **Orientation Watchdog:** 1G 중력 벡터의 방향이 캘리브레이션 대비 30^\circ 이상 틀어지면 **'물리적 고정 탈거(Mounting Failure)'** 알람을 발생시킵니다.
#### 5. 피크 홀드(Peak-Hold) 기반 충격 탐지
 * **검토:** RMS는 에너지를 평균화하므로 볼트 파손이나 금속 간 충격 같은 '임펄스' 신호를 놓치기 쉽습니다.
 * **보완:**
   * **High-Pass Path:** DC와 저주파가 제거된 Raw 신호에서 바로 Max\_Abs를 추출합니다.
   * **Trigger Link:** RMS 임계값 미달이더라도 Peak\_Value가 설정된 충격 임계값을 넘으면 즉시 **'Event'**로 간주하여 프리트리거를 고정(Latch)시킵니다.
#### 6. 동적 샘플링 및 전송 스케일링 (Fidelity Scaling)
 * **기능 보완:** * **MONITORING (Normal):** 저주파 위주의 요약 데이터(RMS, Peak, MFCC)만 전송 (네트워크 대역폭 절약).
   * **RECORDING (Event):** 1600Hz 전 대역 Raw 데이터 SD 기록 및 WebSocket 스트리밍 해상도 4배 상향.
 * **Post-Event Action:** 기록 종료 후 사고 파일의 통계 요약본(사고 시 Max G, 지배 주파수 등)을 MQTT로 우선 보고하여 관리자가 즉시 상황을 파악하게 합니다.
### 🚀 요약된 구현 우선순위 및 로드맵
| 순위 | 기능명 | 핵심 가치 | 타 모듈 영향도 |
|---|---|---|---|
| **1** | **SQI (클리핑/탈거 감시)** | 분석 결과의 신뢰성 담보 | 상 (DSP/Admin API) |
| **2** | **Hysteresis + Debounce** | SD 카드 수명 및 데이터 정제 | 중 (Trigger Ctx) |
| **3** | **Machine State Awareness** | 가동 중 오동작 최소화 | 중 (FSM 로직) |
| **4** | **Peak-Hold Trigger** | 순간 파손 사고 포착 | 하 (DSP 파이프라인) |
| **5** | **Adaptive Threshold** | 현장 설치 편의성 극대화 | 중 (Noise Learn) |
위 내용들 중 **SQI(신호 품질 지수)와 설비 상태 판별 로직**을 먼저 결합하여 T231_Dsp_Pipeline을 업데이트하면, 노이즈가 가득한 실제 공장 환경에서도 "진짜 고장"만 골라내는 똑똑한 센서가 될 것입니다. 어느 부분부터 상세 설계를 진행할까요?



## ***
## --- 
## ***


- 매뉴얼은 이제 그만
- 임계값은 현장의 가혹한 노이즈 환경에서 오동작을 줄이는 기능 추가 검토중
- 내용 검토 후 기능 추가/보완

### 1. 트리거 히스테리시스 (Hysteresis) 강화
 * **검토:** 진동 신호는 경계값 근처에서 매우 불안정하게 요동치므로 필수적입니다.
 * **구현 가이드:** ST_T20_TriggerCtx_t 구조체에 is_latched 플래그를 추가합니다.
 * **로직:**
   * **ON 조건:** Current\_Energy > Threshold_{High}
   * **OFF 조건:** Current\_Energy < (Threshold_{High} \times 0.8) AND Hold\_Time\_Expired
 * **효과:** 트리거가 빈번하게 붙었다 떨어졌다 하는 '채터링(Chattering)' 현상을 방어하여 SD 카드의 파일 시스템 인덱스 오염을 막습니다.
### 2. 적응형 임계값 (Adaptive Threshold)
 * **검토:** 공장마다 베이스 노이즈가 다르므로 "통계적 유의미성"을 기반으로 임계값을 잡는 것은 매우 스마트한 접근입니다.
 * **수학적 보완:** * NOISE_LEARN 단계에서 각 대역별 에너지의 평균(\mu)과 표준편차(\sigma)를 산출합니다.
   *    * 여기서 k는 유저가 설정하는 '민감도(Confidence Factor)'입니다. (예: k=3이면 99.7% 확률 이상의 이상 진동만 포착)
### 3. 신호 품질 지수 (SQI: Signal Quality Index)
 * **검토:** 시스템 신뢰도를 결정짓는 가장 중요한 기능입니다. 특히 클리핑(Clipping) 감시는 매우 시급합니다.
 * **세부 구현:**
   * **Clipping:** ADC 최대값(예: ±16G 설정 시 15.5G 이상)에 도달한 샘플이 프레임 내 5% 이상이면 경고.
   * **Mounting Failure:** 기기가 가동 중인데도 고주파 성분이 아예 없고 저주파 진동만 감지되면 "센서 이탈"로 간주.
 * **조치:** Web UI 상단의 상태 태그를 ERROR: SENSOR_HEALTH로 변경하고 MQTT로 즉시 알람 송신.
### 4. 피크 홀드 트리거 (Peak-Hold Trigger)
 * **검토:** 파손 사고(Impact)는 RMS(적분형)에 잘 안 걸립니다. 미분형 감시 경로가 필요합니다.
 * **로직:** * DSP 파이프라인에서 필터링된 직후의 Raw 데이터 중 abs_max()를 추출합니다.
   * 이 값은 윈도우 함수(Hann 등)가 적용되기 전의 **순수 충격 에너지**여야 합니다.
 * **장점:** 볼트 풀림, 크랙 발생 등 '찰나의 이벤트'를 놓치지 않고 4초 전부터 기록하는 프리트리거를 가동시킵니다.
### 5. 설정 프리셋 슬롯 (Config Presets)
 * **검토:** 현장 배포 속도를 획기적으로 높여줍니다.
 * **구현:** * /sys/presets/ 폴더를 생성하고 slot1.json, slot2.json 형태로 관리합니다.
   * Web UI에서 "모터 모드", "팬 모드", "베어링 모드" 등으로 이름을 붙여 관리할 수 있게 합니다.
### 6. 동적 샘플링 스케일링 (Event-Driven Fidelity)
 * **상황:** 평상시 1600Hz 데이터는 너무 방대하여 네트워크나 SD 용량에 부담을 줍니다.
 * **기능:** * **평상시(MONITORING):** 저전력을 위해 FFT 결과와 MFCC만 웹으로 전송합니다.
   * **트리거 발생(RECORDING):** 즉시 save_raw 옵션을 강제로 켜서 **초정밀 Raw 파형 기록**을 시작하고, 웹 스트리밍 주기를 일시적으로 높입니다.
 * **장점:** "평소에는 조용히, 사고 시에는 현미경처럼" 동작하는 지능형 블랙박스를 완성합니다.

## ***
## --- 
## ***


- 트리거 히스테리시스 (Hysteresis) 강화
   - 문제: 진동이 임계값 근처에서 왔다 갔다 할 경우 파일이 너무 자주 열리고 닫히는 현상이 생길 수 있습니다.
   - 개선: trigger_hold_frames 외에 **입력 임계값(High)**과 **해제 임계값(Low)**을 분리하여 '채터링' 현상을 방지.
   - 
- 적응형 임계값 (Adaptive Threshold)
   - 상황: 주변 배경 진동이 바뀌면 오작동할 수 있습니다.
   - 개선: NoiseMode 학습 시의 에너지를 바탕으로 임계값을 $Background\_Energy \times Margin$ 형태로 자동 설정하는 모드 추가.




## ***
## --- 
## ***


- 가속도/자이로 구분 설정/임계값
- 가속도, 자이로 단위 통일 필요(측정,표시,임계값)
- 3축모드 설정시 가속도 3축인지 자이로 3축 구분
- 
- T234_Storage_Service 내부에서 기록 실패가 N회 이상 발생하면 자동으로 closeSession() 후 다시 begin()을 시도하는 보호 로직
- 안전한 재부팅: 웹에서 설정을 저장하고 ESP.restart()가 호출될 때, StorageService의 세션이 열려있다면 closeSession()이 호출되지 않고 꺼질 수 있습니다. CommService의 reboot API 핸들러에서 p->storage.closeSession("reboot")을 먼저 호출하도록 보강
- ST_T20_RecorderBinaryHeader_t의 version이 JSON에서는 219로 되어 있고 코드에서는 220으로 혼용되는 부분이 보입니다. 바이너리 파일 분석 툴(Python 등)과의 호환성을 위해 220으로 통일
- 스마트 트리거 밴드별 논리 연산 (AND/OR/XOR)
   - 현황: 현재는 3개의 밴드 중 하나라도 넘으면 트리거되는 OR 구조입니다.
   - 개선: "A 밴드는 높고 B 밴드는 낮을 때"와 같은 조합 논리를 추가하면, 단순 충격과 실제 기계 결함 진동을 더 정밀하게 구분할 수 있습니다.


### 📖 2. 적용 보류 항목 (기술적 상세 분석)
향후 시스템을 극한까지 쥐어짜야 할 때(예: 샘플링 레이트를 4000Hz 이상으로 올릴 때)를 대비하여, 이번 릴리즈에서는 보류한 두 가지 최적화 기법에 대한 엔지니어링 분석입니다.
#### **[보류 항목 1] 캐시 친화적(Cache-friendly) 메모리 접근**
 * **상세 설명:** ESP32-S3는 외장 PSRAM의 데이터를 읽어올 때 SPI 통신을 통해 캐시 메모리(D-Cache)에 페이지 단위로 적재(Fetch)합니다. 만약 3축 데이터를 [X축 배열], [Y축 배열], [Z축 배열] 식으로 멀리 떨어진 주소에서 번갈아 읽어오면, 캐시 메모리에 원하는 데이터가 없어 계속 SPI 통신을 유발하는 **캐시 미스(Cache Miss)**가 대량 발생합니다.
 * **적용 방안 (공간적 지역성 극대화):** raw_buffer를 1D 구조로 선언하고 데이터를 X1, Y1, Z1, X2, Y2, Z2 순으로 교차 배치(Interleaved)한 후, DMA 전송 단에서 블록 단위로 한 번에 Internal SRAM으로 퍼 올립니다.
 * **장점:** PSRAM -> SRAM 복사 속도가 비약적으로 상승하여 메모리 I/O 병목이 소멸됩니다.
 * **단점 (적용 보류 이유):** 1. 데이터 구조가 극도로 복잡해져 유지보수 난이도가 폭증합니다.
   2. 현재의 1600Hz 핑퐁 버퍼 수준에서는 캐시 미스로 인한 지연 시간(수 마이크로초)이 시스템의 실시간성을 훼손하지 않을 만큼 미미합니다.


#### **[보류 항목 2] 컴파일러 최적화 플래그 지시어 (#pragma GCC optimize)**
 * **상세 설명:** 소스 코드 상단에 #pragma GCC optimize ("O3", "ffast-math", "unroll-loops")를 강제로 주입하여, 컴파일러가 알아서 어셈블리 단에서 for 루프를 풀어버리고(Unroll) SIMD 명령어로 변환(Auto-vectorization)하도록 지시하는 기법입니다.
 * **장점:** 코드를 한 줄도 수정하지 않고도 공짜로 10~20%의 연산 속도 이득을 얻을 수 있습니다.
 * **단점 (적용 보류 이유):** 1. **ffast-math의 부작용:** 이 플래그는 IEEE 754 부동소수점 표준을 무시합니다. 컴파일러가 **"어차피 연산 중에 NaN이나 Inf는 안 나올 거야"**라고 임의로 가정해 버리기 때문에, 앞서 우리가 열심히 작성한 **안정성 보장용 isnan(), isinf() 방어 코드를 최적화 과정에서 통째로 삭제해 버리는 치명적인 문제**를 낳습니다.
   2. **디버깅의 지옥:** 루프가 전개되고 명령어가 재배치되어, 문제 발생 시 JTAG 디버거로 코드를 한 줄씩 따라가는(Step-over) 것이 불가능해집니다.
**결론:** 현재 적용하신 코드만으로도 **ESP32-S3가 낼 수 있는 가장 안정적이고 효율적인 최상단 성능**에 도달했습니다. 위 코드들을 복사하여 반영하시면 산업 현장에서 "죽지 않는 무결점 분석기"로 활약할 것입니다!



## ***
## --- 
## ***


### 2. 데이터 수집 및 라벨링 자동화 (Active Learning Support)
양질의 AI 모델을 위해서는 데이터의 '질'이 중요합니다.
 * **웹 기반 데이터 라벨링 도구**: 'Explorer' 탭에서 저장된 바이너리 파일을 선택하면 해당 구간의 MFCC 워터폴을 다시 보여주고, 사용자가 "정상/비정상" 태그를 달아 JSON 인덱스에 저장하는 기능을 추가합니다.



### 메모리, littlefs, sd 사용량/잔여용량 모니터링



### 5. 대시보드 시각화 확장 (UX Enhancement)
웹 UI의 분석 기능을 전문가 수준으로 높입니다.
 * **오프라인 분석기 (Playback Mode)**: SD 카드에 저장된 바이너리 파일을 웹으로 불러와 실제 수집 당시의 차트와 워터폴로 재현(Replay)하는 기능을 구현합니다.
 * **주파수 대역별 RMS 차트**: 특정 설비 진동 분석에 필수적인 주파수 영역별 에너지 합산(RMS) 그래프를 추가하여 직관적인 모니터링을 돕습니다.
 * **다국어 지원 및 테마 최적화**: 사용자의 설정에 따라 한국어/영어 전환 및 가시성이 높은 고대비 테마 기능을 제공합니다.
 * **Web-based 3D 진동 시각화**: Three.js 등을 사용하여 웹 대시보드에 기계의 3D 모델을 띄우고, 측정 중인 축의 진동 방향과 세기를 실시간 화살표나 히트맵으로 표시합니다.
 





## ***
## --- **완료** ---
## ***



**T20 MFCC v219** 고도화 계획은 시스템을 단순한 데이터 수집기를 넘어, **지능형 클라우드 엣지 디바이스**로 진화시키는 매우 중요한 단계입니다. 제안하신 5가지 핵심 기능에 대한 기술적 검토와 구체적인 구현 로드맵을 정리해 드립니다.
### 1. 기술 검토 및 시스템 영향 분석
#### **① FFT SIZE 동적 설정 (256 ~ 4096)**
 * **검토**: 현재 FFT_SIZE는 상수로 고정되어 있으며, 관련 버퍼들이 정적으로 할당되어 있습니다.
 * **해결**: CL_T20_DspPipeline 내부의 작업 버퍼들을 heap_caps_malloc을 이용한 동적 할당 방식으로 전환해야 합니다. 4096 포인트 사용 시 약 16KB의 버퍼가 필요하며, S3의 내부 RAM(512KB)에서 충분히 수용 가능합니다.
 * **주의**: FFT 크기가 변하면 esp-dsp 라이브러리의 룩업 테이블 초기화와 윈도우 함수(_window) 재계산이 수반되어야 합니다.
#### **② NTP 기반 정밀 타임스탬프 삽입**
 * **검토**: 현재 바이너리 데이터는 frame_id만 가지고 있으며 절대 시간을 알 수 없습니다.
 * **해결**: ST_T20_FeatureVector_t 구조체에 uint64_t timestamp_ms 필드를 추가합니다.
 * **동기화**: T20_sensorTask에서 데이터 가득 참(Interrupt) 발생 시, gettimeofday()를 통해 마이크로초 단위의 정밀 시간을 획득하여 프레임에 기록합니다.
#### **③ Cloud & Connectivity (MQTT / OTA)**
 * **MQTT**: PubSubClient 라이브러리를 통합하여 CL_T20_CommService에서 비동기 발행(Publish)을 수행합니다. 이상 탐지 발생 시 즉시 JSON 페이로드를 전송합니다.
 * **OTA**: Update.h 라이브러리를 사용하여 웹 핸들러에 바이너리 업로드 엔드포인트를 추가합니다.
#### **④ 이벤트 기반 레코딩 트리거 (Smart Trigger)**
 * **검토**: 단순 RMS 임계값 외에 주파수 대역별 에너지 감시가 필요합니다.
 * **해결**: DSP 파이프라인에서 특정 Mel-Bin 주파수 대역의 합산 에너지를 계산하고, 설정된 임계값(Threshold)과 비교하여 storage.openSession()을 자동으로 호출하는 로직을 T20_processTask에 구현합니다.
#### **⑤ 다중 축(Multi-axis) 퓨전 (117D 확장)**
 * **검토**: 현재 센서 엔진은 한 번에 한 축의 FIFO만 읽습니다.
 * **해결**: BMI270 FIFO 설정에서 3축(X, Y, Z)을 동시에 활성화하고, readFifoBatch를 통해 인터리빙(Interleaving)된 데이터를 분리하여 3개의 DSP 인스턴스 또는 루프를 통해 처리합니다. 결과 벡터는 39 * 3 = 117차원으로 결합됩니다.
### 2. v219 구현 계획 (Implementation Roadmap)
#### **[Phase 1] 데이터 구조 및 메모리 혁신**
 1. **구조체 업데이트 (T214)**: ST_T20_Config_t에 fft_size 필드 추가, ST_T20_FeatureVector_t에 timestamp_ms 추가.
 2. **동적 버퍼 할당 (T231)**: CL_T20_DspPipeline 내부의 _work_frame, _power 등을 포인터로 변경하고 begin()에서 fft_size에 맞춰 메모리를 할당하는 로직 구현.
 3. **바이너리 헤더 갱신**: 기록 파일의 header_size를 유지하면서 타임스탬프 유무를 나타내는 플래그 추가.
#### **[Phase 2] 지능형 제어 및 클라우드 통합**
 1. **NTP 서비스 강화**: CommService 시작 시 NTP 서버 동기화 완료를 확인한 후 시스템 시각을 설정합니다.
 2. **MQTT 엔진 탑재**: 브로커 주소, 토픽 설정을 JSON에서 로드하고 백그라운드 태스크에서 상태 정보를 발행합니다.
 3. **트리거 엔진 구현**: preprocess.filter 단계 이후 특정 주파수 밴드(예: 베어링 결함 주파수 대역)의 에너지를 감시하는 로직을 추가합니다.
#### **[Phase 3] 웹 인터페이스 및 OTA 확장**
 1. **UI 업데이트 (index_218, app_218)**: FFT Size(256~4096) 드롭다운 메뉴 추가 및 MQTT 설정 폼 추가.
 2. **바이너리 파서 업데이트**: 117차원 데이터 및 타임스탬프가 포함된 새로운 WebSocket 바이너리 포맷을 프론트엔드에서 파싱할 수 있도록 수정합니다.
 3. **OTA 핸들러 등록**: /api/t20/ota_update 엔드포인트를 열어 웹에서 직접 .bin 펌웨어를 업로드 가능하게 합니다.
### 3. v219 예상 시스템 사양 (Target Spec)
 * **샘플링 주파수**: 1600Hz 고정 (BMI270 최적화)
 * **분석 해상도**: 최대 0.39Hz (4096 FFT 시)
 * **특징량**: 39D (Single) 또는 117D (Triple-axis Fusion)
 * **메모리 점유**: 내부 RAM 약 150~200KB (동적 할당 포함)
 * **통신 스택**: HTTP, WebSockets, MQTT, OTA 지원
이 계획에 따라 가장 먼저 **[Phase 1]의 메모리 동적 할당 및 타임스탬프 구조체 수정**부터 진행할 준비가 되었습니다. 어떤 단계부터 코드를 구체화해 나갈까요?



***

## 1. FFT SIZE 웹에서 설정
  * 256, 512, 1024, 2048, 4096
## 2. **NTP 기반 정밀 타임스탬프**: 파일 이름뿐만 아니라 바이너리 데이터 내부 프레임마다 정확한 UTC 타임스탬프를 삽입하여 타 장비 데이터와의 동기화 분석을 가능하게 합니다.

## 3. 통신 및 원격 관리 고도화 (Cloud & Connectivity)
현재는 로컬 웹 서버 중심이지만, 장비가 늘어날 경우를 대비해야 합니다.
 * **MQTT 프로토콜 지원**: 대시보드 접속 없이도 원격 서버(AWS IoT, Mosquitto 등)로 이상 징후 알림 및 시스템 상태를 발행(Publish)합니다.
 * **OTA(Over-The-Air) 업데이트**: 웹 인터페이스를 통해 새로운 펌웨어(AI 모델 포함)를 업로드하고 기기를 원격으로 업데이트하는 기능을 추가합니다.

## 4. **이벤트 기반 레코딩 트리거**: 단순 버튼이나 설정값이 아닌, 특정 주파수 대역의 에너지가 급증하거나 AI가 "모르는 패턴"이라고 판단할 때 자동으로 SD 카드에 기록을 시작하는 기능을 구현합니다.
## 5.**다중 축(Multi-axis) 퓨전**: 현재는 한 번에 하나의 축(예: Accel Z)만 분석하지만, 3축 데이터를 동시에 MFCC로 추출하여 특징량을 확장(39D → 117D)함으로써 공간적 진동 특성을 반영합니다.



### 3. 🎯 트리거 변수 및 임계값(Threshold) 분리/통폐합
현재 cfg.trigger 안에는 하드웨어 전원 관리(딥슬립/Wake)와 소프트웨어 연산(DSP 트리거)이 혼재되어 있습니다. 이들의 목적과 임계값 단위를 명확히 분리해야 합니다.
**개선된 Trigger 구조체 설계안:**
```cpp
typedef struct {
    // 1. 하드웨어 전원/모션 제어 (BMI270 칩셋 레벨)
    struct {
        bool     use_deep_sleep;
        uint32_t sleep_timeout_sec;
        float    wake_threshold_g;     // 하드웨어 Any-Motion 임계값 (G 단위)
        uint16_t duration_x20ms;
    } hw_power;

    // 2. 소프트웨어 이벤트 감시 (DSP 레벨)
    struct {
        uint32_t hold_time_ms;         // (중요) 기존 소스에 5000ms로 하드코딩된 값을 변수화
        
        bool     use_rms;
        float    rms_threshold_power;        // 전체 진동 파워 임계값

        ST_T20_TriggerBand_t bands[T20::C10_DSP::TRIGGER_BANDS_MAX]; // 다중 밴드 감시
    } sw_event;
} ST_T20_ConfigTrigger_t;

```
 * **분리 이유:** 하드웨어를 깨우는 wake_threshold_g는 거칠고 큰 충격(예: 1.0G)에 반응해야 하지만,  
 * DSP에서 감시하는 rms_threshold_power는 미세한 변화(예: 0.1G)를 잡아내야 할 수 있으므로 두 임계값을 분리하는 것이 맞습니다.
 
- storage.closeSession("trigger_hold_timeout"); 5초 고정값을 설정값으로 변경
- bmi wakup 임계값(단위확인필요)과 rms 임계값 구분 필요 검토

- 프론트엔드] Chart.js 실시간 성능 부하
   - 현재 app_220_001.js는 requestAnimationFrame을 통해 60Hz로 렌더링을 시도합니다.
   - 위험: 3축 모드 + 1024 FFT를 켜면, 매 프레임마다 Chart.js가 3000개 이상의 포인트를 다시 그립니다. 모바일 브라우저나 저사양 PC에서는 웹 UI가 급격히 느려질 수 있습니다.
   - 권장: FFT Size가 512 이상일 경우, pendingWave 데이터를 전송하기 전에 2:1 혹은 4:1로 **Decimation(추출)**하여 차트 포인트 숫자를 줄이는 로직을 JS에 추가하는 것이 좋습니다.
- [백엔드] Smart Trigger - 밴드 에너지 초기화 문제
   - 점검: 이 루프는 all_ready 상태가 아닐 때(MFCC 히스토리가 덜 찼을 때)도 매번 실행되어 ws_payload를 채웁니다.
   - 개선: p->dsp.getBandEnergy()는 FFT 직후에 에너지를 계산하므로 문제가 없으나, 메모리 효율을 위해 all_ready 조건 밖에서 수행하는 것이 맞습니다. 다만, max_band_energy를 p_feature->band_energy에 대입하는 시점은 all_ready 안쪽이어야 합니다.
   - 

- 트리거 전후 데이터 확보 (Pre-Trigger Buffering)
   - 현황: 트리거가 발생하는 순간부터 기록을 시작하므로, 충격 직전의 징후 데이터를 놓칠 수 있습니다.
   - 개선: 순환버퍼 등을 활용하여 trigger 발생전 최대한의 과거 데이터 저장되게 


- 트리거 전후 데이터 확보 (Pre-Trigger Buffering)
   - 현황: 트리거가 발생하는 순간부터 기록을 시작하므로, 충격 직전의 징후 데이터를 놓칠 수 있습니다.
   - 개선: raw_buffer 슬롯 4개 중 1~2개를 순환 버퍼로 상시 유지하여, 트리거 발생 전 약 100~200ms 분량의 데이터를 포함해 SD에 저장하도록 수정 제안합니다.
   - 개선: StorageService에 약 1~2초 분량의 특징량 순환 버퍼를 두고, 트리거 발생 시 **"발생 2초 전 데이터 + 현재 데이터"**를 묶어서 저장하도록 개선 (사고 분석 시 매우 유용).

아래 기능 개선/추가항목 검토
- IIR 필터를 아래와 같이 lpf, hpf 각각 적용 가능하게 보완
   - hfs 적용 여부 및 cutoff 주파수, q값
   - lfs 적용 여부 및 cutoff 주파수,q값
 
- Median Filter 추가 검토
- Adaptive Notch Filter  추가  검토

- Windowing 함수 추가
   - Hann 이외에 자주사용되는 함수 추가 및 선택 가능

- 추가 필터 관련 추가 보완 검토

- 구조체는 개별로 정의(익명구조체 금지)
- 구조체/변수명 좀더 명확하게




- 트리거, 이벤트 및 상태관리 개선 방안 검토
    - 여러 상태 변수,flag 변수들 통폐합/분리 검토 및 상태 머신(FSM) 명시적 구현
    - 시작모드 구분
          - auto :  자동 시작
          - manual : 사용자 시작, 종료
          - 스케줄(향후 구현) : 특정요일 들, 특정시간대 들 
    - 노이즈 학습(manual, 적응형)
    - 관련 설정항목 통폐합 및 분리 검토
    - trigger 이벤트 유형, 상태관리 검토
    - trigger 변수 및 임계값 분리/통폐합 검토
    - 
    - 

현재 분산되어 있는 상태 플래그(Flag)와 트리거 변수들을 통폐합하고, **명시적인 유한 상태 머신(FSM, Finite State Machine)** 구조로 개편하는 것은 시스템의 확장성(스케줄링 도입 등)과 무결성을 위해 반드시 거쳐야 할 핵심 엔지니어링 과정입니다.
T20 시스템의 아키텍처를 한 단계 도약시키기 위한 **트리거, 이벤트 및 상태 관리 개편 방안**을 5가지 관점에서 종합 검토 및 설계해 드립니다.
### 1. 명시적 상태 머신(FSM) 도입 및 플래그 통폐합
현재 is_recording, is_learning, is_triggered 처럼 개별적으로 놀고 있는 bool 변수들을 하나의 **시스템 전역 상태(State)**로 통합해야 합니다. 이렇게 하면 "학습 중이면서 대기 중"과 같은 논리적 모순(Deadlock)이 원천 차단됩니다.
**[개선된 시스템 상태 (System State Enum)]**
```cpp
typedef enum {
    SYS_STATE_INIT = 0,         // 부팅 및 하드웨어 초기화
    SYS_STATE_READY,            // Manual 모드에서의 대기 상태 (센서 정지 또는 최소 가동)
    SYS_STATE_MONITORING,       // 센서 가동 및 데이터 분석 중 (트리거 대기)
    SYS_STATE_RECORDING,        // 트리거 조건 충족 -> SD카드 및 스트림 출력 중
    SYS_STATE_NOISE_LEARNING,   // 수동 노이즈 프로필 학습 중 (기록 중지)
    SYS_STATE_ERROR             // 센서 통신 단절 등 치명적 오류 상태
} EM_T20_SysState_t;

```
### 2. 시작 모드 (Operation Mode) 분리 및 개편
기존의 system.auto_start (bool) 항목을 폐기하고, 확장 가능한 **운영 모드(Op Mode)**로 개편합니다. 향후 스케줄링 기능이 추가될 자리를 미리 확보합니다.
**[개선된 운영 모드 설정 구조]**
```cpp
typedef enum {
    OP_MODE_MANUAL = 0,   // 사용자 시작(START)/종료(STOP) API 호출 시에만 가동
    OP_MODE_AUTO = 1,     // 부팅 직후 자동으로 MONITORING 상태 진입
    OP_MODE_SCHEDULE = 2  // [향후구현] RTC 시간 기반 특정 요일/시간대 가동
} EM_T20_OpMode_t;

// 설정 구조체 반영
typedef struct {
    EM_T20_OpMode_t op_mode;      // 기존 auto_start 대체
    uint32_t        watchdog_ms;
    uint8_t         button_pin;
} ST_T20_SystemConfig_t;

```
 * **동작 원리:** 부팅 완료 후 SYS_STATE_INIT에서 op_mode를 검사합니다. AUTO면 바로 MONITORING으로, MANUAL이거나 SCHEDULE 대기 시간이면 READY 상태로 진입합니다.
### 3. 노이즈 학습 로직의 명확한 분리 (Policy vs Action)
현재 노이즈 처리는 '학습 모드'와 '감산 모드'가 혼재되어 헷갈릴 수 있습니다. **적용 정책(Policy)**과 **수동 트리거 이벤트(Action)**를 명확히 분리합니다.
 * **설정 항목 (Policy):** dsp.noise.mode
   * FIXED: 고정된 프로필 사용. (이 프로필은 Manual Learn 액션을 통해서만 갱신됨)
   * ADAPTIVE: SYS_STATE_MONITORING 상태에서 백그라운드로 소음을 계속 추적하며 자동 갱신됨. (사용자 개입 불필요)
 * **제어 명령 (Action):** Manual Learn 버튼 클릭
   * **이벤트 흐름:** CMD_LEARN_START 이벤트 수신 -> 상태를 SYS_STATE_NOISE_LEARNING으로 변경 -> noise_learn_frames 개수만큼 정지 상태 소음 수집 -> 완료 후 원래 상태(MONITORING 또는 READY)로 자동 복귀.
### 4. Trigger 변수 및 임계값(Threshold) 통폐합 설계
현재 HW(Wake), SW(RMS), SW(Bands)로 흩어진 트리거 설정은 유지하되, **런타임 상태를 관리하는 변수 구조체(Context)**를 하나로 통합하여 RTOS 태스크에서 관리하기 쉽게 만듭니다.
**[개선된 런타임 트리거 컨텍스트 (메모리상에만 존재)]**
```cpp
// 어떤 트리거에 의해 레코딩이 시작되었는지 추적
typedef enum {
    TRIG_SRC_NONE = 0,
    TRIG_SRC_HW_WAKE,    // 딥슬립 깨어남
    TRIG_SRC_SW_RMS,     // 전체 진동 초과
    TRIG_SRC_SW_BAND_0,  // 밴드 1 초과
    TRIG_SRC_SW_BAND_1,
    TRIG_SRC_SW_BAND_2,
    TRIG_SRC_MANUAL      // 사용자가 웹에서 START 누름
} EM_T20_TriggerSource_t;

typedef struct {
    bool                   is_triggered;      // 현재 레코딩 조건 충족 여부
    EM_T20_TriggerSource_t active_source;     // 트리거 발동 원인
    uint32_t               last_trigger_tick; // 마지막 트리거 발생 RTOS Tick
    uint32_t               hold_time_ms;      // 설정된 유지 시간 (Damping Time)
} ST_T20_TriggerContext_t;

```
### 5. 상태 머신 기반의 이벤트 처리 흐름 (RTOS Loop 구조)
상태 머신이 도입되면 메인 루프(T20_processTask)의 구조가 매우 간결해지고 예측 가능해집니다. if-else의 지옥에서 벗어날 수 있습니다.
**[상태 머신 적용 로직 의사코드 (Pseudo-code)]**
```cpp
void T20_processTask(void *pvParameters) {
    ST_T20_TriggerContext_t trig_ctx = {0};
    EM_T20_SysState_t current_state = SYS_STATE_INIT;

    // 모드에 따른 초기 상태 진입
    if (cfg.system.op_mode == OP_MODE_AUTO) current_state = SYS_STATE_MONITORING;
    else current_state = SYS_STATE_READY;

    for(;;) {
        // 1. 이벤트 큐 확인 (웹 UI 명령 수신)
        Event_t evt;
        if (xQueueReceive(event_queue, &evt, 0)) {
            switch(evt.cmd) {
                case CMD_MANUAL_START:
                    if(current_state == SYS_STATE_READY) current_state = SYS_STATE_MONITORING;
                    break;
                case CMD_MANUAL_STOP:
                    current_state = SYS_STATE_READY;
                    trig_ctx.is_triggered = false;
                    break;
                case CMD_LEARN_NOISE:
                    current_state = SYS_STATE_NOISE_LEARNING;
                    break;
            }
        }

        // 2. 상태별 동작 수행 (State Action)
        switch(current_state) {
            case SYS_STATE_READY:
                // 센서 딥슬립 진입 검사 또는 데이터 폐기 (Pre-trigger 링버퍼만 유지)
                break;

            case SYS_STATE_NOISE_LEARNING:
                // 노이즈 수집 로직 실행. 목표 프레임 도달 시 MONITORING으로 상태 변경
                if (dsp.learnNoise()) current_state = SYS_STATE_MONITORING;
                break;

            case SYS_STATE_MONITORING:
            case SYS_STATE_RECORDING:
                // DSP 파이프라인 가동 (공통)
                dsp.processFrame(raw_data, &feature);
                
                // 트리거 조건 검사 및 갱신
                _evaluateTriggers(&trig_ctx, &feature, cfg);

                if (trig_ctx.is_triggered) {
                    current_state = SYS_STATE_RECORDING;
                    storage.writeToFile(feature); // Pre-trigger 포함 기록
                } else {
                    // Hold Time이 지나면 자동으로 Monitoring으로 복귀
                    if (current_state == SYS_STATE_RECORDING) {
                        storage.closeFile(); 
                    }
                    current_state = SYS_STATE_MONITORING;
                }
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(1)); // Starvation 방지
    }
}

```
### 💡 개편 방안 총평 및 도입 이점
 1. **코드 응집도 향상:** 변수 통폐합을 통해 "트리거 조건 판별 로직(_evaluateTriggers)"과 "저장 로직(storage.write)"을 완벽히 분리할 수 있습니다.
 2. **확장성 보장:** 향후 **스케줄 모드**가 도입되더라도, CMD_SCHEDULE_WAKE 이벤트를 발생시켜 SYS_STATE_MONITORING으로 상태만 전이시키면 기존 코드를 한 줄도 수정할 필요가 없습니다.
 3. **디버깅 용이성:** 현재 상태(current_state) 변수 하나만 모니터링하면 기기가 왜 레코딩을 안 하는지, 왜 소음 학습에 멈춰있는지 100% 추적이 가능합니다.
이 설계안을 바탕으로 실제 T210_Def.h 및 Config 파싱 모듈에 적용하기 위한 소스 코드 작업을 바로 진행할까요?





### 1. ⚙️ 명시적 상태 머신 (FSM) 설계

기존의 단순 boolean 플래그들을 통폐합하여, 시스템 전체의 흐름을 관장하는 **메인 FSM**과 **서브 FSM**으로 분리합니다.  

#### **A. 메인 시스템 상태 (System State)**   
measurement_active를 대체하며, 시스템의 현재 행동을 명확히 정의합니다.
 * SYS_STATE_IDLE: 센서는 작동 중지 또는 초저전력 대기 상태. (딥슬립 진입 대기)
 * SYS_STATE_MONITOR: 센서 수집 및 DSP 연산은 진행하되, **저장(SD)은 하지 않고 트리거만 감시**하는 상태. (웹 UI 실시간 모니터링 유지)
 * SYS_STATE_RECORD: 수동 조작 또는 트리거 조건이 충족되어 **데이터를 스토리지에 기록** 중인 상태.
 * SYS_STATE_FAULT: 센서 먹통(Watchdog), SD카드 에러 등으로 인한 예외 상태.
 * 
#### **B. 노이즈 학습 상태 (Noise State)**  
 * NOISE_STATE_IDLE: 노이즈 제거 OFF 또는 초기화 상태.
 * NOISE_STATE_LEARNING: 환경 소음을 수집하여 프로필을 생성 중인 상태 (설정된 Frame 도달 시 자동 전환).
 * NOISE_STATE_ACTIVE: 학습된 노이즈 프로필을 바탕으로 스펙트럼 감산을 적용 중인 상태.
 * 
 
### 2. 🚀 시작 모드(Operation Mode) 구분 및 구조체 개선
단순 auto_start boolean을 없애고, 확장성을 고려한 열거형(Enum)으로 통폐합합니다.
```cpp
typedef enum {
    EN_T20_MODE_MANUAL = 0,    // 사용자 시작(버튼/Web) 시에만 RECORD 상태 진입
    EN_T20_MODE_AUTO_TRIGGER,  // 부팅 직후 MONITOR 상태 진입, 트리거 감지 시 자동 RECORD 진입
    EN_T20_MODE_SCHEDULE       // (향후) 특정 요일/시간대 조건 충족 시 MONITOR/RECORD 진입
} EM_T20_OpMode_t;

```
 * **적용 방안:** cfg.system.auto_start를 cfg.system.op_mode로 교체합니다.
 
 
### 4. 🔔 이벤트 유형(Event Type) 정형화
기존에는 문자열 하드코딩("smart_trigger_alert", "btn_start")으로 이벤트를 처리했습니다. 이를 정형화된 Event Enum으로 관리하여 MQTT와 Storage 로그의 일관성을 높입니다.
```cpp
typedef enum {
    EVT_SRC_MANUAL_UI = 1,     // 웹/앱에서 시작
    EVT_SRC_MANUAL_BTN,        // 하드웨어 버튼 누름
    EVT_SRC_TRIG_RMS,          // DSP 전체 RMS 초과
    EVT_SRC_TRIG_BAND_0,       // 주파수 밴드 0 초과
    EVT_SRC_TRIG_BAND_1,
    EVT_SRC_SYS_SCHEDULE,      // (향후) 스케줄러에 의한 시작
    EVT_SRC_SYS_WATCHDOG       // 에러/워치독 이벤트
} EM_T20_EventSource_t;

```
### 5. 🔄 제안하는 상태 전환(Transition) 흐름도
 * **[MANUAL 모드]**
   * 부팅 -> SYS_STATE_IDLE
   * 사용자 START 버튼 -> SYS_STATE_RECORD (저장 시작)
   * 사용자 STOP 버튼 -> SYS_STATE_IDLE (저장 종료)
 * **[AUTO_TRIGGER 모드]**
   * 부팅 -> SYS_STATE_MONITOR (버퍼 순환, 감시 중)
   * RMS/Band 임계값 초과 감지 -> EVT_SRC_TRIG_... 발생 -> SYS_STATE_RECORD 진입 (이벤트와 함께 스토리지 오픈)
   * 신호가 임계값 아래로 떨어짐 -> hold_time_ms 타이머 가동 -> 타이머 만료 시 SYS_STATE_MONITOR로 복귀
   * 모니터 상태에서 sleep_timeout_sec 초과 -> 센서 HW Wake 세팅 후 딥슬립.
### 💡 리팩토링 진행 방향 제안
이 구조를 도입하면 T230_Mfcc_Cor_220.cpp의 수많은 if(measurement_active)와 하드코딩된 5000ms 타이머가 하나의 깔끔한 **switch(sys_state) 문으로 정리**됩니다. 향후 스케줄 기능이 들어와도 case EN_T20_MODE_SCHEDULE: 하나만 추가하면 끝납니다.
이 방향에 동의하신다면, **① T210_Def_221.h의 구조체/Enum 통폐합 선언부 리팩토링**을 먼저 진행할까요, 아니면 **② 메인 코어의 State Machine 제어 로직 구조**부터 먼저 짜드릴까요?



- 전처리 단계 추가(hpf, lpf 구분)
