# Agent Guideline - ESP32_MSFM

이 문서는 **ESP32_MSFM (MSFM-T250)** 프로젝트의 아키텍처, 코드 작성 규칙 및 개발 환경을 정의한 AI 에이전트용 가이드라인입니다. 이 프로젝트를 분석, 수정 또는 확장할 때 아래 규칙을 반드시 준수하십시오.

---

## 1. 프로젝트 개요 (Project Overview)

- **목적**: ESP32-S3 기반의 **Vib-Audio(진동-소음) 멀티모달 진단 시스템 (MSFM-T250)** 개발
- **플랫폼 및 프레임워크**: PlatformIO / Arduino (espressif32)
- **타겟 하드웨어**:
  - MCU: ESP32-S3 (N16R8 또는 Zero/Super Mini)
  - 진동 센서: BMI270 (SPI 통신, 1.6kHz FIFO 수집)
  - 소음 센서: ICS43434 (I2S 통신, 32bit PCM, 42kHz 스테레오 수집)
- **주요 기능**:
  - **4-Tier 도메인 격리**: Global(인프라), Shared(공통), Vib(진동), Audio(소음)
  - **신호 처리 및 DSP**: `esp-dsp`를 사용한 FFT, FIR, Notch, HPF 필터링
  - **특징 추출**: RMS, Kurtosis, Crest Factor, Skewness, MFCC(13차원 + Delta/Double Delta)
  - **데이터 로깅 및 통신**: SD 카드/플래시 메모리 로테이션 저장, Wi-Fi, WebSocket, MQTT 전송

---

## 2. 디렉토리 구조 및 역할 (Directory Structure)

현재 프로젝트의 메인 로직은 활성화된 특정 버전 폴더(`src/T2_MSFM_250/[Version]`)를 기준으로 컴파일 및 개발이 수행됩니다.

```text
ESP32_MSFM_110/
├── .agent/                 # 에이전트 스킬 및 설정 정보
├── .pio/                   # 빌드 아웃풋 및 캐시
├── lib/                    # 외부/커스텀 라이브러리 (esp-dsp 등)
├── shared/                 # 빌드 스크립트 등 공유 리소스
├── src/                    # 소스 코드 루트
│   ├── T2_MSFM_250/        # MSFM-T250 진단 시스템
│   │   └── [Version]/      # 현재 활성화된 버전 소스 코드 (핵심 개발 대상, 예: 250, 251 등)
│   ├── T4_MC_013/          # 보조/대체 시스템 (현재 비활성)
│   └── main.cpp            # 펌웨어 메인 진입점 (T2_init, T2_run 호출)
├── platformio.ini          # 프로젝트 빌드 및 보드 설정 파일
└── agent.md                # 본 가이드라인 파일
```

### T2_MSFM 핵심 모듈 목록 및 역할 (활성 버전 디렉토리 `src/T2_MSFM_250/[Version]/` 기준)

- `T200_Main_[Version].h`: `main.cpp` 연동 진입점 (`T2_init()`, `T2_run()`) 및 외부 인터럽트(ISR) 처리
- `T210_Def_[Version].hpp`: 4-Tier 시스템 설정 상수를 정의한 SSOT (Single Source of Truth)
- `T215_Type_[Version].hpp`: 구조체, ENUM 등 명시적 데이터 타입 정의
- `T220_CfgMgr_[Version].hpp` / `.cpp`: JSON 기반 런타임 동적 설정 저장/관리자
- `T230_Sensor_[Version].hpp` / `.cpp`: SPI(BMI270) 및 I2S(ICS43434) 센서 데이터 수집 엔진
- `T240_DspEng_[Version].hpp` / `.cpp`: `esp-dsp`를 사용한 FFT 및 디지털 필터(HPF, Notch) 처리 엔진
- `T245_FeatExtra_[Version].hpp` / `.cpp`: 시간/주파수 영역 특징량 및 MFCC 추출 엔진
- `T248_SeqBuild_[Version].hpp` / `.cpp`: 시퀀스 프레임 구성 모듈
- `T250_TriggerEng_[Version].hpp` / [.cpp]: 임계치 판정 및 스마트 트리거 이벤트 엔진
- `T260_Storage_[Version].hpp` / `.cpp`: SD/플래시 지연 쓰기(Lazy Write) 및 파일 로테이션 저장소 매니저
- `T270_Commu_[Version].hpp` / `.cpp`: Wi-Fi AP/STA, WebSocket, MQTT 네트워크 서비스 매니저
- `T280_Calibrator_[Version].hpp` / `.cpp`: 센서 캘리브레이션 모듈
- `T290_FsmMgr_[Version].hpp` / `.cpp`: 전체 시스템 FSM(Finite State Machine) 제어 및 오케스트레이터 (싱글톤)

---

## 3. 개발 및 빌드 환경 (Build & Development)

이 프로젝트는 PlatformIO CLI를 활용하여 빌드 및 업로드를 제어합니다.

### 3.1 주요 빌드 환경 (Environments)
- `esp32-s3-n16r8`: ESP32-S3 DevKit-like 보드. 16MB Flash, 8MB PSRAM (기본 환경)
- `esp32-s3-zero`: ESP32-S3 Zero/Super Mini 보드. 4MB Flash, 2MB PSRAM

### 3.2 주요 CLI 명령어
- **빌드**: `pio run -e esp32-s3-n16r8`
- **업로드**: `pio run -e esp32-s3-n16r8 -t upload`
- **시리얼 모니터**: `pio device monitor`
- **클린**: `pio run -t clean`

---

## 4. 코드 구현 및 아키텍처 규칙 (Architecture Rules)

### 4.1 4-Tier 도메인 격리 준수
- 상수는 반드시 `T2_Def::[Global | Shared | Vib | Audio]` 네임스페이스 아래에 위치해야 합니다.
- **정적 한계값(`_MAX`)**과 **동적 기본값(`_DEF`)**을 구분하여 메모리 오염(OOM)을 방지하고 런타임 제어 유연성을 확보하십시오.

### 4.2 C++17 표준 및 빌드 플래그 준수
- 프로젝트는 C++17(`-std=gnu++17`) 표준을 준수합니다.
- 중복 인클루드를 방지하기 위해 헤더 상단에 `#pragma once`를 명시하십시오.
- 헤더 선언 시 다중 정의(Multiple Definition) 오류를 막기 위해 함수 선언은 클래스 내부 또는 `inline`으로 처리하십시오.

### 4.3 리소스 및 메모리 제약 관리 (OOM 방어)
- 힙 메모리 동적 할당(`malloc`, `new`)을 루프 내에서 가급적 금지하고, 초기화 시점에 정적 크기로 할당하여 사용하십시오.
- I2S DMA 버퍼나 신호 처리용 배열은 SIMD 연산 최적화를 위해 **`alignas(16)`** 정렬을 사용해야 합니다.
- 파일 시스템 및 SD 카드는 수명을 보호하고 성능 저하를 예방하기 위해 **지연 쓰기(Lazy Write)** 및 **로테이션 정책**을 준수하십시오.

### 4.4 하드웨어/RTOS 제어 규칙
- FreeRTOS 태스크 내에서 블로킹 대기가 필요할 경우, `delay()` 대신 **`vTaskDelay(pdMS_TO_TICKS(ms))`** 또는 `vTaskDelay(ticks)`를 사용하여 스케줄러가 양보될 수 있도록 하십시오.
- 인터럽트 서비스 루틴(ISR)은 **`IRAM_ATTR`** 지시어를 부착하고 최대한 간결하게 처리해야 합니다.

---

## 5. 작업 시 권장 절차 (Recommended Workflow)

1. **빌드 유효성 확인**: 코드 변경 후 반드시 `pio run`을 실행하여 컴파일 오류가 없는지 검증하십시오.
2. **헤더 포함 관계 주의**: `T210_Def_[Version].hpp` 및 `T215_Type_[Version].hpp`를 Single Source of Truth(SSOT)로 활용하여 의존성 순환을 방지하십시오.
3. **버전 정합성**: 소스 코드 수정 시 수정 대상을 빌드 소스 필터(`build_src_filter`)에 등록된 활성 버전(`src/T2_MSFM_250/[Version]`) 내부 파일로 제한하십시오.

---

## 6. 코드 탐색 및 아키텍처 파악 원칙 (타협 불가)
파일을 직접 읽기(`Read`, `Grep`) 전에 반드시 아래의 도구를 목적에 맞게 먼저 사용해야 합니다.

* **거시적 구조 파악 (`graphify` 사용):** 프로젝트 폴더 구조, 파일 간의 종속성(include 구조), 전반적인 아키텍처를 파악할 때 사용합니다. (지식 그래프 데이터는 `src/T2_MSFM_250_wiki/T2_MSFM_250_wiki_graphify_out/graphify-out/graph.json` 을 기준으로 탐색하며, 프로젝트 루트의 `graphify-out` 은 사용하지 않습니다.)
* **미시적 흐름 파악 (`codegraph` 사용):** 특정 센서 제어 함수(예: ESP32 I2C 통신 함수), 객체의 호출 흐름, 심볼의 정의 및 참조 관계를 추적할 때 사용합니다.

---

## 7. CodeGraph 사용 규칙

- 코드베이스 탐색이 필요할 때는 반드시 CodeGraph 도구를 최우선으로 사용한다.
- 먼저 `codegraph_explore`를 호출하여 관련 심볼과 소스 코드, 관계를 한 번에 가져온다.
  - 단, `codegraph_explore`는 메인 세션에서 직접 호출하지 말고 **Explore 에이전트**를 생성하여 실행해야 컨텍스트 오염을 방지할 수 있다.
- `codegraph_explore`가 반환한 소스 코드는 완전한 것으로 간주하고, 파일을 다시 읽지 않는다.
- 추가 정보가 꼭 필요할 때만 `grep`, `glob`, `read` 도구를 보조로 사용한다.
- 변경 영향도를 파악해야 할 때는 `codegraph_impact` 또는 `codegraph_affected`를 사용한다.
- 파일 구조가 궁금할 때는 `codegraph_files`로 확인한다.

---

## 8. 지식 인덱스 및 코드 인텔리전스 관리 규칙 (Knowledge Indexing & Code Intelligence)

이 프로젝트는 코드 아키텍처 탐색 및 분석 시간을 단축하기 위해 **Graphify 지식 그래프**와 **CodeGraph 코드 인텔리전스**를 활성화하여 사용합니다. 후속 작업 시 아래 가이드를 준수해 인덱스의 일관성을 유지하십시오.

### 8.1 Graphify 지식 그래프 & Wiki (`src/T2_MSFM_250_wiki/`)
- **역할**: 프로젝트 핵심 코드 구조와 문서들의 관계를 매핑한 옵시디언(Obsidian) 노트 및 한글 위키(Wiki)가 이 디렉토리에 관리됩니다.
  - **기본 출력 및 참조 경로**: 최종 그래프 산출물 및 분석 메타데이터는 프로젝트 루트가 아닌 `src/T2_MSFM_250_wiki/T2_MSFM_250_wiki_graphify_out/graphify-out/` 경로로 일원화되어 저장 및 보존됩니다.
- **적용 필터**:
  - `SensorFusion` 및 `VectorQuaternionMatrix` 모듈의 노드/관계는 제외됩니다.
  - `esp-dsp` 및 `SparkFun BMI270 Arduino Library` 모듈은 인터페이스 파악을 위해 `.h`, `.hpp`, `.md` 파일에 관련된 항목만 유지되며 소스코드 파일(`.c`, `.cpp`)들은 필터링에서 제외됩니다.
- **개별 임시 폴더 자동 삭제**: 각 라이브러리 및 소스 디렉토리 하위에 산발적으로 생성되는 임시 `graphify-out` 디렉토리는 분석 후 중복 방지를 위해 삭제되어야 합니다. (프로젝트 루트의 임시 `graphify-out` 역시 동기화 및 갱신에 사용하지 않으므로 삭제됩니다.)
- **갱신 및 유지**: 그래프 및 위키 데이터를 업데이트할 때는 [update_wiki.py](file:///c:/2540_Work/ESP32_MSFM_110/shared/graphify/update_wiki.py) 스크립트를 실행하십시오. (기존 군집 한글 레이블을 다수결 매핑 방식으로 자동 상속하며, 최종 산출물은 위의 위키 하위 `graphify-out` 경로에만 보존됩니다.)

### 8.2 CodeGraph 코드 인텔리전스 (`.codegraph/`)
- **역할**: 로컬 SQLite 데이터베이스(`codegraph.db`)를 기반으로 심볼 정의, 호출 흐름 및 참조 관계를 색인하여 에이전트의 코드 인텔리전스를 지원합니다.
- **GitHub 동기화 필수 원칙**:
  - 실제 빌드에 관여하는 모든 소스코드(`.c`, `.cpp`, `.S` 등)는 온전히 GitHub 저장소에 추적 및 동기화(Commit & Push)되어야 합니다.
  - 따라서 **`.gitignore` 또는 `.git/info/exclude`에 인덱스 제외 목적의 소스코드 무시 규칙을 절대 등록하지 마십시오.**
- **데이터베이스 전용 필터링 (후처리)**:
  - CodeGraph 인덱스 갱신 시, GitHub 동기화를 방해하지 않으면서도 DB 내부에서만 불필요한 데이터를 제외하기 위해 [update_codegraph.py](file:///c:/2540_Work/ESP32_MSFM_110/shared/graphify/update_codegraph.py) 스크립트를 사용하여 갱신해야 합니다.
  - 이 스크립트는 `codegraph sync`를 통해 변경 사항을 색인한 직후, DB 내부에서 `SensorFusion`, `VectorQuaternionMatrix` 및 `esp-dsp`/`SparkFun` 의 non-header 파일 관련 노드/엣지 정보만을 SQL `DELETE` 처리하여 정제하고 `VACUUM`을 통해 데이터베이스 최적화를 완료합니다.



