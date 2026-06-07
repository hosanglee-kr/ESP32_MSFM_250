Listed directory 242
Viewed T210_Def_242.hpp:1-395
Viewed T290_FsmMgr_242.hpp:1-131
Viewed T245_FeatExtra_242.hpp:1-86
Viewed T230_Sensor_242.hpp:1-69
Used MCP tool: sequential-thinking/sequentialthinking
Viewed T260_Storage_242.hpp:1-110
Viewed T270_Commu_242.hpp:1-66

분석한 소스 코드를 바탕으로 **ESP32 기반 멀티모달(진동+소음) AI 고장 진단 시스템 (Ver 2.4.2)**의 상세 기능 명세서(Functional Spec)를 작성해 드립니다.

이 문서는 `T210_Def_242.hpp`를 SSOT(Single Source of Truth)로 삼아 하드웨어 구성부터 소프트웨어 알고리즘, 데이터 처리 파이프라인을 상세히 기술합니다.

---

# [기능 명세서] 멀티모달 AI 고장 진단 시스템 (Ver 2.4.2)

## 1. 시스템 개요 (System Overview)
본 시스템은 진동(Vibration)과 소음(Acoustic) 데이터를 실시간으로 동시 수집 및 융합 분석하여, 기계 장치의 초기 결함을 탐지하고 AI 모델(TinyML)을 통해 상태를 분류하는 임베디드 진단 장치입니다.

## 2. 하드웨어 사양 (Hardware Specifications)
*   **Main MCU**: ESP32 (Dual-Core, 240MHz, PSRAM 탑재 모델 권장)
*   **Vibration Sensor**: **BMI270** (6-axis IMU)
    *   인터페이스: SPI (High Speed)
    *   샘플링 레이트: 1,600 Hz (Fixed)
    *   동작 모드: FIFO 배치를 통한 저전력/고속 수집, Any-Motion Wakeup 지원
*   **Acoustic Sensor**: **ICS43434** (MEMS Microphone)
    *   인터페이스: I2S (32-bit DMA)
    *   샘플링 레이트: 42,000 Hz
    *   채널: 2-CH (L/R) 수집 후 빔포밍(Beamforming) 지원
*   **Storage**: Micro SD Card (SD_MMC 방식) 및 내부 LittleFS

## 3. 핵심 기능 (Key Features)

### 3.1 멀티 태스크 데이터 처리 (Dual-Core Orchestration)
*   **Core 0 (Capture)**: I2S 및 SPI 센서 데이터 수집 전담 (Blocking I/O 최소화).
*   **Core 1 (Processing)**: DSP 연산, 특징량 추출, FSM 제어 및 통신 전담.
*   **PSRAM Tiling**: 대규모 데이터 처리를 위해 PSRAM에 Zero-allocation 메모리 풀을 구성하여 힙 파편화 방지.

### 3.2 DSP 엔진 (Digital Signal Processing)
*   **디지털 필터**:
    *   **FIR**: HPF/LPF (63-Taps), EQ 조정 가능.
    *   **IIR**: Notch Filter (60Hz 전원 노이즈 제거용), HPF/LPF.
    *   **Median Filter**: 센서 글리치 제거.
*   **노이즈 캔슬링**: 적응형 스펙트럴 서브트랙션(Adaptive Spectral Subtraction)을 통한 주변 소음 제거.
*   **오디오 전처리**: DC 제거, Pre-emphasis(0.97), 윈도우 함수(Hann/Hamming/Blackman).

### 3.3 특징량 추출 (Feature Extraction)
*   **진동(Vibration)**:
    *   **RMS**: X, Y, Z축 물리적 진폭.
    *   **Kurtosis(첨도)**: 베어링 초기 결함 등 충격성 신호 탐지.
    *   **Crest Factor**: 신호의 피크성 강도 측정.
    *   **Peak Frequency**: NMS(Non-Maximum Suppression)를 적용한 주요 주파수 피크 탐지.
    *   **Band Energy**: 8개 주파수 대역별 에너지 합산.
*   **소음(Acoustic)**:
    *   **Spectral Centroid**: 소리의 날카로움/음색 변화 추적.
    *   **MFCC (39-D)**: 13차 계수 + Delta + Delta-Delta를 포함한 AI 모델 입력용 텐서 생성.

### 3.4 데이터 저장 관리 (Asynchronous Storage)
*   **A-DSE (Async Data Storage Engine)**: 파일 기록 시 발생하는 지연(Latency)이 연산에 영향을 주지 않도록 링버퍼 기반 비동기 기록.
*   **Pre-trigger Recording**: 트리거 발생 전 **3초 분량**의 데이터를 상시 버퍼링하여 이벤트 발생 시점 이전 상황 보존.
*   **Circular Logging**: MB 단위 또는 분 단위 용량 제한을 통한 자동 순환 저장(Rotation).
*   **Binary Format**: 진동/소음을 통합한 `SMEA` 매직 코드가 포함된 고유 바이너리 규격 사용.

### 3.5 통신 및 인터페이스 (Connectivity)
*   **WebSocket Streaming**:
    *   실시간 텔레메트리(Telemetry), 스펙트럼(Spectrum), 파형(Waveform) 바이너리 스트리밍.
*   **MQTT**: 클라우드 서버로의 진단 결과 보고 및 원격 제어 명령(Command) 수신.
*   **Web Server**: 장치 설정 변경, 실시간 상태 모니터링 Dashboard 제공.
*   **OTA (Over-The-Air)**: 네트워크를 통한 펌웨어 업데이트 지원.

## 4. 시스템 상태 관리 (FSM States)
1.  **INIT**: 센서 및 파일 시스템 초기화.
2.  **READY**: 대기 상태 및 프리트리거 버퍼링.
3.  **MONITORING**: 실시간 특징량 분석 및 이상 징후 감시.
4.  **RECORDING**: 이상 탐지 또는 사용자 요청 시 SD 카드에 Raw 데이터 기록.
5.  **NOISE_LEARNING**: 주변 환경 노이즈 프로파일 학습 모드.
6.  **CALIBRATING**: 센서 영점 및 소프트웨어 이득 교정 모드.
7.  **ERROR**: 하드웨어 오류 발생 시 안전 모드 진입.

---

이 분석은 현재 프로젝트의 모든 소스 파일을 관통하는 최신 설계를 요약한 것입니다. 추가적으로 특정 모듈의 상세 구현 코드가 궁금하시면 말씀해 주세요.
