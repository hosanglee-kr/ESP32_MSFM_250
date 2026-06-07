# [MSFM-T240_v250] T230_Sensor 기능 규격서

본 문서는 **MSFM-T240_v250** 임베디드 펌웨어의 멀티모달 센서 데이터 수집엔진인 `T230_Sensor` (Sensor Engine) 모듈에 대한 기능 규격서입니다.

---

## 1. 모듈 개요

`T230_Sensor`는 진동/관성 데이터를 출력하는 **BMI270 IMU 센서(SPI 통신)** 및 소음 음압을 수신하는 **ICS43434 MEMS 마이크(I2S 통신)**에 직접 액세스하여 실시간 스트림 데이터를 수집하는 드라이버/수집 엔진 모듈입니다. 수집된 데이터를 내부 고정 링버퍼(Ring Buffer)에 적재하고 오프셋 보정 및 스케일 팩터 변환을 담당합니다.

---

## 2. API 명세 (인터페이스 선언)

[T230_Sensor_250.hpp](../T230_Sensor_250.hpp) 클래스의 공개 API 규격은 다음과 같습니다.

### `CL_T2_SensorEngine(SPIClass& p_spiBus)`
*   **기능설명**: 모듈 생성자로, 하드웨어 SPI 버스 참조를 입력받아 등록하고 SPI 잠금용 Mutex Semaphore(`_spiLock`)를 초기 생성합니다.

### `bool init(const T2_Type::ST_Global_System_t& p_sysCfg, const T2_Type::ST_Accel_Config_t& p_accCfg, const T2_Type::ST_Gyro_Config_t& p_gyrCfg, const T2_Type::ST_Audio_Config_t& p_audCfg)`
*   **기능설명**: SPI 버스를 기동하고 BMI270 센서의 초기화 시퀀스(펌웨어 바이너리 업로드, ODR 설정, FIFO 모드 활성화)를 밟은 후, I2S 버스 드라이버를 기동하여 오디오 DMA 수집을 개시합니다.
*   **반환값**: 모든 센서 장치 초기화 통과 여부 (`true` / `false`).

### `void pause(void)` / `void resume(void)`
*   **기능설명**: DMA 수집 및 내부 FIFO 폴링 처리를 일시 정지하거나 재개합니다.

### `void clearAudioBuffer(void)`
*   **기능설명**: I2S DMA 수집 대기열 및 마이크 수집 내부 캐시 큐를 제로 클리어하여 잔여 버퍼 데이터를 소거합니다.

### `uint32_t readAudioChunk(float* p_outL, float* p_outR, uint32_t p_reqSamples)`
*   **기능설명**: I2S 드라이버로부터 32비트 PCM 정수 음원 데이터를 DMA 인출하여 정규화된 부동소수점(`-1.0f ~ 1.0f`)으로 스케일 변환하고 좌우 스테레오 채널로 분리 복사합니다.
*   **매개변수**:
    *   `p_outL`: L 채널 Float 데이터를 저장할 대상 버퍼 포인터.
    *   `p_outR`: R 채널 Float 데이터를 저장할 대상 버퍼 포인터.
    *   `p_reqSamples`: 쳉크당 요구 샘플 수.
*   **반환값**: 실제 성공적으로 수집 완료된 샘플의 총개수.

### `uint16_t readVibFifoBatch(float* p_accX, float* p_accY, float* p_accZ, float* p_gyrX, float* p_gyrY, float* p_gyrZ, uint16_t p_maxFrames)`
*   **기능설명**: BMI270의 내부 FIFO 버퍼로부터 가속도/자이로 원시 레지스터 데이터를 대량으로 Burst-Read(SPI 통신)하여 물리 단위($G$, $dps$)로 변환 후 축별 출력 버퍼에 채워넣습니다.
*   **반환값**: 획득에 성공한 프레임(X/Y/Z 세트) 수.

### `uint16_t accumulateFifo(void)`
*   **기능설명**: `readVibFifoBatch`를 반복 호출하여 BMI270의 FIFO 내부를 완전 비운 후, 획득한 물리값들을 개별 3축 가속도/자이로 링버퍼(`_accumX/GX...`)의 쓰기 인덱스에 적재하고 누적 카운트를 상향 조정합니다.
*   **반환값**: 금회 루프에서 최종 축적된 총 진동 데이터 프레임 수.

### `uint16_t getAccumulatedAccel(float* p_outX, float* p_outY, float* p_outZ, uint16_t p_reqCount)` / `getAccumulatedGyro(...)`
*   **기능설명**: 내부에 누적되어 대기 중인 진동 링버퍼에서 요구 수량(`p_reqCount`)만큼의 X, Y, Z 데이터를 추출하여 전달하고 읽기 헤더 포인터를 이동시킵니다.

### `float readTemperatureSensor(void)`
*   **기능설명**: BMI270의 칩 온도 측정 레지스터(0x22, 0x23)를 1회 판독하여 섭씨($^\circ\text{C}$) 단위로 변환해 리턴합니다.
*   **변환 공식**: $T\text{ (}^\circ\text{C)} = \frac{\text{RawValue}}{512.0} + 23.0$

### `bool runAccelCalibration(void)` / `runGyroCalibration(void)`
*   **기능설명**: 장비 평탄 안착 상태에서 수동/자동 영점 오프셋 및 편차 이득(Gain) 보정 연산을 수행하고 그 결과를 파일시스템 내 `/sys/acc_calib_250.json` 및 `/sys/gyr_calib_250.json`에 저장 적용합니다.

### `bool applyStoredAccelCalibration(void)` / `applyStoredGyroCalibration(void)`
*   **기능설명**: 기 저장된 캘리브레이션 팩터 파일을 리드하여 메모리 내 오프셋 및 게인 변수에 즉각 로드 반영합니다.

### `bool enableWakeOnMotion(float p_threshG, uint16_t p_duration)`
*   **기능설명**: 딥슬립 및 유휴 절전 상태 진입을 대비하여 BMI270의 `Any-Motion` 센서 기능(인터럽트 2번 바인딩)을 켭니다. 지정 임계 가속도와 연속 시간 요건을 레지스터에 기록합니다.

---

## 3. 핵심 설계 데이터 및 정렬

1.  **I2S 오디오 수집 설정 (ICS43434 규격)**:
    *   32비트 고해상도 샘플, 42kHz ODR (`Audio::Sensor::RATE_DEF`), 스테레오 구성, ESP32 내부 APLL 필수 활성화 (`USE_APLL_DEF` = `true`).
    *   **정적 스크래치 버퍼**: DMA 인터럽트 지연을 받지 않도록 `_audioDmaBuffer` 배열을 `alignas(16)` 16바이트 정렬 선언하여 PSRAM에 확보합니다.
2.  **물리 스케일 변환 수식**:
    *   **가속도 변환**:
        $$\text{Value (G)} = \left(\text{Raw LSB} \times \frac{\text{Range G}}{32768}\right) \times \text{Gain} - \text{Offset}$$
    *   **자이로 변환**:
        $$\text{Value (dps)} = \left(\text{Raw LSB} \times \frac{\text{Range dps}}{32768}\right) \times \text{Gain} - \text{Offset}$$
3.  **SPI 우선순위 큐 및 저속 레지스터 16바이트 슬라이싱**:
    *   **경합 방지 및 우선순위 분배**:
        `EM_SpiPriority` 구조체 (`LOW_SETTING`, `HIGH_BURST`)를 이용해 통신 성격을 정의하고, 멀티코어 SPI 경합 방지용 뮤텍스 Semaphore `_spiLock`을 통해 버스 트랜잭션을 엄격히 동기화 제어합니다.
    *   **16바이트 슬라이싱 전송**:
        저속 레지스터 설정 전송(`LOW_SETTING`) 시 최대 16바이트 이하 단위로 강제 슬라이싱하여 전송함으로써 SPI 버스 독점 시간을 150µs 이내로 차단하고, 1600Hz FIFO 워터마크 인터럽트 수집 지연을 예방합니다.
    *   **초기화 예외 처리**:
        단, 센서 초기화(`_isBmiInit == false`) 시 대용량 마이크로코드 버스트 전송 구간에서는 예외적으로 16바이트 제한을 우회하여 초기화 정합성을 보장합니다.
