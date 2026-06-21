# [MSFM_T2_250] T230_Sensor 기능 규격서

본 문서는 **MSFM_T2_250** 임베디드 펌웨어의 멀티모달 센서 데이터 수집엔진인 `T230_Sensor` (Sensor Engine) 모듈에 대한 기능 규격서입니다.

---

## 1. 모듈 개요

`T230_Sensor`는 진동/관성 데이터를 출력하는 **BMI270 IMU 센서(SPI 통신)** 및 소음 음압을 수신하는 **ICS43434 MEMS 마이크(I2S 통신)**에 직접 액세스하여 실시간 스트림 데이터를 수집하는 드라이버/수집 엔진 모듈입니다. 수집된 데이터를 내부 고정 링버퍼(Ring Buffer)에 적재하고 오프셋 보정 및 스케일 팩터 변환을 담당합니다.

---

## 2. API 명세 (인터페이스 선언)

[T230_Sensor_250.hpp](../T230_Sensor_250.hpp) 클래스의 공개 API 규격은 다음과 같습니다.

### `CL_T2_SensorEngine(SPIClass& p_spiBus)`
*   **기능설명**: 모듈 생성자로, 하드웨어 SPI 버스 참조를 입력받아 등록하고 SPI 잠금용 Mutex Semaphore(`_spiLock`)를 초기 생성합니다.

### `bool init(const T2_Type::ST_Global_System_t& p_sysCfg, const T2_Type::ST_Accel_Config_t& p_accCfg, const T2_Type::ST_Gyro_Config_t& p_gyrCfg, const T2_Type::ST_Audio_Config_t& p_audCfg)`
*   **기능설명**: SPI 버스를 기동하고 BMI270 센서의 초기화 시퀀스(펌웨어 바이너리 업로드, ODR 설정, FIFO 모드 활성화)를 수행한 후, I2S 버스 드라이버를 기동하여 오디오 DMA 수집을 개시합니다.
*   **DMA 디스크립터 자동 계산**: I2S 초기화 시 `I2S_DMA_CHUNK_SIZE` 크기에 맞추어 `dma_desc_count = (dma_buffer_size + 4091) / 4092` 수식을 적용하여 디스크립터 개수를 자동 계산 및 분할 세팅합니다.
*   **반환값**: 모든 센서 장치 초기화 통과 여부 (`true` / `false`).

### `void pause(void)` / `void resume(void)`
*   **기능설명**: DMA 수집 및 내부 FIFO 폴링 처리를 일시 정지하거나 재개합니다.

### `void clearAudioBuffer(void)`
*   **기능설명**: I2S DMA 수집 대기열 및 마이크 수집 내부 캐시 큐를 제로 클리어하여 잔여 버퍼 데이터를 소거합니다.

### `uint32_t readAudioChunk(float* p_outL, float* p_outR, uint32_t p_reqSamples)`
*   **기능설명**: I2S 드라이버로부터 32비트 PCM 정수 음원 데이터를 DMA 인출하여 정규화된 부동소수점(`-1.0f ~ 1.0f`)으로 스케일 변환하고 좌우 스테레오 채널로 분리 복사합니다.
*   **L1 캐시 동기화**: DMA 전송 완료 후 L1 캐시 무효화 및 일관성 확보를 위해 `esp_cache_msync` API(`sync_cache_after_dma_write`)를 호출하여 물리 RAM에서 데이터를 강제 로드하도록 제어합니다.
*   **매개변수**:
    *   `p_outL`: L 채널 Float 데이터를 저장할 대상 버퍼 포인터.
    *   `p_outR`: R 채널 Float 데이터를 저장할 대상 버퍼 포인터.
    *   `p_reqSamples`: 청크당 요구 샘플 수.
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
*   **기능설명**: 장비 평탄 안착 상태에서 수동/자동 영점 오프셋 및 편차 이득(Gain) 보정 연산을 수행하고 그 결과를 파일시스템 내 설정 구조체에 저장 적용합니다.

### `void updateCalibrationOffsets(const float* offsets)`
*   **기능설명**: ConfigManager로부터 리로드된 보정 오프셋 데이터를 센서 멤버 변수에 즉각 핫스왑 반영합니다.

### `void prepareDeepSleepWakeup(float wake_g, uint16_t wake_dur)`
*   **기능설명**: 딥슬립 진입 전 BMI270을 Any-Motion 감지 모드로 전환하여 물리 충격 감지 시 INT2 핀 매핑(`_writeRegSingle(0x54, 0x04)`)을 활성화하고 임계값을 레지스터에 기록합니다.

### `void flushHardwareFifo()`
*   **기능설명**: FSM 상태 모드 스위칭 시 BMI270 하드웨어 버퍼에 누적되어 있던 이전 모드의 잔여 잔류 데이터를 FIFO 명령 레지스터(`0x5E`) 조작(값 `0xB0` 쓰기)을 통해 강제 플러시하여 특징량 오염을 원천 차단합니다.

### `void restoreFromDeepSleepWakeup()`
*   **기능설명**: 딥슬립 Wake-up 감지 모드로 동작하던 BMI270 센서의 Any-Motion 기능을 비활성화(`0x5F`, `0x60` 레지스터 0x00 작성)하고, 원래의 실시간 `MONITORING`을 위한 INT1 FIFO 워터마크 인터럽트 구조(`0x53` 레지스터에 `0x08` 작성)로 원상 복구합니다.

---

## 3. 핵심 설계 데이터 및 정렬

1.  **I2S 오디오 수집 설정 (ICS43434 규격)**:
    *   32비트 고해상도 샘플, 42kHz ODR (`Audio::Sensor::RATE_DEF`), 스테레오 구성, ESP32 내부 APLL 필수 활성화 (`USE_APLL_DEF` = `true`).
    *   **정적 스크래치 버퍼**: DMA 인터럽트 지연을 받지 않도록 `_audioDmaBuffer` 배열을 `alignas(16)` 16바이트 정렬 선언하여 PSRAM에 확보합니다.
    *   **L1 캐시 일관성 (Cache Coherency)**:
        *   DMA 전송 시작 전 메모리 영역 캐시 무효화 (`esp_cache_msync` with `ESP_CACHE_MSYNC_FLAG_INVALIDATE`)를 통해 캐시 일관성 데이터 오염을 예방합니다.
2.  **물리 스케일 변환 수식**:
    *   **가속도 변환**:
        $$\text{Value (G)} = \left(\text{Raw LSB} \times \frac{\text{Range G}}{32768}\right) \times \text{Gain} - \text{Offset}$$
    *   **자이로 변환**:
        $$\text{Value (dps)} = \left(\text{Raw LSB} \times \frac{\text{Range dps}}{32768}\right) \times \text{Gain} - \text{Offset}$$
3.  **SPI 트랜잭션 큐 스케줄러**:
    *   **경합 방지 및 우선순위 분배**:
        모든 SPI 읽기/쓰기 명령은 직접 하드웨어 API 호출이 차단되며, `ST_SpiTransaction_t` 스케줄러 큐를 경유하도록 강제됩니다.
        `IMU_FIFO_READ`에 최우선 순위(`EM_SpiPriority::IMU_FIFO_READ`)를 부여하여 실시간 특징량 추출 지연 및 지터를 극소화합니다.

---

## 4. 변경 및 갱신 이력 (Revision History)

*   **v2.50 (2026-06-21)**:
    *   `esp_cache_msync` 적용 L1 캐시 일관성 동기화 상세 규격 명세 추가.
    *   `ST_SpiTransaction_t` 우선순위 큐 기반 SPI 스케줄러 통합 설계 반영.
    *   `prepareDeepSleepWakeup` / `restoreFromDeepSleepWakeup` 레지스터 맵핑 제어 규격 동기화.
    *   I2S DMA 디스크립터 자동 계산/분할 초기화 로직 보완.
    *   캘리브레이션 핫스왑 업데이트 API (`updateCalibrationOffsets`) 추가.
