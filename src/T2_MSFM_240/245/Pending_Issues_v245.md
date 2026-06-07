# v245 잔여 구현 이슈 및 기술 검토 보고서

본 문서는 멀티모달(진동/소음) 융합 펌웨어 v245 개발 과정에서 구조적 설계 규모 및 의존성 문제로 보류되어 차후 비동기 고도화 단계에서 연계 진행할 잔여 이슈 3종의 현황과 구현 설계 가이드를 기록합니다.

---

## 1. [이슈 19] `accumulateFifo()` 3중 복사 → Zero-copy 링버퍼 직접 기입

### 현황 및 문제점
- **현재 구현**: BMI270 센서의 FIFO 데이터를 수집할 때, SPI 통신으로 읽어온 바이트 스트림을 파싱하여 스택 임시 배열(`v_batchX[128]`, `v_batchY[128]`, `v_batchZ[128]`)에 1차 적재합니다. 그 후 각 축별 링버퍼(`_accumX`, `_accumY`, `_accumZ`)의 `_accumWriteIdx` 위치에 2차 복사하고, 전체 수집 개수를 누적 계산합니다. 이 과정에서 메모리 3중 복사가 일어나 CPU 연산 사이클과 캐시 미스가 낭비됩니다.
- **최적화 방향**: SPI 통신 바이트 스트림 파싱 단계를 링버퍼에 쓰기 직전에 바인딩하고, 파싱된 센서 값을 즉시 링버퍼 메모리에 직접 기입하도록 리팩토링합니다.

### 구현 설계 가이드
1. **버스트 리드용 포인터 직접 제공**:
   - `accumulateFifo()` 내에서 바이트를 직접 파싱하여 링버퍼 인덱스를 증가시키며 기입합니다.
   ```cpp
   // T230_Sensor_245.cpp 가상 구현 가이드
   void CL_T2_Sensor::accumulateFifoDirect() {
       uint8_t v_fifoData[1024]; // SPI 버스트 리드 버퍼
       uint16_t v_bytesRead = _readFifoRaw(v_fifoData, sizeof(v_fifoData));
       
       uint16_t idx = 0;
       while (idx < v_bytesRead) {
           // BMI270 FIFO 헤더 및 프레임 타입 분석
           if (v_fifoData[idx] == FIFO_HEADER_VIB) {
               float ax = _parseAccelX(&v_fifoData[idx+1]);
               float ay = _parseAccelY(&v_fifoData[idx+3]);
               float az = _parseAccelZ(&v_fifoData[idx+5]);
               
               // 링버퍼에 Zero-copy 직접 쓰기
               _accumX[_accumWriteIdx] = ax;
               _accumY[_accumWriteIdx] = ay;
               _accumZ[_accumWriteIdx] = az;
               
               _accumWriteIdx = (_accumWriteIdx + 1) % ACCUM_BUF_SIZE;
               idx += 7; // 프레임 크기만큼 오프셋 이동
           } else {
               idx++;
           }
       }
   }
   ```
2. **주의사항**:
   - 링버퍼의 경계(Wrap-around) 조건 처리 시 배열 범위를 넘어가지 않도록 모듈러 연산(`% ACCUM_BUF_SIZE`) 및 경계 검증을 면밀히 수행해야 합니다.

---

## 2. [이슈 23] `accumulateFifo()` 내 비활성 축 Zero-bypass 적용 (이슈 19 의존)

### 현황 및 문제점
- **현재 구현**: 3축 가속도/자이로 센서 데이터 중 특정 축이 비활성화(설정의 `axis_mask`로 마스킹)되어 있어도 FIFO 통신 스트림을 전부 분석하고 파싱 및 복사 연산을 수행하고 있습니다.
- **최적화 방향**: 비활성화된 축은 파싱 연산 자체를 생략(Zero-bypass)하고 메모리 쓰기를 완전히 차단(Skip)하여 전력 소모 및 오디오 태스크 지연을 최소화합니다.

### 구현 설계 가이드
1. **축 마스크 결합에 따른 분기 차단**:
   - `_parse` 헬퍼 함수를 마스크별로 최적화하거나 파싱 직전 조건 검사로 통제합니다.
   ```cpp
   // 이슈 19의 파싱 루프 내부
   if (v_fifoData[idx] == FIFO_HEADER_VIB) {
       // 비활성 축 bypass
       if (_accelAxisMask & 0b001) {
           _accumX[_accumWriteIdx] = _parseAccelX(&v_fifoData[idx+1]);
       }
       if (_accelAxisMask & 0b010) {
           _accumY[_accumWriteIdx] = _parseAccelY(&v_fifoData[idx+3]);
       }
       if (_accelAxisMask & 0b100) {
           _accumZ[_accumWriteIdx] = _parseAccelZ(&v_fifoData[idx+5]);
       }
       
       _accumWriteIdx = (_accumWriteIdx + 1) % ACCUM_BUF_SIZE;
       idx += 7;
   }
   ```
2. **주의사항**:
   - SPI로 읽어오는 스트림 내의 바이트 오프셋(7바이트 프레임 단위)은 변하지 않으므로, 데이터 파싱을 Bypass 하더라도 스트림 포인터 `idx`의 누적 오프셋 계산이 어긋나지 않도록 유지해야 합니다.

---

## 3. [이슈 21] 캘리브레이션 파일 저장의 백그라운드 비동기(지연 큐) 분리

### 현황 및 문제점
- **현재 구현**: 진동/소음 캘리브레이션 동작 후, 센서 영점 및 오프셋 파라미터를 LittleFS 파일 시스템에 저장하는 함수(`saveCalibration()`)를 호출합니다. LittleFS Flash 쓰기 연산은 블로킹 방식으로 작동하여 수십~수백ms 동안 CPU 제어권을 독점하므로 오디오 스트리밍 I2S DMA 버퍼 언더런과 센서 FIFO 오버플로우를 유발하여 소리 끊김이나 부팅 오작동을 야기할 수 있습니다.
- **최적화 방향**: 파일 쓰기 요청을 지연 큐(Deferred Queue) 형태로 등록하고, 오디오/센서 처리가 없는 FreeRTOS 저우선순위(Idle 등) 백그라운드 태스크에서 비동기로 파일 쓰기를 안전하게 진행합니다.

### 구현 설계 가이드
1. **비동기 파일 저장 이벤트 큐 설계**:
   - FreeRTOS `QueueHandle_t`를 생성하여 캘리브레이션 저장 구조체 또는 파일 경로 포인터를 큐에 넣습니다.
   ```cpp
   // T280_Calibrator_245.cpp 비동기 가이드
   struct ST_CalibSaveEvent_t {
       char filepath[64];
       T2_Type::ST_CalibrationData_t data;
   };
   
   QueueHandle_t g_calibSaveQueue = NULL;
   
   // 캘리브레이션 완료 시 큐 전송
   void requestAsyncSaveCalib(const char* path, const T2_Type::ST_CalibrationData_t& data) {
       ST_CalibSaveEvent_t event;
       strncpy(event.filepath, path, sizeof(event.filepath));
       memcpy(&event.data, &data, sizeof(data));
       xQueueSend(g_calibSaveQueue, &event, 0); // 논블로킹 전송
   }
   ```
2. **백그라운드 파일 쓰기 워커 데몬 생성**:
   - `app_main` 또는 FSM 초기화 시점에 우선순위가 1~2 수준의 매우 낮은 백그라운드 저장 전용 태스크를 론칭합니다.
   ```cpp
   void calibSaveTask(void* pvParameters) {
       ST_CalibSaveEvent_t event;
       while (1) {
           if (xQueueReceive(g_calibSaveQueue, &event, portMAX_DELAY) == pdTRUE) {
               ESP_LOGI("CALIB_ASYNC", "Writing calibration data to %s...", event.filepath);
               // 실제 파일 시스템 쓰기 수행 (블로킹이 발생해도 저우선순위 태스크이므로 센서 태스크가 선점함)
               _executeFileWrite(event.filepath, &event.data);
           }
       }
   }
   ```
3. **주의사항**:
   - 비동기 쓰기가 수행되는 중에 기기가 강제 전원 차단(Brownout/Reset)될 경우 파일이 손상될 수 있으므로, 쓰기 동작 완료 전까지 진행 상태를 전역 플래그(`_isSavingCalib`)로 유지하고, 저장 중 리셋을 차단하는 세이프 가드를 확보해야 합니다.
