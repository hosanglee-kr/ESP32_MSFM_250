# [MSFM_T2_250] T270_Commu 기능 규격서

본 문서는 **MSFM_T2_250** 임베디드 펌웨어의 멀티플렉싱 유무선 통신 제어엔진인 `T270_Commu` (Commu Manager) 모듈에 대한 기능 규격서입니다.

---

## 1. 모듈 개요

`T270_Commu` 모듈은 디바이스 외부의 SCADA 관제 프레임워크 또는 모니터링 웹 브라우저 단말과의 상호작용을 처리하는 통신 허브 모듈입니다. **Wi-Fi 인프라(STA/AP 모드 및 자동 폴백)**, REST API 호출 수신용 **WebServer**, 초당 수십 회의 고밀도 바이너리 스트리밍을 감당하는 **AsyncWebSocket** 및 신뢰성 진단 데이터 발행을 담당하는 **ESP-IDF Native MQTT Client**를 제어합니다.

---

## 2. API 명세 (인터페이스 선언)

[T270_Commu_250.hpp](../T270_Commu_250.hpp) 클래스의 주요 공개 API 및 콜백 규격은 다음과 같습니다.

### `CL_T2_CommuManager(void)`
*   **기능설명**: 생성자로, 웹서버 포트(기본 80포트) 및 웹소켓 진입 엔드포인트 `/ws`를 할당 매핑하고 내부 상태 플래그를 미실행 상태로 둡니다.

### `bool init(void)`
*   **기능설명**: 통신 인프라를 구동합니다.
*   **동작규격**:
    1.  `ST_DynamicConfig_t` 내에 구성된 와이파이 설정에 입각하여 AP모드 또는 STA모드로 Wi-Fi 모듈을 활성화합니다. 기 지정된 AP 연결 불가 시 2차, 3차 AP로 교대 접근하는 `AUTO_FALLBACK` 폴백 메커니즘을 가동합니다.
    2.  NTP 서버 동기화를 지시하여 장치 로컬 실시간 시계(RTC)를 동기화하고 타임존을 설정합니다.
    3.  웹 핸들러(`_initWebHandlers()`)를 바인딩하여 브라우저 호출 REST 엔드포인트를 마운트하고 웹서버를 개시합니다.
    4.  MQTT 브로커 주소 정보가 설정되어 있는 경우, IDF Native MQTT 클라이언트를 구성하고 이벤트 콜백(`_mqttEventHandler`)을 할당한 후 백그라운드 스레드로 기동합니다.
*   **반환값**: 통신 초기 인프라 셋업 정상 통과 여부.

### `void runNetwork(void)`
*   **기능설명**: 메인 루틴의 백그라운드 관리 스레드에서 주기적으로 호출됩니다. 와이파이 물리 연결 유실 시 자동 재결합(Reconnection) 스캔 시도를 유발하며, MQTT 브로커와의 커넥션 실패 시 `_lastMqttRetryMs` 시간을 추적하여 `10초` 주기로 백그라운드 재접속을 오케스트레이션합니다.
*   **TLS 데드락 예방 가드**: WiFi 연결 직후 잘못된 시스템 연도(1970년) 상태에서 MQTT TLS 인증 실패 및 핸드셰이크 차단(데드락)을 방지하기 위해, NTP 동기 획득 완료(`timeinfo.tm_year > 120`) 시점까지 MQTT 보안 TLS 핸드셰이크 클라이언트 시작(`esp_mqtt_client_start`)을 보류합니다.

### `void processNetworkRestart(void)`
*   **기능설명**: 설정 변경 및 네트워크 재연결 핫스왑 명령(`CMD_RESTART_NETWORK`) 수신 시 리부팅 없이 통신 데몬만 안전하게 재연결을 시도합니다. MQTT 클라이언트를 중지시키고, WiFi 연결을 해제한 후 100ms 대기 후 변경된 설정을 독출하여 재기동 루틴을 수행합니다.

### `void recreateMqttClient(const esp_mqtt_client_config_t& new_cfg)`
*   **기능설명**: 기존 MQTT 클라이언트 리소스를 안전하게 소멸(`esp_mqtt_client_destroy`)한 후 신규 설정 기반 인스턴스로 재생성하여 커넥션 핸들 누수 및 충돌을 차단합니다. 생성 후 `_mqttConnected = false` 플래그를 리셋하여 재연결 루프가 구동되도록 유도합니다.

### `void broadcastBinary(const void* p_buffer, size_t p_bytes)`
*   **기능설명**: WebSocket 클라이언트들에게 고속 바이너리 메트릭 패킷(파형, 텔레메트리 등)을 일제 브로드캐스팅 전송합니다.
*   **SD I/O 에러 시 Mute 정책**: 시스템 장애로 FSM이 `ERROR` 상태로 전이된 시점에는 실시간 특징량 이송을 안전하게 차단(Mute)하여 소켓 버퍼 폭주 및 OOM을 예방합니다.
*   **성능가드**: 수신 측 브라우저의 전송 지연이나 느린 패킷 처리로 인해 ESP32 내부에 소켓 버퍼 백로그(Backlog)가 대량 누적되는 것을 방지하기 위해, 웹소켓 큐 프레임 제한 및 전송 에러 시 자동 연결 끊기 가드를 동시 수행합니다.

### `void publishResultMqtt(const char* p_topic, const T2_Type::ST_FeatureSlot_Vib_t& p_data)`
*   **기능설명**: 특징량 및 판정 결과를 JSON으로 패킹하여 MQTT 지정 토픽으로 발행(Publish)합니다.
*   **SD I/O 에러 긴급 패킷**: SD 카드 물리 쓰기 에러(`hasIoError`) 감지 시, 일반 바이너리 스트리밍 발행을 즉시 차단(Mute)하고 스토리지 에러 긴급 패킷(Severity: CRITICAL)을 최상위 토픽에 1회 단독 발행하여 가시성을 보장합니다.
*   **메모리 최적화 (ArduinoJson V7)**:
    *   공용 `JsonDocument`인 `_commuDocPool`을 웹소켓/웹서버 병렬 보호를 위해 `_commuLock` 뮤텍스로 잠근 상태에서 팩킹에 활용합니다.
    *   사용 직후 `_commuDocPool.shrinkToFit()`을 호출하여 메모리를 획득 평탄화합니다.

---

## 3. 핵심 통신 규격 및 보안(CORS) 정책

1.  **CORS 헤더 구성 (`_setCorsHeaders`)**:
    외부 웹 단말 브라우저에서 직접 REST API 포트를 찌를 수 있도록 모든 HTTP 응답 헤더에 아래 필드를 인젝트합니다.
    *   `Access-Control-Allow-Origin: *`
    *   `Access-Control-Allow-Methods: GET, POST, PUT, OPTIONS`
    *   `Access-Control-Allow-Headers: Content-Type`
2.  **TCP Keep-Alive 옵션 강제화 (`_enforceTcpKeepAlive`)**:
    소켓 포트 연결 해제 처리가 유실되어 무한 링거 상태로 메모리(SRAM) 소켓 커넥션 구조체가 누수되는 "좀비 소켓" 현상을 막기 위해 소켓 FD 획득 즉시 아래의 커널 옵션을 소켓 옵션(`setsockopt`)에 바인딩 처리합니다:
    *   `SO_KEEPALIVE` 활성화
    *   `TCP_KEEPIDLE` = 60초 (유휴 대기 시 첫 프로브 발생)
    *   `TCP_KEEPINTVL` = 10초 (프로브 간격)
    *   `TCP_KEEPCNT` = 3회 (3회 미응답 시 즉시 소켓 파괴)

---

## 4. 변경 및 갱신 이력 (Revision History)

*   **v2.50 (2026-06-21)**:
    *   클래스명을 `CL_T2_CommuManager`로 현행화.
    *   ArduinoJson V7 표준 규격 `JsonDocument` 및 `_commuLock` 뮤텍스 적용 텔레메트리 팩킹 반영.
    *   `CMD_RESTART_NETWORK` 네트워크 재설정 핫스왑 재연결 및 `recreateMqttClient` 인스턴스 재생성 설계 반영.
    *   SD 카드 물리 에러 발생 시 스트림 Mute 및 Alarm CRITICAL 패킷 최우선 단독 송출 정책 추가.
