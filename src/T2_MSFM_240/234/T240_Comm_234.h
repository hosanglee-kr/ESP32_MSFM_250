/* ============================================================================
 * File: T240_Comm_234.h
 * Summary: Network, Web, MQTT & OTA Communication Engine (FSM Synchronized)
 * * [AI 메모: 제공 기능 요약]
 * 1. WiFi 연결 관리 (AP 단독, STA, Auto-Fallback 다중 AP 순회 접속).
 * 2. NTP 시간 동기화 (KST 타임존 적용).
 * 3. ESPAsyncWebServer 기반 비동기 REST API (설정, 제어, 파일 스트리밍).
 * 4. AsyncWebSocket 기반 실시간 파형/스펙트럼/MFCC 바이너리 브로드캐스트.
 * 5. MQTT 발행(Publish) 및 자동 재접속 유지보수.
 * 6. OTA(Over-The-Air) 무선 펌웨어 업데이트 지원.
 * * [AI 메모: 구현 및 유지보수 주의사항]
 * 1. ESPAsyncWebServer의 콜백(Lambda) 함수들은 메인 루프가 아닌 별도의 
 * 비동기 태스크(AsyncEvent) 컨텍스트에서 실행됩니다. 따라서 내부 구현체(ST_Impl)를 
 * 직접 제어하지 않고, 반드시 g_t20->postCommand()를 통해 FSM 큐로 명령을 하달해야 
 * Race Condition과 논리적 데드락을 원천 차단할 수 있습니다.
 * 2. broadcastBinary() 호출 시 전송 버퍼의 크기가 너무 크면 내부 큐가 가득 차 
 * 전송이 누락될 수 있으므로 availableForWriteAll() 검사를 반드시 유지하세요.
  * * * * [AI 셀프 회고 및 구현 원칙 - Phase 4: 비동기 통신 및 네트워크 방어] * * *
 * * [방어 1: 네트워크 페이로드 절단 및 좀비 소켓(OOM) 방어]
 * - 실수: 웹소켓 전송 시 sizeof(float)를 이중으로 곱해 메모리 경계 초과 참조(LoadProhibited) 패닉 발생.
 * - 원칙: 메인 태스크에서 이미 바이트(Byte)로 계산했으므로 통신 모듈은 순수 바이트 크기만 취급할 것.
 * - 실수: 통신 불량으로 죽은 클라이언트가 누적되어 TCP 버퍼 고갈 및 OOM 발생.
 * - 원칙: 메인 네트워크 루프에서 _ws.cleanupClients()를 주기적으로 호출하여 닫힌 소켓을 강제 회수할 것.
 * * [방어 2: 비동기 스레드 블로킹(데드락) 및 상태 머신 파괴 차단]
 * - 실수: 웹 콜백 내부에서 delay()나 ESP.restart()를 호출하여 비동기 스레드가 마비되고 응답 발송이 누락됨.
 * - 원칙: 콜백 내 지연(Delay)을 금지하고, 재부팅은 EN_T20_CMD_REBOOT 커맨드로 메인 FSM에 위임할 것.
 * - 실수: 대용량 설정 청크(Chunk) 업로드 시 OOM 발생 시 매 조각마다 send(500)을 호출해 HTTP 엔진이 파괴됨.
 * - 원칙: 에러 발생 시 메모리 포인터 상태만 마킹하고, 응답(send)은 최종 청크(final == true)에서 1회만 수행할 것.
 * * [방어 3: OTA 파티션 락(Lock) 및 SD 병목(I/O 마비) 방어]
 * - 실수: 다중 클라이언트가 동시에 OTA를 시도하면 플래시 메모리 경합으로 기기가 영구 벽돌(Brick)화 됨.
 * - 원칙: 정적 플래그(g_ota_running)를 도입하여 동시 다발적인 OTA 접근을 HTTP 423 Locked로 방어할 것.
 * - 실수: SD 카드 맹렬히 기록(RECORDING) 중 대용량 파일 다운로드 요청 시 SPI 버스 경합으로 사고 데이터 증발.
 * - 원칙: FSM 상태가 RECORDING일 경우 다운로드 API 접근 시 즉각 HTTP 423 Locked로 차단하여 I/O를 보호할 것.

 * ========================================================================== 
 */
 
#pragma once

#include <WiFi.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <PubSubClient.h>

#include "T210_Def_234.h"

class CL_T20_CommService {
   public:
	CL_T20_CommService();
	~CL_T20_CommService();

	// 네트워크 및 MQTT 초기화
	bool begin(const ST_T20_Config_t& cfg);

	// REST API 및 WebSocket 핸들러 라우팅
	void initHandlers(void* p_master_impl);

	// 실시간 바이너리 데이터 브로드캐스트 (WS, OOM 방어 적용)
	// B8/B14 방어: 요소 개수가 아닌 정확한 바이트(Bytes) 단위로 수신하여 절단 방지
	void broadcastBinary(const void* p_buffer, size_t bytes);
	
	// C19/C21 방어: MQTT뿐만 아니라 WiFi 재연결, WS 클린업을 총괄하도록 이름 및 기능 확장
	void runNetwork();
	bool publishMqtt(const char* sub_topic, const JsonDocument& doc);

	// 상태 확인 유틸리티
	bool isConnected() const {
		return WiFi.status() == WL_CONNECTED;
	}
	uint32_t calcStatusHash(uint32_t frame_id, uint32_t rec_count, bool measuring);

   private:
    // CORS 및 JSON 응답 헬퍼
	void _setCorsHeaders(AsyncWebServerResponse* response);
	void _sendJson(AsyncWebServerRequest* request, const JsonDocument& doc);

	void _reconnectMqtt(); // MQTT 재접속 로직

   private:
	AsyncWebServer _server;
	AsyncWebSocket _ws;

	// MQTT 및 통신 자원
	WiFiClient          _wifi_client;
	PubSubClient        _mqtt_client;
	ST_T20_ConfigMqtt_t _mqtt_cfg;
	uint32_t            _last_mqtt_retry_ms;
	// C19/C24 방어: Non-blocking WiFi 연결 유지보수를 위한 타이머
	uint32_t            _last_wifi_retry_ms;
};

