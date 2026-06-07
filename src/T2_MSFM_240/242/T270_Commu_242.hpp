/* ============================================================================
 * File: T270_Commu_242.hpp
 * Summary: Network, Web, MQTT & OTA Communication Engine (Full-Stack Integrated)
 * ============================================================================
 * [로직 상태 추적 (Logic State Tracker)]
 * - 유지: T2의 Multi-AP 순회 접속 및 T4의 바이너리 멀티플렉싱 WebSocket 스트리밍.
 * - 갱신: [완전체] MQTT 구독(Command) 기능 및 정적 웹 리소스 서비스(serveStatic) 복원.
 * - 신규: [원칙 준수] LwIP TCP Keep-Alive 옵션을 통한 좀비 소켓 원천 차단.
 *
 * [축소/누락 방어 체크리스트 (Omission Defense)]
 * 1. [RTOS 방어]: 모든 delay()를 vTaskDelay()로 교체하여 스케줄러 락업 방지.
 * 2. [비동기 방어]: 웹/MQTT 콜백에서 dispatchCommand()를 통해 FSM으로 제어 위임.
 * 3. [메모리 방어]: WebSocket 송신 큐가 4KB 초과 시 클라이언트를 강제 종료하여 OOM 차단.
 * 4. [보안 방어]: CORS 헤더 및 API별 HTTP 상태 코드(423 Locked 등) 명시적 적용.
 * ========================================================================== */
#pragma once

#include "T210_Def_242.hpp"
#include <WiFi.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <mqtt_client.h>

class CL_T2_Communicator {
private:
    AsyncWebServer _server;
    AsyncWebSocket _ws;
    WiFiClient     _wifiClient;

    esp_mqtt_client_handle_t _mqttHandle = nullptr;

    uint32_t _lastMqttRetryMs = 0;
    uint32_t _lastWifiRetryMs = 0;
    bool     _isOtaRunning = false;

    struct MqttCredentials {
        char broker[64];
        bool enable;
        uint16_t port;
    } _mqttCreds;

    // 내부 유틸리티 및 핸들러
    void _enforceTcpKeepAlive();
    void _initWebHandlers();
    void _setCorsHeaders(AsyncWebServerResponse* p_response);
    void _sendJsonResponse(AsyncWebServerRequest* p_request, const JsonDocument& p_doc);

    // MQTT 네이티브 콜백 (Static)
    static void _mqttEventHandler(void* p_handlerArgs, esp_event_base_t p_base, int32_t p_eventId, void* p_eventData);

public:
    CL_T2_Communicator();
    ~CL_T2_Communicator();

    bool init();
    void runNetwork();

    // 바이너리 데이터 송출 (O(1) Copy-on-write 방어 적용)
    void broadcastBinary(const void* p_buffer, size_t p_bytes);

    // 융합 진단 결과 보고 (Vib + Audio + MFCC)
    bool publishResultMqtt(const T2_Type::UnifiedFeatureSlot& p_slot, T2_Type::DetectionResult p_result);

    bool isConnected() const { return WiFi.status() == WL_CONNECTED; }
};
