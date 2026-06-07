/* ============================================================================
 * File: T270_Commu_244.hpp
 * Summary: 멀티플렉싱 통신 엔진 (WiFi, MQTT, WebSocket, WebServer) - v243
 * ============================================================================
 * [로직 상태 추적 (Logic State Tracker)]
 * - 유지: ESP-IDF Native MQTT 및 AsyncWebServer 기반 고속 통신.
 * - 갱신: v243 4-Tier 설정 체계 및 alignas(16) 바이너리 패킷 규격 준수.
 * - 신규: [원칙 준수] TCP Keep-Alive 옵션을 통한 좀비 소켓 차단 및 OOM 방어.
 * ========================================================================== */
#pragma once

#include "T210_Def_244.hpp"
#include "T215_Type_244.hpp"
#include <WiFi.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <mqtt_client.h>

class CL_T2_Communicator {
private:
    AsyncWebServer _server;
    AsyncWebSocket _ws;
    WiFiClient     _wifiClient;

    esp_mqtt_client_handle_t _mqttHandle;

    uint32_t _lastMqttRetryMs;
    uint32_t _lastWifiRetryMs;
    bool     _isOtaRunning;

    // 내부 유틸리티
    void _enforceTcpKeepAlive(int p_fd);
    void _initWebHandlers();
    void _setCorsHeaders(AsyncWebServerResponse* p_response);

    // MQTT 콜백 (Static)
    static void _mqttEventHandler(void* p_handlerArgs, esp_event_base_t p_base, int32_t p_eventId, void* p_eventData);

public:
    CL_T2_Communicator();
    ~CL_T2_Communicator();

    /**
     * @brief 통신 인프라 초기화 (WiFi 연결, 웹서버 시작, MQTT 핸들 생성)
     */
    bool init();

    /**
     * @brief 네트워크 상태 유지 및 재연결 관리 (Main Loop에서 호출)
     */
    void runNetwork();

    /**
     * @brief WebSocket을 통한 실시간 바이너리 데이터 브로드캐스트
     */
    void broadcastBinary(const void* p_buffer, size_t p_bytes);

    /**
     * @brief MQTT를 통한 진단 결과 보고
     */
    bool publishResultMqtt(const T2_Type::UnifiedFeatureSlot& p_slot, T2_Type::DetectionResult p_result);

    bool isConnected() const { return WiFi.status() == WL_CONNECTED; }
};
