/* ============================================================================
 * File: T270_Commu_250.hpp
 * Summary: 멀티플렉싱 통신 엔진 (WiFi, MQTT, WebSocket, WebServer)
 * ============================================================================
 * [로직 상태 추적 (Logic State Tracker)]
 * - 유지: ESP-IDF Native MQTT 및 AsyncWebServer 기반 고속 통신.
 * - 갱신: 4-Tier 설정 체계 및 alignas(16) 바이너리 패킷 규격 준수.
 * - 신규: [원칙 준수] TCP Keep-Alive 옵션을 통한 좀비 소켓 차단 및 OOM 방어.
 * ========================================================================== */
#pragma once

#include "T210_Def_250.hpp"
#include "T215_Type_250.hpp"
#include <WiFi.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <mqtt_client.h>

class CL_T2_Communicator {
private:
    AsyncWebServer _server;                 // AsyncWebServer 객체
    AsyncWebSocket _ws;                     // AsyncWebSocket 객체
    WiFiClient     _wifiClient;             // WiFi 클라이언트

    esp_mqtt_client_handle_t _mqttHandle;   // MQTT 핸들

    uint32_t _lastMqttRetryMs;              // 마지막 MQTT 재시도 시간
    uint32_t _lastWifiRetryMs;              // 마지막 WiFi 재시도 시간
    bool     _isOtaRunning;                 // OTA 실행 중 여부

    // 내부 유틸리티
    // TCP Keep-Alive 강제 설정 (좀비 소켓 방지)
    void _enforceTcpKeepAlive(int p_fd);

    // 웹 핸들러 초기화
    void _initWebHandlers();

    // CORS 헤더 설정
    void _setCorsHeaders(AsyncWebServerResponse* p_response);

    // MQTT 콜백 (Static)
    static void _mqttEventHandler(void* p_handlerArgs, esp_event_base_t p_base, int32_t p_eventId, void* p_eventData);

public:
    CL_T2_Communicator();
    ~CL_T2_Communicator();

    // 통신 인프라 초기화 (WiFi 연결, 웹서버 시작, MQTT 핸들 생성)
    bool init();

    // 네트워크 상태 유지 및 재연결 관리 (Main Loop에서 호출)
    void runNetwork();

    // WebSocket을 통한 실시간 바이너리 데이터 브로드캐스트
    void broadcastBinary(const void* p_buffer, size_t p_bytes);

    // MQTT를 통한 진단 결과 보고
    bool publishResultMqtt(const T2_Type::ST_FeatureSlot_Aud_t& p_audSlot,
                           const T2_Type::ST_FeatureSlot_Vib_t& p_vibSlot,
                           T2_Type::EM_DetectionResult_t p_result);

    // WiFi 연결 상태 확인 게터
    bool isConnected() const { return WiFi.status() == WL_CONNECTED; }

    // MQTT 클라이언트 동적 재생성
    void recreateMqttClient(const esp_mqtt_client_config_t& p_newCfg);

    // WebSocket 연결 확인 게터
    bool hasActiveWebsockets() const { return _ws.count() > 0; }
};
