/* ============================================================================
 * File: T270_Commu_245.hpp
 * Summary: 멀티플렉싱 통신 엔진 (WiFi, MQTT, WebSocket, WebServer)
 * ============================================================================ */

#pragma once

#include "T210_Def_245.hpp"
#include "T215_Type_245.hpp"
#include <WiFi.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <mqtt_client.h>

/**
 * @class CL_T2_Communicator
 * @brief Wi-Fi 인프라 관리, 비동기 웹 서버 및 WebSocket 실시간 브로드캐스팅, 그리고 MQTT 통신 접속을 총괄하는 통신 관리 엔진
 */
class CL_T2_Communicator {
private:
    AsyncWebServer           _server;             ///< HTTP REST API 요청 수신 비동기 웹서버 객체
    AsyncWebSocket           _ws;                 ///< 실시간 특징/파형 스트리밍용 비동기 WebSocket 서버 객체
    WiFiClient               _wifiClient;         ///< 기초 TCP 소켓 클라이언트

    esp_mqtt_client_handle_t _mqttHandle;         ///< ESP-IDF 네이티브 MQTT 클라이언트 핸들

    uint32_t                 _lastMqttRetryMs;    ///< 마지막 MQTT 재접속 재시도 시간 밀리초
    uint32_t                 _lastWifiRetryMs;    ///< 마지막 Wi-Fi 재연결 재시도 시간 밀리초
    bool                     _isOtaRunning;       ///< 무선 OTA(Firmware Upgrade) 활성 여부 플래그

    // 내부 유틸리티
    void _enforceTcpKeepAlive(int p_fd);
    void _initWebHandlers();
    void _setCorsHeaders(AsyncWebServerResponse* p_response);

    // MQTT 콜백 함수 (Static)
    static void _mqttEventHandler(void* p_handlerArgs, esp_event_base_t p_base, int32_t p_eventId, void* p_eventData);

public:
    CL_T2_Communicator();
    ~CL_T2_Communicator();

    /**
     * @brief 무선 네트워크(STA/AP), 비동기 웹 서버 가동 및 MQTT 클라이언트 접속 수행
     * @return 구동 성공 여부
     */
    bool init();

    /**
     * @brief Wi-Fi 링크 단선 및 MQTT 세션 유실 체크 후 자동 재연결 기동 (runMaintenance에서 주기적 구동)
     */
    void runNetwork();

    /**
     * @brief 연결되어있는 모든 웹소켓 클라이언트들에 바이너리 스트림 데이터를 전송
     * @param p_buffer 전송 데이터 버퍼 주소
     * @param p_bytes 전송 바이트 크기
     */
    void broadcastBinary(const void* p_buffer, size_t p_bytes);

    /**
     * @brief 기 지정된 이상 룰 감지 판정 결과를 JSON으로 패키징하여 MQTT 토픽으로 게시
     * @param p_slot 진단 결과 메타가 포함된 특징 슬롯 참조
     * @param p_result 최종 판정 등급 코드
     * @return 게시 성공 여부
     */
    bool publishResultMqtt(const T2_Type::ST_UnifiedFeatureSlot_t& p_slot, T2_Type::EM_DetectionResult_t p_result);

    /**
     * @brief 현재 AP 무선 라우터 연결 여부 조회
     */
    bool isConnected() const { return WiFi.status() == WL_CONNECTED; }
};
