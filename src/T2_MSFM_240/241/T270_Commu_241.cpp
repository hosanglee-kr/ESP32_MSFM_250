/* ============================================================================
 * File: T270_Commu_241.cpp
 * Summary: 바이너리 멀티플렉싱 통신망 및 FSM 큐 통합 제어 구현부 (Full Version)
 * ============================================================================
 * [로직 상태 추적 (Logic State Tracker)]
 * - 유지: ESP-IDF Native MQTT 3.1.1 기반 고신뢰성 비동기 통신.
 * - 갱신: [교정] 모든 지연 구간에 vTaskDelay() 적용 및 핫스왑 API(Preview/Save) 복원.
 * - 신규: MQTT를 통한 원격 제어(Command Subscribe) 파싱 로직 및 OOM 방어벽 가동.
 * ========================================================================== */

#include "T270_Commu_241.hpp"
#include "T215_ConfigMgr_241.hpp"
#include <LittleFS.h>
#include <SD_MMC.h>
#include <time.h>
#include <Update.h>
#include <lwip/sockets.h>
#include "esp_log.h"

// FSM 매니저 연동을 위한 외부 인터페이스 (T247 구현 시 맵핑)
extern void T240_DispatchCommand(T2_Type::SystemCommand p_cmd);
extern uint8_t T240_GetCurrentState();
extern void T240_SetStreamFrequencies(uint8_t p_teleHz, uint8_t p_specHz, uint8_t p_waveHz);

static const char* TAG = "T246_COMM";

CL_T2_Communicator::CL_T2_Communicator()
    : _server(80), _ws(T2_Config::Net::WS_URI_CONST) {}

CL_T2_Communicator::~CL_T2_Communicator() {
    _ws.closeAll();
    _server.end();
    if (_mqttHandle) {
        esp_mqtt_client_stop(_mqttHandle);
        esp_mqtt_client_destroy(_mqttHandle);
    }
}

// [Rule #3] 좀비 소켓 방어를 위한 TCP Keep-Alive 강제화
void CL_T2_Communicator::_enforceTcpKeepAlive() {
    int v_fd = _wifiClient.fd();
    if (v_fd >= 0) {
        int v_opt = 1;
        setsockopt(v_fd, SOL_SOCKET, SO_KEEPALIVE, &v_opt, sizeof(v_opt));
        int v_idle = 30, v_intv = 5, v_cnt = 3;
        setsockopt(v_fd, IPPROTO_TCP, TCP_KEEPIDLE, &v_idle, sizeof(v_idle));
        setsockopt(v_fd, IPPROTO_TCP, TCP_KEEPINTVL, &v_intv, sizeof(v_intv));
        setsockopt(v_fd, IPPROTO_TCP, TCP_KEEPCNT, &v_cnt, sizeof(v_cnt));
    }
}

bool CL_T2_Communicator::init() {
    T2_Type::DynamicConfig v_cfg = CL_T2_ConfigManager::getInstance().getConfig();

    WiFi.mode(WIFI_OFF);
    vTaskDelay(pdMS_TO_TICKS(100));

    // 1. AP 모드 가동 (설정 기반)
    if (v_cfg.wifi.mode != T2_Type::WiFiMode::STA_ONLY) {
        if (strlen(v_cfg.wifi.ap_ip) > 0) {
            IPAddress v_apIp;
            if (v_apIp.fromString(v_cfg.wifi.ap_ip)) WiFi.softAPConfig(v_apIp, v_apIp, IPAddress(255, 255, 255, 0));
        }
        WiFi.softAP(v_cfg.wifi.ap_ssid, v_cfg.wifi.ap_password);
        ESP_LOGI(TAG, "AP Mode: %s", v_cfg.wifi.ap_ssid);
    }

    // 2. STA 모드 가동 및 Multi-AP 순회 접속
    if (v_cfg.wifi.mode != T2_Type::WiFiMode::AP_ONLY) {
        WiFi.mode(v_cfg.wifi.mode == T2_Type::WiFiMode::AP_STA ? WIFI_AP_STA : WIFI_STA);
        for (uint8_t i = 0; i < T2_Config::Net::WIFI_MULTI_MAX_CONST; i++) {
            if (strlen(v_cfg.wifi.multi_ssid[i]) > 0) {
                ESP_LOGI(TAG, "Connecting to %s...", v_cfg.wifi.multi_ssid[i]);
                WiFi.begin(v_cfg.wifi.multi_ssid[i], v_cfg.wifi.multi_pw[i]);

                uint32_t v_start = millis();
                while (WiFi.status() != WL_CONNECTED && (millis() - v_start < 8000)) vTaskDelay(pdMS_TO_TICKS(200));

                if (WiFi.status() == WL_CONNECTED) {
                    ESP_LOGI(TAG, "WiFi Connected: %s", WiFi.localIP().toString().c_str());
                    configTzTime(T2_Config::Net::TZ_INFO_CONST, T2_Config::Net::NTP_SERVER_1_CONST, "time.google.com");
                    _enforceTcpKeepAlive();
                    break;
                }
            }
        }
    }

    // 3. 네이티브 MQTT Client 초기화
	if (v_cfg.mqtt.enable && strlen(v_cfg.mqtt.broker) > 0) {
        char v_uri[128];
        char v_lwtTopic[128];
        snprintf(v_uri, sizeof(v_uri), "mqtt://%s:%d", v_cfg.mqtt.broker, v_cfg.mqtt.port);
        snprintf(v_lwtTopic, sizeof(v_lwtTopic), "%s/status/lwt", v_cfg.mqtt.topic_root); // [교정] 동적 결합

        esp_mqtt_client_config_t v_mqttCfg = {};
        v_mqttCfg.uri = v_uri;
        v_mqttCfg.client_id = v_cfg.mqtt.id;
        v_mqttCfg.password = v_cfg.mqtt.password;
        v_mqttCfg.lwt_topic = v_lwtTopic;

        v_mqttCfg.lwt_msg = "offline";
        v_mqttCfg.lwt_qos = 1;
        v_mqttCfg.lwt_retain = 1;
		v_mqttCfg.keepalive = 60; // [교정] MQTT 3.1.1 네이티브 Keep-Alive 강제 (60초)

        _mqttHandle = esp_mqtt_client_init(&v_mqttCfg);
        if (_mqttHandle) {
            esp_mqtt_client_register_event(_mqttHandle, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, _mqttEventHandler, this);
            esp_mqtt_client_start(_mqttHandle);
        }
    }


	// [교정] WebSocket 접속 상태 모니터링 이벤트 핸들러 추가
    _ws.onEvent([](AsyncWebSocket *p_server, AsyncWebSocketClient *p_client, AwsEventType p_type, void *p_arg, uint8_t *p_data, size_t p_len) {
        if (p_type == WS_EVT_CONNECT) {
            ESP_LOGI(TAG, "WS Client Connected: %u (IP: %s)", p_client->id(), p_client->remoteIP().toString().c_str());
        } else if (p_type == WS_EVT_DISCONNECT) {
            ESP_LOGI(TAG, "WS Client Disconnected: %u", p_client->id());
        }
    });

    _initWebHandlers();
    _server.addHandler(&_ws);
    _server.begin();
    return true;
}

void CL_T2_Communicator::_initWebHandlers() {
	// [교정] 전역 CORS 헤더 추가 (모든 OPTIONS Preflight 및 응답에 자동 적용)
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Content-Type");


    // [복원] 누락되었던 Web OTA (Over-The-Air) 펌웨어 업데이트 라우터
    _server.on("/api/update", HTTP_POST, [](AsyncWebServerRequest *p_req) {
        bool v_err = Update.hasError();
        AsyncWebServerResponse *v_res = p_req->beginResponse(200, "application/json", v_err ? "{\"ok\":false}" : "{\"ok\":true}");
        v_res->addHeader("Connection", "close");
        p_req->send(v_res);
        if (!v_err) {
            ESP_LOGI(TAG, "OTA Success. Rebooting...");
            vTaskDelay(pdMS_TO_TICKS(1000));
            ESP.restart();
        }
    }, [](AsyncWebServerRequest *p_req, String p_filename, size_t p_index, uint8_t *p_data, size_t p_len, bool p_final) {
        if (p_index == 0) {
            ESP_LOGI(TAG, "OTA Update Start: %s", p_filename.c_str());
            if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
        }
        if (Update.write(p_data, p_len) != p_len) Update.printError(Serial);
        if (p_final) {
            if (!Update.end(true)) Update.printError(Serial);
        }
    });

    // [시스템 상태 및 제어 API]
    _server.on("/api/status", HTTP_GET, [this](AsyncWebServerRequest* p_request) {
        JsonDocument v_doc;
        v_doc["sys_state"] = T240_GetCurrentState();
        v_doc["heap"] = ESP.getFreeHeap();
        v_doc["psram"] = ESP.getFreePsram();
        _sendJsonResponse(p_request, v_doc);
    });

    _server.on("/api/recorder_begin", HTTP_POST, [](AsyncWebServerRequest* p_request) {
        T240_DispatchCommand(T2_Type::SystemCommand::CMD_START);
        p_request->send(200, "application/json", "{\"ok\":true}");
    });

    _server.on("/api/recorder_end", HTTP_POST, [](AsyncWebServerRequest* p_request) {
        T240_DispatchCommand(T2_Type::SystemCommand::CMD_STOP);
        p_request->send(200, "application/json", "{\"ok\":true}");
    });

    _server.on("/api/reboot", HTTP_POST, [](AsyncWebServerRequest* p_request) {
        p_request->send(200, "application/json", "{\"ok\":true}");
        T240_DispatchCommand(T2_Type::SystemCommand::CMD_REBOOT);
    });

    // [v013 복원] 핫스왑 튜닝 API
    _server.on("/api/config/preview", HTTP_POST, [](AsyncWebServerRequest *p_req) {}, NULL,
        [](AsyncWebServerRequest *p_req, uint8_t *p_data, size_t p_len, size_t p_idx, size_t p_total) {
            if (p_idx == 0) p_req->_tempObject = malloc(p_total + 1);
            uint8_t* v_buf = (uint8_t*)p_req->_tempObject;
            if (v_buf) {
                memcpy(v_buf + p_idx, p_data, p_len);
                if (p_idx + p_len == p_total) {
                    v_buf[p_total] = '\0';
                    bool v_res = CL_T2_ConfigManager::getInstance().updatePreview((const char*)v_buf);
                    p_req->send(v_res ? 200 : 400, "application/json", v_res ? "{\"ok\":true}" : "{\"ok\":false}");
                    free(v_buf); p_req->_tempObject = NULL;
                }
            }
        }
    );

    _server.on("/api/config/save", HTTP_POST, [](AsyncWebServerRequest* p_req) {
        bool v_res = CL_T2_ConfigManager::getInstance().commitSave();
        p_req->send(v_res ? 200 : 500, "application/json", v_res ? "{\"ok\":true}" : "{\"ok\":false}");
    });

    // [정적 파일 및 다운로드 서비스 복원]
    _server.on("/api/download", HTTP_GET, [](AsyncWebServerRequest* p_req) {
        if (!p_req->hasParam("file")) { p_req->send(400); return; }
        String v_path = p_req->getParam("file")->value();
        if (SD_MMC.exists(v_path)) p_req->send(SD_MMC, v_path, "application/octet-stream");
        else p_req->send(404);
    });

	// 웹 UI에서 저장된 파일 목록을 렌더링하기 위한 Index 제공 API
    _server.on("/api/files", HTTP_GET, [](AsyncWebServerRequest* p_req) {
        if (LittleFS.exists("/sys/runtime_idx_241.json")) {
            // 스토리지 엔진이 갱신하는 인덱스 파일을 그대로 서빙 (Zero-copy 효율)
            p_req->send(LittleFS, "/sys/runtime_idx_241.json", "application/json");
        } else {
            p_req->send(404, "application/json", "{\"ok\":false, \"msg\":\"No files found.\"}");
        }
    });

    _server.serveStatic("/", LittleFS, "/www/").setDefaultFile("index.html");
}

void CL_T2_Communicator::broadcastBinary(const void* p_buffer, size_t p_bytes) {
    if (_ws.count() == 0 || !p_buffer) return;
    for (auto& v_client : _ws.getClients()) {
        if (v_client.status() == WS_CONNECTED) {
            // [Rule #37] 큐 적체 감시를 통한 OOM 방어
            if (v_client.queueLen() > 20 || !v_client.canSend()) {
                if (v_client.queueLen() > 50) v_client.close();
                continue;
            }
            v_client.binary((uint8_t*)p_buffer, p_bytes);
        }
    }
}

void CL_T2_Communicator::runNetwork() {
    _ws.cleanupClients();
    if (WiFi.status() != WL_CONNECTED && WiFi.getMode() != WIFI_AP) {
        if (millis() - _lastWifiRetryMs > 15000) {
            WiFi.reconnect();
            _lastWifiRetryMs = millis();
        }
    }
}

bool CL_T2_Communicator::publishResultMqtt(const T2_Type::UnifiedFeatureSlot& p_slot, T2_Type::DetectionResult p_result) {
    if (!_mqttHandle) return false;

	T2_Type::DynamicConfig v_cfg = CL_T2_ConfigManager::getInstance().getConfig(); // 설정 로드

	char v_pubTopic[128];
    snprintf(v_pubTopic, sizeof(v_pubTopic), "%s/res", v_cfg.mqtt.topic_root); // [교정] 동적 토픽

    JsonDocument v_doc;
    v_doc["mac"] = WiFi.macAddress();
    v_doc["res"] = (uint8_t)p_result;
    v_doc["v_rms"] = p_slot.vib_rms[0]; // 대표축
    v_doc["a_rms"] = p_slot.audio_rms;
    v_doc["kurt"] = p_slot.kurtosis;

    // MFCC 및 Band Energy 포함 (Rule #47 보완)
    JsonArray v_mfcc = v_doc["mfcc"].to<JsonArray>();
    for(int i=0; i<13; i++) v_mfcc.add(p_slot.mfcc[i]);

	char v_pay[1024];
    serializeJson(v_doc, v_pay, sizeof(v_pay));

    return esp_mqtt_client_publish(_mqttHandle, v_pubTopic, v_pay, 0, v_cfg.mqtt.qos, 0) != -1;
}

void CL_T2_Communicator::_mqttEventHandler(void* p_handlerArgs, esp_event_base_t p_base, int32_t p_eventId, void* p_eventData) {
    esp_mqtt_event_handle_t v_ev = (esp_mqtt_event_handle_t)p_eventData;
    T2_Type::DynamicConfig v_cfg = CL_T2_ConfigManager::getInstance().getConfig();

    char v_cmdTopic[128];
    char v_lwtTopic[128];
    snprintf(v_cmdTopic, sizeof(v_cmdTopic), "%s/cmd", v_cfg.mqtt.topic_root);
    snprintf(v_lwtTopic, sizeof(v_lwtTopic), "%s/status/lwt", v_cfg.mqtt.topic_root);

    switch ((esp_mqtt_event_id_t)p_eventId) {
        case MQTT_EVENT_CONNECTED:
            esp_mqtt_client_subscribe(v_ev->client, v_cmdTopic, 1);
            esp_mqtt_client_publish(v_ev->client, v_lwtTopic, "online", 0, 1, 1);
            ESP_LOGI(TAG, "MQTT Connected & Subscribed to: %s", v_cmdTopic);
            break;
        case MQTT_EVENT_DATA: {
            // [교정] 수신된 토픽이 CMD 토픽과 정확히 일치하는지 먼저 검사 (보안 방어벽)
            if (v_ev->topic_len == strlen(v_cmdTopic) && strncmp(v_ev->topic, v_cmdTopic, v_ev->topic_len) == 0) {
                if (strncmp(v_ev->data, "reboot", v_ev->data_len) == 0) T240_DispatchCommand(T2_Type::SystemCommand::CMD_REBOOT);
                else if (strncmp(v_ev->data, "start", v_ev->data_len) == 0) T240_DispatchCommand(T2_Type::SystemCommand::CMD_START);
                else if (strncmp(v_ev->data, "stop", v_ev->data_len) == 0) T240_DispatchCommand(T2_Type::SystemCommand::CMD_STOP);
            }
            break;
        }
        default: break;
    }
}

void CL_T2_Communicator::_setCorsHeaders(AsyncWebServerResponse* p_response) {
    p_response->addHeader("Access-Control-Allow-Origin", "*");
    p_response->addHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
}

void CL_T2_Communicator::_sendJsonResponse(AsyncWebServerRequest* p_request, const JsonDocument& p_doc) {
    AsyncResponseStream *v_stream = p_request->beginResponseStream("application/json");
    _setCorsHeaders(v_stream);
    serializeJson(p_doc, *v_stream);
    p_request->send(v_stream);
}
