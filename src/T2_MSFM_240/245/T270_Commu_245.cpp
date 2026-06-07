/* ============================================================================
 * File: T270_Commu_245.cpp
 * Summary: 멀티플렉싱 통신 엔진 구현부
 * ========================================================================== */
#include "T270_Commu_245.hpp"
#include "T220_CfgMgr_245.hpp"
#include <LittleFS.h>
#include <SD_MMC.h>
#include <time.h>
#include <Update.h>
#include <lwip/sockets.h>
#include "esp_log.h"

// FSM 매니저 연동 인터페이스
extern void T240_DispatchCommand(T2_Type::EM_SystemCommand_t p_cmd);
extern uint8_t T240_GetCurrentState();

static const char* TAG = "T245_COMM";

/**
 * @brief CL_T2_Communicator 생성자
 * @details 멤버 변수 초기화 및 기본값 할당
 */
CL_T2_Communicator::CL_T2_Communicator()
    : _server(80), _ws("/ws"), _mqttHandle(nullptr),
      _lastMqttRetryMs(0), _lastWifiRetryMs(0), _isOtaRunning(false) {}

/**
 * @brief CL_T2_Communicator 소멸자
 * @details 리소스를 정리하고 웹소켓, 웹서버, MQTT 클라이언트를 종료
 */
CL_T2_Communicator::~CL_T2_Communicator() {
    _ws.closeAll();
    _server.end();
    if (_mqttHandle) {
        esp_mqtt_client_stop(_mqttHandle);
        esp_mqtt_client_destroy(_mqttHandle);
    }
}

/**
 * @brief 통신 모듈 초기화
 * @details AP 모드 설정, STA 모드 설정, MQTT 및 웹서버 초기화 수행
 * @return initialization success or failure
 */
bool CL_T2_Communicator::init() {
    const auto& v_cfg = CL_T2_ConfigManager::getInstance().getConfig();

    // [처리 단위 1] AP 모드 가동 (설정 기반)
    if (v_cfg.wifi.mode != T2_Type::EM_WiFiMode_t::STA_ONLY) {
        if (strlen(v_cfg.wifi.ap_ip) > 0) {
            IPAddress v_apIp;
            if (v_apIp.fromString(v_cfg.wifi.ap_ip)) {
                WiFi.softAPConfig(v_apIp, v_apIp, IPAddress(255, 255, 255, 0));
            }
        }
        WiFi.softAP(v_cfg.wifi.ap_ssid, v_cfg.wifi.ap_pw);
        ESP_LOGI(TAG, "AP Mode Active: %s", v_cfg.wifi.ap_ssid);
    }

    // [처리 단위 2] STA 모드 가동 및 Multi-AP 순회 접속
    if (v_cfg.wifi.mode != T2_Type::EM_WiFiMode_t::AP_ONLY) {
        WiFi.mode(v_cfg.wifi.mode == T2_Type::EM_WiFiMode_t::AP_STA ? WIFI_AP_STA : WIFI_STA);

        bool v_connected = false;
        for (uint8_t i = 0; i < T2_Def::Global::NetLimit::NET_MULTI_AP_MAX; i++) {
            if (strlen(v_cfg.wifi.multi_ssid[i]) > 0) {
                ESP_LOGI(TAG, "Connecting to %s...", v_cfg.wifi.multi_ssid[i]);
                WiFi.begin(v_cfg.wifi.multi_ssid[i], v_cfg.wifi.multi_pw[i]);

                uint32_t v_start = millis();
                while (WiFi.status() != WL_CONNECTED && (millis() - v_start < 8000)) {
                    vTaskDelay(pdMS_TO_TICKS(200));
                }

                if (WiFi.status() == WL_CONNECTED) {
                    ESP_LOGI(TAG, "WiFi Connected: %s (RSSI: %d)", WiFi.localIP().toString().c_str(), WiFi.RSSI());
                    // NTP 서버 동기화
                    configTzTime(T2_Def::Global::Net::NTP_TZ_INFO_CONST, T2_Def::Global::Net::NTP_SERVER_1_CONST, T2_Def::Global::Net::NTP_SERVER_2_CONST);
                    _enforceTcpKeepAlive(_wifiClient.fd()); // 좀비 소켓 방어
                    v_connected = true;
                    break;
                }
            }
        }
        if (!v_connected) ESP_LOGW(TAG, "Failed to connect to any registered AP.");
    }

    // [처리 단위 3] 네이티브 MQTT Client 초기화
    if (v_cfg.mqtt.enable && strlen(v_cfg.mqtt.broker) > 0) {
        char v_uri[128];
        char v_lwtTopic[128];
        snprintf(v_uri, sizeof(v_uri), "mqtt://%s:%d", v_cfg.mqtt.broker, v_cfg.mqtt.port);
        snprintf(v_lwtTopic, sizeof(v_lwtTopic), "%s/lwt", v_cfg.mqtt.topic_root);

        esp_mqtt_client_config_t v_mqttCfg = {};
        v_mqttCfg.uri = v_uri;
        v_mqttCfg.client_id = v_cfg.mqtt.id;
        v_mqttCfg.username = v_cfg.mqtt.id;
        v_mqttCfg.password = v_cfg.mqtt.pw;
        v_mqttCfg.lwt_topic = v_lwtTopic;
        v_mqttCfg.lwt_msg = "offline";
        v_mqttCfg.lwt_qos = v_cfg.mqtt.qos;
        v_mqttCfg.lwt_retain = 1;

        _mqttHandle = esp_mqtt_client_init(&v_mqttCfg);
        esp_mqtt_client_register_event(_mqttHandle, (esp_mqtt_event_id_t)MQTT_EVENT_ANY, _mqttEventHandler, this);
        esp_mqtt_client_start(_mqttHandle);
    }

    // [처리 단위 4] 웹서버 핸들러 초기화 및 가동
    _initWebHandlers(); // 웹서버 핸들러 초기화 누락 방지
    _server.begin();

    ESP_LOGI(TAG, "Communication Engine Ready.");
    return true;
}

/**
 * @brief TCP 소켓 Keep-Alive 강제 설정
 * @details 비정상 종료된 데드 소켓 누적으로 인한 포트 점유 현상을 극복하기 위한 커널 옵션 설정
 * @param p_fd 대상 소켓 파일 디스크립터
 */
void CL_T2_Communicator::_enforceTcpKeepAlive(int p_fd) {
    if (p_fd < 0) return;

    int v_opt = 1;
    setsockopt(p_fd, SOL_SOCKET, SO_KEEPALIVE, &v_opt, sizeof(v_opt));

    // 유휴 30초 후, 5초 간격으로 3회 재시도 (총 45초 내에 데드 소켓 확정)
    int v_idle = 30, v_intv = 5, v_cnt = 3;
    setsockopt(p_fd, IPPROTO_TCP, TCP_KEEPIDLE, &v_idle, sizeof(v_idle));
    setsockopt(p_fd, IPPROTO_TCP, TCP_KEEPINTVL, &v_intv, sizeof(v_intv));
    setsockopt(p_fd, IPPROTO_TCP, TCP_KEEPCNT, &v_cnt, sizeof(v_cnt));
}

/**
 * @brief 웹 서버 API 핸들러 등록
 * @details WebSocket 엔드포인트 및 REST API(/api/status, /api/command, /api/config, /api/files 등) 라우팅 설정
 */
void CL_T2_Communicator::_initWebHandlers() {
    // [처리 단위 1] WebSocket 연결 설정
    _server.addHandler(&_ws);

    // [처리 단위 2] REST API - 상태 조회 (/api/status)
    _server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* p_req) {
        JsonDocument v_doc;
        v_doc["state"] = T240_GetCurrentState();
        v_doc["heap"] = ESP.getFreeHeap();
        v_doc["psram"] = ESP.getFreePsram();
        v_doc["wifi_rssi"] = WiFi.RSSI();

        String v_out;
        serializeJson(v_doc, v_out);
        p_req->send(200, "application/json", v_out);
    });

    // [처리 단위 3] REST API - 명령 제어 (/api/command)
    _server.on("/api/command", HTTP_POST, [](AsyncWebServerRequest* p_req) {
        if (!p_req->hasParam("cmd", true)) { p_req->send(400); return; }
        uint8_t v_cmd = p_req->getParam("cmd", true)->value().toInt();
        T240_DispatchCommand((T2_Type::EM_SystemCommand_t)v_cmd);
        p_req->send(200, "application/json", "{\"ok\":true}");
    });

    // [처리 단위 4] REST API - 설정 핫스왑 (Preview)
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

    // [처리 단위 5] REST API - 설정 영구 저장
    _server.on("/api/config/save", HTTP_POST, [](AsyncWebServerRequest* p_req) {
        bool v_res = CL_T2_ConfigManager::getInstance().commitSave();
        p_req->send(v_res ? 200 : 500, "application/json", v_res ? "{\"ok\":true}" : "{\"ok\":false}");
    });

    // [처리 단위 6] REST API - 파일 브라우징 및 다운로드
    _server.on("/api/files", HTTP_GET, [](AsyncWebServerRequest* p_req) {
        if (LittleFS.exists("/sys/runtime_idx_245.json")) {
            p_req->send(LittleFS, "/sys/runtime_idx_245.json", "application/json");
        } else {
            p_req->send(404, "application/json", "{\"ok\":false}");
        }
    });

    _server.on("/api/download", HTTP_GET, [](AsyncWebServerRequest* p_req) {
        if (!p_req->hasParam("file")) { p_req->send(400); return; }
        String v_path = p_req->getParam("file")->value();
        if (SD_MMC.exists(v_path)) p_req->send(SD_MMC, v_path, "application/octet-stream");
        else p_req->send(404);
    });

    // [처리 단위 7] REST API - Web OTA 핸들러
    _server.on("/api/update", HTTP_POST, [](AsyncWebServerRequest *p_req) {
        AsyncWebServerResponse *v_res = p_req->beginResponse(200, "application/json", Update.hasError() ? "{\"ok\":false}" : "{\"ok\":true}");
        v_res->addHeader("Connection", "close");
        p_req->send(v_res);

        // OTA 종료 후 시스템 정상화 또는 재부팅
        T240_DispatchCommand(T2_Type::EM_SystemCommand_t::CMD_OTA_END);
        if (!Update.hasError()) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_restart();
        }
    }, [](AsyncWebServerRequest *p_req, String p_filename, size_t p_index, uint8_t *p_data, size_t p_len, bool p_final) {
        if (p_index == 0) {
            ESP_LOGI(TAG, "OTA Start: %s", p_filename.c_str());
            T240_DispatchCommand(T2_Type::EM_SystemCommand_t::CMD_OTA_START);
            if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
        }
        if (Update.write(p_data, p_len) != p_len) Update.printError(Serial);
        if (p_final) {
            if (!Update.end(true)) Update.printError(Serial);
        }
    });

    // [처리 단위 8] 정적 파일 서비스
    _server.serveStatic("/", LittleFS, "/www/").setDefaultFile("index.html");
}

/**
 * @brief 바이너리 데이터 브로드캐스트
 * @details 접속된 모든 웹소켓 클라이언트 대상 바이너리 실시간 송출
 * @param p_buffer 송신 데이터 버퍼 포인터
 * @param p_bytes 전송 바이트 수
 */
void CL_T2_Communicator::broadcastBinary(const void* p_buffer, size_t p_bytes) {
    if (_ws.count() == 0 || !p_buffer) return;

    // OOM 방어형 브로드캐스트
    for (auto& v_client : _ws.getClients()) {
        if (v_client.status() == WS_CONNECTED) {
            if (v_client.queueLen() > 10) { // 적체 시 스킵
                if (v_client.queueLen() > 30) v_client.close();
                continue;
            }
            v_client.binary((uint8_t*)p_buffer, p_bytes);
        }
    }
}

/**
 * @brief 검출 결과를 MQTT로 퍼블리시
 * @details RMS 값 및 Coherence, IPD를 구조화하여 JSON 페이로드 생성 후 전송
 * @param p_slot 통합 특징 슬롯 데이터 레퍼런스
 * @param p_result 판정 결과 플래그
 * @return 성공 여부
 */
bool CL_T2_Communicator::publishResultMqtt(const T2_Type::ST_UnifiedFeatureSlot_t& p_slot, T2_Type::EM_DetectionResult_t p_result) {
    if (!_mqttHandle || p_result == T2_Type::EM_DetectionResult_t::PASS) return false;

    const auto& v_cfg = CL_T2_ConfigManager::getInstance().getConfig();
    char v_topic[128];
    snprintf(v_topic, sizeof(v_topic), "%s/result", v_cfg.mqtt.topic_root);

    // [처리 단위 1] JSON 문서 초기화 및 고유 식별 필드 매핑
    JsonDocument v_doc;
    v_doc["site"] = v_cfg.system.site_id;
    v_doc["res"] = (uint8_t)p_result;

    // [처리 단위 2] 3축 가속도 RMS 데이터 직렬화
    JsonArray v_rmsAcc = v_doc["rms_acc"].to<JsonArray>();
    v_rmsAcc.add(p_slot.accel.rms[0]);
    v_rmsAcc.add(p_slot.accel.rms[1]);
    v_rmsAcc.add(p_slot.accel.rms[2]);

    // [처리 단위 3] 자이로 RMS 데이터 직렬화
    JsonArray v_rmsGyr = v_doc["rms_gyr"].to<JsonArray>();
    v_rmsGyr.add(p_slot.gyro.rms[0]);
    v_rmsGyr.add(p_slot.gyro.rms[1]);
    v_rmsGyr.add(p_slot.gyro.rms[2]);

    // [처리 단위 4] 듀얼 마이크 오디오 RMS 및 위상 간 결합성 데이터 직렬화
    JsonArray v_rmsA = v_doc["rms_a"].to<JsonArray>();
    v_rmsA.add(p_slot.audio.ch[0].rms);
    v_rmsA.add(p_slot.audio.ch[1].rms);

    v_doc["coh"] = p_slot.audio.coh;
    v_doc["ipd"] = p_slot.audio.ipd;
    v_doc["trial"] = p_slot.header.trial;

    // [처리 단위 5] 최종 JSON 페이로드 발행 수행
    char v_payload[384]; 
    serializeJson(v_doc, v_payload);

    esp_mqtt_client_publish(_mqttHandle, v_topic, v_payload, 0, v_cfg.mqtt.qos, 0);
    return true;
}

/**
 * @brief 네트워크 백그라운드 관리
 * @details 웹소켓 클라이언트 정리 및 WiFi 단절 시 10초 주기 재연결 수행
 */
void CL_T2_Communicator::runNetwork() {
    _ws.cleanupClients();

    // WiFi 자동 복구
    if (WiFi.status() != WL_CONNECTED && WiFi.getMode() != WIFI_AP) {
        if (millis() - _lastWifiRetryMs > 10000) {
            WiFi.reconnect();
            _lastWifiRetryMs = millis();
        }
    }
}

/**
 * @brief MQTT 클라이언트 이벤트 핸들러
 * @details MQTT 연결 수립 및 토픽 구독 처리, 원격 명령 데이터 수신 핸들링
 * @param p_handlerArgs 핸들러 등록 컨텍스트 포인터
 * @param p_base 이벤트 베이스
 * @param p_eventId 이벤트 ID
 * @param p_eventData 이벤트 데이터 구조체 포인터
 */
void CL_T2_Communicator::_mqttEventHandler(void* p_handlerArgs, esp_event_base_t p_base, int32_t p_eventId, void* p_eventData) {
    CL_T2_Communicator* v_this = (CL_T2_Communicator*)p_handlerArgs;
    esp_mqtt_event_handle_t v_event = (esp_mqtt_event_handle_t)p_eventData;

    switch (p_eventId) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Connected");
            esp_mqtt_client_subscribe(v_event->client, "smea/t240/cmd", 1);
            break;
        case MQTT_EVENT_DATA:
            if (strncmp(v_event->topic, "smea/t240/cmd", v_event->topic_len) == 0) {
                // 원격 명령 파싱 (단순 숫자로 가정)
                uint8_t v_cmd = atoi(v_event->data);
                T240_DispatchCommand((T2_Type::EM_SystemCommand_t)v_cmd);
            }
            break;
        default: break;
    }
}

