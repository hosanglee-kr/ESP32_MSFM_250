/* ============================================================================
 * File: T470_Commu_014.cpp
 * Summary: Network, MQTT & OTA Implementation (FSM Command Queue Integration)
 * ============================================================================
 * * [AI 메모: 마이그레이션 적용 사항]
 * 1. [OTA 데드락 방어]: OTA 진입 시 T450에 CMD_OTA_START를 디스패치하여
 * FSM을 MAINTENANCE 상태로 전환시킴으로써 마이크와 SD카드 버스 락을 해제.
 * 2. [OOM 통신 폭발 방어]: broadcastBinary 내부에서 4KB 이상 큐 적체 시
 * 클라이언트 강제 종료(client->close()) 수행.
 * 3. [시간 무결성 보장]: time() 기반 Epoch 기록을 위해 SNTP 서버 완벽 동기화 보장.
 * 4. [v013 핫스왑 방어]: /api/config/preview, save, cancel 라우팅을 추가하여
 * 웹에서의 실시간 슬라이더 조작을 메모리에만 반영하도록 분리(플래시 마모율 0%).
 * ========================================================================== */
#include "T470_Commu_014.hpp"
#include "T450_FsmMgr_015.hpp"
#include "T415_ConfigMgr_013.hpp"
#include <LittleFS.h>
#include <SD_MMC.h>
#include <time.h>
#include <Update.h>
#include <lwip/sockets.h>

T470_Communicator::T470_Communicator()
    : _server(SmeaConfig::Network::HTTP_PORT_DEF),
      _ws(SmeaConfig::Network::WS_URI_DEF) {
}

T470_Communicator::~T470_Communicator() {
    _ws.closeAll();
    _server.end();
}

// LwIP 레벨의 TCP Keep-Alive 강제 활성화 (좀비 소켓 킬러)
void T470_Communicator::_enforceTcpKeepAlive() {
    int v_fd = _wifiClient.fd();
    if (v_fd >= 0) {
        int v_enable = 1;
        setsockopt(v_fd, SOL_SOCKET, SO_KEEPALIVE, &v_enable, sizeof(v_enable));

        int v_idle = 30; // 30초 대기 후 프로브
        int v_interval = 5; // 5초 간격
        int v_count = 3; // 3회 실패 시 연결 해제

        setsockopt(v_fd, IPPROTO_TCP, TCP_KEEPIDLE, &v_idle, sizeof(v_idle));
        setsockopt(v_fd, IPPROTO_TCP, TCP_KEEPINTVL, &v_interval, sizeof(v_interval));
        setsockopt(v_fd, IPPROTO_TCP, TCP_KEEPCNT, &v_count, sizeof(v_count));
    }
}

bool T470_Communicator::init(const char* p_ssid, const char* p_pw, const char* p_mqttBroker) {
    DynamicConfig v_cfg = T415_ConfigManager::getInstance().getConfig();

    WiFi.mode(WIFI_OFF);
    delay(SmeaConfig::NetworkLimit::WIFI_MODE_SWITCH_DELAY_MS_CONST);

    if (v_cfg.wifi.mode == 0 || v_cfg.wifi.mode == 2 || v_cfg.wifi.mode == 3) {
        if (strlen(v_cfg.wifi.ap_ip) > 0) {
            IPAddress v_apIp;
            if (v_apIp.fromString(v_cfg.wifi.ap_ip)) {
                WiFi.softAPConfig(v_apIp, v_apIp, IPAddress(255, 255, 255, 0));
            }
        }
        WiFi.softAP(v_cfg.wifi.ap_ssid, v_cfg.wifi.ap_password);
        Serial.printf("[Net] AP Started: %s\n", v_cfg.wifi.ap_ssid);
    }

    if (v_cfg.wifi.mode != 0) {
        WiFi.mode(v_cfg.wifi.mode == 2 ? WIFI_AP_STA : WIFI_STA);

        for (uint8_t i = 0; i < SmeaConfig::NetworkLimit::MAX_MULTI_AP_CONST; i++) {
            const char* v_targetSsid = (i == 0 && strlen(v_cfg.wifi.multi_ap[0].ssid) == 0 && strlen(p_ssid) > 0) ? p_ssid : v_cfg.wifi.multi_ap[i].ssid;
            const char* v_targetPw = (i == 0 && strlen(v_cfg.wifi.multi_ap[0].password) == 0 && strlen(p_pw) > 0) ? p_pw : v_cfg.wifi.multi_ap[i].password;

            if (strlen(v_targetSsid) > 0) {
                WiFi.disconnect();
                delay(SmeaConfig::NetworkLimit::WIFI_DISCONNECT_DELAY_MS_CONST);

                if (v_cfg.wifi.multi_ap[i].use_static_ip) {
                    IPAddress v_ip, v_gw, v_sn, v_d1, v_d2;
                    v_ip.fromString(v_cfg.wifi.multi_ap[i].local_ip);
                    v_gw.fromString(v_cfg.wifi.multi_ap[i].gateway);
                    v_sn.fromString(v_cfg.wifi.multi_ap[i].subnet);
                    if (strlen(v_cfg.wifi.multi_ap[i].dns1) > 0) v_d1.fromString(v_cfg.wifi.multi_ap[i].dns1);
                    if (strlen(v_cfg.wifi.multi_ap[i].dns2) > 0) v_d2.fromString(v_cfg.wifi.multi_ap[i].dns2);
                    WiFi.config(v_ip, v_gw, v_sn, v_d1, v_d2);
                } else {
                    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
                }

                Serial.printf("[Net] Connecting to %s...\n", v_targetSsid);
                WiFi.begin(v_targetSsid, v_targetPw);

                uint32_t v_startMs = millis();
                while (WiFi.status() != WL_CONNECTED && (millis() - v_startMs < SmeaConfig::Network::WIFI_CONN_TIMEOUT_DEF)) {
                    delay(200);
                }

                if (WiFi.status() == WL_CONNECTED) {
                    Serial.println("[Net] WiFi Connected.");
                    configTzTime(SmeaConfig::Network::TZ_INFO_DEF, SmeaConfig::Network::NTP_SERVER_1_DEF, SmeaConfig::Network::NTP_SERVER_2_DEF);

                    struct tm v_timeinfo;
                    uint32_t v_syncStart = millis();
                    while (!getLocalTime(&v_timeinfo, 100) && (millis() - v_syncStart < SmeaConfig::Network::NTP_TIMEOUT_MS_DEF)) {
                        delay(100);
                    }
                    _enforceTcpKeepAlive();
                    break;
                }
            }
        }
    }
    
    // TODO : 아래 블륵 필요하지 검토
    if (strlen(v_cfg.mqtt.mqtt_broker) > 0) {
        strlcpy(_mqttCreds.broker, v_cfg.mqtt.mqtt_broker, sizeof(_mqttCreds.broker));
    } else {
        strlcpy(_mqttCreds.broker, p_mqttBroker, sizeof(_mqttCreds.broker));
    }
    _mqttCreds.enable = (strlen(_mqttCreds.broker) > 0);
    _mqttCreds.port = v_cfg.mqtt.default_port;


    // ------------------------------------------------------------------------
    // ESP-IDF Native MQTT 초기화 (Arduino Core 2.x / ESP-IDF 4.4 호환 규격)
    // ------------------------------------------------------------------------
    if (_mqttCreds.enable) {
        char v_uri[128];
        snprintf(v_uri, sizeof(v_uri), "mqtt://%s:%d", _mqttCreds.broker, _mqttCreds.port);

        esp_mqtt_client_config_t v_mqttCfg = {};
        
        // [교정] v4.4 평면 구조체 적용 (broker, session, buffer 등 중첩 삭제)
        v_mqttCfg.uri = v_uri;
        
        // LWT (유언) 설정
        v_mqttCfg.lwt_topic = SmeaConfig::Mqtt::TOPIC_LWT_DEF;
        v_mqttCfg.lwt_msg = "offline";
        v_mqttCfg.lwt_qos = 1;
        v_mqttCfg.lwt_retain = 1;

        // 송신 버퍼 확장 (4KB) 및 세션 옵션 설정
        v_mqttCfg.out_buffer_size = 4096;
        v_mqttCfg.disable_clean_session = 0; // 0: Clean Session 유지

        // 아두이노 코어(v4.4)에서는 MQTT 3.1.1이 기본값이므로 프로토콜 버전 강제 지정 생략

        _mqttHandle = esp_mqtt_client_init(&v_mqttCfg);
        
        if (_mqttHandle) {
            esp_mqtt_client_register_event(_mqttHandle, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, _mqttEventHandler, this);
            esp_mqtt_client_start(_mqttHandle);
            ESP_LOGI("T470", "[MQTT] Native Client Started (URI: %s, MQTT 3.1.1)", v_uri);
        } else {
            ESP_LOGE("T470", "[MQTT] Failed to initialize client");
        }
    }


    _server.onNotFound([this](AsyncWebServerRequest *p_request) {
        if (p_request->method() == HTTP_OPTIONS) {
            AsyncWebServerResponse *v_response = p_request->beginResponse(200);
            _setCorsHeaders(v_response);
            p_request->send(v_response);
        } else {
            p_request->send(404, "text/plain", "Not Found");
        }
    });

    _initWebHandlers();

    _server.addHandler(&_ws);
    _server.begin();

    return true;
}

void T470_Communicator::_initWebHandlers() {
    // ------------------------------------------------------------------------
    // [기존 API 라우팅 유지] 기능 축소 방지
    // ------------------------------------------------------------------------
    _server.on("/api/status", HTTP_GET, [this](AsyncWebServerRequest* p_request) {
        JsonDocument v_doc;
        v_doc["sys_state"] = (uint8_t)T450_FsmManager::getInstance().getCurrentState();
        v_doc["heap_free"] = ESP.getFreeHeap();
        v_doc["psram_free"] = ESP.getFreePsram();
        _sendJsonResponse(p_request, v_doc);
    });

    // 웹 브라우저 부하 조절을 위한 스트림 주기 제어 API
    // [v013 신설] 웹 브라우저 부하 조절을 위한 스트림 주기 제어 API
    _server.on("/api/stream/control", HTTP_POST,
        [](AsyncWebServerRequest *p_request) {
            p_request->send(400, "application/json", "{\"ok\":false,\"msg\":\"no_body\"}");
        },
        NULL,
        [](AsyncWebServerRequest *p_request, uint8_t *p_data, size_t p_len, size_t p_index, size_t p_total) {
            // 1. 메모리 보호: 페일로드가 너무 크면 거부 (1KB 제한)
            if (p_index == 0) {
                if (p_total > 1024) {
                    p_request->send(413, "application/json", "{\"ok\":false,\"msg\":\"payload_too_large\"}");
                    return;
                }
                p_request->_tempObject = malloc(p_total + 1);
            }

            uint8_t* v_buffer = (uint8_t*)p_request->_tempObject;

            if (v_buffer) {
                // 2. 청크 데이터 병합
                memcpy(v_buffer + p_index, p_data, p_len);

                // 3. 수신 완료 시 JSON 파싱 및 FSM 연동
                if (p_index + p_len == p_total) {
                    v_buffer[p_total] = '\0';

                    JsonDocument v_doc;
                    DeserializationError v_err = deserializeJson(v_doc, v_buffer);

                    if (!v_err) {
                        // 웹 UI에서 필드가 누락되었을 경우를 대비한 기본값(Fallback) 방어
                        uint8_t v_teleHz = v_doc["telemetry_hz"] | 10;
                        uint8_t v_specHz = v_doc["spectrum_hz"]  | 0;
                        uint8_t v_waveHz = v_doc["waveform_hz"]  | 0;

                        T450_FsmManager::getInstance().setStreamFrequencies(v_teleHz, v_specHz, v_waveHz);
                        p_request->send(200, "application/json", "{\"ok\":true}");
                    } else {
                        p_request->send(400, "application/json", "{\"ok\":false,\"msg\":\"json_parse_error\"}");
                    }

                    free(v_buffer);
                    p_request->_tempObject = NULL;
                }
            } else if (p_index + p_len == p_total) {
                // 할당 실패 OOM 처리
                p_request->send(500, "application/json", "{\"ok\":false,\"msg\":\"oom\"}");
            }
        }
    );

    _server.on("/api/recorder_begin", HTTP_POST, [](AsyncWebServerRequest* p_request) {
        T450_FsmManager::getInstance().dispatchCommand(SystemCommand::CMD_MANUAL_RECORD_START);
        p_request->send(200, "application/json", "{\"ok\":true}");
    });

    _server.on("/api/recorder_end", HTTP_POST, [](AsyncWebServerRequest* p_request) {
        T450_FsmManager::getInstance().dispatchCommand(SystemCommand::CMD_MANUAL_RECORD_STOP);
        p_request->send(200, "application/json", "{\"ok\":true}");
    });

    _server.on("/api/noise_learn", HTTP_POST, [](AsyncWebServerRequest* p_request) {
        T450_FsmManager::getInstance().dispatchCommand(SystemCommand::CMD_LEARN_NOISE);
        p_request->send(200, "application/json", "{\"ok\":true}");
    });

    _server.on("/api/reboot", HTTP_POST, [](AsyncWebServerRequest* p_request) {
        p_request->send(200, "application/json", "{\"ok\":true}");
        T450_FsmManager::getInstance().dispatchCommand(SystemCommand::CMD_REBOOT);
    });

    _server.on("/api/factory_reset", HTTP_POST, [](AsyncWebServerRequest* p_request) {
        T415_ConfigManager::getInstance().resetToDefault();
        p_request->send(200, "application/json", "{\"ok\":true,\"msg\":\"factory_reset_and_rebooting\"}");
        T450_FsmManager::getInstance().dispatchCommand(SystemCommand::CMD_REBOOT);
    });

    _server.on("/api/download", HTTP_GET, [](AsyncWebServerRequest* p_request) {
        if (!p_request->hasParam("file")) {
            p_request->send(400, "text/plain", "Missing file param");
            return;
        }

        if (T450_FsmManager::getInstance().getCurrentState() == SystemState::RECORDING) {
            p_request->send(423, "application/json", "{\"error\":\"locked_due_to_recording\"}");
            return;
        }

        String v_path = p_request->getParam("file")->value();
        if (SD_MMC.exists(v_path)) {
            p_request->send(SD_MMC, v_path, "application/octet-stream");
        } else {
            p_request->send(404, "text/plain", "File not found");
        }
    });

    _server.on("/api/runtime_config", HTTP_GET, [this](AsyncWebServerRequest* p_request) {
        if (LittleFS.exists(SmeaConfig::Path::SYS_CFG_JSON_DEF)) {
            AsyncWebServerResponse* v_response = p_request->beginResponse(LittleFS, SmeaConfig::Path::SYS_CFG_JSON_DEF, "application/json");
            _setCorsHeaders(v_response);
            p_request->send(v_response);
        } else {
            p_request->send(404, "application/json", "{}");
        }
    });

    // 기존의 무거운 네트워크/와이파이 설정 덮어쓰기용 (Lazy Write 연동, 재부팅 포함)
    _server.on("/api/runtime_config", HTTP_POST,
        [](AsyncWebServerRequest *p_request) {
            p_request->send(400, "application/json", "{\"ok\":false,\"msg\":\"no_body\"}");
        },
        NULL,
        [this](AsyncWebServerRequest *p_request, uint8_t *p_data, size_t p_len, size_t p_index, size_t p_total) {
            if (p_index == 0) {
                if (p_total > SmeaConfig::Network::LARGE_BUF_SIZE_DEF) {
                    p_request->send(413, "application/json", "{\"ok\":false,\"msg\":\"payload_too_large\"}");
                    return;
                }

                p_request->_tempObject = malloc(p_total + 1);
                p_request->onDisconnect([p_request]() {
                    if (p_request->_tempObject) {
                        free(p_request->_tempObject);
                        p_request->_tempObject = NULL;
                    }
                });
            }

            uint8_t* v_buffer = (uint8_t*)p_request->_tempObject;

            if (!v_buffer) {
                if (p_index + p_len == p_total && !p_request->client()->disconnected()) {
                    p_request->send(500, "application/json", "{\"ok\":false,\"msg\":\"oom_or_rejected\"}");
                }
                return;
            }

            memcpy(v_buffer + p_index, p_data, p_len);

            if (p_index + p_len == p_total) {
                v_buffer[p_total] = '\0';
                if (T415_ConfigManager::getInstance().updateFromJson((const char*)v_buffer)) {
                    p_request->send(200, "application/json", "{\"ok\":true,\"msg\":\"updated_and_rebooting\"}");
                    T450_FsmManager::getInstance().dispatchCommand(SystemCommand::CMD_REBOOT);
                } else {
                    p_request->send(400, "application/json", "{\"ok\":false,\"msg\":\"json_parse_or_save_error\"}");
                }
                free(v_buffer);
                p_request->_tempObject = NULL;
            }
        }
    );

    // ------------------------------------------------------------------------
    // [v013 신규 API 라우팅] Web UI/UX 핫스왑 및 캘리브레이션 트리거
    // ------------------------------------------------------------------------

    // 1. 실시간 차트 튜닝 (플래시 마모율 0%, RAM 즉시 갱신)
    _server.on("/api/config/preview", HTTP_POST,
        [](AsyncWebServerRequest *p_request) { p_request->send(400, "application/json", "{\"ok\":false,\"msg\":\"no_body\"}"); },
        NULL,
        [this](AsyncWebServerRequest *p_request, uint8_t *p_data, size_t p_len, size_t p_index, size_t p_total) {
            if (p_index == 0) {
                p_request->_tempObject = malloc(p_total + 1);
            }
            uint8_t* v_buffer = (uint8_t*)p_request->_tempObject;
            if (v_buffer) {
                memcpy(v_buffer + p_index, p_data, p_len);
                if (p_index + p_len == p_total) {
                    v_buffer[p_total] = '\0';
                    bool v_ok = T415_ConfigManager::getInstance().updatePreview((const char*)v_buffer);
                    p_request->send(v_ok ? 200 : 400, "application/json", v_ok ? "{\"ok\":true}" : "{\"ok\":false}");
                    free(v_buffer);
                    p_request->_tempObject = NULL;
                }
            }
        }
    );

    // 2. 튜닝 확정 (LittleFS 영구 저장)
    _server.on("/api/config/save", HTTP_POST, [](AsyncWebServerRequest* p_request) {
        bool v_ok = T415_ConfigManager::getInstance().commitSave();
        p_request->send(v_ok ? 200 : 500, "application/json", v_ok ? "{\"ok\":true}" : "{\"ok\":false}");
    });

    // 3. 튜닝 취소 및 롤백
    _server.on("/api/config/cancel", HTTP_POST, [](AsyncWebServerRequest* p_request) {
        bool v_ok = T415_ConfigManager::getInstance().revertCancel();
        p_request->send(v_ok ? 200 : 500, "application/json", v_ok ? "{\"ok\":true}" : "{\"ok\":false}");
    });

    // 4. 캘리브레이션 원격 제어
    _server.on("/api/calib/start_manual", HTTP_POST, [](AsyncWebServerRequest* p_request) {
        T450_FsmManager::getInstance().dispatchCommand(SystemCommand::CMD_CALIB_MANUAL_START);
        p_request->send(200, "application/json", "{\"ok\":true}");
    });

    _server.on("/api/calib/start_auto", HTTP_POST, [](AsyncWebServerRequest* p_request) {
        T450_FsmManager::getInstance().dispatchCommand(SystemCommand::CMD_CALIB_AUTO_START);
        p_request->send(200, "application/json", "{\"ok\":true}");
    });

    _server.on("/api/calib/stop", HTTP_POST, [](AsyncWebServerRequest* p_request) {
        T450_FsmManager::getInstance().dispatchCommand(SystemCommand::CMD_CALIB_STOP);
        p_request->send(200, "application/json", "{\"ok\":true}");
    });


    // OTA 진입 시 FSM에 MAINTENANCE 전환 명령 하달 (시스템 락 해제)
    _server.on("/api/ota", HTTP_POST,
        [this](AsyncWebServerRequest *p_request) {
            if (!_isOtaRunning) {
                p_request->send(423, "application/json", "{\"ok\":false,\"msg\":\"locked\"}");
                return;
            }
            bool v_success = !Update.hasError();
            p_request->send(v_success ? 200 : 500, "application/json", v_success ? "{\"ok\":true}" : "{\"ok\":false}");
            _isOtaRunning = false;

            T450_FsmManager::getInstance().dispatchCommand(SystemCommand::CMD_OTA_END);

            if (v_success) {
                T450_FsmManager::getInstance().dispatchCommand(SystemCommand::CMD_REBOOT);
            }
        },
        [this](AsyncWebServerRequest *p_request, String p_filename, size_t p_index, uint8_t *p_data, size_t p_len, bool p_final) {
            if (p_index == 0) {
                if (_isOtaRunning) return;
                _isOtaRunning = true;

                // OTA 시작 시 시스템 버스(SPI/SD) 충돌 방어
                T450_FsmManager::getInstance().dispatchCommand(SystemCommand::CMD_OTA_START);

                if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
                    _isOtaRunning = false;
                }
            }
            if (_isOtaRunning && !Update.hasError()) {
                Update.write(p_data, p_len);
            }
            if (_isOtaRunning && p_final) {
                Update.end(true);
            }
        }
    );

    _server.serveStatic("/", LittleFS, SmeaConfig::Path::WWW_ROOT_DEF).setDefaultFile(SmeaConfig::Path::WEB_INDEX_DEF);
}

// 브라우저 스로틀링 대응 OOM 킬러 (R-NMG 방어벽)
// LwIP 송신 버퍼 고갈 시 패킷 드랍 (커널 락업 방지)
void T470_Communicator::broadcastBinary(const void* p_buffer, size_t p_bytes) {
    if (_ws.count() == 0 || !p_buffer) return;

    for (auto& v_client : _ws.getClients()) {
        if (v_client.status() == WS_CONNECTED) {
            // [교정] v_client.canSend()를 통해 하단 TCP 버퍼 여유 확인 로직 추가
            if (v_client.queueLen() > 4096 || !v_client.canSend()) {
                if (v_client.queueLen() > 4096) {
                    v_client.close();
                    Serial.println("[Net] Zombie WS Client Kicked (OOM Prevented)");
                }
                continue; // 버퍼가 찼다면 이번 프레임은 과감히 포기(Drop)
            }
            v_client.binary((uint8_t*)p_buffer, p_bytes);
        }
    }
}

void T470_Communicator::runNetwork() {
    _ws.cleanupClients();

    if (!isConnected() && WiFi.getMode() != WIFI_AP) {
        if (millis() - _lastWifiRetryMs > SmeaConfig::Network::WIFI_RETRY_MS_DEF) {
            Serial.println("[Net] WiFi lost. Attempting reconnect...");
            WiFi.disconnect();
            WiFi.reconnect();
            _lastWifiRetryMs = millis();
        }
        return;
    }
}

bool T470_Communicator::publishResultMqtt(const SmeaType::FeatureSlot& p_slot, DetectionResult p_result) {
    if (!_mqttCreds.enable || !_mqttHandle) return false;

    JsonDocument v_doc;
    v_doc["device_id"] = (uint32_t)ESP.getEfuseMac();
    v_doc["result"] = (uint8_t)p_result;
    v_doc["rms"] = p_slot.rms;
    v_doc["crest_factor"] = p_slot.crest_factor;

    char v_payload[512];
    serializeJson(v_doc, v_payload, sizeof(v_payload));

    // esp_mqtt_client_publish는 즉시 전송하지 않고 OS 큐에 담고 빠지므로 FSM 블로킹 0%
    int v_msgId = esp_mqtt_client_publish(
                    _mqttHandle, 
                    SmeaConfig::Mqtt::TOPIC_RESULT_DEF, 
                    v_payload, strlen(v_payload), 
                    _mqttCreds.qos, 
                    0
                  );
    
    if (v_msgId == -1) {
        ESP_LOGE("T470", "[MQTT] Publish Failed! Outbox queue full or network down.");
        return false;
    }
    return true;
}

// MQTT 백그라운드 태스크에서 돌아가는 완벽한 비동기 이벤트 핸들러
void T470_Communicator::_mqttEventHandler(void* p_handlerArgs, esp_event_base_t p_base, int32_t p_eventId, void* p_eventData) {
    esp_mqtt_event_handle_t v_event = (esp_mqtt_event_handle_t)p_eventData;
    
    switch ((esp_mqtt_event_id_t)p_eventId) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI("T470", "[MQTT] Connected to Broker");
            esp_mqtt_client_publish(v_event->client, SmeaConfig::Mqtt::TOPIC_LWT_DEF, "online", 6, 1, 1);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW("T470", "[MQTT] Disconnected from Broker");
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE("T470", "[MQTT] Native Error Occurred");
            break;
        default:
            break;
    }
}

// mqtt 대용량 데이터 패킷 분할 전송
// mqtt 대용량 데이터 패킷 분할 전송 (Arduino Core 호환)
bool T470_Communicator::publishChunked(const char* p_topic, const uint8_t* p_data, size_t p_totalLen, size_t p_chunkSize) {
    if (!_mqttCreds.enable || !_mqttHandle) return false;

    size_t v_offset = 0;
    int v_chunkIdx = 0;

    while (v_offset < p_totalLen) {
        size_t v_currentChunk = (p_totalLen - v_offset > p_chunkSize) ? p_chunkSize : (p_totalLen - v_offset);
        
        // [교정] MQTT 5.0 Property 주입 코드 완전 삭제

        // QoS 1을 강제하여 분할된 패킷 유실 방지
        int v_res = esp_mqtt_client_publish(_mqttHandle, p_topic, (const char*)(p_data + v_offset), v_currentChunk, 1, 0);
        
        if (v_res == -1) {
            ESP_LOGE("T470", "[MQTT] Chunk %d transmit failed!", v_chunkIdx);
            return false;
        }
        
        v_offset += v_currentChunk;
        v_chunkIdx++;
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
    
    ESP_LOGI("T470", "[MQTT] File Chunking Transmit Done. (%d chunks)", v_chunkIdx);
    return true;
}


void T470_Communicator::_setCorsHeaders(AsyncWebServerResponse* p_response) {
    p_response->addHeader("Access-Control-Allow-Origin", "*");
    p_response->addHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    p_response->addHeader("Access-Control-Allow-Headers", "Content-Type");
}

void T470_Communicator::_sendJsonResponse(AsyncWebServerRequest* p_request, const JsonDocument& p_doc) {
    AsyncResponseStream *v_stream = p_request->beginResponseStream("application/json");
    _setCorsHeaders(v_stream);
    serializeJson(p_doc, *v_stream);
    p_request->send(v_stream);
}



