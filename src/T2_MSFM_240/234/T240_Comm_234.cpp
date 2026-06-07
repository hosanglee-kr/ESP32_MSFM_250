/* ============================================================================
 * File: T240_Comm_234.cpp
 * Summary: Network, MQTT & OTA Implementation (FSM Command Queue Integration)
 * Description: HTTP 요청을 받아 메인 FSM(유한 상태 머신)으로 비동기 명령(Command)을 
 * 하달하고, 현재 시스템의 정확한 런타임 상태를 웹 UI로 응답하는 통신 인터페이스입니다.
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
 * ========================================================================== */

#include "T240_Comm_234.h"
#include "T221_Mfcc_Int_234.h" 
#include <LittleFS.h>
#include <SD_MMC.h>
#include <time.h> 
#include <Update.h> 

CL_T20_CommService::CL_T20_CommService()
    : _server(80), _ws(T20::C10_Web::WS_URI), _mqtt_client(_wifi_client) {
    _last_mqtt_retry_ms = 0;
    _last_wifi_retry_ms = 0; // [C19/C24 방어] 논블로킹 WiFi 타이머 초기화
}

CL_T20_CommService::~CL_T20_CommService() {
    _ws.closeAll();
    _server.end();
}

bool CL_T20_CommService::begin(const ST_T20_Config_t& cfg) {

    _mqtt_cfg = cfg.mqtt;
    const ST_T20_ConfigWiFi_t& w_cfg = cfg.wifi;
    
    WiFi.mode(WIFI_OFF);
    delay(50);

    // [1] AP 모드 및 커스텀 IP 설정
    if (w_cfg.mode == EN_T20_WIFI_AP_ONLY || w_cfg.mode == EN_T20_WIFI_AP_STA || w_cfg.mode == EN_T20_WIFI_AUTO_FALLBACK) {
        if (w_cfg.ap_ip[0] != '\0') {
            IPAddress apIP;
            if (apIP.fromString(w_cfg.ap_ip)) {
                WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
            }
        }
        WiFi.softAP(w_cfg.ap_ssid, w_cfg.ap_password);
    }

    // [2] STA 모드 (개별 라우터별 고정IP/DHCP 적용 및 순차 접속)
    if (w_cfg.mode != EN_T20_WIFI_AP_ONLY) {
        WiFi.mode(w_cfg.mode == EN_T20_WIFI_AP_STA ? WIFI_AP_STA : WIFI_STA);

        for (int i = 0; i < T20::C10_Net::WIFI_MULTI_MAX; i++) {
            if (w_cfg.multi_ap[i].ssid[0] != '\0') {

                WiFi.disconnect(); 
                delay(100);

                if (w_cfg.multi_ap[i].use_static_ip) {
                    IPAddress ip, gw, sn, d1, d2;
                    ip.fromString(w_cfg.multi_ap[i].local_ip);
                    gw.fromString(w_cfg.multi_ap[i].gateway);
                    sn.fromString(w_cfg.multi_ap[i].subnet);
                    if (w_cfg.multi_ap[i].dns1[0] != '\0') d1.fromString(w_cfg.multi_ap[i].dns1);
                    if (w_cfg.multi_ap[i].dns2[0] != '\0') d2.fromString(w_cfg.multi_ap[i].dns2);
                    WiFi.config(ip, gw, sn, d1, d2);
                } else {
                    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
                }

                WiFi.begin(w_cfg.multi_ap[i].ssid, w_cfg.multi_ap[i].password);

                uint32_t start_ms = millis();
                while (WiFi.status() != WL_CONNECTED && (millis() - start_ms < 4000)) {
                    delay(200);
                }

                if (WiFi.status() == WL_CONNECTED) {
                    configTzTime(T20::C10_Time::TZ_INFO, T20::C10_Time::NTP_SERVER_1, T20::C10_Time::NTP_SERVER_2);

                    struct tm timeinfo;
                    uint32_t start_time = millis();
                    while (!getLocalTime(&timeinfo, 100) && (millis() - start_time < T20::C10_Time::SYNC_TIMEOUT_MS)) {
                        delay(100);
                    }
                    break;
                }
            }
        }
    }
    
    // MQTT 서버 설정
    if (_mqtt_cfg.enable && isConnected()) {
        _mqtt_client.setServer(_mqtt_cfg.broker, _mqtt_cfg.port);
    }

    // [3] WebSocket 바인딩
    _ws.onEvent([](AsyncWebSocket* s, AsyncWebSocketClient* c, AwsEventType t, void* a, uint8_t* d, size_t l) {
        // WS 연결 이벤트 핸들링 (연결 수립/해제 로깅 등 필요시 추가)
    });
    _server.addHandler(&_ws);

    // [4] CORS Preflight 전역 허용
    _server.onNotFound([this](AsyncWebServerRequest *request) {
        if (request->method() == HTTP_OPTIONS) {
            AsyncWebServerResponse *response = request->beginResponse(200);
            _setCorsHeaders(response);
            request->send(response);
        } else {
            request->send(404, "text/plain", "Not Found");
        }
    });

    _server.begin();
    return true;
}

void CL_T20_CommService::initHandlers(void* p_master_impl) {
    CL_T20_Mfcc::ST_Impl* p = (CL_T20_Mfcc::ST_Impl*)p_master_impl;
    if (!p) return;

    // ========================================================================
    // 1. 상태 및 모니터링 API (FSM 상태 동기화)
    // ========================================================================
    _server.on("/api/t20/status", HTTP_GET, [this, p](AsyncWebServerRequest* request) {
        JsonDocument doc;
        doc["running"] = p->running;
        doc["sys_state"] = (int)p->current_state; 
        
        bool is_active = (p->current_state >= EN_T20_STATE_MONITORING);
        doc["hash"] = calcStatusHash(p->sample_counter, p->storage.getRecordCount(), is_active); 
        
        doc["sensor_status"] = p->sensor.getStatusText();
        doc["storage_open"] = p->storage.isOpen();
        _sendJson(request, doc);
    });

    // ========================================================================
    // 2. 레코더 및 커맨드 제어 API (FSM Command Queue 하달)
    // ========================================================================
    _server.on("/api/t20/recorder_begin", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (g_t20) g_t20->postCommand(EN_T20_CMD_START);
        req->send(200, "application/json", "{\"ok\":true}");
    });

    _server.on("/api/t20/recorder_end", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (g_t20) g_t20->postCommand(EN_T20_CMD_STOP);
        req->send(200, "application/json", "{\"ok\":true}");
    });

    _server.on("/api/t20/noise_learn", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (g_t20) g_t20->postCommand(EN_T20_CMD_LEARN_NOISE);
        req->send(200, "application/json", "{\"ok\":true}");
    });

    _server.on("/api/t20/calibrate", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (g_t20) g_t20->postCommand(EN_T20_CMD_CALIBRATE);
        req->send(200, "application/json", "{\"ok\":true}");
    });

    // ========================================================================
    // 3. 파일 스트리밍 및 다운로드 API
    // ========================================================================
    _server.on("/api/t20/recorder_index", HTTP_GET, [this](AsyncWebServerRequest* request) {
        if (!LittleFS.exists("/sys/recorder_index.json")) {
            request->send(404, "application/json", "{\"error\":\"not_found\"}");
            return;
        }
        AsyncWebServerResponse* response = request->beginResponse(LittleFS, "/sys/recorder_index.json", "application/json");
        _setCorsHeaders(response);
        request->send(response);
    });

    _server.on("/api/t20/recorder/download", HTTP_GET, [this, p](AsyncWebServerRequest* request) {
        if (!request->hasParam("path")) {
            request->send(400, "text/plain", "path_required");
            return;
        }
        
        // [B11 방어] SD카드에 맹렬히 기록 중일 때 대용량 다운로드 요청 시 
        // SPI 버스 병목으로 레코더 큐가 마비되는 현상(데이터 증발)을 막기 위해 423 Locked 반환
        if (p->current_state == EN_T20_STATE_RECORDING) {
            request->send(423, "application/json", "{\"error\":\"locked_due_to_recording\"}");
            return;
        }

        String path = request->getParam("path")->value();

        if (SD_MMC.exists(path)) {
            AsyncWebServerResponse *response = request->beginResponse(SD_MMC, path, "application/octet-stream");
            _setCorsHeaders(response);
            request->send(response);
        } else if (LittleFS.exists(path)) {
            AsyncWebServerResponse *response = request->beginResponse(LittleFS, path, "application/octet-stream");
            _setCorsHeaders(response);
            request->send(response);
        } else {
            request->send(404, "text/plain", "file_not_found");
        }
    });

    // ========================================================================
    // 4. 시스템 제어 및 OTA
    // ========================================================================
    _server.on("/api/t20/reboot", HTTP_POST, [this](AsyncWebServerRequest* request) {
        request->send(200, "application/json", "{\"ok\":true}");
        // [B9 방어] 비동기 스레드 블로킹 철거. 메인 FSM에 재부팅 위임
        if (g_t20) g_t20->postCommand(EN_T20_CMD_REBOOT); 
    });

    _server.on("/api/t20/runtime_config", HTTP_GET, [this](AsyncWebServerRequest* request) {
        if (LittleFS.exists(T20::C10_Path::FILE_CFG_JSON)) {
            AsyncWebServerResponse* response = request->beginResponse(LittleFS, T20::C10_Path::FILE_CFG_JSON, "application/json");
            _setCorsHeaders(response);
            request->send(response);
        } else {
            request->send(404, "application/json", "{}");
        }
    });
    
    _server.on("/api/t20/runtime_config", HTTP_POST,
        [](AsyncWebServerRequest *request) {
            request->send(400, "application/json", "{\"ok\":false,\"msg\":\"no_body\"}");
        },
        NULL,
        [this](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {

            if (index == 0) {
                if (total > T20::C10_Web::LARGE_JSON_BUF_SIZE) {
                    request->send(413, "application/json", "{\"ok\":false,\"msg\":\"payload_too_large\"}");
                    return;
                }
                
                request->_tempObject = malloc(total + 1);
                request->onDisconnect([request]() {
                    if (request->_tempObject) {
                        free(request->_tempObject);
                        request->_tempObject = NULL;
                    }
                });
            }
            
            uint8_t* buffer = (uint8_t*)request->_tempObject;
            
            // [B10 상태 붕괴 방어] OOM 발생 시 매 청크마다 send(500)을 동시 호출하는 치명적 에러 차단
            if (!buffer) {
                if (index + len == total && !request->client()->disconnected()) { 
                    request->send(500, "application/json", "{\"ok\":false,\"msg\":\"oom_or_rejected\"}");
                }
                return;
            }

            memcpy(buffer + index, data, len);

			if (index + len == total) {
				buffer[total] = '\0';

				JsonDocument doc;
				DeserializationError err = deserializeJson(doc, buffer);

				if (!err) {
					File f = LittleFS.open(T20::C10_Path::FILE_CFG_JSON, "w");
					if (f) {
						serializeJson(doc, f);
						f.close();
						request->send(200, "application/json", "{\"ok\":true,\"msg\":\"rebooting\"}");
						// [B9 방어] FSM 연계 안전 재부팅
						if (g_t20) g_t20->postCommand(EN_T20_CMD_REBOOT); 
					} else {
						request->send(500, "application/json", "{\"ok\":false,\"msg\":\"fs_error\"}");
					}
				} else {
					request->send(400, "application/json", "{\"ok\":false,\"msg\":\"json_error\"}");
				}
				free(buffer);
				request->_tempObject = NULL;
            }
        }
    );
    
    // [B16/B17 방어] 동시 다발적 OTA 시도에 의한 하드웨어 파티션 병합 충돌 방어 락
    static bool g_ota_running = false;

    _server.on("/api/t20/ota_update", HTTP_POST, 
        [this](AsyncWebServerRequest *request) {
            if (!g_ota_running) {
                request->send(423, "application/json", "{\"ok\":false,\"msg\":\"locked_by_another_ota\"}");
                return;
            }
            
            bool success = !Update.hasError();
            AsyncWebServerResponse *response = request->beginResponse(success ? 200 : 500, "application/json", success ? "{\"ok\":true}" : "{\"ok\":false}");
            _setCorsHeaders(response);
            request->send(response);
            
            g_ota_running = false; 
            
            if (success) {
                // [B9 방어] FSM 연계 안전 재부팅
                if (g_t20) g_t20->postCommand(EN_T20_CMD_REBOOT); 
            }
        },
        [this](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
            if (index == 0) {
                if (g_ota_running) return; 
                g_ota_running = true;
                
                Serial.printf("[OTA] Update Start: %s\n", filename.c_str());
                if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
                    Update.printError(Serial);
                    g_ota_running = false;
                }
            }
            if (g_ota_running && !Update.hasError()) {
                if (Update.write(data, len) != len) {
                    Update.printError(Serial);
                    g_ota_running = false;
                }
            }
            if (g_ota_running && final) {
                if (Update.end(true)) {
                    Serial.printf("[OTA] Update Success: %u Bytes\n", index + len);
                } else {
                    Update.printError(Serial);
                    g_ota_running = false;
                }
            }
        }
    );

    // ========================================================================
    // 5. 프론트엔드 정적 파일 서빙
    // ========================================================================
    _server.serveStatic("/", LittleFS, "/www").setDefaultFile(T20::C10_Path::WEB_INDEX);
}

// [B8 방어] 이중 곱셈(sizeof(float)) 삭제. 외부에서 계산된 순수 바이트(Bytes) 크기로 정확히 전송
void CL_T20_CommService::broadcastBinary(const void* p_buffer, size_t bytes) {
    if (_ws.count() == 0 || !p_buffer) return;

    if (_ws.availableForWriteAll()) {
        _ws.binaryAll((uint8_t*)p_buffer, bytes);
    }
}

uint32_t CL_T20_CommService::calcStatusHash(uint32_t frame_id, uint32_t rec_count, bool measuring) {
    uint32_t h = 2166136261UL;
    h ^= frame_id;  h *= 16777619UL;
    h ^= rec_count; h *= 16777619UL;
    h ^= (measuring ? 0x80000000UL : 0UL);
    return h;
}

void CL_T20_CommService::_setCorsHeaders(AsyncWebServerResponse* response) {
    response->addHeader("Access-Control-Allow-Origin", "*");
    response->addHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    response->addHeader("Access-Control-Allow-Headers", "Content-Type");
}

void CL_T20_CommService::_sendJson(AsyncWebServerRequest* request, const JsonDocument& doc) {
    AsyncResponseStream *stream = request->beginResponseStream("application/json");
    _setCorsHeaders(stream);
    serializeJson(doc, *stream);
    request->send(stream);
}

// ========================================================================
// 네트워크 유지보수 및 MQTT 로직
// ========================================================================
// [B12/C19 방어] 좀비 소켓 회수(OOM 방어) 및 통신 단절 복구를 통합한 메인 네트워크 루프
void CL_T20_CommService::runNetwork() {
    
    // 1. 좀비 클라이언트 메모리 강제 회수 (TCP 버퍼 OOM 방어)
    _ws.cleanupClients();

    // 2. WiFi Auto-Reconnect (논블로킹 백오프 10초 적용)
    if (!isConnected() && WiFi.getMode() != WIFI_AP) {
        if (millis() - _last_wifi_retry_ms > 10000) { 
            Serial.println(F("[Net] WiFi lost. Attempting reconnect..."));
            WiFi.disconnect();
            WiFi.reconnect();
            _last_wifi_retry_ms = millis();
        }
        return; // WiFi 끊김 시 MQTT 루프 진입 차단
    }

    // 3. MQTT 유지보수
    if (_mqtt_cfg.enable) {
        if (!_mqtt_client.connected()) _reconnectMqtt();
        else _mqtt_client.loop(); 
    }
}

void CL_T20_CommService::_reconnectMqtt() {
    if (millis() - _last_mqtt_retry_ms > 5000) {
        Serial.print(F("[MQTT] Attempting connection..."));
        
        bool connected = false;
        if (strlen(_mqtt_cfg.password) > 0) {
            connected = _mqtt_client.connect(_mqtt_cfg.id, _mqtt_cfg.id, _mqtt_cfg.password);
        } else {
            connected = _mqtt_client.connect(_mqtt_cfg.id);
        }

        if (connected) {
            Serial.println(F(" connected"));
        } else {
            Serial.printf(" failed, rc=%d\n", _mqtt_client.state());
        }
        _last_mqtt_retry_ms = millis();
    }
}

bool CL_T20_CommService::publishMqtt(const char* sub_topic, const JsonDocument& doc) {
    if (!_mqtt_cfg.enable || !_mqtt_client.connected()) return false;

    char full_topic[128];
    snprintf(full_topic, sizeof(full_topic), "%s/%s", _mqtt_cfg.topic_root, sub_topic);

    char payload[512];
    serializeJson(doc, payload, sizeof(payload));

    return _mqtt_client.publish(full_topic, payload);
}

