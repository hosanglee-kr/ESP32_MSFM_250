/* ============================================================================
 * File: T219_Config_234.h
 * Summary: System Configuration JSON Serialization
 * * [AI 메모: 제공 기능 요약]
 * 1. T210_Def_231.h에 정의된 ST_T20_Config_t 마스터 구조체와 JSON 간 양방향 변환.
 * 2. ArduinoJson V7.4.x의 동적 메모리 할당(JsonDocument) 방식 완벽 지원.
 * 3. 부분 업데이트 지원 (JSON에 없는 필드는 기존 구조체의 값을 보존함).
 * * [AI 메모: 구현 및 유지보수 주의사항]
 * 1. ArduinoJson V7부터는 JsonObjectConst 및 JsonArrayConst를 사용하여 읽기 전용 
 * 객체를 다뤄야 합니다. (const 파라미터 전달 시 타입 에러 방지)
 * 2. ENUM 타입(EM_T20_...)은 정수형(int)으로 캐스팅하여 직렬화/역직렬화해야 합니다.
 * 3. 새로운 설정 항목이 추가되면 이 클래스의 parseFromJson과 buildJson 양쪽 
 * 모두에 반드시 업데이트를 누락 없이 반영해야 합니다.
 * ========================================================================== */
#pragma once

#include <ArduinoJson.h>
#include "T210_Def_234.h"

class CL_T20_ConfigJson {
public:
    // JSON 문자열을 구조체로 파싱 (기존 설정값을 덮어씀)
    static bool parseFromJson(const char* json_str, ST_T20_Config_t& out_cfg);
    static bool parseFromJson(const JsonDocument& doc, ST_T20_Config_t& out_cfg);

    // 구조체를 JSON 문자열로 변환
    static void buildJson(const ST_T20_Config_t& cfg, JsonDocument& out_doc);
    static void buildJsonString(const ST_T20_Config_t& cfg, String& out_str);
};

