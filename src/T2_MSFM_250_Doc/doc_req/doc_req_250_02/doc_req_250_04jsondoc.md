검토 결과: 이슈 식별 정확, 메모리 안정성을 위한 구조 개선이 시급
JsonDocument V7의 핵심 설계 철학인 “재사용 가능한 단일 버퍼 풀링(Pooling)” 이 전혀 적용되지 않았습니다. 
이로 인해 ESP32-S3의 제한된 내부 힙(DRAM)에서 장기간 운용 시 치명적인 OOM 및 성능 저하가 발생할 수밲에 없습니다.

---

1. 현 코드의 문제점 상세 분석

1.1 지역 변수로 매번 할당/해제 → 힙 단편화 가속

영향을 받는 주요 지점 (모두 함수 내부에서 JsonDocument v_doc; 선언)

파일 함수 트리거 빈도
T220_CfgMgr_250.cpp load() 부팅 및 복구 시 1회
T220_CfgMgr_250.cpp save() 설정 저장마다 (수동/지연쓰기)
T220_CfgMgr_250.cpp updateFromJson() WebSocket/API 설정 변경 시 수시
T220_CfgMgr_250.cpp updatePreview() 튜닝 모드에서 반복 호출
T220_CfgMgr_250.cpp serializeToBuffer() 웹 대시보드 갱신 시 주기적 호출
T270_Commu_250.cpp /api/status 핸들러 HTTP 요청마다 할당/해제
T270_Commu_250.cpp publishResultMqtt() 감지 이벤트 발생 시마다

JsonDocument 객체는 내부적으로 동적 할당된 버퍼를 가지며, 소멸 시 해제됩니다. 이러한 패턴이 반복되면 작은 크기의 메모리 청크가 빈번히 할당·해제되어 외부 단편화가 급격히 증가하고, 결국 충분한 연속 공간을 찾지 못해 할당 실패(→ JSON 직렬화 실패, 시스템 정지)로 이어집니다.

1.2 멤버 변수 풀(재사용 버퍼)의 부재

CL_T2_ConfigManager의 멤버 변수 목록에는 직렬화용 JsonDocument가 전혀 없습니다.

```cpp
// 현재 T220_CfgMgr_250.hpp
private:
    T2_Type::ST_DynamicConfig_t _dynConfig;
    SemaphoreHandle_t           _lock;
    // ...
    // JsonDocument 멤버 풀 없음!
```

때문에 모든 직렬화/역직렬화 작업이 임시 객체에 의존하고 있습니다.

1.3 shrinkToFit() 미활용

V7에서는 반복 사용 후 남은 잉여 버퍼를 줄이기 위해 shrinkToFit()을 명시적으로 호출해야 합니다. 현재 코드에는 이 API가 단 한 줄도 존재하지 않아, 사용하지 않는 메모리가 해제되지 않고 점유된 채로 남을 수 있습니다.

---

2. 개선 방안 (권장 아키텍처)

2.1 핵심 전략: ConfigManager에 재사용 가능한 JsonDocument 멤버 추가

설정 매니저가 JSON 변환을 독점하도록 하여, 단 하나의 버퍼를 스레드 안전하게 재활용합니다.

수정 헤더 (T220_CfgMgr_250.hpp)

```cpp
class CL_T2_ConfigManager {
private:
    T2_Type::ST_DynamicConfig_t _dynConfig;
    // ...

    // 추가: JSON 직렬화/역직렬화 전용 재사용 가능 버퍼
    JsonDocument _jsonBuffer;

    // _lock을 통해 보호됨 (이미 존재)
};
```

메서드 구현 변경 원칙

· 모든 JsonDocument v_doc; 지역 변수 선언을 제거하고, _jsonBuffer를 직접 사용.
· 사용 전 반드시 _lock을 획득한 상태에서 _jsonBuffer.clear()를 호출하여 이전 데이터를 지움.
· 역직렬화(deserialize) 또는 직렬화(serialize) 작업 수행.
· 함수 종료 시 락 해제 (기존 로직 유지).
· 필요 시, 설정 크기가 최대 용량을 초과하지 않도록 적절한 capacity를 지정하거나 동적 증가를 허용하되, shrinkToFit()으로 메모리 회수.

예시: save() 함수 수정

```cpp
bool CL_T2_ConfigManager::save() {
    xSemaphoreTake(_lock, portMAX_DELAY);
    
    // 1. 버퍼 클리어
    _jsonBuffer.clear();
    
    // 2. 직렬화 (기존 코드에서 v_doc → _jsonBuffer)
    JsonObject v_sys = _jsonBuffer["system"].to<JsonObject>();
    // ... (나머지 직렬화 로직 그대로)

    // 3. 파일 쓰기
    File v_tmp = LittleFS.open(...);
    serializeJson(_jsonBuffer, v_tmp);
    v_tmp.flush(); v_tmp.close();

    // 4. (선택) 사용 후 필요 이상 큰 버퍼 축소 (선택적)
    // _jsonBuffer.shrinkToFit();

    xSemaphoreGive(_lock);
    return true;
}
```

적용 대상 함수 목록
load(), save(), updateFromJson(), updatePreview(), serializeToBuffer() 모두 동일 패턴으로 변경.

2.2 T270_Commu의 임시 버퍼 대응

통신 계층에서는 ConfigManager의 버퍼를 직접 사용할 수 없습니다(별도 스레드, 다른 역할). 두 가지 선택지가 있습니다.

Option A: 스태틱 전역 JsonDocument 풀 생성

```cpp
// T270_Commu_250.cpp 상단
static JsonDocument g_commuJsonBuffer;
```

각 콜백에서 g_commuJsonBuffer.clear() 후 재사용. 단, WebServer의 비동기 응답 콜백이 여러 개 동시에 실행될 수 있으므로 SemaphoreHandle_t g_commuJsonMutex를 추가하여 보호해야 함.

Option B: 적정 크기로 미리 할당 후 shrink 관리
MQTT 페이로드(publishResultMqtt)는 크기가 작으므로, 함수 내에서 매번 할당해도 큰 위험은 적으나, 그래도 반복 호출 시 누적 효과를 고려해 정적 버퍼로 전환하는 것이 안전.

가장 좋은 방법: ConfigManager처럼 통신 클래스에도 멤버 _jsonBuffer를 두고, _ws와 _mqttHandle 관련 작업에서 이를 사용. 이미 CL_T2_Communicator 클래스가 싱글톤이 아니지만, 보통 하나의 인스턴스만 존재하므로 멤버 변수로 풀링해도 무방.

수정 예시
CL_T2_Communicator에 JsonDocument _jsonBuf; 추가 후,

· /api/status 핸들러: 람다 캡처로 this 접근 가능하게 설계 (단, 서버 라이브러리가 멀티태스킹에서 안전한지 확인 필요)
· publishResultMqtt: 멤버 _jsonBuf 사용

2.3 shrinkToFit() 정책

JsonDocument는 clear() 이후에도 이전에 할당된 최대 크기의 메모리를 유지합니다. 오디오 EQ 계수 등으로 인해 직렬화 크기가 최대 8~10KB에 달할 수 있는데, 소형 메시지(e.g., 상태)를 직렬화한 후에도 그 큰 버퍼를 계속 점유하게 됩니다. 따라서 다음을 추천합니다.

· 가변 크기 작업 후에는 shrinkToFit()을 호출하여 현재 사용량에 맞춰 버퍼를 축소.
· 단, shrinkToFit()은 재할당을 유발하므로 너무 빈번히 호출하면 역효과. 대신 “마지막 사용 후 일정 시간이 지났거나, 사용량이 현저히 줄었을 때” 호출하는 전략을 취할 수 있습니다.

초기 단계에서는 안전을 위해 직렬화 후 항상 _jsonBuffer.shrinkToFit()을 호출하고, 나중에 프로파일링 후 최적화합니다.

---

3. 예상 효과 및 주의사항

· 힙 단편화 감소: 메모리 청크를 재사용하므로 할당/해제 패턴이 사라져 장기간 안정성 확보.
· 응답 시간 단축: 할당 오버헤드 제거.
· OOM 리스크 감소: 특히 최대 부하 상태에서 JSON 처리 실패 방지.

주의: _lock으로 보호되는 ConfigManager 버퍼를 사용할 때, 절대 블로킹 I/O나 긴 작업 중에 락을 홀드하지 않도록 할 것. 기존 설계대로 xSemaphoreGive 후에 save()가 실제 파일 쓰기를 수행하는 것은 옳지만, _jsonBuffer는 락 해제 전까지 유효해야 함을 명심 (잘못하면 락 해제 후 버퍼가 다른 스레드에서 변경됨). 따라서 save()에서는 락을 건 상태에서 serializeJson(_jsonBuffer, v_tmp)까지 완료한 후 파일을 닫고 락을 해제하는 것이 안전합니다. 파일 쓰기는 버퍼 내용을 복사해서 쓰므로 문제 없음.

---

4. 결론 및 권고

이슈 17은 여전히 해결되지 않은 상태이며, 메모리 안정성 측면에서 심각한 위험을 내포합니다. 지금 바로 조치하지 않으면, 시스템이 현장에서 무작위로 멈추거나 JSON 직렬화 오류를 뿜으며 재부팅되는 현상이 발생할 것입니다.

즉각적인 대응:

1. CL_T2_ConfigManager에 JsonDocument _jsonBuffer; 멤버 추가.
2. 모든 save(), load(), updateFromJson(), updatePreview(), serializeToBuffer() 함수에서 지역 변수 대신 멤버 버퍼 사용.
3. CL_T2_Communicator에도 유사한 풀 버퍼 도입 (혹은 전역 정적 버퍼 + 뮤텍스).
4. 필요시 shrinkToFit() 적용.
5. 테스트 후 힙 상태 모니터링 (예: 1시간 부하 테스트에서 heap_caps_get_free_size(MALLOC_CAP_8BIT) 추이 확인).

이 권고안을 적용하면 ArduinoJson V7의 이점을 완전히 살리면서, 임베디드 환경에서 안정적인 장기 운용이 가능해집니다.



---


제공된 최신 코드를 재점검한 결과, ArduinoJson V7의 메모리 풀링(Pooling) 미적용 문제는 전혀 해결되지 않았습니다. 이전 분석에서 지적한 모든 취약점이 그대로 남아 있으며, 장기 운용 시 힙 단편화와 OOM 위험은 여전히 심각합니다.

---

1. 현재 상태 요약 – 바뀐 점 없음

CL_T2_ConfigManager 클래스

· 멤버 변수에 JsonDocument 재사용 버퍼가 추가되지 않음.
· 여전히 load(), save(), updateFromJson(), updatePreview(), serializeToBuffer() 내부에서 매 호출 시마다 지역 JsonDocument v_doc;를 새로 생성하고 소멸시킴.

CL_T2_Communicator / T270_Commu_250.cpp

· /api/status 핸들러, publishResultMqtt() 함수 등에서도 동일하게 지역 JsonDocument가 무분별하게 생성·소멸됨.
· 통신 클래스에도 공용 JsonDocument 멤버 풀이 존재하지 않음.

---

2. 실제 문제 발생 지점 (변화 없음)

파일 함수 동작 빈도 영향
T220_CfgMgr_250.cpp load() 부팅/복구 시 부팅 직후부터 단편화 촉진
 save() 설정 저장마다 저장 동작 시 급격한 단편화
 updateFromJson() Web 요청 시 빈번 외부 입력마다 할당/해제 반복
 updatePreview() 튜닝 모드에서 지속 힙 상태 악화 가속
 serializeToBuffer() 주기적 상태 전송 잦은 할당으로 조각화 심화
T270_Commu_250.cpp /api/status 콜백 HTTP 요청 시 다중 접속 시 할당 폭증
 publishResultMqtt() 감지 이벤트 발생 시 이벤트 빈도에 따라 누적

결과: 작은 크기의 JSON 버퍼가 할당·해제를 반복하며 DRAM에 수많은 빈 공간(외부 단편화)을 만들어, 결국 연속된 공간 부족으로 직렬화 실패 또는 시스템 정지를 초래합니다.

---

3. 권고 조치 사항 (이전 답변과 동일하나 더욱 시급)

3.1 CL_T2_ConfigManager에 재사용 버퍼 도입

```cpp
// T220_CfgMgr_250.hpp
class CL_T2_ConfigManager {
private:
    T2_Type::ST_DynamicConfig_t _dynConfig;
    SemaphoreHandle_t           _lock;
    // ...기타 멤버...
    JsonDocument _jsonBuffer;   // ← 추가
};
```

수정할 함수들

· load(), save(), updateFromJson(), updatePreview(), serializeToBuffer()
    → 각 함수 내 JsonDocument v_doc; 대신 _jsonBuffer를 사용.
    → 사용 전 반드시 _jsonBuffer.clear() 호출 (이미 _lock 안에 진입했으므로 안전).
    → 선택적으로 사용 후 _jsonBuffer.shrinkToFit()을 호출하여 불필요한 메모리 점유 방지.

예시 (save 함수 일부)

```cpp
bool CL_T2_ConfigManager::save() {
    xSemaphoreTake(_lock, portMAX_DELAY);
    _jsonBuffer.clear();                         // 재사용 준비
    // 이후 v_doc 대신 _jsonBuffer 사용
    JsonObject v_sys = _jsonBuffer["system"].to<JsonObject>();
    // ... 모든 직렬화 코드 그대로 ...
    serializeJson(_jsonBuffer, v_tmp);
    // _jsonBuffer.shrinkToFit();  // 필요시 호출
    xSemaphoreGive(_lock);
    return true;
}
```

3.2 CL_T2_Communicator에도 적용

· 멤버 JsonDocument _jsonCommBuf; 추가
· /api/status 핸들러에서 람다 캡처 방식으로 _jsonCommBuf 접근 (단, AsyncWebServer가 멀티스레드 환경에서 안전하도록 필요 시 별도 뮤텍스 고려)
· publishResultMqtt()에서 멤버 버퍼 사용

3.3 shrinkToFit() 정책

· save()처럼 큰 JSON을 빌드한 후에는 shrinkToFit()으로 버퍼를 줄여라.
· serializeToBuffer()처럼 작은 메시지를 빌드한 후에는 호출하지 않거나, 선택적으로 호출하여 과잉 축소를 방지할 수 있음.

---

4. 결론

현재 코드는 ArduinoJson V7의 핵심 장점(메모리 풀링)을 전혀 활용하지 못하고 있으며, 이는 ESP32-S3의 제한된 DRAM에서 장기간 운용 불가능을 의미합니다.
즉시 위의 조치를 취하여 하나의 재사용 버퍼로 모든 JSON 직렬화/역직렬화를 처리하도록 아키텍처를 변경해야 합니다. 그렇지 않으면 현장에서 불특정 시점에 JSON 처리 실패, 웹 응답 중단, MQTT 전송 실패로 이어질 확률이 매우 높습니다.
