---
name: Claude Guidelines
description: Behavioral guidelines adapted for ESP32_MSFM embedded development to reduce common LLM coding mistakes.
---

# CLAUDE.md - ESP32_MSFM Adaptation

Behavioral guidelines to reduce common LLM coding mistakes, tailored specifically for the **ESP32_MSFM (MSFM-T240)** embedded firmware project.

---

## 1. Think Before Coding (코딩 전 분석 및 검토)

**Don't assume. Don't hide confusion. Surface tradeoffs.**

*   **상수 및 정의 검토**: 코드를 작성하거나 수정하기 전에 반드시 `T210_Def_[Version].hpp` 및 `T215_Type_[Version].hpp`를 검토하여 의존성 순환이나 상수 중복이 발생하지 않는지 확인한다.
*   **하드웨어 제약 의식**: ESP32-S3의 메모리 구조(SRAM, PSRAM) 및 핀 매핑 정보를 의식한다. 불확실한 핀 설정이나 통신 프로토콜(SPI/I2S) 사양이 있다면 작업을 중단하고 질문한다.
*   **설계 대안 제시**: 임베디드 환경에서는 동일한 기능도 큐(Queue), 세마포어(Semaphore), FSM 상태 전이, 전역 플래그 등 다양한 방식으로 구현할 수 있으므로 최적의 대안을 제안한다.

---

## 2. Simplicity First (단순성 최우선)

**Minimum code that solves the problem. Nothing speculative.**

*   **동적 할당 배제**: FreeRTOS 루프 내에서 `malloc`, `new`, `std::vector` 동적 확장 등을 절대로 사용하지 않고 정적/초기 할당을 고수한다.
*   **불필요한 추상화 금지**: 단일 목적으로 사용되는 가상 함수나 과도한 템플릿 사용을 제한하여 바이너리 크기(Flash)와 메모리(RAM) 오염을 방지한다.
*   **최소 기능 구현**: 요청받은 기능 이외의 speculative한 기능이나 예외 처리를 추가하지 않는다.

---

## 3. Surgical Changes (수술적 변경 및 격리)

**Touch only what you must. Clean up only your own mess.**

*   **4-Tier 격리 준수**: `Global`, `Shared`, `Vib`, `Audio` 도메인 간의 코드 경계를 침범하지 않는다. 각 모듈은 자신의 도메인 역할에만 집중해야 한다.
*   **활성 버전 한정 수정**: 코드 수정은 반드시 `build_src_filter`에 활성화된 `src/T2_MSFM_240/[Version]/` 디렉토리 내부로 제한하고 타 버전 코드는 절대 건드리지 않는다.
*   **스타일 유지**: 기존 C++17 표준 코드 스타일(네이밍 룰, 브레이스 스타일, 주석 템플릿)을 그대로 유지한다.

---

## 4. Goal-Driven Execution (목표 지향 검증 루프)

**Define success criteria. Loop until verified.**

*   **빌드 검증**: 모든 코드 변경 후에는 반드시 `pio run -e esp32-s3-n16r8` 명령어를 통해 컴파일 성공 여부를 검증한다.
*   **메모리/사이즈 모니터링**: 빌드 후 출력되는 RAM(SRAM) 및 Flash 사용량을 확인하여 비정상적인 증가가 없는지 모니터링한다.
*   **테스트 시나리오 정의**: 코드 수정 전에 아래와 같은 성공 기준을 수립한다.
    ```
    1. 코드 수정 및 컴파일 검증 → verify: pio run 성공 확인
    2. 데이터 구조 및 메모리 정렬 확인 → verify: alignas(16) 등 메모리 얼라인먼트 확인
    3. FSM 상태 및 태스크 주기 유효성 확인 → verify: vTaskDelay 등 블로킹 지연 함수 적절성 확인
    ```
