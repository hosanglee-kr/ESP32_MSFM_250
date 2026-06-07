# [T216_RingBuf] 스레드 안전 락프리 링 버퍼

본 문서는 **MSFM-T240_v250** 임베디드 펌웨어에서 높은 우선순위를 지닌 수집 태스크(또는 ISR)와 주 연산 태스크 간의 데이터 통신을 락프리(Lock-free) 방식으로 실시간 중계하는 `StaticSafeRingBuffer` 템플릿 클래스의 기능 규격서입니다.

---

## 1. 개요 및 설계 원칙

1.  **동적 할당 배제 (Zero Dynamic Allocation)**:
    *   실시간 신호 수집 중 빈번한 동적 힙 할당으로 인한 파편화 및 성능 저하를 방지하기 위해 컴파일 타임에 버퍼의 최대 크기(`MaxSize`)를 템플릿 인자로 넘겨 받아 내부 정적 배열 구조로 메모리를 고정 선언합니다.
2.  **락 프리 단일 생산자-단일 소비자 (SPSC)**:
    *   IMU 인터럽트 ISR 혹은 고속 DMA 오디오 수집 태스크(Producer)와 특징 추출 태스크(Consumer)가 임계 영역(Critical Section) 진입 차단으로 인한 우선순위 역전(Priority Inversion)을 겪지 않도록 뮤텍스(Mutex) 없이 원자적 연산으로 동기화합니다.
3.  **순환 데이터 덮어쓰기 허용 (Ring Overwrite)**:
    *   실시간 모니터링 시스템의 특성에 맞게, 데이터 큐가 가득 찼을 때 쓰기 연산을 대기시키거나 실패하게 만드는 대신, 가장 오래된 데이터를 원자적으로 버리고(Overwriting) 최신 데이터 적재를 유지하는 정책을 사용합니다.

---

## 2. 클래스 인터페이스 (`StaticSafeRingBuffer`)

```cpp
template <typename T, size_t MaxSize>
class StaticSafeRingBuffer {
private:
    T _buffer[MaxSize];
    std::atomic<size_t> _head;
    std::atomic<size_t> _tail;
    ...
```

### 2.1 핵심 API 상세 명세

#### 1. `bool enqueue(const T& item)`
*   **역할**: 링 버퍼의 `_head` 위치에 새로운 데이터를 삽입합니다.
*   **동작 흐름**:
    1.  `_head` 인덱스를 relaxed 모드로 읽어오고, `_tail` 인덱스를 acquire 메모리 순서로 읽어옵니다.
    2.  `(head + 1) % MaxSize` 가 `tail`과 같은 경우(버퍼 가득 참)를 감지합니다.
    3.  포화 상태일 때 `_tail`을 원자적으로 1 전진시켜 가장 오래된 데이터를 덮어쓸 수 있는 안전 공간을 확보합니다.
    4.  버퍼 배열에 데이터를 캐싱 및 쓰기를 완료합니다.
    5.  메모리 펜스(`std::atomic_thread_fence`) 및 어셈블리 메모리 배리어(`asm volatile("memw")`)를 강제하여 컴파일러 및 CPU 파이프라인에서 실제 배열 쓰기가 완료되기 전 `_head`가 전진하는 현상을 방지합니다.
    6.  `_head` 인덱스를 release 메모리 순서로 업데이트합니다.

#### 2. `bool dequeue(T& outItem)`
*   **역할**: 링 버퍼의 `_tail` 위치에서 데이터를 꺼내어 전달하고 한 칸 전진합니다.
*   **동작 흐름**:
    1.  `_tail`을 relaxed 모드로, `_head`를 acquire 모드로 읽어옵니다.
    2.  `tail == head`인 경우(버퍼 비었음) 즉시 `false`를 리턴하고 종료합니다.
    3.  `_buffer[tail]` 데이터를 꺼내어 외부 참조 인자(`outItem`)에 복사합니다.
    4.  `_tail` 인덱스를 release 메모리 순서로 한 칸 전진(`(tail + 1) % MaxSize`)시킵니다.
    5.  성공 여부(`true`)를 리턴합니다.

#### 3. `void clear()`
*   **역할**: 버퍼의 헤드와 테일 인덱스를 `0`으로 원자적 초기화하여 버퍼를 즉각 비웁니다.

#### 4. `size_t getCount() const`
*   **역할**: 현재 버퍼에 미처리 상태로 적재되어 있는 아이템 개수를 반환합니다.
*   **계산식**:
    *   `head >= tail` 인 경우: `head - tail`
    *   `head < tail` 인 경우: `MaxSize - (tail - head)`

---

## 3. 하드웨어 레벨의 동기화 보장 (Memory Barrier)

StaticSafeRingBuffer는 멀티코어 환경(Core 0 수집, Core 1 처리) 및 컴파일러 아웃오브오더(Out-of-order) 실행 부작용을 방지하기 위해 다음과 같은 저수준 장벽을 강제합니다.

1.  **메모리 펜스 (`std::atomic_thread_fence(std::memory_order_release)`)**:
    *   배열 데이터의 실제 물리적 쓰기 명령어들이 완료되기 전에 `_head` 인덱스의 전진 신호가 다른 코어(Consumer)에 노출되지 않도록 컴파일러 레지스터 재배치 장벽 역할을 수행합니다.
2.  **어셈블리 배리어 (`asm volatile("memw")`)**:
    *   Xtensa 아키텍처 레벨에서 데이터 버스 쓰기 버퍼가 플러시(Flush)될 때까지 대기시키는 하드웨어 메모리 배리어(`memw`) 명령을 직접 주입하여 메모리 쓰기 순서 무결성을 100% 하드웨어 수준에서 강제합니다.
