# [T216_RingBuf] 스레드 안전 락프리 링 버퍼

본 문서는 **MSFM_T2_250** 임베디드 펌웨어에서 높은 우선순위를 지닌 수집 태스크(또는 ISR)와 주 연산 태스크 간의 데이터 통신을 락프리(Lock-free) 방식으로 실시간 중계하는 `CL_T2_RingBuffer` 템플릿 클래스의 기능 규격서입니다.

---

## 1. 개요 및 설계 원칙

1.  **동적 할당 배제 (Zero Dynamic Allocation)**:
    *   실시간 신호 수집 중 빈번한 동적 힙 할당으로 인한 파편화 및 성능 저하를 방지하기 위해 컴파일 타임에 버퍼의 최대 크기(`_capacity`)를 템플릿 인자로 넘겨 받아 내부 정적 배열 구조로 메모리를 고정 선언합니다.
2.  **락 프리 단일 생산자-단일 소비자 (SPSC)**:
    *   IMU 인터럽트 ISR 혹은 고속 DMA 오디오 수집 태스크(Producer)와 특징 추출 태스크(Consumer)가 임계 영역(Critical Section) 진입 차단으로 인한 우선순위 역전(Priority Inversion)을 겪지 않도록 뮤텍스(Mutex) 없이 원자적 연산으로 동기화합니다.
3.  **단일 샘플 오버라이트 차단 및 프레임 단위 드롭 정책**:
    *   링버퍼 오버런(Full) 시 단일 샘플을 임의로 덮어쓰게 되면 신호 위상 왜곡 및 스펙트럼 붕괴가 초래되어 오경보가 발생합니다.
    *   따라서 버퍼가 Full 상태일 때는 단일 샘플 오버라이트를 차단하고, 버퍼를 일괄 클리어한 후 DSP 히스토리(위상 정보)를 강제 리셋하여 프레임 단위로 안전하게 드롭 처리합니다.

---

## 2. 클래스 인터페이스 (`CL_T2_RingBuffer`)

```cpp
template <typename T>
class CL_T2_RingBuffer {
private:
    T* _buffer;
    uint16_t _capacity;
    std::atomic<uint16_t> _head;
    std::atomic<uint16_t> _tail;
    ...
```

### 2.1 핵심 API 상세 명세

#### 1. `bool enqueue(const T& item)`
*   **역할**: 링 버퍼의 `_head` 위치에 새로운 데이터를 삽입합니다.
*   **동작 흐름**:
    1.  `_head` 인덱스를 relaxed 모드로 읽어오고, `_tail` 인덱스를 acquire 메모리 순서로 읽어와 동기화합니다.
    2.  `next_head == tail` 인 포화 상태를 검증합니다.
    3.  버퍼가 가득 찬 경우, 단일 오버라이트를 막고 `_buffer[head] = item` 대입 후 메모리 펜스(`std::atomic_thread_fence(std::memory_order_release)`)를 쳐서 데이터 가시성을 전파하고 `_tail`을 한 칸 전진시킵니다.
    4.  그렇지 않은 경우 일반 대입 후 펜스를 적용하고 `_head`를 release 모드로 갱신합니다.

```cpp
template <typename T>
bool CL_T2_RingBuffer<T>::enqueue(const T& item) {
    uint16_t head = _head.load(std::memory_order_relaxed);
    uint16_t next_head = (head + 1) % _capacity;
    uint16_t tail = _tail.load(std::memory_order_acquire); // acquire를 통해 tail 변화 동기화

    if (next_head == tail) {
        // [중요] tail 전진 전 메모리 배리어를 쳐 데이터 복사 완료 보장
        _buffer[head] = item;
        std::atomic_thread_fence(std::memory_order_release);
        _tail.store((tail + 1) % _capacity, std::memory_order_relaxed);
    } else {
        _buffer[head] = item;
        std::atomic_thread_fence(std::memory_order_release);
    }
    _head.store(next_head, std::memory_order_release); // release를 통한 가시성 전파
    return true;
}
```

#### 2. `bool dequeue(T& outItem)`
*   **역할**: 링 버퍼의 `_tail` 위치에서 데이터를 꺼내어 전달하고 한 칸 전진합니다.
*   **동작 흐름**:
    1.  `_tail`을 relaxed 모드로, `_head`를 acquire 모드로 읽어옵니다.
    2.  `tail == head`인 경우(버퍼 비었음) 즉시 `false`를 리턴하고 종료합니다.
    3.  `_buffer[tail]` 데이터를 꺼내어 외부 참조 인자(`outItem`)에 복사합니다.
    4.  `_tail` 인덱스를 release 메모리 순서로 한 칸 전진시킵니다.
    5.  성공 여부(`true`)를 리턴합니다.

#### 3. `void clear()`
*   **역할**: 버퍼의 헤드와 테일 인덱스를 `0`으로 원자적 초기화하여 버퍼를 즉각 비웁니다.

#### 4. `uint16_t getCount() const`
*   **역할**: 현재 버퍼에 미처리 상태로 적재되어 있는 아이템 개수를 반환합니다.

---

## 3. 하드웨어 레벨의 동기화 보장 (Memory Ordering)

`CL_T2_RingBuffer`는 멀티코어 환경(Core 0 수집, Core 1 처리) 및 컴파일러 아웃오브오더(Out-of-order) 실행 부작용을 방지하기 위해 다음과 같은 저수준 장벽을 강제합니다.

1.  **메모리 펜스 (`std::atomic_thread_fence(std::memory_order_release)`)**:
    *   배열 데이터의 실제 물리적 쓰기 명령어들이 완료되기 전에 포인터 인덱스의 전진 신호가 다른 코어(Consumer)에 노출되지 않도록 컴파일러 레지스터 재배치 장벽 역할을 수행합니다.
2.  **Acquire-Release 시맨틱스**:
    *   `load(std::memory_order_acquire)`와 `store(std::memory_order_release)`를 조합하여 코어 간 캐시 라인의 일관성 및 메모리 오더링의 물리적 동기화를 보장합니다.

---

## 4. 변경 및 갱신 이력 (Revision History)

*   **v2.50 (2026-06-21)**:
    *   클래스명을 `CL_T2_RingBuffer`로 현행화.
    *   락 프리 링버퍼 메모리 오더링 기법(`std::atomic_thread_fence` 및 Acquire/Release) 상세 시그니처 반영.
    *   단일 샘플 오버라이트 차단 및 프레임 드롭(DSP 리셋 결합) 정책 추가.
