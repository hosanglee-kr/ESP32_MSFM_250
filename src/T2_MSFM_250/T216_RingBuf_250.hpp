#pragma once

#include <atomic>
#include <algorithm>

template <typename T, size_t MaxSize>
class StaticSafeRingBuffer {
private:
    T _buffer[MaxSize];
    std::atomic<size_t> _head;
    std::atomic<size_t> _tail;

public:
    StaticSafeRingBuffer() : _head(0), _tail(0) {
        std::fill_n(_buffer, MaxSize, T());
    }

    // 단일 생산자(Producer: ISR 또는 수집 태스크) 호출용 (락 프리)
    bool enqueue(const T& item) {
        size_t head = _head.load(std::memory_order_relaxed);
        size_t tail = _tail.load(std::memory_order_acquire);

        size_t next_head = (head + 1) % MaxSize;
        if (next_head == tail) {
            // [중요] tail 전진 전 메모리 배리어를 쳐 데이터 복사 완료 보장
            _buffer[head] = item;
            std::atomic_thread_fence(std::memory_order_release);
            _tail.store((tail + 1) % MaxSize, std::memory_order_relaxed);
        } else {
            _buffer[head] = item;
            std::atomic_thread_fence(std::memory_order_release);
        }

        asm volatile("memw");
        _head.store(next_head, std::memory_order_release);
        return true;
    }


    // 단일 소비자(Consumer: 처리 태스크) 호출용 (락 프리)
    bool dequeue(T& outItem) {
        size_t tail = _tail.load(std::memory_order_relaxed);
        size_t head = _head.load(std::memory_order_acquire);

        if (tail == head) return false; // 버퍼 빔

        outItem = _buffer[tail];
        _tail.store((tail + 1) % MaxSize, std::memory_order_release);
        return true;
    }

    void clear() {
        _head.store(0, std::memory_order_relaxed);
        _tail.store(0, std::memory_order_release);
    }

    size_t getCount() const {
        size_t head = _head.load(std::memory_order_relaxed);
        size_t tail = _tail.load(std::memory_order_relaxed);
        if (head >= tail) return head - tail;
        return MaxSize - (tail - head);
    }
};
