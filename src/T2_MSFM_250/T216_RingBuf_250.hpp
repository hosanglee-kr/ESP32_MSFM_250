#pragma once

#include <atomic>
#include <algorithm>
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"

// 1. 단일 생산자 - 단일 소비자(SPSC) 락프리 링버퍼 (동시성 펜스 정밀 보완 및 덮어쓰기 원천 차단)
template <typename T, size_t MaxSize>
class StaticSafeRingBuffer {
private:
    T                   _buffer[MaxSize];
    std::atomic<size_t> _head;
    std::atomic<size_t> _tail;

public:
    StaticSafeRingBuffer() : _head(0), _tail(0) {
        std::fill_n(_buffer, MaxSize, T());
    }

    // 단일 생산자(Producer: ISR 또는 수집 태스크) 호출용 (락 프리)
    bool enqueue(const T& p_item) {
        size_t v_head = _head.load(std::memory_order_relaxed);
        size_t v_tail = _tail.load(std::memory_order_acquire); // 소비자 tail 로딩

        // 원천 차단 로직
        size_t v_next_head = (v_head + 1) % MaxSize;
        if (v_next_head == v_tail) {
            return false;           // 버퍼 꽉 참: 덮어쓰지 않고 실패 처리 (원천 차단)
        }

        
        _buffer[v_head] = p_item;
        std::atomic_thread_fence(std::memory_order_release); // 데이터 복사 완료 장벽
        _head.store(v_next_head, std::memory_order_release);
        return true;
    }

    // 단일 소비자(Consumer: 처리 태스크) 호출용 (락 프리)
    bool dequeue(T& p_outItem) {
        size_t v_tail = _tail.load(std::memory_order_relaxed);
        size_t v_head = _head.load(std::memory_order_acquire); // 생산자 head 로딩

        if (v_tail == v_head) return false; // 버퍼 빔

        p_outItem = _buffer[v_tail];
        std::atomic_thread_fence(std::memory_order_release); // 데이터 읽기 완료 장벽
        _tail.store((v_tail + 1) % MaxSize, std::memory_order_release);
        return true;
    }

    void clear() {
        _head.store(0, std::memory_order_relaxed);
        _tail.store(0, std::memory_order_release);
    }

    size_t getCount() const {
        size_t v_head = _head.load(std::memory_order_relaxed);
        size_t v_tail = _tail.load(std::memory_order_relaxed);
        if (v_head >= v_tail) return v_head - v_tail;
        return MaxSize - (v_tail - v_head);
    }
};

// 2. ESP-IDF 표준 Ringbuf API 연동 래퍼 (NOSPLIT 모드 동작 및 덮어쓰기 원천 차단)
template <typename T>
class EspRingBufferWrapper {
private:
    RingbufHandle_t _handle;
    size_t _bufferSize;

public:
    EspRingBufferWrapper(size_t p_bufferSizeBytes) : _handle(nullptr), _bufferSize(p_bufferSizeBytes) {
        // NOSPLIT 모드로 쪼개짐 없이 통째로 전송 보장
        _handle = xRingbufferCreate(_bufferSize, RINGBUF_TYPE_NOSPLIT);
    }

    ~EspRingBufferWrapper() {
        if (_handle) {
            vRingbufferDelete(_handle);
        }
    }

    bool enqueue(const T& p_item, TickType_t p_waitTicks = 0) {
        if (!_handle) return false;
        BaseType_t v_res = xRingbufferSend(_handle, (const void*)&p_item, sizeof(T), p_waitTicks);
        return (v_res == pdTRUE);
    }

    bool dequeue(T& p_outItem, TickType_t p_waitTicks = 0) {
        if (!_handle) return false;
        size_t v_itemSize = 0;
        void* v_itemPtr = xRingbufferReceive(_handle, &v_itemSize, p_waitTicks);
        if (v_itemPtr) {
            if (v_itemSize == sizeof(T)) {
                p_outItem = *static_cast<T*>(v_itemPtr);
                vRingbufferReturnItem(_handle, v_itemPtr);
                return true;
            }
            vRingbufferReturnItem(_handle, v_itemPtr);
        }
        return false;
    }

    void clear() {
        if (_handle) {
            vRingbufferDelete(_handle);
            _handle = xRingbufferCreate(_bufferSize, RINGBUF_TYPE_NOSPLIT);
        }
    }

    size_t getCount() const {
        if (!_handle) return 0;
        UBaseType_t v_freeSize = 0;
        vRingbufferGetInfo(_handle, nullptr, nullptr, nullptr, nullptr, &v_freeSize);
        size_t v_usedSize = _bufferSize - v_freeSize;
        return v_usedSize / sizeof(T);
    }
};
