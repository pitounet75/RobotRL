#pragma once

#include <Arduino.h>
#include <cstring>

#ifndef RING_BUF_SIZE
#define RING_BUF_SIZE 4096
#endif

static_assert((RING_BUF_SIZE & (RING_BUF_SIZE - 1)) == 0, "RING_BUF_SIZE must be a power of 2");

class ByteRingBuffer {
 public:
  ByteRingBuffer() : head_(0), tail_(0), overflow_(0), high_water_(0) {}

  size_t available() const { return (head_ - tail_) & (RING_BUF_SIZE - 1); }

  size_t free_space() const { return (RING_BUF_SIZE - 1) - available(); }

  uint32_t overflow_count() const { return overflow_; }

  size_t high_water_mark() const { return high_water_; }

  size_t push(const uint8_t *data, size_t len) {
    size_t written = 0;
    portENTER_CRITICAL(&mux_);
    for (; written < len; ++written) {
      const size_t next = (head_ + 1) & (RING_BUF_SIZE - 1);
      if (next == tail_) {
        overflow_ += static_cast<uint32_t>(len - written);
        break;
      }
      buf_[head_] = data[written];
      head_ = next;
      const size_t fill = (head_ - tail_) & (RING_BUF_SIZE - 1);
      if (fill > high_water_) {
        high_water_ = fill;
      }
    }
    portEXIT_CRITICAL(&mux_);
    return written;
  }

  size_t pop(uint8_t *data, size_t len) {
    size_t read = 0;
    portENTER_CRITICAL(&mux_);
    while (read < len && tail_ != head_) {
      data[read++] = buf_[tail_];
      tail_ = (tail_ + 1) & (RING_BUF_SIZE - 1);
    }
    portEXIT_CRITICAL(&mux_);
    return read;
  }

  void discard_all() {
    portENTER_CRITICAL(&mux_);
    tail_ = head_;
    portEXIT_CRITICAL(&mux_);
  }

  /** Read byte at tail+offset without consuming. Returns false if offset >= available(). */
  bool peek(size_t offset, uint8_t *out) {
    portENTER_CRITICAL(&mux_);
    const size_t avail = (head_ - tail_) & (RING_BUF_SIZE - 1);
    if (offset >= avail || out == nullptr) {
      portEXIT_CRITICAL(&mux_);
      return false;
    }
    *out = buf_[(tail_ + offset) & (RING_BUF_SIZE - 1)];
    portEXIT_CRITICAL(&mux_);
    return true;
  }

  /** Copy bytes from tail+offset without consuming them. */
  size_t peek_bytes(size_t offset, uint8_t *out, size_t len) {
    if (out == nullptr || len == 0) {
      return 0;
    }
    portENTER_CRITICAL(&mux_);
    const size_t avail = (head_ - tail_) & (RING_BUF_SIZE - 1);
    if (offset >= avail) {
      portEXIT_CRITICAL(&mux_);
      return 0;
    }
    size_t copied = avail - offset;
    if (copied > len) {
      copied = len;
    }
    for (size_t i = 0; i < copied; ++i) {
      out[i] = buf_[(tail_ + offset + i) & (RING_BUF_SIZE - 1)];
    }
    portEXIT_CRITICAL(&mux_);
    return copied;
  }

  /** Drop up to len bytes from the front of the ring. Returns bytes discarded. */
  size_t discard_prefix(size_t len) {
    portENTER_CRITICAL(&mux_);
    const size_t avail = (head_ - tail_) & (RING_BUF_SIZE - 1);
    if (len > avail) {
      len = avail;
    }
    tail_ = (tail_ + len) & (RING_BUF_SIZE - 1);
    portEXIT_CRITICAL(&mux_);
    return len;
  }

  size_t push_from_stream(HardwareSerial &stream) {
    uint8_t chunk[64];
    size_t total = 0;
    int avail = stream.available();
    while (avail > 0) {
      const int to_read = avail > static_cast<int>(sizeof(chunk)) ? static_cast<int>(sizeof(chunk)) : avail;
      const int n = static_cast<int>(stream.read(chunk, static_cast<size_t>(to_read)));
      if (n <= 0) {
        break;
      }
      total += push(chunk, static_cast<size_t>(n));
      avail = stream.available();
    }
    return total;
  }

 private:
  uint8_t buf_[RING_BUF_SIZE];
  volatile size_t head_ = 0;
  volatile size_t tail_ = 0;
  uint32_t overflow_ = 0;
  size_t high_water_ = 0;
  portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
};
