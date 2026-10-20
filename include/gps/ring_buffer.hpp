#pragma once
#include <array>
#include <cstddef>

template <size_t Size>
class RingBuffer {
public:
    // Adds a character unless the buffer is full.
    bool push(char c) {
        size_t next = (head_ + 1) % Size;
        if (next == tail_) return false;
        buffer_[head_] = c;
        head_ = next;
        return true;
    }

    // Removes the oldest character unless the buffer is empty.
    bool pop(char& c) {
        if (head_ == tail_) return false;
        c = buffer_[tail_];
        tail_ = (tail_ + 1) % Size;
        return true;
    }

    // Reports whether the buffer contains no characters.
    bool empty() const { return head_ == tail_; }

private:
    std::array<char, Size> buffer_;
    volatile size_t head_ = 0;
    volatile size_t tail_ = 0;
};