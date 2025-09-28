#pragma once
#include <atomic>
#include <optional>

// N must be a power of 2 (like 128, 256, 1024) for the bit magic to work
template <typename T, size_t N>
struct SpscRing
{
    static_assert((N & (N - 1)) == 0, "N must be power of two");

    T buf[N];                             // The actual buffer
    std::atomic<size_t> head{0}, tail{0}; // Read/write positions

    // Producer calls this to add data
    bool push(const T &v)
    {
        size_t h = head.load(std::memory_order_relaxed);
        size_t t = tail.load(std::memory_order_acquire);

        // Check if buffer is full
        if (((h + 1) & (N - 1)) == (t & (N - 1)))
            return false;

        buf[h & (N - 1)] = v; // Write data (& does modulo for power of 2)
        head.store(h + 1, std::memory_order_release);
        return true;
    }

    std::optional<T> pop()
    {
        size_t t = tail.load(std::memory_order_relaxed);
        size_t h = head.load(std::memory_order_acquire);

        if (t == h)
            return std::nullopt; // Buffer empty

        T v = buf[t & (N - 1)]; // Read data
        tail.store(t + 1, std::memory_order_release);
        return v; // Return the data wrapped in optional
    }
};