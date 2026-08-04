#pragma once
#include <cstddef>
#include <cstdint>
struct Print {
    virtual size_t write(uint8_t) { return 0; }
    virtual size_t write(const uint8_t*, size_t n) { return n; }
    void print(char) {}
    void print(int) {}
    void print(unsigned int) {}
    void print(long) {}
    void print(unsigned long) {}
    void print(float, int=2) {}
    void print(double, int=2) {}
    void print(const char*) {}
};
