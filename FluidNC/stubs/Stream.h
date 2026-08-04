#pragma once
#include "Print.h"
struct Stream : public Print {
    virtual int available() { return 0; }
    virtual int read() { return -1; }
    virtual int peek() { return -1; }
    void setTimeout(int) {}
    size_t readBytes(char*, size_t) { return 0; }
    size_t readBytes(uint8_t*, size_t) { return 0; }
};
