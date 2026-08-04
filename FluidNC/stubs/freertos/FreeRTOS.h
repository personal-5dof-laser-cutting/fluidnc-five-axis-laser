#pragma once
#pragma once
#include <cstdint>
typedef uint32_t TickType_t;
typedef void* TaskHandle_t;
typedef void* xQueueHandle;
typedef unsigned int UBaseType_t;
inline xQueueHandle xQueueCreate(int, int) { return nullptr; }
typedef uint8_t pinnum_t;
typedef void step_engine_t;
#include <cstdarg>
