#pragma once
#include <string>
#include <cstdint>
struct IPAddress { uint32_t addr = 0; };
inline std::string IP_string(IPAddress) { return "0.0.0.0"; }
