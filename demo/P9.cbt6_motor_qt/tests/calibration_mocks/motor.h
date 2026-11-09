#pragma once
#include <cstdint>
struct Motor {
    struct { struct { int32_t caliCurrent = 1000; } motionParams; } config;
};
extern Motor motor;
