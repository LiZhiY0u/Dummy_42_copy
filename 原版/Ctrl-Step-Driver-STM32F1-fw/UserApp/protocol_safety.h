#pragma once
#include <cstdint>
#include <cmath>
#include <cstring>
namespace ProtocolSafety {
constexpr int32_t MaxCurrent = 2000; // mA; first hardware-test release
constexpr int32_t MaxVelocity = 30 * 51200;
constexpr int32_t MaxAcceleration = 100 * 51200;
constexpr int32_t MaxPosition = 1000 * 51200;
inline float ReadFloat(const uint8_t* data) { float v; std::memcpy(&v, data, 4); return v; }
inline uint32_t ReadUint(const uint8_t* data) { uint32_t v; std::memcpy(&v, data, 4); return v; }
inline int32_t ReadInt(const uint8_t* data) { int32_t v; std::memcpy(&v, data, 4); return v; }
inline bool ScaledFloat(float v, float scale, int32_t low, int32_t high, int32_t& out) {
    const double scaled = static_cast<double>(v) * scale;
    if (!std::isfinite(scaled) || scaled < low || scaled > high) return false;
    out = static_cast<int32_t>(scaled);
    return true;
}
inline bool CommandLength(uint8_t cmd, uint32_t len) {
    if (len > 8) return false;
    switch (cmd) {
        case 0x06: case 0x07: return len == 8;
        case 0x01: case 0x03: case 0x04: case 0x05:
        case 0x11: case 0x12: case 0x13: case 0x14:
        case 0x16: case 0x17: case 0x18: case 0x19: case 0x1a: case 0x1b: return len >= 4;
        case 0x02: case 0x15: case 0x21: case 0x22: case 0x23: case 0x24:
        case 0x7e: case 0x7f: return true;
        default: return false;
    }
}
inline bool ValidCommand(uint8_t cmd, const uint8_t* data, uint32_t len) {
    if (!data || !CommandLength(cmd, len)) return false;
    int32_t ignored;
    switch (cmd) {
        case 0x01: case 0x16: case 0x1b: return ReadUint(data) <= 1;
        case 0x11: return ReadUint(data) >= 1 && ReadUint(data) <= 15; // zero is broadcast
        case 0x03: return ScaledFloat(ReadFloat(data), 1000, -MaxCurrent, MaxCurrent, ignored);
        case 0x04: return ScaledFloat(ReadFloat(data), 51200, -MaxVelocity, MaxVelocity, ignored);
        case 0x05: return ScaledFloat(ReadFloat(data), 51200, -MaxPosition, MaxPosition, ignored);
        case 0x06: return ScaledFloat(ReadFloat(data), 51200, -MaxPosition, MaxPosition, ignored)
            && std::isfinite(ReadFloat(data+4)) && ReadFloat(data+4) > 0 && ReadFloat(data+4) <= 3600;
        case 0x07: return ScaledFloat(ReadFloat(data), 51200, -MaxPosition, MaxPosition, ignored)
            && ScaledFloat(ReadFloat(data+4), 51200, 1, MaxVelocity, ignored);
        case 0x12: return ScaledFloat(ReadFloat(data), 1000, 1, MaxCurrent, ignored);
        case 0x13: return ScaledFloat(ReadFloat(data), 51200, 1, MaxVelocity, ignored);
        case 0x14: return ScaledFloat(ReadFloat(data), 51200, 1, MaxAcceleration, ignored);
        case 0x17: case 0x18: case 0x19: case 0x1a: return ReadInt(data) >= 0 && ReadInt(data) <= 1000;
        default: return true;
    }
}
}
