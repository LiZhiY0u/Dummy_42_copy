#pragma once
#include <cstdint>
#include <cstddef>
namespace StorageIntegrity {
constexpr uint32_t Magic = 0x314c4143; // CAL1
constexpr uint32_t Committed = 0x54494d43;
constexpr uintptr_t HeaderAddress = 0x08017800;
constexpr uintptr_t TableAddress = 0x08017c00;
constexpr uint32_t TableBytes = 32768;
struct CalibrationHeader { uint32_t magic, version, bytes, crc, commit; };
inline uint32_t Crc32(const void* data, size_t size) {
    const auto* p = static_cast<const uint8_t*>(data);
    uint32_t crc = 0xffffffffU;
    while (size--) {
        crc ^= *p++;
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ (0xedb88320U & (0U - (crc & 1U)));
    }
    return ~crc;
}
inline bool Valid(const CalibrationHeader& h, const void* table, size_t size) {
    return h.magic == Magic && h.version == 1 && h.bytes == size && h.commit == Committed
        && h.crc == Crc32(table, size);
}
}
