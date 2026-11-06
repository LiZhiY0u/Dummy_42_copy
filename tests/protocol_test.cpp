#include "protocol_safety.h"
#include <cassert>
#include <cmath>
#include <cstring>
#include <cstdio>
int main() {
    using namespace ProtocolSafety;
    int32_t value = 99;
    assert(!ScaledFloat(NAN, 1000, -1000, 1000, value) && value == 99);
    assert(!ScaledFloat(INFINITY, 51200, -51200000, 51200000, value));
    assert(!ScaledFloat(-1, 1000, 1, 2000, value));
    assert(!ScaledFloat(1e30f, 51200, -51200000, 51200000, value));
    assert(ScaledFloat(1.5f, 1000, -2000, 2000, value) && value == 1500);
    assert(!CommandLength(0x03, 3));
    assert(CommandLength(0x03, 4));
    assert(!CommandLength(0x06, 7));
    assert(CommandLength(0x06, 8));
    assert(!CommandLength(0x06, 9));
    assert(!CommandLength(0x66, 8));
    assert(CommandLength(0x02, 0));
    uint8_t bytes[9]{};
    float expected = 1.25f;
    std::memcpy(bytes+1, &expected, 4);
    assert(ReadFloat(bytes+1) == expected); // deliberately unaligned
    uint8_t payload[8]{};
    std::memcpy(payload, &expected, 4);
    assert(ValidCommand(0x05, payload, 4));
    assert(!ValidCommand(0x05, payload, 3));
    expected = NAN; std::memcpy(payload, &expected, 4);
    assert(!ValidCommand(0x03, payload, 4));
    uint32_t node = 0; std::memcpy(payload, &node, 4);
    assert(!ValidCommand(0x11, payload, 4));
    node = 15; std::memcpy(payload, &node, 4);
    assert(ValidCommand(0x11, payload, 4));
    node = 16; std::memcpy(payload, &node, 4);
    assert(!ValidCommand(0x11, payload, 4));
    assert(!ValidCommand(0x02, nullptr, 0));
    std::puts("protocol: short/unknown frames, unaligned floats, finite/range guards passed");
}
