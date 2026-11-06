#pragma once
#include <cstdint>
#include <cstddef>
class UartFramer {
public:
    enum Event { None, Text, Binary };
    uint8_t binary[5]{};
    char text[64]{};
    Event Feed(uint8_t byte) {
        if (binaryUsed) {
            if (binaryUsed == 1 && byte != 0x55) {
                binaryUsed = byte == 0xaa ? 1 : 0;
                return None;
            }
            binary[binaryUsed++] = byte;
            if (binaryUsed == 5) { binaryUsed = 0; return Binary; }
            return None;
        }
        if (byte == 0xaa) { used = 0; discard = false; binary[0] = byte; binaryUsed = 1; return None; }
        if (byte == '\r' || byte == '\n') return Idle();
        if (discard) return None;
        if (byte < 0x20 || byte > 0x7e || used == sizeof(text)-1) {
            discard = true; return None;
        }
        text[used++] = static_cast<char>(byte);
        return None;
    }
    Event Idle() {
        if (binaryUsed) return None;
        if (discard) { discard = false; used = 0; return None; }
        if (!used) return None;
        text[used] = 0;
        used = 0;
        return Text;
    }
    void Reset() { used = binaryUsed = 0; discard = false; }
private:
    size_t used = 0;
    uint8_t binaryUsed = 0;
    bool discard = false;
};
