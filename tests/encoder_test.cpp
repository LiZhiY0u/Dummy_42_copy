#include "Sensor/Encoder/mt6816_base.h"
#include <cassert>
#include <cstdio>
#include <algorithm>
void MT6816Base::SpiInit() {}
uint16_t MT6816Base::SpiTransmitAndRead16Bits(uint16_t) { return 0xffff; }
class FakeEncoder : public MT6816Base {
public:
    explicit FakeEncoder(uint16_t* table) : MT6816Base(table) {}
    uint16_t raw = 0;
    int calls = 0;
private:
    void SpiInit() override {}
    uint16_t SpiTransmitAndRead16Bits(uint16_t command) override {
        ++calls;
        return command == 0x8300 ? raw >> 8 : raw & 0xff;
    }
};
int main() {
    static uint16_t table[16384];
    for (int i=0; i<16384; ++i) table[i] = i;
    FakeEncoder e(table);
    e.Init();
    e.raw = 5; // angle=1, even parity, magnet present
    assert(e.UpdateAngle() == 1);
    e.raw = 4; // same angle, bad parity: must NOT return the previous valid sample
    assert(e.UpdateAngle() == UINT16_MAX);
    e.raw = 3; // even parity but no magnet
    assert(e.UpdateAngle() == UINT16_MAX);
    e.raw = 0xffff; // transport failure sentinel
    assert(e.UpdateAngle() == UINT16_MAX);
    e.raw = 5;
    assert(e.UpdateAngle() == 1);
    std::puts("encoder: parity, no-magnet, transport failure, recovery samples passed");
}
