#include "mt6816_base.h"

bool MT6816Base::Init()
{
    SpiInit();
    angleData.rectifyValid = true;
    for (int32_t i = 0; i < RESOLUTION; ++i)
        if (quickCaliDataPtr[i] >= 51200U)
            angleData.rectifyValid = false;
    UpdateAngle();
    return angleData.rectifyValid;
}

uint16_t MT6816Base::UpdateAngle()
{
    angleData.sampleValid = false;
    for (unsigned attempt = 0; attempt < 3; ++attempt)
    {
        const uint16_t high = SpiTransmitAndRead16Bits(0x8300);
        if (high == UINT16_MAX) return UINT16_MAX;
        const uint16_t low = SpiTransmitAndRead16Bits(0x8400);
        if (low == UINT16_MAX) return UINT16_MAX;
        const uint16_t raw = ((high & 0xffU) << 8) | (low & 0xffU);
        unsigned parity = 0;
        for (unsigned bit = 0; bit < 16; ++bit)
            parity ^= (raw >> bit) & 1U;
        if (parity || (raw & 2U))
            continue;
        angleData.rawAngle = raw >> 2;
        if (angleData.rectifyValid)
            angleData.rectifiedAngle = quickCaliDataPtr[angleData.rawAngle];
        angleData.sampleValid = true;
        return angleData.rectifiedAngle;
    }
    return UINT16_MAX;
}

bool MT6816Base::IsCalibrated() { return angleData.rectifyValid; }
