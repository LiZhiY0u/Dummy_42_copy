#include <cassert>
#include <cstdio>
#include "spi.h"
#include "mt6816_base.h"
SPI_TypeDef spiRegisters;
SPI_HandleTypeDef hspi1 = { &spiRegisters };
GPIO_TypeDef gpioRegisters;
GPIO_TypeDef *GPIOA = &gpioRegisters;
static uint16_t calibration[16384];
int main()
{
    MT6816Base encoder(calibration);
    encoder.Init();
    assert((spiRegisters.CR1 & SPI_CR1_SPE) != 0);
    assert(!encoder.angleData.sampleValid);
    // Missing RXNE must terminate and leave the chip deselected.
    spiRegisters.SR = SPI_SR_TXE;
    assert(!encoder.UpdateAngle());
    assert(gpioRegisters.BSRR == GPIO_PIN_15);
    assert(encoder.GetConsecutiveErrorCount() == 1);
    std::puts("PASS: encoder initialization enables SPI; stalled RX terminates safely");
}
