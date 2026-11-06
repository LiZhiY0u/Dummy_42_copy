#include "mt6816_stm32.h"
#include "spi.h"
#include "storage_integrity.h"

void MT6816::SpiInit() { MX_SPI1_Init(); }

uint16_t MT6816::SpiTransmitAndRead16Bits(uint16_t value)
{
    // Bounded polling: HAL tick does not advance inside the 20 kHz IRQ.
    // At 72 MHz / SPI prescaler 8, a word takes approximately 1.8 us.
    constexpr uint32_t pollLimit = 256;
    GPIOA->BRR = GPIO_PIN_15;
    uint32_t left = pollLimit;
    while (!(hspi1.Instance->SR & SPI_SR_TXE) && --left) {}
    if (!left) { GPIOA->BSRR = GPIO_PIN_15; return UINT16_MAX; }
    *(__IO uint16_t*)&hspi1.Instance->DR = value;
    left = pollLimit;
    while (!(hspi1.Instance->SR & SPI_SR_RXNE) && --left) {}
    if (!left) { GPIOA->BSRR = GPIO_PIN_15; return UINT16_MAX; }
    const uint16_t result = *(__IO uint16_t*)&hspi1.Instance->DR;
    left = pollLimit;
    while ((hspi1.Instance->SR & SPI_SR_BSY) && --left) {}
    GPIOA->BSRR = GPIO_PIN_15;
    return left ? result : UINT16_MAX;
}

bool MT6816::Init()
{
    MT6816Base::Init();
    using namespace StorageIntegrity;
    angleData.rectifyValid = angleData.rectifyValid &&
        Valid(*reinterpret_cast<const CalibrationHeader*>(HeaderAddress),
              reinterpret_cast<const void*>(TableAddress), TableBytes);
    return angleData.rectifyValid;
}
