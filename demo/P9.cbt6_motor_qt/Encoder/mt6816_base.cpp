#include "mt6816_base.h"
#include "spi.h"


// void MT6816Base::SpiInit()
// {
//     MX_SPI1_Init();
// }

// uint16_t MT6816Base::SpiTransmitAndRead16Bits(uint16_t _dataTx)
// {
//     uint16_t dataRx;

//     GPIOA->BRR = GPIO_PIN_15; // Chip select
//     HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&_dataTx, (uint8_t *)&dataRx, 1, HAL_MAX_DELAY);
//     GPIOA->BSRR = GPIO_PIN_15;

//     return dataRx;
// }

extern "C" void MT6816Base::SpiInit()
{
    MX_SPI1_Init();
}

extern "C" bool MT6816Base::SpiTransmitAndRead16Bits(uint16_t _dataTx, uint16_t *_dataRx)
{
    // The control loop runs at 20 kHz. Never use HAL_MAX_DELAY here: a broken
    // encoder or SPI bus must not lock the motor-control interrupt forever.
    static const uint32_t SPI_POLL_LIMIT = 256;
    uint32_t timeout = SPI_POLL_LIMIT;

    GPIOA->BRR = GPIO_PIN_15; // Chip select

    while (((hspi1.Instance->SR & SPI_SR_TXE) == 0U) && (--timeout != 0U))
    {
    }
    if (timeout == 0U)
    {
        GPIOA->BSRR = GPIO_PIN_15;
        return false;
    }

    *(__IO uint16_t *)&hspi1.Instance->DR = _dataTx;

    timeout = SPI_POLL_LIMIT;
    while (((hspi1.Instance->SR & SPI_SR_RXNE) == 0U) && (--timeout != 0U))
    {
    }
    if (timeout == 0U)
    {
        GPIOA->BSRR = GPIO_PIN_15;
        return false;
    }

    *_dataRx = *(__IO uint16_t *)&hspi1.Instance->DR;

    timeout = SPI_POLL_LIMIT;
    while (((hspi1.Instance->SR & SPI_SR_BSY) != 0U) && (--timeout != 0U))
    {
    }
    GPIOA->BSRR = GPIO_PIN_15;

    return timeout != 0U;
}

extern "C" void MT6816Base::Init()
{
    // SpiInit();
    // UpdateAngle();

    // Check if the stored calibration data are valid
    angleData.rectifyValid = true;
    angleData.sampleValid = false;
    for (uint32_t i = 0; i < RESOLUTION; i++)
    {
        if (quickCaliDataPtr[i] == 0xFFFF)
            angleData.rectifyValid = false;
    }
}
void MT6816Base::test()
{
    if (_g >= 100)
    {
        _g = 1;
    }
    else
    {
        _g++;
    }
}

uint8_t MT6816Base::test1()
{
    static uint8_t _a = 0;
    if (_a >= 100)
    {
        _a = 1;
    }
    else
    {
        _a++;
    }

    return _a;
}

// 获取校准数据
extern "C" bool MT6816Base::UpdateAngle()
{

    dataTx[0] = (0x80 | 0x03) << 8; // 0x8300
    dataTx[1] = (0x80 | 0x04) << 8; // 0x8400
    spiRawData.checksumFlag = false;
    spiRawData.noMagFlag = true;

    for (uint8_t i = 0; i < 3; i++)
    {
        if (!SpiTransmitAndRead16Bits(dataTx[0], &dataRx[0]) ||
            !SpiTransmitAndRead16Bits(dataTx[1], &dataRx[1]))
        {
            continue;
        }

        spiRawData.rawData = ((dataRx[0] & 0x00FF) << 8) | (dataRx[1] & 0x00FF);

        // 奇偶校验
        hCount = 0;
        for (uint8_t j = 0; j < 16; j++)
        {
            if (spiRawData.rawData & (0x0001 << j))
                hCount++;
        }
        if (hCount & 0x01)
        {
            spiRawData.checksumFlag = false;
        }
        else
        {
            spiRawData.checksumFlag = true;
            spiRawData.rawAngle = spiRawData.rawData >> 2;
            spiRawData.noMagFlag = (bool)(spiRawData.rawData & (0x0001 << 1));
            if (!spiRawData.noMagFlag)
                break;
        }
    }

    if (!spiRawData.checksumFlag || spiRawData.noMagFlag)
    {
        angleData.sampleValid = false;
        if (consecutiveErrorCount != UINT16_MAX)
            consecutiveErrorCount++;
        return false;
    }

    angleData.rawAngle = spiRawData.rawAngle;
    if (angleData.rectifyValid)
        angleData.rectifiedAngle = quickCaliDataPtr[angleData.rawAngle];
    angleData.sampleValid = true;
    consecutiveErrorCount = 0;

    return true;
}

bool MT6816Base::IsCalibrated()
{
    return angleData.rectifyValid;
}

uint16_t MT6816Base::GetConsecutiveErrorCount() const
{
    return consecutiveErrorCount;
}
