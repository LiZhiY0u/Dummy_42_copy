#include <cassert>
#include <cstdio>
#include "spi.h"
#include "mt6816_base.h"
#include "calibration_record.h"
SPI_TypeDef spiRegisters;
SPI_HandleTypeDef hspi1 = { &spiRegisters };
GPIO_TypeDef gpioRegisters;
GPIO_TypeDef *GPIOA = &gpioRegisters;
static uint16_t calibration[16384];
uint32_t encoderMetadata[5];
uint16_t encoderFlashKib=128;
static void commitRecord(){
    using namespace stepper::calibration;
    encoderMetadata[0]=Magic;encoderMetadata[1]=Version;encoderMetadata[2]=Bytes;
    encoderMetadata[3]=tableCrc(calibration);encoderMetadata[4]=Commit;
}
int main()
{
    MT6816Base encoder(calibration);
    encoder.Init();
    assert(!encoder.IsCalibrated()); // old numeric-only table is not committed
    commitRecord();
    encoder.Init();
    assert((spiRegisters.CR1 & SPI_CR1_SPE) != 0);
    assert(!encoder.angleData.sampleValid);
    calibration[16383] = 51199;
    commitRecord();
    encoder.Init();
    assert(encoder.IsCalibrated());
    encoderFlashKib=64;encoder.Init();assert(!encoder.IsCalibrated());
    encoderFlashKib=128;encoder.Init();assert(encoder.IsCalibrated());
    calibration[16383] = 51200;
    commitRecord();
    encoder.Init();
    assert(!encoder.IsCalibrated());
    calibration[16383] = 65535;
    commitRecord();
    encoder.Init();
    assert(!encoder.IsCalibrated());
    calibration[16383] = 0;
    commitRecord();
    encoder.Init();
    assert(encoder.IsCalibrated());
    // Missing RXNE must terminate and leave the chip deselected.
    spiRegisters.SR = SPI_SR_TXE;
    assert(!encoder.UpdateAngle());
    assert(gpioRegisters.BSRR == GPIO_PIN_15);
    assert(encoder.GetConsecutiveErrorCount() == 1);
    spiRegisters.SR = 0; // TXE timeout
    assert(!encoder.UpdateAngle());
    assert(gpioRegisters.BSRR == GPIO_PIN_15);
    spiRegisters.SR = SPI_SR_TXE | SPI_SR_RXNE | SPI_SR_BSY;
    assert(!encoder.UpdateAngle()); // BSY timeout
    assert(!encoder.angleData.sampleValid);
    assert(gpioRegisters.BSRR == GPIO_PIN_15);
    spiRegisters.SR = SPI_SR_TXE | SPI_SR_RXNE;
    // The register double echoes the two command writes: raw=0, valid parity.
    assert(encoder.UpdateAngle());
    assert(encoder.angleData.sampleValid);
    assert(encoder.GetConsecutiveErrorCount() == 0);
    spiRegisters.SR = 0;
    for (unsigned i = 0; i < 65537; ++i) assert(!encoder.UpdateAngle());
    assert(encoder.GetConsecutiveErrorCount() == UINT16_MAX);
    assert(!encoder.angleData.sampleValid);
    spiRegisters.SR = SPI_SR_TXE | SPI_SR_RXNE;
    assert(encoder.UpdateAngle());
    assert(encoder.GetConsecutiveErrorCount() == 0);
    calibration[1] = 123;
    spiRegisters.SR.raw(5,5,5); // angle 1, even parity, magnet present
    assert(encoder.UpdateAngle());
    assert(encoder.angleData.rawAngle == 1 && encoder.angleData.rectifiedAngle == 123);
    assert(spiRegisters.SR.cursor == 2);
    spiRegisters.SR.raw(4,4,4); // odd parity; do not reuse valid sample
    assert(!encoder.UpdateAngle());
    assert(!encoder.angleData.sampleValid && spiRegisters.SR.cursor == 6);
    assert(encoder.angleData.rectifiedAngle == 123); // retained value is marked invalid
    spiRegisters.SR.raw(3,3,3); // even parity but magnet absent
    assert(!encoder.UpdateAngle());
    assert(!encoder.angleData.sampleValid && spiRegisters.SR.cursor == 6);
    spiRegisters.SR.raw(4,3,5); // recover on final bounded attempt
    assert(encoder.UpdateAngle());
    assert(encoder.angleData.sampleValid && spiRegisters.SR.cursor == 6);
    assert(encoder.GetConsecutiveErrorCount() == 0);
    spiRegisters.SR.raw(5,5,5);
    spiRegisters.SR = SPI_SR_TXE; // transport fails before consuming a wire sample
    assert(!encoder.UpdateAngle());
    assert(!encoder.angleData.sampleValid && spiRegisters.SR.cursor == 0);
    spiRegisters.SR = SPI_SR_TXE | SPI_SR_RXNE;
    assert(encoder.UpdateAngle());
    assert(encoder.angleData.sampleValid && encoder.GetConsecutiveErrorCount() == 0);
    std::puts("PASS: calibration, bounded timeouts, parity/no-magnet sequences, recovery, saturation");
}
