#include "encoder_calibrator_stm32.h"
#include "storage_integrity.h"
#include "stm32f1xx_hal.h"

namespace {
bool ErasePage(uint32_t address) {
    FLASH_EraseInitTypeDef erase{};
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = address;
    erase.NbPages = 1;
    uint32_t error;
    return HAL_FLASHEx_Erase(&erase, &error) == HAL_OK;
}
bool Program16(uint32_t address, uint16_t value) {
    return HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, value) == HAL_OK
        && *reinterpret_cast<volatile uint16_t*>(address) == value;
}
}

void EncoderCalibrator::ClearFlash()
{
    using namespace StorageIntegrity;
    writeOk = HAL_FLASH_Unlock() == HAL_OK;
    if (writeOk) {
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR);
        // Invalidate the old commit BEFORE erasing any table page.
        writeOk = ErasePage(HeaderAddress);
        for (uint32_t address = TableAddress; writeOk && address < TableAddress + TableBytes; address += 1024)
            writeOk = ErasePage(address);
    }
    HAL_FLASH_Lock();
}

void EncoderCalibrator::BeginWriteFlash()
{
    nextAddress = StorageIntegrity::TableAddress;
    if (writeOk) writeOk = HAL_FLASH_Unlock() == HAL_OK;
}

void EncoderCalibrator::WriteFlash16bitsAppend(uint16_t value)
{
    if (!writeOk) return;
    if (nextAddress >= StorageIntegrity::TableAddress + StorageIntegrity::TableBytes || value >= 51200U) {
        writeOk = false;
        return;
    }
    writeOk = Program16(nextAddress, value);
    if (writeOk) nextAddress += 2;
}

bool EncoderCalibrator::EndWriteFlash()
{
    using namespace StorageIntegrity;
    writeOk = writeOk && nextAddress == TableAddress + TableBytes;
    if (writeOk) {
        const void* table = reinterpret_cast<const void*>(TableAddress);
        const CalibrationHeader header{Magic, 1, TableBytes, Crc32(table, TableBytes), Committed};
        const auto* bytes = reinterpret_cast<const uint8_t*>(&header);
        // The last word is the commit marker; a reset before it finishes is invalid.
        for (uint32_t offset = 0; writeOk && offset < sizeof header; offset += 2) {
            const uint16_t half = bytes[offset] | (uint16_t(bytes[offset+1]) << 8);
            writeOk = Program16(HeaderAddress + offset, half);
        }
        writeOk = writeOk && Valid(*reinterpret_cast<const CalibrationHeader*>(HeaderAddress), table, TableBytes);
    }
    HAL_FLASH_Lock();
    return writeOk;
}
