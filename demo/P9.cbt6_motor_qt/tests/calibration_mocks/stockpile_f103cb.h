#pragma once
#include <cstdint>
#define Stockpile_Page_Size 1024U
extern uint16_t storageMetadata[512];
extern uint16_t storageFlashKib;
#define FLASHSIZE_BASE reinterpret_cast<uintptr_t>(&storageFlashKib)
extern bool metadataFlashActive;
#define P9_CALIBRATION_RECORD_ADDRESS reinterpret_cast<uintptr_t>(storageMetadata)
struct Stockpile_FLASH_Typedef { uintptr_t begin_add=0, area_size=0, asce_write_add=0; unsigned page_num=0; };
extern Stockpile_FLASH_Typedef stockpile_quick_cali;
extern Stockpile_FLASH_Typedef *activeFlashRegion;
extern unsigned flashCalls;
extern void (*flashHook)(unsigned);
inline void Stockpile_Flash_Data_Begin(Stockpile_FLASH_Typedef *p) { metadataFlashActive=p->begin_add==reinterpret_cast<uintptr_t>(storageMetadata);++flashCalls;p->asce_write_add=p->begin_add;if(flashHook)flashHook(1); }
inline void Stockpile_Flash_Data_End(Stockpile_FLASH_Typedef *) { ++flashCalls;if(flashHook)flashHook(2); }
inline void Stockpile_Flash_Data_Set_Write_Add(Stockpile_FLASH_Typedef *p, uintptr_t address) {
    if(address>=p->begin_add && address<=p->begin_add+p->area_size)p->asce_write_add=address;
}
inline void Stockpile_Flash_Data_Empty(Stockpile_FLASH_Typedef *p) {
    metadataFlashActive=p->begin_add==reinterpret_cast<uintptr_t>(storageMetadata);
    ++flashCalls;for(uintptr_t a=p->begin_add;a<p->begin_add+p->area_size;a+=2)*reinterpret_cast<uint16_t*>(a)=0xffff;
    if(flashHook)flashHook(0);
}
inline void Stockpile_Flash_Data_Write_Data16(Stockpile_FLASH_Typedef *p, uint16_t *data, unsigned n) {
    activeFlashRegion=p;
    metadataFlashActive=p->begin_add==reinterpret_cast<uintptr_t>(storageMetadata);
    ++flashCalls;if(flashHook)flashHook(3);
    for(unsigned i=0;i<n && p->asce_write_add>=p->begin_add && p->asce_write_add+2<=p->begin_add+p->area_size;++i) {
        *reinterpret_cast<uint16_t*>(p->asce_write_add)=data[i];p->asce_write_add+=2;
    }
    if(flashHook)flashHook(4);
}
inline uint32_t __get_PRIMASK() { return 0; }
inline void __disable_irq() {}
inline void __set_PRIMASK(uint32_t) {}
