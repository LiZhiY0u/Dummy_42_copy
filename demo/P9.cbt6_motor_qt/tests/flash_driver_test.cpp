#include "stockpile_f103cb.h"
#include <cstdio>
#include <cstdlib>
#define CHECK(c) do{if(!(c)){std::fprintf(stderr,"FAIL %d: %s\n",__LINE__,#c);std::exit(1);}}while(0)
FLASH_TypeDef registers={};
FLASH_TypeDef *FLASH=&registers;
static unsigned unlocks,locks,erases,waits,programs;
static HAL_StatusTypeDef unlockResult=HAL_OK,eraseResult=HAL_OK,programResult=HAL_OK;
HAL_StatusTypeDef HAL_FLASH_Unlock(){++unlocks;return unlockResult;}
HAL_StatusTypeDef HAL_FLASH_Lock(){++locks;return HAL_OK;}
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *p,uint32_t *page){
    ++erases;CHECK(p->NbPages==1);*page=eraseResult==HAL_OK?0xffffffffU:p->PageAddress;return eraseResult;
}
HAL_StatusTypeDef FLASH_WaitForLastOperation(uint32_t){++waits;return HAL_OK;}
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t,uint32_t,uint64_t){++programs;return programResult;}
static void reset(){unlocks=locks=erases=waits=programs=0;unlockResult=eraseResult=programResult=HAL_OK;}
int main(){
    Stockpile_FLASH_Typedef area={0x08017c00U,3072,3,0};
    reset();eraseResult=HAL_TIMEOUT;Stockpile_Flash_Data_Empty(&area);
    CHECK(erases==1 && locks==1 && waits==0);
    reset();unlockResult=HAL_ERROR;Stockpile_Flash_Data_Empty(&area);
    CHECK(erases==0 && waits==0);
    reset();Stockpile_Flash_Data_Empty(&area);CHECK(erases==3 && locks==1 && waits==0);
    uint16_t data16[3]={1,2,3};uint32_t data32[3]={1,2,3};uint64_t data64[3]={1,2,3};
    reset();programResult=HAL_ERROR;area.asce_write_add=area.begin_add;
    Stockpile_Flash_Data_Write_Data16(&area,data16,3);CHECK(programs==1 && area.asce_write_add==area.begin_add);
    reset();programResult=HAL_ERROR;area.asce_write_add=area.begin_add;
    Stockpile_Flash_Data_Write_Data32(&area,data32,3);CHECK(programs==1 && area.asce_write_add==area.begin_add);
    reset();programResult=HAL_ERROR;area.asce_write_add=area.begin_add;
    Stockpile_Flash_Data_Write_Data64(&area,data64,3);CHECK(programs==1 && area.asce_write_add==area.begin_add);
    reset();area.asce_write_add=area.begin_add;
    Stockpile_Flash_Data_Write_Data16(&area,data16,3);CHECK(programs==3 && area.asce_write_add==area.begin_add+6);
    reset();area.asce_write_add=area.begin_add+area.area_size;
    Stockpile_Flash_Data_Write_Data16(&area,data16,1);CHECK(programs==0);
    reset();area.asce_write_add=area.begin_add;
    Stockpile_Flash_Data_Write_Data16(&area,data16,0x80000000U);CHECK(programs==0);
    reset();Stockpile_Flash_Data_Empty(nullptr);CHECK(unlocks==0);
    reset();Stockpile_Flash_Data_Write_Data16(&area,nullptr,1);CHECK(programs==0);
    std::puts("PASS: real Flash C driver error exit, no redundant infinite wait, cursor and overflow guards");
}
