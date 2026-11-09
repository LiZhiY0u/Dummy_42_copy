#pragma once
#include <stdint.h>
#define FLASH_PAGE_SIZE 1024U
#define HAL_MAX_DELAY 0xffffffffU
#define FLASH_CR_PER 2U
#define FLASH_TYPEERASE_PAGES 1U
#define FLASH_TYPEPROGRAM_HALFWORD 1U
#define FLASH_TYPEPROGRAM_WORD 2U
#define FLASH_TYPEPROGRAM_DOUBLEWORD 3U
#define CLEAR_BIT(reg,mask) ((reg)&=~(mask))
typedef enum { HAL_OK=0,HAL_ERROR=1,HAL_BUSY=2,HAL_TIMEOUT=3 } HAL_StatusTypeDef;
typedef struct {uint32_t TypeErase,PageAddress,NbPages;} FLASH_EraseInitTypeDef;
typedef struct {uint32_t CR;} FLASH_TypeDef;
extern FLASH_TypeDef *FLASH;
HAL_StatusTypeDef HAL_FLASH_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void);
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef*,uint32_t*);
HAL_StatusTypeDef FLASH_WaitForLastOperation(uint32_t);
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t,uint32_t,uint64_t);
