#include "config.h"

static void erase_cfg_sector(void)
{
    FLASH_EraseInitTypeDef er = {
        .TypeErase    = FLASH_TYPEERASE_SECTORS,
        .Sector       = CFG_FLASH_SECTOR,
        .NbSectors    = 1,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3    
    };
    uint32_t err;
    HAL_FLASHEx_Erase(&er, &err);  
}

uint32_t cfg_load_uint32(uint32_t *dest)
{
    uint32_t v = *(__IO uint32_t *)CFG_FLASH_ADDRESS;
    if (dest) *dest = v;
    return v;
}

void cfg_store_uint32(uint32_t val)
{
    HAL_FLASH_Unlock();

    uint32_t cur = *(__IO uint32_t *)CFG_FLASH_ADDRESS;
    if (cur != val)                    
    {
        erase_cfg_sector();
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, CFG_FLASH_ADDRESS, val);
    }

    HAL_FLASH_Lock();
}
