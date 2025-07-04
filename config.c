/* config.c – super-tiny flash helper (sector 11, STM32F407) */
#include "config.h"

static void flash_unlock(void)
{
    HAL_FLASH_Unlock();
    /* clear previous flags */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
                           FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                           FLASH_FLAG_PGPERR| FLASH_FLAG_PGSERR);
}
static void flash_lock(void) { HAL_FLASH_Lock(); }

HAL_StatusTypeDef cfg_load_uint32(uint32_t *dst)
{
    if (!dst) return HAL_ERROR;
    *dst = *(uint32_t *)CFG_FLASH_ADDR;
    return HAL_OK;
}

HAL_StatusTypeDef cfg_store_uint32(uint32_t val)
{
    flash_unlock();

    FLASH_EraseInitTypeDef er = {0};
    uint32_t sector_error     = 0;
    er.TypeErase   = FLASH_TYPEERASE_SECTORS;
    er.VoltageRange= FLASH_VOLTAGE_RANGE_3;
    er.Sector      = FLASH_SECTOR_11;
    er.NbSectors   = 1;
    if (HAL_FLASHEx_Erase(&er, &sector_error) != HAL_OK) {
        flash_lock(); return HAL_ERROR;
    }
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                          CFG_FLASH_ADDR, val) != HAL_OK) {
        flash_lock(); return HAL_ERROR;
    }
    flash_lock();
    return HAL_OK;
}
