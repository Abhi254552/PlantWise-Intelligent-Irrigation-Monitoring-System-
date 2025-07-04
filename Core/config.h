#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define CFG_FLASH_SECTOR     FLASH_SECTOR_11
#define CFG_FLASH_ADDRESS    ((uint32_t)0x080E0000u)

void     cfg_store_uint32 (uint32_t val);     
uint32_t cfg_load_uint32  (uint32_t *dest);    

static inline void save_threshold_to_flash(uint32_t v) { cfg_store_uint32(v); }
static inline uint32_t load_threshold_from_flash(void)
{
    uint32_t tmp;
    return cfg_load_uint32(&tmp);
}

#endif 
