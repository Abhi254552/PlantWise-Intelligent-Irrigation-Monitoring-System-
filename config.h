/* config.h – very small helper to persist one 32-bit value in STM32F4
 *
 * We keep the threshold at the first word of the last  sector (sector 11)
 * of the F407 flash (address 0x080E 0000).  The helpers are intentionally
 * blocking and very small because we only touch them when the user sends
 * a new value.
 */
#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define CFG_FLASH_ADDR  0x080E0000U   /* start of sector 11 (128 KiB) */

HAL_StatusTypeDef cfg_load_uint32(uint32_t *dst);
HAL_StatusTypeDef cfg_store_uint32(uint32_t val);

#endif /* __CONFIG_H__ */
