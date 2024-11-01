#ifndef __BSP_FLASH_H
#define __BSP_FLASH_H

#include "hpm_soc.h"
#include "hpm_romapi.h"
#include "hpm_clock_drv.h"

/* Flash section */
#define BOARD_XPI_NOR_XPI_BASE     (HPM_XPI0)
#define BOARD_XPI_NOR_CFG_OPT_HDR  (0xfcf90002U)
#define BOARD_XPI_NOR_CFG_OPT_OPT0 (0x00000006U)
#define BOARD_XPI_NOR_CFG_OPT_OPT1 (0x00001000U)

#define USER_PARAMS_FLASH_INDEX_LAST   (2U)    // 倒数第2个SECTOR

#define USER_PARAMS_FLASH_WORDS_SIZE   (820U)

void bsp_flash_init(void);

hpm_stat_t bsp_flash_read_words(uint32_t addr, uint32_t *pbuf, uint16_t len);
hpm_stat_t bsp_flash_write_words(uint32_t addr, const uint32_t *pbuf, uint16_t len);

hpm_stat_t bsp_flash_user_params_read_words(uint32_t addr_words_offset, uint32_t *pbuf, uint16_t len);
hpm_stat_t bsp_flash_user_params_write_words(uint32_t addr_words_offset, const uint32_t *pbuf, uint16_t len);

#endif /* __BSP_FLASH_H */