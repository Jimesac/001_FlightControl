#include "bsp_flash.h"
#include <stdio.h>

struct {
    xpi_nor_config_option_t option;
    xpi_nor_config_t xpi_nor_config;

    uint32_t flash_size;
    uint32_t sector_size;
    uint32_t page_size;
}nor_flash_info;

void bsp_flash_init(void)
{
    hpm_stat_t status;

    nor_flash_info.option.header.U = BOARD_XPI_NOR_CFG_OPT_HDR;
    nor_flash_info.option.option0.U = BOARD_XPI_NOR_CFG_OPT_OPT0;
    nor_flash_info.option.option1.U = BOARD_XPI_NOR_CFG_OPT_OPT1;

    status = rom_xpi_nor_auto_config(BOARD_XPI_NOR_XPI_BASE, &nor_flash_info.xpi_nor_config, &nor_flash_info.option);
    if (status != status_success) {
        printf("nor flash configure failed\n");
    }
    rom_xpi_nor_get_property(BOARD_XPI_NOR_XPI_BASE, &nor_flash_info.xpi_nor_config, xpi_nor_property_total_size, &nor_flash_info.flash_size);
    rom_xpi_nor_get_property(BOARD_XPI_NOR_XPI_BASE, &nor_flash_info.xpi_nor_config, xpi_nor_property_sector_size, &nor_flash_info.sector_size);
    rom_xpi_nor_get_property(BOARD_XPI_NOR_XPI_BASE, &nor_flash_info.xpi_nor_config, xpi_nor_property_page_size, &nor_flash_info.page_size);

    if (nor_flash_info.sector_size < USER_PARAMS_FLASH_WORDS_SIZE)
    {
        printf("ERROR: USER FLASH SIZE AND FLASH SECTOR SIZE ERR \n");
    }

}

hpm_stat_t bsp_flash_read_words(uint32_t addr, uint32_t *pbuf, uint16_t len)
{
    hpm_stat_t status;

    status = rom_xpi_nor_read(BOARD_XPI_NOR_XPI_BASE, xpi_xfer_channel_auto, &nor_flash_info.xpi_nor_config, pbuf, addr, len*4);
    return status;
}

hpm_stat_t bsp_flash_write_words(uint32_t addr, const uint32_t *pbuf, uint16_t len)
{
    hpm_stat_t status;
    
    if (addr % nor_flash_info.sector_size != 0)  return status_invalid_argument;

    status = rom_xpi_nor_program(BOARD_XPI_NOR_XPI_BASE, xpi_xfer_channel_auto, &nor_flash_info.xpi_nor_config, pbuf, addr, len*4);
    return status;
}

hpm_stat_t bsp_flash_user_params_read_words(uint32_t addr_words_offset, uint32_t *pbuf, uint16_t len)
{
    hpm_stat_t status;
    uint32_t addr;

    addr = nor_flash_info.flash_size - nor_flash_info.sector_size*USER_PARAMS_FLASH_INDEX_LAST;

    status = rom_xpi_nor_read(BOARD_XPI_NOR_XPI_BASE, xpi_xfer_channel_auto, &nor_flash_info.xpi_nor_config, pbuf, addr, len*4);
    return status;
}

hpm_stat_t bsp_flash_user_params_write_words(uint32_t addr_words_offset, const uint32_t *pbuf, uint16_t len)
{
    hpm_stat_t status;
    uint32_t s_read_buf[USER_PARAMS_FLASH_WORDS_SIZE];
    uint32_t addr;
    
    addr = nor_flash_info.flash_size - nor_flash_info.sector_size*USER_PARAMS_FLASH_INDEX_LAST;
    status = bsp_flash_read_words(addr, s_read_buf, USER_PARAMS_FLASH_WORDS_SIZE);

    if (status == status_success)
    {
        for (uint16_t i = 0, j = addr_words_offset; i < len; i++, j++)
        {
            s_read_buf[j] = pbuf[i];
            if (j >= (nor_flash_info.sector_size>>2)) return 0;
        }
        disable_irq_from_intc();
        status = rom_xpi_nor_erase(BOARD_XPI_NOR_XPI_BASE, xpi_xfer_channel_auto, &nor_flash_info.xpi_nor_config, addr, nor_flash_info.sector_size);
        status = rom_xpi_nor_program(BOARD_XPI_NOR_XPI_BASE, xpi_xfer_channel_auto, &nor_flash_info.xpi_nor_config, s_read_buf, \
                addr, USER_PARAMS_FLASH_WORDS_SIZE*4);
        enable_irq_from_intc();
        return status;
    }
    else 
    {
        return status;
    }
}