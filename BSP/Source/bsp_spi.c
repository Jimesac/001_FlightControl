#include "bsp_spi.h"
#include <stdio.h>

spi_control_config_t spi_magencoder_ctrl_cfg = {0};

void bsp_spi_init(void)
{
    uint32_t spi_clcok;
    spi_timing_config_t timing_config = {0};
    spi_format_config_t format_config = {0};
    

    HPM_IOC->PAD[BOARD_MAGENCODER_SPI_CS_GPIO_PIN].FUNC_CTL = BOARD_MAGENCODER_SPI_CS_GPIO_FUNC;
    HPM_IOC->PAD[BOARD_MAGENCODER_SPI_CLK_GPIO_PIN].FUNC_CTL = BOARD_MAGENCODER_SPI_CLK_GPIO_FUNC | IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1);
    HPM_IOC->PAD[BOARD_MAGENCODER_SPI_MIS0_GPIO_PIN].FUNC_CTL = BOARD_MAGENCODER_SPI_MIS0_GPIO_FUNC;
    HPM_IOC->PAD[BOARD_MAGENCODER_SPI_MOSI_GPIO_PIN].FUNC_CTL = BOARD_MAGENCODER_SPI_MOSI_GPIO_FUNC;
    gpio_set_pin_output_with_initial(BOARD_MAGENCODER_SPI_GPIO_CTRL,  BOARD_MAGENCODER_SPI_CS_GPIO_INDEX, BOARD_MAGENCODER_SPI_CS_GPIO_PIN, 1);
  
    spi_clcok = clock_get_frequency(BOARD_MAGENCODER_SPI_CLK_NAME);

    /* set SPI sclk frequency for master */
    spi_master_get_default_timing_config(&timing_config);
    timing_config.master_config.clk_src_freq_in_hz = spi_clcok;
    timing_config.master_config.sclk_freq_in_hz = BOARD_MAGENCODER_SPI_CLK_FREQ;
    if (status_success != spi_master_timing_init(BOADR_MAGENCODER_SPI_BASE, &timing_config)) {
        printf("SPI master timming init failed\n");
        while (1) {
        }
    }

    /* set SPI format config for master */
    spi_master_get_default_format_config(&format_config);
    format_config.common_config.data_len_in_bits = 8UL;
    format_config.common_config.mode = spi_master_mode;
    format_config.common_config.cpol = spi_sclk_high_idle;
    format_config.common_config.cpha = spi_sclk_sampling_even_clk_edges;
    spi_format_init(BOADR_MAGENCODER_SPI_BASE, &format_config);
    printf("SPI-Master transfer format is configured.\n");

    /* set SPI control config for master */
    spi_master_get_default_control_config(&spi_magencoder_ctrl_cfg);
    spi_magencoder_ctrl_cfg.master_config.cmd_enable = true;  /* cmd phase control for master */
    spi_magencoder_ctrl_cfg.master_config.addr_enable = true; /* address phase control for master */
    spi_magencoder_ctrl_cfg.common_config.trans_mode = spi_trans_write_read_together;

}


hpm_stat_t bsp_magencoder_spi_send(uint8_t cmd, uint8_t addr, uint8_t *wbuff, uint16_t len)
{
    hpm_stat_t stat;

    BOARD_MAGENCODER_SPI_CS_LOW
    
    spi_magencoder_ctrl_cfg.common_config.trans_mode = spi_trans_write_only;
    stat = spi_transfer(BOADR_MAGENCODER_SPI_BASE,
                &spi_magencoder_ctrl_cfg,
                &cmd, (uint32_t *)&addr,
                wbuff, len, NULL, 0);

    BOARD_MAGENCODER_SPI_CS_HIGH

    return stat;
}


hpm_stat_t bsp_magencoder_spi_receive(uint8_t cmd, uint8_t addr, uint8_t *rbuff, uint16_t len)
{
    hpm_stat_t stat;

    BOARD_MAGENCODER_SPI_CS_LOW
    
    spi_magencoder_ctrl_cfg.common_config.trans_mode = spi_trans_read_only;
    stat = spi_transfer(BOADR_MAGENCODER_SPI_BASE,
                &spi_magencoder_ctrl_cfg,
                &cmd, (uint32_t *)&addr,
                NULL, 0, rbuff, len);

    BOARD_MAGENCODER_SPI_CS_HIGH

    return stat;
}