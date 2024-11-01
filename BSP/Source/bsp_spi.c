#include "bsp_spi.h"
#include "bsp_timer.h"
#include <stdio.h>

static void bsp_magencoder_imu_spi_init(void);

spi_control_config_t spi_magencoder_imu_ctrl_cfg = {0};

uint32_t magencoder_imu_spi_clcok = 0;

void bsp_spi_init(void)
{
    bsp_magencoder_imu_spi_init();
}


static void bsp_magencoder_imu_spi_init(void)
{
    spi_timing_config_t timing_config = {0};
    spi_format_config_t format_config = {0};
    

    HPM_IOC->PAD[BOARD_MAGENCODER_SPI_CS_GPIO_PIN].FUNC_CTL = BOARD_MAGENCODER_SPI_CS_GPIO_FUNC;
    HPM_IOC->PAD[BOARD_IMU_SPI_CS_GPIO_PIN].FUNC_CTL = BOARD_IMU_SPI_CS_GPIO_FUNC;
    HPM_IOC->PAD[BOARD_MAGENCODER_IMU_SPI_CLK_GPIO_PIN].FUNC_CTL = BOARD_MAGENCODER_IMU_SPI_CLK_GPIO_FUNC | IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1);
    HPM_IOC->PAD[BOARD_MAGENCODER_IMU_SPI_MIS0_GPIO_PIN].FUNC_CTL = BOARD_MAGENCODER_IMU_SPI_MIS0_GPIO_FUNC;
    HPM_IOC->PAD[BOARD_MAGENCODER_IMU_SPI_MOSI_GPIO_PIN].FUNC_CTL = BOARD_MAGENCODER_IMU_SPI_MOSI_GPIO_FUNC;
    gpio_set_pin_output_with_initial(BOARD_MAGENCODER_IMU_SPI_GPIO_CTRL,  BOARD_MAGENCODER_SPI_CS_GPIO_INDEX, BOARD_MAGENCODER_SPI_CS_GPIO_PIN, 1);
    gpio_set_pin_output_with_initial(BOARD_MAGENCODER_IMU_SPI_GPIO_CTRL,  BOARD_IMU_SPI_CS_GPIO_INDEX, BOARD_IMU_SPI_CS_GPIO_PIN, 1);
  
    magencoder_imu_spi_clcok = clock_get_frequency(BOARD_MAGENCODER_IMU_SPI_CLK_NAME);

    /* set SPI sclk frequency for master */
    spi_master_get_default_timing_config(&timing_config);
    timing_config.master_config.clk_src_freq_in_hz = magencoder_imu_spi_clcok;
    timing_config.master_config.sclk_freq_in_hz = BOARD_MAGENCODER_IMU_SPI_CLK_FREQ;
    if (status_success != spi_master_timing_init(BOADR_MAGENCODER_IMU_SPI_BASE, &timing_config)) {
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
    spi_format_init(BOADR_MAGENCODER_IMU_SPI_BASE, &format_config);
    printf("SPI-Master transfer format is configured.\n");

    /* set SPI control config for master */
    spi_master_get_default_control_config(&spi_magencoder_imu_ctrl_cfg);
    spi_magencoder_imu_ctrl_cfg.master_config.cmd_enable = true;  /* cmd phase control for master */
    spi_magencoder_imu_ctrl_cfg.master_config.addr_enable = false; /* address phase control for master */
    spi_magencoder_imu_ctrl_cfg.master_config.token_enable = false;
    spi_magencoder_imu_ctrl_cfg.common_config.trans_mode = spi_trans_write_read_together;
}

hpm_stat_t bsp_magencoder_spi_send(uint8_t cmd, uint8_t addr, uint8_t *wbuff, uint16_t len)
{
    hpm_stat_t stat;

    BOARD_MAGENCODER_SPI_CS_LOW

    spi_magencoder_imu_ctrl_cfg.common_config.trans_mode = spi_trans_write_only;
    stat = spi_transfer(BOADR_MAGENCODER_IMU_SPI_BASE,
                &spi_magencoder_imu_ctrl_cfg,
                &cmd, (uint32_t *)&addr,
                wbuff, len, NULL, 0);

    BOARD_MAGENCODER_SPI_CS_HIGH

    return stat;
}


hpm_stat_t bsp_magencoder_spi_receive(uint8_t cmd, uint8_t addr, uint8_t *rbuff, uint16_t len)
{
    hpm_stat_t stat;

    BOARD_MAGENCODER_SPI_CS_LOW
    
    spi_magencoder_imu_ctrl_cfg.common_config.trans_mode = spi_trans_read_only;
    stat = spi_transfer(BOADR_MAGENCODER_IMU_SPI_BASE,
                &spi_magencoder_imu_ctrl_cfg,
                &cmd, (uint32_t *)&addr,
                NULL, 0, rbuff, len);

    BOARD_MAGENCODER_SPI_CS_HIGH

    return stat;
}

hpm_stat_t bsp_magencoder_spi_receive_simple(uint8_t cmd, uint8_t addr, uint8_t *rbuff, uint16_t len)
{
    uint32_t transferred = 0;
    uint32_t retry = 0;
    uint8_t rx_valid_size = 0;
    uint8_t j = 0;

    BOARD_MAGENCODER_SPI_CS_LOW
    
    BOADR_MAGENCODER_IMU_SPI_BASE->TRANSCTRL |= SPI_TRANSCTRL_RDTRANCNT_SET(len - 1);
    BOADR_MAGENCODER_IMU_SPI_BASE->RD_TRANS_CNT = len - 1;

    BOADR_MAGENCODER_IMU_SPI_BASE->CMD = SPI_CMD_CMD_SET(cmd);

    /* data transfer */
    while (transferred < len) {
        rx_valid_size = (((BOADR_MAGENCODER_IMU_SPI_BASE->STATUS & 0x03000000) >> 18) | ((BOADR_MAGENCODER_IMU_SPI_BASE->STATUS & 0x00003F00) >> 8));
        if (rx_valid_size > 0) {
            for (j = 0; j < rx_valid_size; j++) {
                //temp = BOADR_MAGENCODER_IMU_SPI_BASE->DATA;
                *(rbuff++) = (uint8_t)(BOADR_MAGENCODER_IMU_SPI_BASE->DATA);
            }
            /* transfer count increment */
            transferred += rx_valid_size;
            retry = 0;
        } else {
            if (retry > 5000) {
                break;
            }
            retry++;
        }
    }

    if (retry > 5000) {
        BOARD_MAGENCODER_SPI_CS_HIGH
        /* dummy state may triggers timeout if dummy count, retry count, spi rate and cpu frequency are inappropriate */
        return status_timeout;
    }
    BOARD_MAGENCODER_SPI_CS_HIGH
    return status_success;
}

hpm_stat_t bsp_imu_spi_send(uint8_t cmd, uint8_t addr, uint8_t *wbuff, uint16_t len)
{
    hpm_stat_t stat;

    BOARD_IMU_SPI_CS_LOW
    
    spi_magencoder_imu_ctrl_cfg.common_config.trans_mode = spi_trans_write_only;
    stat = spi_transfer(BOADR_MAGENCODER_IMU_SPI_BASE,
                &spi_magencoder_imu_ctrl_cfg,
                &cmd, (uint32_t *)&addr,
                wbuff, len, NULL, 0);

    BOARD_IMU_SPI_CS_HIGH

    return stat;
}

hpm_stat_t bsp_imu_spi_receive(uint8_t cmd, uint8_t addr, uint8_t *rbuff, uint16_t len)
{
    hpm_stat_t stat;

    BOARD_IMU_SPI_CS_LOW
    
    spi_magencoder_imu_ctrl_cfg.common_config.trans_mode = spi_trans_read_only;
    stat = spi_transfer(BOADR_MAGENCODER_IMU_SPI_BASE,
                &spi_magencoder_imu_ctrl_cfg,
                &cmd, (uint32_t *)&addr,
                NULL, 0, rbuff, len);

    BOARD_IMU_SPI_CS_HIGH

    return stat;
}

uint32_t spi_test_us[10][2] = {0};
hpm_stat_t bsp_imu_spi_receive_simple(uint8_t cmd, uint8_t addr, uint8_t *rbuff, uint16_t len)
{
    uint32_t transferred = 0;
    uint32_t retry = 0;
    uint8_t rx_valid_size = 0;
    uint8_t j = 0;

    BOARD_IMU_SPI_CS_LOW
    
    //spi_test_us[0][0] = bsp_timer_clock_start_0p1us();
    //spi_magencoder_imu_ctrl_cfg.common_config.trans_mode = spi_trans_read_only;
    //stat = spi_transfer(BOADR_MAGENCODER_IMU_SPI_BASE,
    //            &spi_magencoder_imu_ctrl_cfg,
    //            &cmd, (uint32_t *)&addr,
    //            NULL, 0, rbuff, len);

    //SPI_Type *ptr = BOADR_MAGENCODER_IMU_SPI_BASE;
    //spi_control_config_t *config = &spi_magencoder_imu_ctrl_cfg;
    //uint8_t *cmd0 = &cmd;
    //uint32_t *addr0 = (uint32_t *)&addr;
    //uint8_t *wbuff =  NULL;
    //uint32_t wcount = 0;
    //uint8_t *rbuff0 = rbuff;
    //uint32_t rcount = len;

    //uint8_t mode = 0, data_len_in_bytes = 1, trans_mode = 2;
    //spi_test_us[0][1] = bsp_timer_clock_end_0p1us(spi_test_us[0][0]);


    //spi_test_us[1][0] = bsp_timer_clock_start_0p1us();
    //stat = spi_control_init(BOADR_MAGENCODER_IMU_SPI_BASE, &spi_magencoder_imu_ctrl_cfg, 0, len);
    BOADR_MAGENCODER_IMU_SPI_BASE->TRANSCTRL |= SPI_TRANSCTRL_RDTRANCNT_SET(len - 1);
    BOADR_MAGENCODER_IMU_SPI_BASE->RD_TRANS_CNT = len - 1;
    //if (stat != status_success) {
    //    BOARD_IMU_SPI_CS_HIGH
    //    return stat;
    //}
    //spi_test_us[1][1] = bsp_timer_clock_end_0p1us(spi_test_us[1][0]);


    //spi_test_us[2][0] = bsp_timer_clock_start_0p1us();
    /* write command on master mode */
    //stat = spi_write_command(ptr, mode, config, cmd0);
    BOADR_MAGENCODER_IMU_SPI_BASE->CMD = SPI_CMD_CMD_SET(cmd);
    //if (stat != status_success) {
    //    BOARD_IMU_SPI_CS_HIGH
    //    return stat;
    //}
    //spi_test_us[2][1] = bsp_timer_clock_end_0p1us(spi_test_us[2][0]);


    //stat = spi_read_data(BOADR_MAGENCODER_IMU_SPI_BASE, 1, rbuff, len);



    /* data transfer */
    while (transferred < len) {
        rx_valid_size = (((BOADR_MAGENCODER_IMU_SPI_BASE->STATUS & 0x03000000) >> 18) | ((BOADR_MAGENCODER_IMU_SPI_BASE->STATUS & 0x00003F00) >> 8));
        if (rx_valid_size > 0) {
            for (j = 0; j < rx_valid_size; j++) {
                //temp = BOADR_MAGENCODER_IMU_SPI_BASE->DATA;
                *(rbuff++) = (uint8_t)(BOADR_MAGENCODER_IMU_SPI_BASE->DATA);
            }
            /* transfer count increment */
            transferred += rx_valid_size;
            retry = 0;
        } else {
            if (retry > 5000) {
                break;
            }
            retry++;
        }
    }

    if (retry > 5000) {
        BOARD_IMU_SPI_CS_HIGH
        /* dummy state may triggers timeout if dummy count, retry count, spi rate and cpu frequency are inappropriate */
        return status_timeout;
    }
    BOARD_IMU_SPI_CS_HIGH
    return status_success;

}


//hpm_stat_t bsp_imu_spi_receive(uint8_t cmd, uint8_t addr, uint8_t *rbuff, uint16_t len)
//{
//    hpm_stat_t stat;

//    BOARD_IMU_SPI_CS_LOW

//    stat = spi_control_init(BOADR_MAGENCODER_IMU_SPI_BASE, &spi_magencoder_imu_ctrl_cfg, 0, len);
//    if (stat != status_success) {
//        return stat;
//    }


//    /* write address on master mode */
//    stat = spi_write_address(BOADR_MAGENCODER_IMU_SPI_BASE, 1, &spi_magencoder_imu_ctrl_cfg, 0);
//    if (stat != status_success) {
//        return stat;
//    }

//    /* write command on master mode */
//    stat = spi_write_command(BOADR_MAGENCODER_IMU_SPI_BASE, 1, &spi_magencoder_imu_ctrl_cfg, &cmd);
//    if (stat != status_success) {
//        return stat;
//    }

//    /* data phase */
//    stat = spi_read_data(BOADR_MAGENCODER_IMU_SPI_BASE, 1, rbuff, len);

//    /* on the slave mode, if master keeps asserting the cs signal, it's maybe timeout */
//    stat = spi_wait_for_idle_status(BOADR_MAGENCODER_IMU_SPI_BASE);
//    BOARD_IMU_SPI_CS_HIGH
//    return stat;

//}

void bsp_magencoder_imu_spi_receive_init(void)
{
    spi_magencoder_imu_ctrl_cfg.common_config.trans_mode = spi_trans_read_only;
    spi_control_init(BOADR_MAGENCODER_IMU_SPI_BASE, &spi_magencoder_imu_ctrl_cfg, 0, 1);
}

void bsp_magencoder_imu_spi_set_clock_div(uint8_t div)
{
    spi_master_set_sclk_div(BOADR_MAGENCODER_IMU_SPI_BASE, div);
}
