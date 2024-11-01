#ifndef __BSP_SPI_H
#define __BSP_SPI_H

#include "hpm_soc.h"
#include "hpm_spi_drv.h"
#include "hpm_gpio_drv.h"
#include "hpm_common.h"
#include "hpm_clock_drv.h"

#define BOARD_MAGENCODER_IMU_SPI_GPIO_CTRL  HPM_GPIO0
 
#define BOADR_MAGENCODER_IMU_SPI_BASE        HPM_SPI3
#define BOADR_MAGENCODER_IMU_SPI_IRQ         IRQn_SPI3
#define BOARD_MAGENCODER_IMU_SPI_CLK_FREQ    (500000UL)   // 5MHz
#define BOARD_MAGENCODER_IMU_SPI_CLK_NAME    clock_spi3
#define BOARD_MAGENCODER_IMU_SPI_RX_DMA_REQ  HPM_DMA_SRC_SPI3_RX
#define BOARD_MAGENCODER_IMU_SPI_TX_DMA_REQ  HPM_DMA_SRC_SPI3_TX

#define BOARD_MAGENCODER_IMU_SPI_CLK_GPIO_PIN    IOC_PAD_PA11
#define BOARD_MAGENCODER_IMU_SPI_MIS0_GPIO_PIN   IOC_PAD_PA12
#define BOARD_MAGENCODER_IMU_SPI_MOSI_GPIO_PIN   IOC_PAD_PA13
#define BOARD_MAGENCODER_IMU_SPI_CLK_GPIO_FUNC   IOC_PA11_FUNC_CTL_SPI3_SCLK
#define BOARD_MAGENCODER_IMU_SPI_MIS0_GPIO_FUNC  IOC_PA12_FUNC_CTL_SPI3_MISO
#define BOARD_MAGENCODER_IMU_SPI_MOSI_GPIO_FUNC  IOC_PA13_FUNC_CTL_SPI3_MOSI

#define BOARD_MAGENCODER_SPI_CS_GPIO_PIN    IOC_PAD_PA10
#define BOARD_MAGENCODER_SPI_CS_GPIO_FUNC   IOC_PA10_FUNC_CTL_GPIO_A_10
#define BOARD_MAGENCODER_SPI_CS_GPIO_INDEX  GPIO_DI_GPIOA
#define BOARD_IMU_SPI_CS_GPIO_PIN      IOC_PAD_PB08
#define BOARD_IMU_SPI_CS_GPIO_FUNC     IOC_PB08_FUNC_CTL_GPIO_B_08
#define BOARD_IMU_SPI_CS_GPIO_INDEX    GPIO_DI_GPIOB

#define BOARD_MAGENCODER_SPI_CS_HIGH   gpio_write_pin(BOARD_MAGENCODER_IMU_SPI_GPIO_CTRL, BOARD_MAGENCODER_SPI_CS_GPIO_INDEX, BOARD_MAGENCODER_SPI_CS_GPIO_PIN, 1);
#define BOARD_MAGENCODER_SPI_CS_LOW    gpio_write_pin(BOARD_MAGENCODER_IMU_SPI_GPIO_CTRL, BOARD_MAGENCODER_SPI_CS_GPIO_INDEX, BOARD_MAGENCODER_SPI_CS_GPIO_PIN, 0);

#define BOARD_IMU_SPI_CS_HIGH   gpio_write_pin(BOARD_MAGENCODER_IMU_SPI_GPIO_CTRL, BOARD_IMU_SPI_CS_GPIO_INDEX, BOARD_IMU_SPI_CS_GPIO_PIN, 1);
#define BOARD_IMU_SPI_CS_LOW    gpio_write_pin(BOARD_MAGENCODER_IMU_SPI_GPIO_CTRL, BOARD_IMU_SPI_CS_GPIO_INDEX, BOARD_IMU_SPI_CS_GPIO_PIN, 0);

extern uint32_t magencoder_imu_spi_clcok;

void bsp_spi_init(void);

hpm_stat_t bsp_magencoder_spi_send(uint8_t cmd, uint8_t addr, uint8_t *wbuff, uint16_t len);
hpm_stat_t bsp_magencoder_spi_receive(uint8_t cmd, uint8_t addr, uint8_t *rbuff, uint16_t len);
hpm_stat_t bsp_magencoder_spi_receive_simple(uint8_t cmd, uint8_t addr, uint8_t *rbuff, uint16_t len);

hpm_stat_t bsp_imu_spi_send(uint8_t cmd, uint8_t addr, uint8_t *wbuff, uint16_t len);
hpm_stat_t bsp_imu_spi_receive(uint8_t cmd, uint8_t addr, uint8_t *rbuff, uint16_t len);
hpm_stat_t bsp_imu_spi_receive_simple(uint8_t cmd, uint8_t addr, uint8_t *rbuff, uint16_t len);

void bsp_magencoder_imu_spi_receive_init(void);
void bsp_magencoder_imu_spi_set_clock_div(uint8_t div);

#endif /* __BSP_SPI_H */