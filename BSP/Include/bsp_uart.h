#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "hpm_soc.h"
#include "hpm_uart_drv.h"
#include "hpm_gpio_drv.h"
#include "hpm_common.h"
#include "hpm_clock_drv.h"

#define BOARD_DEBUG_UART_VALID


#ifdef BOARD_DEBUG_UART_VALID
#define BOARD_CONSOLE_TYPE   (0)
 
#define BOADR_DBG_UART_BASE        HPM_UART0
#define BOADR_DBG_UART_IRQ         IRQn_UART0
#define BOARD_DBG_UART_BAUDRATE    (115200UL)
#define BOARD_DBG_UART_CLK_NAME    clock_uart0
#define BOARD_DBG_UART_RX_DMA_REQ  HPM_DMA_SRC_UART0_RX
#define BOARD_DBG_UART_TX_DMA_REQ  HPM_DMA_SRC_UART0_TX

#define BOARD_DBG_UART_RX_GPIO_PIN   IOC_PAD_PA01
#define BOARD_DBG_UART_TX_GPIO_PIN   IOC_PAD_PA00
#define BOARD_DBG_UART_RX_GPIO_FUNC  IOC_PA01_FUNC_CTL_UART0_RXD
#define BOARD_DBG_UART_TX_GPIO_FUNC  IOC_PA00_FUNC_CTL_UART0_TXD
#endif


void bsp_uart_init(void);


#endif /* __BSP_UART_H */