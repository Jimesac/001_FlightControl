/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-11-12 15:00:08
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-11-25 14:04:59
 * @FilePath: \test_v1\BSP\Include\bsp_uart.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "hpm_soc.h"
#include "hpm_uart_drv.h"
#include "hpm_gpio_drv.h"
#include "hpm_common.h"
#include "hpm_clock_drv.h"

#define BOARD_DEBUG_UART_VALID

#define UART_TX_MESSAGE_QUEUES_NUM   (3)

#define UART_TX_BUF_SIZE   (BOARD_DBG_UART_TX_DMA_BUF_SIZE)


#ifdef BOARD_DEBUG_UART_VALID
#define BOARD_CONSOLE_TYPE   (0)
 
#define BOADR_DBG_UART_BASE        HPM_UART0
#define BOADR_DBG_UART_IRQ         IRQn_UART0
#define BOARD_DBG_UART_BAUDRATE    (460800UL)
#define BOARD_DBG_UART_CLK_NAME    clock_uart0

#define BOARD_DBG_UART_RX_GPIO_PIN   IOC_PAD_PA01
#define BOARD_DBG_UART_TX_GPIO_PIN   IOC_PAD_PA00
#define BOARD_DBG_UART_RX_GPIO_FUNC  IOC_PA01_FUNC_CTL_UART0_RXD
#define BOARD_DBG_UART_TX_GPIO_FUNC  IOC_PA00_FUNC_CTL_UART0_TXD
#endif

#define BOADR_COM_MA_UART_BASE        HPM_UART7
#define BOADR_COM_MA_UART_IRQ         IRQn_UART7
#define BOARD_COM_MA_UART_BAUDRATE    (115200UL)
#define BOARD_COM_MA_UART_CLK_NAME    clock_uart7

#define BOARD_COM_MA_UART_RX_GPIO_PIN   IOC_PAD_PA30
#define BOARD_COM_MA_UART_TX_GPIO_PIN   IOC_PAD_PA31
#define BOARD_COM_MA_UART_RX_GPIO_FUNC  IOC_PA30_FUNC_CTL_UART7_RXD
#define BOARD_COM_MA_UART_TX_GPIO_FUNC  IOC_PA31_FUNC_CTL_UART7_TXD

/* 飞控串口 begin*/
#define BOADR_COM_FC_UART_BASE        HPM_UART3
#define BOADR_COM_FC_UART_IRQ         IRQn_UART3
#define BOARD_COM_FC_UART_BAUDRATE    (115200UL)
#define BOARD_COM_FC_UART_CLK_NAME    clock_uart3

#define BOARD_COM_FC_UART_RX_GPIO_PIN   IOC_PAD_PA14
#define BOARD_COM_FC_UART_TX_GPIO_PIN   IOC_PAD_PA15
#define BOARD_COM_FC_UART_RX_GPIO_FUNC  IOC_PA14_FUNC_CTL_UART3_RXD
#define BOARD_COM_FC_UART_TX_GPIO_FUNC  IOC_PA15_FUNC_CTL_UART3_TXD
/*end*/

void bsp_uart_init(void);

uint8_t* bsp_uart_dbg_get_tx_pbuf(void);
void bsp_uart_dbg_set_tx_size(uint16_t len);
void bsp_uart_dbg_schedule_transmit(void);

uint8_t* bsp_uart_com_ma_get_tx_pbuf(void);
void bsp_uart_com_ma_set_tx_size(uint16_t len);
void bsp_uart_com_ma_schedule_transmit(void);

/*飞控*/
uint8_t* bsp_uart_com_fc_get_tx_pbuf(void);
void bsp_uart_com_fc_set_tx_size(uint16_t len);
void bsp_uart_com_fc_schedule_transmit(void);
#endif /* __BSP_UART_H */