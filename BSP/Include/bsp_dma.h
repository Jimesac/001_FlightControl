#ifndef __BSP_DMA_H
#define __BSP_DMA_H

#include "hpm_soc.h"
#include "hpm_dmav2_drv.h"
#include "hpm_dmamux_drv.h"


#define BOARD_APP_HDMA      HPM_HDMA
#define BOARD_APP_DMAMUX    HPM_DMAMUX

#define BOARD_DBG_UART_TX_DMA_BUF_SIZE   (256U)
#define BOARD_DBG_UART_RX_DMA_BUF_SIZE   (512U)
#define BOARD_COM_MA_UART_TX_DMA_BUF_SIZE   (256U)
#define BOARD_COM_MA_UART_RX_DMA_BUF_SIZE   (576U)

#define BOARD_DBG_UART_TX_DMA_CHN        (0U)
#define BOARD_DBG_UART_RX_DMA_CHN        (1U)
#define BOARD_DBG_UART_TX_DMAMUX_CHN     DMA_SOC_CHN_TO_DMAMUX_CHN(BOARD_APP_DMAMUX, BOARD_DBG_UART_TX_DMA_CHN)
#define BOARD_DBG_UART_RX_DMAMUX_CHN     DMA_SOC_CHN_TO_DMAMUX_CHN(BOARD_APP_DMAMUX, BOARD_DBG_UART_RX_DMA_CHN)
#define BOARD_DBG_UART_RX_DMA_REQ        HPM_DMA_SRC_UART0_RX
#define BOARD_DBG_UART_TX_DMA_REQ        HPM_DMA_SRC_UART0_TX

#define BOARD_COM_MA_UART_TX_DMA_CHN        (2U)
#define BOARD_COM_MA_UART_RX_DMA_CHN        (3U)
#define BOARD_COM_MA_UART_TX_DMAMUX_CHN     DMA_SOC_CHN_TO_DMAMUX_CHN(BOARD_APP_DMAMUX, BOARD_COM_MA_UART_TX_DMA_CHN)
#define BOARD_COM_MA_UART_RX_DMAMUX_CHN     DMA_SOC_CHN_TO_DMAMUX_CHN(BOARD_APP_DMAMUX, BOARD_COM_MA_UART_RX_DMA_CHN)
#define BOARD_COM_MA_UART_RX_DMA_REQ        HPM_DMA_SRC_UART0_RX
#define BOARD_COM_MA_UART_TX_DMA_REQ        HPM_DMA_SRC_UART0_TX

extern bool uart_dbg_tx_dma_done;
extern bool uart_dbg_rx_dma_done;

extern bool uart_com_ma_tx_dma_done;
extern bool uart_com_ma_rx_dma_done;

void bsp_dma_init(void);


hpm_stat_t uart_debug_tx_trigger_dma(DMA_Type *dma_ptr, uint32_t size);
hpm_stat_t uart_debug_rx_trigger_dma(DMA_Type *dma_ptr, uint32_t dst, uint32_t size);
hpm_stat_t uart_debug_rx_circle_dma(DMA_Type *dma_ptr, uint32_t dst, uint32_t size);
void bsp_dma_set_uart_dbg_type(UART_Type *uart_ptr);
void bsp_dma_set_uart_com_ma_type(UART_Type *uart_ptr);
uint8_t* bsp_dma_get_uart_dbg_tx_pbuf(void);
uint8_t* bsp_dma_get_uart_com_ma_tx_pbuf(void);
#endif /* __BSP_LED_H */