#include "bsp_dma.h"
#include "user_config.h"
#include <stdio.h>

bool uart_dbg_tx_dma_done = false;
bool uart_dbg_rx_dma_done = false;
bool uart_com_ma_tx_dma_done = false;
bool uart_com_ma_rx_dma_done = false;

UART_Type* uart_dbg_type = (void *) 0;
UART_Type* uart_com_ma_type = (void *) 0;

ATTR_PLACE_AT_NONCACHEABLE_BSS_WITH_ALIGNMENT(4) uint8_t uart_dbg_rx_dma_buf[BOARD_DBG_UART_RX_DMA_BUF_SIZE] = {0};
ATTR_PLACE_AT_NONCACHEABLE_BSS_WITH_ALIGNMENT(4) uint8_t uart_dbg_tx_dma_buf[BOARD_DBG_UART_TX_DMA_BUF_SIZE] = {0};
ATTR_PLACE_AT_NONCACHEABLE_BSS_WITH_ALIGNMENT(4) uint8_t uart_com_ma_rx_dma_buf[BOARD_COM_MA_UART_RX_DMA_BUF_SIZE] = {0};
ATTR_PLACE_AT_NONCACHEABLE_BSS_WITH_ALIGNMENT(4) uint8_t uart_com_ma_tx_dma_buf[BOARD_COM_MA_UART_TX_DMA_BUF_SIZE] = {0};
ATTR_PLACE_AT_NONCACHEABLE_BSS_WITH_ALIGNMENT(8) dma_linked_descriptor_t uart_dbg_rx_descriptors[2];

void bsp_dma_init(void)
{
    if (uart_dbg_type == (void *) 0 || uart_com_ma_type == (void *) 0)
    {
        printf("failed to initialize dma\n");
        while (1) {
        }
    }
    uart_debug_rx_circle_dma(BOARD_APP_HDMA, (uint32_t)(&uart_dbg_rx_dma_buf[0]), BOARD_DBG_UART_RX_DMA_BUF_SIZE);
    uart_debug_rx_circle_dma(BOARD_APP_HDMA, (uint32_t)(&uart_com_ma_rx_dma_buf[0]), BOARD_COM_MA_UART_RX_DMA_BUF_SIZE);
    

    intc_m_enable_irq_with_priority(BOARD_HDMA_IRQ, BOARD_HDMA_IRQ_LEVEL);
    dmamux_config(BOARD_APP_DMAMUX, BOARD_DBG_UART_RX_DMAMUX_CHN, BOARD_DBG_UART_RX_DMA_REQ, true);
    dmamux_config(BOARD_APP_DMAMUX, BOARD_DBG_UART_TX_DMAMUX_CHN, BOARD_DBG_UART_TX_DMA_REQ, true);
    dmamux_config(BOARD_APP_DMAMUX, BOARD_COM_MA_UART_RX_DMAMUX_CHN, BOARD_COM_MA_UART_RX_DMA_REQ, true);
    dmamux_config(BOARD_APP_DMAMUX, BOARD_COM_MA_UART_TX_DMAMUX_CHN, BOARD_COM_MA_UART_TX_DMA_REQ, true);

    uart_dbg_tx_dma_done = true;
    uart_dbg_rx_dma_done = false;
    uart_com_ma_tx_dma_done = true;
    uart_com_ma_rx_dma_done = false;
}


void bsp_dma_isr(void)
{
    volatile hpm_stat_t stat_uart_dbg_rx_chn, stat_uart_dbg_rx_tx_chn;

    stat_uart_dbg_rx_chn = dma_check_transfer_status(BOARD_APP_HDMA, BOARD_DBG_UART_RX_DMA_CHN);
    if (stat_uart_dbg_rx_chn & DMA_CHANNEL_STATUS_TC) {
        dma_clear_transfer_status(BOARD_APP_HDMA, BOARD_DBG_UART_RX_DMA_CHN);
        uart_dbg_rx_dma_done = true;
    }

    stat_uart_dbg_rx_tx_chn = dma_check_transfer_status(BOARD_APP_HDMA, BOARD_DBG_UART_TX_DMA_CHN);
    if (stat_uart_dbg_rx_tx_chn & DMA_CHANNEL_STATUS_TC) {
        dma_clear_transfer_status(BOARD_APP_HDMA, BOARD_DBG_UART_TX_DMA_CHN);
        uart_dbg_tx_dma_done = true;
    }
}
SDK_DECLARE_EXT_ISR_M(BOARD_HDMA_IRQ, bsp_dma_isr)

hpm_stat_t uart_debug_tx_trigger_dma(DMA_Type *dma_ptr, uint32_t size)
{
    dma_handshake_config_t config;

    dma_default_handshake_config(dma_ptr, &config);
    config.ch_index = BOARD_DBG_UART_TX_DMA_CHN;
    config.dst = (uint32_t)&uart_dbg_type->THR;
    config.dst_fixed = true;
    config.src = (uint32_t)uart_dbg_tx_dma_buf;
    config.src_fixed = false;
    config.data_width = DMA_TRANSFER_WIDTH_BYTE;
    config.size_in_byte = size;

    return dma_setup_handshake(dma_ptr, &config, true);
}

hpm_stat_t uart_com_ma_tx_trigger_dma(DMA_Type *dma_ptr, uint32_t size)
{
    dma_handshake_config_t config;

    dma_default_handshake_config(dma_ptr, &config);
    config.ch_index = BOARD_COM_MA_UART_TX_DMA_CHN;
    config.dst = (uint32_t)&uart_com_ma_type->THR;
    config.dst_fixed = true;
    config.src = (uint32_t)uart_com_ma_tx_dma_buf;
    config.src_fixed = false;
    config.data_width = DMA_TRANSFER_WIDTH_BYTE;
    config.size_in_byte = size;

    return dma_setup_handshake(dma_ptr, &config, true);
}

hpm_stat_t uart_debug_rx_trigger_dma(DMA_Type *dma_ptr, uint32_t dst, uint32_t size)
{
    dma_handshake_config_t config;

    dma_default_handshake_config(dma_ptr, &config);
    config.ch_index = BOARD_DBG_UART_RX_DMA_CHN;
    config.dst = dst;
    config.dst_fixed = false;
    config.src = (uint32_t)&uart_dbg_type->RBR;
    config.src_fixed = true;
    config.data_width = DMA_TRANSFER_WIDTH_BYTE;
    config.size_in_byte = size;

    return dma_setup_handshake(dma_ptr, &config, true);
}

hpm_stat_t uart_com_ma_rx_trigger_dma(DMA_Type *dma_ptr, uint32_t dst, uint32_t size)
{
    dma_handshake_config_t config;

    dma_default_handshake_config(dma_ptr, &config);
    config.ch_index = BOARD_COM_MA_UART_RX_DMA_CHN;
    config.dst = dst;
    config.dst_fixed = false;
    config.src = (uint32_t)&uart_com_ma_type->RBR;
    config.src_fixed = true;
    config.data_width = DMA_TRANSFER_WIDTH_BYTE;
    config.size_in_byte = size;

    return dma_setup_handshake(dma_ptr, &config, true);
}

hpm_stat_t uart_debug_rx_circle_dma(DMA_Type *dma_ptr, uint32_t dst, uint32_t size)
{
    hpm_stat_t stat;
    dma_channel_config_t config = {0};

    /* 1.1 config chain descriptors */
    dma_default_channel_config(dma_ptr, &config);
    config.src_addr = (uint32_t)&uart_dbg_type->RBR;
    config.src_width = DMA_TRANSFER_WIDTH_BYTE; /*  In DMA handshake case, source width and destination width must be BYTE. */
    config.src_addr_ctrl = DMA_ADDRESS_CONTROL_FIXED;
    config.src_mode = DMA_HANDSHAKE_MODE_HANDSHAKE;
    config.dst_addr = core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)dst);
    config.dst_width = DMA_TRANSFER_WIDTH_BYTE; /*  In DMA handshake case, source width and destination width must be BYTE. */
    config.dst_addr_ctrl = DMA_ADDRESS_CONTROL_INCREMENT;
    config.dst_mode = DMA_HANDSHAKE_MODE_NORMAL;
    config.size_in_byte = size;
    config.src_burst_size = DMA_NUM_TRANSFER_PER_BURST_1T; /*  In DMA handshake case, source burst size must be 1 transfer, that is 0. */
    config.linked_ptr = core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)&uart_dbg_rx_descriptors[1]);
    stat = dma_config_linked_descriptor(dma_ptr, &uart_dbg_rx_descriptors[0], BOARD_DBG_UART_RX_DMA_CHN, &config);
    if (stat != status_success) {
        while (1) {
        };
    }

    config.linked_ptr = core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)&uart_dbg_rx_descriptors[0]);
    stat = dma_config_linked_descriptor(dma_ptr, &uart_dbg_rx_descriptors[1], BOARD_DBG_UART_RX_DMA_CHN, &config);
    if (stat != status_success) {
        while (1) {
        };
    }

    /* 1.2 config rx dma */
    config.linked_ptr = core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)&uart_dbg_rx_descriptors[0]);
    stat = dma_setup_channel(dma_ptr, BOARD_DBG_UART_RX_DMA_CHN, &config, true);
    if (stat != status_success) {
        while (1) {
        };
    }
}

hpm_stat_t uart_com_ma_rx_circle_dma(DMA_Type *dma_ptr, uint32_t dst, uint32_t size)
{
    hpm_stat_t stat;
    dma_channel_config_t config = {0};

    /* 1.1 config chain descriptors */
    dma_default_channel_config(dma_ptr, &config);
    config.src_addr = (uint32_t)&uart_com_ma_type->RBR;
    config.src_width = DMA_TRANSFER_WIDTH_BYTE; /*  In DMA handshake case, source width and destination width must be BYTE. */
    config.src_addr_ctrl = DMA_ADDRESS_CONTROL_FIXED;
    config.src_mode = DMA_HANDSHAKE_MODE_HANDSHAKE;
    config.dst_addr = core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)dst);
    config.dst_width = DMA_TRANSFER_WIDTH_BYTE; /*  In DMA handshake case, source width and destination width must be BYTE. */
    config.dst_addr_ctrl = DMA_ADDRESS_CONTROL_INCREMENT;
    config.dst_mode = DMA_HANDSHAKE_MODE_NORMAL;
    config.size_in_byte = size;
    config.src_burst_size = DMA_NUM_TRANSFER_PER_BURST_1T; /*  In DMA handshake case, source burst size must be 1 transfer, that is 0. */
    config.linked_ptr = core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)&uart_dbg_rx_descriptors[1]);
    stat = dma_config_linked_descriptor(dma_ptr, &uart_dbg_rx_descriptors[0], BOARD_COM_MA_UART_RX_DMA_CHN, &config);
    if (stat != status_success) {
        while (1) {
        };
    }

    config.linked_ptr = core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)&uart_dbg_rx_descriptors[0]);
    stat = dma_config_linked_descriptor(dma_ptr, &uart_dbg_rx_descriptors[1], BOARD_COM_MA_UART_RX_DMA_CHN, &config);
    if (stat != status_success) {
        while (1) {
        };
    }

    /* 1.2 config rx dma */
    config.linked_ptr = core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)&uart_dbg_rx_descriptors[0]);
    stat = dma_setup_channel(dma_ptr, BOARD_COM_MA_UART_RX_DMA_CHN, &config, true);
    if (stat != status_success) {
        while (1) {
        };
    }
}

void bsp_dma_set_uart_dbg_type(UART_Type *uart_ptr)
{
    uart_dbg_type = uart_ptr;
}

void bsp_dma_set_uart_com_ma_type(UART_Type *uart_ptr)
{
    uart_com_ma_type = uart_ptr;
}

uint8_t* bsp_dma_get_uart_dbg_tx_pbuf(void)
{
    return uart_dbg_tx_dma_buf;
}

uint8_t* bsp_dma_get_uart_com_ma_tx_pbuf(void)
{
    return uart_com_ma_tx_dma_buf;
}