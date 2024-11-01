#include "bsp_uart.h"
#include "bsp_dma.h"
#include <stdio.h>

typedef struct{
    uint8_t *tx_pbuf[UART_TX_MESSAGE_QUEUES_NUM];
    uint16_t tx_buf_len[UART_TX_MESSAGE_QUEUES_NUM];
    uint8_t tx_index;
    uint8_t tx_queue_index;
}uart_info_t; 

ATTR_PLACE_AT_NONCACHEABLE_BSS_WITH_ALIGNMENT(4) uint8_t uart_dbg_tx_buf[UART_TX_MESSAGE_QUEUES_NUM][UART_TX_BUF_SIZE];
ATTR_PLACE_AT_NONCACHEABLE_BSS_WITH_ALIGNMENT(4) uint8_t uart_com_ma_tx_buf[UART_TX_MESSAGE_QUEUES_NUM][UART_TX_BUF_SIZE];

uart_info_t uart_dbg_info = 
{
    .tx_pbuf = {(void *)0},
    .tx_buf_len = {0},
    .tx_index = 0,
    .tx_queue_index = 0,
 
};

uart_info_t uart_com_ma_info = 
{
    .tx_pbuf = {(void *)0},
    .tx_buf_len = {0},
    .tx_index = 0,
    .tx_queue_index = 0,
 
};

static void bsp_uart_dbg_init(void);
static void bsp_uart_com_ma_init(void);

void bsp_uart_init(void)
{
    uint8_t i = 0;

    for (i = 0; i < UART_TX_MESSAGE_QUEUES_NUM; i++)
    {
        uart_dbg_info.tx_pbuf[i] = &uart_dbg_tx_buf[i][0];
    }

      for (i = 0; i < UART_TX_MESSAGE_QUEUES_NUM; i++)
    {
        uart_com_ma_info.tx_pbuf[i] = &uart_com_ma_tx_buf[i][0];
    }

    bsp_uart_dbg_init();
    bsp_uart_com_ma_init();
}

static void bsp_uart_dbg_init(void)
{
    uart_config_t config;
    hpm_stat_t stat;
    uint32_t freq = 0U;

    // GPIO初始化
    HPM_IOC->PAD[BOARD_DBG_UART_RX_GPIO_PIN].FUNC_CTL = BOARD_DBG_UART_RX_GPIO_FUNC;
    HPM_IOC->PAD[BOARD_DBG_UART_TX_GPIO_PIN].FUNC_CTL = BOARD_DBG_UART_TX_GPIO_FUNC;

    // 串口初始化
    clock_set_source_divider(BOARD_DBG_UART_CLK_NAME, clk_src_osc24m, 1);
    clock_add_to_group(BOARD_DBG_UART_CLK_NAME, 0);
    freq = clock_get_frequency(BOARD_DBG_UART_CLK_NAME);

    // 串口默认设置
    config.baudrate = BOARD_DBG_UART_BAUDRATE;
    config.word_length = word_length_8_bits;
    config.parity = parity_none;
    config.num_of_stop_bits = stop_bits_1;
    config.fifo_enable = true;
    config.rx_fifo_level = uart_rx_fifo_trg_not_empty;
    config.tx_fifo_level = uart_tx_fifo_trg_not_full;
    config.dma_enable = false;
    config.modem_config.auto_flow_ctrl_en = false;
    config.modem_config.loop_back_en = false;
    config.modem_config.set_rts_high = false;

    config.rxidle_config.detect_enable = false;
    config.rxidle_config.detect_irq_enable = false;
    config.rxidle_config.idle_cond = uart_rxline_idle_cond_rxline_logic_one;
    config.rxidle_config.threshold = 10; /* 10-bit for typical UART configuration (8-N-1) */

    config.txidle_config.detect_enable = false;
    config.txidle_config.detect_irq_enable = false;
    config.txidle_config.idle_cond = uart_rxline_idle_cond_rxline_logic_one;
    config.txidle_config.threshold = 10; /* 10-bit for typical UART configuration (8-N-1) */

    config.rx_enable = true;
    
    // 串口配置
    config.fifo_enable = true;
    config.dma_enable = true;
    config.src_freq_in_hz = freq;
    config.tx_fifo_level = uart_tx_fifo_trg_not_full;
    config.rx_fifo_level = uart_rx_fifo_trg_not_empty;
    stat = uart_init(BOADR_DBG_UART_BASE, &config);
    if (stat != status_success) {
        printf("failed to initialize uart\n");
        while (1) {
        }
    }
  
    bsp_dma_set_uart_dbg_type(BOADR_DBG_UART_BASE);
}

static void bsp_uart_com_ma_init(void)
{
    uart_config_t config;
    hpm_stat_t stat;
    uint32_t freq = 0U;

    // GPIO初始化
    HPM_IOC->PAD[BOARD_COM_MA_UART_RX_GPIO_PIN].FUNC_CTL = BOARD_COM_MA_UART_RX_GPIO_FUNC;
    HPM_IOC->PAD[BOARD_COM_MA_UART_TX_GPIO_PIN].FUNC_CTL = BOARD_COM_MA_UART_TX_GPIO_FUNC;

    // 串口初始化
    clock_set_source_divider(BOARD_COM_MA_UART_CLK_NAME, clk_src_osc24m, 1);
    clock_add_to_group(BOARD_COM_MA_UART_CLK_NAME, 0);
    freq = clock_get_frequency(BOARD_COM_MA_UART_CLK_NAME);

    // 串口默认设置
    config.baudrate = BOARD_COM_MA_UART_BAUDRATE;
    config.word_length = word_length_8_bits;
    config.parity = parity_none;
    config.num_of_stop_bits = stop_bits_1;
    config.fifo_enable = true;
    config.rx_fifo_level = uart_rx_fifo_trg_not_empty;
    config.tx_fifo_level = uart_tx_fifo_trg_not_full;
    config.dma_enable = false;
    config.modem_config.auto_flow_ctrl_en = false;
    config.modem_config.loop_back_en = false;
    config.modem_config.set_rts_high = false;

    config.rxidle_config.detect_enable = false;
    config.rxidle_config.detect_irq_enable = false;
    config.rxidle_config.idle_cond = uart_rxline_idle_cond_rxline_logic_one;
    config.rxidle_config.threshold = 10; /* 10-bit for typical UART configuration (8-N-1) */

    config.txidle_config.detect_enable = false;
    config.txidle_config.detect_irq_enable = false;
    config.txidle_config.idle_cond = uart_rxline_idle_cond_rxline_logic_one;
    config.txidle_config.threshold = 10; /* 10-bit for typical UART configuration (8-N-1) */

    config.rx_enable = true;
    
    // 串口配置
    config.fifo_enable = true;
    config.dma_enable = true;
    config.src_freq_in_hz = freq;
    config.tx_fifo_level = uart_tx_fifo_trg_not_full;
    config.rx_fifo_level = uart_rx_fifo_trg_not_empty;
    stat = uart_init(BOADR_COM_MA_UART_BASE, &config);
    if (stat != status_success) {
        printf("failed to initialize uart\n");
        while (1) {
        }
    }
  
    bsp_dma_set_uart_com_ma_type(BOADR_COM_MA_UART_BASE);
}

void bsp_uart_dbg_schedule_transmit(void)
{
    uint8_t* dma_pbuf;


    if (uart_dbg_tx_dma_done)
    {
        dma_pbuf = bsp_dma_get_uart_dbg_tx_pbuf();

        //memcpy(bsp_dma_get_uart_tx_pbuf(), uart_dbg_info.tx_pbuf[uart_dbg_info.tx_index], 
        //      uart_dbg_info.tx_buf_len[uart_dbg_info.tx_index]);
        for(uint16_t i = 0; i < uart_dbg_info.tx_buf_len[uart_dbg_info.tx_index]; i++)
        {
            dma_pbuf[i] = uart_dbg_info.tx_pbuf[uart_dbg_info.tx_index][i];
        }
        
        uart_debug_tx_trigger_dma(BOARD_APP_HDMA, uart_dbg_info.tx_buf_len[uart_dbg_info.tx_index]);

        if (++uart_dbg_info.tx_index >= UART_TX_MESSAGE_QUEUES_NUM) 
        {
            uart_dbg_info.tx_index = 0;
        }

        uart_dbg_tx_dma_done = false;
    }
}

void bsp_uart_com_ma_schedule_transmit(void)
{
    uint8_t* dma_pbuf;


    if (uart_com_ma_tx_dma_done)
    {
        dma_pbuf = bsp_dma_get_uart_com_ma_tx_pbuf();

        //memcpy(bsp_dma_get_uart_tx_pbuf(), uart_com_ma_info.tx_pbuf[uart_com_ma_info.tx_index], 
        //      uart_com_ma_info.tx_buf_len[uart_com_ma_info.tx_index]);
        for(uint16_t i = 0; i < uart_com_ma_info.tx_buf_len[uart_com_ma_info.tx_index]; i++)
        {
            dma_pbuf[i] = uart_com_ma_info.tx_pbuf[uart_com_ma_info.tx_index][i];
        }
        
        uart_debug_tx_trigger_dma(BOARD_APP_HDMA, uart_com_ma_info.tx_buf_len[uart_com_ma_info.tx_index]);

        if (++uart_com_ma_info.tx_index >= UART_TX_MESSAGE_QUEUES_NUM) 
        {
            uart_com_ma_info.tx_index = 0;
        }

        uart_com_ma_tx_dma_done = false;
    }
}

uint8_t* bsp_uart_dbg_get_tx_pbuf(void)
{
    return uart_dbg_info.tx_pbuf[uart_dbg_info.tx_queue_index];
}

uint8_t* bsp_uart_com_ma_get_tx_pbuf(void)
{
    return uart_com_ma_info.tx_pbuf[uart_com_ma_info.tx_queue_index];
}

void bsp_uart_dbg_set_tx_size(uint16_t len)
{
    uart_dbg_info.tx_buf_len[uart_dbg_info.tx_queue_index] = len;

    if (++uart_dbg_info.tx_queue_index >= UART_TX_MESSAGE_QUEUES_NUM) 
    {
        uart_dbg_info.tx_queue_index = 0;
    }
}

void bsp_uart_com_ma_set_tx_size(uint16_t len)
{
    uart_com_ma_info.tx_buf_len[uart_com_ma_info.tx_queue_index] = len;

    if (++uart_com_ma_info.tx_queue_index >= UART_TX_MESSAGE_QUEUES_NUM) 
    {
        uart_com_ma_info.tx_queue_index = 0;
    }
}

