#include "bsp_uart.h"
#include "hpm_debug_console.h"

void bsp_uart_init(void)
{
    console_config_t cfg;
    uint32_t freq = 0U;

    // GPIO初始化
    HPM_IOC->PAD[BOARD_DBG_UART_RX_GPIO_PIN].FUNC_CTL = BOARD_DBG_UART_RX_GPIO_FUNC;
    HPM_IOC->PAD[BOARD_DBG_UART_TX_GPIO_PIN].FUNC_CTL = BOARD_DBG_UART_TX_GPIO_FUNC;

    // 串口初始化
    clock_set_source_divider(BOARD_DBG_UART_CLK_NAME, clk_src_osc24m, 1);
    clock_add_to_group(BOARD_DBG_UART_CLK_NAME, 0);
    freq = clock_get_frequency(BOARD_DBG_UART_CLK_NAME);

    cfg.type = BOARD_CONSOLE_TYPE;
    cfg.base = (uint32_t)BOADR_DBG_UART_BASE;
    cfg.src_freq_in_hz = freq;
    cfg.baudrate = BOARD_DBG_UART_BAUDRATE;

    if (status_success != console_init(&cfg)) {
        /* failed to  initialize debug console */
        while (1) {
        }
    }
}