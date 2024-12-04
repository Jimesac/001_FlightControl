#include "bsp_led.h"

void bsp_led_init(void)
{

    HPM_IOC->PAD[BOARD_LED1_GPIO_PIN].FUNC_CTL = BOARD_LED1_GPIO_FUNC;      // 配置为GPIO功能 PA02
    gpio_set_pin_output_with_initial(BOARD_LED_GPIO_CTRL,  BOARD_LED1_GPIO_INDEX, BOARD_LED1_GPIO_PIN, 1);  // 输出高电平

    BOARD_LED1_ON;         // 点亮LED (低电平点亮)
}
