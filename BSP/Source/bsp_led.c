#include "bsp_led.h"

void bsp_led_init(void)
{

    HPM_IOC->PAD[BOARD_LED1_GPIO_PIN].FUNC_CTL = BOARD_LED1_GPIO_FUNC;
    HPM_IOC->PAD[BOARD_LED2_GPIO_PIN].FUNC_CTL = BOARD_LED2_GPIO_FUNC;
    gpio_set_pin_output_with_initial(BOARD_LED_GPIO_CTRL,  BOARD_LED1_GPIO_INDEX, BOARD_LED1_GPIO_PIN, 1);
    gpio_set_pin_output_with_initial(BOARD_LED_GPIO_CTRL,  BOARD_LED2_GPIO_INDEX, BOARD_LED2_GPIO_PIN, 1);

    BOARD_LED1_ON;
    BOARD_LED2_ON;
}
