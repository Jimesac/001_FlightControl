#ifndef __BSP_LED_H
#define __BSP_LED_H

#include "hpm_soc.h"
#include "hpm_gpio_drv.h"

#define BOARD_LED_GPIO_CTRL  HPM_GPIO0

#define BOARD_LED1_GPIO_PIN    IOC_PAD_PA02
#define BOARD_LED1_GPIO_FUNC   IOC_PA02_FUNC_CTL_GPIO_A_02
#define BOARD_LED1_GPIO_INDEX  GPIO_DI_GPIOA


#define BOARD_LED1_ON       gpio_write_pin(BOARD_LED_GPIO_CTRL, BOARD_LED1_GPIO_INDEX, BOARD_LED1_GPIO_PIN, 0);
#define BOARD_LED1_OFF      gpio_write_pin(BOARD_LED_GPIO_CTRL, BOARD_LED1_GPIO_INDEX, BOARD_LED1_GPIO_PIN, 1);
#define BOARD_LED1_TOGGLE   gpio_toggle_pin(BOARD_LED_GPIO_CTRL, BOARD_LED1_GPIO_INDEX, BOARD_LED1_GPIO_PIN);



void bsp_led_init(void);


#endif /* __BSP_LED_H */