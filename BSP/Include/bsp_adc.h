#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include "hpm_soc.h"
#include "hpm_gpio_drv.h"
#include "hpm_ioc_regs.h"
#include "hpm_clock_drv.h"
#include "hpm_adc16_drv.h"

#define BOARD_ADC16_VERSION_BASE     HPM_ADC0
#define BOARD_ADC16_VERSION_IRQn     IRQn_ADC0
#define BOARD_ADC16_VERSION_CH       (6U)
#define BOARD_ADC16_VERSION_CLK_NAME (clock_adc0)

#define BOARD_ADC16_VERSION_GPIO_PIN   IOC_PAD_PB14
#define BOARD_ADC16_VERSION_GPIO_FUNC  IOC_PAD_FUNC_CTL_ANALOG_MASK


#define BOARD_ADC16_VBUS_BASE     HPM_ADC0
#define BOARD_ADC16_VBUS_IRQn     IRQn_ADC0
#define BOARD_ADC16_VBUS_CH       (1U)
#define BOARD_ADC16_VBUS_CLK_NAME (clock_adc0)

#define BOARD_ADC16_VBUS_GPIO_PIN   IOC_PAD_PB09
#define BOARD_ADC16_VBUS_GPIO_FUNC  IOC_PAD_FUNC_CTL_ANALOG_MASK

void bsp_adc_init(void);


#endif /* __BSP_LED_H */