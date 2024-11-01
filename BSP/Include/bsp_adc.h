#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include "hpm_soc.h"
#include "hpm_gpio_drv.h"
#include "hpm_ioc_regs.h"
#include "hpm_clock_drv.h"
#include "hpm_adc16_drv.h"
#include "hpm_trgm_drv.h"

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

#define BOARD_ADC16_VBUS_GPIO_PIN   IOC_PAD_PB15
#define BOARD_ADC16_VBUS_GPIO_FUNC  IOC_PAD_FUNC_CTL_ANALOG_MASK



#define BOARD_ADC16_HALL_BASE     HPM_ADC1

#define BOARD_ADC16_HALL_P1_CH       (2U)
#define BOARD_ADC16_HALL_P2_CH       (3U)
#define BOARD_ADC16_HALL_R1_CH       (4U)
#define BOARD_ADC16_HALL_R2_CH       (5U)
#define BOARD_ADC16_HALL_CLK_NAME (clock_adc1)

#define BOARD_ADC16_HALL_P1_GPIO_PIN   IOC_PAD_PB10
#define BOARD_ADC16_HALL_P2_GPIO_PIN   IOC_PAD_PB11
#define BOARD_ADC16_HALL_R1_GPIO_PIN   IOC_PAD_PB12
#define BOARD_ADC16_HALL_R2_GPIO_PIN   IOC_PAD_PB13
#define BOARD_ADC16_HALL_GPIO_FUNC     IOC_PAD_FUNC_CTL_ANALOG_MASK

#define BOARD_ADC16_HALL_TRGM       HPM_TRGM0
#define BOARD_HALL_TRIGMUX_IN_NUM   HPM_TRGM0_INPUT_SRC_GPTMR2_OUT2
#define BOARD_HALL_TRG_NUM          TRGM_TRGOCFG_ADCX_PTRGI0A
#define BOARD_HALL_ADC_TRG          ADC16_CONFIG_TRG0A

#define BOARD_ADC16_HALL_PMT_IRQ_EVENT  adc16_event_trig_complete

void bsp_adc_init(void);

uint16_t *bsp_adc_get_hall_pitch_filter_ptr(void);
uint16_t *bsp_adc_get_hall_roll_filter_ptr(void);
#endif /* __BSP_LED_H */