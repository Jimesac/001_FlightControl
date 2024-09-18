#ifndef __BSP_PWM_H
#define __BSP_PWM_H

#include "hpm_soc.h"
#include "hpm_gpio_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_pwm_drv.h"
#include "hpm_interrupt.h"

#define PWM_UPDATE_FREQ        (40000)


#define BOARD_YAW_PWM_1_GPIO_PIN   IOC_PAD_PY00
#define BOARD_YAW_PWM_2_GPIO_PIN   IOC_PAD_PY02
#define BOARD_YAW_PWM_3_GPIO_PIN   IOC_PAD_PY03
#define BOARD_YAW_PWM_1_GPIO_FUNC  IOC_PY00_FUNC_CTL_PWM0_P_0
#define BOARD_YAW_PWM_2_GPIO_FUNC  IOC_PY02_FUNC_CTL_PWM0_P_2
#define BOARD_YAW_PWM_3_GPIO_FUNC  IOC_PY03_FUNC_CTL_PWM0_P_3

#define BOARD_YAW_PWM_BASE        HPM_PWM0
#define BOARD_YAW_PWM_CLOCK_NAME  clock_mot0
#define BOARD_YAW_PWM_1_CH        (0)
#define BOARD_YAW_PWM_2_CH        (2)
#define BOARD_YAW_PWM_3_CH        (3)
#define BOARD_YAW_PWM_IRQ         IRQn_PWM0
#define BOARD_YAW_TRGM            HPM_TRGM0
#define BOARD_YAW_PWM_IRQ         IRQn_PWM0
#define BOARD_YAW_TRGM_PWM_OUTPUT TRGM_TRGOCFG_PWM0_SYNCI
#define BOARD_YAW_TRGM_PWM_INPUT  HPM_TRGM0_INPUT_SRC_PWM0_TRGO_0


void bsp_pwm_init(void);

#endif /* __BSP_TIMER_H */