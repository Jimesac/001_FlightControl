#ifndef __BSP_PWM_H
#define __BSP_PWM_H

#include "hpm_soc.h"
#include "hpm_gpio_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_pwm_drv.h"
#include "hpm_interrupt.h"

#define PWM_UPDATE_FREQ        (20000U)

#define BOARD_PWM_CLOCK_NAME  clock_mot0

#define BOARD_PITCH_PWM_BASE        HPM_PWM1

#define BOARD_PITCH_PWM_1_GPIO_PIN   IOC_PAD_PY03
#define BOARD_PITCH_PWM_2_GPIO_PIN   IOC_PAD_PY02
#define BOARD_PITCH_PWM_3_GPIO_PIN   IOC_PAD_PY01
#define BOARD_PITCH_PWM_1_GPIO_FUNC  IOC_PY03_FUNC_CTL_PWM1_P_7
#define BOARD_PITCH_PWM_2_GPIO_FUNC  IOC_PY02_FUNC_CTL_PWM1_P_6
#define BOARD_PITCH_PWM_3_GPIO_FUNC  IOC_PY01_FUNC_CTL_PWM1_P_5
#define BOARD_PITCH_PWM_1_CH        (7U)
#define BOARD_PITCH_PWM_2_CH        (6U)
#define BOARD_PITCH_PWM_3_CH        (5U)


#define BOARD_ROLL_YAW_PWM_BASE        HPM_PWM0

#define BOARD_ROLL_PWM_1_GPIO_PIN   IOC_PAD_PA27
#define BOARD_ROLL_PWM_2_GPIO_PIN   IOC_PAD_PA28
#define BOARD_ROLL_PWM_3_GPIO_PIN   IOC_PAD_PA29
#define BOARD_ROLL_PWM_1_GPIO_FUNC  IOC_PA27_FUNC_CTL_PWM0_P_3
#define BOARD_ROLL_PWM_2_GPIO_FUNC  IOC_PA28_FUNC_CTL_PWM0_P_4
#define BOARD_ROLL_PWM_3_GPIO_FUNC  IOC_PA29_FUNC_CTL_PWM0_P_5
#define BOARD_ROLL_PWM_1_CH        (3U)
#define BOARD_ROLL_PWM_2_CH        (4U)
#define BOARD_ROLL_PWM_3_CH        (5U)

#define BOARD_YAW_PWM_1_GPIO_PIN   IOC_PAD_PA26
#define BOARD_YAW_PWM_2_GPIO_PIN   IOC_PAD_PA25
#define BOARD_YAW_PWM_3_GPIO_PIN   IOC_PAD_PA24
#define BOARD_YAW_PWM_1_GPIO_FUNC  IOC_PA26_FUNC_CTL_PWM0_P_2
#define BOARD_YAW_PWM_2_GPIO_FUNC  IOC_PA25_FUNC_CTL_PWM0_P_1
#define BOARD_YAW_PWM_3_GPIO_FUNC  IOC_PA24_FUNC_CTL_PWM0_P_0
#define BOARD_YAW_PWM_1_CH        (2U)
#define BOARD_YAW_PWM_2_CH        (1U)
#define BOARD_YAW_PWM_3_CH        (0U)




void bsp_pwm_init(void);

void bsp_pwm_enable_all_pwm_output(void);
void bsp_pwm_disable_all_pwm_output(void);

void remo_pwm_set_pwm_dutycycle_ptr(uint16_t *ptr, uint8_t index);
uint32_t remo_pwm_get_pwm_reload_value(void);
#endif /* __BSP_TIMER_H */