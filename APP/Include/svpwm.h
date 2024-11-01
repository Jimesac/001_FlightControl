#ifndef __APP_SVPWM_H
#define __APP_SVPWM_H

#include <stdbool.h>
#include <stdint.h>
#include "hpm_soc.h"
#include "hpm_gpio_drv.h"

#define BOARD_MOTOR_DRIVER_GPIO_CTRL  HPM_GPIO0

#define BOARD_MOTOR_DRIVER_ENABLE_GPIO_PIN    IOC_PAD_PB00
#define BOARD_MOTOR_DRIVER_ENABLE_GPIO_FUNC   IOC_PB00_FUNC_CTL_GPIO_B_00
#define BOARD_MOTOR_DRIVER_ENABLE_GPIO_INDEX  GPIO_DI_GPIOB

#define BOARD_MOTOR_PITCH_DRIVER_FAULT_GPIO_PIN    IOC_PAD_PY00
#define BOARD_MOTOR_PITCH_DRIVER_FAULT_GPIO_FUNC   IOC_PY00_FUNC_CTL_GPIO_Y_00
#define BOARD_MOTOR_PITCH_DRIVER_FAULT_GPIO_INDEX  GPIO_DI_GPIOY

#define BOARD_MOTOR_ROLL_DRIVER_FAULT_GPIO_PIN    IOC_PAD_PA09
#define BOARD_MOTOR_ROLL_DRIVER_FAULT_GPIO_FUNC   IOC_PA09_FUNC_CTL_GPIO_A_09
#define BOARD_MOTOR_ROLL_DRIVER_FAULT_GPIO_INDEX  GPIO_DI_GPIOA


#define BOARD_MOTOR_YAW_DRIVER_FAULT_GPIO_PIN    IOC_PAD_PB01
#define BOARD_MOTOR_YAW_DRIVER_FAULT_GPIO_FUNC   IOC_PB01_FUNC_CTL_GPIO_B_01
#define BOARD_MOTOR_YAW_DRIVER_FAULT_GPIO_INDEX  GPIO_DI_GPIOB

void svpwm_init(void);
void svpwm_update(void);

#endif   // __APP_SVPWM_H
