#ifndef __APP_USER_CONFIG_H
#define __APP_USER_CONFIG_H

#include <stdint.h>
#include "hpm_math.h"

#define XAXIS  ((uint8_t) 0)
#define YAXIS  ((uint8_t) 1)
#define ZAXIS  ((uint8_t) 2)

#define ROLL  ((uint8_t) 0)
#define PITCH  ((uint8_t) 1)
#define YAW    ((uint8_t) 2)

#define IMU_ODR__VEL_CTRL__FREQ  (3840)  // 960, 1920, 3840, 7680
#define IMU_ODR__VEL_CTRL__TIME  (1.0f/IMU_ODR__VEL_CTRL__FREQ)

#define SELTIMER_SCHEDULE_FREQ    (IMU_ODR__VEL_CTRL__FREQ)
#define SCHEDULE_SECTION_NUM  (6)
#define SCHEDULE_SECTION_FREQ (SELTIMER_SCHEDULE_FREQ/SCHEDULE_SECTION_NUM)
#define SCHEDULE_SECTION_TIME (1.0f/(float)SCHEDULE_SECTION_FREQ)

#define MOTOR_ROLL_TORQUE_LIMIT  (20000)
#define MOTOR_PITCH_TORQUE_LIMIT (21000)
#define MOTOR_YAW_TORQUE_LIMIT   (21000)

#define  GIMBALLOCK_ROLL_DEG   (60.0f)

#define IMU_ACCEL_SAMPLE_SCHD_INDEX  (1)
#define MAG_ENCODE_SAMPLE_SCHD_INDEX (0)

#define BOARD_PTMR_SCHEDULE_IRQ        IRQn_GPTMR2
#define BOARD_PTMR_SCHEDULE_IRQ_LEVEL  (20U)

#define BOARD_MOTOR_PITCH_DRIVER_FAULT_GPIO_IRQ        IRQn_GPIO0_Y
#define BOARD_MOTOR_PITCH_DRIVER_FAULT_GPIO_IRQ_LEVEL  (2U)

#define BOARD_MOTOR_ROLL_DRIVER_FAULT_GPIO_IRQ        IRQn_GPIO0_A
#define BOARD_MOTOR_ROLL_DRIVER_FAULT_GPIO_IRQ_LEVEL  (3U)

#define BOARD_MOTOR_YAW_DRIVER_FAULT_GPIO_IRQ        IRQn_GPIO0_B
#define BOARD_MOTOR_YAW_DRIVER_FAULT_GPIO_IRQ_LEVEL  (4U)

#define BOARD_ADC16_HALL_IRQn        IRQn_ADC1
#define BOARD_ADC16_HALL_IRQn_LEVEL  (5U)

#define BOARD_HDMA_IRQ        IRQn_HDMA
#define BOARD_HDMA_IRQ_LEVEL  (6U)

#define BOARD_PITCH_PWM_IRQ           IRQn_PWM1
#define BOARD_PITCH_PWM_IRQ_LEVEL     (10U)
#define BOARD_ROLL_YAW_PWM_IRQ        IRQn_PWM0
#define BOARD_ROLL_YAW_PWM_IRQ_LEVEL  (11U)


typedef enum{
    SCHEDULE_ATT_UPDATA = 0,
    SCHEDULE_POS_PLAN = 1,
    SCHEDULE_COM = 2,
    SCJEDULE_LOGGER = 3,
    SCJEDULE_RSVD0,
    SCJEDULE_RSVD1,
    SCJEDULE_RSVD2,
    SCJEDULE_RSVD3,
    SCJEDULE_RSVD4,
    SCJEDULE_RSVD5,
}schedule_state_t;

extern schedule_state_t schedule_ptmr_state;

#endif   // __APP_USER_CONFIG_H
