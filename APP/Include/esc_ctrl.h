#ifndef REMO_ESC_CTRL_H
#define REMO_ESC_CTRL_H

#include "fixed_point.h"

#define MOTOR_TORQUE_LIMIT    (5000)
#define VOLT_DQ_LIMIT  ((uint16_t)20000)

#define ALIGNMENT_NUM1   (IMU_ODR__VEL_CTRL__FREQ * 3)
#define ALIGNMENT_NUM2   (IMU_ODR__VEL_CTRL__FREQ * 5) 
#define ALIGNMENT_NUM3   (ALIGNMENT_NUM2 + 40 * 128)

#define DELTA_DQ_TH ((int16_t)700)

#define ESC_CTRL_MOTOR_NUM  (3)

#define ESC_SPEED_CTRL

#pragma pack(1) // 按字节对齐

// ESC_FAULT_INFO
// 异常类型
typedef enum
{
    NO_FAULT = 0,   // 没有问题
    OVER_CURR,      // 过流错误
    OVER_OFFSET,    // 电流偏置错误
    DRIVER_ERROR,   // 驱动芯片错误
    OVER_TIME,      // 一段时间内未收到主控板指令
    AENCODER_ZERO,  // 磁编码数据一直为0
    MAGNET_WEAK,       // 磁钢弱
} fault_type_t;

// ESC_RUN_STATUS
typedef enum
{
    ESC_STOP = 0,
    ESC_IDLE,
    ESC_WAIT,
    ESC_ALIGNMENT,
    ESC_RUN,
    ESC_TEST,
    ESC_ERR,
}esc_state_t;

typedef enum
{
    EL_ALIGN_IDLE = 0,
    EL_ALIGN_RUN,
    EL_ALIGN_SUCCESS,
    EL_ALIGN_FAILURE,
} el_angle_align_state_t;

typedef enum 
{
    MOTOR_STOP = 0,
    MOTOR_RUN,
    MOTOR_ERR,
    MOTOR_OTHERS
}motor_state_t;

#pragma pack() // 取消按字节对齐

void remo_esc_ctrl_init(void);
void remo_esc_ctrl_update(void);

void remo_esc_set_torque_cmd(int16_t torque, uint8_t index);
void remo_esc_set_el_angle_ptr(const int16_t *ptr, uint8_t index);
void remo_esc_set_esc_status(esc_state_t state, uint8_t index);
void remo_esc_set_esc_align_already_flag(uint8_t flag, uint8_t index);
void remo_esc_set_el_angle_offset(int16_t offset, uint8_t index);
void remo_esc_set_pwm_output_ptr(uint16_t *ptr, uint8_t index);
void remo_esc_set_motors_stop(void);
void remo_esc_set_motors_run(void);

int16_t *remo_esc_get_el_angle_offset_ptr(uint8_t index);
int16_t remo_esc_get_el_angle_offset(uint8_t index);
fault_type_t remo_esc_get_fault_type(uint8_t index);
esc_state_t remo_esc_get_esc_status(uint8_t index);
uint8_t remo_esc_get_align_start_flag(uint8_t index);
uint8_t remo_esc_get_align_already_flag(uint8_t index);
motor_state_t remo_esc_get_motors_state(void);

#ifdef ESC_SPEED_CTRL
void esc_ctrl_set_angle_deg(float angle_deg, uint8_t index);
void esc_ctrl_set_angle_ctrl_flag(uint8_t flag, uint8_t index);
void esc_ctrl_set_angle_ctrl_state(uint8_t state, uint8_t index);
void esc_ctrl_set_rate_ref(float ref, uint8_t index);
#endif

#endif // REMO_ESC_CTRL_H
