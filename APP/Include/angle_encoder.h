#ifndef __APP_ANGLE_ENCODER_H
#define __APP_ANGLE_ENCODER_H

#include <stdint.h>
#include "misc.h"
#include "fixed_point.h"
#include "hpm_math.h"
#include "filter.h"

typedef struct
{
    bool data_update;           // 霍尔传感器的数据是否已经更新

    float raw_data;             // 霍尔传感器原始数据
    float raw_data_last;        // 霍尔传感器上一次的原始数据
    float fp_alpha;             // 滤波器的平滑因子，用于低通滤波计算
    float raw_data0_offset;     // 霍尔传感器原始数据偏移量
    float raw_data_new;         // 霍尔传感器滤波后的数据       S?
    uint16_t hall_offset[2];    // 霍尔传感器偏移量
    
    bool linear_flag;           // 线性拟合标志
    uint16_t linear_num;        // 线性化补偿点的数量，用于插值计算
    float *linear_pbuf[2];      // 指向线性化补偿的数据缓冲区，存储补偿前和补偿后的数据


    int32_q7_t rotor_angle;     //原始数据转化为度 将霍尔传感器的原始数据转换为角度值

    low_pass_filter_2nd_t lp_2nd_filter;    // 二阶低通滤波器，用于对霍尔传感器数据进行滤波，去除高频噪声
    // int32_t speed_dps_filter;
    //low_pass_filter_2nd_fp_t speed_dps_lp_2nd;
}hall_info_t;

typedef struct
{
    float joint_deg[3];             // 关节的角度值
    int32_t joint_deg_raw[3];       // 原始的关节角度值
    int32_t joint_deg_offset[3];    // 关节角度的偏移量
    trig_func_f_t joint_trig[3];    // 三角函数值 
}joint_state_t;

void angle_encoder_init(void);

void angle_encoder_update(void);

void remo_encoder_set_joint_use_euler_flag(bool flag, uint8_t index);

void remo_encoder_set_hall_linear_pbuf(float *pbuf1, float *pbuf2, uint8_t index);
void remo_encoder_set_hall_linear_flag(bool flag, uint8_t index);
void remo_encoder_set_hall_linear_num(uint16_t num, uint8_t index);
void remo_encoder_set_hall_raw_data0_offset(float offset, uint8_t index);
void remo_encoder_set_joint_deg_offset(float offset, uint8_t index);

float remo_encoder_get_hall_raw_data(uint8_t index);

const float *remo_encoder_get_joint_deg_ptr(void);
const trig_func_f_t *remo_encoder_get_joint_trig(uint8_t type);

#endif /* __APP_ANGLE_ENCODER_H */
