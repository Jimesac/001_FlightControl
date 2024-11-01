#ifndef __APP_ANGLE_ENCODER_H
#define __APP_ANGLE_ENCODER_H

#include <stdint.h>
#include "misc.h"
#include "fixed_point.h"
#include "hpm_math.h"
#include "filter.h"

typedef struct
{
    bool data_update;

    float raw_data;
    float raw_data_last;
    float fp_alpha;
    float raw_data0_offset;
    float raw_data_new;
    uint16_t hall_offset[2];
    
    bool linear_flag;
    uint16_t linear_num;
    float *linear_pbuf[2];


    int32_q7_t rotor_angle; //原始数据转化为度

    low_pass_filter_2nd_t lp_2nd_filter;
    // int32_t speed_dps_filter;
    //low_pass_filter_2nd_fp_t speed_dps_lp_2nd;
}hall_info_t;

typedef struct
{
    float joint_deg[3];    // 角度
    int32_t joint_deg_raw[3];
    int32_t joint_deg_offset[3];
    trig_func_f_t joint_trig[3];   // 三角函数值 
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