#ifndef __CTRL_LOOP_H
#define __CTRL_LOOP_H

#include "fixed_point.h"
#include "ctrl.h"
#include "misc.h"
#include "filter.h"

//#define VEL_CTRL_USE_NOTCH_FILTER

#define POS_CTRL_ROLL_KP    (10.0f)
#define POS_CTRL_PITCH_KP   (15.0f)
#define POS_CTRL_YAW_KP     (5.0f)

#define VEL_CTRL_ROLL_KP    (160.0f)
#define VEL_CTRL_PITCH_KP   (200.0f)
#define VEL_CTRL_YAW_KP     (200.0f)
#define VEL_CTRL_ROLL_KI    (6.5f)
#define VEL_CTRL_PITCH_KI   (10.0f)
#define VEL_CTRL_YAW_KI     (6.0f)
#define VEL_CTRL_ROLL_KD    (8.0f)
#define VEL_CTRL_PITCH_KD   (10.0f)
#define VEL_CTRL_YAW_KD     (10.5f)

typedef enum{
    POS_EULER_CTRL = 0,
    POS_JOINT_CTRL,
}pos_ctrl_type_t;

typedef struct
{
    ctrl_p_t ctrl_p;
    
    bool ctrl_enable;
    
    pos_ctrl_type_t ctrl_type;

    float pos_ref;
    float pos_mea;

    float ctrl_output;
}pos_ctrl_t;

typedef struct
{
    ctrl_pid_t ctrl_pid;
    notch_filter_2nd_t notch_filter[2];
    low_pass_filter_2nd_t lp_2nd_filter;

    ctrl_leadlag_t leadlag;

    bool ctrl_enable;
    bool kp_adaptive_flag;
    
    float vel_target;
    float vel_ref;
    float vel_mea;
	
    float ctrl_comp;
    float ctrl_comp_last;
    bool ctrl_comp_flag;

    float ctrl_output;
    float ctrl_output_last;
    float delta_ctrl_output_th;
}vel_ctrl_t;

extern bool pitch_yaw_cmp_flag[2];


void remo_ctrl_loop_params_init(void);
void remo_ctrl_loop_scale_pos_limit(float scale, uint8_t index);

void remo_ctrl_loop_pos_ctrl(void);
void remo_ctrl_loop_vel_ctrl(void);

void remo_ctrl_loop_set_pos_ref(float ref, uint8_t index);
void remo_ctrl_loop_set_vel_target(float ref, uint8_t index);
void remo_ctrl_loop_set_pos_mea(float mea, uint8_t index);
void remo_ctrl_loop_set_vel_mea(float mea, uint8_t index);

void remo_ctrl_loop_reset_roll_pos_pid(void);
void remo_ctrl_loop_reset_pitch_pos_pid(void);
void remo_ctrl_loop_reset_yaw_pos_pid(void);

void remo_ctrl_loop_reset_roll_vel_pid(void);
void remo_ctrl_loop_reset_pitch_vel_pid(void);
void remo_ctrl_loop_reset_yaw_vel_pid(void);
void remo_ctrl_loop_set_pitch_start_vel_pid(void);

void remo_ctrl_loop_set_torque_ctrl_flag(bool flag, uint8_t index);
void remo_ctrl_loop_set_yaw_pos_fixed_flag(bool flag);

void remo_ctrl_loop_set_pos_kp_scale(float scale, uint8_t index);
void remo_ctrl_loop_set_vel_kp_scale(float scale, uint8_t index);
void remo_ctrl_loop_set_vel_ki_scale(float scale, uint8_t index);
void remo_ctrl_loop_set_vel_kd_scale(float scale, uint8_t index);

void remo_ctrl_loop_set_pitch_vel_limit(uint16_t limit);
void remo_ctrl_loop_set_yaw_vel_limit(uint16_t limit);

void remo_ctrl_loop_set_yaw_gimballock_vel_pid(void);
void remo_ctrl_loop_set_roll_gimballock_vel_pid(void);

void remo_ctrl_loop_reset_roll_notch_filter(void);
void remo_ctrl_loop_reset_pitch_notch_filter(void);
void remo_ctrl_loop_reset_yaw_notch_filter(void);

void remo_ctrl_loop_set_pos_ctrl_enable_state(uint8_t state, uint8_t index);
void remo_ctrl_loop_set_pos_ctrl_type(pos_ctrl_type_t type, uint8_t index);
void remo_ctrl_loop_set_pos_ctrl(pos_ctrl_type_t type, float ref, uint8_t index);
void remo_ctrl_loop_set_angle_linear_calib_flag(bool flag, uint8_t index);
void remo_ctrl_loop_set_jacob_matrix_state(uint8_t state);

void remo_ctrl_loop_set_vel_ctrl_lpf(bool flag, uint8_t index);

float remo_ctrl_loop_get_pos_err(uint8_t index);

float remo_ctrl_loop_get_pos_output(uint8_t index);
int16_t remo_ctrl_loop_get_vel_output(uint8_t index);


void remo_ctrl_loop_get_vel_pidout_and_limit(int32_t *vel_output, uint8_t index);

int16_t remo_ctrl_loop_get_torque(uint8_t index);

float remo_ctrl_loop_get_pos_err(uint8_t index);

void remo_ctrl_loop_set_zero_torque(bool flag);

void remo_ctrl_loop_set_vel_ki_dec(float ki_dec, uint8_t index);

void remo_ctrl_loop_set_vel_kp_adaptive_flag(bool flag, uint8_t index);

void remo_ctrl_set_vel_test_state(uint8_t state);

void remo_ctrl_loop_reset_pitch_pos_pid(void);
void remo_ctrl_loop_set_pitch_pos_work1_pid(void);

float remo_ctrl_loop_get_vel_err(uint8_t index);
float remo_ctrl_loop_get_vel_delta_err(uint8_t index);
float remo_ctrl_loop_get_vel_sum_err(uint8_t index);
float remo_ctrl_loop_get_vel_sum_err_filter(uint8_t index);
float remo_ctrl_loop_get_vel_ctrl_comp(uint8_t index);

int16_t remo_ctrl_loop_get_roll_vel_output(void);
int16_t remo_ctrl_loop_get_pitch_vel_output(void);
int16_t remo_ctrl_loop_get_yaw_vel_output(void);

uint8_t remo_ctrl_loop_get_pos_kp_params(uint8_t *buf);
uint8_t remo_ctrl_loop_get_vel_pid_params(uint8_t *buf);
uint8_t remo_ctrl_loop_get_vel_leadlag_params(uint8_t *buf);
uint8_t remo_ctrl_loop_get_vel_2nd_lfp_params(uint8_t *buf);
uint8_t remo_ctrl_loop_get_vel_notch_filter_params(uint8_t *buf);
uint8_t remo_ctrl_loop_get_vel_diff_filter_params(uint8_t *buf);
uint8_t remo_ctrl_loop_get_vel_diff_track_params(uint8_t *buf);
uint8_t remo_ctrl_loop_get_cur_pid_params(uint8_t *buf);

#endif // __CTRL_LOOP_H
