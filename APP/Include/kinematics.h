#ifndef REMO_KINEMATICS_H
#define REMO_KINEMATICS_H

#include "fixed_point.h"
#include "misc.h"

#define GIMBAL_LCOK_VEL_SIGN_TH       (60)
#define GIMBAL_LCOK_GYRO_STATIC_TH    (100)

// 速度插值方式
typedef enum
{
    MOTION_STEP = 0, // 阶跃插值
    MOTION_LINEAR,   // 线性插值
    MOTION_EXPONENT, // 指数插值
    MOTION_IM_BUFF
} motion_im_t; // interpolation method

typedef struct
{
    // YAW轴欧拉角和Z轴关节角不使用
    const trig_func_f_t *euler_trig[3]; // 欧拉角三角函数值
    const trig_func_f_t *joint_trig[3]; // 关节角三角函数值

    float b2j_matrix[9]; // 机体坐标系映射到关节坐标系
	float j2b_matrix[9]; // 关节坐标系射到机体坐标系映
    float e2b_matrix[9]; // 欧拉角误差映射到机体坐标系
    float b2e_matrix[9]; // 机体坐标系映射欧拉角坐标系
    float n2b_matrix[9]; // ZYX坐标系映射到机体坐标系
    float b2n_matrix[9]; // 机体坐标系映射到ZYX坐标系
    float n2j_matrix[9]; // ZYX坐标系映射到关节角坐标系
    float e2j_matrix[9]; // 欧拉角坐标系到关节角坐标系
} motion_matrix_t;

typedef struct
{
    float vel_gyro_smp[3];
    float joint_vel_mea[3];
	float joint_vel_mea_filter_sum[3];
	
	uint16_t joint_vel_mea_filter_cnt[3];

    float vel_ref[3];       // 当前时刻的参考角速度，ZYX坐标系下
    float vel_aim[3];       // 目标角速度，ZYX坐标系下
    float joint_vel_ref[3]; // 关节角处的参考角速度
	
//	    // 斜率定义：如果斜率等于1，表示1秒钟速度变化360度
//    int16_q14_t velo_slope[3];        // 角速度变化斜率（指令）
//    int16_q14_t velo_slope_max[3];    // 最大角速度变化斜率（指令）
//    int16_q14_t velo_delta_pc_deg[3]; // 每个开关周期速度变化的大小（指令），pc->per cycle

    struct{
        motion_im_t ip_method;   // 速度插值方法
        float vel_delta_pc[3]; 

        nonlinear_unit_t vel_nl_uint;

        float vel_nl_coe[3];  // 非线性参数
        float scurve_shrink[3];    // 非线性值的比例缩小因子

    }ip_params;   // 速度插值参数

    struct{
        uint32_t vel_duration[3];
        uint32_t vel_cnt[3];

        uint32_t vel_start_end_time_num[3];
        float vel_start[3];
    }ip_vars;   // 速度插值变量

    bool  motionless_flag;
    bool vel_special_cal_flag;
}vel_status_t;

void remo_kinematics_params_init(void);
void remo_vel_ref_update(void);

void remo_kinematics_b2j_vel_map(void);

void remo_kinematics_matrix_calculation(void);

void remo_kinematics_set_vel_aim(float vel_aim, uint8_t index);
void remo_kinematics_set_vel_start(float vel, uint8_t index);
void remo_kinematics_reset_vel_ref(uint8_t index);
void remo_kinematics_set_vel_duration_cnt(uint32_t duration, uint32_t num, uint8_t index);

void remo_kinematics_set_joint_vel_ref(float vel_ref, uint8_t index);

void remo_kinematics_set_vel_ip_method(motion_im_t method);

void remo_kinematics_set_vel_spcl_motionless(bool flag);
void remo_kinematics_set_vel_cal_way(bool flag);
void remo_kinematics_set_gyro_bias_motion_fuse_flag(uint8_t flag0, uint8_t flag1, uint8_t flag2);

float remo_kinematics_get_vel_delta_pc(uint8_t index);
float remo_kinematics_get_joint_vel_smp_filter(uint8_t index);

const float *remo_kinematics_get_joint_vel_smp(void);
const float *remo_kinematics_get_joint_vel_ref(void);
const float *remo_kinematics_get_joint_vel_smp(void);
const float *remo_kinematics_get_joint_vel_ref(void);
int16_t remo_kinematics_get_single_joint_vel_smp(uint8_t index);

const motion_matrix_t *remo_kinematics_get_motion_matrix(void);

void remo_kinematics_reset_vel_ref_and_ip_var(uint8_t index);

vel_status_t *remo_kinematics_get_vel_status(void);

int16_t remo_kinematics_get_roll_vel_nl_coe(void);
int16_t remo_kinematics_get_pitch_vel_nl_coe(void);
int16_t remo_kinematics_get_yaw_vel_nl_coe(void);
		
int16_t remo_kinematics_get_roll_joint_vel_smp_ddps(void);
int16_t remo_kinematics_get_pitch_joint_vel_smp_ddps(void);
int16_t remo_kinematics_get_yaw_joint_vel_smp_ddps(void);
int16_t remo_kinematics_get_roll_ref_vel_ddps(void);
int16_t remo_kinematics_get_pitch_ref_vel_ddps(void);
int16_t remo_kinematics_get_yaw_ref_vel_ddps(void);
int16_t remo_kinematics_get_roll_joint_vel_fit_ddps(void);
int16_t remo_kinematics_get_yaw_joint_vel_fit_ddps(void);
#endif // REMO_KINEMATICS_H
