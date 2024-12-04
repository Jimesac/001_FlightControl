#ifndef TEST_AHRS_H
#define TEST_AHRS_H

#include <stdint.h>
#include "imu.h"
#include "misc.h"
#include "user_config.h"
#include "hpm_math.h"

#define G_MS2_TO_STAND1    0.10204f   // 1/9.8
extern bool imu_data_update_flag;

#define AHRS_BASE_VECTICAL_TH    (0.5f)   // 基座竖直判断阈值，q14(0.5) = 8192, 0.5对应差不多60°
#define AHRS_BASE_HORIZONTAL_TH  (0.6f)   // 基座水平方位判断阈值，q14(0.6) = 9830

// 云台基座方位变量
typedef enum
{
    
    AHRS_BASE_VERT_UP = 0,  // 正挂。 
    AHRS_BASE_VERT_DOWN,    // 倒挂。
    AHRS_BASE_HOR_CAM_HOR,  // 竖拍。
    AHRS_BASE_HOR_CAM_VERT, // 俯拍。
    AHRS_BASE_ROLL_SPECIAL = 10,
    AHRS_BASE_PITCH_SPECIAL = 11,
    AHRS_BASE_YAW_SPECIAL = 12,
    AHRS_BASE_HOR_CAM_HOR01 = 21,    // 竖拍，镜头左倾朝前
    AHRS_BASE_HOR_CAM_HOR_V11 = 22,  // 竖拍，镜头左倾朝下
    AHRS_BASE_HOR_CAM_HOR_V21 = 23,  // 竖拍，镜头左倾朝上
    AHRS_BASE_HOR_CAM_HOR02 = 24,    // 竖拍，镜头右倾朝前
    AHRS_BASE_HOR_CAM_HOR_V12 = 25,  // 竖拍，镜头右倾朝下
    AHRS_BASE_HOR_CAM_HOR_V22 = 26,  // 竖拍，镜头右倾朝上
    
    AHRS_BASE_HOR_CAM_VERT1 = 31,   // 俯拍
    AHRS_BASE_HOR_CAM_VERT2 = 32,   // 仰拍
} ahrs_base_ori_t; // 云台底座姿态

// 互补滤波参数
typedef struct
{
    nonlinear_unit_t kp_params;
    nonlinear_unit_t ki_params;
    nonlinear_unit_t g_norm_scale;
    float kp;
    float ki;
    float ki_err_decay;
    float ki_decay;

    float err[3];
    float kp_output[3];
    float ki_output[3];
    float pi_output[3];

    float err_limit;
    float kp_limit;
    float ki_limit;
    float output_limit;
}ahrs_comp_params_t;

// 姿态误差
typedef struct
{
    float err_sum;
    float err_sum_th;
    uint16_t err_count;
    uint16_t err_num;
    bool err_flag;
}ahrs_err_status_t;

// 姿态
typedef struct
{
    bool init_finished_flag;            //初始化完成标志

    float euler_rad[3];     // 欧拉角   //欧拉角（单位：弧度）  S!
    trig_func_f_t euler_trigf[3];       //欧拉角的三角函数值    S!
    float euler_deg[3];     // 欧拉角   //欧拉角（单位：度）    S!
    float euler_deg_yaw_offset;         //偏航角偏移量
    float quaternion[4];    // 四元素
    float dcm_b2n[3][3]; // 机体坐标系到地球坐标系的转换矩阵
  
    // 根据MPU6500测量的原始数据，经过PI修正后得到的四元素和欧拉角，内部使用
    float euler_raw_rad[3];             //未经滤波或处理的欧拉角（单位：弧度）
    
    float dcm_ba2b[3][3];               //机体坐标系的姿态调整矩阵
    struct 
    {
        float base_dcmz[3];             //底座当前方向的 z 轴矢量
        ahrs_base_ori_t base_ori;       // 底座当前姿态
        ahrs_base_ori_t base_ori_last;  // 底座上一次的姿态
        bool base_init_done;            //底座初始化完成标志
        bool base_change_flag;          //底座姿态变化标志
		
        float base_dcmz_var[3];         //底座方向 z 轴矢量的变化量
    }base_state;
    bool base_ori_fixed;                // 底座姿态固定标志

    bool gimbal_lock_flag;   // 万向节标志，正常运行时不会出现此类情况。但是在校准时会出现，特殊处理
} ahrs_status_t;

//存储与惯性导航系统（AHRS，姿态航向参考系统）中的加速度补偿相关的数据和参数
typedef struct
{
    ahrs_comp_params_t accel_params;         // 加速度互补滤波参数

    float accel_filter_alpha;               // 加速度补偿的预分系数
    float accel_filter[3];                  // 滤波后的 3 轴加速度数据      S!
    
    float accel_norm;                       //加速度向量的模值（即归一化值），用于检测异常或姿态计算
    bool use_accel_flag;                    //是否使用加速度数据进行补偿
	bool user_flag;
    float accel_valid_limit[2];             //加速度向量模值的有效范围（例如：[下限, 上限]）
	
	float limit_scale;                      //比例缩放因子，用于调整加速度数据的限制范围

    float accel_comp_divison;               // 加速度补偿的预分系数 

    float ext_acc[3];                       // 外部加速度       S?
    float ext_acc_coef;                     //外部加速度的权重系数，控制其对最终计算结果的影响程度

    const float *gyro_deg;                  //指向陀螺仪角速度数据（单位：度每秒）
    const float *accel_g;                   //指向加速度数据（单位：g）

    float gyro_rad_comp[3];                 //补偿后的陀螺仪角速度（单位：弧度每秒）S!
    float err_sum_sq;                       // 姿态误差平方和
} ahrs_comp_t;

void ahrs_params_init(void);
void ahrs_init(void);

void ahrs_update_attitude(float samle_interval);
void ahrs_dcm_euler_update(void);

void ahrs_euler2quat(float euler_rad[3], float *quat);
void ahrs_euler2dcm(float euler_rad[3], float *dcm_b2n);
void ahrs_quat2euler(float quat[4], float *euler_rad);
void ahrs_dmcbn2euler(float *dcm_b2n, float *euler_rad, float *euler_deg);
void ahrs_quat2dcmbn(float quat[3], float *dcm_b2n);

void ahrs_set_dcm_cor(float Tb2n_e[3][3]);
void ahrs_set_correct_quat(float err_theta0, float err_theta1, float err_theta2);
void remo_ahrs_set_base_init_done(bool flag);
void ahrs_set_base_ori(ahrs_base_ori_t base_ori);

bool ahrs_get_init_finished_flag(void);
float ahrs_get_euler_axis_deg(uint8_t axis);
const float *ahrs_get_base_dcmz(void);
float ahrs_get_base_dcmz_var(uint8_t axis);
bool remo_ahrs_get_base_vert_flag(void);

const float *ahrs_get_dcm(void);
const float *ahrs_get_euler_deg_ptr(void);
const float *ahrs_get_accel_ext(void);
const trig_func_f_t *remo_ahrs_get_euler_trig(uint8_t index);
float ahrs_get_accel_norm(void);
ahrs_base_ori_t ahrs_get_base_ori(void);
bool ahrs_get_base_vert_flag(void);
float ahrs_get_ahrs_err_sum_sq(void);
void remo_ahrs_set_base_ori_fixed_flag(bool flag);
void remo_ahrs_set_comp_filter_limit(float limit);

void ahrs_set_gimbal_lock_flag(bool flag);

int16_t remo_ahrs_get_roll_euler_cdeg(void);
int16_t remo_ahrs_get_pitch_euler_cdeg(void);
int16_t remo_ahrs_get_yaw_euler_cdeg(void);
uint8_t remo_ahrs_get_comp_filter_params(uint8_t *pbuf);
void remo_ahrs_set_comp_fliter_params(const uint8_t *pbuf);


#endif // TEST_AHRS_HTEST_AHRS_H
