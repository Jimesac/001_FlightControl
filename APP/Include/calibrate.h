#ifndef __CALIBRATE_H
#define __CALIBRATE_H

#include "fixed_point.h"
#include "user_config.h"
#include <stdbool.h>

#define GRAVITY 8192

#define JOINT_DATA_FIT_SIZE   (120)

#pragma pack(1) // 按字节对齐
typedef enum
{
    CALI_STATE_IDLE = 0,         // 空闲状态，没有校准
    CALI_STATE_MOTOR_ALIGN,      // 电机电角度对齐
    CALI_STATE_DYNAMIC,          // 动态校准，椭球拟合，装配误差等校准
    CALI_STATE_GYROTRANS,        // 陀螺仪转换矩阵校准
    CALI_STATE_USER_ACCELTRANS,  // 加速度转换矩阵校准
    // CALI_STATE_STATIC,        // 静态校准，云台处于静止水平状态，校准陀螺仪加速度、角速度等
    CALI_STATE_ROLL_FT,          // roll轴微调
    CALI_STATE_YAW_INIT,         // yaw轴初始位置校准
    CALI_STATE_USER_MOTOR_ALIGN,
    CALI_ROLL_HALL_LINER = 8,
    CALI_PITCH_HALL_LINER,
    CALI_YAW_HALL_LINER,
    CALI_ROLL_HALL_FIT = 11,
    CALI_PITCH_HALL_FIT,
    CALI_YAW_HALL_FIT,
    

    CALI_ERR_STRT,        // 云台校准错误分割线
    CALI_ERR_MOTOR_ALIGN, // 电机电角度对齐错误
    CALI_ERR_DYNAMIC,     // 动态校准错误
    CALI_ERR_GYROTRANS,   // 陀螺仪转换矩阵校准错误
    CALI_ERR_ACCELTRANS,  // 加速度转换矩阵校准错误
    CALI_ERR_ROLL_FT,     // ROLL轴微调错误
    CALI_ERR_YAW_INIT,    // yaw轴初始位置校准错误
    CALI_ERR_USER_MOTOR_ALIGN,
    CALI_ERR_ROLL_HALL_LINER,
    CALI_ERR_PITCH_HALL_LINER,
    CALI_ERR_YAW_HALL_LINER,
    CALI_ERR_ROLL_HALL_FIT,
    CALI_ERR_PITCH_HALL_FIT,
    CALI_ERR_YAW_HALL_FIT,
    CALI_STATE_BUTT,
} cali_state_t;

typedef enum {
    EL_ALIGNMENT_PREPARE = 0,
    EL_MOTOR_EL_ALIGMENT,
    EL_GYRO_BIAS,
    EL_DATA_REFLASH,
    EL_ATTI_CONVERGE_CHECK,
    EL_ALIGNMENT_FINISH,
    EL_ALIGNMENT_WAIT,
    ERR_EL_MOTOR_ERR,
    ERR_EL_MOTOR_ROLL_ALIGMENT,
    ERR_EL_MOTOR_PITCH_ALIGMENT,
    ERR_EL_MOTOR_YAW_ALIGMENT,
    ERR_EL_MOTOR_CANNT_STOP,
    ERR_EL_MOTOR_CANNT_RUN,
    ERR_EL_ROLL_JOINT,
    ERR_EL_PITCH_JOINT,
    ERR_EL_YAW_JOINT,
    ERR_EL_GYRO_BIAS,
    ERR_EL_DATA_REFLASH,
    ERR_EL_ATTI_CONVERGE_CHECK,
    ERR_EL_IMU,
    ERR_EL_HALL_PITCH,
    ERR_EL_HALL_YAW,
    ERR_EL_END,
}elec_align_state_t;

typedef union{
    struct{
        uint32_t elec_angle_valid: 2;
        uint32_t joint_angle_valid: 2;
        uint32_t gyro_bias_valid: 2;
        uint32_t gyro_bias_heat_valid: 2;
        uint32_t accel_bias_valid: 2;
        uint32_t led_state: 2;
        uint32_t pcba_check_state: 2;   // 0:未检测，1:成功，2:失败
        uint32_t factory_test_state: 2; // 0:未检测，1:成功，2:失败
        uint32_t chasis_check_state:2;   // 0:未检测，1:成功，2:失败
        uint32_t reserved0: 2;
    }types;
    uint32_t which_type;
}cali_flag_t;  

typedef enum {
    JO_PREPARE = 0,
    JO_YAW_UPPER_LIMIT,
    JO_YAW_LOWER_LIMIT,
    JO_YAW_OFFSET,
    JO_PITCH_ROLL_OFFSET,
    JO_CALI_FINISH,
    ERR_JO_YAW_UPPER_LIMIT,
    ERR_JO_YAW_LOWER_LIMIT,
    ERR_JO_YAW_OFFSET,
    ERR_JO_PITCH_ROLL_OFFSET,
}joint_offset_state_t;

typedef enum {
    AC_FIT_PREPARE = 0,
    AC_ACCEL_SAMPLE,
    AC_LM_CAL,
    AC_MOTOR_JOINT_OFFSET,
    AC_DATA_REFLASH,
    AC_ATTI_CONVERGE_CHECK,
    AC_FIT_FINISH,
    ERR_AC_EL_NONFINISH,        // 电角度还没有对齐
    ERR_AC_PITCH_INIT,           // pitch无法准确到初始位置
    ERR_AC_ACCEL_SAMPLE_DATA,   // 不合格数据太多
    ERR_AC_ACCEL_SAMPLE_LONG,   // 采样时间太长
    ERR_AC_LM_CAL_DATA,    // 计算的校准参数异常
    ERR_AC_LM_CAL_LONG,    // 最小二乘计算时间太长
    ERR_AC_MOTOR_JOINT_OFFSET,
    ERR_AC_DATA_REFLASH,
    ERR_AC_ATTI_CONVERGE_CHECK,
}accel_fit_state_t;

typedef enum{
    JFIT_IDLE = 0,
    JFIT_EL_ALIGN,
    JFIT_INIT_POS,
    JFIT_LOWER_LIMIT,
    JFIT_UPPER_LIMIT,
    JFIT_SAVE_DATA,
    JFIT_FINISH,
    JFIT_TEST,
    JFIT_ERR,
    JFIT_ERR_EL_ALIGN,
    JFIT_ERR_INIT_POS,
    JFIT_ERR_LOWER_LIMIT,
    JFIT_ERR_UPPER_LIMIT,
    JFIT_ERR_SAVE_DATA,
}joint_data_fit_state_t;

//校准参数
typedef struct{             
    cali_flag_t cali_flag;            // 2个数据，标志位        
    uint32_t rsvd_flag;
    float joint_angle_offset[3];   // 3个数据，磁编码零偏
    int32_t roll_el_angle_offset;  // 2个数据，roll轴电角度偏置
    int32_t pitch_el_angle_offset; // 2个数据，pitch轴电角度偏置
    int32_t yaw_el_angle_offset;   // 2个数据，yaw轴电角度偏置
    int32_t gyro_offset[3];        // 3个数据，室温陀螺仪零偏
    int32_t gyro_heat_offset[3];   // 3个数据，高温陀螺仪零偏
    float reserve1_data[4];         // 4个数据，保留
    float gyro_matrix[9];          // 9个数据，陀螺仪转换矩阵(室温或加热指定温度，根据实际校准的情况确定)
    int32_t accel_offset[3];       // 3个数据，加速度计零偏
    float accel_matrix[9];         // 9个数据，加速度计转换矩阵
    float roll_offset[2];          // 2个数据，roll横竖拍角度偏移
    float reserve2_data[12];         // 4个数据，保留
    uint32_t roll_linear_cnt;
    float roll_linear_buf[2][JOINT_DATA_FIT_SIZE];   
    uint32_t pitch_linear_cnt;
    float pitch_linear_buf[2][JOINT_DATA_FIT_SIZE];
    uint32_t yaw_linear_cnt;
    float yaw_linear_buf[2][JOINT_DATA_FIT_SIZE];
}cali_params_t;

//描述与机械关节相关的数据拟合或校正信息
 typedef struct {
    float *data_pbuf[2];                //拟合或校准过程中的数据
    bool init_flag;                     //初始化标志

    joint_data_fit_state_t jfit_state;
    uint32_t align_clock_0p1us[2];      // 拟合过程中对齐的时间戳
    uint32_t cnt;                       // 拟合或校准过程中的计数器
    
    float angle_th;                     // 拟合或校准过程中的角度阈值
    float delta_angle_th;               // 角度变化的阈值，用于判断关节角度的变化是否超出合理范围
    float angle_last;                   // 上一次的角度值
    
    float params[4];                    // 校准过程中的参数
}joint_data_fit_info_t;

#pragma pack() // 取消按字节对齐

typedef struct
{
    // MPU6500静态偏置校准
    uint8_t gyro_bias;          // 陀螺仪的偏置校准值
    uint8_t accel_bias;         // 加速度计的偏置校准值

    joint_offset_state_t joint_offset_state;    // 关节偏移的状态信息
    

    // 电机电角度对齐
    uint8_t roll_elec_angle;    //电机的横滚角度（电角度）校准值
    uint8_t pitch_elec_angle;   //电机的俯仰角度（电角度）校准值
    uint8_t yaw_elec_angle;     //电机的偏航角度（电角度）校准值

    elec_align_state_t elec_align_state;        // 电机电角度对齐状态
    accel_fit_state_t accel_fit_state;          // 加速度拟合状态

    uint8_t ahrs_convergence_valid;             // 姿态解算收敛标志

    uint8_t acc_bias_valid;  // 加速度校准值有效，即处于水平状态

    uint8_t mpu_accel_reset;                        // MPU6500加速度计复位标志
    
    bool cali_set_state_flag;                       // 校准设置标志
    uint8_t cali_test_state;                        // 校准测试状态
    cali_state_t cali_state;                        // 校准状态
    cali_state_t cali_state_last;                   // 校准状态上一次

    uint32_t cali_params_size;                          // 校准参数大小
} cali_status_t;

void remo_cali_init(void);

void remo_cali_set_cali_state(cali_state_t state);

void remo_cali_run(void);

void remo_cali_reflash_imu_gyro_offset(int16_t gyrox_offet, int16_t gyroy_offet, int16_t gyroz_offet);

uint8_t remo_cali_get_velo_euler_flag(void);
bool remo_cali_get_motor_run_cali_flag(void);
bool remo_cali_get_elec_angle_flag(void);
bool remo_cali_get_motor_joint_valid(void);
bool remo_cali_get_cali_executing(void);
bool remo_cali_get_accel_bias_valid(void);
void remo_cali_set_accel_bias_valid(uint8_t flag);

uint8_t remo_cali_get_cali_state_log(void);
bool remo_cali_get_cali_running(void);
bool remo_cali_get_user_accel_succeed_flag(void);

uint8_t remo_cali_get_user_accel_percentage(void);
uint8_t remo_cali_get_gyro_state(void);

cali_state_t remo_cali_get_cali_state(void);

bool remo_cali_get_cali_finish_flag(void);

bool remo_cali_get_mpu_shutdown_flag(void);
uint8_t remo_cali_get_pcba_check_state(void);
uint8_t remo_cali_get_fac_res_state(void);
uint8_t remo_cali_get_chasis_check_state(void);

cali_flag_t *remo_cali_get_cali_flag(void);
bool remo_cali_get_joint_angle_valid(void);

uint8_t remo_cali_get_elec_align_state(void);
uint8_t remo_cali_get_accel_fit_state(void);

uint8_t remo_cali_get_err_state(void);

uint16_t remo_cali_get_gyro_bias_var_sum(void);

void remo_cali_set_chasis_check_state(uint8_t state);
void remo_cali_set_pcba_check_state(uint8_t state);
void remo_cali_set_fac_res_state(uint8_t state);

uint8_t remo_cali_get_err_state(void);

uint8_t remo_cali_set_yaw_hall_calib_info(uint8_t index, int16_t cdeg);

#endif // __CALIBRATE_H
