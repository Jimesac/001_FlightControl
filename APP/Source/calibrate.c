#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "calibrate.h"
#include "imu.h"
#include "angle_encoder.h"
#include "esc_ctrl.h"
#include "ctrl_loop.h"
#include "bsp_flash.h"
#include "bsp_timer.h"
#include "ahrs.h"

#define ROLL_OFFSET_LIMIT 0

#define REMO_DEBUG_CALIBRATE 0

#define CALI_GYRO_TYEP 1
#define CALI_ACCEL_TYEP 2

// 以下参数根据IMU选取的量程进行调整
#define CALI_GYRO_VAR_TH 100     // 陀螺仪校准方差阈值
#define CALI_ACCEL_VAR_TH 20000   // 加速度计校准方差阈值
#define CALI_GYRO_AVG_DIFF_TH 4  // 陀螺仪校准两次平均值差阈值
#define CALI_ACCEL_AVG_DIFF_TH 50 // 陀螺仪校准两次平均值差阈值

#define CALI_STATIC_TIME_TH (2000000) // 静态校准最大时间


#ifdef ROLL_AXIS_EXISTENCE
#define CALI_FIT_LM_ITER_NUM 28            // 加速度计校准9参数模型LM迭代次数
#define ACCEL_FIT_SUBSMP_NUM 10             // 加速度计校准pitch转一圈采样个数
#define ACCEL_FIT_ROLLSMP_NUM 5             // 加速度计校准roll转动角度个数
#else
#define CALI_FIT_LM_ITER_NUM 18               // 加速度计校准9参数模型LM迭代次数
#define ACCEL_FIT_SUBSMP_NUM 18             // 加速度计校准pitch转一圈采样个数
#define ACCEL_FIT_ROLLSMP_NUM 2             // 加速度计校准roll转动角度个数，目的是检测两个方向pitch运动是否正常
#endif
#define ACCEL_FIT_SMP_NUM ACCEL_FIT_SUBSMP_NUM * ACCEL_FIT_ROLLSMP_NUM
#define ACCEL_FIT_AIM_ANGLE_DEG   80
#define ACCEL_FIT_DELTA_ANGLE_TH  (2*ACCEL_FIT_AIM_ANGLE_DEG / ACCEL_FIT_SUBSMP_NUM)
#define ACCEL_FIT_RANSAC_NUM    (30)

#define ACCEL_SMP_NORM2_TH 10000000
#define ACCEL_SMP_NORM     (8192.0f)


// LM算法参数
#define ACCEL_FIT_LM_LAMBDA 0.0001f
#define ACCEL_FIT_LM_ALPHA 0.04f


// 求关节角度平均值的计数值
#define JOINT_ANGLE_OFFSET_MEAN_NUM    ((uint8_t)128)
#define JOINT_ANGLE_OFFSET_MEAN_SBN    ((uint8_t)7)

#define YAW_LIMIT_ANGLE   (300.0f)

// 加速度计椭球拟合结构体（9参数模型参数）
// TODO 椭球拟合数据采样过程有待调整
typedef struct
{
    // 参数说明：[6]~[8]->偏置；[0]]~[2]->校准矩阵对角线元素；[3]~[5]->校准矩阵非对角线元素
    // 矩阵是对称矩阵：
    //    | params[0] params[3] params[4] |
    //    | params[3] params[1] params[5] |
    //    | params[4] params[5] params[2] |
    float params[9]; 

    float smp[ACCEL_FIT_SMP_NUM][3]; // 采样值
    int16_t smp_tmp[3];
    uint16_t smp_renew_cnt;
    uint16_t smp_cnt;
    uint8_t smp_index; // 当前采样索引
    uint8_t smp_valid; // 1->表示当前采样数据有效
    uint8_t iter;

    float f_sum;

    float RANSAC_residual_th;

    float rand_matrix[ACCEL_FIT_SMP_NUM];
    uint8_t select_index[ACCEL_FIT_SMP_NUM];
    uint8_t RANSAC_select_num;
    uint8_t RANSAC_iter;
    uint8_t RANSAC_iter_num;
    float RANSAC_th;
    float RANSAC_ratio;
    uint8_t RANSAC_k;
    uint8_t RANSAC_pretotal;
} cali_accel_t;

cali_accel_t cali_accel = {
    .params = {1.0f, 1.0f, 1.0f, 0, 0, 0, 0, 0, 0},
    .smp = {0},
    .smp_tmp = {0},
    .smp_renew_cnt = 0,
    .smp_cnt = 0,
    .smp_index = 0,
    .smp_valid = 0,
    .iter = 0,
    .f_sum = 0,
    .RANSAC_residual_th = 0.006f,//0.003f,
    .rand_matrix = {0},
    .select_index = {0},
    .RANSAC_select_num = 0,
    .RANSAC_iter = 0,
    .RANSAC_iter_num = ACCEL_FIT_RANSAC_NUM,
    .RANSAC_th = 0,
    .RANSAC_ratio = 58982,   // 90%
    .RANSAC_k = 0,
    .RANSAC_pretotal = 0,
};

// 陀螺仪和加速度计校准质量
// 校准值是否过大，太大说明imu质量不好
struct imu_cali_state_t
{
    uint8_t gyro_state;   // 1:均值大，2:第一次校准方差大
    uint8_t accel_state;  // 1:转换矩阵病态，2:长时间不过

    uint16_q7_t gyro_state_th;
    uint16_q7_t accel_state_th;
}imu_cali_state;

// 加速度计校准采样roll轴角度
int16_q7_t accel_cali_sample_roll_deg[ACCEL_FIT_ROLLSMP_NUM] = {0};


// TODO 以下校准过程，有待考评，后续再规范
// 陀螺仪9参数模型参数
uint16_q14_t cali_gyro_9params[9] = {CONST_1_Q14, 0, 0, 0, CONST_1_Q14, 0, 0, 0, CONST_1_Q14};


static bool remo_imu_bias_average(int32_t *avg, uint8_t pow2, uint8_t bias_type);
static bool remo_imu_bias_average_variance(int32_t *var, int32_t *avg_new, int32_t *avg, uint8_t pow2, uint8_t bias_type);

// 加速度计椭球拟合的数据采样过程
static void remo_cali_accel_cali_smp(int16_t *accel_smp);
static bool remo_cali_accel_fit_smp(int16_t accel_fit_smp[3]);
// 加速度计椭球拟合的数据拟合过程
static bool remo_cali_accel_cali_LM(float *accel_smp, uint8_t *accel_smp_index, uint8_t accel_smp_num);

// 校准二步骤：电机电角度对齐，动态校准
static void remo_cali_pitch_hall_linear(void);
static void remo_cali_yaw_hall_linear(void);
static void remo_cali_motor_alignment(void);
static void remo_cali_new_dynamic_state(void);

static void remo_cali_joint_linear(uint8_t axis_index);

// 角速度计、加速度计静态偏置校准
static void remo_cali_imu_gyro_bias(void);
static void remo_cali_imu_accel_bias(void);

static void remo_cali_gyro_trans(void);
static void remo_cali_user_accel_trans(void);

static void remo_cali_accel_trans_ctrl_params(void);

void remo_cali_reset_accel_params(void);

static bool remo_cali_roll_offset_by_limit(int16_t *roll_offset);

static void remo_cali_roll_ft(void);

static void remo_cali_factory_cali_finish(void);

static bool remo_cali_gyro_temp_bias(void);

static void remo_cali_yaw_initial_position(void);

static void remo_cali_init_joint_deg(void);


cali_params_t cali_params = {
    .cali_flag = 0,
    .joint_angle_offset = {0} ,
    .gyro_offset = {0},
    .gyro_heat_offset = {0},
    .gyro_matrix = {1.0f, 0, 0, 0, 1.0f, 0, 0, 0, 1.0f},
    .accel_offset = {0},
    .accel_matrix = {1.0f, 0, 0, 0, 1.0f, 0, 0, 0, 1.0f},
    .roll_offset = {0},
    .reserve1_data = {0},
    .roll_el_angle_offset = 0,
    .pitch_el_angle_offset = 0,
    .yaw_el_angle_offset = 0,
    .reserve2_data = {0},
    .roll_linear_cnt = 0,
    .pitch_linear_cnt = 0,
    .yaw_linear_cnt = 0,
    .roll_linear_buf = {0},
    .pitch_linear_buf = {0},
    .yaw_linear_buf = {0}
};

cali_status_t cali_status = {
    .gyro_bias = 0,
    .accel_bias = 0,
    .joint_offset_state = JO_PREPARE,
    .roll_elec_angle = 0,
    .pitch_elec_angle = 0,
    .yaw_elec_angle = 0,
    .elec_align_state = EL_ALIGNMENT_PREPARE,
    .accel_fit_state = AC_FIT_PREPARE,
    .ahrs_convergence_valid = 0,
    .acc_bias_valid = 0,
    .mpu_accel_reset = 0,
    .cali_set_state_flag = false,
    .cali_test_state = 0,
    .cali_state = CALI_STATE_IDLE,
    .cali_state_last = CALI_STATE_IDLE,
    .cali_params_size = 0
};

joint_data_fit_info_t joint_data_fit_info[3];

uint32_t new_dynamic_cnt = 0;
uint8_t gyro_motor_run_flag = 0;

uint8_t cali_reset_roll_ft_rad_flag = 0;
int16_q7_t cali_roll_ft_rad_temp[2] = {0};

uint8_t user_accel_cali_percentage = 0;

uint8_t accel_cali_start_flag = 0;

int16_t temperature_mean = 0;

int16_q7_t gyro_temp_bias[3] = {0};

uint8_t accel_smp_failure_flag = 0, accel_smp_failure_flag_last = 0;

bool yaw_motor_align_motion_flag = false;

int16_t gyro_rst_offset[3] = {0};


static void remo_cali_yaw_hall_calib(void);
static void remo_cali_yaw_data_fit(void);
static void remo_cali_accel_fit(uint8_t state);

/**************************************************************************************************
 * 函数名称: remo_cali_init
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 校准参数初始化，陀螺仪上电校准（角速度偏移）
**************************************************************************************************/
void remo_cali_init(void)
{
    cali_status.cali_params_size = sizeof(cali_params);
    if (cali_status.cali_params_size > USER_PARAMS_FLASH_WORDS_SIZE*4)
    {
        printf("Error: USER FLASH SIZE AND CALIBRATION PARAMS ERR!\n");
        while (1) {
        }
    }

    remo_encoder_set_hall_linear_pbuf(&cali_params.roll_linear_buf[0][0], &cali_params.roll_linear_buf[1][0], 0);
    remo_encoder_set_hall_linear_pbuf(&cali_params.pitch_linear_buf[0][0], &cali_params.pitch_linear_buf[1][0], 1);
    remo_encoder_set_hall_linear_pbuf(&cali_params.yaw_linear_buf[0][0], &cali_params.yaw_linear_buf[1][0], 2);

    joint_data_fit_info[ROLL].data_pbuf[0] = &cali_params.roll_linear_buf[0][0];
    joint_data_fit_info[ROLL].data_pbuf[1] = &cali_params.roll_linear_buf[1][0];
    joint_data_fit_info[PITCH].data_pbuf[0] = &cali_params.pitch_linear_buf[0][0];
    joint_data_fit_info[PITCH].data_pbuf[1] = &cali_params.pitch_linear_buf[1][0];
    joint_data_fit_info[YAW].data_pbuf[0] = &cali_params.yaw_linear_buf[0][0];
    joint_data_fit_info[YAW].data_pbuf[1] = &cali_params.yaw_linear_buf[1][0];


    bsp_flash_user_params_read_words(0, (uint32_t *)&cali_params, cali_status.cali_params_size/4);

    if (cali_params.roll_linear_cnt != 0 && cali_params.roll_linear_cnt != 0xFFFFFFFF && !isnan(cali_params.roll_linear_buf[0][0]))
    {
        remo_encoder_set_hall_linear_flag(true, ROLL);
        remo_encoder_set_hall_linear_num(cali_params.roll_linear_cnt, ROLL);
        remo_encoder_set_hall_raw_data0_offset(cali_params.roll_linear_buf[1][0], ROLL);
        for (uint16_t k = 1;k < cali_params.roll_linear_cnt; k++)
        {
            cali_params.roll_linear_buf[1][k] -= cali_params.roll_linear_buf[1][0];
            if (cali_params.roll_linear_buf[1][k] < 0.0f) cali_params.roll_linear_buf[1][k] += 360.0f;
            else if (cali_params.roll_linear_buf[1][k] > 360.0f) cali_params.roll_linear_buf[1][k] -= 360.0f;
        }
        cali_params.roll_linear_buf[1][0] = 0.0f;
    }
    if (cali_params.pitch_linear_cnt != 0 && cali_params.pitch_linear_cnt != 0xFFFFFFFF && !isnan(cali_params.pitch_linear_buf[0][0]))
    {
        remo_encoder_set_hall_linear_flag(true, PITCH);
        remo_encoder_set_hall_linear_num(cali_params.pitch_linear_cnt, PITCH);
        remo_encoder_set_hall_raw_data0_offset(cali_params.pitch_linear_buf[1][0], PITCH);
        for (uint16_t k = 1;k < cali_params.pitch_linear_cnt; k++)
        {
            cali_params.pitch_linear_buf[1][k] -= cali_params.pitch_linear_buf[1][0];
            if (cali_params.pitch_linear_buf[1][k] < 0.0f) cali_params.pitch_linear_buf[1][k] += 360.0f;
            else if (cali_params.pitch_linear_buf[1][k] > 360.0f) cali_params.pitch_linear_buf[1][k] -= 360.0f;
        }
        cali_params.pitch_linear_buf[1][0] = 0.0f;
    }
    if (cali_params.yaw_linear_cnt != 0 && cali_params.yaw_linear_cnt != 0xFFFFFFFF && !isnan(cali_params.yaw_linear_buf[0][0]))
    {
        remo_encoder_set_hall_linear_flag(true, YAW);
        remo_encoder_set_hall_linear_num(cali_params.yaw_linear_cnt, YAW);
        remo_encoder_set_hall_raw_data0_offset(cali_params.yaw_linear_buf[1][0], YAW);
        for (uint16_t k = 1;k < cali_params.yaw_linear_cnt; k++)
        {
            cali_params.yaw_linear_buf[1][k] -= cali_params.yaw_linear_buf[1][0];
            if (cali_params.yaw_linear_buf[1][k] < 0.0f) cali_params.yaw_linear_buf[1][k] += 360.0f;
            else if (cali_params.yaw_linear_buf[1][k] > 360.0f) cali_params.yaw_linear_buf[1][k] -= 360.0f;
        }
        cali_params.yaw_linear_buf[1][0] = 0.0f;
    }

    
    
    
    // remo_flash_read_words(FLASH_ADDR_SECTOR_59, 930, (uint32_t *)&yaw_data_fit_info.data_buf[0][0]);
    // WDT_RefreshCounter();
    // remo_flash_read_words(FLASH_ADDR_SECTOR_60, 930, (uint32_t *)&yaw_data_fit_info.data_buf[1][0]);
    // for (uint16_t i = 0; i < 1860; i++)
    // {
    //     yaw_data_fit_info.data_buf[0][i] -= 2048;
    //     yaw_data_fit_info.data_buf[1][i] -= 2048;
    // }
    // WDT_RefreshCounter();
    // Ellipsoid_LSQ_4p((const int16_t *)&yaw_data_fit_info.data_buf[0][0], (const int16_t *)&yaw_data_fit_info.data_buf[1][0], \
    //         1860, &yaw_data_fit_info.params[0]);
    
    if (cali_params.cali_flag.types.elec_angle_valid)
    {
        remo_esc_set_esc_align_already_flag(true, ROLL);
        remo_esc_set_esc_align_already_flag(true, PITCH);
        remo_esc_set_esc_align_already_flag(true, YAW);
        remo_esc_set_el_angle_offset(cali_params.roll_el_angle_offset, ROLL);
        remo_esc_set_el_angle_offset(cali_params.pitch_el_angle_offset, PITCH);
        remo_esc_set_el_angle_offset(cali_params.yaw_el_angle_offset, YAW);
    }

    if (cali_params.cali_flag.types.joint_angle_valid)
    {
        remo_esc_set_motors_run();
        remo_ctrl_loop_set_jacob_matrix_state(1);
        remo_ctrl_loop_set_pos_ctrl(POS_EULER_CTRL, 0.0f, ROLL);
        remo_ctrl_loop_set_pos_ctrl(POS_EULER_CTRL, 0.0f, PITCH);
        remo_ctrl_loop_set_pos_ctrl(POS_JOINT_CTRL, 0.0f, YAW);
    }

//(cali_params.cali_flag.types.joint_angle_valid == 0)
//    remo_ahrs_set_base_init_done(false);
    // 校准质量
    imu_cali_state.gyro_state = 0;
    imu_cali_state.accel_state = 0;
    imu_cali_state.gyro_state_th = 500;
    imu_cali_state.accel_state_th = 0;

    //// remo_imu_set_gyro_bias(cali_params.gyro_offset[0], cali_params.gyro_offset[1], cali_params.gyro_offset[2]);
    //if (cali_params.cali_flag.types.joint_angle_valid)
    //{
    //    remo_hall_set_joint_offset(cali_params.joint_angle_offset[0], cali_params.joint_angle_offset[1], \
    //                                cali_params.joint_angle_offset[2]); 
    //}
    //if (!isnan(cali_params.accel_offset[0]) && !isnan(cali_params.accel_matrix[0]))
    //{
    //    remo_imu_set_accel_bias(cali_params.accel_offset[0], cali_params.accel_offset[1], cali_params.accel_offset[2]);
    //    remo_imu_set_accel_trans_matrix(cali_params.accel_matrix);  
    //}

    //// 校准角速度的偏移
    //cali_status.gyro_bias = 1;
    //remo_flash_read_words(FLASH_ADDR_GYRO_RST_OFFSET, 3, (uint32_t *)cali_params.gyro_offset);
    //remo_imu_set_gyro_bias(cali_params.gyro_offset[0], cali_params.gyro_offset[1], cali_params.gyro_offset[2]);
    //remo_cali_imu_gyro_bias();
    
}


/**************************************************************************************************
 * 函数名称: remo_cali_new_run
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 设置校准状态
 *          1. 电机电角度对齐；
 *          2. 动态校准，陀螺仪零偏→磁编码零位置校准→加速度计9参数校准→磁编码零位置校准；
 *          3. 陀螺仪9参数校准（根据实际情况选择是否进行）
*           4. 用户校准，加速度校准
**************************************************************************************************/
uint32_t mytest_cali_count = 0;
void remo_cali_run(void)
{
    if (cali_status.cali_set_state_flag)
    {
        cali_status.cali_state = cali_status.cali_test_state;
        cali_status.cali_set_state_flag = false;
    }

    // 校准顺序按照9 -8 -10 - 1 - 2 -3的顺序
    switch (cali_status.cali_state)
    {
        case CALI_STATE_MOTOR_ALIGN:
            remo_cali_motor_alignment();
            break;
        case CALI_STATE_DYNAMIC:
            break;
        case CALI_STATE_GYROTRANS:
            break;
        case CALI_STATE_USER_ACCELTRANS:
            break;
        case CALI_STATE_ROLL_FT:
            break;
        case CALI_STATE_YAW_INIT:
            break;
        case CALI_STATE_USER_MOTOR_ALIGN:
            break;
        case CALI_ROLL_HALL_LINER:   // 8
            remo_cali_joint_linear(ROLL);
            break;
        case CALI_PITCH_HALL_LINER:  // 9
            remo_cali_joint_linear(PITCH);
            break;
        case CALI_YAW_HALL_LINER:    // 10
            remo_cali_joint_linear(YAW);
            break;
        case CALI_ROLL_HALL_FIT:
            break;
        case CALI_PITCH_HALL_FIT:
            break;
        case CALI_YAW_HALL_FIT:
            break;
        default:
            break;
    }

    cali_status.cali_state_last = cali_status.cali_state;

}
    
static void remo_cali_joint_linear(uint8_t axis_index)
{
    const float *joint_deg = remo_encoder_get_joint_deg_ptr();
    const float *euler_deg = ahrs_get_euler_deg_ptr();
    float temp_f = 0.0f;
    uint32_t addr_offset = 0;
    bool temp_flag = false;

    if (axis_index >= 3) return;

    if (!joint_data_fit_info[axis_index].init_flag)
    {
        cali_params.cali_flag.types.joint_angle_valid = 0;
        joint_data_fit_info[axis_index].init_flag = true;
        joint_data_fit_info[axis_index].jfit_state = JFIT_IDLE;
    }
    
    switch(joint_data_fit_info[axis_index].jfit_state)
    {
        case JFIT_IDLE:

                remo_esc_set_esc_status(ESC_ALIGNMENT, axis_index);
                remo_esc_set_esc_align_already_flag(false, axis_index);
                for (uint8_t j = 0; j < JOINT_DATA_FIT_SIZE; j++)
                {
                    *(joint_data_fit_info[axis_index].data_pbuf[0]+j) = 0.0f;
                    *(joint_data_fit_info[axis_index].data_pbuf[1]+j) = 0.0f;
                }
                switch(axis_index)
                {
                    case PITCH:
                        remo_esc_set_esc_status(ESC_STOP, ROLL);
                        remo_esc_set_esc_status(ESC_STOP, YAW);
                        remo_encoder_set_joint_use_euler_flag(true, PITCH);
                        remo_encoder_set_joint_use_euler_flag(false, ROLL);
                        remo_encoder_set_joint_use_euler_flag(false, YAW);
                        remo_ctrl_loop_set_angle_linear_calib_flag(true, PITCH);
                        ahrs_set_base_ori(AHRS_BASE_PITCH_SPECIAL);
                        joint_data_fit_info[PITCH].angle_th = 120.0f;
                        break;
                    case ROLL:
                        remo_esc_set_esc_status(ESC_ALIGNMENT, PITCH);
                        remo_esc_set_esc_align_already_flag(false, PITCH);
                        remo_esc_set_esc_status(ESC_STOP, YAW);
                        remo_encoder_set_joint_use_euler_flag(true, ROLL);
                        remo_encoder_set_joint_use_euler_flag(false, PITCH);
                        remo_encoder_set_joint_use_euler_flag(false, YAW);
                        remo_ctrl_loop_set_angle_linear_calib_flag(true, ROLL);
                        remo_ctrl_loop_set_angle_linear_calib_flag(true, PITCH);
                        ahrs_set_base_ori(AHRS_BASE_ROLL_SPECIAL);
                        joint_data_fit_info[ROLL].angle_th = 100.0f;
                        break;
                    case YAW:
                        remo_esc_set_esc_status(ESC_ALIGNMENT, PITCH);
                        remo_esc_set_esc_align_already_flag(false, PITCH);
                        remo_esc_set_esc_status(ESC_ALIGNMENT, ROLL);
                        remo_esc_set_esc_align_already_flag(false, ROLL);
                        remo_encoder_set_joint_use_euler_flag(false, ROLL);
                        remo_encoder_set_joint_use_euler_flag(false, PITCH);
                        remo_encoder_set_joint_use_euler_flag(true, YAW);
                        remo_ctrl_loop_set_angle_linear_calib_flag(true, ROLL);
                        remo_ctrl_loop_set_angle_linear_calib_flag(true, PITCH);
                        remo_ctrl_loop_set_angle_linear_calib_flag(true, YAW);
                        joint_data_fit_info[YAW].angle_th = 150.0f;
                        ahrs_set_base_ori(AHRS_BASE_YAW_SPECIAL);
                        break;
                    default:
                        break;
                }

                joint_data_fit_info[axis_index].jfit_state = JFIT_EL_ALIGN;
                joint_data_fit_info[axis_index].align_clock_0p1us[0] = bsp_timer_clock_start_0p1us();
                remo_encoder_set_joint_use_euler_flag(true, axis_index);

                joint_data_fit_info[axis_index].angle_last = -300.0f;
                joint_data_fit_info[axis_index].delta_angle_th = 3.0f;
                joint_data_fit_info[axis_index].cnt = 0;
            break;
        case JFIT_EL_ALIGN:
            joint_data_fit_info[axis_index].align_clock_0p1us[1] = bsp_timer_clock_end_0p1us(joint_data_fit_info[axis_index].align_clock_0p1us[0]);
            
            if (joint_data_fit_info[axis_index].align_clock_0p1us[1] > 20e7)
            {
                joint_data_fit_info[axis_index].jfit_state = JFIT_ERR_EL_ALIGN;
                return;
            }
            else if (!remo_esc_get_align_start_flag(axis_index))
            {
                switch(axis_index)
                {
                    case ROLL:
                        if (!remo_esc_get_align_start_flag(PITCH))
                        {
                            remo_esc_set_esc_align_already_flag(true, PITCH);
                            remo_esc_set_esc_status(ESC_RUN, PITCH);
                            remo_ctrl_loop_set_pos_ctrl(POS_JOINT_CTRL, 0.0f, PITCH);
                            remo_ctrl_loop_set_jacob_matrix_state(2);
                            temp_flag = true;
                        }
                        break;
                    case PITCH:
                        remo_ctrl_loop_set_jacob_matrix_state(0);
                        temp_flag = true;
                        break;
                    case YAW:
                        if (!remo_esc_get_align_start_flag(ROLL) && !remo_esc_get_align_start_flag(PITCH))
                        {
                            remo_esc_set_esc_align_already_flag(true, ROLL);
                            remo_esc_set_esc_status(ESC_RUN, ROLL);
                            remo_ctrl_loop_set_pos_ctrl(POS_JOINT_CTRL, 0.0f, ROLL);
                            remo_esc_set_esc_align_already_flag(true, PITCH);
                            remo_esc_set_esc_status(ESC_RUN, PITCH);
                            remo_ctrl_loop_set_pos_ctrl(POS_JOINT_CTRL, 0.0f, PITCH);
                            remo_ctrl_loop_set_jacob_matrix_state(1);
                            temp_flag = true;
                        }
                        break;
                    default:
                        break;
                }
                if (temp_flag)
                {
                    remo_esc_set_esc_align_already_flag(true, axis_index);
                    remo_esc_set_esc_status(ESC_RUN, axis_index);
                    joint_data_fit_info[axis_index].jfit_state = JFIT_INIT_POS;
                    remo_ctrl_loop_set_pos_ctrl(POS_EULER_CTRL, 0.0f, axis_index);
                    joint_data_fit_info[axis_index].align_clock_0p1us[0] = bsp_timer_clock_start_0p1us();
                }
            }
            break;
        case JFIT_INIT_POS:
            joint_data_fit_info[axis_index].align_clock_0p1us[1] = bsp_timer_clock_end_0p1us(joint_data_fit_info[axis_index].align_clock_0p1us[0]);
            
            if (fabs(ahrs_get_euler_axis_deg(axis_index)) < 1.0f)
            {
                joint_data_fit_info[axis_index].jfit_state = JFIT_LOWER_LIMIT;
                remo_ctrl_loop_set_pos_ctrl_enable_state(0, axis_index);
                remo_ctrl_loop_set_vel_target(-50.0f, axis_index);
                joint_data_fit_info[axis_index].align_clock_0p1us[0] = bsp_timer_clock_start_0p1us();
            }

            if (joint_data_fit_info[axis_index].align_clock_0p1us[1] > 10e7)
            {
                joint_data_fit_info[axis_index].jfit_state = JFIT_ERR_INIT_POS;
                return;
            }
            break;
        case JFIT_LOWER_LIMIT:
            joint_data_fit_info[axis_index].align_clock_0p1us[1] = bsp_timer_clock_end_0p1us(joint_data_fit_info[axis_index].align_clock_0p1us[0]);
            if (ahrs_get_euler_axis_deg(axis_index) < -joint_data_fit_info[axis_index].angle_th + 5.0f && joint_data_fit_info[axis_index].align_clock_0p1us[1] > 7e7)
            {
                joint_data_fit_info[axis_index].jfit_state = JFIT_UPPER_LIMIT;
                remo_ctrl_loop_set_pos_ctrl_enable_state(0, axis_index);
                remo_ctrl_loop_set_vel_target(5.0f, axis_index);
                joint_data_fit_info[axis_index].align_clock_0p1us[0] = bsp_timer_clock_start_0p1us();
            }
            if (joint_data_fit_info[axis_index].align_clock_0p1us[1] > 10e7)
            {
                joint_data_fit_info[axis_index].jfit_state = JFIT_ERR_LOWER_LIMIT;
                return;
            }
            break;
        case JFIT_UPPER_LIMIT:
            joint_data_fit_info[axis_index].align_clock_0p1us[1] = bsp_timer_clock_end_0p1us(joint_data_fit_info[axis_index].align_clock_0p1us[0]);

            remo_ctrl_loop_set_vel_target(5.0f, axis_index);

            if (euler_deg[axis_index] - joint_data_fit_info[axis_index].angle_last >= joint_data_fit_info[axis_index].delta_angle_th)
            {
                *(joint_data_fit_info[axis_index].data_pbuf[0] + joint_data_fit_info[axis_index].cnt) = euler_deg[axis_index];
                *(joint_data_fit_info[axis_index].data_pbuf[1] + joint_data_fit_info[axis_index].cnt) = remo_encoder_get_hall_raw_data(axis_index);

                joint_data_fit_info[axis_index].angle_last = euler_deg[axis_index];
                if (++joint_data_fit_info[axis_index].cnt >= JOINT_DATA_FIT_SIZE) 
                {
                    joint_data_fit_info[axis_index].jfit_state = JFIT_ERR;
                }

                if (euler_deg[axis_index] >= joint_data_fit_info[axis_index].angle_th - 5.0f)
                {
                    joint_data_fit_info[axis_index].jfit_state = JFIT_SAVE_DATA;
                }
            }

            if (joint_data_fit_info[axis_index].align_clock_0p1us[1] > 60e7)
            {
                joint_data_fit_info[axis_index].jfit_state = JFIT_ERR_UPPER_LIMIT;
                return;
            }
            
            break;
        case JFIT_SAVE_DATA:
            switch(axis_index)
            {
                case ROLL:
                    cali_params.roll_linear_cnt = joint_data_fit_info[axis_index].cnt;
                    addr_offset = (uint32_t)&cali_params.roll_linear_cnt - (uint32_t)&cali_params;
                    bsp_flash_user_params_write_words(addr_offset/4, &cali_params.roll_linear_cnt, JOINT_DATA_FIT_SIZE*2+1);
                    remo_encoder_set_hall_raw_data0_offset(cali_params.roll_linear_buf[1][0], 0);
                    for (uint16_t k = 1;k < cali_params.roll_linear_cnt; k++)
                    {
                        cali_params.roll_linear_buf[1][k] -= cali_params.roll_linear_buf[1][0];
                    }
                    cali_params.roll_linear_buf[1][0] = 0.0f;
                    break;
                case PITCH:
                    cali_params.pitch_linear_cnt = joint_data_fit_info[axis_index].cnt;
                    addr_offset = (uint32_t)&cali_params.pitch_linear_cnt - (uint32_t)&cali_params;
                    bsp_flash_user_params_write_words(addr_offset/4, &cali_params.pitch_linear_cnt, JOINT_DATA_FIT_SIZE*2+1);
                    remo_encoder_set_hall_raw_data0_offset(cali_params.pitch_linear_buf[1][0], 1);
                    for (uint16_t k = 1;k < cali_params.pitch_linear_cnt; k++)
                    {
                        cali_params.pitch_linear_buf[1][k] -= cali_params.pitch_linear_buf[1][0];
                    }
                    cali_params.pitch_linear_buf[1][0] = 0.0f;
                    break;
                case YAW:
                    cali_params.yaw_linear_cnt = joint_data_fit_info[axis_index].cnt;
                    addr_offset = (uint32_t)&cali_params.yaw_linear_cnt - (uint32_t)&cali_params;
                    bsp_flash_user_params_write_words(addr_offset/4, &cali_params.yaw_linear_cnt, JOINT_DATA_FIT_SIZE*2+1);
                    remo_encoder_set_hall_raw_data0_offset(cali_params.yaw_linear_buf[1][0], 2);
                    for (uint16_t k = 1;k < cali_params.yaw_linear_cnt; k++)
                    {
                        cali_params.yaw_linear_buf[1][k] -= cali_params.yaw_linear_buf[1][0];
                    }
                    cali_params.yaw_linear_buf[1][0] = 0.0f;
                    break;
                default:
                    break;
            }
            
            // TODO:需要对线性度进行判断，即一定欧拉角间隔内关节角是否太小。
            
            
            cali_status.cali_state = CALI_STATE_IDLE;
            remo_ctrl_loop_set_pos_ctrl(POS_EULER_CTRL, 0.0f, axis_index);
            break;
        case JFIT_FINISH:
            break;
        case JFIT_ERR:
        case JFIT_ERR_EL_ALIGN:
        case JFIT_ERR_INIT_POS:
        case JFIT_ERR_LOWER_LIMIT:
        case JFIT_ERR_UPPER_LIMIT:
        case JFIT_ERR_SAVE_DATA:
            cali_status.cali_state = CALI_ERR_PITCH_HALL_LINER;
            break;
        default:
            break;
    }
    
}




/**************************************************************************************************
 * 函数名称: remo_cali_motor_alignment
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 电机电角度校准
**************************************************************************************************/
uint32_t align_timer_0p1us[2] = {0};
static void remo_cali_motor_alignment(void)
{
    bool gyro_temp_bias_flag = false;
    static bool init_flag = false;
    static uint16_t ahrs_ok_count = 0;

    const float *joint_deg = remo_encoder_get_joint_deg_ptr();
    const float *euler_deg = ahrs_get_euler_deg_ptr();

    uint32_t addr_offset = 0;

    // 判断是否有异常。一般imu和电机控制模块容易出问题
    if (!remo_imu_get_run_normal_flag())
    {
        cali_status.elec_align_state = ERR_EL_IMU;
        cali_status.cali_state = CALI_ERR_MOTOR_ALIGN;
        init_flag = false;
        return;
    }
    else if (remo_esc_get_fault_type(ROLL) || remo_esc_get_fault_type(PITCH) || remo_esc_get_fault_type(YAW))
    {
        cali_status.elec_align_state = ERR_EL_MOTOR_ERR;
        cali_status.cali_state = CALI_ERR_MOTOR_ALIGN;
        init_flag = false;
        return;
    }
    
    

    // 1. 检测电机控制模块是否正常
    if (!init_flag)
    {
        init_flag = true;
//        remo_cali_reset_accel_params();
        align_timer_0p1us[0] = bsp_timer_clock_start_0p1us();
        cali_status.elec_align_state = EL_ALIGNMENT_PREPARE; // 电机控制模块没有问题进入电角度校准
    }

    switch(cali_status.elec_align_state)
    {
        case EL_ALIGNMENT_PREPARE:
            align_timer_0p1us[1] = bsp_timer_clock_end_0p1us(align_timer_0p1us[0]);
            // 进入电角度对齐后，首先要关闭电机，并将校准标志清零
            if (align_timer_0p1us[1] > 12e4)
            {
                if (remo_esc_get_motors_state() != MOTOR_STOP)
                {
                    cali_params.cali_flag.types.elec_angle_valid = 0;
                    cali_params.cali_flag.types.joint_angle_valid = 0;
                    cali_status.roll_elec_angle = 0;
                    cali_status.pitch_elec_angle = 0;
                    cali_status.yaw_elec_angle = 0;
                    cali_status.joint_offset_state = JO_PREPARE;
                    cali_status.ahrs_convergence_valid = 0;

                    // 发送电机停止指令
                    remo_esc_set_motors_stop();
                    if (align_timer_0p1us[1] > 800e4)   // 长时间停止不了，异常退出
                    {
                        cali_status.elec_align_state = ERR_EL_MOTOR_CANNT_STOP;
                        cali_status.cali_state = CALI_ERR_MOTOR_ALIGN;
                        init_flag = false;
                        return;
                    }
                }
                else
                {
                    // 进入正式的校准阶段
                    addr_offset = (uint32_t)&cali_params.cali_flag - (uint32_t)&cali_params;
                    bsp_flash_user_params_write_words(0, (uint32_t *)&cali_params.cali_flag.which_type, sizeof(cali_params.cali_flag)/4);
                    align_timer_0p1us[0] = bsp_timer_clock_start_0p1us();
                    cali_status.elec_align_state = EL_MOTOR_EL_ALIGMENT;
                }
            }
            break;

        case EL_MOTOR_EL_ALIGMENT:     // 2. 电机电角度校准 
            align_timer_0p1us[1] = bsp_timer_clock_end_0p1us(align_timer_0p1us[0]);
            // 分时发送电机电角度停止指令，时间间隔50ms
            if (align_timer_0p1us[1] > 200e4)
            {
                if (cali_status.yaw_elec_angle == 0)
                {
                    cali_status.yaw_elec_angle = 1;
                    remo_esc_set_esc_status(ESC_ALIGNMENT, YAW);
                }
            }
            else if (align_timer_0p1us[1] > 150e4)
            {
                if (cali_status.pitch_elec_angle == 0)
                {
                    cali_status.pitch_elec_angle = 1;
                    remo_esc_set_esc_status(ESC_ALIGNMENT, PITCH);
                }
            }
            else if (align_timer_0p1us[1] > 100e4)
            {
                if (cali_status.roll_elec_angle == 0)
                {
                    cali_status.roll_elec_angle = 1;
                    remo_esc_set_esc_status(ESC_ALIGNMENT, ROLL);
                }
            }
            if (align_timer_0p1us[1] > 15e7)
            {
                // 电角度校准完成后会进入stop状态
                // 30秒后，电角度还未校准完，强制停止校准（校准过程出现错误）
                if (remo_esc_get_motors_state() != MOTOR_STOP)
                {
                    // 停止电机运行
                    remo_esc_set_motors_stop();
                    cali_status.elec_align_state = ERR_EL_MOTOR_CANNT_STOP;
                    cali_status.cali_state = CALI_ERR_MOTOR_ALIGN;
                    init_flag = false;
                    return;
                }
                else
                {
                    cali_params.cali_flag.types.elec_angle_valid = 1;
                    cali_params.roll_el_angle_offset = remo_esc_get_el_angle_offset(ROLL);
                    cali_params.pitch_el_angle_offset = remo_esc_get_el_angle_offset(PITCH);
                    cali_params.yaw_el_angle_offset = remo_esc_get_el_angle_offset(YAW);
                    cali_status.elec_align_state = EL_GYRO_BIAS;
                    cali_status.joint_offset_state = JO_PREPARE;
                    align_timer_0p1us[0] = bsp_timer_clock_start_0p1us();
                  }
            }
            break;

        case EL_GYRO_BIAS:     // 4. 陀螺仪零偏校准
            align_timer_0p1us[1] = bsp_timer_clock_end_0p1us(align_timer_0p1us[0]);
            if (remo_esc_get_motors_state() != MOTOR_STOP)
            {
                remo_esc_set_motors_stop();
                if (align_timer_0p1us[1] > 300e4) 
                {
                    cali_status.elec_align_state = ERR_EL_MOTOR_CANNT_STOP;
                    cali_status.cali_state = CALI_ERR_MOTOR_ALIGN;
                    init_flag = false;
                }
                break;
            }
            gyro_temp_bias_flag = remo_cali_gyro_temp_bias();
            if (gyro_temp_bias_flag)
            {
                cali_params.gyro_heat_offset[0] = gyro_temp_bias[0] + remo_imu_get_gyro_bias(0);
                cali_params.gyro_heat_offset[1] = gyro_temp_bias[1] + remo_imu_get_gyro_bias(1);
                cali_params.gyro_heat_offset[2] = gyro_temp_bias[2] + remo_imu_get_gyro_bias(2);
                cali_params.gyro_offset[0] = gyro_temp_bias[0] + remo_imu_get_gyro_bias(0);
                cali_params.gyro_offset[1] = gyro_temp_bias[1] + remo_imu_get_gyro_bias(1);
                cali_params.gyro_offset[2] = gyro_temp_bias[2] + remo_imu_get_gyro_bias(2);
                cali_params.cali_flag.types.gyro_bias_heat_valid = 1;
                cali_status.elec_align_state = EL_DATA_REFLASH; 
                remo_imu_set_gyro_bias(cali_params.gyro_offset[0], cali_params.gyro_offset[1], cali_params.gyro_offset[2]);
            }
            else if (align_timer_0p1us[1] > 15e7)
            {
                cali_status.elec_align_state = ERR_EL_GYRO_BIAS;
                cali_status.cali_state = CALI_ERR_MOTOR_ALIGN;
                return;
            }
            break;

        case EL_DATA_REFLASH:     // 5. 保存数据
            cali_params.cali_flag.types.joint_angle_valid = 1;
            // 校准1完成，led闪烁改变
            cali_params.cali_flag.types.led_state = 1;
            cali_params.cali_flag.types.factory_test_state = 2;   // 默认测试失败，只有最后成功才改写为成功
            
            addr_offset = (uint32_t)&cali_params.reserve1_data - (uint32_t)&cali_params;
            bsp_flash_user_params_write_words(0, (uint32_t *)&cali_params.cali_flag, addr_offset/4);

            cali_status.elec_align_state = EL_ATTI_CONVERGE_CHECK;
            align_timer_0p1us[0] = bsp_timer_clock_start_0p1us();
            break;

        case EL_ATTI_CONVERGE_CHECK:   // 姿态收敛性检验
            align_timer_0p1us[1] = bsp_timer_clock_end_0p1us(align_timer_0p1us[0]);
            if(ahrs_get_ahrs_err_sum_sq() < 0.01f && align_timer_0p1us[1] >= 10e7) 
            {
                if (++ahrs_ok_count >= 15)
                {
                    cali_status.elec_align_state = EL_ALIGNMENT_FINISH;
                    remo_esc_set_motors_run();
                    remo_ctrl_loop_params_init();
                    align_timer_0p1us[0] = bsp_timer_clock_start_0p1us();
                    break;
                }
            }
            if (align_timer_0p1us[1] >= 20e7)
            {
                cali_status.elec_align_state = ERR_EL_ATTI_CONVERGE_CHECK;
                cali_status.cali_state = CALI_ERR_MOTOR_ALIGN;
                init_flag = false;
                return;
            }           
            break;

        case EL_ALIGNMENT_FINISH:
            align_timer_0p1us[1] = bsp_timer_clock_end_0p1us(align_timer_0p1us[0]);
            if (remo_esc_get_motors_state() != MOTOR_RUN)
            {
                remo_esc_set_motors_run();
                if (align_timer_0p1us[1] > 500e4) 
                {
                    cali_status.elec_align_state = ERR_EL_MOTOR_CANNT_RUN;
                    cali_status.cali_state = CALI_ERR_MOTOR_ALIGN;
                    init_flag = false;
                }
                break;
            }
            else
            {
                remo_ctrl_loop_set_jacob_matrix_state(1);
                remo_ctrl_loop_set_pos_ctrl(POS_JOINT_CTRL, 0.0f, ROLL);
                remo_ctrl_loop_set_pos_ctrl(POS_JOINT_CTRL, 0.0f, PITCH);
                remo_ctrl_loop_set_pos_ctrl(POS_JOINT_CTRL, 0.0f, YAW);
                if (fabs(joint_deg[ROLL]) < 2.0f && fabs(joint_deg[PITCH]) < 2.0f && fabs(joint_deg[YAW]) < 2.0f)
                {
                    ahrs_ok_count = 0;
                    cali_status.roll_elec_angle = 0;
                    cali_status.pitch_elec_angle = 0;
                    cali_status.yaw_elec_angle = 0;
                    cali_status.elec_align_state = EL_ALIGNMENT_PREPARE;
                    cali_status.cali_state = CALI_STATE_IDLE;
                    remo_ctrl_loop_set_pos_kp_scale(1.0f, YAW);
                    init_flag = false;
                }
                else if (align_timer_0p1us[1] > 7e7)
                {
                    if (fabs(joint_deg[ROLL]) >= 2.0f) cali_status.elec_align_state = ERR_EL_ROLL_JOINT;
                    else if (fabs(joint_deg[PITCH]) >= 2.0f) cali_status.elec_align_state = ERR_EL_PITCH_JOINT;
                    else if (fabs(joint_deg[YAW]) >= 2.0f) cali_status.elec_align_state = ERR_EL_YAW_JOINT;
                    cali_status.cali_state = CALI_ERR_MOTOR_ALIGN;
                    init_flag = false;
                }
            }
            break;
        case EL_ALIGNMENT_WAIT:
        default:
            break;
    }
}


static bool remo_cali_gyro_temp_bias(void)
{
    static const int16_q7_t *gyro_smp;
    static int32_t gyro_mean1[3] = {0}, gyro_mean2[3] = {0}; 
    static uint16_t count = 0;
    uint32_t mean_diff = 0.0f;
    uint8_t i = 0;

    gyro_smp = remo_imu_get_gyro_smp();

    if (count == 0)
    {
        for (i = 0; i <= 2; i++)
        {
            gyro_mean1[i] = 0;
            gyro_mean2[i] = 0;
            gyro_temp_bias[i] = 0;
        }
    }

    count ++;
    if (count <= 256)
    {
        gyro_mean1[0] += gyro_smp[0];
        gyro_mean1[1] += gyro_smp[1];
        gyro_mean1[2] += gyro_smp[2];
    }
    else if (count <= 512)
    {
        gyro_mean2[0] += gyro_smp[0];
        gyro_mean2[1] += gyro_smp[1];
        gyro_mean2[2] += gyro_smp[2];
    }
	else
	{
		count = 0;
	}

    if (count == 512)
    {
        for (i = 0; i <= 2; i++)
        {
            gyro_mean1[i] >>= 8;
            gyro_mean2[i] >>= 8;
            gyro_temp_bias[i] = (gyro_mean1[i] + gyro_mean2[i]) >> 1;
        }

        mean_diff = abs(gyro_mean1[0] - gyro_mean2[0]) + abs(gyro_mean1[1] - gyro_mean2[1]) +
                   abs(gyro_mean1[2] - gyro_mean2[2]);

        
        if (mean_diff < CALI_GYRO_AVG_DIFF_TH && 
                abs(gyro_temp_bias[0]) < imu_cali_state.gyro_state_th && 
                abs(gyro_temp_bias[1]) < imu_cali_state.gyro_state_th &&
                abs(gyro_temp_bias[2]) < imu_cali_state.gyro_state_th)
		{
            return true;
        }

        count = 0;
    }

    return false;
}


//#if 0
///**************************************************************************************************
// * 函数名称: remo_cali_new_dynamic_state
// * 输入参数: void
// * 返回结果: void
// * 功能描述: 动态校准
// *          陀螺仪零偏→磁编码零位置校准→加速度计9参数校准→磁编码零位置校准
//**************************************************************************************************/
//uint32_t acfit_timer_us[2] = {0};
//static void remo_cali_new_dynamic_state(void)
//{
//    static uint16_t pitch_delay_cnt = 0, motor_delay_cnt = 0, run_count = 0;
//    static bool params_init_flag = false;

//    const float *euler_deg, *joint_deg;

//    static int16_q7_t pitch_joint_deg_last = 0;
//    int16_t accel_fit_smp_temp[3] = {0};

//    bool lm_flag = false;
//    float x = 0, y = 0, z = 0, A = 0, B = 0, C = 0, len = 0;
//	float residual = 0;
//    int16_t accel_lm_params[9] = {0};
//    static float accel_cali_params_temp[9] = {0};
//    static bool accel_vel_init_flag = false;

//    uint8_t i = 0;

//    // 获取欧拉角
//    euler_deg = ahrs_get_euler_deg();
//    // 获取关节角
//    joint_deg = remo_hall_get_joint_deg_ptr();
	
//    // 如果没有进行电角度对齐，则不能进行此步校准
//    if(cali_params.cali_flag.types.elec_angle_valid == 0)
//    {
//        cali_status.accel_fit_state = ERR_AC_EL_NONFINISH;
//        cali_status.cali_state = CALI_ERR_DYNAMIC;
//        return;
//    }
    
//    if (!params_init_flag)
//    {
//        cali_status.accel_fit_state = AC_FIT_PREPARE;
//        acfit_timer_us[0] = remo_clock_timing_start_us();
//        acfit_timer_us[1] = 0;
//        // 重置加速度计的参数
//        remo_cali_reset_accel_params();

//            // 初始化云台位置
//            remo_kinematics_reset_vel_ref_and_ip_var(0); // 防止某些速度参数没有没清零
//            remo_kinematics_reset_vel_ref_and_ip_var(1);
//            remo_kinematics_reset_vel_ref_and_ip_var(2);
////            remo_ctrl_loop_set_pos_kp_scale(2.0f, PITCH);
////            remo_ctrl_loop_set_vel_ki_scale(2.5f, PITCH);   // 270

//        cali_accel.smp_renew_cnt = 0;
//        cali_accel.smp_cnt = 0;
//        cali_accel.smp_index = 0;
//        cali_accel.smp_valid = 0;
//        cali_accel.RANSAC_iter = 0;
//        params_init_flag = true;
//    }


//    // 初始化变量和状态
//    if (cali_status.accel_fit_state == AC_FIT_PREPARE)
//    {
//        acfit_timer_us[1] += remo_clock_timing_end_us(acfit_timer_us[0]);
//        acfit_timer_us[0] = remo_clock_timing_start_us();
//		remo_motion_plan_set_aim_pos_deg(ACCEL_FIT_AIM_ANGLE_DEG, MOTION_CTRL_JOINT, PITCH);
//		remo_motion_plan_set_aim_pos_deg(0, MOTION_CTRL_JOINT, YAW);
//        accel_vel_init_flag = false;
//        // 检测到pitch运动到极限位置，等待一段时间后控制pitch以一定速度运行到指定位置
//        if (fabs(joint_deg[PITCH] - (ACCEL_FIT_AIM_ANGLE_DEG)) < 1.0f)
//        {
//            if (acfit_timer_us[1] > 1000e3)
//            {
//                remo_motion_cmd_set_velo_posi(700, -ACCEL_FIT_AIM_ANGLE_DEG*100, PITCH);
//                cali_status.accel_fit_state = AC_ACCEL_SAMPLE;
//                pitch_joint_deg_last = joint_deg[PITCH];
//                acfit_timer_us[0] = remo_clock_timing_start_us();
//                acfit_timer_us[1] = 0;
//            }
//        }

//        if (acfit_timer_us[1] > 6000e3)
//        {
//            cali_status.accel_fit_state = ERR_AC_PITCH_INIT;
////                fac_gimbal_err = CALI2_PITCH_CTRL_ERR;
//            cali_status.cali_state = CALI_ERR_DYNAMIC;
//            params_init_flag = false;
//            return;
//        }
//    }

//    switch (cali_status.accel_fit_state)
//    {
//        case AC_ACCEL_SAMPLE:
//            acfit_timer_us[1] += remo_clock_timing_end_us(acfit_timer_us[0]);
//            acfit_timer_us[0] = remo_clock_timing_start_us();
//            if (remo_cali_accel_fit_smp(accel_fit_smp_temp))
//            {
//                // 采样间隔判断
//                if (fabs(pitch_joint_deg_last - joint_deg[PITCH]) > ACCEL_FIT_DELTA_ANGLE_TH)
//                {
//                    cali_accel.smp[cali_accel.smp_index][0] = (float)(accel_fit_smp_temp[0] / ACCEL_SMP_NORM);
//                    cali_accel.smp[cali_accel.smp_index][1] = (float)(accel_fit_smp_temp[1] / ACCEL_SMP_NORM);
//                    cali_accel.smp[cali_accel.smp_index][2] = (float)(accel_fit_smp_temp[2] / ACCEL_SMP_NORM);
//                    cali_accel.smp_index ++;
//                    pitch_joint_deg_last = joint_deg[PITCH];
//                }
//                // 采样是否完成
//                if ((cali_accel.smp_index > 1 && fabs(joint_deg[PITCH] - ACCEL_FIT_AIM_ANGLE_DEG) < 1.0f) ||
//                        (cali_accel.smp_index >= ACCEL_FIT_SMP_NUM))
//                {
//                    if (cali_accel.smp_index > ACCEL_FIT_SMP_NUM - 8)
//                    {
//                        cali_status.accel_fit_state = AC_LM_CAL;
//                        remo_esc_set_motors_stop();
//                        acfit_timer_us[0] = remo_clock_timing_start_us();
//                        acfit_timer_us[1] = 0;
//                    }
//                    else
//                    {
//                        cali_status.accel_fit_state = ERR_AC_ACCEL_SAMPLE_DATA;
////                        fac_gimbal_err = CALI2_ACCEL_ERR;
//                        cali_status.cali_state = CALI_ERR_DYNAMIC;
//                        params_init_flag = false;
//                        return;
//                    }
//                }
//            }
//            // 反向运动
//            if (fabs(joint_deg[PITCH] + ACCEL_FIT_AIM_ANGLE_DEG) < 1.0f)
//            {
//                if (!accel_vel_init_flag)
//                {
//                    accel_vel_init_flag = true;
//                    remo_motion_cmd_set_velo_posi(700, ACCEL_FIT_AIM_ANGLE_DEG*100, PITCH);
//                }
//            }

//            if (acfit_timer_us[1] > 50000e3)
//            {
//                cali_status.accel_fit_state = ERR_AC_ACCEL_SAMPLE_LONG;
////                fac_gimbal_err = CALI2_ACCEL_ERR;
//                cali_status.cali_state = CALI_ERR_DYNAMIC;
//                params_init_flag = false;
//                return;
//            }

//            break;
//        case AC_LM_CAL:
//            acfit_timer_us[1] += remo_clock_timing_end_us(acfit_timer_us[0]);
//            acfit_timer_us[0] = remo_clock_timing_start_us();
//            // 停电机 
//            if (remo_esc_get_motors_state() != MOTOR_STOP)
//            {
//                // 停止电机运行
//                if (acfit_timer_us[1] >= 200e3)
//                {
//                    remo_esc_set_motors_stop();
//                    acfit_timer_us[0] = remo_clock_timing_start_us();
//                }
//                return;
//            }
//			else
//			{
//				if (acfit_timer_us[1] < 2000e3)
//				{
//					return;
//				}
//			}

//            if(cali_accel.RANSAC_iter++ < cali_accel.RANSAC_iter_num)
//            {
//                //  生成随机采样索引
//                cali_accel.RANSAC_select_num = 0;
//                remo_rand(ACCEL_FIT_ROLLSMP_NUM, cali_accel.smp_index / ACCEL_FIT_ROLLSMP_NUM, cali_accel.rand_matrix);
//                for (i = 0; i < cali_accel.smp_index; i ++ )
//                {
//                    cali_accel.select_index[i] = 0;
//                    if(cali_accel.rand_matrix[i] < cali_accel.RANSAC_ratio)
//                    {
//                        cali_accel.select_index[cali_accel.RANSAC_select_num++] = i;
//                    }
//                }
//                // 计算校准值
//                lm_flag = remo_cali_accel_cali_LM(&cali_accel.smp[0][0], cali_accel.select_index, cali_accel.RANSAC_select_num);
//                if(!lm_flag)
//                {
//                    return;
//                }
//                cali_accel.RANSAC_k = 0;
//                for (i = 0; i < 6; i++)
//                {
//                    accel_lm_params[i] = cali_accel.params[i] * CONST_1_Q23;
//                }
//                accel_lm_params[6] = cali_accel.params[i] * IMU_ACC_G_VALUE;
//                for(i = 0; i < cali_accel.smp_index; i++)
//                {
//                    // 减去偏置
//                    x = cali_accel.smp[i][0] + cali_accel.params[6];
//                    y = cali_accel.smp[i][1] + cali_accel.params[7];
//                    z = cali_accel.smp[i][2] + cali_accel.params[8];
//                    // 非正交和Scale校正
//                    A = cali_accel.params[0] * x + cali_accel.params[3] * y + cali_accel.params[4] * z;
//                    B = cali_accel.params[3] * x + cali_accel.params[1] * y + cali_accel.params[5] * z;
//                    C = cali_accel.params[4] * x + cali_accel.params[5] * y + cali_accel.params[2] * z;
//                    // 求拟合残差
//                    len = sqrtf(A * A + B * B + C * C);
//                    residual = 1.0f - len;

//                    if(fabs(residual) < cali_accel.RANSAC_residual_th)
//                    {
//                        cali_accel.RANSAC_k++;
//                    }
//                }
//                // 记录最优解
//                if(cali_accel.RANSAC_k > cali_accel.RANSAC_pretotal)
//                {
//                    cali_accel.RANSAC_pretotal = cali_accel.RANSAC_k;
//                    for (uint8_t j = 0; j < 9; j++)
//                    {
//                        accel_cali_params_temp[j] = cali_accel.params[j];
//                    }
//                }
//				WDT_RefreshCounter();
//            }
//            else
//            {
//                if (fabs(accel_cali_params_temp[6]) > 0.25f || fabs(accel_cali_params_temp[7]) > 0.25f || fabs(accel_cali_params_temp[8]) > 0.25f)
//                {
//                    cali_status.accel_fit_state = ERR_AC_LM_CAL_DATA;
////                    fac_gimbal_err = CALI2_ACCEL_ERR;
//                    cali_status.cali_state = CALI_ERR_DYNAMIC;
//                    params_init_flag = false;
//                    return;
//                }
//                if (fabs(accel_cali_params_temp[6]) > 0.1f || fabs(accel_cali_params_temp[7]) > 0.1f || fabs(accel_cali_params_temp[8]) > 0.1f)
//                {
//                    accel_cali_params_temp[0] = 1.0f;
//                    accel_cali_params_temp[1] = 1.0f;
//                    accel_cali_params_temp[2] = 1.0f;
//                    accel_cali_params_temp[3] = 0.0f;
//                    accel_cali_params_temp[4] = 0.0f;
//                    accel_cali_params_temp[5] = 0.0f;
//                    accel_cali_params_temp[6] = 0.0f;
//                    accel_cali_params_temp[7] = 0.0f;
//                    accel_cali_params_temp[8] = 0.0f;
//                }
//                cali_status.accel_fit_state = AC_MOTOR_JOINT_OFFSET;
//                cali_status.joint_offset_state = 1;
//                cali_params.accel_offset[0] = -accel_cali_params_temp[6] * IMU_ACC_G_VALUE;
//                cali_params.accel_offset[1] = -accel_cali_params_temp[7] * IMU_ACC_G_VALUE;
//                cali_params.accel_offset[2] = -accel_cali_params_temp[8] * IMU_ACC_G_VALUE;
//                remo_imu_set_accel_bias(cali_params.accel_offset[0], cali_params.accel_offset[1], \
//                                        cali_params.accel_offset[2]);
//                cali_params.accel_matrix[0] = accel_cali_params_temp[0] * IMU_ACC_TO_MS2;
//                cali_params.accel_matrix[1] = accel_cali_params_temp[3] * IMU_ACC_TO_MS2;
//                cali_params.accel_matrix[2] = accel_cali_params_temp[4] * IMU_ACC_TO_MS2;
//                cali_params.accel_matrix[3] = accel_cali_params_temp[3] * IMU_ACC_TO_MS2;
//                cali_params.accel_matrix[4] = accel_cali_params_temp[1] * IMU_ACC_TO_MS2;
//                cali_params.accel_matrix[5] = accel_cali_params_temp[5] * IMU_ACC_TO_MS2;
//                cali_params.accel_matrix[6] = accel_cali_params_temp[4] * IMU_ACC_TO_MS2;
//                cali_params.accel_matrix[7] = accel_cali_params_temp[5] * IMU_ACC_TO_MS2;
//                cali_params.accel_matrix[8] = accel_cali_params_temp[2] * IMU_ACC_TO_MS2;
//                remo_imu_set_accel_trans_matrix(cali_params.accel_matrix);
//                remo_motion_plan_set_aim_pos_deg(0, MOTION_CTRL_EULER, PITCH);
//                remo_motion_plan_set_aim_pos_deg(0, MOTION_CTRL_JOINT, YAW);
//                remo_esc_set_motors_run();
//                acfit_timer_us[0] = remo_clock_timing_start_us();
//                acfit_timer_us[1] = 0;
//            }

//            if (acfit_timer_us[1] > 20000e3)
//            {
//                cali_status.accel_fit_state = ERR_AC_LM_CAL_LONG;
////                fac_gimbal_err = CALI2_ACCEL_ERR;
//                cali_status.cali_state = CALI_ERR_DYNAMIC;
//                params_init_flag = false;
//                return;
//            }

//            break;
//        case AC_MOTOR_JOINT_OFFSET:
//            acfit_timer_us[1] += remo_clock_timing_end_us(acfit_timer_us[0]);
//            acfit_timer_us[0] = remo_clock_timing_start_us();
//            // 开启电机 
//            if (remo_esc_get_motors_state() != MOTOR_RUN)
//            {
//                // 停止电机运行
//                if (acfit_timer_us[1] >= 200e3)
//                {
//                    remo_esc_set_motors_run();
//                    acfit_timer_us[0] = remo_clock_timing_start_us();
//                }
//                return;
//            }
//            remo_motion_plan_set_aim_pos_deg(0, MOTION_CTRL_EULER, PITCH);
//            if (fabs(euler_deg[PITCH]) < 1.0f)
//            {
//                if (acfit_timer_us[1] > 4000e3)
//                {
//                    cali_params.joint_angle_offset[1] = remo_hall_get_joint_deg_raw(PITCH);
//                    remo_hall_set_joint_offset(cali_params.joint_angle_offset[0], cali_params.joint_angle_offset[1], \
//                                            cali_params.joint_angle_offset[2]);
//                    cali_status.accel_fit_state = AC_DATA_REFLASH;
//                    remo_esc_set_motors_stop();
//                    acfit_timer_us[0] = remo_clock_timing_start_us();
//                    acfit_timer_us[1] = 0;
//                }
//            }

//            if (acfit_timer_us[1] > 10000e3)
//            {
//                cali_status.accel_fit_state = ERR_AC_MOTOR_JOINT_OFFSET;
////                fac_gimbal_err = CALI2_MAGEN_ERR;
//                cali_status.cali_state = CALI_ERR_DYNAMIC;
//                params_init_flag = false;
//                return;
//            }

//            break;
//        case AC_DATA_REFLASH:
//            acfit_timer_us[1] += remo_clock_timing_end_us(acfit_timer_us[0]);
//            acfit_timer_us[0] = remo_clock_timing_start_us();
//            if (remo_esc_get_motors_state() != MOTOR_STOP)
//            {
//                if (acfit_timer_us[1] >= 200e3)
//                {
//                    // 发送电机停止指令
//                    remo_esc_set_motors_stop();
//                    acfit_timer_us[0] = remo_clock_timing_start_us();
//                }
//                return;
//            }
//            else
//            {
//                // 写入磁编码零偏值
//                remo_flash_local_refresh(JOINT_ADDR_OFFSET, (uint32_t *)&cali_params.joint_angle_offset[0], 3);
//                // 写入加速度计校准值
//                remo_flash_local_refresh(ACC_ADDR_OFFSET, (uint32_t *)&cali_params.accel_offset[0], 12);
//                cali_status.accel_fit_state = AC_ATTI_CONVERGE_CHECK;
//                remo_esc_set_motors_run();
//                acfit_timer_us[0] = remo_clock_timing_start_us();
//                acfit_timer_us[1] = 0;
//            }

//            if (acfit_timer_us[1] > 5000e3)
//            {
//                cali_status.accel_fit_state = ERR_AC_DATA_REFLASH;
////                fac_gimbal_err = CALI2_SAVE_PARAMS;
//                cali_status.cali_state = CALI_ERR_DYNAMIC;
//                params_init_flag = false;
//                return;
//            }

//            break;
//        case AC_ATTI_CONVERGE_CHECK:
//            acfit_timer_us[1] += remo_clock_timing_end_us(acfit_timer_us[0]);
//            acfit_timer_us[0] = remo_clock_timing_start_us();
//            if (remo_esc_get_motors_state() != MOTOR_RUN)
//            {
//                if (acfit_timer_us[1] >= 100e3)
//                {
//                    // 发送电机开启指令
//                    remo_esc_set_motors_run();
//                    acfit_timer_us[0] = remo_clock_timing_start_us();
//                }
//                return;
//            }
//            else
//            {
//                remo_motion_plan_set_aim_pos_deg(0, MOTION_CTRL_EULER, PITCH);
//                remo_motion_plan_set_aim_pos_deg(0, MOTION_CTRL_JOINT, YAW);
//                if (fabs(joint_deg[PITCH]) < 1.0f && fabs(joint_deg[YAW]) < 1.0f)
//                cali_status.accel_fit_state = AC_FIT_PREPARE;
//                cali_status.cali_state = CALI_STATE_IDLE;
//                params_init_flag = false;
//            }

//            if (acfit_timer_us[1] > 10000e3)
//            {
//                cali_status.accel_fit_state = ERR_AC_ATTI_CONVERGE_CHECK;
////                fac_gimbal_err = CALI2_AHRS_ERR;
//                cali_status.cali_state = CALI_ERR_DYNAMIC;
//                params_init_flag = false;
//                return;
//            }
//            break;
//        default:
//            break;
//    }
//}


///**************************************************************************************************
// * 函数名称: remo_imu_bias_average
// * 输入参数: avg->均值地址；pow_2->计算次数指数(times = 2^pow2)；
// *          bias_type->校准类型，1：陀螺仪；2：加速度计
// * 返回结果: void
// * 功能描述: 计算imu采样结果的样本均值
//**************************************************************************************************/
//static bool remo_imu_bias_average(int32_t *avg, uint8_t pow2, uint8_t bias_type)
//{
//    static const int16_t *imu_smp, *temp_smp;
//    uint8_t index;
//    static uint16_t times = 0;

//    if (times == 0)
//    {
//        if (bias_type == CALI_GYRO_TYEP)
//        {
//            imu_smp = remo_imu_get_gyro_smp();
//        }
//        else if (bias_type == CALI_ACCEL_TYEP)
//        {
//            imu_smp = remo_imu_get_accel_smp();
//        }
//        temp_smp = remo_imu_get_temperature_smp();

//        // 确保每次计算其初始值为0
//        for (index = 0; index < 3; index++)
//        {
//            avg[index] = 0;
//        }

//        times = 1 << pow2; // times = 2^pow2
//    }

//    if (remo_imu_data_refreshed())
//    {
//        switch (bias_type)
//        {
//        case CALI_GYRO_TYEP:
//        case CALI_ACCEL_TYEP:
//            avg[0] += imu_smp[0];
//            avg[1] += imu_smp[1];
//            avg[2] += imu_smp[2];
//            temperature_mean += *temp_smp;
//            break;
//        default:
//            break;
//        }

//        remo_imu_clear_data_refresh_flag();

//        // 此过程时间较长，需要喂狗
//        WDT_RefreshCounter();

//        if(times > 0)
//        {
//            times --;
//            if(times == 0)
//            {
//                // 求平均数
//                for (index = 0; index < 3; index++)
//                {
//                    avg[index] >>= pow2;
//                }
//                temperature_mean >>= pow2;
//                return true;
//            }
//            else
//            {
//                return false;
//            }
//        }
//    }
//    else
//    {
//        return false;
//    }
//}

///**************************************************************************************************
// * 函数名称: remo_imu_bias_variance
// * 输入参数: var->存储样本方差的地址；avg->均值地址；pow2->计算次数指数(times = 2^pow2 + 1);
// *           bias_type->imu校准类型，1:陀螺仪，2:加速度计
// * 返回结果: void
// * 功能描述: 计算imu采样结果的样本方差
//**************************************************************************************************/
//bool remo_imu_bias_variance(int32_t *var, int32_t *avg, uint8_t pow2, uint8_t bias_type)
//{
//    static const int16_t *imu_smp;
//    uint8_t index;
//    static uint16_t times = 0;
//    uint32_t temp[3];
//    static int32_t temp_var[3];
//    static uint16_t num = 512;

//    if (times == 0)
//    {
//        if (bias_type == CALI_GYRO_TYEP)
//        {
//            imu_smp = remo_imu_get_gyro_smp();
//        }
//        else if (bias_type == CALI_ACCEL_TYEP)
//        {
//            imu_smp = remo_imu_get_accel_smp();
//        }

//        times = (1 << pow2) + 1; // times = 2^pow2 + 1
//        num = (float)(times - 1);

//        // 确保每次计算其初始值为0
//        for (index = 0; index < 3; index++)
//        {
//            var[index] = 0;
//            temp_var[index] = 0.0f;
//        }
//    }

//    if (remo_imu_data_refreshed())
//    {
//        switch (bias_type)
//        {
//        case CALI_GYRO_TYEP:
//        case CALI_ACCEL_TYEP:
//            temp[0] = (imu_smp[0] - avg[0]) * (imu_smp[0] - avg[0]);
//            temp[1] = (imu_smp[1] - avg[1]) * (imu_smp[1] - avg[1]);
//            temp[2] = (imu_smp[2] - avg[2]) * (imu_smp[2] - avg[2]);
//            break;

//        default:
//            break;
//        }

//        temp_var[0] += temp[0] / num;
//        temp_var[1] += temp[1] / num;
//        temp_var[2] += temp[2] / num;

//        remo_imu_clear_data_refresh_flag();

//        // 此过程时间较长，需要喂狗
//        WDT_RefreshCounter();

//        if(times > 0)
//        {
//            times--;
//            if(times == 0)
//            {
//                // 样本方差
//                for (index = 0; index < 3; index++)
//                {
//                    var[index] = (int32_t)temp_var[index];
//                }
//                return true;
//            }
//            else
//            {
//                return false;
//            }
//        }
//    }
//    else
//    {
//        return false;
//    }
        


//}


///**************************************************************************************************
// * 函数名称: remo_imu_bias_average_variance
// * 输入参数: var->存储样本方差的地址；avg->新均值地址；avg->均值地址;pow2->计算次数指数(times = 2^pow2 + 1);
// *           bias_type->imu校准类型，1:陀螺仪，2:加速度计
// * 返回结果: bool
// * 功能描述: 计算imu采样结果的样本方差和此样本的均值
//**************************************************************************************************/
//static bool remo_imu_bias_average_variance(int32_t *var, int32_t *avg_new, int32_t *avg, uint8_t pow2, uint8_t bias_type)
//{
//    static const int16_t *imu_smp;
//    uint8_t index;
//    static uint16_t times = 0;
//    uint32_t temp[3];
//    static int32_t temp_var[3];
//    static uint16_t num = 512;

//    if (times == 0)
//    {
//        if (bias_type == CALI_GYRO_TYEP)
//        {
//            imu_smp = remo_imu_get_gyro_smp();
//        }
//        else if (bias_type == CALI_ACCEL_TYEP)
//        {
//            imu_smp = remo_imu_get_accel_smp();
//        }

//        times = (1 << pow2) + 1; // times = 2^pow2 + 1
//        num = times - 1;

//        // 确保每次计算其初始值为0
//        for (index = 0; index < 3; index++)
//        {
//            var[index] = 0;
//            temp_var[index] = 0.0f;
//            avg_new[index] = 0;
//        }
//    }

//    if (remo_imu_data_refreshed())
//    {
//        switch (bias_type)
//        {
//        case CALI_GYRO_TYEP:
//        case CALI_ACCEL_TYEP:
//            temp[0] = (imu_smp[0] - avg[0]) * (imu_smp[0] - avg[0]);
//            temp[1] = (imu_smp[1] - avg[1]) * (imu_smp[1] - avg[1]);
//            temp[2] = (imu_smp[2] - avg[2]) * (imu_smp[2] - avg[2]);
//            avg_new[0] += imu_smp[0];
//            avg_new[1] += imu_smp[1];
//            avg_new[2] += imu_smp[2];
//            break;
//        default:
//            break;
//        }

//        temp_var[0] += temp[0];
//        temp_var[1] += temp[1];
//        temp_var[2] += temp[2];

//        remo_imu_clear_data_refresh_flag();

//        // 此过程时间较长，需要喂狗
//        WDT_RefreshCounter();

//        if(times > 0)
//        {
//            times--;
//            if(times == 0)
//            {
//                // 样本方差和此样本均值
//                for (index = 0; index < 3; index++)
//                {
//                    var[index] = (int32_t)temp_var[index] / num;
//                    avg_new[index] >>= pow2;
//                }
//                return true;
//            }
//            else
//            {
//                return false;
//            }
//        }
//    }
//    else
//    {
//        return false;
//    }
//}

///**************************************************************************************************
// * 函数名称: remo_cali_accel_cali_smp
// * 输入参数: void
// * 返回结果: void
// * 功能描述: 加速度计采样，用于加速度计9参数校准
// *          串口为7的中值滤波，并分析串口内的均值和方差，如果方差太大则不选取
//**************************************************************************************************/
//static bool remo_cali_accel_fit_smp(int16_t accel_fit_smp[3])
//{
//    static int16_t accel_median_smp[3][7] = {0};
//    static uint8_t median_count = 0;

//    const int16_t *accel_smp;

//    int16_t accel_median[3] = {0};
//    int32_t accel_sum[3] = {0};
//    int16_t accel_mean[3] = {0};
//    int32_t accel_var[3] = {0};
//    int32_t accel_norm2 = 0;
    
//    int16_t temp = 0;
//    uint8_t i = 0;

//    accel_smp = remo_imu_get_accel_smp();

//    // 数据采样
//    accel_median_smp[0][median_count] = accel_smp[0];
//    accel_median_smp[1][median_count] = accel_smp[1];
//    accel_median_smp[2][median_count] = accel_smp[2];
//    median_count = (++median_count >= 7) ? 0:median_count;

//    // 对数据进行中值滤波
//    accel_median[0] = filter_median_i16w7(&accel_median_smp[0][0]);
//    accel_median[1] = filter_median_i16w7(&accel_median_smp[1][0]);
//    accel_median[2] = filter_median_i16w7(&accel_median_smp[2][0]);
//    for (i = 0; i < 7; i++)
//    {
//    accel_sum[0] += accel_median_smp[0][median_count];
//    accel_sum[1] += accel_median_smp[1][median_count];
//    accel_sum[2] += accel_median_smp[2][median_count];
//    }
//    // 求数据的均值和方差
//    accel_mean[0] = accel_sum[0] / 7;
//    accel_mean[1] = accel_sum[1] / 7;
//    accel_mean[2] = accel_sum[2] / 7;
//    for (i = 0; i < 7; i++)
//    {
//        temp = (accel_median_smp[0][i] - accel_mean[0]);
//        accel_var[0] += temp * temp;
//        temp = (accel_median_smp[1][i] - accel_mean[1]);
//        accel_var[1] += temp * temp;
//        temp = (accel_median_smp[2][i] - accel_mean[2]);
//        accel_var[2] += temp * temp;
//    }
//    // 求均值的模
//    accel_norm2 = accel_mean[0] * accel_mean[0] + accel_mean[1] * accel_mean[1] + accel_mean[2] * accel_mean[2];
//	accel_norm2 -= IMU_ACC_G_SQUARE_VALUE;    // 67108864 = 8192^2
	
//    if (abs(accel_norm2) < ACCEL_SMP_NORM2_TH)
//    {
//        accel_fit_smp[0] = accel_median[0];
//        accel_fit_smp[1] = accel_median[1];
//        accel_fit_smp[2] = accel_median[2];
//        return true;
//    }
//    else
//    {
//        return false;
//    }
//}

///**************************************************************************************************
// * 函数名称: remo_cali_accel_cali_LM
// * 输入参数: void
// * 返回结果: void
// * 功能描述: 加速度计9参数校准模型Levenberg-Marquardt法求解
//**************************************************************************************************/
//static bool remo_cali_accel_cali_LM(float *accel_smp, uint8_t *accel_smp_index, uint8_t accel_smp_num)
//{
//    float ret[9] = {0.0f};
//    float f = 0.0f;
//    float JTJ[9][9] = {0.0f};    // Jacobian矩阵
//    float invJTJ[9][9] = {0.0f}; // Jacobian矩阵的逆
//    float JTFI[9] = {0.0f};      // 偏导数
//    float d[9] = {0.0f};
//    uint8_t flag = 0;

//    float x = 0.0f, y = 0.0f, z = 0.0f;
//    float A = 0.0f, B = 0.0f, C = 0.0f;
//    float len = 0.0f;

//    uint8_t smp_index = 0;

//    static float f_sum_last = 0.0f;

//    uint8_t i = 0, j = 0, k = 0;

//    cali_accel.params[0] = 1.0f;
//    cali_accel.params[1] = 1.0f;
//    cali_accel.params[2] = 1.0f;
//    cali_accel.params[3] = 0.0f;
//    cali_accel.params[4] = 0.0f;
//    cali_accel.params[5] = 0.0f;
//    cali_accel.params[6] = 0.0f;
//    cali_accel.params[7] = 0.0f;
//    cali_accel.params[8] = 0.0f;

//    // 迭代次数
//    for (cali_accel.iter = 0; cali_accel.iter < CALI_FIT_LM_ITER_NUM; cali_accel.iter++)
//    {
//        cali_accel.f_sum = 0.0f;
//        for (j = 0; j < 9; j++)
//        {
//            for (k = 0; k < 9; k++)
//            {
//                JTJ[j][k] = 0.0f;
//            }
//            JTFI[j] = 0.0f;
//        }
//        // for (i = 0; i < ACCEL_FIT_SMP_NUM; i++)
//        for (i = 0; i < accel_smp_num; i++)
//        {
//            smp_index = accel_smp_index[i];
			
//			#ifndef ROLL_AXIS_EXISTENCE
//            cali_accel.params[1] = 1.0f;
//            cali_accel.params[3] = 0.0f;
//            cali_accel.params[5] = 0.0f;
//            cali_accel.params[7] = 0.0f;
//            #endif

//            // 减去偏置
//            x = accel_smp[smp_index * 3 + 0] + cali_accel.params[6];
//            y = accel_smp[smp_index * 3 + 1] + cali_accel.params[7];
//            z = accel_smp[smp_index * 3 + 2] + cali_accel.params[8];

//            // 非正交和Scale校正
//            A = cali_accel.params[0] * x + cali_accel.params[3] * y + cali_accel.params[4] * z;
//            B = cali_accel.params[3] * x + cali_accel.params[1] * y + cali_accel.params[5] * z;
//            C = cali_accel.params[4] * x + cali_accel.params[5] * y + cali_accel.params[2] * z;

//            // 求取加速度计的模（理想情况下len的值为1）
//            len = sqrtf(A * A + B * B + C * C);

//            // diag partial derivative
//            ret[0] = -1.0f * (x * A) / len;
//            ret[1] = -1.0f * (y * B) / len;
//            ret[2] = -1.0f * (z * C) / len;
//            // offdiag partial derivative
//            ret[3] = -1.0f * ((y * A) + (x * B)) / len;
//            ret[4] = -1.0f * ((z * A) + (x * C)) / len;
//            ret[5] = -1.0f * ((z * B) + (y * C)) / len;
//            // offset partial derivative
//            ret[6] = -1.0f * ((cali_accel.params[0] * A) + (cali_accel.params[3] * B) + (cali_accel.params[4] * C)) / len;
//            ret[7] = -1.0f * ((cali_accel.params[3] * A) + (cali_accel.params[1] * B) + (cali_accel.params[5] * C)) / len;
//            ret[8] = -1.0f * ((cali_accel.params[4] * A) + (cali_accel.params[5] * B) + (cali_accel.params[2] * C)) / len;
            

//            f = 1.0f - len;

//            cali_accel.f_sum += f;

//            for (j = 0; j < 9; j++)
//            {
//                for (k = 0; k < 9; k++)
//                {
//                    JTJ[j][k] += ret[j] * ret[k];
//                }
//                // 偏导数
//                JTFI[j] += ret[j] * f;
//            }
//        }

//        // 小于阈值则退出
//        if (fabsf(cali_accel.f_sum - f_sum_last) < 2e-6)
//        {
//            break;
//        }
//        f_sum_last = cali_accel.f_sum;

//        // 校准参数的迭代更新
//        for (j = 0; j < 9; j++)
//        {
//            JTJ[j][j] += ACCEL_FIT_LM_LAMBDA;
//        }

//        flag = remo_misc_matrix_inversion(&JTJ[0][0], 9, &invJTJ[0][0]);
//        if (flag)
//        {
//            remo_misc_matrix_multiply(9, 9, 1, d, &invJTJ[0][0], JTFI);
//            for (j = 0; j < 9; j++)
//            {
//                cali_accel.params[j] -= ACCEL_FIT_LM_ALPHA * d[j];
//            }
//            cali_accel.params[3] = remo_misc_constrainf(cali_accel.params[3], -0.01f, 0.01f);
//            cali_accel.params[4] = remo_misc_constrainf(cali_accel.params[4], -0.01f, 0.01f);
//            cali_accel.params[5] = remo_misc_constrainf(cali_accel.params[5], -0.01f, 0.01f);
//        }
//        else
//        {
//            f_sum_last = 0.0f;
//            return false;
//        }
//        // WDT_RefreshCounter();
//    }
//    f_sum_last = 0.0f;
//    return true;
//}

///**************************************************************************************************
// * 函数名称: remo_calibration_imu_trim
// * 输入参数: void
// * 返回结果: void
// * 功能描述: 姿态传感器装配误差计算，并保存在flash中。
// * 			陀螺仪的转换矩阵校准其实已经包含了装配误差，此时只针对加速度计
//**************************************************************************************************/
//void remo_calibration_imu_trim(void)
//{
////    if (1 == CALI_STATE.imu_trim)
////    {
////        remo_cali_imu_accel_trim();
////    }
//}

///**************************************************************************************************
// * 函数名称: remo_cali_init_joint_deg
// * 输入参数: void
// * 返回结果: void
// * 功能描述: 标定关节角0位（云台水平放置，欧拉角为0时对应的关节角）
//**************************************************************************************************/
//static void remo_cali_init_joint_deg(void)
//{
//    // 0:等待加速度稳定，1:校准中，2:校准完成
//    static uint16_t  accel_delay_count = 0;
//    static float smp_mean[3] = {0, 0, 0};
//    static uint16_t smp_mean_count = 0;
//    static const int16_t *accel_smp;
//    static const float *joint_smp;
//#ifndef YAW_360_DEG_ROTATION 
////    static uint8_t yaw_init_state = 0;
//    static float joint_limit_angle[2] = {0};
//    float joint_middle_angle = 0;
//#endif
//    float delta_joint_angle = 0.0f;
//    static uint32_t joint_offset_timer_us[2] = {0};

//    static uint32_t el_angle_timer_us[2] = {0};
//    static float joint_smp_last[3] = {0.0f};
//    static int32_t el_angle_offset[3] = {0};
//    static uint8_t el_angle_corr_state[3] = {0};
//    static float el_angle_delta_us = 0;

//    // 获取加速度值
//    accel_smp = remo_imu_get_accel_smp();
//    // 获取关节角
//    joint_smp = remo_hall_get_joint_deg_raw_ptr();

//#ifndef YAW_360_DEG_ROTATION 
//    switch(cali_status.joint_offset_state)
//    {
//        case JO_PREPARE:
//            smp_mean[0] = 0;
//            smp_mean[1] = 0;
//            smp_mean[2] = 0;
//            smp_mean_count = 0;
//            joint_limit_angle[0] = 0;
//            joint_limit_angle[1] = 0;
//            joint_offset_timer_us[0] =  remo_clock_timing_start_us();
//            joint_offset_timer_us[1] = 0;
//            el_angle_timer_us[0] =  remo_clock_timing_start_us();
//            el_angle_timer_us[1] = 0;
//            joint_smp_last[0] = joint_smp[0];
//            joint_smp_last[1] = joint_smp[1];
//            joint_smp_last[2] = joint_smp[2];
//            el_angle_offset[0] = remo_hall_get_el_angle_offset(0);
//            el_angle_offset[1] = remo_hall_get_el_angle_offset(1);
//            el_angle_offset[2] = remo_hall_get_el_angle_offset(2);
//            el_angle_corr_state[0] = 0;
//            el_angle_corr_state[1] = 0;
//            el_angle_corr_state[2] = 0;
//            remo_ctrl_loop_set_yaw_vel_limit(MOTOR_YAW_TORQUE_LIMIT*0.9);
//            cali_status.joint_offset_state = JO_YAW_UPPER_LIMIT;
//        case JO_YAW_UPPER_LIMIT:
//            // remo_motion_plan_set_aim_pos_deg(0, MOTION_CTRL_EULER, ROLL);
//            // remo_motion_plan_set_aim_pos_deg(0, MOTION_CTRL_EULER, PITCH);

//            // // yaw往极限位置运动
//            // cali_params.joint_angle_offset[2] = joint_smp[YAW];
//            // remo_hall_set_joint_offset(cali_params.joint_angle_offset[0], cali_params.joint_angle_offset[1], \
//            //                         cali_params.joint_angle_offset[2]);
//            // remo_motion_plan_set_aim_pos_deg(90, MOTION_CTRL_JOINT, YAW);
//            // remo_ctrl_loop_set_pos_kp_scale(4.0f, YAW);
//            // remo_ctrl_loop_set_vel_kp_scale(1.0f, YAW);    // 190

//            remo_ctrl_loop_set_pos_ctrl_enable_state(false, YAW);
//            remo_kinematics_set_vel_aim(5.0f, YAW);
//            remo_kinematics_set_joint_vel_ref(5.0f, YAW);

//            el_angle_timer_us[1] +=  remo_clock_timing_end_us(el_angle_timer_us[0]);
//            el_angle_timer_us[0] =  remo_clock_timing_start_us();
//            if (el_angle_timer_us[1] > 7e5)
//            {
//                el_angle_timer_us[1] = 0;
//                if (el_angle_corr_state[2] == 0)
//                {
//                    el_angle_corr_state[2] = 1;
//                }
//                else
//                {
//                    if (joint_smp[2] - joint_smp_last[2] < 1.5f && joint_smp[2] < 275.0f)
//                    {
//                        if (el_angle_corr_state[2] == 1)
//                        {
//                            remo_hall_set_el_angle_offset(el_angle_offset[2] - 2000, YAW);
//                            el_angle_corr_state[2] = 2;
//                        }
//                        else if (el_angle_corr_state[2] == 2)
//                        {
//                            remo_hall_set_el_angle_offset(el_angle_offset[2] + 2000, YAW);
//                            el_angle_corr_state[2] = 3;
//                        }
//                    }
//                }
//                joint_smp_last[2] = joint_smp[2];
//            }

//            // 延迟一段时间，等走到极限位置求平均值
//            joint_offset_timer_us[1] +=  remo_clock_timing_end_us(joint_offset_timer_us[0]);
//            joint_offset_timer_us[0] =  remo_clock_timing_start_us();
//            if (joint_offset_timer_us[1] > 53e6)
//            {
//                if ((smp_mean_count++) < 16)
//                {
//                    smp_mean[YAW] += joint_smp[YAW];
//                }
//                else
//                {
//                    joint_limit_angle[0] = smp_mean[YAW] / 16;
//                    smp_mean[YAW] = 0.0f;
//                    smp_mean_count = 0;
//                    cali_status.joint_offset_state = JO_YAW_LOWER_LIMIT;
//                    joint_offset_timer_us[1] = 0;

//                    el_angle_corr_state[2] = 0;
//                    joint_smp_last[2] = joint_smp[2];
//                    el_angle_timer_us[0] =  remo_clock_timing_start_us();
//                    el_angle_timer_us[1] = 0;
//                    remo_ctrl_loop_set_yaw_vel_limit(MOTOR_YAW_TORQUE_LIMIT*0.9);
//                }
//            }
//            else if (joint_offset_timer_us[1] > 50e6)
//            {
//                remo_ctrl_loop_set_yaw_vel_limit(MOTOR_YAW_TORQUE_LIMIT*1.15);
//                remo_kinematics_set_vel_aim(60.0f, YAW);
//                remo_kinematics_set_joint_vel_ref(60.0f, YAW);
//            }
//            break;
//        case JO_YAW_LOWER_LIMIT:
//            // yaw往另一极限位置运动
//            // cali_params.joint_angle_offset[2] = joint_smp[YAW];
//            // remo_hall_set_joint_offset(cali_params.joint_angle_offset[0], cali_params.joint_angle_offset[1], \
//            //                         cali_params.joint_angle_offset[2]);
//            // remo_motion_plan_set_aim_pos_deg(-90, MOTION_CTRL_JOINT, YAW);

//            remo_kinematics_set_vel_aim(-5.0f, YAW);
//            remo_kinematics_set_joint_vel_ref(-5.0f, YAW);

//            el_angle_timer_us[1] +=  remo_clock_timing_end_us(el_angle_timer_us[0]);
//            el_angle_timer_us[0] =  remo_clock_timing_start_us();
//            if (el_angle_timer_us[1] > 7e5)
//            {
//                el_angle_timer_us[1] = 0;
//                if (el_angle_corr_state[2] == 0)
//                {
//                    el_angle_corr_state[2] = 1;
//                }
//                else
//                {
//                    if (joint_smp[2] - joint_smp_last[2] > -1.5f && joint_smp[2] > 15.0f)
//                    {
//                        if (el_angle_corr_state[2] == 0)
//                        {
//                            remo_hall_set_el_angle_offset(el_angle_offset[2] + 2000, YAW);
//                            el_angle_corr_state[2] = 1;
//                        }
//                        else if (el_angle_corr_state[2] == 1)
//                        {
//                            remo_hall_set_el_angle_offset(el_angle_offset[2] - 2000, YAW);
//                            el_angle_corr_state[2] = 2;
//                        }
//                    }
//                }
//                joint_smp_last[2] = joint_smp[2];
//            }

//            // 延迟一段时间，等走到极限位置求平均值
//            joint_offset_timer_us[1] +=  remo_clock_timing_end_us(joint_offset_timer_us[0]);
//            joint_offset_timer_us[0] =  remo_clock_timing_start_us();

//            if (joint_offset_timer_us[1] > 102e6)
//            {
//                if ((smp_mean_count++) < 16)
//                {
//                    smp_mean[YAW] += joint_smp[YAW];
//                }
//                else
//                {
//                    joint_limit_angle[1] = smp_mean[YAW] / 16;
//                    joint_middle_angle = (joint_limit_angle[0] + joint_limit_angle[1]) / 2;
                    
//                    delta_joint_angle = fabs(joint_limit_angle[0] - joint_limit_angle[1]);
//                    if (delta_joint_angle  < 180.0f) delta_joint_angle = 360.0f - delta_joint_angle;
//                    if (fabs(delta_joint_angle - YAW_LIMIT_ANGLE) > 30.0f)
//                    {
//                        cali_status.joint_offset_state = ERR_JO_YAW_OFFSET;
//                        return;
//                    }
//                    else 
//                    {
//                        if (fabs(joint_limit_angle[0] - joint_limit_angle[1]) < 180.0f)
//                        {
//                            joint_middle_angle = (joint_middle_angle < 0) ? (joint_middle_angle + 180.0f):(joint_middle_angle - 180.0f);
//                        }
//                        cali_params.joint_angle_offset[2] = joint_middle_angle;
//                        smp_mean[YAW] = 0;
//                        smp_mean_count = 0;
//                        joint_limit_angle[0] = 0;
//                        joint_limit_angle[1] = 0;

//                        cali_status.joint_offset_state = JO_YAW_OFFSET;
//                        joint_offset_timer_us[1] = 0;
//                        remo_ctrl_loop_set_yaw_vel_limit(MOTOR_YAW_TORQUE_LIMIT);
//                    }   
//                }
//            }
//            else if (joint_offset_timer_us[1] > 98e6)
//            {
//                remo_ctrl_loop_set_yaw_vel_limit(MOTOR_YAW_TORQUE_LIMIT*1.15);
//                remo_kinematics_set_vel_aim(-60.0f, YAW);
//                remo_kinematics_set_joint_vel_ref(-60.0f, YAW);
//            }
//            break;
//        case JO_YAW_OFFSET:
//            remo_hall_set_joint_offset(cali_params.joint_angle_offset[0], cali_params.joint_angle_offset[1], \
//                                     cali_params.joint_angle_offset[2]);
//            cali_status.joint_offset_state = JO_PITCH_ROLL_OFFSET;
//            joint_offset_timer_us[0] =  remo_clock_timing_start_us();
//            joint_offset_timer_us[1] = 0;
//            break;
//        default:
//            break;
//    }
//#else 
//    smp_mean[0] = 0;
//    smp_mean[1] = 0;
//    smp_mean[2] = 0;
//    smp_mean_count = 0;
//    cali_status.joint_offset_state = JO_PITCH_ROLL_OFFSET;
//    joint_offset_timer_ms[0] =  remo_clock_timing_start_ms();
//#endif

//    // 校准前加速度要稳定
//    if (cali_status.joint_offset_state == JO_PITCH_ROLL_OFFSET)
//    {
//        // 关节角和加速度计校准标志的设置，校准完后都要延时一段时间
//        // 关节角零位置对齐的目标角
//        remo_motion_plan_set_aim_pos_deg(0, MOTION_CTRL_EULER, ROLL);
//        remo_motion_plan_set_aim_pos_deg(0, MOTION_CTRL_EULER, PITCH);
//        remo_motion_plan_set_aim_pos_deg(0, MOTION_CTRL_JOINT, YAW);

//        joint_offset_timer_us[1] +=  remo_clock_timing_end_us(joint_offset_timer_us[0]);
//        joint_offset_timer_us[0] =  remo_clock_timing_start_us();
//        if (joint_offset_timer_us[1] < 5000e3) 
//        {
//            accel_delay_count = 0;
//            return;
//        }

//        // 关节角对齐要等加速度计稳定后才行进行
//        if (accel_delay_count <= 15)
//        {
//            smp_mean[0] += accel_smp[0];
//            smp_mean[1] += accel_smp[1];
//            if ( ++smp_mean_count == JOINT_ANGLE_OFFSET_MEAN_NUM)
//            {
//                smp_mean[0] /= JOINT_ANGLE_OFFSET_MEAN_NUM;
//                smp_mean[1] /= JOINT_ANGLE_OFFSET_MEAN_NUM;
//            #ifdef ROLL_AXIS_EXISTENCE
//                if(fabs(smp_mean[0]) < 300 && fabs(smp_mean[1]) < 200)
//            #else
//                if(fabs(smp_mean[0]) < 900 && fabs(smp_mean[1]) < 900)
//            #endif
//                {
//                    accel_delay_count ++;
//                }
//                else accel_delay_count = 0;
//                smp_mean[0] = 0;
//                smp_mean[1] = 0;
//                smp_mean_count = 0;
//            }
//        }
//        else // 数据处理
//        {
//            smp_mean[0] += joint_smp[0];
//            smp_mean[1] += joint_smp[1];
//            smp_mean[2] += joint_smp[2];
//            if(++smp_mean_count == JOINT_ANGLE_OFFSET_MEAN_NUM)
//            {
//                smp_mean[0] /= JOINT_ANGLE_OFFSET_MEAN_NUM;
//                smp_mean[1] /= JOINT_ANGLE_OFFSET_MEAN_NUM;
//                smp_mean[2] /= JOINT_ANGLE_OFFSET_MEAN_NUM;
//                smp_mean_count = 0;
                
//                cali_params.joint_angle_offset[0] = smp_mean[0];
//                cali_params.joint_angle_offset[1] = smp_mean[1];
//                // cali_params.joint_angle_offset[2] = smp_mean[2];
//                remo_hall_set_joint_offset(cali_params.joint_angle_offset[0], cali_params.joint_angle_offset[1], \
//                                            cali_params.joint_angle_offset[2]);
//                cali_params.cali_flag.types.joint_angle_valid = 1;

//                cali_status.joint_offset_state = JO_CALI_FINISH;

//            }
//        }
//    }
    
//}

///**************************************************************************************************
// * 函数名称: remo_cali_gyro_temp_bias
// * 输入参数: void
// * 返回结果: void
// * 功能描述: 一定温度下陀螺仪零偏的校准。出厂电角度对齐的时候，给imu加热，校准一定温度下的零偏
//**************************************************************************************************/
//static bool remo_cali_gyro_temp_bias(void)
//{
//    static const int16_q7_t *gyro_smp;
//    static int32_t gyro_mean1[3] = {0}, gyro_mean2[3] = {0}; 
//    static uint16_t count = 0;
//    uint32_t mean_diff = 0.0f;
//    uint8_t i = 0;

//    gyro_smp = remo_imu_get_gyro_smp();

//    if (count == 0)
//    {
//        for (i = 0; i <= 2; i++)
//        {
//            gyro_mean1[i] = 0;
//            gyro_mean2[i] = 0;
//            gyro_temp_bias[i] = 0;
//        }
//    }

//    count ++;
//    if (count <= 256)
//    {
//        gyro_mean1[0] += gyro_smp[0];
//        gyro_mean1[1] += gyro_smp[1];
//        gyro_mean1[2] += gyro_smp[2];
//    }
//    else if (count <= 512)
//    {
//        gyro_mean2[0] += gyro_smp[0];
//        gyro_mean2[1] += gyro_smp[1];
//        gyro_mean2[2] += gyro_smp[2];
//    }
//	else
//	{
//		count = 0;
//	}

//    if (count == 512)
//    {
//        for (i = 0; i <= 2; i++)
//        {
//            gyro_mean1[i] >>= 8;
//            gyro_mean2[i] >>= 8;
//            gyro_temp_bias[i] = (gyro_mean1[i] + gyro_mean2[i]) >> 1;
//        }

//        mean_diff = abs(gyro_mean1[0] - gyro_mean2[0]) + abs(gyro_mean1[1] - gyro_mean2[1]) +
//                   abs(gyro_mean1[2] - gyro_mean2[2]);

        
//        if (mean_diff < CALI_GYRO_AVG_DIFF_TH && 
//                abs(gyro_temp_bias[0]) < imu_cali_state.gyro_state_th && 
//                abs(gyro_temp_bias[1]) < imu_cali_state.gyro_state_th &&
//                abs(gyro_temp_bias[2]) < imu_cali_state.gyro_state_th)
//		{
//            return true;
//        }

//        count = 0;
//    }

//    return false;
//}




///**************************************************************************************************
// * 函数名称: remo_cali_reset_accel_params
// * 输入参数: void
// * 返回结果: void
// * 功能描述: 重置加速度校准参数
// * 			校准异常时，参数错误引起姿态错误，再次校准可能无法达到目标位置，因此需要重置
//**************************************************************************************************/
//void remo_cali_reset_accel_params(void)
//{

//        // 重置加速度计转换矩阵数据
//        cali_params.accel_offset[0] = 0;
//        cali_params.accel_offset[1] = 0;
//        cali_params.accel_offset[2] = 0;
//        remo_imu_set_accel_bias(cali_params.accel_offset[0], cali_params.accel_offset[1], \
//                        cali_params.accel_offset[2]);
    
//        cali_params.accel_matrix[0] = IMU_ACC_TO_MS2;
//        cali_params.accel_matrix[1] = 0;
//        cali_params.accel_matrix[2] = 0;
//        cali_params.accel_matrix[3] = 0;
//        cali_params.accel_matrix[4] = IMU_ACC_TO_MS2;
//        cali_params.accel_matrix[5] = 0;
//        cali_params.accel_matrix[6] = 0;
//        cali_params.accel_matrix[7] = 0;
//        cali_params.accel_matrix[8] = IMU_ACC_TO_MS2;
//        remo_imu_set_accel_trans_matrix(cali_params.accel_matrix);
//        // 此处fresh可重擦也可不擦。但擦除时间过长，会引起电调误判没指令下发
//        // remo_flash_local_refresh(ACC_ADDR_OFFSET, remo_imu_get_accel_bias(), 3);
//        // remo_flash_local_refresh(BAIS_ADDR_OFFSET + 80, remo_imu_get_accel_trans_matrix(), 9);
//}

///**************************************************************************************************
// * 函数名称: remo_cali_get_CALI_STATE
// * 输入参数: void
// * 返回结果: void
// * 功能描述: 获取校准状态结构体变量
//**************************************************************************************************/
//cali_status_t *remo_cali_get_cali_status(void)
//{
//    return &cali_status;
//}

///**************************************************************************************************
// * 函数名称: remo_cali_set_CALI_STATE
// * 输入参数: state->校准状态
// * 返回结果: void
// * 功能描述: 设置校准状态
//**************************************************************************************************/
//void remo_cali_set_CALI_STATE(cali_state_t state)
//{
//    if (state < CALI_ERR_STRT)
//    {
//        cali_status.cali_state = state;
//    }
//}

///**************************************************************************************************
// * 函数名称: remo_cali_factory_cali_finish
// * 输入参数: void
// * 返回结果: void
// * 功能描述: 产线云台校准之后的动作提示完成校准完成
//**************************************************************************************************/
//static void remo_cali_factory_cali_finish(void)
//{
//    /*
//    static uint16_t count = 0, delay_count = 0;
//    if (CALI_STATE.CALI_STATE == CALI_STATE_IDLE && CALI_STATE.CALI_STATE_last != CALI_STATE_IDLE)
//    {
//        // 防止重心靠后，直接跳出
//        if (delay_count < 5000)
//        {
//            delay_count ++;
//        }
//        else if (remo_ahrs_get_roll_axis_slope() > 0.9f)
//        {
//            CALI_STATE.CALI_STATE_last = CALI_STATE_IDLE;
//            count = 0;
//            delay_count = 0;
//            return;
//        }

//        count ++;
//        if (CALI_STATE.CALI_STATE_last == CALI_STATE_MOTOR_ALIGN)
//        {
//            #ifdef YAW_360_DEG_ROTATION 
//            if (count == 3000)   //计数器每隔2000转动方向转换一次
//            #else
//            if (count == 3000)   //计数器每隔2000转动方向转换一次
//            #endif
//            {
//                if (yaw_motor_align_motion_flag)
//                {
//                    remo_motion_plan_set_aim_pos_deg(PI1_2, MOTION_CTRL_JOINT, YAW);
//                }
//                remo_motion_plan_set_aim_pos_deg(PI1_2 * 0.5f, MOTION_CTRL_JOINT, PITCH);
//            }
//            else if (count == 6000)
//            {
//                if (yaw_motor_align_motion_flag)
//                {
//                    remo_motion_plan_set_aim_pos_deg(-PI1_2, MOTION_CTRL_JOINT, YAW);
//                }
//                remo_motion_plan_set_aim_pos_deg(-PI1_2 * 0.5f, MOTION_CTRL_JOINT, PITCH);
//                count = 1;
//            }
//        }
//        else if (CALI_STATE.CALI_STATE_last == CALI_STATE_DYNAMIC)
//        {
//            if (count == 3000)   //计数器每隔2000转动方向转换一次
//            {
//                remo_motion_plan_set_aim_pos_deg(PI1_2 * 0.4f, MOTION_CTRL_JOINT, PITCH);
//            }
//            else if (count == 6000)
//            {
//                remo_motion_plan_set_aim_pos_deg(-PI1_2 * 0.4f, MOTION_CTRL_JOINT, PITCH);
//                count = 1;
//            }
//        }
//    }
//    else 
//    {
//        count = 0;
//    }
//    */
//}


///**************************************************************************************************
// * 函数名称: remo_cali_roll_ft
// * 输入参数: void
// * 返回结果: void
// * 功能描述: roll轴微调的处理
//**************************************************************************************************/
//static void remo_cali_roll_ft(void)
//{
//    /*
//    static uint16_t roll_ft_reflash_count = 0, roll_ft_stop_count = 0;
//    static uint8_t roll_ft_flag_last = 0;
//    static float roll_ft_rad_last[2] = {0.0f, 0.0f};

//#ifndef ROLL_AXIS_EXISTENCE
//    if (remo_motion_cmd_get_roll_ft_flag() != 0)
//    {
//        CALI_STATE.CALI_STATE = CALI_STATE_ROLL_FT;
//        return;
//    }
//    else if (roll_ft_flag_last != 0 && remo_motion_cmd_get_roll_ft_flag() == 0)
//    {
//        CALI_STATE.CALI_STATE = CALI_STATE_IDLE;
//        cali_reset_roll_ft_rad_flag = 0;
//        roll_ft_reflash_count = 0;
//    }

//    roll_ft_flag_last = remo_motion_cmd_get_roll_ft_flag();
//#endif

//    // 校准时应该排除roll微调参数的影响，校准之后也应把roll微调值设置为0
//    if(remo_cali_get_cali_executing())
//    {
//        if(cali_reset_roll_ft_rad_flag == 0)
//        {
//            cali_reset_roll_ft_rad_flag = 1;
//            remo_motion_set_roll_finetune_rad(cali_roll_ft_rad_temp[0], 0);
//            remo_motion_set_roll_finetune_rad(cali_roll_ft_rad_temp[1], 1);
//        }
//    }

//    cali_roll_ft_rad_temp[0] = remo_motion_cmd_get_roll_ft_rad(0);
//    cali_roll_ft_rad_temp[1] = remo_motion_cmd_get_roll_ft_rad(1); 

//    // 长时间(4000/2000 = 30s))roll轴微调值没有改变，则直接判断跳出roll轴微调操作， 不保存数据
//    if (roll_ft_rad_last[0] == cali_roll_ft_rad_temp[0] && roll_ft_rad_last[1] == cali_roll_ft_rad_temp[1])
//    {
//        roll_ft_stop_count ++;
//        if (roll_ft_stop_count >= 60000)
//        {
//            remo_motion_cmd_reset_roll_ft_flag();
//            roll_ft_stop_count = 0;
//            return;
//        }
//    }
//    else
//    {
//        roll_ft_stop_count = 0;
//    }
//    roll_ft_rad_last[0] = cali_roll_ft_rad_temp[0];
//    roll_ft_rad_last[1] = cali_roll_ft_rad_temp[1];

//    // 设置roll轴微调标志
//    if (remo_motion_cmd_get_roll_ft_flag() != 0)
//    {
//        CALI_STATE.CALI_STATE = CALI_STATE_ROLL_FT;
//    }

//    // 针对roll轴微调指令下不保存直接退出的情况
//    if (roll_ft_flag_last != 0 && remo_motion_cmd_get_roll_ft_flag() == 0)
//    {
//        CALI_STATE.CALI_STATE = CALI_STATE_IDLE;
//        cali_reset_roll_ft_rad_flag = 0;
//        roll_ft_reflash_count = 0;
//    }

//    // 针对roll轴微调指令下数据的保存，先停电机，后写入flash,然后再启动电机
//    if (remo_motion_cmd_get_roll_ft_flag() == 2)
//    {
//        roll_ft_reflash_count ++;
//        if (cali_reset_roll_ft_rad_flag == 0)
//        {
//            if (remo_motor_get_motor_state() == MOTOR_RUN)
//            {
//                // 关闭电机
//                if(roll_ft_reflash_count > 2000)
//                {
//                    remo_esc_set_motors_stop();
//                    roll_ft_reflash_count = 1;
//                    return;
//                }
//            }
//            else if (remo_motor_get_motor_state() == MOTOR_STOP)
//            {
//                cali_reset_roll_ft_rad_flag = 1;
//            }
//        }
//        else if(cali_reset_roll_ft_rad_flag == 2)
//        {
//            if (remo_motor_get_motor_state() != MOTOR_RUN)
//            {
//                // 开启电机
//                if(roll_ft_reflash_count > 2000)
//                {
//                    remo_esc_motor_start_cmd_send();
//                    roll_ft_reflash_count = 1;
//                    return;
//                }
//            }
//            else
//            {
//                remo_motion_cmd_reset_roll_ft_flag();
//                CALI_STATE.CALI_STATE = CALI_STATE_IDLE;
//                cali_reset_roll_ft_rad_flag = 0;
//                roll_ft_reflash_count = 0;
//            }
//        }
//    }

//    roll_ft_flag_last = remo_motion_cmd_get_roll_ft_flag();

//    // roll轴微调写flash
//    if(CALI_STATE.CALI_STATE == CALI_STATE_ROLL_FT && cali_reset_roll_ft_rad_flag == 1 && roll_ft_reflash_count > 6000)
//    {
//        if(remo_motion_cmd_get_roll_ft_flag() == 2)
//        {
//            remo_flash_local_gyro_refresh(ROLL_ADDR_TUNE, &cali_roll_ft_rad_temp[0], 2); 
//            cali_roll_ft_rad_temp[0] = 0.0f;
//            cali_roll_ft_rad_temp[1] = 0.0f;
//            cali_reset_roll_ft_rad_flag = 2;
//        }
//    }
//    */
//}


///**************************************************************************************************
// * 函数名称: remo_cali_roll_offset_by_limit
// * 输入参数: float *校准值
// * 返回结果: bool 校准完成标志
// * 功能描述: roll轴寻极限找零偏值
//**************************************************************************************************/
//float roll_accel_limit[2] = {0.0f};
//static bool remo_cali_roll_offset_by_limit(int16_t *roll_offset)
//{
//    /*
//    const float *euler_smp_rad, *accel;
//    // static float roll_accel_limit[2] = {0.0f};

//    static float roll_accel_mean = 0.0f, roll_accel_mean_last = 0.0f;
//    static uint16_t roll_accel_count = 0;
//    static uint8_t  roll_reach_limit_flag = 0;

//    // 获取欧拉角
//    euler_smp_rad = remo_ahrs_get_euler();
//    // 获取加速度
//    accel = remo_imu_get_accel();

//    if (fabs(euler_smp_rad[1]) < 0.01f)
//    {
//        if (roll_reach_limit_flag == 0)
//        {
//            remo_motion_plan_set_aim_pos_deg(145.0f * DEG_TO_RAD, MOTION_CTRL_EULER, ROLL);
//        }
//        else if (roll_reach_limit_flag == 1)
//        {
//            remo_motion_plan_set_aim_pos_deg(-145.0f * DEG_TO_RAD, MOTION_CTRL_EULER, ROLL);
//        }
//    }

//    roll_accel_count ++;
//    roll_accel_mean += accel[1];
//    if(roll_accel_count == 500)
//    {
//        roll_accel_mean /= (float)roll_accel_count;
//        if(fabs(roll_accel_mean - roll_accel_mean_last) < 20)
//        {
//            if (fabs(euler_smp_rad[0] - 135.0f * DEG_TO_RAD) < 0.2f)
//            { 
//                roll_accel_limit[0] = 0.5f *(roll_accel_mean + roll_accel_mean_last);
//                roll_reach_limit_flag = 1;
//            }
//            else if (fabs(euler_smp_rad[0] + 135.0f * DEG_TO_RAD) < 0.2f)
//            { 
//                roll_accel_limit[1] = 0.5f *(roll_accel_mean + roll_accel_mean_last);
//                roll_reach_limit_flag = 2;
//            }
//        }
//        roll_accel_mean_last = roll_accel_mean;

//        roll_accel_count = 0;
//    }

//    if(roll_reach_limit_flag == 2)
//    {
//        *roll_offset = (int16_t)(0.5f * (roll_accel_limit[0] + roll_accel_limit[1]));
//        remo_motion_plan_set_aim_pos_deg(0.0f, MOTION_CTRL_EULER, ROLL);
//        roll_accel_limit[0] = 0.0f;
//        roll_accel_limit[1] = 0.0f;
//        roll_accel_mean = 0.0f;
//        roll_accel_mean_last = 0.0f;
//        roll_accel_count = 0;
//        roll_reach_limit_flag = 0;

        

//        return true;
//    }
//    else
//    {
//        return false;
//    }
//*/
//}

///**************************************************************************************************
// * 函数名称: remo_cali_gyro_trans
// * 输入参数: void
// * 返回结果: void
// * 功能描述: 陀螺仪9参数模型校准
//**************************************************************************************************/
//static void remo_cali_gyro_trans(void)
//{
//    /*
//    static uint32_t gyro_dynamic_cnt = 0;

//    if(CALI_STATE.elec_angle_valid == 0 || CALI_STATE.motor_joint_valid == 0)
//    {
//        CALI_STATE.CALI_STATE = CALI_STATE_IDLE;
//        return;
//    }

//    if (gyro_dynamic_cnt == 0)
//    {
//        CALI_STATE.imu_gyro_trans = 1;
//    }

//     // 启动电机
//    if (remo_motor_get_motor_state() != MOTOR_RUN && CALI_STATE.imu_gyro_trans == 1)
//    {
//        // 启动电机
//        gyro_dynamic_cnt ++;
//        if(gyro_dynamic_cnt > 2000)
//        {
//            remo_esc_motor_start_cmd_send();
//            gyro_dynamic_cnt = 1;
//        }
//        return;
//    }

////    remo_motor_set_velo_pid_nonlinear_param(1500.0f, -1000.0f, 20.0f, 300.0f, -150.0f, 30.0f, 0.0f, 0.0f, 0.0f, YAW);
////    remo_motor_set_velo_2nd_nf_param(2000, 0, 0.0f, YAW);
////    remo_motor_velo_set_lead_lag_param(0.0005, 20, 42, 400, 300, YAW);

//    if (CALI_STATE.imu_gyro_trans == 1)
//    {
//        remo_cali_imu_rough_gyro_9params();
//		gyro_dynamic_cnt = 1;
//    }
//    else
//    {
//        // remo_motor_set_posi_p_nonlinear_param( 0.0f,  0.0f, 30.0f, YAW);
//        remo_motion_plan_set_aim_pos_deg(0.0f, MOTION_CTRL_EULER, ROLL);
//        remo_motion_plan_set_aim_pos_deg(0.0f, MOTION_CTRL_EULER, PITCH);
//        remo_motion_plan_set_aim_pos_deg(0.0f, MOTION_CTRL_JOINT, YAW);
////        remo_motor_reset_posi_roll();
////        remo_motor_reset_posi_pitch();
////        remo_motor_reset_posi_yaw();

//        gyro_dynamic_cnt ++;
//        if (remo_motor_get_motor_state() == MOTOR_RUN)
//        {
//            // 关闭电机
//            if(gyro_dynamic_cnt > 2000)
//            {
//                remo_esc_set_motors_stop();
//                gyro_dynamic_cnt = 1;
//            }
//        }
//        else if(gyro_dynamic_cnt > 6000)
//        {
//            // 写入陀螺仪转换矩阵数据
//            remo_flash_local_refresh(GYRO_ADDR_MATRIX, remo_imu_get_gyro_trans_matrix(), 9);
//            // roll轴偏置重置为0
//            remo_flash_local_gyro_refresh(ROLL_ADDR_TUNE, &cali_roll_ft_rad_temp[0], 2); 
//            cali_reset_roll_ft_rad_flag = 0;

//            gyro_dynamic_cnt = 0;
//            CALI_STATE.CALI_STATE = CALI_STATE_IDLE;
//        }
//    }
//    */
//}


///**************************************************************************************************
// * 函数名称: remo_cali_user_accel_trans
// * 输入参数: void
// * 返回结果: void
// * 功能描述: 用户加速度9参数模型校准
//**************************************************************************************************/
//float mytest_time = 0.0f;
//static void remo_cali_user_accel_trans(void)
//{
//    /*
//    float temp = 0.0f;
//    static uint32_t accel_dynamic_cnt = 0;
//    const float *euler_smp_rad;
//    const float *base_dcmz;
//    static uint16_t percentage_count = 0;
//    static bool accel_params_reflash_flag = false;
//    static uint32_t cali_time_start, cali_time;
//    static motion_roll_mode_t roll_hv_mode = 0;

//    // 获取欧拉角
//    euler_smp_rad = remo_ahrs_get_euler();
//    // 获取关节角
//    base_dcmz = remo_ahrs_get_base_dcmz();

//    if(accel_dynamic_cnt == 0)
//    {
//        // 确保基座处于水平状态
//        if(fabs(base_dcmz[2] - 1.0f) > 0.05f)
//        {
//            CALI_STATE.CALI_STATE = CALI_ERR_ACCELTRANS;
//            return;
//        }
//        user_accel_cali_percentage = 0;
//        cali_time_start = remo_timing_start();
//        imu_CALI_STATE.accel_state = 0;
//        CALI_STATE.imu_accel_reset = 1;
//        CALI_STATE.imu_accel_trans = 1;
//        accel_cali_start_flag = 0;
//        // 强制重置为0，以便再次校准
//        CALI_STATE.accel_bias_valid = 0;
//        remo_cali_reset_accel_params();

//        remo_cali_accel_trans_ctrl_params();
//        remo_motion_plan_set_aim_pos_deg(0.0f, MOTION_CTRL_JOINT, YAW);

//        accel_dynamic_cnt = 1;

//        roll_hv_mode = remo_motion_get_roll_mode();
//    }
    
//     // 启动电机，正常用户校准是不会跑到这的
//    if (remo_motor_get_motor_state() != MOTOR_RUN && CALI_STATE.imu_accel_trans == 1)
//    {
//        // 启动电机
//        accel_dynamic_cnt ++;
//        if(accel_dynamic_cnt > 2000)
//        {
//            remo_esc_motor_start_cmd_send();
//            accel_dynamic_cnt = 1;
//        }
//        return;
//    }

//    remo_cali_imu_accel_9params_sample_LM();

//    if (CALI_STATE.imu_accel_trans == 0)
//    {
//        // 计算校准精度百分比存储部分
//        percentage_count ++;
//        user_accel_cali_percentage = (uint8_t)( 8.0f * percentage_count / 8000.0f) + 90;
//        user_accel_cali_percentage = (user_accel_cali_percentage > 98) ? 98:user_accel_cali_percentage;

//        accel_dynamic_cnt ++;
//        if (remo_motor_get_motor_state() == MOTOR_RUN)
//        {
//            if(accel_dynamic_cnt > 2000)
//            {
//                remo_esc_set_motors_stop();
//                accel_dynamic_cnt = 1;
//            }
//        }
//        else if(accel_dynamic_cnt > 8000)
//        {
//            if (!accel_params_reflash_flag)
//            {
//                // 写入加速度偏置数据
//                remo_flash_local_refresh(ACC_ADDR_OFFSET, remo_imu_get_accel_bias(), 3);
//                // 写入加速度计转换矩阵数据
//                remo_flash_local_refresh(ACCEL_ADDR_MATRIX, remo_imu_get_accel_trans_matrix(), 9);
//                // roll轴偏置重置为0
//                remo_flash_local_gyro_refresh(ROLL_ADDR_TUNE, &cali_roll_ft_rad_temp[0], 2); 
//                accel_params_reflash_flag = true;
//            }
//            else if (remo_ahrs_get_ahrs_err_sum() < 0.025f)
//            {
//                cali_reset_roll_ft_rad_flag = 0;
//                CALI_STATE.CALI_STATE = CALI_STATE_IDLE;
//                if (roll_hv_mode == MOTION_ROLL_VERT)
//                {
//                    remo_motion_plan_set_aim_pos_deg(PI1_2, MOTION_CTRL_EULER, ROLL);
//                }
//                else
//                {
//                    remo_motion_plan_set_aim_pos_deg(0.0f, MOTION_CTRL_EULER, ROLL);
//                }
//                remo_motion_plan_set_aim_pos_deg(0.0f, MOTION_CTRL_EULER, PITCH);
//                remo_motion_plan_set_aim_pos_deg(0.0f, MOTION_CTRL_JOINT, YAW);
//                remo_motor_reset_roll_velo_param();
//                remo_motor_reset_pitch_velo_param();
//                remo_motor_reset_yaw_velo_param();
//                accel_dynamic_cnt = 0;
//                user_accel_cali_percentage = 100;
//                percentage_count = 0;
//                accel_params_reflash_flag = false;


//            }
//        }
//        // remo_ahrs_init();
//    }  

//    // 计算校准耗时
//    cali_time = remo_timing_end(cali_time_start);
//    mytest_time = cali_time * TIME_RESOLUTION;
//    // 如果长时间不过，则认为imu陀螺仪错误
//    if (cali_time * TIME_RESOLUTION > 250.0f)
//    {
//        // 清除偏置更新标志位后返回
//        accel_dynamic_cnt = 0;
//        percentage_count = 0;
//        accel_params_reflash_flag = false;
//        cali_reset_roll_ft_rad_flag = 0;
//        imu_CALI_STATE.accel_state = 2;
//        CALI_STATE.CALI_STATE = CALI_ERR_ACCELTRANS;
//        remo_motion_plan_set_aim_pos_deg(0.0f, MOTION_CTRL_EULER, ROLL);
//        remo_motion_plan_set_aim_pos_deg(0.0f, MOTION_CTRL_EULER, PITCH);
//        remo_motion_plan_set_aim_pos_deg(0.0f, MOTION_CTRL_JOINT, YAW);
//    }
//*/
//}



///**************************************************************************************************
// * 函数名称: remo_cali_yaw_initial_position
// * 输入参数: void
// * 返回结果: void
// * 功能描述: yaw轴中间位置
//**************************************************************************************************/
//static void remo_cali_yaw_initial_position(void)
//{
//    /*
//    static uint32_t initial_position_cnt = 0;
//	static float yaw_joint_offset_mean = 0;
//	const float *joint_smp_rad, *joint_offset_deg;
	
//	// 获取关节角
//    joint_smp_rad = remo_ahrs_get_joint_angle();
//    joint_offset_deg = remo_motion_get_joint_offset_deg();

//	if (initial_position_cnt == 0)
//	{
//		// remo_motion_set_joint_offset_deg(0.0f, 2);
//	}

//	initial_position_cnt ++;

//	if (initial_position_cnt < 500)
//	{
//		yaw_joint_offset_mean += joint_smp_rad[2] * RAD_TO_DEG + joint_offset_deg[2];
//	}
//	else if (initial_position_cnt == 500)
//	{
//		yaw_joint_offset_mean /= 500.0f;
//		if (yaw_joint_offset_mean > 180.0f)
//			yaw_joint_offset_mean -= 360.0f;
//		else if (yaw_joint_offset_mean < -180.0f)
//			yaw_joint_offset_mean += 360.0f;
//    }
//    else
//    {
		
//		if (remo_motor_get_motor_state() == MOTOR_RUN)
//        {
//            // 关闭电机
//            if(initial_position_cnt > 2500)
//            {
//                remo_esc_set_motors_stop();
//                initial_position_cnt = 500;
//            }
//        }
//        else if(initial_position_cnt > 3000)
//        {
//			// 写入yaw轴初始位置
//            if (initial_position_cnt == 3001)
//            {
//                remo_motion_set_joint_offset_deg(yaw_joint_offset_mean, 2);
//			    remo_flash_local_refresh(JOINT_ADDR_OFFSET + 8, &yaw_joint_offset_mean, 1);
//            }
//            else
//            {
//                if (remo_motor_get_motor_state() == MOTOR_STOP)
//                {
//                    // 开启电机
//                    if(initial_position_cnt > 3500)
//                    {
//                        remo_esc_motor_start_cmd_send();
//                        initial_position_cnt = 3100;
//                    }
//                }
//                else
//                {
//                    initial_position_cnt = 0;
//                    yaw_joint_offset_mean = 0;

//                    CALI_STATE.CALI_STATE = CALI_STATE_IDLE;
//					remo_motion_plan_set_aim_pos_deg(0.0f, MOTION_CTRL_JOINT, YAW);
//                }
//            }
//        }
//	}
//    */

//}
//#endif

/**************************************************************************************************
 * 函数名称: remo_cali_motor_alignment_error_check
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 电机电角度对齐错误检测
 *          通过角速度进行判断
**************************************************************************************************/
static void remo_cali_motor_alignment_error_check(void)
{
}

/**************************************************************************************************
 * 函数名称: remo_cali_set_cali_state
 * 输入参数: cali_state_t
 * 返回结果: void
 * 功能描述: 设置校准状态
**************************************************************************************************/
void remo_cali_set_cali_state(cali_state_t state)
{
    cali_status.cali_state = state;
}

/**************************************************************************************************
 * 函数名称: remo_cali_get_motor_run_cali_flag
 * 输入参数: void
 * 返回结果: bool
 * 功能描述: 满足电机启动的校准条件
**************************************************************************************************/
bool remo_cali_get_motor_run_cali_flag(void)
{ 
    return cali_params.cali_flag.types.elec_angle_valid && cali_params.cali_flag.types.joint_angle_valid;
}

/**************************************************************************************************
 * 函数名称: remo_cali_get_velo_euler_flag
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 获取角速度解耦是否用姿态进行求解标志
**************************************************************************************************/
uint8_t remo_cali_get_velo_euler_flag(void)
{
//    return (cali_status.motor_joint && (cali_status.CALI_STATE == CALI_STATE_DYNAMIC));
}

/**************************************************************************************************
 * 函数名称: remo_cali_get_elec_angle_flag
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 获取是否电角度对齐标志
**************************************************************************************************/
bool remo_cali_get_elec_angle_flag(void)
{
    return (cali_params.cali_flag.types.elec_angle_valid);
}

/**************************************************************************************************
 * 函数名称: remo_cali_get_motor_joint_valid
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 获取磁编码零角度是否对齐标志
**************************************************************************************************/
bool remo_cali_get_motor_joint_valid(void)
{
    return (cali_params.cali_flag.types.joint_angle_valid);
}

/**************************************************************************************************
 * 函数名称: remo_cali_get_accel_bias_valid
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 获取加速度计校准参数
**************************************************************************************************/
bool remo_cali_get_accel_bias_valid(void)
{
//    return cali_status.accel_bias_valid;
}

/**************************************************************************************************
 * 函数名称: remo_cali_set_accel_bias_valid
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 设置加速度计校准参数
**************************************************************************************************/
void remo_cali_set_accel_bias_valid(uint8_t flag)
{
//    cali_status.accel_bias_valid = flag;
}

/**************************************************************************************************
 * 函数名称: remo_cali_get_cali_running
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 获取是否校准校准完成，第一步和第二步校准中间时也为真
**************************************************************************************************/
bool remo_cali_get_cali_running(void)
{
    if(remo_cali_get_cali_executing() || !remo_cali_get_motor_joint_valid())
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**************************************************************************************************
 * 函数名称: remo_cali_get_cali_executing
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 获取是否在进行校准
**************************************************************************************************/
bool remo_cali_get_cali_executing(void)
{
    if(cali_status.cali_state >= CALI_STATE_MOTOR_ALIGN && cali_status.cali_state <= CALI_YAW_HALL_FIT)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**************************************************************************************************
 * 函数名称: remo_cali_get_cali_state
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 获取是校准状态
**************************************************************************************************/
cali_state_t remo_cali_get_cali_state(void)
{
    return cali_status.cali_state;
}

/**************************************************************************************************
 * 函数名称: remo_cali_get_user_accel_succeed_flag
 * 输入参数: void
 * 返回结果: bool
 * 功能描述: 设置加速度计校准参数
**************************************************************************************************/
bool remo_cali_get_user_accel_succeed_flag(void)
{
    if(cali_status.cali_state == CALI_ERR_ACCELTRANS)
    {
        return false;
    }
    else
    {
        return true;
    }
}

///**************************************************************************************************
// * 函数名称: remo_cali_accel_trans_ctrl_params
// * 输入参数: void
// * 返回结果: void
// * 功能描述: 加速度计转换矩阵校准时的速度环控制参数
//**************************************************************************************************/
//static void remo_cali_accel_trans_ctrl_params(void)
//{
////    remo_motor_set_velo_param(500.0f, 0.0f, 20.0f, CTRL_PID_KP, ROLL);
////    remo_motor_set_velo_param(70.0f, 0.0f, 20.0f, CTRL_PID_KI, ROLL);
////    remo_motor_set_velo_param(0.0f, 0.0f, 0.0f, CTRL_PID_KD, ROLL);

////    remo_motor_set_velo_param(800.0f, -300.0f, 20.0f, CTRL_PID_KP, PITCH);
////    remo_motor_set_velo_param(200.0f, 0.0f, 20.0f, CTRL_PID_KI, PITCH);
////    remo_motor_set_velo_param(0.0f, 0.0f, 0.0f, CTRL_PID_KD, PITCH);

////    remo_motor_set_velo_param(1700.0f, 000.0f, 20.0f, CTRL_PID_KP, YAW);
////    remo_motor_set_velo_param(150.0f, 0.0f, 20.0f, CTRL_PID_KI, YAW);
////    remo_motor_set_velo_param(0.0f, 0.0f, 0.0f, CTRL_PID_KD, YAW);
//}


/**************************************************************************************************
 * 函数名称: remo_cali_get_CALI_STATE_log
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 获取校准状态或imu校准质量
 *          0：未进行校准
 *          1：正在校准中
 *          2：校准正常
 *          3：陀螺仪校准误差大
 *          4：加速度校准误差大
 *          5：陀螺仪和加速度校准误差大
**************************************************************************************************/
uint8_t remo_cali_get_CALI_STATE_log(void)
{
   if(remo_cali_get_cali_executing())
   {
       return 1;
   }
   else if(!remo_cali_get_elec_angle_flag() || !remo_cali_get_motor_joint_valid())
   {
       // 进入此条件时有可能刚进入校准，所以在return 1后
       return 0;
   }
   else if(imu_cali_state.gyro_state && !imu_cali_state.accel_state)
   {
       return 3;
   }
   else if(!imu_cali_state.gyro_state && imu_cali_state.accel_state)
   {
       return 4;
   }
   else if(imu_cali_state.gyro_state && imu_cali_state.accel_state)
   {
       return 5;
   }
   else 
   {
       return 2;
   }
}


/**************************************************************************************************
 * 函数名称: remo_cali_get_user_accel_percentage
 * 输入参数: void
 * 返回结果: uint8_t
 * 功能描述: 获取用户加速度校准进度百分比
**************************************************************************************************/
uint8_t remo_cali_get_user_accel_percentage(void)
{
    return user_accel_cali_percentage;
}


/**************************************************************************************************
 * 函数名称: remo_cali_get_gyro_state
 * 输入参数: void
 * 返回结果: uint8_t
 * 功能描述: 获取陀螺仪校准标志
**************************************************************************************************/
uint8_t remo_cali_get_gyro_state(void)
{
    return imu_cali_state.gyro_state;
}

/**************************************************************************************************
 * 函数名称: remo_cali_get_imu_shutdown_flag
 * 输入参数: void
 * 返回结果: bool
 * 功能描述: 获取led开关标志
**************************************************************************************************/
bool remo_cali_get_cali_finish_flag(void)
{
    return (cali_status.cali_state_last == CALI_STATE_IDLE) ? true:false;
}

/**************************************************************************************************
 * 函数名称: remo_cali_get_pcba_check_flag
 * 输入参数: void
 * 返回结果: uint8_t
 * 功能描述: 获取pcba单板检测状态
**************************************************************************************************/
//uint8_t remo_cali_get_pcba_check_state(void)
//{
//    return cali_params.cali_flag.types.pcba_check_state;
//}

/**************************************************************************************************
 * 函数名称: remo_cali_get_pcba_check_flag
 * 输入参数: void
 * 返回结果: uint8_t
 * 功能描述: 获取pcba单板检测状态
**************************************************************************************************/
//uint8_t remo_cali_get_fac_res_state(void)
//{
//    return cali_params.cali_flag.types.factory_test_state;
//}

/**************************************************************************************************
 * 函数名称: remo_cali_get_cali_flag
 * 输入参数: void
 * 返回结果: cali_flag_t *
 * 功能描述: 获取校准标志
**************************************************************************************************/
cali_flag_t *remo_cali_get_cali_flag(void)
{
    return &cali_params.cali_flag;
}


void remo_cali_set_fac_res_state(uint8_t state)
{
    cali_params.cali_flag.types.factory_test_state = state;
//    remo_flash_local_refresh(CAL_STATUS_ADDR_OFFSET, (uint32_t *)&cali_params.cali_flag.which_type, 1);
}

void remo_cali_set_pcba_check_state(uint8_t state)
{
    cali_params.cali_flag.types.pcba_check_state = state;
//    remo_flash_local_refresh(CAL_STATUS_ADDR_OFFSET, (uint32_t *)&cali_params.cali_flag.which_type, 1);
}

uint8_t remo_cali_get_pcba_check_state(void)
{
//    remo_flash_read_words(FLASH_ADDR_CAL_STATUS, 1, (uint32_t *)&cali_params.cali_flag);
    return cali_params.cali_flag.types.pcba_check_state;
}

uint8_t remo_cali_get_fac_res_state(void)
{
//    remo_flash_read_words(FLASH_ADDR_CAL_STATUS, 1, (uint32_t *)&cali_params.cali_flag);
    return cali_params.cali_flag.types.factory_test_state;
}

/**************************************************************************************************
 * 函数名称: remo_cali_get_joint_angle_valid
 * 输入参数: void
 * 返回结果: bool
 * 功能描述: 获取校准标志
**************************************************************************************************/
bool remo_cali_get_joint_angle_valid(void)
{
    return cali_params.cali_flag.types.joint_angle_valid == 1;
}

uint8_t remo_cali_get_elec_align_state(void)
{
    return cali_status.elec_align_state;
}

uint8_t remo_cali_get_accel_fit_state(void)
{
    return cali_status.accel_fit_state;
}

uint8_t remo_cali_get_err_state(void)
{
    if (cali_status.cali_state == CALI_ERR_MOTOR_ALIGN)
    {
        return cali_status.elec_align_state;
    }
    else if (cali_status.cali_state == CALI_ERR_DYNAMIC)
    {
        return cali_status.accel_fit_state;
    }
    else if (cali_status.cali_state == CALI_ERR_ROLL_HALL_LINER || cali_status.cali_state == CALI_ERR_PITCH_HALL_LINER || cali_status.cali_state == CALI_ERR_YAW_HALL_LINER)
    {
        return joint_data_fit_info[PITCH].jfit_state || joint_data_fit_info[ROLL].jfit_state || joint_data_fit_info[YAW].jfit_state;
    }
    else return 0;
}


