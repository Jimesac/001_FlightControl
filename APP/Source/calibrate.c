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

    if (cali_params.cali_flag.which_type == 0xFFFFFFFF)
    {
        cali_params.cali_flag.which_type = 0;
    }

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
            if (ahrs_get_euler_axis_deg(axis_index) < -joint_data_fit_info[axis_index].angle_th + 12.0f && joint_data_fit_info[axis_index].align_clock_0p1us[1] > 7e7)
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

            if (joint_data_fit_info[axis_index].align_clock_0p1us[1] > 80e7)
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

void remo_cali_reflash_imu_gyro_offset(int16_t gyrox_offet, int16_t gyroy_offet, int16_t gyroz_offet)
{
    uint32_t addr_offset = 0;
    cali_params.gyro_offset[XAXIS] = gyrox_offet;
    cali_params.gyro_offset[YAXIS] = gyroy_offet;
    cali_params.gyro_offset[ZAXIS] = gyroz_offet;
    addr_offset = (uint32_t)&cali_params.gyro_offset - (uint32_t)&cali_params;
    bsp_flash_user_params_write_words(addr_offset/4, &cali_params.gyro_offset, 3);
}

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


