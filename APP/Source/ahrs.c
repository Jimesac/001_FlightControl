#include <stdlib.h>
#include <math.h>
#include "hpm_math.h"
#include "ahrs.h"
#include "misc.h"
#include "angle_encoder.h"
#include "calibrate.h"
#include "bsp_timer.h"
#include "filter.h"

#define ROTATE_BY_Z_X_Y
#define RPY_DEFINE_AS_XYZ

#define EKF_UPDATE_TIME (0.01f)    // EKF执行更新时间

#define AHRS_INIT_GYRO_OFFSET 

bool imu_data_update_flag = false;
int16_t gyro_bias[3] = {0};

// 注意，此处采用北东地（NED）的坐标系
// 初始化采样数
#define AHRS_INIT_SAMPLE_NUM       ((uint16_t)800)
#define AHRS_INIT_SAMPLE_HALF_NUM  (AHRS_INIT_SAMPLE_NUM/2)
#define AHRS_INIT_SAMPLE_NORM  (1.2f)

ahrs_status_t ahrs_status = {
    .init_finished_flag = false,
    .euler_rad = {0},
    .euler_deg = {0},
    .quaternion = {0},
    .dcm_b2n = {0},
  
    .euler_raw_rad = {0},
    .gimbal_lock_flag = false
};

ahrs_comp_t ahrs_comp = {0};

ahrs_err_status_t ahrs_err_status = {0};

// 外部加速度的加减速
struct
{
    float ext_acc_posneg[3][2];   // 外部加减速度值
    float pi_beta[3];             // 外部加减速值的比值
    bool  continue_flag[3];    // 连续加减速度标志
    
    uint16_t count[3][2];   // 连续加减速度计数器
    uint16_t num;           // 连续加减速的持续间隔
}ahrs_extern_acc_beta;

struct {
    float err_theta[3];
    bool flag;
}quat_coorct = {
        .err_theta = {0.0f},
        .flag = false
};

static void ahrs_attitude_init(const float *accel);
static void ahrs_correct_quat(void);
static void remo_ahrs_base_dcm_by_jiont_angle(void);
static void remo_ahrs_base_orientation(void);

/**************************************************************************************************
 * 函数名称: ahrs_params_init
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 姿态相关参数变量初始化
**************************************************************************************************/
void ahrs_params_init(void)
{
    // 1-- 互补滤波参数
    // 加速度互补滤波参数初始化
    ahrs_comp.accel_params.kp_params.offset = 20.0f;
    ahrs_comp.accel_params.kp_params.amplitude = 19.5f;
    ahrs_comp.accel_params.kp_params.slope = 10000.0f;
    ahrs_comp.accel_params.ki_params.offset = 0.02f;
    ahrs_comp.accel_params.ki_params.amplitude = 0.015f;
    ahrs_comp.accel_params.ki_params.slope = 50000.0f;
    ahrs_comp.accel_params.g_norm_scale.offset = 0.1f;
    ahrs_comp.accel_params.g_norm_scale.amplitude = -25.0f;
    ahrs_comp.accel_params.g_norm_scale.slope = 150.0f;
    ahrs_comp.accel_params.err_limit = 0.5f;
    ahrs_comp.accel_params.kp_limit = 0.12f;
    ahrs_comp.accel_params.ki_limit = 0.12f;
    ahrs_comp.accel_params.output_limit = 0.22f;
    ahrs_comp.accel_params.kp = 0.0f;
    ahrs_comp.accel_params.ki = 0.0f;

    ahrs_comp.accel_params.ki_err_decay = 0.9f;
    ahrs_comp.accel_params.ki_decay = 0.5f;

    // 加速度二阶滤波参数初始化
    ahrs_comp.accel_filter_alpha = 0.1f;
    ahrs_comp.accel_filter[0] = 0.0f;
    ahrs_comp.accel_filter[1] = 0.0f;
    ahrs_comp.accel_filter[2] = 0.0f;
	
    ahrs_comp.user_flag = true;
    

    // 加速度有效的阈值
    ahrs_comp.accel_valid_limit[0] = 0.98f;
    ahrs_comp.accel_valid_limit[1] = 1.02f;

    // 加速度和磁力计补偿时的分频系数
    ahrs_comp.accel_comp_divison = 0.02f;     

    // 外部加速度系数
    ahrs_comp.ext_acc_coef = 0.0f;

    // 其它参数变量初始化
    ahrs_comp.use_accel_flag = false;
    ahrs_comp.ext_acc[0] = 0.0f;
    ahrs_comp.ext_acc[1] = 0.0f;
    ahrs_comp.ext_acc[2] = 0.0f;

    ahrs_comp.gyro_rad_comp[0] = 0.0f;
    ahrs_comp.gyro_rad_comp[1] = 0.0f;
    ahrs_comp.gyro_rad_comp[2] = 0.0f;

    // 2-- 姿态误差参数
    // 误差阈值
    ahrs_err_status.err_sum_th = 0.1f;
    // 误差累积阈值
    ahrs_err_status.err_num = 200;

    // 其它参数变量初始化
    ahrs_err_status.err_sum = 0.0f;
    ahrs_err_status.err_count = 0;
    ahrs_err_status.err_flag = false;
}

/**************************************************************************************************
 * 函数名称: ahrs_attitude_init
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 根据重力加速度的值，初始化欧拉角和四元素的值
**************************************************************************************************/
static void ahrs_attitude_init(const float *accel)
{
    float temp;
    float ax = 0.0f, ay = 0.0f, az = 0.0f;

    // 将加速度归一化处理
	temp = Q_rsqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
    ax = (float)accel[XAXIS] * temp;
    ay = (float)accel[YAXIS] * temp;
    az = (float)accel[ZAXIS] * temp;
    
    ahrs_comp.accel_filter[0] = ax/G_MS2_TO_STAND1;
    ahrs_comp.accel_filter[1] = ay/G_MS2_TO_STAND1;
    ahrs_comp.accel_filter[2] = az/G_MS2_TO_STAND1;
	
	// 根据加速度计求四元素
    // 设q3=0,得
    // ax = -2*q0*q2;
    // ay = 2*q0*q1;
    // az = q0^2-q1^2-q2^2;
    // 且设ax^2+ay^2+az^2 = 1
    // 解方程组得
    // q0 = sqrt((1+az)/2);
    // q1 = ay/(2*q0);
    // q2 = -ax/(2*q0)
    az = (az > -0.998f) ? az : -0.998f;
    ahrs_status.quaternion[0] = sqrtf((1.0f + az) * 0.5f);
    ahrs_status.quaternion[1] = ay / (2.0f * ahrs_status.quaternion[0]);
    ahrs_status.quaternion[2] = -ax / (2.0f * ahrs_status.quaternion[0]);
    ahrs_status.quaternion[3] = 0.0f;
	
	// 由四元素初始化机体坐标系到地球坐标系的旋转矩阵
    ahrs_quat2dcmbn(ahrs_status.quaternion, &ahrs_status.dcm_b2n[0][0]);
    
    ahrs_params_init();
}

/**************************************************************************************************
 * 函数名称: ahrs_init
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 姿态解算相关变量的初始化：
 *          1、初始化陀螺仪角速度补偿相关的PID参数
 *          2、初始化加速度滤波用的低通滤波器
 *          3、初始化sensor_meas变量（主要是初始化其中四元素的值）
**************************************************************************************************/
void ahrs_init(void)
{
    static uint16_t i = 0, j = 0;
    bool flag = false;
    static float init_acc[3] = {0};
    uint32_t acc_norm2 = 0;
    static uint32_t ahrs_init_timer[2] = {0};
    static bool init_flag = false;
    float arhs_init_norm2_th = AHRS_INIT_SAMPLE_NORM;
    static float gyro_mean[2][3] = {0}, gyro_mean2[3] = {0}, gyro_var[2][3] = {0};
    static int32_t gyro_offset_init[3] = {0};
    int32_t temp_i32 = 0;
    float temp_f = 0.0f, temp_f0 = 0.0f;
    float gyro_fine_offset[3] = {0.0f};
	
    const float *accel_smp = remo_imu_get_accel_corr();
    const int16_t *gyro_raw = remo_imu_get_gyro_smp();
    static uint32_t init_cnt = 0;

    if (ahrs_status.init_finished_flag) return;

    if (!init_flag)
    {
        init_flag = true;
        init_cnt = 0;
        ahrs_init_timer[0] = bsp_timer_clock_start_0p1us();
#ifdef AHRS_INIT_GYRO_OFFSET
        gyro_offset_init[0] = remo_imu_get_gyro_bias(0);
        gyro_offset_init[1] = remo_imu_get_gyro_bias(1);
        gyro_offset_init[2] = remo_imu_get_gyro_bias(2);
        remo_imu_set_gyro_bias(0, 0, 0);
#endif
    }

    while(1)
    {

        while (!bsp_timer_schedule_get_update_flag())
        {
        }

        remo_imu_update();
        bsp_timer_schedule_reset_update_flag();

        // 先求取加速度的平均值
        if (i < AHRS_INIT_SAMPLE_NUM)
        {
              remo_imu_clear_data_refresh_flag();
              acc_norm2 = accel_smp[0] * accel_smp[0] + accel_smp[1] * accel_smp[1]+ accel_smp[2] * accel_smp[2];
              acc_norm2 = sqrt(acc_norm2);
              flag = (bool)((accel_smp[0] + accel_smp[1] + accel_smp[2]) != 0.0f); 
  //			flag |= (bool)(acc_norm2 < arhs_init_norm2_th); 
          
              for (j = 0; j < 3 ; j++)
              {
                  gyro_mean[0][j] += gyro_raw[j];
                  gyro_mean2[j] += gyro_raw[j] * gyro_raw[j];
              }
              if (i == AHRS_INIT_SAMPLE_HALF_NUM)
              {
                  for (j = 0; j < 3 ; j++)
                  {
                      gyro_mean[0][j] /= AHRS_INIT_SAMPLE_HALF_NUM;
                      gyro_var[0][j] = gyro_mean2[j] / AHRS_INIT_SAMPLE_HALF_NUM - \
                                  gyro_mean[0][i] * gyro_mean[0][i];
                      gyro_mean[1][j] = gyro_mean[0][j];
                      gyro_mean[0][j] = 0;
                      gyro_mean2[j] = 0;
                  }
              }
          
          
              if(flag)
              {
                  init_acc[0] += accel_smp[0];
                  init_acc[1] += accel_smp[1];
                  init_acc[2] += accel_smp[2];

                  i++;
              }
                      
              ahrs_init_timer[1] += bsp_timer_clock_end_0p1us(ahrs_init_timer[0]);
              ahrs_init_timer[0] = bsp_timer_clock_start_0p1us();
                      //if (ahrs_init_timer[1] > 3000e3)
              if (++init_cnt>4000)
              {
                      // 取加速度的平均值,初始化姿态
                      for (j = 0; j < 3; j++)
                      {
                              init_acc[j] /= i;
                      }
                      ahrs_attitude_init(init_acc);
                      remo_imu_set_gyro_bias(gyro_offset_init[0], gyro_offset_init[1], gyro_offset_init[2]);
                      ahrs_status.init_finished_flag = true;
              }
        }
        else
        {
    #ifdef AHRS_INIT_GYRO_OFFSET
            for (j = 0; j < 3 ; j++)
            {
                gyro_mean[0][j] = (float)gyro_mean[0][j] / (float)AHRS_INIT_SAMPLE_HALF_NUM;
                gyro_var[1][j] = gyro_mean2[j] / AHRS_INIT_SAMPLE_HALF_NUM - gyro_mean[0][i] * gyro_mean[0][i];
            }
            if ((fabs(gyro_mean[0][0]) < 100 && fabs(gyro_mean[0][1]) < 100 && fabs(gyro_mean[0][2]) < 200) && \
                (fabs(gyro_mean[1][0] - gyro_mean[0][0]) <= 5 && fabs(gyro_mean[1][1] - gyro_mean[0][1]) <= 5 && \
                fabs(gyro_mean[1][2] - gyro_mean[0][2]) <= 5) && \
                (gyro_var[0][0] < 5000 && gyro_var[0][1] < 5000 && gyro_var[0][2] < 5000))
            {
                for (j = 0; j < 3 ; j++)
                {
                    gyro_mean[0][j] = (gyro_mean[0][j] + gyro_mean[1][j])/2;
                    gyro_fine_offset[j] = gyro_mean[0][j] - (int)(gyro_mean[0][j]);
                }
			
                remo_imu_set_gyro_bias(gyro_mean[0][0], gyro_mean[0][1], gyro_mean[0][2]);
                remo_imu_set_fine_offset(gyro_fine_offset);

                
                gyro_offset_init[0] = gyro_mean[0][0];
                gyro_offset_init[1] = gyro_mean[0][1];
                gyro_offset_init[2] = gyro_mean[0][2];
                remo_imu_set_gyro_bias(gyro_offset_init[0], gyro_offset_init[1], gyro_offset_init[2]);
                if (fabs(gyro_offset_init[0] - gyro_mean[0][0]) >=2 || fabs(gyro_offset_init[1] - gyro_mean[0][1]) >=2 ||\
                    fabs(gyro_offset_init[2] - gyro_mean[0][2]) >=2)
                {
                    remo_cali_reflash_imu_gyro_offset(gyro_offset_init[0], gyro_offset_init[1], gyro_offset_init[2]);
                }
                
            
                ahrs_status.init_finished_flag = true;
            }
            else
            {
                remo_imu_set_gyro_bias(gyro_offset_init[0], gyro_offset_init[1], gyro_offset_init[2]);
            }
    #endif
        
            // 取加速度的平均值,初始化姿态
            for (j = 0; j < 3; j++)
            {
                init_acc[j] /= AHRS_INIT_SAMPLE_NUM;
            }
            ahrs_attitude_init(init_acc);
    //        ahrs_status.init_finished_flag = true;
            i = 0;
        }
	
        if (ahrs_init_timer[1] > 1500)
        {
            arhs_init_norm2_th = AHRS_INIT_SAMPLE_NORM + 0.02f;
        }
        else if (ahrs_init_timer[1] > 2000)
        {
            arhs_init_norm2_th = AHRS_INIT_SAMPLE_NORM + 0.04f;
        }
        else if (ahrs_init_timer[1] > 2500)
        {
            arhs_init_norm2_th = AHRS_INIT_SAMPLE_NORM + 0.1f;
        }

        if (ahrs_status.init_finished_flag) return;

    }
    
}


/**************************************************************************************************
 * 函数名称: ahrs_update_attitude
 * 输入参数: samle_interval->时间周期（也就是控制周期）
 * 返回结果: void
 * 功能描述: 根据MPU6500采样得到的加速度和角速度，采用PI算法，计算得到当前的欧拉角
**************************************************************************************************/
float accel_stand[3] = {0.0f};
void ahrs_update_attitude(float samle_interval)
{
    float temp = 0.0f;
    float acc_temp[3] = {0.0f};
    float q0i = 0.0f, q1i = 0.0f, q2i = 0.0f, q3i = 0.0f; // 四元素的变化量
    int i = 0;
	
//    float accel_stand[3] = {0.0f};

    const float *accel_g = remo_imu_get_accel_corr();
    const float *gyro_deg = remo_imu_get_gyro_corr();
	
    static float gyro_rad[3] = {0.0f};
    
    gyro_rad[0] = gyro_deg[0]*DEG_TO_RAD;
    gyro_rad[1] = gyro_deg[1]*DEG_TO_RAD;
    gyro_rad[2] = gyro_deg[2]*DEG_TO_RAD;
    
	
    // 通过判断加速度的模来决定加速度是否用于融合
    if (fabs(gyro_deg[0]) < 3.0f && fabs(gyro_deg[1]) < 3.0f && fabs(gyro_deg[2]) < 3.0f)
    {
        for(i = 0; i < 3; i++)
        {
            temp = fabs(gyro_deg[i]) / 3.0f;
            temp += ahrs_comp.accel_filter_alpha;
            if (temp > 1.0f) 
            {
                    temp = 1.0f;
            }
            ahrs_comp.accel_filter[i] += (accel_g[i] - ahrs_comp.accel_filter[i]) * temp;
            accel_stand[i] = ahrs_comp.accel_filter[i] * G_MS2_TO_STAND1;
        }
    }
    else
    {
        for(i = 0; i < 3; i++)
        {
            ahrs_comp.accel_filter[i] = accel_g[i];
            accel_stand[i] = ahrs_comp.accel_filter[i] * G_MS2_TO_STAND1;
        }
    }

//	if (fabs(gyro_deg[0]) > 3.0f || fabs(gyro_deg[1]) > 3.0f  || fabs(gyro_deg[2]) > 3.0f)
//	{
//		ahrs_comp.user_flag = false;
//	}
//	else 
//	{
//		ahrs_comp.user_flag = true;
//	}
	
    ahrs_comp.accel_norm = Q_rsqrt(accel_stand[0] * accel_stand[0] + accel_stand[1] * accel_stand[1] + \
			accel_stand[2] * accel_stand[2]);
    ahrs_comp.use_accel_flag = (bool)(ahrs_comp.accel_norm < ahrs_comp.accel_valid_limit[1] && ahrs_comp.accel_norm >ahrs_comp.accel_valid_limit[0]);
    accel_stand[0] *= ahrs_comp.accel_norm;
    accel_stand[1] *= ahrs_comp.accel_norm;
    accel_stand[2] *= ahrs_comp.accel_norm;

    if (ahrs_comp.use_accel_flag && ahrs_comp.user_flag)
    {
        ahrs_comp.accel_params.err[0] = accel_stand[1] * ahrs_status.dcm_b2n[2][2] - accel_stand[2] * ahrs_status.dcm_b2n[2][1];
        ahrs_comp.accel_params.err[1] = accel_stand[2] * ahrs_status.dcm_b2n[2][0] - accel_stand[0] * ahrs_status.dcm_b2n[2][2];
        ahrs_comp.accel_params.err[2] = accel_stand[0] * ahrs_status.dcm_b2n[2][1] - accel_stand[1] * ahrs_status.dcm_b2n[2][0];

        ahrs_comp.accel_params.err[0] = remo_misc_constrainf(ahrs_comp.accel_params.err[0],
                                -ahrs_comp.accel_params.err_limit, ahrs_comp.accel_params.err_limit);
        ahrs_comp.accel_params.err[1] = remo_misc_constrainf(ahrs_comp.accel_params.err[1],
                                -ahrs_comp.accel_params.err_limit, ahrs_comp.accel_params.err_limit);
        ahrs_comp.accel_params.err[2] = remo_misc_constrainf(ahrs_comp.accel_params.err[2],
                                -ahrs_comp.accel_params.err_limit, ahrs_comp.accel_params.err_limit);
		
		ahrs_comp.err_sum_sq = ahrs_comp.accel_params.err[0] *ahrs_comp.accel_params.err[0] + ahrs_comp.accel_params.err[1] *ahrs_comp.accel_params.err[1] + \
				ahrs_comp.accel_params.err[2] *ahrs_comp.accel_params.err[2];
		ahrs_comp.accel_params.kp = ahrs_comp.accel_params.kp_params.offset - ahrs_comp.accel_params.kp_params.amplitude / (1.0f + ahrs_comp.accel_params.kp_params.slope * ahrs_comp.err_sum_sq);
		ahrs_comp.accel_params.ki = ahrs_comp.accel_params.ki_params.offset - ahrs_comp.accel_params.ki_params.amplitude / (1.0f + ahrs_comp.accel_params.ki_params.slope * ahrs_comp.err_sum_sq);
        for (i = 0; i < 3; i++)
        {
            ahrs_comp.accel_params.kp_output[i] = ahrs_comp.accel_params.kp * ahrs_comp.accel_params.err[i];
            ahrs_comp.accel_params.ki_output[i] += ahrs_comp.accel_params.ki * ahrs_comp.accel_params.err[i];
            if ((ahrs_comp.accel_params.err[i] < 0.0f && ahrs_comp.accel_params.ki_output[i] > 0.0f ) ||
                    (ahrs_comp.accel_params.err[i] > 0.0f && ahrs_comp.accel_params.ki_output[i] < 0.0f ))
            {
                ahrs_comp.accel_params.ki_output[i] *= ahrs_comp.accel_params.ki_err_decay;
            }
        }
    }
    else
    {
        // ki参数清零
        for (i = 0; i < 3; i++)
        {
                ahrs_comp.accel_params.kp_output[i] = 0;
                ahrs_comp.accel_params.ki_output[i] *= ahrs_comp.accel_params.ki_decay;
        }
    }
    
    ahrs_comp.limit_scale = 1.0f;
//    ahrs_comp.limit_scale = ahrs_comp.accel_params.g_norm_scale.offset - ahrs_comp.accel_params.g_norm_scale.amplitude / \
//                (1.0f + ahrs_comp.accel_params.g_norm_scale.slope * (ahrs_comp.accel_norm-1.0f)*(ahrs_comp.accel_norm-1.0f));

    // 对误差采用PI控制
    for (i = 0; i < 3; i++)
    {
        ahrs_comp.accel_params.kp_output[i] = remo_misc_constrainf(ahrs_comp.accel_params.kp_output[i],
                            -ahrs_comp.accel_params.kp_limit*ahrs_comp.limit_scale, ahrs_comp.accel_params.kp_limit*ahrs_comp.limit_scale);
        ahrs_comp.accel_params.ki_output[i] = remo_misc_constrainf(ahrs_comp.accel_params.ki_output[i],
                            -ahrs_comp.accel_params.ki_limit*ahrs_comp.limit_scale, ahrs_comp.accel_params.ki_limit*ahrs_comp.limit_scale);
		ahrs_comp.accel_params.pi_output[i] = ahrs_comp.accel_params.kp_output[i] + ahrs_comp.accel_params.ki_output[i];
		ahrs_comp.accel_params.pi_output[i] = remo_misc_constrainf(ahrs_comp.accel_params.pi_output[i],
                            -ahrs_comp.accel_params.output_limit*ahrs_comp.limit_scale, ahrs_comp.accel_params.output_limit*ahrs_comp.limit_scale);
        ahrs_comp.gyro_rad_comp[i] = gyro_rad[i] + ahrs_comp.accel_params.pi_output[i];
    }

    // 计算四元素的变化量
    temp = samle_interval * 0.5f;
    q0i = temp * (-ahrs_status.quaternion[1] * ahrs_comp.gyro_rad_comp[XAXIS] -
                  ahrs_status.quaternion[2] * ahrs_comp.gyro_rad_comp[YAXIS] -
                  ahrs_status.quaternion[3] * ahrs_comp.gyro_rad_comp[ZAXIS]);
    q1i = temp * (ahrs_status.quaternion[0] * ahrs_comp.gyro_rad_comp[XAXIS] -
                  ahrs_status.quaternion[3] * ahrs_comp.gyro_rad_comp[YAXIS] +
                  ahrs_status.quaternion[2] * ahrs_comp.gyro_rad_comp[ZAXIS]);
    q2i = temp * (ahrs_status.quaternion[3] * ahrs_comp.gyro_rad_comp[XAXIS] +
                  ahrs_status.quaternion[0] * ahrs_comp.gyro_rad_comp[YAXIS] -
                  ahrs_status.quaternion[1] * ahrs_comp.gyro_rad_comp[ZAXIS]);
    q3i = temp * (-ahrs_status.quaternion[2] * ahrs_comp.gyro_rad_comp[XAXIS] +
                  ahrs_status.quaternion[1] * ahrs_comp.gyro_rad_comp[YAXIS] +
                  ahrs_status.quaternion[0] * ahrs_comp.gyro_rad_comp[ZAXIS]);

    // 更新四元素值
    ahrs_status.quaternion[0] += q0i;
    ahrs_status.quaternion[1] += q1i;
    ahrs_status.quaternion[2] += q2i;
    ahrs_status.quaternion[3] += q3i;

//    if (quat_coorct.flag) {
////        ahrs_status.quaternion[0] = 0.0900405503916830;
////        ahrs_status.quaternion[1] = -0.292502353931936;
////        ahrs_status.quaternion[2] = 0.952015958720919;
////        ahrs_status.quaternion[3] = 0.000828595275075330;
////        quat_coorct.err_theta[2] = 0.01;
//        ahrs_correct_quat();
//        quat_coorct.flag = false;
//    }
    


//    ahrs_comp.ext_acc[0] = ahrs_status.dcm_b2n[0][0] * ahrs_comp.accel_filter[0] + ahrs_status.dcm_b2n[0][1] * ahrs_comp.accel_filter[1] + \
//            ahrs_status.dcm_b2n[0][2] * ahrs_comp.accel_filter[2];
//    ahrs_comp.ext_acc[1] = ahrs_status.dcm_b2n[1][0] * ahrs_comp.accel_filter[0] + ahrs_status.dcm_b2n[1][1] * ahrs_comp.accel_filter[1] + \
//            ahrs_status.dcm_b2n[1][2] * ahrs_comp.accel_filter[2];
//    ahrs_comp.ext_acc[2] = ahrs_status.dcm_b2n[2][0] * ahrs_comp.accel_filter[0] + ahrs_status.dcm_b2n[2][1] * ahrs_comp.accel_filter[1] + \
//            ahrs_status.dcm_b2n[2][2] * ahrs_comp.accel_filter[2] - 9.8f;
}

void ahrs_dcm_euler_update(void)
{
    // 求机体坐标到地理坐标系的转换矩阵
    ahrs_quat2dcmbn(&ahrs_status.quaternion[0], &ahrs_status.dcm_b2n[0][0]);
    ahrs_dmcbn2euler(&ahrs_status.dcm_b2n[0][0], &ahrs_status.euler_rad[0], &ahrs_status.euler_deg[0]);

    remo_ahrs_base_orientation();
}

/**************************************************************************************************
 * 函数名称: ahrs_dmcbn2euler
 * 输入参数: dcm_b2n->指向四元素的数据指针，euler_rad->欧拉角
 * 返回结果: void
 * 功能描述: 根据机体坐标系到地理坐标系的dcm矩阵求欧拉角
**************************************************************************************************/
float mytest_euler_pitch = 0.0f;
void ahrs_dmcbn2euler(float *dcm_b2n, float *euler_rad, float *euler_deg)
{
    int16_q7_t angle_temp_q7;
    trig_func_q14_t trig_q14 = {0};
    float angle_xyz_rad[3] = {0};
#ifdef ROTATE_BY_Z_Y_X
    //          [cy*cz,  cz*sx*sy - cx*sz, sx*sz + cx*cz*sy
    // Tb2n =    cy*sz,  cx*cz + sx*sy*sz, cx*sy*sz - cz*sx
    //           -sy,         cy*sx,            cx*cy  ]
    angle_xyz_rad[0] = Rajan_FastArcTan2(dcm_b2n[2][1], dcm_b2n[2][2]);
    angle_xyz_rad[1] = -safe_asinf(dcm_b2n[2][0]);
    angle_xyz_rad[2] = Rajan_FastArcTan2(dcm_b2n[1][0], dcm_b2n[0][0]);

    if (ahrs_status.gimbal_lock_flag)
    {
        angle_xyz_rad[2] = Rajan_FastArcTan2(-dcm_b2n[0][1], dcm_b2n[1][1]);
    }

#elif defined(ROTATE_BY_Z_X_Y)
    //          [cy*cz - sx*sy*sz, -cx*sz, cz*sy + cy*sx*sz
    // Tb2n =    cy*sz + cz*sx*sy,  cx*cz, sy*sz - cy*cz*sx
    //            -cx*sy,            sx,        cx*cy]

    switch (ahrs_status.base_state.base_ori)
	{
        case AHRS_BASE_VERT_UP:
            angle_xyz_rad[0] = asinf(dcm_b2n[7]);
            angle_xyz_rad[1] = Rajan_FastArcTan2(-dcm_b2n[6], dcm_b2n[8]);
            angle_xyz_rad[2] = Rajan_FastArcTan2(-dcm_b2n[1], dcm_b2n[4]);	
            //if (angle_xyz_rad[2] >= 3.14159265f) angle_xyz_rad[2] -= 6.283185f;
            //else if (angle_xyz_rad[2] < -3.14159265f) angle_xyz_rad[2] += 6.283185f;
            break;
        case AHRS_BASE_VERT_DOWN:
            angle_xyz_rad[0] = safe_asinf(dcm_b2n[7]);
            angle_xyz_rad[1] = Rajan_FastArcTan2(-dcm_b2n[6], -dcm_b2n[8]);
            angle_xyz_rad[2] = -Rajan_FastArcTan2(-dcm_b2n[1], dcm_b2n[4]);	  
            if (angle_xyz_rad[2] >= 3.14159265f) angle_xyz_rad[2] -= 6.283185f;
            else if (angle_xyz_rad[2] < -3.14159265f) angle_xyz_rad[2] += 6.283185f;
                break;
        case AHRS_BASE_HOR_CAM_VERT:
            angle_xyz_rad[0] = Rajan_FastArcTan2(dcm_b2n[1], dcm_b2n[4]);
            angle_xyz_rad[1] = Rajan_FastArcTan2(dcm_b2n[8], dcm_b2n[6]);
            angle_xyz_rad[2] = safe_asinf(dcm_b2n[7]);
            if(ahrs_status.base_state.base_dcmz[1] < 0.0f)
            {
                    angle_xyz_rad[1] = -angle_xyz_rad[1];
                    angle_xyz_rad[2] = -angle_xyz_rad[2];
            }
            break;
        case AHRS_BASE_HOR_CAM_HOR:
            angle_xyz_rad[0] = Rajan_FastArcTan2(dcm_b2n[0], dcm_b2n[3]);
            //angle_xyz_rad[1] = Rajan_FastArcTan2(dcm_b2n[6], dcm_b2n[8]);
            //angle_xyz_rad[2] = safe_asinf(dcm_b2n[7]);
            //if(ahrs_status.base_state.base_dcmz[0] > 0.0f)
            //{
            //        angle_xyz_rad[2] = -angle_xyz_rad[2];
            //}

              //if(motorAngle.pitch_angle > 48) oriVeticalPitch_flag = true;
              //else oriVeticalPitch_flag = false;
              //if(oriVeticalPitch_flag)
              //{
              //        EulerRad[1] 	= Q_asin(ahrs_atti.Tbn[2][0]);
              //        EulerRad[2] 	= Rajan_FastArcTan2(ahrs_atti.Tbn[2][1], -ahrs_atti.Tbn[2][2]);
              //}
              //else
              //{
              //        if(ahrs_atti.Tbn[2][1] < 0) 
              //        {
              //                EulerRad[1] 	= Rajan_FastArcTan2(ahrs_atti.Tbn[2][2], -ahrs_atti.Tbn[2][1]);
              //                EulerRad[2] 	= -Q_acos(ahrs_atti.Tbn[2][0]);
              //        }
              //        else 
              //        {
              //                EulerRad[1] 	= Rajan_FastArcTan2(ahrs_atti.Tbn[2][2], ahrs_atti.Tbn[2][1]);
              //                EulerRad[2] 	= Q_acos(ahrs_atti.Tbn[2][0]);
              //        }
              //}
            break;
        case AHRS_BASE_ROLL_SPECIAL:
            angle_xyz_rad[0] = Rajan_FastArcTan2(dcm_b2n[7], dcm_b2n[8]);
            angle_xyz_rad[1] = asinf(-dcm_b2n[6]);
            angle_xyz_rad[2] = Rajan_FastArcTan2(dcm_b2n[3], dcm_b2n[0]);	
            break;
        case AHRS_BASE_PITCH_SPECIAL:
            angle_xyz_rad[0] = asinf(dcm_b2n[7]);
            angle_xyz_rad[1] = Rajan_FastArcTan2(-dcm_b2n[6], dcm_b2n[8]);
            angle_xyz_rad[2] = Rajan_FastArcTan2(-dcm_b2n[1], dcm_b2n[4]);	
            break;
        case AHRS_BASE_YAW_SPECIAL:
            angle_xyz_rad[0] = Rajan_FastArcTan2(dcm_b2n[5], dcm_b2n[2]);
            angle_xyz_rad[1] = asinf(dcm_b2n[8]);
            angle_xyz_rad[2] = Rajan_FastArcTan2(dcm_b2n[7], -dcm_b2n[6]);
            break;
        default:
            //跟磁编码
            break;
	}
  
    if (ahrs_status.gimbal_lock_flag)
    {
        angle_xyz_rad[2] = Rajan_FastArcTan2(dcm_b2n[3], dcm_b2n[0]);
    }
#endif

#ifdef RPY_DEFINE_AS_XYZ
    euler_rad[ROLL] = angle_xyz_rad[XAXIS];
    euler_rad[PITCH] = angle_xyz_rad[YAXIS];
    euler_rad[YAW] = angle_xyz_rad[ZAXIS];
    euler_deg[ROLL] = euler_rad[ROLL] * RAD_TO_DEG;
    euler_deg[PITCH] = euler_rad[PITCH] * RAD_TO_DEG;
    euler_deg[YAW] = euler_rad[YAW] * RAD_TO_DEG;
#endif
    
    for (uint8_t i = 0; i < 3; i++)
    {
        angle_temp_q7 = euler_deg[i]*128;
        remo_trig_func(angle_temp_q7, &trig_q14);
        ahrs_status.euler_trigf[i].cos = (float)trig_q14.cos/CONST_1_Q14;
        ahrs_status.euler_trigf[i].sin = (float)trig_q14.sin/CONST_1_Q14;
    }
}

/**************************************************************************************************
 * 函数名称: ahrs_quat2dcmbn
 * 输入参数: quat->指向四元素的数据指针，Tbn->机体坐标到地理坐标系的转换矩阵
 * 返回结果: void
 * 功能描述: 根据四元素求机体坐标到地理坐标系的转换矩阵
**************************************************************************************************/
void ahrs_quat2dcmbn(float quat[4], float *dcm_b2n)
{
    float q00, q01, q02, q03, q11, q12, q13, q22, q23, q33;
    float norm;

    q00 = quat[0] * quat[0];
    q11 = quat[1] * quat[1];
    q22 = quat[2] * quat[2];
    q33 = quat[3] * quat[3];

    // 对四元素进行归一化
    norm = Q_rsqrt(q00 + q11 + q22 + q33);
    quat[0] *= norm;
    quat[1] *= norm;
    quat[2] *= norm;
    quat[3] *= norm;

    q00 = quat[0] * quat[0];
    q01 = quat[0] * quat[1];
    q02 = quat[0] * quat[2];
    q03 = quat[0] * quat[3];
    q11 = quat[1] * quat[1];
    q12 = quat[1] * quat[2];
    q13 = quat[1] * quat[3];
    q22 = quat[2] * quat[2];
    q23 = quat[2] * quat[3];
    q33 = quat[3] * quat[3];

    dcm_b2n[0] = q00 + q11 - q22 - q33;
    dcm_b2n[1] = 2.0f * (q12 - q03);
    dcm_b2n[2] = 2.0f * (q13 + q02);

    dcm_b2n[3] = 2.0f * (q12 + q03);
    dcm_b2n[4] = q00 - q11 + q22 - q33;
    dcm_b2n[5] = 2.0f * (q23 - q01);

    dcm_b2n[6] = 2.0f * (q13 - q02);
    dcm_b2n[7] = 2.0f * (q23 + q01);
    dcm_b2n[8] = q00 - q11 - q22 + q33;
}



/**************************************************************************************************
 * 函数名称: remo_ahrs_base_dcm_by_jiont_angle
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 根据机体到地理坐标系机体到基座的转换矩阵，计算基座到地理坐标系的转换矩阵中的重力方向向量
 *          注意，以下代码是经过转置处理后
**************************************************************************************************/
static void remo_ahrs_base_dcm_by_jiont_angle(void)
{
    float sr, sp, cr, cp;
    float temp;    
    
    const trig_func_f_t *joint_trig[2];

    joint_trig[0] = remo_encoder_get_joint_trig(ROLL);
    joint_trig[1] = remo_encoder_get_joint_trig(PITCH);

    // 电机移动角度三角函数值
    sr = joint_trig[0]->sin;
    cr = joint_trig[0]->cos;
    sp = joint_trig[1]->sin;
    cp = joint_trig[1]->cos;

    // 基座到机体的转换矩阵
    ahrs_status.dcm_ba2b[0][0] = cp;
    ahrs_status.dcm_ba2b[0][1] = 0;
    ahrs_status.dcm_ba2b[0][2] = -sp;
    temp = sr * sp;
    ahrs_status.dcm_ba2b[1][0] = temp;
    ahrs_status.dcm_ba2b[1][1] = cr;
    temp = cp * sr;
    ahrs_status.dcm_ba2b[1][2] = temp;
    temp = cr * sp;
    ahrs_status.dcm_ba2b[2][0] = temp;
    ahrs_status.dcm_ba2b[2][1] = -sr;
    temp = cr * cp;
    ahrs_status.dcm_ba2b[2][2] = temp;

    // 基座到地理坐标系的转换矩阵中重力方向向量
    ahrs_status.base_state.base_dcmz[0] = ahrs_status.dcm_b2n[2][0] * ahrs_status.dcm_ba2b[0][0] +
                              ahrs_status.dcm_b2n[2][1] * ahrs_status.dcm_ba2b[1][0] +
                              ahrs_status.dcm_b2n[2][2] * ahrs_status.dcm_ba2b[2][0];
    ahrs_status.base_state.base_dcmz[1] = ahrs_status.dcm_b2n[2][0] * ahrs_status.dcm_ba2b[0][1] +
                              ahrs_status.dcm_b2n[2][1] * ahrs_status.dcm_ba2b[1][1] +
                              ahrs_status.dcm_b2n[2][2] * ahrs_status.dcm_ba2b[2][1];
    ahrs_status.base_state.base_dcmz[2] = ahrs_status.dcm_b2n[2][0] * ahrs_status.dcm_ba2b[0][2] +
                              ahrs_status.dcm_b2n[2][1] * ahrs_status.dcm_ba2b[1][2] +
                              ahrs_status.dcm_b2n[2][2] * ahrs_status.dcm_ba2b[2][2];
}


/**************************************************************************************************
 * 函数名称: remo_ahrs_base_orientation
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 根据重力在基座坐标系中的表示，以及基座方位变化，判断基座的方位
**************************************************************************************************/
static void remo_ahrs_base_orientation(void)
{
    // 水平时初始化标志
    // 水平初始化只能是AHRS_BASE_HOR_CAM_HOR，AHRS_BASE_HOR_CAM_VERT效果太差了
    static bool hor_init_flag = false;
    static uint32_t hor_changed_cnt = 0;
	
    static float dcmz_mean[3] = {0.0f, 0.0f, 0.0f}, dcmz_mean2[3] = {0.0f, 0.0f, 0.0f};
    static uint32_t dcmz_mean_cnt = 0;
    uint8_t j = 0;
	
    //ahrs_status.base_state.base_ori = AHRS_BASE_YAW_SPECIAL;

    if (ahrs_status.base_state.base_ori == AHRS_BASE_ROLL_SPECIAL || ahrs_status.base_state.base_ori == AHRS_BASE_PITCH_SPECIAL || \
        ahrs_status.base_state.base_ori == AHRS_BASE_YAW_SPECIAL)
    {
        return;
    }
    
    // 需要等到关节角零偏校准完成
    if (!ahrs_status.base_state.base_init_done)
    {
        ahrs_status.base_state.base_ori = AHRS_BASE_VERT_UP;
        return;
    }

    remo_ahrs_base_dcm_by_jiont_angle();
	
    if (ahrs_status.base_state.base_dcmz[2] > AHRS_BASE_VECTICAL_TH)
    {
        // 竖直，云台在上
        if (ahrs_status.base_state.base_dcmz[2] > (AHRS_BASE_VECTICAL_TH))
        {
            ahrs_status.base_state.base_ori = AHRS_BASE_VERT_UP;
        }
    }
    else if (ahrs_status.base_state.base_dcmz[2] < -AHRS_BASE_VECTICAL_TH)
    {
        // 竖直，云台在下
        if (ahrs_status.base_state.base_dcmz[2] < (-AHRS_BASE_VECTICAL_TH))
        {
            ahrs_status.base_state.base_ori = AHRS_BASE_VERT_DOWN;
        }
    }
    else
    {
        // 水平两种情况之间不能相互变化，水平方位只能通过竖直方向变化得到
        if ((ahrs_status.base_state.base_ori_last != AHRS_BASE_HOR_CAM_HOR) &&
            (ahrs_status.base_state.base_ori_last != AHRS_BASE_HOR_CAM_VERT))
        {
            if (fabs(ahrs_status.base_state.base_dcmz[1]) < AHRS_BASE_HORIZONTAL_TH)
            {
                // 水平，相机视轴仍水平
                ahrs_status.base_state.base_ori = AHRS_BASE_HOR_CAM_VERT;
            }
            else
            {
                // 水平，相机视轴变化为竖直
                ahrs_status.base_state.base_ori = AHRS_BASE_HOR_CAM_HOR;
            }
        }
        
        if ( ahrs_status.base_state.base_ori == AHRS_BASE_HOR_CAM_HOR)
        {
            if (fabs(ahrs_status.euler_deg[1]) >= 70 && fabs(ahrs_status.euler_deg[1]) <= 110.0f )
            {
                if (hor_changed_cnt < 500)
                {
                    hor_changed_cnt++;
                }
                else
                {
                    ahrs_status.base_state.base_ori = AHRS_BASE_HOR_CAM_VERT;
                }
            }
            else 
            {
                hor_changed_cnt = 0;
            }
        }
        else if  ( ahrs_status.base_state.base_ori == AHRS_BASE_HOR_CAM_VERT)
        {
            if (fabs(ahrs_status.euler_deg[2]) >= 70.0f && fabs(ahrs_status.euler_deg[2]) <= 110.0f )
            {
                if (hor_changed_cnt < 500)
                {
                    hor_changed_cnt++;
                }
                else
                {
                    ahrs_status.base_state.base_ori = AHRS_BASE_HOR_CAM_HOR;
                }
            }
            else 
            {
                hor_changed_cnt = 0;
            }
        }
    }
	
	for (j = 0; j < 3 ; j++)
	{
            dcmz_mean[j] += ahrs_status.base_state.base_dcmz[j];
            dcmz_mean2[j] += ahrs_status.base_state.base_dcmz[j] * ahrs_status.base_state.base_dcmz[j];
	}
	if (++dcmz_mean_cnt == 40)
	{
            dcmz_mean_cnt = 0;
            for (j = 0; j < 3 ; j++)
            {
                    dcmz_mean[j] /= 40;
                    ahrs_status.base_state.base_dcmz_var[j] = (dcmz_mean2[j] / 40 - dcmz_mean[j] * dcmz_mean[j])*1000;
                    dcmz_mean[j] = 0;
                    dcmz_mean2[j] = 0;
            }
	}

    // 初始化时若基座水平则默认是HOR_CAM_HOR状态
    if (!hor_init_flag)
    {
        hor_init_flag = true;
        if (ahrs_status.base_state.base_ori == AHRS_BASE_HOR_CAM_VERT)
        {
            ahrs_status.base_state.base_ori = AHRS_BASE_HOR_CAM_HOR;
        }
    }
    // 基座方位变化时，需重置某些参数
    ahrs_status.base_state.base_change_flag = (ahrs_status.base_state.base_ori_last == ahrs_status.base_state.base_ori) ? false : true;

    // 赋值当前基座方位
    ahrs_status.base_state.base_ori_last = ahrs_status.base_state.base_ori;

    // 基座方位初始化完成
    ahrs_status.base_state.base_init_done = true;
}

void ahrs_set_correct_quat(float err_theta0, float err_theta1, float err_theta2)
{
    quat_coorct.err_theta[0] = err_theta0;
    quat_coorct.err_theta[1] = err_theta1;
    quat_coorct.err_theta[2] = err_theta2;
    quat_coorct.flag = true;
}

static void ahrs_correct_quat(void)
{
    float delta_theta = 0.0f, temp = 0.0f;
    float delta_q[4] = {0.0f}, q_temp[4] = {0.0f};

    q_temp[0] = ahrs_status.quaternion[0];
    q_temp[1] = ahrs_status.quaternion[1];
    q_temp[2] = ahrs_status.quaternion[2];
    q_temp[3] = ahrs_status.quaternion[3];
    delta_theta = sqrtf(quat_coorct.err_theta[0]*quat_coorct.err_theta[0] + quat_coorct.err_theta[1]*quat_coorct.err_theta[1] + \
                    quat_coorct.err_theta[2]*quat_coorct.err_theta[2]);
    temp = delta_theta*0.5f;
    if (delta_theta < 1e-6) {
        delta_q[0] = 1.0f;
        delta_q[1] = 0.5*quat_coorct.err_theta[0];
        delta_q[2] = 0.5*quat_coorct.err_theta[1];
        delta_q[3] = 0.5*quat_coorct.err_theta[2];
    }
    else{
        delta_q[0] = cosf(temp);
        temp = sinf(temp)/delta_theta;
        delta_q[1] = temp*quat_coorct.err_theta[0];
        delta_q[2] = temp*quat_coorct.err_theta[1];
        delta_q[3] = temp*quat_coorct.err_theta[2];
    }
    ahrs_status.quaternion[0] = delta_q[0]*q_temp[0] - delta_q[1]*q_temp[1] - delta_q[2]*q_temp[2] - delta_q[3]*q_temp[3];
    ahrs_status.quaternion[1] = delta_q[1]*q_temp[0] + delta_q[0]*q_temp[1] - delta_q[3]*q_temp[2] + delta_q[2]*q_temp[3];
    ahrs_status.quaternion[2] = delta_q[2]*q_temp[0] + delta_q[3]*q_temp[1] + delta_q[0]*q_temp[2] - delta_q[1]*q_temp[3];
    ahrs_status.quaternion[3] = delta_q[3]*q_temp[0] - delta_q[2]*q_temp[1] + delta_q[1]*q_temp[2] + delta_q[0]*q_temp[3];
}

void ahrs_set_dcm_cor(float Tb2n_e[3][3])
{
    for (uint8_t i = 0; i < 3; i++) {
        ahrs_status.dcm_b2n[i][0] -= Tb2n_e[i][0];
        ahrs_status.dcm_b2n[i][1] -= Tb2n_e[i][1];
        ahrs_status.dcm_b2n[i][2] -= Tb2n_e[i][2];
    }
}

void remo_ahrs_set_base_init_done(bool flag)
{
    ahrs_status.base_state.base_init_done = flag;
}

void ahrs_set_base_ori(ahrs_base_ori_t base_ori)
{
    ahrs_status.base_state.base_ori = base_ori;
}

bool ahrs_get_init_finished_flag(void)
{
    return ahrs_status.init_finished_flag;
}

void remo_ahrs_set_comp_filter_limit(float limit)
{
    ahrs_comp.accel_params.kp_limit = limit;
}

float ahrs_get_euler_axis_deg(uint8_t axis)
{
    if (axis > 2) return 0.0f;
    return ahrs_status.euler_deg[axis];
}

const float *ahrs_get_base_dcmz(void)
{
    return &ahrs_status.base_state.base_dcmz[0];
}

float ahrs_get_base_dcmz_var(uint8_t axis)
{
	if (axis > 2) return 0.0f;
	return ahrs_status.base_state.base_dcmz_var[axis];
}

/**************************************************************************************************
 * 函数名称: remo_ahrs_get_base_ori
 * 输入参数: void
 * 返回结果: ahrs_base_ori_t
 * 功能描述: 返回基座方位
**************************************************************************************************/
ahrs_base_ori_t ahrs_get_base_ori(void)
{
    return ahrs_status.base_state.base_ori;
}

/**************************************************************************************************
 * 函数名称: remo_ahrs_get_base_vert_flag
 * 输入参数: void
 * 返回结果: bool
 * 功能描述: 返回基座是否竖直
**************************************************************************************************/
bool ahrs_get_base_vert_flag(void)
{
	if (ahrs_status.base_state.base_ori == AHRS_BASE_VERT_UP || ahrs_status.base_state.base_ori == AHRS_BASE_VERT_DOWN)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**************************************************************************************************
 * 函数名称: remo_ahrs_get_base_vert_flag
 * 输入参数: void
 * 返回结果: bool
 * 功能描述: 返回基座是否竖直
**************************************************************************************************/
bool remo_ahrs_get_base_vert_flag(void)
{
	if (ahrs_status.base_state.base_ori == AHRS_BASE_VERT_UP || ahrs_status.base_state.base_ori == AHRS_BASE_VERT_DOWN)
    {
        return true;
    }
    else
    {
        return false;
    }
}



float ahrs_get_ahrs_err_sum_sq(void)
{
    return ahrs_comp.err_sum_sq;
}

float ahrs_get_accel_norm(void)
{
    return ahrs_comp.accel_norm;
}

const float *ahrs_get_dcm(void)
{
    return &ahrs_status.dcm_b2n[0][0];
}

const float *ahrs_get_accel_ext(void)
{
    return &ahrs_comp.ext_acc[0];
}

const float *ahrs_get_accel_filter(void)
{
    return &ahrs_comp.accel_filter[0];
}

const float *ahrs_get_euler_deg_ptr(void)
{
    return &ahrs_status.euler_deg[0];
}

void ahrs_set_gimbal_lock_flag(bool flag)
{
    ahrs_status.gimbal_lock_flag = flag;
}

const trig_func_f_t *remo_ahrs_get_euler_trig(uint8_t index)
{
    return &ahrs_status.euler_trigf[index];
}

int16_t remo_ahrs_get_roll_euler_cdeg(void)
{   
    return 100*ahrs_status.euler_deg[ROLL];
}
int16_t remo_ahrs_get_pitch_euler_cdeg(void)
{   
    return 100*ahrs_status.euler_deg[PITCH];
}
int16_t remo_ahrs_get_yaw_euler_cdeg(void)
{   
    return 100*ahrs_status.euler_deg[YAW];
}

uint8_t remo_ahrs_get_comp_filter_params(uint8_t *pbuf)
{
    pbuf[0] = 200; 
    pbuf[1] = 0; 
    pbuf[2] = (uint16_t)ahrs_comp.accel_params.kp;
    pbuf[3] = (uint16_t)ahrs_comp.accel_params.kp >> 8;
    pbuf[4] = 0;
    pbuf[5] = 0;
    pbuf[6] = 0;
    pbuf[7] = 0;
    pbuf[8] = (uint16_t)ahrs_comp.accel_params.ki;
    pbuf[9] = (uint16_t)ahrs_comp.accel_params.ki >> 8;
    pbuf[10] = 0;
    pbuf[11] = 0;
    pbuf[12] = 0;
    pbuf[13] = 0;
    pbuf[14] = (uint16_t)ahrs_comp.accel_params.err_limit;
    pbuf[15] = (uint16_t)ahrs_comp.accel_params.err_limit >> 8;
    pbuf[16] = (uint16_t)ahrs_comp.accel_params.output_limit;
    pbuf[17] = (uint16_t)ahrs_comp.accel_params.output_limit >> 8;
	
    return 18;
}

void remo_ahrs_set_comp_fliter_params(const uint8_t *pbuf)
{
//    ahrs_comp_filter.ahrs_pi.kp = (uint16_t)(pbuf[3] << 8) + pbuf[2];
    ahrs_comp.accel_params.ki = (uint16_t)(pbuf[9] << 8) + pbuf[8];
    ahrs_comp.accel_params.err_limit = (uint16_t)(pbuf[15] << 8) + pbuf[14];
    ahrs_comp.accel_params.output_limit = (uint16_t)(pbuf[17] << 8) + pbuf[16];
}
