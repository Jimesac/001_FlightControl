#include <math.h>
#include <stdlib.h>
#include "kinematics.h"
#include "esc_ctrl.h"
#include "imu.h"
#include "angle_encoder.h"
#include "ahrs.h"
#include "filter.h"

#define POSI_TIME_VEL_REF_SCALE_SHIFT_BIT_NUM  0x06

#define VEL_DPS_DELTA_PER_CYCLE   (0.12f)

vel_status_t vel_status = {
    .vel_gyro_smp = {0},
    .joint_vel_mea = {0},
	.joint_vel_mea_filter_sum = {0},
	.joint_vel_mea_filter_cnt = {0},
    .vel_ref = {0},
    .vel_aim = {0},
    .joint_vel_ref = {0},
    .ip_params = {
        .ip_method = MOTION_LINEAR,
        .vel_delta_pc = {0},
        .vel_nl_uint = {0},
        .vel_nl_coe = {0},
        .scurve_shrink = {0}
    },
    .ip_vars = {
        .vel_duration = {0},
        .vel_cnt = {0},

        .vel_start_end_time_num = {0},
        .vel_start = {0}
    },
    .motionless_flag = false,
    .vel_special_cal_flag = false
};

motion_matrix_t motion_matrix = {
    .euler_trig = {PTR_NULL, PTR_NULL, PTR_NULL},
    .joint_trig = {PTR_NULL, PTR_NULL, PTR_NULL},
    .b2j_matrix = {0},
    .e2b_matrix = {0},
    .b2e_matrix = {0},
    .n2b_matrix = {0},
    .b2n_matrix = {0},
    .n2j_matrix = {0},
    .e2j_matrix = {0}
};

#define DIFF_FILTER_ORDER   (15)
#define VEL_FIT_NUM   (9)
typedef struct{
	uint8_t median_filter_cnt;
	float median_filter[7];
	
	differential_filter_t diff_filter;
	float input[DIFF_FILTER_ORDER];
	
	float fit_data[VEL_FIT_NUM];
	uint8_t fit_index;
	float fit_sumx;
	float fit_delta;
	
	low_pass_filter_2nd_t lp_2nd_filter;
	
	float smp_time;
	float vel_filter;
	
	float vel_filter_last;
	float lpf1_alpha;
}filter_vel_t;

filter_vel_t roll_filter_vel, yaw_filter_vel;


static void remo_motion_calc_vel_ref_i(void);
static void remo_kinematics_calc_vel_ref_joint(void);

/**************************************************************************************************
 * 函数名称: remo_kinematics_params_init
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 速度处理的参数初始化
**************************************************************************************************/
void remo_kinematics_params_init(void)
{
    uint8_t i = 0;
	float temp_f = 0.0f;

    motion_matrix.euler_trig[ROLL]  = remo_ahrs_get_euler_trig(ROLL);
    motion_matrix.euler_trig[PITCH] = remo_ahrs_get_euler_trig(PITCH);
    motion_matrix.joint_trig[ROLL] = remo_encoder_get_joint_trig(ROLL);
    motion_matrix.joint_trig[PITCH] = remo_encoder_get_joint_trig(PITCH);
    motion_matrix.joint_trig[YAW] = remo_encoder_get_joint_trig(YAW);
    for (i = 0; i < 9; i ++)
    {
        motion_matrix.b2j_matrix[i] = 0;
        motion_matrix.e2b_matrix[i] = 0; 
        motion_matrix.b2e_matrix[i] = 0;
        motion_matrix.n2b_matrix[i] = 0; 
        motion_matrix.b2n_matrix[i] = 0; 
        motion_matrix.n2j_matrix[i] = 0;
        motion_matrix.e2j_matrix[i] = 0;
    }

    vel_status.ip_params.ip_method = MOTION_EXPONENT;
    for (i = 0; i < 3; i ++)
    {
        if (vel_status.ip_params.ip_method == MOTION_LINEAR)
        {
            vel_status.ip_params.vel_delta_pc[i] = VEL_DPS_DELTA_PER_CYCLE*5;
        }
        else if (vel_status.ip_params.ip_method == MOTION_EXPONENT)
        {
            // y = offset - amplitude / (1.0 + slope * x * x)
            // dif_y_max = 3 / 8 * c * sqrt(3 * b) = 3*sqrt(3) / 8 * c * sqrt(b) = 0.64952 * amp * sqrt(slope)
            // 0.64952 * 2^14 = 10641, 
            // vel_status.ip_params.vel_delta_pc[i] = 10641 * vel_status.ip_params.vel_nl_uint[i].amplitude >> 14;
            // vel_status.ip_params.vel_delta_pc[i] *= remo_sqrt_q14(vel_status.ip_params.vel_nl_uint[i].slope);
            // vel_status.ip_params.vel_delta_pc[i] >>= 14;
            vel_status.ip_params.vel_delta_pc[i] = VEL_DPS_DELTA_PER_CYCLE;
        }
        else 
        {
            vel_status.ip_params.vel_delta_pc[i] = 0;
        }
        vel_status.ip_params.vel_nl_coe[i] = 0;  // 非线性参数
    }
    // y = offset - amplitude / (1.0 + slope * x * x)
    vel_status.ip_params.vel_nl_uint.amplitude = -0.05;
    vel_status.ip_params.vel_nl_uint.slope = 0.01;
    vel_status.ip_params.vel_nl_uint.offset = 0.07;

    vel_status.ip_params.scurve_shrink[ROLL] = 0.07;
    vel_status.ip_params.scurve_shrink[PITCH] = 0.07;
    vel_status.ip_params.scurve_shrink[YAW] = 0.05;
	
	roll_filter_vel.smp_time = IMU_ODR__VEL_CTRL__TIME;
	differential_filter_one_sided_init(roll_filter_vel.smp_time, 1, DIFF_FILTER_ORDER, &roll_filter_vel.diff_filter);
	roll_filter_vel.diff_filter.input = roll_filter_vel.input;
	filter_butterworth_lpf_2nd_init(160, IMU_ODR__VEL_CTRL__FREQ, &roll_filter_vel.lp_2nd_filter);
	
	yaw_filter_vel.smp_time = IMU_ODR__VEL_CTRL__TIME;
	differential_filter_one_sided_init(yaw_filter_vel.smp_time, 1, DIFF_FILTER_ORDER, &yaw_filter_vel.diff_filter);
	yaw_filter_vel.diff_filter.input = yaw_filter_vel.input;
	filter_butterworth_lpf_2nd_init(150, IMU_ODR__VEL_CTRL__FREQ, &yaw_filter_vel.lp_2nd_filter);
	
	roll_filter_vel.fit_index = 0;
	roll_filter_vel.fit_sumx = 0.0f;
	roll_filter_vel.fit_delta = 0.0f;
	temp_f = 0.0f;
    for (uint8_t i = 0; i < VEL_FIT_NUM; i++)
    {
        roll_filter_vel.fit_sumx += i;
        temp_f += i*i;
        roll_filter_vel.fit_data[i] = 0.0f;
    }
    roll_filter_vel.fit_delta =  VEL_FIT_NUM*temp_f - (roll_filter_vel.fit_sumx*roll_filter_vel.fit_sumx);
    roll_filter_vel.fit_index = 0;
	
	roll_filter_vel.vel_filter_last = 0.0f;
	roll_filter_vel.lpf1_alpha = 0.9f;
	
	yaw_filter_vel.fit_index = 0;
	yaw_filter_vel.fit_sumx = 0.0f;
	yaw_filter_vel.fit_delta = 0.0f;
	temp_f = 0.0f;
    for (uint8_t i = 0; i < VEL_FIT_NUM; i++)
    {
        yaw_filter_vel.fit_sumx += i;
        temp_f += i*i;
        yaw_filter_vel.fit_data[i] = 0.0f;
    }
    yaw_filter_vel.fit_delta =  VEL_FIT_NUM*temp_f - (yaw_filter_vel.fit_sumx*yaw_filter_vel.fit_sumx);
    yaw_filter_vel.fit_index = 0;
	
	yaw_filter_vel.vel_filter_last = 0.0f;
	yaw_filter_vel.lpf1_alpha = 0.9f;
	
}

/**************************************************************************************************
 * 函数名称: remo_vel_ref_update
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 速度参考的更新
**************************************************************************************************/
void remo_vel_ref_update(void)
{
    remo_motion_calc_vel_ref_i();

    remo_kinematics_calc_vel_ref_joint();
}

/**************************************************************************************************
 * 函数名称: remo_motion_fp_matrix_calculation
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 计算转换矩阵，矩阵格式为：
 *          | matrix[0] matrix[1] matrix[2] |
 *          | matrix[3] matrix[4] matrix[5] |
 *          | matrix[6] matrix[7] matrix[8] |
 *          更新频率不必需速度环一样，可以低一些
**************************************************************************************************/
void remo_kinematics_matrix_calculation(void)
{
    float temp;
    float sr_sp, sr_cp, cr_sp, cr_cp;
	static float temp_filter = 0.0f;
	static bool flag_filter = false;

    const float *joint_deg = remo_encoder_get_joint_deg_ptr();

    sr_sp = motion_matrix.euler_trig[ROLL]->sin * motion_matrix.euler_trig[PITCH]->sin;
    sr_cp = motion_matrix.euler_trig[ROLL]->sin * motion_matrix.euler_trig[PITCH]->cos;
    cr_sp = motion_matrix.euler_trig[ROLL]->cos * motion_matrix.euler_trig[PITCH]->sin;
    cr_cp = motion_matrix.euler_trig[ROLL]->cos * motion_matrix.euler_trig[PITCH]->cos;

    // 欧拉角变化率->机体角速度的映射（用于将欧拉角的误差映射到机体坐标系上等）
    motion_matrix.e2b_matrix[0] = 1.0f;
    motion_matrix.e2b_matrix[1] = 0;
    motion_matrix.e2b_matrix[2] = -motion_matrix.euler_trig[PITCH]->sin;
    motion_matrix.e2b_matrix[3] = 0;
    motion_matrix.e2b_matrix[4] = motion_matrix.euler_trig[ROLL]->cos;
    motion_matrix.e2b_matrix[5] = sr_cp;
    motion_matrix.e2b_matrix[6] = 0;
    motion_matrix.e2b_matrix[7] = -motion_matrix.euler_trig[ROLL]->sin;
    motion_matrix.e2b_matrix[8] = cr_cp;


    if (fabs(motion_matrix.euler_trig[PITCH]->cos) <= 0.1f)
    {
        temp = (motion_matrix.euler_trig[PITCH]->cos > 0) ? 0.1f : -0.1f; // 防止除0
    }
    else
    {
        temp = motion_matrix.euler_trig[PITCH]->cos;
    }
    motion_matrix.b2e_matrix[0] = 1.0f;
	motion_matrix.b2e_matrix[1] = sr_sp / temp;
	motion_matrix.b2e_matrix[2] = cr_sp / temp;
	motion_matrix.b2e_matrix[3] = 0;
	motion_matrix.b2e_matrix[4] = motion_matrix.euler_trig[ROLL]->cos;
	motion_matrix.b2e_matrix[5] = -motion_matrix.euler_trig[ROLL]->sin;
	motion_matrix.b2e_matrix[6] = 0;
	motion_matrix.b2e_matrix[7] = motion_matrix.euler_trig[ROLL]->sin / temp;
	motion_matrix.b2e_matrix[8] = motion_matrix.euler_trig[ROLL]->cos / temp;

    // ZYX坐标系->机体坐标系的映射
    // 假设yaw欧拉角一直为0
    // motion_matrix.n2b_matrix[0] = motion_matrix.euler_trig[PITCH]->cos;
    // motion_matrix.n2b_matrix[1] = 0;
    // motion_matrix.n2b_matrix[2] = -motion_matrix.euler_trig[PITCH]->sin;
    // motion_matrix.n2b_matrix[3] = sr_sp;
    // motion_matrix.n2b_matrix[4] = motion_matrix.euler_trig[ROLL]->cos;
    // motion_matrix.n2b_matrix[5] = sr_cp;
    // motion_matrix.n2b_matrix[6] = cr_sp;
    // motion_matrix.n2b_matrix[7] = -motion_matrix.euler_trig[ROLL]->sin;
    // motion_matrix.n2b_matrix[8] = cr_cp;

    // 机体坐标系的映射->ZYX坐标系
    // motion_matrix.b2n_matrix[0] = motion_matrix.euler_trig[PITCH]->cos;
    // motion_matrix.b2n_matrix[1] = sr_sp;
    // motion_matrix.b2n_matrix[2] = cr_sp;
    // motion_matrix.b2n_matrix[3] = 0;
    // motion_matrix.b2n_matrix[4] = motion_matrix.euler_trig[PITCH]->cos;
    // motion_matrix.b2n_matrix[5] = -motion_matrix.euler_trig[ROLL]->sin;
    // motion_matrix.b2n_matrix[6] = -motion_matrix.euler_trig[PITCH]->sin;
    // motion_matrix.b2n_matrix[7] = sr_cp;
    // motion_matrix.b2n_matrix[8] = cr_cp;

    // 机体坐标系->关节角坐标系（用于将机体角速度映射到关节角速度上等）
    if (fabs(motion_matrix.joint_trig[PITCH]->cos) <= 0.35f)   // 0.35为69.5°
    {
        temp = (motion_matrix.joint_trig[PITCH]->cos > 0) ? 0.35f : -0.35f; // 防止除0
    }
    else
    {
        temp = motion_matrix.joint_trig[PITCH]->cos;
    }
	if (!flag_filter)
	{
		flag_filter = true;
		temp_filter = temp;
	}
	temp_filter += (temp - temp_filter)*0.04f;
    sr_sp = motion_matrix.joint_trig[ROLL]->sin * motion_matrix.joint_trig[PITCH]->sin;
    cr_sp = motion_matrix.joint_trig[ROLL]->cos * motion_matrix.joint_trig[PITCH]->sin;
    motion_matrix.b2j_matrix[0] = 1.0f;
    motion_matrix.b2j_matrix[1] = sr_sp / temp_filter;
    motion_matrix.b2j_matrix[2] = cr_sp / temp_filter;
    motion_matrix.b2j_matrix[3] = 0;
    motion_matrix.b2j_matrix[4] = motion_matrix.joint_trig[ROLL]->cos;
    motion_matrix.b2j_matrix[5] = -motion_matrix.joint_trig[ROLL]->sin;
    motion_matrix.b2j_matrix[6] = 0;
    motion_matrix.b2j_matrix[7] = motion_matrix.joint_trig[ROLL]->sin / temp_filter;
    motion_matrix.b2j_matrix[8] = motion_matrix.joint_trig[ROLL]->cos / temp_filter;
	
	motion_matrix.j2b_matrix[0] = 1.0f;
    motion_matrix.j2b_matrix[1] = 0.0f;
    motion_matrix.j2b_matrix[2] = -motion_matrix.joint_trig[PITCH]->sin;
    motion_matrix.j2b_matrix[3] = 0;
    motion_matrix.j2b_matrix[4] = motion_matrix.joint_trig[ROLL]->cos;
    motion_matrix.j2b_matrix[5] = motion_matrix.joint_trig[ROLL]->sin * motion_matrix.joint_trig[PITCH]->cos;
    motion_matrix.j2b_matrix[6] = 0;
    motion_matrix.j2b_matrix[7] = -motion_matrix.joint_trig[ROLL]->sin;
    motion_matrix.j2b_matrix[8] = motion_matrix.joint_trig[ROLL]->cos * motion_matrix.joint_trig[PITCH]->cos;
//    motion_matrix.b2j_matrix[6] = -motion_matrix.joint_trig[PITCH]->sin;
//    motion_matrix.b2j_matrix[7] = 0;// / temp;
//    motion_matrix.b2j_matrix[8] = motion_matrix.joint_trig[PITCH]->cos;// / temp;

    // 欧拉角坐标系->关节角坐标系的映射 
    remo_misc_matrix_multiply(3, 3, 3, motion_matrix.e2j_matrix, motion_matrix.b2j_matrix,
                              motion_matrix.e2b_matrix);

    // ZYX坐标系->关节角坐标系的映射
    // remo_fp_transfer_matrix_matrix_multiply(motion_matrix.n2j_matrix, motion_matrix.b2j_matrix,
    //                           motion_matrix.n2b_matrix);
}

/**************************************************************************************************
 * 函数名称: remo_motion_b2j_velo_map
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 将机体坐标系下的角速度映射到关节角坐标系下
*************************************************************s*************************************/
struct {
    float base_dcmz_start[3];
    float base_dcmz_err;
    float base_dcmz_err0;
    float base_dcmz_err_th;
    float gyro_mean[3];
    float joint_deg_last[3];
    float joint_deg_delta[3];
    float euler_deg_delta[3];
    float euler_deg_sum[3];
    float gyro_temp_offset[3];
    float gyro_temp_offset_last[3];
    uint16_t cnt;
    uint16_t num;
	
	uint8_t motion_fuse_flag[3];
}gyro_bias_correct = {
    .base_dcmz_start = {0.0f},
    .base_dcmz_err = 0.0f,
    .base_dcmz_err0 = 0.0f,
    .base_dcmz_err_th = 5e-6,
    .gyro_mean = {0.0f},
    .joint_deg_last = {0.0f},
    .joint_deg_delta = {0.0f},
    .euler_deg_delta = {0.0f},
    .euler_deg_sum = {0.0f},
    .gyro_temp_offset = {0},
    .gyro_temp_offset_last = {0},
    .cnt = 0,
    .num = 400,

    .motion_fuse_flag = {1 ,1, 1}};

void remo_kinematics_b2j_vel_map(void)
{
    const float *gyro_raw;
    uint32_t temp = 0;
    uint8_t i = 0;
    const float *joint_deg = remo_encoder_get_joint_deg_ptr();
    const float *gyro_fine_corr = remo_imu_get_gyro_fine_corr();
    const float *gyro_temp_offset_last = remo_imu_get_gyro_tempreture_offset();
    // static uint16_t static_cnt = 0;
    // static int32_t gyro_mean[3] = {0}, gyro_mean_last[3] = {0};
    // static float joint_deg_var[3] = {0}, joint_deg_sum[3] = {0}, joint_deg_mean[3] = {0};
    float gyro_temperature_temp[3] = {0.0f};
    
    const float *base_dcmz = ahrs_get_base_dcmz();
    float base_dcmz_err = 0.0f;
//    const int16_t *gyro_temp_offset = remo_imu_get_gyro_tempreture_offset();
    float temp_f[3] = {0.0f};
    static float jonit_deg_filter[3] = {0.0f};
    static uint8_t joint_deg_fliter_flag = 0;
    float temp_f0 = 0.0f;
    int16_t temp_i16 = 0;
    float imu_deg_filter[3] = {0.0f};
    
    float sum_y = 0;
    float sum_xy = 0;
    uint8_t index = 0;


    // 获取陀螺仪采样值。16位采样的话，直接用陀螺仪采样值范围就是(-16384~16383),与q14格式的旋转矩阵相乘不会出现越界的情况
    gyro_raw = remo_imu_get_gyro_corr();  
    vel_status.vel_gyro_smp[0] = gyro_raw[0];  
    vel_status.vel_gyro_smp[1] = gyro_raw[1];  
    vel_status.vel_gyro_smp[2] = gyro_raw[2];  
	
	
    

    // if (vel_status.motionless_flag && abs(vel_status.vel_gyro_smp[0]) < GIMBAL_LCOK_GYRO_STATIC_TH)
	if (vel_status.vel_special_cal_flag)
    {
//        temp = vel_status.vel_gyro_smp[0] * vel_status.vel_gyro_smp[0] +
//                    vel_status.vel_gyro_smp[2] * vel_status.vel_gyro_smp[2];
//        temp = sqrt(temp);

//        if (joint_deg[PITCH] < -GIMBAL_LCOK_VEL_SIGN_TH)
//        {
//            vel_status.joint_vel_mea[2] = (vel_status.vel_gyro_smp[0] > 0) ? temp:-temp;
//        }
//        else
//        {
//            vel_status.joint_vel_mea[2] = (vel_status.vel_gyro_smp[0] > 0) ? -temp:temp;
//        }
        
		if (joint_deg[ROLL] > GIMBALLOCK_ROLL_DEG)
		{
			vel_status.joint_vel_mea[2] = -vel_status.vel_gyro_smp[0];
		}
		else 
		{
			vel_status.joint_vel_mea[2] = vel_status.vel_gyro_smp[0];
		}
//        vel_status.joint_vel_mea[2] = -motion_matrix.joint_trig[PITCH]->sin*vel_status.vel_gyro_smp[0] + motion_matrix.joint_trig[PITCH]->cos* vel_status.vel_gyro_smp[2];
        vel_status.joint_vel_mea[2] = yaw_filter_vel.vel_filter;
        vel_status.joint_vel_mea[0] = roll_filter_vel.vel_filter;
        vel_status.joint_vel_mea[1] = motion_matrix.b2j_matrix[4] * vel_status.vel_gyro_smp[1] + motion_matrix.b2j_matrix[5] * vel_status.vel_gyro_smp[2];
		
        return;
    }

	//// 使用IMU测量得到的角速度，使用欧拉角计算得到关节角的角速度
	//if(!remo_cali_get_elec_angle_flag() || !remo_cali_get_motor_joint_valid() || mytest_vel_euler_flag)
	//{
	//	remo_misc_matrix_multiply(3, 3, 1, vel_status.joint_vel_mea, motion_matrix.b2e_matrix, vel_status.vel_gyro_smp);
	//	return;
	//}
	
	// 使用IMU测量得到的角速度，使用欧拉角计算得到关节角的角速度
	remo_misc_matrix_multiply(3, 3, 1, vel_status.joint_vel_mea, motion_matrix.b2j_matrix, vel_status.vel_gyro_smp);
	
	
	vel_status.joint_vel_mea_filter_sum[0] += vel_status.joint_vel_mea[0];
	vel_status.joint_vel_mea_filter_sum[1] += vel_status.joint_vel_mea[1];
	vel_status.joint_vel_mea_filter_sum[2] += vel_status.joint_vel_mea[2];
	vel_status.joint_vel_mea_filter_cnt[0]++;
	vel_status.joint_vel_mea_filter_cnt[1]++;
	vel_status.joint_vel_mea_filter_cnt[2]++;
	
	// 对IMU的漂移进行估计
	if (joint_deg_fliter_flag == 0)
	{
            joint_deg_fliter_flag = 1;
            jonit_deg_filter[0] = joint_deg[0];
            jonit_deg_filter[1] = joint_deg[1];
            jonit_deg_filter[2] = joint_deg[2];
	}
	else
	{
            jonit_deg_filter[0] += (joint_deg[0] - jonit_deg_filter[0])*0.08f;
            jonit_deg_filter[1] += (joint_deg[1] - jonit_deg_filter[1])*0.08f;
            jonit_deg_filter[2] += (joint_deg[2] - jonit_deg_filter[2])*0.08f;
	}
	if (gyro_bias_correct.cnt == 0)
	{
            for (i = 0; i < 3; i++)
            {
                    gyro_bias_correct.base_dcmz_start[i] = base_dcmz[i];
                    gyro_bias_correct.joint_deg_last[i] = jonit_deg_filter[i];
                    gyro_bias_correct.euler_deg_sum[i] = 0.0f;
            }
            gyro_bias_correct.base_dcmz_err = 0.0f;
            gyro_bias_correct.base_dcmz_err0 = base_dcmz[0] * base_dcmz[0] +  base_dcmz[1] * base_dcmz[1] + base_dcmz[2] * base_dcmz[2];
	}   
	if (++gyro_bias_correct.cnt <= gyro_bias_correct.num)   // 默认值是1600，也就是1s更新频率
	{
            for (i = 0; i < 3; i++)
            {
                    gyro_bias_correct.gyro_mean[i] += gyro_fine_corr[i];
                    gyro_bias_correct.euler_deg_sum[i] += vel_status.joint_vel_mea[i]*IMU_ODR__VEL_CTRL__TIME;
            }
		
            // 计算基座误差，判断基座是否移动
            base_dcmz_err = fabs(gyro_bias_correct.base_dcmz_start[0]*base_dcmz[0] + gyro_bias_correct.base_dcmz_start[1]*base_dcmz[1] + \
                            gyro_bias_correct.base_dcmz_start[2] * base_dcmz[2] - gyro_bias_correct.base_dcmz_err0);
            if (base_dcmz_err > gyro_bias_correct.base_dcmz_err) gyro_bias_correct.base_dcmz_err = base_dcmz_err;  // 求最大的基座误差，波动
            
            if (gyro_bias_correct.cnt == gyro_bias_correct.num)
            {
                // 计算关节角变化量和陀螺仪平均值
                for (i = 0; i < 3; i++)
                {
                        gyro_bias_correct.joint_deg_delta[i] = jonit_deg_filter[i] - gyro_bias_correct.joint_deg_last[i];
                        gyro_bias_correct.euler_deg_delta[i] = gyro_bias_correct.euler_deg_sum[i];
                        gyro_bias_correct.gyro_mean[i] /= gyro_bias_correct.num;
//				gyro_bias_correct.euler_deg_delta[i] = gyro_bias_correct.gyro_mean[i];
                }
                gyro_bias_correct.cnt = 0;

                // 基座没有动且处于竖直状态
                if (gyro_bias_correct.base_dcmz_err < gyro_bias_correct.base_dcmz_err_th)
                {
                    if (fabs(gyro_bias_correct.joint_deg_delta[0]) <= 0.03f && fabs(gyro_bias_correct.joint_deg_delta[1]) <= 0.02f && \
                            fabs(gyro_bias_correct.joint_deg_delta[2]) <= 0.02f)
                    {
                            // pitch和yaw都不动的情况。
                            // 静止不动时，gyro_mean会将gyro_temp_offset平缓收敛回gyro_offset上
                            for (i = 0; i < 3; i++)
                            {
                                    gyro_bias_correct.gyro_temp_offset[i] = gyro_bias_correct.gyro_mean[i];
                                    if (gyro_bias_correct.gyro_temp_offset[i] > 2.0f) gyro_bias_correct.gyro_temp_offset[i] = 2.0f;
                                    else if (gyro_bias_correct.gyro_temp_offset[i] < -2.0f) gyro_bias_correct.gyro_temp_offset[i] = -2.0f;
                            }

                            remo_imu_set_tempreture_offset(gyro_bias_correct.gyro_temp_offset, 0);
                    }
//				else if (fabs(gyro_bias_correct.joint_deg_delta[0]) <= 0.10f && fabs(gyro_bias_correct.joint_deg_delta[1]) <= 0.10f && \
//					fabs(gyro_bias_correct.joint_deg_delta[2]) <= 0.10f && fabs(joint_deg[0]) < 30.0f && \
//					fabs(joint_deg[1]) < 30.0f && fabs(joint_deg[2]) < 30.0f && fabs(gyro_bias_correct.euler_deg_delta[0]) < 0.3f && \
//					fabs(gyro_bias_correct.euler_deg_delta[1]) < 0.3f && fabs(gyro_bias_correct.euler_deg_delta[2]) < 0.3f )
                    else if (fabs(gyro_bias_correct.joint_deg_delta[0]) <= 0.10f && fabs(gyro_bias_correct.joint_deg_delta[1]) <= 0.10f && \
                            fabs(gyro_bias_correct.joint_deg_delta[2]) <= 0.10f && fabs(joint_deg[1]) < 60.0f &&\
                            fabs(gyro_bias_correct.euler_deg_delta[0]) < 0.3f && \
                            fabs(gyro_bias_correct.euler_deg_delta[1]) < 0.3f && fabs(gyro_bias_correct.euler_deg_delta[2]) < 0.3f )
                    {
                        remo_misc_matrix_multiply(3, 3, 1, imu_deg_filter, motion_matrix.j2b_matrix, gyro_bias_correct.joint_deg_delta);
                        
                        // 速度不是特别大的情况，1s内关节角变化为0.3°
                        for (i = 0; i < 3; i++)
                        {
                                if (gyro_bias_correct.motion_fuse_flag[i])
                                {
                                        
                                        gyro_temperature_temp[i] = -0.9f*(gyro_bias_correct.joint_deg_delta[i] - gyro_bias_correct.euler_deg_delta[i])* \
                                                        (IMU_ODR__VEL_CTRL__FREQ/gyro_bias_correct.num);
                                        if (gyro_temperature_temp[i] > 2.0f) gyro_temperature_temp[i] = 2.0f;
                                        else if (gyro_temperature_temp[i] < -2.0f) gyro_temperature_temp[i] = -2.0f;
                                }
                                else
                                {
                                        gyro_temperature_temp[i] = 0.0f;
                                }
                        }
                        
                        remo_misc_matrix_multiply(3, 3, 1, imu_deg_filter, motion_matrix.j2b_matrix, gyro_temperature_temp);
                        gyro_bias_correct.gyro_temp_offset[0] = imu_deg_filter[0] + gyro_temp_offset_last[0];
                        gyro_bias_correct.gyro_temp_offset[1] = imu_deg_filter[1] + gyro_temp_offset_last[1];
                        gyro_bias_correct.gyro_temp_offset[2] = imu_deg_filter[2] + gyro_temp_offset_last[2];
                        
//					// 速度不是特别大的情况，1s内关节角变化为0.3°
//					for (i = 0; i < 3; i++)
//					{
//						if (gyro_bias_correct.motion_fuse_flag[i])
//						{

//							gyro_bias_correct.gyro_temp_offset[i] = -0.9f*(imu_deg_filter[i] - gyro_bias_correct.euler_deg_delta[i])* \
//									(IMU_ODR__VEL_CTRL_FREQ/gyro_bias_correct.num);
//							
////							gyro_bias_correct.gyro_temp_offset[i] = -0.9f*(gyro_bias_correct.joint_deg_delta[i] - gyro_bias_correct.euler_deg_delta[i])* \
////									(IMU_ODR__VEL_CTRL_FREQ/gyro_bias_correct.num);
//							if (gyro_bias_correct.gyro_temp_offset[i] > 2.0f) gyro_bias_correct.gyro_temp_offset[i] = 2.0f;
//							else if (gyro_bias_correct.gyro_temp_offset[i] < -2.0f) gyro_bias_correct.gyro_temp_offset[i] = -2.0f;
//							
//						
//							gyro_bias_correct.gyro_temp_offset[i] += gyro_temp_offset_last[i];
//						}
//						else
//						{
//							gyro_bias_correct.gyro_temp_offset[i] = gyro_temp_offset_last[i];
//						}
//					}
                        
                        remo_imu_set_tempreture_offset(gyro_bias_correct.gyro_temp_offset, 1);
                    }
                }
            }
	}
}


/**************************************************************************************************
 * 函数名称: remo_motion_calc_vel_ref_i
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 启动和停止时目标角速度的变化处理，中间时刻目标角速度不变
**************************************************************************************************/
float scale_nfp[3] = {0};
bool nl_start_flag[3] = {false, false, false};
static void remo_motion_calc_vel_ref_i(void)
{
    uint8_t i;
    float temp_nfp;

    uint32_t delta_duration = 0;

    float delta_vel = 0;
    float temp;
    float velo_coe;
	float temp_f = 0.0f;

    // 根据目标速度求参考速度，参考速度不能突增突降
    switch (vel_status.ip_params.ip_method)
    {
    case MOTION_STEP:
        // 目前，不要使用这种模式，否则会出问题（以后可能会删除这种模式）
        vel_status.vel_ref[ROLL] = vel_status.vel_aim[ROLL];
        vel_status.vel_ref[PITCH] = vel_status.vel_aim[PITCH];
        vel_status.vel_ref[YAW] = vel_status.vel_aim[YAW];
        break;

    case MOTION_LINEAR:
        // 速度参考值线性增长
        // X轴为机体坐标系的速度，YZ为旋转的ZYX坐标系速度
        for (i = 0; i < 3; i++)
        {
            if (vel_status.vel_ref[i] < vel_status.vel_aim[i])
            {
                temp = vel_status.vel_ref[i] + vel_status.ip_params.vel_delta_pc[i];
                if (temp < vel_status.vel_aim[i])
                {
                    vel_status.vel_ref[i] = temp;
                }
                else
                {
                    vel_status.vel_ref[i] = vel_status.vel_aim[i];
                }
            }
            else
            {
                temp = vel_status.vel_ref[i] - vel_status.ip_params.vel_delta_pc[i];
                if (temp > vel_status.vel_aim[i])
                {
                    vel_status.vel_ref[i] = temp;
                }
                else
                {
                    vel_status.vel_ref[i] = vel_status.vel_aim[i];
                }
            }
        }
        break;

    case MOTION_EXPONENT:
        // 速度参数值按照S型曲线（或者类S型曲线）增长
        // 分为两种情况：
        //   一是速度响应固定型曲线，它是指速度响应时间是固定的，
        //     比如从0度到10度的时间，与0度到20度的时间是相同的
        //   二是速度响应线性型曲线；它是指速度响应时间与速度变化量成正比，
        //     即前后速度差越大，响应的时间越长
        for(i = 0; i < 3; i++)
        {
            if (vel_status.ip_vars.vel_cnt[i] == 0)
            {
                continue;
            }   

            if (vel_status.ip_vars.vel_cnt[i] == vel_status.ip_vars.vel_duration[i])
            {
                // 根据ai指令端的情况，自适应调整参数大小velo_coe
                // hypothesis:目标速度大时参数大，目标速度小时参数小
                scale_nfp[i] = nonlinear_func(vel_status.ip_params.vel_nl_uint, vel_status.vel_aim[i]);
                nl_start_flag[i] = true;
            }
            else if (vel_status.ip_vars.vel_cnt[i] > 0)
            {}
            vel_status.ip_vars.vel_cnt[i]--;
            
			temp_f = vel_status.ip_vars.vel_start[i] - vel_status.vel_aim[i];
			if (fabs(temp_f) < 2.0f && fabs(vel_status.vel_aim[i]) < 1.0f)
			{
				vel_status.vel_ref[i] = vel_status.vel_aim[i];
			}	
            else if (fabs(vel_status.vel_ref[i] - vel_status.vel_aim[i]) > vel_status.ip_params.vel_delta_pc[i])
            {
                if (nl_start_flag[i])
                {
                    delta_vel = vel_status.ip_vars.vel_duration[i] - vel_status.ip_vars.vel_cnt[i];
                }
                delta_vel = (delta_vel * delta_vel);
                temp = scale_nfp[i] * vel_status.ip_params.scurve_shrink[i];
                temp_nfp = (temp * delta_vel);
                vel_status.ip_params.vel_nl_coe[i] = 1.0f / (1.0f + temp_nfp);
				
				temp_f = vel_status.ip_vars.vel_start[i] - vel_status.vel_aim[i];
				if (fabs(temp_f) < 1.0f && fabs(vel_status.vel_aim[i]) < 1.0f)
				{
					vel_status.ip_params.vel_nl_coe[i] = 0.0f;
				}
                vel_status.vel_ref[i] = vel_status.vel_aim[i] + (vel_status.ip_params.vel_nl_coe[i] * temp_f);
				if (vel_status.vel_ref[i]*vel_status.vel_aim[i] < 0.0f)
                {
                    vel_status.vel_ref[i] = 0.1f * vel_status.vel_aim[i];
                }
            }
            else
            {
                vel_status.vel_ref[i] = vel_status.vel_aim[i];
                nl_start_flag[i] = false;
            }
        }
        break;

    default:;
    }
}

/**************************************************************************************************
 * 函数名称: remo_kinematics_calc_vel_ref_joint
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 计算关节角的参考速度
 *          ROLL轴的参考速度是机体坐标系下的速度；PITCH轴和YAW轴的参考速度是旋转的ZYX坐标系下的速度
**************************************************************************************************/
static void remo_kinematics_calc_vel_ref_joint(void)
{
    uint8_t i = 0;

    int32_q7_t temp = 0;
	int32_q7_t temp1 = 0, temp2 = 0;
	int32_q7_t q = 0;

	static int32_q14_t joint_vel_last[3] = {0};
	int32_q7_t joint_vel_filter[3] = {0};

	uint8_t cmd_velo_axis_mode = 0;

    for(i = 0; i <= 2; i++)
    {
        // 简单低通滤波，去除测量值的噪声干扰 y = 0.1*x + 0.9*y_minus
        joint_vel_filter[i] = 1638 * vel_status.joint_vel_mea[i] + 14746 * joint_vel_last[i];
	    joint_vel_last[i] = joint_vel_filter[i] >> Q14_SHIFT_BIT_NUM;

        // 计算哪些轴执行速度指令
        if(vel_status.ip_vars.vel_cnt[i] != 0)
	    {
		    cmd_velo_axis_mode += (0x01 << i);
	    }
    }

    vel_status.joint_vel_ref[0] = vel_status.vel_ref[0];
    vel_status.joint_vel_ref[1] = vel_status.vel_ref[1];
    vel_status.joint_vel_ref[2] = vel_status.vel_ref[2];
    return;

    switch(cmd_velo_axis_mode)
	{
		case 1: // 001B
			// 只控alpha轴速度
			vel_status.joint_vel_ref[0] = vel_status.vel_ref[0];
			break;
		case 2: // 010B
			// 只控y轴速度
			// temp = 1.0f / (motion_matrix.e2j_matrix[0] * motion_matrix.e2j_matrix[8]);
			// q = (motion_matrix.e2j_matrix[0] * motion_matrix.e2j_matrix[5] - motion_matrix.e2j_matrix[2] * motion_matrix.e2j_matrix[3]) * temp;
			// motion_joint.joint_vel_ref_rad[1] = (motion_matrix.e2j_matrix[4]  - q *  motion_matrix.e2j_matrix[7])* motion_cmd_ctrl.vel_ref_i_rad[1] 
			// 							+ q * joint_velo_rad_filter[2];
            temp = (motion_matrix.e2j_matrix[0] * motion_matrix.e2j_matrix[8]);  // 两个小于q14的数相乘，值肯定不会超过溢出
			temp = 1.0f / temp;
            temp1 = (motion_matrix.e2j_matrix[0] * motion_matrix.e2j_matrix[5]);
            temp2 = (motion_matrix.e2j_matrix[2] * motion_matrix.e2j_matrix[3]);
            q  = ((temp1 - temp2) * temp);

            temp1 = (q *  motion_matrix.e2j_matrix[7]);
            temp1 = motion_matrix.e2j_matrix[4] - temp1;
            temp1 = (temp1 * vel_status.vel_ref[1]);
            temp2 = (q * joint_vel_filter[2]) >> Q14_SHIFT_BIT_NUM;

            vel_status.joint_vel_ref[1] = temp1 + temp2;
            break;
		case 4: // 100B
			// 只控z轴速度
			// temp = 1.0f / (motion_matrix.e2j_matrix[0] * motion_matrix.e2j_matrix[4]);
			// q = (motion_matrix.e2j_matrix[0] * motion_matrix.e2j_matrix[7] - motion_matrix.e2j_matrix[1] * motion_matrix.e2j_matrix[6]) * temp;
			// motion_joint.joint_vel_ref_rad[2] = (motion_matrix.e2j_matrix[8] - q *  motion_matrix.e2j_matrix[5])* motion_cmd_ctrl.vel_ref_i_rad[2] 
			// 							+ q * joint_velo_rad_filter[1];
            
            temp = motion_matrix.e2j_matrix[0] * motion_matrix.e2j_matrix[4];
            temp = 1.0f / temp;
            temp1 = (motion_matrix.e2j_matrix[0] * motion_matrix.e2j_matrix[7]);
            temp2 = (motion_matrix.e2j_matrix[1] * motion_matrix.e2j_matrix[6]);
            q  = ((temp1 - temp2) * temp) >> Q14_SHIFT_BIT_NUM;

            temp1 = (q *  motion_matrix.e2j_matrix[5]);
            temp1 = motion_matrix.e2j_matrix[8] - temp1;
            temp1 = (temp1 * vel_status.vel_ref[2]);
            temp2 = (q * joint_vel_filter[1]); 

            vel_status.joint_vel_ref[2] = temp1 + temp2;
			break;  
		case 3: // 011B
			// 只控alpha和y轴速度
			// motion_joint.joint_vel_ref_rad[0] = motion_cmd_ctrl.vel_ref_i_rad[0];
			// temp = 1.0f / (motion_matrix.e2j_matrix[0] * motion_matrix.e2j_matrix[8]);
			// q = (motion_matrix.e2j_matrix[0] * motion_matrix.e2j_matrix[5] - motion_matrix.e2j_matrix[2] * motion_matrix.e2j_matrix[3]) * temp;
			// motion_joint.joint_vel_ref_rad[1] = (motion_matrix.e2j_matrix[4] - q *  motion_matrix.e2j_matrix[7])* motion_cmd_ctrl.vel_ref_i_rad[1] 
			// 							+ q * joint_velo_rad_filter[2];

            vel_status.joint_vel_ref[0] = vel_status.vel_ref[0];

            temp = motion_matrix.e2j_matrix[0] * motion_matrix.e2j_matrix[8];
            temp = 1.0f / temp;
            temp1 = (motion_matrix.e2j_matrix[0] * motion_matrix.e2j_matrix[5]);
            temp2 = (motion_matrix.e2j_matrix[2] * motion_matrix.e2j_matrix[3]);
            q  = ((temp1 - temp2) * temp);

            temp1 = (q *  motion_matrix.e2j_matrix[7]);
            temp1 = motion_matrix.e2j_matrix[4] - temp1;
            temp1 = (temp1 * vel_status.vel_ref[1]);
            temp2 = (q * joint_vel_filter[2]); 

            vel_status.joint_vel_ref[1] = temp1 + temp2;
			break;
		case 5: // 101B
			// 只控alpha和z轴速度
			// motion_joint.joint_vel_ref_rad[0] = motion_cmd_ctrl.vel_ref_i_rad[0];
			// temp = 1.0f / (motion_matrix.e2j_matrix[0] * motion_matrix.e2j_matrix[4]);
			// q = (motion_matrix.e2j_matrix[0] * motion_matrix.e2j_matrix[7]) * temp;
			// motion_joint.joint_vel_ref_rad[2] = (motion_matrix.e2j_matrix[8] - q *  motion_matrix.e2j_matrix[5])* motion_cmd_ctrl.vel_ref_i_rad[2]
			// 							+ q * joint_velo_rad_filter[1];

            vel_status.joint_vel_ref[0] = vel_status.vel_ref[0];

            temp = motion_matrix.e2j_matrix[0] * motion_matrix.e2j_matrix[4];
            temp = 1.0f / temp;
            temp1 = (motion_matrix.e2j_matrix[0] * motion_matrix.e2j_matrix[7]);
            q = (temp1 * temp) >> Q14_SHIFT_BIT_NUM;

            temp1 = (q *  motion_matrix.e2j_matrix[5]);
            temp1 = motion_matrix.e2j_matrix[8] - temp1;
            temp1 = (temp1 * vel_status.vel_ref[2]);
            temp2 = (q * joint_vel_filter[1]); 

            vel_status.joint_vel_ref[1] = temp1 + temp2;
			break;
		case 6: // 110B
			// 只控y和z轴速度
			// motion_joint.joint_vel_ref_rad[1] = motion_matrix.e2j_matrix[4] * motion_cmd_ctrl.vel_ref_i_rad[1] + motion_matrix.e2j_matrix[5] * motion_cmd_ctrl.vel_ref_i_rad[2];
			// motion_joint.joint_vel_ref_rad[2] = motion_matrix.e2j_matrix[7] * motion_cmd_ctrl.vel_ref_i_rad[1] + motion_matrix.e2j_matrix[8] * motion_cmd_ctrl.vel_ref_i_rad[2];

            temp1 = (motion_matrix.e2j_matrix[4] * vel_status.vel_ref[1]);
            temp2 = (motion_matrix.e2j_matrix[5] * vel_status.vel_ref[2]);
            vel_status.joint_vel_ref[1] = temp1 + temp2;

            temp1 = (motion_matrix.e2j_matrix[7] * vel_status.vel_ref[1]);
            temp2 = (motion_matrix.e2j_matrix[8] * vel_status.vel_ref[2]);
            vel_status.joint_vel_ref[2] = temp1 + temp2;
			break;
		case 7: // 111B
			// 控alpha，x，z轴速度
			// motion_joint.joint_vel_ref_rad[0] = motion_cmd_ctrl.vel_ref_i_rad[0];
			// motion_joint.joint_vel_ref_rad[1] = motion_matrix.e2j_matrix[4] * motion_cmd_ctrl.vel_ref_i_rad[1] + motion_matrix.e2j_matrix[5] * motion_cmd_ctrl.vel_ref_i_rad[2];
			// motion_joint.joint_vel_ref_rad[2] = motion_matrix.e2j_matrix[7] * motion_cmd_ctrl.vel_ref_i_rad[1]	+ motion_matrix.e2j_matrix[8] * motion_cmd_ctrl.vel_ref_i_rad[2];
			
            vel_status.joint_vel_ref[0] = vel_status.vel_ref[0];

            temp1 = (motion_matrix.e2j_matrix[4] * vel_status.vel_ref[1]);
            temp2 = (motion_matrix.e2j_matrix[5] * vel_status.vel_ref[2]);
            vel_status.joint_vel_ref[1] = temp1 + temp2;

            temp1 = (motion_matrix.e2j_matrix[7] * vel_status.vel_ref[1]);
            temp2 = (motion_matrix.e2j_matrix[8] * vel_status.vel_ref[2]);
            vel_status.joint_vel_ref[2] = temp1 + temp2;
            break;
		default:
			// motion_joint.joint_vel_ref_rad[0] = 0.0f;
			// motion_joint.joint_vel_ref_rad[1] = 0.0f;
			// motion_joint.joint_vel_ref_rad[2] = 0.0f;

            vel_status.joint_vel_ref[0] = 0;
            vel_status.joint_vel_ref[1] = 0;
            vel_status.joint_vel_ref[2] = 0;
			break;
	}
}

/**************************************************************************************************
 * 函数名称: remo_kinematics_reset_vel_ref_and_ip_var
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 返回关节角速度参考值
**************************************************************************************************/
void remo_kinematics_reset_vel_ref_and_ip_var(uint8_t index)
{
    if (index > 2)
    {
        return;
    }
    
    vel_status.vel_ref[index] = 0;
    vel_status.vel_aim[index] = 0;
    vel_status.joint_vel_ref[index] = 0;

    vel_status.ip_vars.vel_duration[index] = 0;
    vel_status.ip_vars.vel_cnt[index] = 0;
    vel_status.ip_vars.vel_start_end_time_num[index] = 0;
    vel_status.ip_vars.vel_start[index] = 0;
}

/**************************************************************************************************
 * 函数名称: remo_kinematics_set_vel_aim
 * 输入参数: int16_q7_t, uint8_t
 * 返回结果: void
 * 功能描述: 设置目标速度
**************************************************************************************************/
void remo_kinematics_set_vel_aim(float vel_aim, uint8_t index)
{
    if (index > 2)
    {
        return;
    }
    else
    {
        vel_status.vel_aim[index] = vel_aim;
    }
}



void remo_kinematics_set_joint_vel_ref(float vel_ref, uint8_t index)
{
    if (index > 2)
    {
        return;
    }
    else
    {
        vel_status.joint_vel_ref[index] = vel_ref;
    }
}

/**************************************************************************************************
 * 函数名称: remo_kinematics_set_vel_aim
 * 输入参数: int16_q7_t, uint8_t
 * 返回结果: void
 * 功能描述: 设置目标速度
**************************************************************************************************/
void remo_kinematics_set_vel_start(float vel, uint8_t index)
{
    if (index > 2)
    {
        return;
    }
    else
    {
        vel_status.ip_vars.vel_start[index] = vel;
    }
    
}

/**************************************************************************************************
 * 函数名称: remo_kinematics_set_vel_duration_cnt
 * 输入参数: 
 * 返回结果: void
 * 功能描述: 设置速度技术参数
**************************************************************************************************/
void remo_kinematics_set_vel_duration_cnt(uint32_t duration, uint32_t num, uint8_t index)
{
    if (index > 2)
    {
        return;
    }
    vel_status.ip_vars.vel_duration[index] = duration;
    vel_status.ip_vars.vel_cnt[index] = num;
}

/**************************************************************************************************
 * 函数名称: remo_kinematics_reset_vel_ref
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 重置返回关节角速度参考值为0
**************************************************************************************************/
void remo_kinematics_reset_vel_ref(uint8_t index)
{
    if (index > 2)
    {
        return;
    }
    else
    {
        vel_status.vel_ref[index] = 0;
    }
}

/**************************************************************************************************
 * 函数名称: remo_kinematics_set_vel_ip_method
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 设置速度插值方式
**************************************************************************************************/
void remo_kinematics_set_vel_ip_method(motion_im_t method)
{
    vel_status.ip_params.ip_method = method;
}

/**************************************************************************************************
 * 函数名称: remo_kinematics_set_vel_spcl_motionless
 * 输入参数: bool 
 * 返回结果: void
 * 功能描述: 设置速度特殊处理方式下的静止标志
**************************************************************************************************/
void remo_kinematics_set_vel_spcl_motionless(bool flag)
{
    vel_status.motionless_flag = flag;
}

/**************************************************************************************************
 * 函数名称: remo_kinematics_set_vel_cal_way
 * 输入参数: bool 
 * 返回结果: void
 * 功能描述: 设置速度特殊处理方式标志
**************************************************************************************************/
void remo_kinematics_set_vel_cal_way(bool flag)
{
    vel_status.vel_special_cal_flag = flag;
}

void remo_kinematics_set_gyro_bias_motion_fuse_flag(uint8_t flag0, uint8_t flag1, uint8_t flag2)
{
	gyro_bias_correct.motion_fuse_flag[0] = flag0;
	gyro_bias_correct.motion_fuse_flag[1] = flag1;
	gyro_bias_correct.motion_fuse_flag[2] = flag2;
}


vel_status_t *remo_kinematics_get_vel_status(void)
{
    return &vel_status;
}

/**************************************************************************************************
 * 函数名称: remo_kinematics_get_vel_delta_pc
 * 输入参数: uint8_t
 * 返回结果: void
 * 功能描述: 返回每个周期速度变化量，单位dps
**************************************************************************************************/
float remo_kinematics_get_vel_delta_pc(uint8_t index)
{
    if (index > 2)
    {
        return 10000;
    }
    else
    {
        return vel_status.ip_params.vel_delta_pc[index];
    }
}

/**************************************************************************************************
 * 函数名称: remo_kinematics_get_joint_vel_smp
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 返回关节角速度地址
**************************************************************************************************/
const float *remo_kinematics_get_joint_vel_smp(void)
{
    return  vel_status.joint_vel_mea;
}

float remo_kinematics_get_joint_vel_smp_filter(uint8_t index)
{
	float filter = 0.0f;
	if (index > 2) return 0.0f;
	
	if (vel_status.joint_vel_mea_filter_cnt[index] == 0)
	{
		filter = vel_status.joint_vel_mea[index];
	}
	else
	{
		filter = vel_status.joint_vel_mea_filter_sum[index] / vel_status.joint_vel_mea_filter_cnt[index];
	}
	vel_status.joint_vel_mea_filter_sum[index] = 0.0f;
	vel_status.joint_vel_mea_filter_cnt[index] = 0;
	
    return  filter;
}


/**************************************************************************************************
 * 函数名称: remo_kinematics_get_joint_vel_ref_i16
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 返回关节角速度地址
**************************************************************************************************/
const float *remo_kinematics_get_joint_vel_ref(void)
{
    return vel_status.joint_vel_ref;
}

/**************************************************************************************************
 * 函数名称: remo_kinematics_get_motion_matrix
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 返回运行矩阵
**************************************************************************************************/
const motion_matrix_t *remo_kinematics_get_motion_matrix(void)
{
    return &motion_matrix;
}

int16_t remo_kinematics_get_roll_vel_nl_coe(void)
{
	return vel_status.ip_params.vel_nl_coe[ROLL]*1000;
}

int16_t remo_kinematics_get_pitch_vel_nl_coe(void)
{
	return vel_status.ip_params.vel_nl_coe[PITCH]*1000;
}

int16_t remo_kinematics_get_yaw_vel_nl_coe(void)
{
	return vel_status.ip_params.vel_nl_coe[YAW]*1000;
}

int16_t remo_kinematics_get_roll_joint_vel_smp_ddps(void)
{    
    return 100*vel_status.joint_vel_mea[ROLL];
}
int16_t remo_kinematics_get_pitch_joint_vel_smp_ddps(void)
{    
    return 100*vel_status.joint_vel_mea[PITCH];
}
int16_t remo_kinematics_get_yaw_joint_vel_smp_ddps(void)
{    
    return 100*vel_status.joint_vel_mea[YAW];
}


int16_t remo_kinematics_get_roll_ref_vel_ddps(void)
{ 
    return 100*vel_status.joint_vel_ref[ROLL]; 
}
int16_t remo_kinematics_get_pitch_ref_vel_ddps(void)
{ 
    return 100*vel_status.joint_vel_ref[PITCH]; 
}
int16_t remo_kinematics_get_yaw_ref_vel_ddps(void)
{ 
    return 100*vel_status.joint_vel_ref[YAW]; 
}

int16_t remo_kinematics_get_roll_joint_vel_fit_ddps(void)
{
	return 100*roll_filter_vel.vel_filter;
}

int16_t remo_kinematics_get_yaw_joint_vel_fit_ddps(void)
{
	return 100*yaw_filter_vel.vel_filter;
}