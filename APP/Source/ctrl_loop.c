#include "ctrl_loop.h"
#include "esc_ctrl.h"
#include "user_config.h"
#include "misc.h"
#include "imu.h"
#include "ahrs.h"
#include "math.h"
#include "bsp_timer.h"
#include "angle_encoder.h"


float POS_CTRL_ROLL_P_LIMIT = 50.0f;
float POS_CTRL_PITCH_P_LIMIT = 50.0f;
float POS_CTRL_YAW_P_LIMIT = 50.0f;

// ---- 扫频信号激励
struct {
    uint8_t run_state;
    float u_amp;
    uint16_t sample_freq;
    float start_freq;
    float end_freq;
    uint16_t end_time_ms;

    uint16_t num;
    uint16_t count;

    float u;
    float y;
    int8_t sign;

    bool params_init_flag;
}chrip_resp = 
{
    .run_state = 0,

    .u_amp = 8000.0f,
    .sample_freq = IMU_ODR__VEL_CTRL__FREQ,
    .start_freq = 20.0f,
    .end_freq = 100.0f,
    .end_time_ms = 2048*1000/IMU_ODR__VEL_CTRL__FREQ,

    .num = 0,
    .count = 0,

    .u = 0,
    .y = 0,
    .sign = -1,

    .params_init_flag = false
};

pos_ctrl_t pos_ctrl[3];
vel_ctrl_t vel_ctrl[3];

struct {
    float b2e_matrix[3][3];
    float b2j_matrix[3][3];
    uint8_t state;
}jacob_matrix;

bool torque_zero_output_flag = false;

bool ctrl_renew_params_flag[3] = {false, false, false};
bool vel_ctrl_lpf_flag[3] = {0, 0, true};

bool angle_linear_calib_flag[3] = {false, false, false};

const float *ctrl_joint_deg;
const float *ctrl_euler_deg;
const float *ctrl_gyro_deg; 

static void remo_ctrl_joint_vel_update(void);
static void remo_ctrl_loop_renew_params(void); 

/**************************************************************************************************
 * 函数名称: remo_ctrl_loop_init
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 控制环的参数初始化
**************************************************************************************************/
void remo_ctrl_loop_params_init(void)
{
    //--- 1.设置位置环参数
    // 1.1 设置误差的限幅, 参考和测量角度定标采用q7格式(精度0.008)，则角度误差为90*2^7 = 11520
    pos_ctrl[ROLL].ctrl_p.err_limit = 90.0f;
    pos_ctrl[PITCH].ctrl_p.err_limit = 90.0f;
    pos_ctrl[YAW].ctrl_p.err_limit = 90.0f;
    // 1.2 设置kp非线性参数
    remo_ctrl_loop_reset_roll_pos_pid();
    remo_ctrl_loop_reset_pitch_pos_pid();
    remo_ctrl_loop_reset_yaw_pos_pid();
    // 1.3 设置比例控制的限幅
    pos_ctrl[ROLL].ctrl_p.p_limit = POS_CTRL_ROLL_P_LIMIT;
    pos_ctrl[PITCH].ctrl_p.p_limit = POS_CTRL_PITCH_P_LIMIT;
    pos_ctrl[YAW].ctrl_p.p_limit = POS_CTRL_YAW_P_LIMIT;
    

    //--- 2.设置速度环参数
    // 2.1 设置误差的限幅,实际电机对角度误差(角加速度)响应有限，此处不必按角速度的测量值范围来限幅
    vel_ctrl[ROLL].ctrl_pid.err_limit = 20.0f;
    vel_ctrl[PITCH].ctrl_pid.err_limit = 20.0f;
    vel_ctrl[YAW].ctrl_pid.err_limit = 20.0f;
    remo_ctrl_loop_reset_roll_vel_pid();
    remo_ctrl_loop_reset_pitch_vel_pid();
    remo_ctrl_loop_reset_yaw_vel_pid();
    filter_butterworth_lpf_2nd_init(100, IMU_ODR__VEL_CTRL__FREQ, &vel_ctrl[ROLL].ctrl_pid.butterworth_2nd);
    filter_butterworth_lpf_2nd_init(200, IMU_ODR__VEL_CTRL__FREQ, &vel_ctrl[PITCH].ctrl_pid.butterworth_2nd);
    filter_butterworth_lpf_2nd_init(200, IMU_ODR__VEL_CTRL__FREQ, &vel_ctrl[YAW].ctrl_pid.butterworth_2nd);
    ctrl_pid_controller_init_err_fit(&vel_ctrl[ROLL].ctrl_pid);
    ctrl_pid_controller_init_err_fit(&vel_ctrl[PITCH].ctrl_pid);
    ctrl_pid_controller_init_err_fit(&vel_ctrl[YAW].ctrl_pid);
    // 2.4 设置比例积分控制的限幅
    vel_ctrl[ROLL].ctrl_pid.ki_limit = MOTOR_ROLL_TORQUE_LIMIT;
    vel_ctrl[PITCH].ctrl_pid.ki_limit = MOTOR_PITCH_TORQUE_LIMIT;
    vel_ctrl[YAW].ctrl_pid.ki_limit = MOTOR_YAW_TORQUE_LIMIT;
    vel_ctrl[ROLL].ctrl_pid.pid_limit = MOTOR_ROLL_TORQUE_LIMIT;
    vel_ctrl[PITCH].ctrl_pid.pid_limit = MOTOR_PITCH_TORQUE_LIMIT;
    vel_ctrl[YAW].ctrl_pid.pid_limit = MOTOR_YAW_TORQUE_LIMIT;

    // 陷波
    remo_ctrl_loop_reset_roll_notch_filter();
    remo_ctrl_loop_reset_pitch_notch_filter();
    remo_ctrl_loop_reset_yaw_notch_filter();

    // 低通滤波
    filter_butterworth_lpf_2nd_init(250, IMU_ODR__VEL_CTRL__FREQ, &vel_ctrl[ROLL].lp_2nd_filter);
    filter_butterworth_lpf_2nd_init(250, IMU_ODR__VEL_CTRL__FREQ, &vel_ctrl[PITCH].lp_2nd_filter);
    filter_butterworth_lpf_2nd_init(280, IMU_ODR__VEL_CTRL__FREQ, &vel_ctrl[YAW].lp_2nd_filter);

    // 超前滞后补偿
    ctrl_leadlag_unit_init(1.0f/(float)IMU_ODR__VEL_CTRL__FREQ, 70, 110, &vel_ctrl[ROLL].leadlag.lead);
    ctrl_leadlag_unit_init(1.0f/(float)IMU_ODR__VEL_CTRL__FREQ, 55, 25, &vel_ctrl[ROLL].leadlag.lag);
    ctrl_leadlag_unit_init(1.0f/(float)IMU_ODR__VEL_CTRL__FREQ, 60, 120, &vel_ctrl[PITCH].leadlag.lead);
    ctrl_leadlag_unit_init(1.0f/(float)IMU_ODR__VEL_CTRL__FREQ, 55, 35, &vel_ctrl[PITCH].leadlag.lag);
    ctrl_leadlag_unit_init(1.0f/(float)IMU_ODR__VEL_CTRL__FREQ, 65, 105, &vel_ctrl[YAW].leadlag.lead);
    ctrl_leadlag_unit_init(1.0f/(float)IMU_ODR__VEL_CTRL__FREQ, 55, 26, &vel_ctrl[YAW].leadlag.lag);
    ctrl_leadlag_set_out_limit(-MOTOR_ROLL_TORQUE_LIMIT, MOTOR_ROLL_TORQUE_LIMIT, &vel_ctrl[ROLL].leadlag);
    ctrl_leadlag_set_out_limit(-MOTOR_PITCH_TORQUE_LIMIT, MOTOR_PITCH_TORQUE_LIMIT, &vel_ctrl[PITCH].leadlag);
    ctrl_leadlag_set_out_limit(-MOTOR_YAW_TORQUE_LIMIT, MOTOR_YAW_TORQUE_LIMIT, &vel_ctrl[YAW].leadlag);

    pos_ctrl[ROLL].ctrl_enable = true;
    pos_ctrl[PITCH].ctrl_enable = true;
    pos_ctrl[YAW].ctrl_enable = true;
    
    vel_ctrl[ROLL].ctrl_enable = true;
    vel_ctrl[PITCH].ctrl_enable = true;
    vel_ctrl[YAW].ctrl_enable = true;
	
    vel_ctrl[ROLL].kp_adaptive_flag = true;
    vel_ctrl[PITCH].kp_adaptive_flag = true;
    vel_ctrl[YAW].kp_adaptive_flag = true;
    
    vel_ctrl[ROLL].delta_ctrl_output_th = 1000;
    vel_ctrl[PITCH].delta_ctrl_output_th = 3000;
    vel_ctrl[YAW].delta_ctrl_output_th = 3000;
    
    vel_ctrl[ROLL].ctrl_output_last = 0.0f;
    vel_ctrl[PITCH].ctrl_output_last = 0.0f;
    vel_ctrl[YAW].ctrl_output_last = 0.0f;

    ctrl_joint_deg = remo_encoder_get_joint_deg_ptr();
    ctrl_euler_deg = ahrs_get_euler_deg_ptr();
    ctrl_gyro_deg = remo_imu_get_gyro_corr(); 
}

/**************************************************************************************************
 * 函数名称: remo_ctrl_loop_pos_ctrl
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 位置环控制
**************************************************************************************************/
int8_t mytest_pos_fp_flag[3] = {1, 1, 1};
float mytest_rotor_pos_kp[3] = {10.0f, 6.0f, 0.1f};
float yaw_euler_pos[2] = {0.0f};
void remo_ctrl_loop_pos_ctrl(void)
{
    static uint16_t limit_scale_count[3] = {0, 0, 0};
    static float yaw_pos_mea_last = 0.0f;
    static uint8_t ctrl_enable_last[3] = {1 ,1 , 1};
    uint32_t vel_pidout_limit = 0;
    static uint32_t pos_ctrl_cnt[3] = {0};


    if (limit_scale_count[0] < 2000)   // 10 * 1200
    {
        limit_scale_count[0] ++;

        vel_pidout_limit = MOTOR_ROLL_TORQUE_LIMIT * limit_scale_count[0];
        vel_pidout_limit /= 2000; 
        if (vel_pidout_limit < 200)
        {
            vel_pidout_limit = MOTOR_ROLL_TORQUE_LIMIT * 0.1;
        }
        vel_ctrl[ROLL].ctrl_pid.ki_limit = vel_pidout_limit;
    }
    if (limit_scale_count[1] < 3000)   // 10 * 1200
    {
        limit_scale_count[1] ++;
        vel_pidout_limit = MOTOR_PITCH_TORQUE_LIMIT * limit_scale_count[1];
        vel_pidout_limit /= 3000; 
        if (vel_pidout_limit < 300)
        {
            vel_pidout_limit = MOTOR_PITCH_TORQUE_LIMIT * 0.1;
        }
        vel_ctrl[PITCH].ctrl_pid.ki_limit = vel_pidout_limit;
    }
    if (limit_scale_count[2] < 3000)   // 10 * 1200
    {
        limit_scale_count[2] ++;
        vel_pidout_limit = MOTOR_YAW_TORQUE_LIMIT * limit_scale_count[2];
        vel_pidout_limit /= 3000; 
        if (vel_pidout_limit < 300)
        {
            vel_pidout_limit = MOTOR_YAW_TORQUE_LIMIT * 0.1f;
        }
        vel_ctrl[YAW].ctrl_pid.ki_limit = vel_pidout_limit;
    }
    
    for (uint8_t i = 0; i < 3; i++)
    {
        if (pos_ctrl[i].ctrl_type == POS_JOINT_CTRL)
        {
            pos_ctrl[i].pos_mea = ctrl_joint_deg[i];
        }
        else 
        {
            pos_ctrl[i].pos_mea = ctrl_euler_deg[i];
        }
    }

    if (pos_ctrl[0].ctrl_enable)
    {
        pos_ctrl[0].ctrl_p.err = pos_ctrl[0].pos_ref - pos_ctrl[0].pos_mea;
        pos_ctrl[0].ctrl_output = ctrl_p_controller(&pos_ctrl[0].ctrl_p);
    }
    else if (pos_ctrl[0].ctrl_enable != ctrl_enable_last[0])
    {
        pos_ctrl[0].ctrl_output = 0;
    }
	ctrl_enable_last[0] = pos_ctrl[0].ctrl_enable;



    if (pos_ctrl[1].ctrl_enable)
    {
        pos_ctrl[1].ctrl_p.err = pos_ctrl[1].pos_ref - pos_ctrl[1].pos_mea;
        pos_ctrl[1].ctrl_output = ctrl_p_controller(&pos_ctrl[1].ctrl_p);
    }
    else if (pos_ctrl[1].ctrl_enable != ctrl_enable_last[1])
    {
        pos_ctrl[1].ctrl_output = 0;
    }
    ctrl_enable_last[1] = pos_ctrl[1].ctrl_enable;

    if (pos_ctrl[2].ctrl_enable)
    {
        pos_ctrl[2].ctrl_p.err = pos_ctrl[2].pos_ref - pos_ctrl[2].pos_mea;
        pos_ctrl[2].ctrl_output = ctrl_p_controller(&pos_ctrl[2].ctrl_p);
    }
    else if (pos_ctrl[2].ctrl_enable != ctrl_enable_last[2])
    {
        pos_ctrl[2].ctrl_output = 0;
    }
    ctrl_enable_last[2] = pos_ctrl[2].ctrl_enable;

}

/**************************************************************************************************
 * 函数名称: remo_ctrl_loop_pos_ctrl
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 速度环控制
**************************************************************************************************/
int8_t mytest_vel_ctrl_sign[3] = {1, 1, 1};
int8_t mytest_torque_valid_flag[3] = {0, 1, -1};
int32_t torque_sum = 0;
uint32_t torque_cnt[3] = {0, 0, 0};
bool torque_ctrl_flag[3] = {false, false, false};


float mytest_ctrl_params[2] = {0.1f, 0.2f};
uint32_t vel_ctrl_cnt = 0;
uint32_t ctrl_cmp_cnt[3] = {0};
uint32_t ctrl_comp_delay_cnt[3] = {0};
float ctrl_cmp_last[3][3] = {0};
void remo_ctrl_loop_vel_ctrl(void)
{
    uint8_t i = 0;
    
    remo_ctrl_joint_vel_update();
    remo_ctrl_loop_renew_params();
    
    if (pos_ctrl[ROLL].ctrl_enable)  vel_ctrl[ROLL].vel_target = 0.0f;
    vel_ctrl[ROLL].vel_ref = vel_ctrl[ROLL].vel_target + pos_ctrl[ROLL].ctrl_output;
    vel_ctrl[ROLL].ctrl_pid.err = vel_ctrl[ROLL].vel_ref - vel_ctrl[ROLL].vel_mea;
    vel_ctrl[ROLL].ctrl_pid.sum_err = 0.98*vel_ctrl[ROLL].ctrl_pid.sum_err + vel_ctrl[ROLL].ctrl_pid.err;

	
    vel_ctrl[ROLL].ctrl_output = ctrl_pid_controller(&vel_ctrl[ROLL].ctrl_pid);
    //vel_ctrl[ROLL].ctrl_output = ctrl_leadlag_controller(vel_ctrl[ROLL].ctrl_output, &vel_ctrl[ROLL].leadlag);
#ifdef VEL_CTRL_USE_NOTCH_FILTER
    vel_ctrl[ROLL].ctrl_output = filter_notch_2nd(vel_ctrl[ROLL].ctrl_output, &vel_ctrl[ROLL].notch_filter[0]);
    vel_ctrl[ROLL].ctrl_output = filter_notch_2nd(vel_ctrl[ROLL].ctrl_output, &vel_ctrl[ROLL].notch_filter[1]);
#endif
    if (vel_ctrl_lpf_flag[ROLL])
    {
        vel_ctrl[ROLL].ctrl_output = filter_low_pass_2nd(vel_ctrl[ROLL].ctrl_output, &vel_ctrl[ROLL].lp_2nd_filter);
    }
	
    vel_ctrl[ROLL].ctrl_output += vel_ctrl[ROLL].ctrl_comp;

    

    if (pos_ctrl[PITCH].ctrl_enable)  vel_ctrl[PITCH].vel_target = 0.0f;
    vel_ctrl[PITCH].vel_ref = vel_ctrl[PITCH].vel_target + pos_ctrl[PITCH].ctrl_output;
    vel_ctrl[PITCH].ctrl_pid.err = vel_ctrl[PITCH].vel_ref - vel_ctrl[PITCH].vel_mea; 
    vel_ctrl[PITCH].ctrl_pid.sum_err = 0.98*vel_ctrl[PITCH].ctrl_pid.sum_err + vel_ctrl[PITCH].ctrl_pid.err;

    vel_ctrl[PITCH].ctrl_output = ctrl_pid_controller(&vel_ctrl[PITCH].ctrl_pid);
    //vel_ctrl[PITCH].ctrl_output = ctrl_leadlag_controller(vel_ctrl[PITCH].ctrl_output, &vel_ctrl[PITCH].leadlag);
#ifdef VEL_CTRL_USE_NOTCH_FILTER
    vel_ctrl[PITCH].ctrl_output = filter_notch_2nd(vel_ctrl[PITCH].ctrl_output, &vel_ctrl[PITCH].notch_filter[0]);
    vel_ctrl[PITCH].ctrl_output = filter_notch_2nd(vel_ctrl[PITCH].ctrl_output, &vel_ctrl[PITCH].notch_filter[1]);
#endif
    if (vel_ctrl_lpf_flag[PITCH])
    {
        vel_ctrl[PITCH].ctrl_output = filter_low_pass_2nd(vel_ctrl[PITCH].ctrl_output, &vel_ctrl[PITCH].lp_2nd_filter);
    } 
    
    if (vel_ctrl[PITCH].ctrl_comp_flag)
    {
            vel_ctrl[PITCH].ctrl_output += vel_ctrl[PITCH].ctrl_comp;
    }

    
    if (pos_ctrl[YAW].ctrl_enable)  vel_ctrl[YAW].vel_target = 0.0f;
    vel_ctrl[YAW].vel_ref = vel_ctrl[YAW].vel_target + pos_ctrl[YAW].ctrl_output;
    vel_ctrl[YAW].ctrl_pid.err = vel_ctrl[YAW].vel_ref - vel_ctrl[YAW].vel_mea;
    vel_ctrl[YAW].ctrl_pid.sum_err = 0.98*vel_ctrl[YAW].ctrl_pid.sum_err + vel_ctrl[YAW].ctrl_pid.err;
	
	
    vel_ctrl[YAW].ctrl_output = ctrl_pid_controller(&vel_ctrl[YAW].ctrl_pid);
    //vel_ctrl[YAW].ctrl_output = ctrl_leadlag_controller(vel_ctrl[YAW].ctrl_output, &vel_ctrl[YAW].leadlag);
#ifdef VEL_CTRL_USE_NOTCH_FILTER
    vel_ctrl[YAW].ctrl_output = filter_notch_2nd(vel_ctrl[YAW].ctrl_output, &vel_ctrl[YAW].notch_filter[0]);
    vel_ctrl[YAW].ctrl_output = filter_notch_2nd(vel_ctrl[YAW].ctrl_output, &vel_ctrl[YAW].notch_filter[1]);
#endif
    if (vel_ctrl_lpf_flag[YAW])
    {
        vel_ctrl[YAW].ctrl_output = filter_low_pass_2nd(vel_ctrl[YAW].ctrl_output, &vel_ctrl[YAW].lp_2nd_filter);
    }
	
    if (vel_ctrl[YAW].ctrl_comp_flag)
    {
            vel_ctrl[YAW].ctrl_output += vel_ctrl[YAW].ctrl_comp;
    }
    
    
    if (!vel_ctrl[ROLL].ctrl_enable)
    {
        vel_ctrl[ROLL].ctrl_output = 0;
    }
    
    if (!vel_ctrl[PITCH].ctrl_enable)
    {
        vel_ctrl[PITCH].ctrl_output = 0;
    }
    
    if (!vel_ctrl[YAW].ctrl_enable)
    {
        vel_ctrl[YAW].ctrl_output = 0;
    }

    //  参数和数据的初始化
    if (chrip_resp.run_state == 1)
    {
        // 参数初始化
        if (!chrip_resp.params_init_flag)
        {
            chrip_resp.num =  ((float)chrip_resp.sample_freq / 1000)* chrip_resp.end_time_ms;
            chrip_resp.count = 0;
            chrip_resp.params_init_flag = true;
        }

        // 生成扫频信号
        chrip_resp.u = chirp_signal_generator(chrip_resp.u_amp, chrip_resp.sample_freq, \
                        chrip_resp.start_freq, chrip_resp.end_time_ms, chrip_resp.end_freq, chrip_resp.count);
        if (++chrip_resp.count <= chrip_resp.num)
        {
            vel_ctrl[PITCH].ctrl_output = chrip_resp.u;
        }
        else
        {
            chrip_resp.count = 0;
            chrip_resp.run_state = 2;
            chrip_resp.params_init_flag = false;
        }
    }
  
    remo_esc_set_torque_cmd(vel_ctrl[ROLL].ctrl_output, ROLL);
    remo_esc_set_torque_cmd(vel_ctrl[PITCH].ctrl_output, PITCH);
    remo_esc_set_torque_cmd(vel_ctrl[YAW].ctrl_output, YAW);
	
}

static void remo_ctrl_joint_vel_update(void)
{
    const trig_func_f_t *degree_trig[2];
    float temp_f = 1.0f;

    float joint_vel[3] = {0.0f};


    degree_trig[ROLL] = remo_ahrs_get_euler_trig(ROLL);
    degree_trig[PITCH] = remo_ahrs_get_euler_trig(PITCH);
    if (fabs(degree_trig[ROLL]->cos) <= 0.1f)
    {
        temp_f = (degree_trig[ROLL]->cos > 0) ? 0.1f : -0.1f; // 防止除0
    }
    else
    {
        temp_f = degree_trig[ROLL]->cos;
    }
    jacob_matrix.b2e_matrix[0][0] = degree_trig[PITCH]->cos;
    jacob_matrix.b2e_matrix[0][1] = 0.0f;
    jacob_matrix.b2e_matrix[0][2] = degree_trig[PITCH]->sin;
    jacob_matrix.b2e_matrix[1][0] = degree_trig[PITCH]->sin*degree_trig[ROLL]->sin / temp_f;
    jacob_matrix.b2e_matrix[1][1] = 1.0f;
    jacob_matrix.b2e_matrix[1][2] = -degree_trig[PITCH]->cos*degree_trig[ROLL]->sin / temp_f;
    jacob_matrix.b2e_matrix[2][0] = -degree_trig[PITCH]->sin / temp_f;
    jacob_matrix.b2e_matrix[2][1] = 0.0f;
    jacob_matrix.b2e_matrix[2][2] = degree_trig[PITCH]->cos / temp_f;

    degree_trig[ROLL] = remo_encoder_get_joint_trig(ROLL);
    degree_trig[PITCH] = remo_encoder_get_joint_trig(PITCH);
    if (fabs(degree_trig[ROLL]->cos) <= 0.1f)
    {
        temp_f = (degree_trig[ROLL]->cos > 0) ? 0.1f : -0.1f; // 防止除0
    }
    else
    {
        temp_f = degree_trig[ROLL]->cos;
    }
    jacob_matrix.b2j_matrix[0][0] = degree_trig[PITCH]->cos;
    jacob_matrix.b2j_matrix[0][1] = 0.0f;
    jacob_matrix.b2j_matrix[0][2] = degree_trig[PITCH]->sin;
    jacob_matrix.b2j_matrix[1][0] = degree_trig[PITCH]->sin*degree_trig[ROLL]->sin / temp_f;
    jacob_matrix.b2j_matrix[1][1] = 1.0f;
    jacob_matrix.b2j_matrix[1][2] = -degree_trig[PITCH]->cos*degree_trig[ROLL]->sin / temp_f;
    jacob_matrix.b2j_matrix[2][0] = -degree_trig[PITCH]->sin / temp_f;
    jacob_matrix.b2j_matrix[2][1] = 0.0f;
    jacob_matrix.b2j_matrix[2][2] = degree_trig[PITCH]->cos / temp_f;

    switch(jacob_matrix.state)
    {
        case 0:
            remo_misc_matrix_multiply(3, 3, 1, joint_vel, &jacob_matrix.b2e_matrix[0][0], ctrl_gyro_deg);
            break;
        case 1:
            remo_misc_matrix_multiply(3, 3, 1, joint_vel, &jacob_matrix.b2j_matrix[0][0], ctrl_gyro_deg);
            break;
        case 2:
            joint_vel[0] = ctrl_gyro_deg[0];
            joint_vel[1] = ctrl_gyro_deg[1];
            joint_vel[2] = ctrl_gyro_deg[2];
            break;
        default:
            break;
    }
    

    vel_ctrl[ROLL].vel_mea = joint_vel[ROLL];
    vel_ctrl[PITCH].vel_mea = joint_vel[PITCH];
    vel_ctrl[YAW].vel_mea = joint_vel[YAW];

}

/**************************************************************************************************
 * 函数名称: remo_ctrl_loop_set_pos_ref
 * 输入参数: ref->参考值， index->轴索引
 * 返回结果: void
 * 功能描述: 设置位置环的参考值
**************************************************************************************************/
static void remo_ctrl_loop_renew_params(void)
{
    if (ctrl_renew_params_flag[ROLL])
    {
        ctrl_leadlag_unit_init(1.0f/(float)IMU_ODR__VEL_CTRL__FREQ, vel_ctrl[ROLL].leadlag.lead.wz, vel_ctrl[ROLL].leadlag.lead.wp, &vel_ctrl[ROLL].leadlag.lead);
        ctrl_leadlag_unit_init(1.0f/(float)IMU_ODR__VEL_CTRL__FREQ, vel_ctrl[ROLL].leadlag.lag.wz, vel_ctrl[ROLL].leadlag.lag.wp, &vel_ctrl[ROLL].leadlag.lag);
        
        filter_notchfilter_2nd_init(vel_ctrl[ROLL].notch_filter[0].cutoff_freq, 
            vel_ctrl[ROLL].notch_filter[0].sample_freq, vel_ctrl[ROLL].notch_filter[0].BW, &vel_ctrl[ROLL].notch_filter[0]);
        filter_notchfilter_2nd_init(vel_ctrl[ROLL].notch_filter[1].cutoff_freq, 
            vel_ctrl[ROLL].notch_filter[1].sample_freq, vel_ctrl[ROLL].notch_filter[1].BW, &vel_ctrl[ROLL].notch_filter[1]);
        ctrl_renew_params_flag[ROLL] = false;
    }

    if (ctrl_renew_params_flag[PITCH])
    {
        ctrl_leadlag_unit_init(1.0f/(float)IMU_ODR__VEL_CTRL__FREQ, vel_ctrl[PITCH].leadlag.lead.wz, vel_ctrl[PITCH].leadlag.lead.wp, &vel_ctrl[PITCH].leadlag.lead);
        ctrl_leadlag_unit_init(1.0f/(float)IMU_ODR__VEL_CTRL__FREQ, vel_ctrl[PITCH].leadlag.lag.wz, vel_ctrl[PITCH].leadlag.lag.wp, &vel_ctrl[PITCH].leadlag.lag);
        
        filter_notchfilter_2nd_init(vel_ctrl[PITCH].notch_filter[0].cutoff_freq, 
            vel_ctrl[PITCH].notch_filter[0].sample_freq, vel_ctrl[PITCH].notch_filter[0].BW, &vel_ctrl[PITCH].notch_filter[0]);
        filter_notchfilter_2nd_init(vel_ctrl[PITCH].notch_filter[1].cutoff_freq, 
            vel_ctrl[PITCH].notch_filter[1].sample_freq, vel_ctrl[PITCH].notch_filter[1].BW, &vel_ctrl[PITCH].notch_filter[1]);
        ctrl_renew_params_flag[PITCH] = false;
    }

    if (ctrl_renew_params_flag[YAW])
    {
        ctrl_leadlag_unit_init(1.0f/(float)IMU_ODR__VEL_CTRL__FREQ, vel_ctrl[YAW].leadlag.lead.wz, vel_ctrl[YAW].leadlag.lead.wp, &vel_ctrl[YAW].leadlag.lead);
        ctrl_leadlag_unit_init(1.0f/(float)IMU_ODR__VEL_CTRL__FREQ, vel_ctrl[YAW].leadlag.lag.wz, vel_ctrl[YAW].leadlag.lag.wp, &vel_ctrl[YAW].leadlag.lag);
        
        filter_notchfilter_2nd_init(vel_ctrl[YAW].notch_filter[0].cutoff_freq, 
            vel_ctrl[YAW].notch_filter[0].sample_freq, vel_ctrl[YAW].notch_filter[0].BW, &vel_ctrl[YAW].notch_filter[0]);
        filter_notchfilter_2nd_init(vel_ctrl[YAW].notch_filter[1].cutoff_freq, 
            vel_ctrl[YAW].notch_filter[1].sample_freq, vel_ctrl[YAW].notch_filter[1].BW, &vel_ctrl[YAW].notch_filter[1]);
        ctrl_renew_params_flag[YAW] = false;
    }
}

void remo_ctrl_loop_scale_pos_limit(float scale, uint8_t index)
{
    if (index > 2)
    {
        return;
    }
    else
    {
        switch(index)
        {
            case 0:
                pos_ctrl[ROLL].ctrl_p.p_limit = POS_CTRL_ROLL_P_LIMIT * scale;
                break;
            case 1:
                pos_ctrl[PITCH].ctrl_p.p_limit = POS_CTRL_PITCH_P_LIMIT * scale;
                break;
            case 2:
                pos_ctrl[YAW].ctrl_p.p_limit = POS_CTRL_YAW_P_LIMIT * scale;
                break;
            default:
                break;
        }
    }
}

/**************************************************************************************************
 * 函数名称: remo_ctrl_loop_set_pos_ref
 * 输入参数: ref->参考值， index->轴索引
 * 返回结果: void
 * 功能描述: 设置位置环的参考值
**************************************************************************************************/
void remo_ctrl_loop_set_pos_ref(float ref, uint8_t index)
{
    if (index > 2)
    {
        return;
    }
    else
    {
        pos_ctrl[index].ctrl_enable = true;
        pos_ctrl[index].pos_ref = ref;
    }
}

/**************************************************************************************************
 * 函数名称: remo_ctrl_loop_set_pos_mea
 * 输入参数: mea->测量值， index->轴索引
 * 返回结果: void
 * 功能描述: 设置位置环的测量值
**************************************************************************************************/
void remo_ctrl_loop_set_pos_mea(float mea, uint8_t index)
{
    if (index > 2)
    {
        return;
    }
    else
    {
        pos_ctrl[index].pos_mea = mea;
    }
}


/**************************************************************************************************
 * 函数名称: remo_ctrl_loop_set_vel_target
 * 输入参数: ref->参考值， index->轴索引
 * 返回结果: void
 * 功能描述: 设置速度环的参考值
**************************************************************************************************/
void remo_ctrl_loop_set_vel_target(float target, uint8_t index)
{
    if (index > 2)
    {
        return;
    }
    else
    {
        vel_ctrl[index].vel_target = target;
    }
}

/**************************************************************************************************
 * 函数名称: remo_ctrl_loop_set_vel_mea
 * 输入参数: mea->测量值， index->轴索引
 * 返回结果: void
 * 功能描述: 设置速度环的测量值
**************************************************************************************************/
void remo_ctrl_loop_set_vel_mea(float mea, uint8_t index)
{
    if (index > 2)
    {
        return;
    }
    else
    {
        vel_ctrl[index].vel_mea = mea;
    }
}

/**************************************************************************************************
 * 函数名称: remo_ctrl_loop_reset_roll_pos_pid
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 重置ROLL位置环参数
**************************************************************************************************/
void remo_ctrl_loop_reset_roll_pos_pid(void)
{
    nonlinear_unit_init(POS_CTRL_ROLL_KP, 0.0f, 0.01f, &pos_ctrl[ROLL].ctrl_p.kp_nl_unit);
}

/**************************************************************************************************
 * 函数名称: remo_ctrl_loop_reset_pitch_pos_pid
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 重置PITCH位置环参数
**************************************************************************************************/
void remo_ctrl_loop_reset_pitch_pos_pid(void)
{
    nonlinear_unit_init(POS_CTRL_PITCH_KP, 0.0f, 0.0f, &pos_ctrl[PITCH].ctrl_p.kp_nl_unit);
}

/**************************************************************************************************
 * 函数名称: remo_ctrl_loop_reset_pitch_pos_pid
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 重置PITCH位置环参数
**************************************************************************************************/
void remo_ctrl_loop_set_pitch_pos_work1_pid(void)
{
	    
    nonlinear_unit_init(POS_CTRL_PITCH_KP, 0.0f, 0.0f, &pos_ctrl[PITCH].ctrl_p.kp_nl_unit);
}

/**************************************************************************************************
 * 函数名称: remo_ctrl_loop_reset_yaw_pos_pid
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 重置YAW位置环参数
**************************************************************************************************/
void remo_ctrl_loop_reset_yaw_pos_pid(void)
{
    nonlinear_unit_init(POS_CTRL_YAW_KP, 0.0f, 0.0f, &pos_ctrl[YAW].ctrl_p.kp_nl_unit);
}

/**************************************************************************************************
 * 函数名称: remo_ctrl_loop_reset_roll_vel_pid
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 重置ROLL速度环参数
**************************************************************************************************/
void remo_ctrl_loop_reset_roll_vel_pid(void)
{
    nonlinear_unit_init(VEL_CTRL_ROLL_KP, 0.0f, 0.0f, &vel_ctrl[ROLL].ctrl_pid.kp_nl_unit);    // kp 
    nonlinear_unit_init(VEL_CTRL_ROLL_KI, 0.0f, 0.0f, &vel_ctrl[ROLL].ctrl_pid.ki_nl_unit);    // ki
    nonlinear_unit_init(VEL_CTRL_ROLL_KD, 0.0f, 0.0f, &vel_ctrl[ROLL].ctrl_pid.kd_nl_unit);
    vel_ctrl[ROLL].ctrl_pid.ki_dec = 1.0f;
    vel_ctrl[ROLL].ctrl_pid.ki_out = 0;
    vel_ctrl[ROLL].ctrl_pid.ki_limit = MOTOR_ROLL_TORQUE_LIMIT;
    vel_ctrl[ROLL].ctrl_pid.pid_limit = MOTOR_ROLL_TORQUE_LIMIT;
    vel_ctrl[ROLL].kp_adaptive_flag = true;
}

/**************************************************************************************************
 * 函数名称: remo_ctrl_loop_reset_pitch_vel_pid
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 重置PITCH速度环参数
**************************************************************************************************/
void remo_ctrl_loop_reset_pitch_vel_pid(void)
{
    nonlinear_unit_init(VEL_CTRL_PITCH_KP, 0.0f, 0.0f, &vel_ctrl[PITCH].ctrl_pid.kp_nl_unit);
    nonlinear_unit_init(VEL_CTRL_PITCH_KI, 0.0f, 0.0f, &vel_ctrl[PITCH].ctrl_pid.ki_nl_unit);
    nonlinear_unit_init(VEL_CTRL_PITCH_KD, 0.0f, 0.0f, &vel_ctrl[PITCH].ctrl_pid.kd_nl_unit);
    vel_ctrl[PITCH].ctrl_pid.ki_dec = 1.0f;
    vel_ctrl[PITCH].ctrl_pid.ki_out = 0;
    vel_ctrl[PITCH].ctrl_pid.ki_limit = MOTOR_PITCH_TORQUE_LIMIT;
    vel_ctrl[PITCH].ctrl_pid.pid_limit = MOTOR_PITCH_TORQUE_LIMIT;
    vel_ctrl[PITCH].kp_adaptive_flag = true;
}

void remo_ctrl_loop_set_pitch_vel_limit(uint16_t limit)
{
    vel_ctrl[PITCH].ctrl_pid.ki_limit = limit;
    vel_ctrl[PITCH].ctrl_pid.pid_limit = limit;
}

/**************************************************************************************************
 * 函数名称: remo_ctrl_loop_reset_yaw_vel_pid
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 重置YAW速度环参数
**************************************************************************************************/
void remo_ctrl_loop_reset_yaw_vel_pid(void)
{
    nonlinear_unit_init(VEL_CTRL_YAW_KP, 0.0f, 0.0f, &vel_ctrl[YAW].ctrl_pid.kp_nl_unit); 
    nonlinear_unit_init(VEL_CTRL_YAW_KI, 0.0f, 0.0f, &vel_ctrl[YAW].ctrl_pid.ki_nl_unit);
    nonlinear_unit_init(VEL_CTRL_YAW_KD, 0.0f, 0.0f, &vel_ctrl[YAW].ctrl_pid.kd_nl_unit);
    vel_ctrl[YAW].ctrl_pid.ki_dec = 1.0f;
    vel_ctrl[YAW].ctrl_pid.ki_out = 0;
    vel_ctrl[YAW].ctrl_pid.ki_limit = MOTOR_YAW_TORQUE_LIMIT;
    vel_ctrl[YAW].ctrl_pid.pid_limit = MOTOR_YAW_TORQUE_LIMIT;
    vel_ctrl[YAW].kp_adaptive_flag = true;
}

void remo_ctrl_loop_set_yaw_vel_limit(uint16_t limit)
{
    vel_ctrl[YAW].ctrl_pid.ki_limit = limit;
    vel_ctrl[YAW].ctrl_pid.pid_limit = limit;
}

void remo_ctrl_loop_set_vel_ki_dec(float ki_dec, uint8_t index)
{
    vel_ctrl[index].ctrl_pid.ki_dec = ki_dec;
}

/**************************************************************************************************
 * 函数名称: remo_ctrl_loop_set_pitch_start_vel_pid
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 设置启动时的pitch速度环参数
**************************************************************************************************/
void remo_ctrl_loop_set_pitch_start_vel_pid(void)
{
//    nonlinear_unit_init(1.0f, 0, 0, &vel_ctrl[PITCH].ctrl_pid.kp_nl_unit);
//    nonlinear_unit_init(0.001f, 0, 0, &vel_ctrl[PITCH].ctrl_pid.ki_nl_unit);
}

/**************************************************************************************************
 * 函数名称: remo_ctrl_loop_set_yaw_gimballock_vel_pid
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 设置gimballock速度环参数
**************************************************************************************************/
void remo_ctrl_loop_set_yaw_gimballock_vel_pid(void)
{
    // uint32_t limit_temp = MOTOR_YAW_TORQUE_LIMIT >> 1;
    uint32_t limit_temp = MOTOR_YAW_TORQUE_LIMIT;
	
    filter_notchfilter_2nd_init(30, IMU_ODR__VEL_CTRL__FREQ, 0, &vel_ctrl[YAW].notch_filter[0]);   // 11.5 : 1600
    filter_notchfilter_2nd_init(62, IMU_ODR__VEL_CTRL__FREQ, 0, &vel_ctrl[YAW].notch_filter[1]);

    nonlinear_unit_init(200.0f, 0, 0, &vel_ctrl[YAW].ctrl_pid.kp_nl_unit);
    nonlinear_unit_init(10, -20, 0.1, &vel_ctrl[YAW].ctrl_pid.ki_nl_unit);
    nonlinear_unit_init(40.0f, 0, 0, &vel_ctrl[YAW].ctrl_pid.kd_nl_unit);
    vel_ctrl[YAW].ctrl_pid.ki_out = 0;
    vel_ctrl[YAW].ctrl_pid.ki_limit = MOTOR_YAW_TORQUE_LIMIT;
    vel_ctrl[YAW].ctrl_pid.pid_limit = MOTOR_YAW_TORQUE_LIMIT;
	
    vel_ctrl[YAW].kp_adaptive_flag = false;
}

void remo_ctrl_loop_set_roll_gimballock_vel_pid(void)
{
    // uint32_t limit_temp = MOTOR_YAW_TORQUE_LIMIT >> 1;
    uint32_t limit_temp = MOTOR_ROLL_TORQUE_LIMIT;
	
    filter_notchfilter_2nd_init(50, IMU_ODR__VEL_CTRL__FREQ, 0.0, &vel_ctrl[ROLL].notch_filter[0]);
    filter_notchfilter_2nd_init(120, IMU_ODR__VEL_CTRL__FREQ, 0.0, &vel_ctrl[ROLL].notch_filter[1]);

    nonlinear_unit_init(100.0f, 0, 0, &vel_ctrl[ROLL].ctrl_pid.kp_nl_unit);
    nonlinear_unit_init(1.0f, -0.4f, 0.01, &vel_ctrl[ROLL].ctrl_pid.ki_nl_unit);
    nonlinear_unit_init(5.0f, 0, 0, &vel_ctrl[ROLL].ctrl_pid.kd_nl_unit);
    vel_ctrl[ROLL].ctrl_pid.ki_out = 0;
    vel_ctrl[ROLL].ctrl_pid.ki_limit = MOTOR_ROLL_TORQUE_LIMIT;
    vel_ctrl[ROLL].ctrl_pid.pid_limit = MOTOR_ROLL_TORQUE_LIMIT;
	
    vel_ctrl[ROLL].kp_adaptive_flag = false;
}

void remo_ctrl_loop_reset_roll_notch_filter(void)
{
//    filter_notchfilter_2nd_init(36, IMU_ODR__VEL_CTRL__FREQ, 0.10, &vel_ctrl[ROLL].notch_filter[0]);
    filter_notchfilter_2nd_init(50, IMU_ODR__VEL_CTRL__FREQ, 0.12, &vel_ctrl[ROLL].notch_filter[0]);
    filter_notchfilter_2nd_init(120, IMU_ODR__VEL_CTRL__FREQ, 0.1, &vel_ctrl[ROLL].notch_filter[1]);
}

void remo_ctrl_loop_reset_pitch_notch_filter(void)
{
    filter_notchfilter_2nd_init(34, IMU_ODR__VEL_CTRL__FREQ, 0, &vel_ctrl[PITCH].notch_filter[0]);
    filter_notchfilter_2nd_init(147, IMU_ODR__VEL_CTRL__FREQ, 0.00, &vel_ctrl[PITCH].notch_filter[1]);
}
	
void remo_ctrl_loop_reset_yaw_notch_filter(void)
{
    filter_notchfilter_2nd_init(30, IMU_ODR__VEL_CTRL__FREQ, 0, &vel_ctrl[YAW].notch_filter[0]);   // 11.5 : 1600
    filter_notchfilter_2nd_init(62, IMU_ODR__VEL_CTRL__FREQ, 0, &vel_ctrl[YAW].notch_filter[1]);
}

void remo_ctrl_loop_set_torque_ctrl_flag(bool flag, uint8_t index)
{
    if (index >2)
    {
        return;
    }
    else 
    {
        torque_ctrl_flag[index] = flag;
    }
}

void remo_ctrl_loop_set_vel_kp_adaptive_flag(bool flag, uint8_t index)
{
    vel_ctrl[index].kp_adaptive_flag = flag;
}


void remo_ctrl_loop_set_pos_ctrl_enable_state(uint8_t state, uint8_t index)
{
    if (index >2)
    {
        return;
    }
    else 
    {
        pos_ctrl[index].ctrl_enable = state;
    }
}

void remo_ctrl_loop_set_pos_ctrl_type(pos_ctrl_type_t type, uint8_t index)
{
    if (index >2)
    {
        return;
    }
    else 
    {
        pos_ctrl[index].ctrl_enable = true;
        pos_ctrl[index].ctrl_type = type;
    }
}

void remo_ctrl_loop_set_pos_ctrl(pos_ctrl_type_t type, float ref, uint8_t index)
{
    if (index >2)
    {
        return;
    }
    else 
    {
        pos_ctrl[index].ctrl_enable = true;
        pos_ctrl[index].pos_ref = ref;
        pos_ctrl[index].ctrl_type = type;
    }
}

void remo_ctrl_loop_set_angle_linear_calib_flag(bool flag, uint8_t index)
{
    if (index >2)
    {
        return;
    }
    else 
    {
        angle_linear_calib_flag[index] = flag;
    }
}

void remo_ctrl_loop_set_jacob_matrix_state(uint8_t state)
{
    jacob_matrix.state = state;
}

/**************************************************************************************************
 * 函数名称: remo_ctrl_loop_set_pitch_start_vel_pid
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 设置启动时的pitch速度环参数
**************************************************************************************************/
void remo_ctrl_loop_set_pos_kp_scale(float scale, uint8_t index)
{
    uint32_q7_t temp = 0;
    if (index >2)
    {
        return;
    }
    else
    {
        switch(index)
        {
            case ROLL:
                temp = POS_CTRL_ROLL_KP * scale;
                break;
            case PITCH:
                temp = POS_CTRL_PITCH_KP * scale;
                break;
            case YAW:
                temp = POS_CTRL_YAW_KP * scale;
                break;
            case ROLL+10:
            case PITCH+10:
            case YAW+10:
                    temp = scale;
                    index -= 10;
                break;
            default:
                break;
        }
        nonlinear_unit_init(temp, 0, 0, &pos_ctrl[index].ctrl_p.kp_nl_unit);
    }
}

/**************************************************************************************************
 * 函数名称: remo_ctrl_loop_set_pitch_start_vel_pid
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 设置启动时的pitch速度环参数
**************************************************************************************************/
void remo_ctrl_loop_set_vel_kp_scale(float scale, uint8_t index)
{
    float temp = 0;
    if (index >2)
    {
        return;
    }
    else
    {
        switch(index)
        {
            case ROLL:
                temp = VEL_CTRL_ROLL_KP * scale;
                break;
            case PITCH:
                temp = VEL_CTRL_PITCH_KP * scale;
                break;
            case YAW:
                temp = VEL_CTRL_YAW_KP * scale;
                break;
            default:
                break;
        }
        nonlinear_unit_init(temp, 0, 0, &vel_ctrl[index].ctrl_pid.kp_nl_unit);
    }

}


/**************************************************************************************************
 * 函数名称: remo_ctrl_loop_set_vel_ki_scale
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 设置启动时的pitch速度环参数
**************************************************************************************************/
void remo_ctrl_loop_set_vel_ki_scale(float scale, uint8_t index)
{
    float temp = 0;
    if (index >2)
    {
        return;
    }
    else
    {
        switch(index)
        {
            case ROLL:
                temp = VEL_CTRL_ROLL_KI * scale;
                break;
            case PITCH:
                temp = VEL_CTRL_PITCH_KI * scale;
                break;
            case YAW:
                temp = VEL_CTRL_YAW_KI * scale;
                break;
            default:
                break;
        }
        nonlinear_unit_init(temp, 0, 0, &vel_ctrl[index].ctrl_pid.ki_nl_unit);
    }

}

/**************************************************************************************************
 * 函数名称: remo_ctrl_loop_set_vel_kd_scale
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 设置启动时的pitch速度环参数
**************************************************************************************************/
void remo_ctrl_loop_set_vel_kd_scale(float scale, uint8_t index)
{
    float temp = 0;
    if (index >2)
    {
        return;
    }
    else
    {
        switch(index)
        {
            case ROLL:
                temp = VEL_CTRL_ROLL_KD * scale;
                break;
            case PITCH:
                temp = VEL_CTRL_PITCH_KD * scale;
                break;
            case YAW:
                temp = VEL_CTRL_YAW_KD * scale;
                break;
            default:
                break;
        }
        nonlinear_unit_init(temp, 0, 0, &vel_ctrl[index].ctrl_pid.kd_nl_unit);
    }

}

/**************************************************************************************************
 * 函数名称: remo_ctrl_loop_set_vel_kp_scale
 * 输入参数: bool
 * 返回结果: void
 * 功能描述: 设置yaw轴进行低通滤波标志
**************************************************************************************************/
void remo_ctrl_loop_set_vel_ctrl_lpf(bool flag, uint8_t index)
{
    if (index > 2)
    {
        return;
    }
    else
    {
        vel_ctrl_lpf_flag[index] = flag;
    }
}

/**************************************************************************************************
 * 函数名称: remo_ctrl_loop_get_pos_output
 * 输入参数: index->轴索引
 * 返回结果: void
 * 功能描述: 获取速度环的控制输出
**************************************************************************************************/
float remo_ctrl_loop_get_pos_err(uint8_t index)
{
    if (index > 2)
    {
        return 0;
    }
    else
    {
        return pos_ctrl[index].ctrl_p.err;
    }
}

/**************************************************************************************************
 * 函数名称: remo_ctrl_loop_get_pos_output
 * 输入参数: index->轴索引
 * 返回结果: void
 * 功能描述: 获取速度环的控制输出
**************************************************************************************************/
float remo_ctrl_loop_get_pos_output(uint8_t index)
{
    if (index > 2)
    {
        return 0;
    }
    else
    {
        return pos_ctrl[index].ctrl_output;
    }
}

/**************************************************************************************************
 * 函数名称: remo_ctrl_loop_get_vel_output
 * 输入参数: index->轴索引
 * 返回结果: void
 * 功能描述: 获取速度环的控制输出
**************************************************************************************************/
int16_t remo_ctrl_loop_get_vel_output(uint8_t index)
{
    if (index > 2)
    {
        return 0;
    }
    else
    {
        return vel_ctrl[index].ctrl_output;
    }
}

void remo_ctrl_loop_get_vel_pidout_and_limit(int32_t *vel_output, uint8_t index)
{
    vel_output[0] = vel_ctrl[index].ctrl_output;
    vel_output[1] = vel_ctrl[index].ctrl_pid.pid_limit;
}

int16_t remo_ctrl_loop_get_torque(uint8_t index)
{
    if (index > 2)
    {
        return 0;
    }
    else
    {
        return vel_ctrl[index].ctrl_output;
    }
}

float remo_ctrl_loop_get_vel_err(uint8_t index)
{
    if (index > 2)
    {
        return 0;
    }
    else
    {
        return vel_ctrl[index].ctrl_pid.err;
    }
}

float remo_ctrl_loop_get_vel_delta_err(uint8_t index)
{
    if (index > 2)
    {
        return 0;
    }
    else
    {
        return vel_ctrl[index].ctrl_pid.delta_err;
    }
}

float remo_ctrl_loop_get_vel_sum_err(uint8_t index)
{
    if (index > 2)
    {
        return 0;
    }
    else
    {
        return vel_ctrl[index].ctrl_pid.sum_err;
    }
}

float remo_ctrl_loop_get_vel_sum_err_filter(uint8_t index)
{
    if (index > 2)
    {
        return 0;
    }
    else
    {
        return vel_ctrl[index].ctrl_pid.sum_err_filter;
    }
}

float remo_ctrl_loop_get_vel_ctrl_comp(uint8_t index)
{
	if (index > 2)
    {
        return 0;
    }
    else
    {
        return vel_ctrl[index].ctrl_comp;
    }
}

void remo_ctrl_loop_set_zero_torque(bool flag)
{
    torque_zero_output_flag = flag;
}

int16_t remo_ctrl_loop_get_roll_vel_output(void)
{
	return vel_ctrl[ROLL].ctrl_output;
}

int16_t remo_ctrl_loop_get_pitch_vel_output(void)
{
	return vel_ctrl[PITCH].ctrl_output;
}


int16_t remo_ctrl_loop_get_yaw_vel_output(void)
{
	return vel_ctrl[YAW].ctrl_output;
}

uint8_t remo_ctrl_loop_get_pos_kp_params(uint8_t *buf)
{
    buf[0] = (uint16_t)pos_ctrl[ROLL].ctrl_p.kp_nl_unit.amplitude;
    buf[1] = (uint16_t)pos_ctrl[ROLL].ctrl_p.kp_nl_unit.amplitude >> 8;
    buf[2] = (uint16_t)pos_ctrl[ROLL].ctrl_p.kp_nl_unit.slope;
    buf[3] = (uint16_t)pos_ctrl[ROLL].ctrl_p.kp_nl_unit.slope >> 8;
    buf[4] = (uint16_t)pos_ctrl[ROLL].ctrl_p.kp_nl_unit.offset;
    buf[5] = (uint16_t)pos_ctrl[ROLL].ctrl_p.kp_nl_unit.offset >> 8;

    buf[6] = (uint16_t)pos_ctrl[PITCH].ctrl_p.kp_nl_unit.amplitude;
    buf[7] = (uint16_t)pos_ctrl[PITCH].ctrl_p.kp_nl_unit.amplitude >> 8;
    buf[8] = (uint16_t)pos_ctrl[PITCH].ctrl_p.kp_nl_unit.slope;
    buf[9] = (uint16_t)pos_ctrl[PITCH].ctrl_p.kp_nl_unit.slope >> 8;
    buf[10] = (uint16_t)pos_ctrl[PITCH].ctrl_p.kp_nl_unit.offset;
    buf[11] = (uint16_t)pos_ctrl[PITCH].ctrl_p.kp_nl_unit.offset >> 8;

    buf[12] = (uint16_t)pos_ctrl[YAW].ctrl_p.kp_nl_unit.amplitude;
    buf[13] = (uint16_t)pos_ctrl[YAW].ctrl_p.kp_nl_unit.amplitude >> 8;
    buf[14] = (uint16_t)pos_ctrl[YAW].ctrl_p.kp_nl_unit.slope;
    buf[15] = (uint16_t)pos_ctrl[YAW].ctrl_p.kp_nl_unit.slope >> 8;
    buf[16] = (uint16_t)pos_ctrl[YAW].ctrl_p.kp_nl_unit.offset;
    buf[17] = (uint16_t)pos_ctrl[YAW].ctrl_p.kp_nl_unit.offset >> 8;

    return 18;
}

uint8_t remo_ctrl_loop_get_vel_pid_params(uint8_t *buf)
{
    uint8_t i = 0, index = 0;
    for (i = 0; i < 3; i++)
    {
        index = i * 18;
        buf[index + 0] = (uint16_t)vel_ctrl[i].ctrl_pid.kp_nl_unit.amplitude;
        buf[index + 1] = (uint16_t)vel_ctrl[i].ctrl_pid.kp_nl_unit.amplitude >> 8;
        buf[index + 2] = (uint16_t)vel_ctrl[i].ctrl_pid.kp_nl_unit.slope;
        buf[index + 3] = (uint16_t)vel_ctrl[i].ctrl_pid.kp_nl_unit.slope >> 8;
        buf[index + 4] = (uint16_t)vel_ctrl[i].ctrl_pid.kp_nl_unit.offset;
        buf[index + 5] = (uint16_t)vel_ctrl[i].ctrl_pid.kp_nl_unit.offset >> 8;
        buf[index + 6] = (uint16_t)vel_ctrl[i].ctrl_pid.ki_nl_unit.amplitude;
        buf[index + 7] = (uint16_t)vel_ctrl[i].ctrl_pid.ki_nl_unit.amplitude >> 8;
        buf[index + 8] = (uint16_t)vel_ctrl[i].ctrl_pid.ki_nl_unit.slope;
        buf[index + 9] = (uint16_t)vel_ctrl[i].ctrl_pid.ki_nl_unit.slope >> 8;
        buf[index + 10] = (uint16_t)vel_ctrl[i].ctrl_pid.ki_nl_unit.offset;
        buf[index + 11] = (uint16_t)vel_ctrl[i].ctrl_pid.ki_nl_unit.offset >> 8;
        buf[index + 12] = (uint16_t)0;
        buf[index + 13] = (uint16_t)0 >> 8;
        buf[index + 14] = (uint16_t)0;
        buf[index + 15] = (uint16_t)0 >> 8;
        buf[index + 16] = (uint16_t)0;
        buf[index + 17] = (uint16_t)0 >> 8;
    }

    return 54;
}

uint8_t remo_ctrl_loop_get_vel_leadlag_params(uint8_t *buf)
{
    uint8_t i = 0, index = 0;
    for (i = 0; i < 3; i++)
    {
        index = i * 8;
        buf[index + 0] = (uint16_t)vel_ctrl[YAW].leadlag.lead.wp;
        buf[index + 1] = (uint16_t)vel_ctrl[YAW].leadlag.lead.wp >> 8;
        buf[index + 2] = (uint16_t)vel_ctrl[YAW].leadlag.lead.wz;
        buf[index + 3] = (uint16_t)vel_ctrl[YAW].leadlag.lead.wz >> 8;
        buf[index + 4] = (uint16_t)vel_ctrl[YAW].leadlag.lag.wp;
        buf[index + 5] = (uint16_t)vel_ctrl[YAW].leadlag.lag.wp >> 8;
        buf[index + 6] = (uint16_t)vel_ctrl[YAW].leadlag.lag.wz;
        buf[index + 7] = (uint16_t)vel_ctrl[YAW].leadlag.lag.wz >> 8;
    }

    return 24;
}

uint8_t remo_ctrl_loop_get_vel_2nd_lfp_params(uint8_t *buf)
{
    uint8_t i = 0, index = 0;
    for (i = 0; i < 3; i++)
    {
        index = i * 8;
        buf[index + 0] = (uint16_t)vel_ctrl[YAW].lp_2nd_filter.sample_freq;
        buf[index + 1] = (uint16_t)vel_ctrl[YAW].lp_2nd_filter.sample_freq >> 8;
        buf[index + 2] = (uint16_t)vel_ctrl[YAW].lp_2nd_filter.cutoff_freq;
        buf[index + 3] = (uint16_t)vel_ctrl[YAW].lp_2nd_filter.cutoff_freq >> 8;
        buf[index + 4] = (uint16_t)0;
        buf[index + 5] = (uint16_t)0;
        buf[index + 6] = (uint16_t)0;
        buf[index + 7] = (uint16_t)0;
    }

    return 24;
}

uint8_t remo_ctrl_loop_get_vel_notch_filter_params(uint8_t *buf)
{
    uint8_t i = 0, index = 0;
    for (i = 0; i < 3; i++)
    {
        index = i * 10;
        buf[index + 0] = (uint16_t)vel_ctrl[YAW].notch_filter[0].sample_freq;
        buf[index + 1] = (uint16_t)vel_ctrl[YAW].notch_filter[0].sample_freq >> 8;
        buf[index + 2] = (uint16_t)vel_ctrl[YAW].notch_filter[0].cutoff_freq;
        buf[index + 3] = (uint16_t)vel_ctrl[YAW].notch_filter[0].cutoff_freq >> 8;
        buf[index + 4] = (uint16_t)vel_ctrl[YAW].notch_filter[0].BW;
        buf[index + 5] = (uint16_t)vel_ctrl[YAW].notch_filter[0].BW >> 8;
        buf[index + 6] = (uint16_t)0;
        buf[index + 7] = (uint16_t)0;
        buf[index + 8] = (uint16_t)0;
        buf[index + 9] = (uint16_t)0;
    }

    return 30;
}

uint8_t remo_ctrl_loop_get_vel_diff_filter_params(uint8_t *buf)
{
    uint8_t i = 0, index = 0;
    for (i = 0; i < 3; i++)
    {
        index = i * 6;
        buf[index + 0] = (uint16_t)1;
        buf[index + 1] = (uint16_t)2;
        buf[index + 2] = (uint16_t)3;
        buf[index + 3] = (uint16_t)4;
        buf[index + 4] = (uint16_t)5;
        buf[index + 5] = (uint16_t)6;
    }

    return 18;
}

uint8_t remo_ctrl_loop_get_vel_diff_track_params(uint8_t *buf)
{
    uint8_t i = 0, index = 0;
    for (i = 0; i < 3; i++)
    {
        index = i * 6;
        buf[index + 0] = (uint16_t)1;
        buf[index + 1] = (uint16_t)2;
        buf[index + 2] = (uint16_t)3;
        buf[index + 3] = (uint16_t)4;
        buf[index + 4] = (uint16_t)5;
        buf[index + 5] = (uint16_t)6;
    }

    return 18;
}


uint8_t remo_ctrl_loop_get_cur_pid_params(uint8_t *buf)
{
    uint8_t i = 0, index = 0;
    for (i = 0; i < 3; i++)
    {
        index = i * 18;
        buf[index + 0] = (uint16_t)vel_ctrl[i].ctrl_pid.kp_nl_unit.amplitude;
        buf[index + 1] = (uint16_t)vel_ctrl[i].ctrl_pid.kp_nl_unit.amplitude >> 8;
        buf[index + 2] = (uint16_t)vel_ctrl[i].ctrl_pid.kp_nl_unit.slope;
        buf[index + 3] = (uint16_t)vel_ctrl[i].ctrl_pid.kp_nl_unit.slope >> 8;
        buf[index + 4] = (uint16_t)vel_ctrl[i].ctrl_pid.kp_nl_unit.offset;
        buf[index + 5] = (uint16_t)vel_ctrl[i].ctrl_pid.kp_nl_unit.offset >> 8;
        buf[index + 6] = (uint16_t)vel_ctrl[i].ctrl_pid.ki_nl_unit.amplitude;
        buf[index + 7] = (uint16_t)vel_ctrl[i].ctrl_pid.ki_nl_unit.amplitude >> 8;
        buf[index + 8] = (uint16_t)vel_ctrl[i].ctrl_pid.ki_nl_unit.slope;
        buf[index + 9] = (uint16_t)vel_ctrl[i].ctrl_pid.ki_nl_unit.slope >> 8;
        buf[index + 10] = (uint16_t)vel_ctrl[i].ctrl_pid.ki_nl_unit.offset;
        buf[index + 11] = (uint16_t)vel_ctrl[i].ctrl_pid.ki_nl_unit.offset >> 8;
        buf[index + 12] = (uint16_t)0;
        buf[index + 13] = (uint16_t)0 >> 8;
        buf[index + 14] = (uint16_t)0;
        buf[index + 15] = (uint16_t)0 >> 8;
        buf[index + 16] = (uint16_t)0;
        buf[index + 17] = (uint16_t)0 >> 8;
    }

    return 54;
}
