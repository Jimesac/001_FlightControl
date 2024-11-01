#include "esc_ctrl.h"
#include <stdlib.h>
#include <math.h>
#include "bsp_pwm.h"
#include "imu.h"
                                        
uint32_t T_PWM = 0;                   // 一个开关周期对应的计数值
uint32_t T_SQRT3 = 0;
uint32_t T_MAX = 0;

typedef struct
{
    int32_t alpha;
    int32_t beta;
} alpha_beta_t;

typedef struct
{
    int32_t d;
    int32_t q;
} d_q_t;

typedef struct
{
    int32_t torque;
    uint16_t aligment_torque;
	
    bool esc_test_flag;
    uint8_t esc_test_state;
    esc_state_t esc_state;
    fault_type_t fault_type;
    uint8_t esc_align_start_flag;
    uint8_t esc_already_align_flag;
	
    const int16_q7_t *el_angle_ptr;
    int16_q7_t el_angle_offset;

    trig_func_q14_t el_angle_trig;
    
    d_q_t volt_dq_ref;
    d_q_t volt_dq_ref_last;
    alpha_beta_t volt_alpha_beta_ref;
    
    uint16_t *timer_pwm_output_ptr;
}esc_ctrl_t;

#ifdef ESC_SPEED_CTRL
// 速度拟合
#define ENCODER_VEL_LR_FIT_NUM    (7)
#pragma pack(4) // 按字节对齐
typedef struct {
	float *Y;

	float sum_x;
	float sum_y;
	float sum_xy;
	float sum_x2;

	float a;
	float b;

	uint8_t index;
	uint8_t len;
} linear_reg_t;
float vel_lr_data[ESC_CTRL_MOTOR_NUM][ENCODER_VEL_LR_FIT_NUM] = {0};
struct {
    float angle_deg;
    float speed_dps_last;
    float rotor_angle_last;
    int8_t rotor_angle_corr_cnt;
    
    linear_reg_t vel_lr;
    uint8_t vel_lr_num;
    
    float speed_dps;
    float speed_lpf_alpha;
    float speed_dps_filter;
}vel_lr_by_angle[ESC_CTRL_MOTOR_NUM];
#pragma pack() // 取消按字节对齐

static void linear_regression_offset(linear_reg_t *linear_reg, float y, float offset, uint8_t len);
static void angle_hall_speed_fit(uint16_t freq, uint8_t index);
static void angle_hall_reset_speed_fit(uint8_t index);

// 速度控制
#define DIFF_FIT_NUM   (6)

typedef struct
{
    float amplitude;
    float slope;
    float offset;
}nonlinear_unit_t;

// 二阶低通滤波器参数
typedef struct
{
    uint16_t sample_freq; // 采样频率
    uint16_t cutoff_freq; // 截止频率

    float a[2];  // 分母：1+a[0]*z^(-1)+a[1]*z^(-2)
    float b[3];  // 分子：b[0]+b[1]*z^(-1)+b[2]*z^(-2)
    float de[3]; // 延时元素，delay_element
} low_pass_filter_2nd_t;

typedef struct
{
    uint8_t ctrl_flag;
	uint8_t ctrl_state;
        
    float ref;
    float mea;
    
    float kp;
    float ki;
    float ki_dec; // 积分项退饱和系数
    float kd;

    nonlinear_unit_t kp_nl_unit; // Kp非线性参数
    nonlinear_unit_t ki_nl_unit; // Ki非线性参数
    nonlinear_unit_t kd_nl_unit; // Kd非线性参数

    float err;
    float err_last;
    float err_limit;
    
    float err_fit_data[DIFF_FIT_NUM];
    uint8_t err_fit_index;
    float err_fit_sumx;
    float err_fit_delta;
    low_pass_filter_2nd_t err_butterworth_2nd;

    float kp_out;
    float ki_out;
    float kd_out;
    float pid_out;
    low_pass_filter_2nd_t out_butterworth_2nd;

    float ki_limit;
    float pid_limit;
} ctrl_pid_t;

ctrl_pid_t angle_rate_pid[ESC_CTRL_MOTOR_NUM];

static void nonlinear_unit_init(float of, float am, float sl, nonlinear_unit_t *nl_unit);
static float nonlinear_func(nonlinear_unit_t nl_uint, float input_x);
static float filter_low_pass_2nd(float input, low_pass_filter_2nd_t *filter_param);
static void filter_butterworth_lpf_2nd_init(uint16_t cutoff_freq, uint16_t smp_freq,
                                          low_pass_filter_2nd_t *butterworth_2nd);
static void ctrl_pid_controller_init_err_fit(ctrl_pid_t *pid);
static float ctrl_pid_controller(ctrl_pid_t *pid);

#endif
      
// foc
static void remo_mc_math_rev_park(d_q_t d_q_input, trig_func_q14_t vector_components, alpha_beta_t *alpha_beta_output);
static void remo_mc_math_calc_duty_cycles(uint8_t index);


esc_ctrl_t esc_ctrl[ESC_CTRL_MOTOR_NUM] = {
    {
        .el_angle_ptr = PTR_NULL
    },
    {
        .el_angle_ptr = PTR_NULL
    },
    {
        .el_angle_ptr = PTR_NULL
    },
};

uint8_t esc_ctrl_flag = 0;
int16_t mytest_d_ref = 0; 
uint8_t mytest_d_state = 0;

/**************************************************************************************************
 * 函数名称: remo_esc_ctrl_init
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 计算电调相关的角度值，关节角、电角度、关节角三角函数值、电角度三角函数值
**************************************************************************************************/
void remo_esc_ctrl_init(void)
{
    for (uint8_t i = 0; i < ESC_CTRL_MOTOR_NUM; i++)
    {
        esc_ctrl[i].torque = 0;
        esc_ctrl[i].esc_test_flag = false;
        esc_ctrl[i].esc_test_state = ESC_STOP;
        esc_ctrl[i].esc_state = ESC_STOP;
        esc_ctrl[i].fault_type = NO_FAULT;
        esc_ctrl[i].esc_align_start_flag = 0;
        esc_ctrl[i].el_angle_trig.cos = CONST_1_Q14;
        esc_ctrl[i].el_angle_trig.sin = 0;
        esc_ctrl[i].volt_dq_ref.d = 0;
        esc_ctrl[i].volt_dq_ref.q = 0;
        esc_ctrl[i].volt_dq_ref_last.d = 0;
        esc_ctrl[i].volt_dq_ref_last.q = 0;
        
        esc_ctrl[i].el_angle_offset = 0;
        esc_ctrl[i].aligment_torque = VOLT_DQ_LIMIT*1.2;

    #ifdef ESC_SPEED_CTRL    
        angle_hall_reset_speed_fit(i);
        angle_rate_pid[i].ctrl_flag = 0;
        angle_rate_pid[i].err_limit = 160.0f;
        
        nonlinear_unit_init(120.0, -30.0, 0.01, &angle_rate_pid[i].kp_nl_unit);
        nonlinear_unit_init(3, -4, 0.01, &angle_rate_pid[i].ki_nl_unit);
        nonlinear_unit_init(2.0, 0, 1, &angle_rate_pid[i].kd_nl_unit);
        angle_rate_pid[i].ki_dec = 1.0f;
        angle_rate_pid[i].ki_out = 0;
        angle_rate_pid[i].ki_limit = VOLT_DQ_LIMIT;
        angle_rate_pid[i].pid_limit = VOLT_DQ_LIMIT;
        
        filter_butterworth_lpf_2nd_init(200, IMU_ODR__VEL_CTRL__FREQ, &angle_rate_pid[i].err_butterworth_2nd);
        filter_butterworth_lpf_2nd_init(200, IMU_ODR__VEL_CTRL__FREQ, &angle_rate_pid[i].out_butterworth_2nd);
        ctrl_pid_controller_init_err_fit(&angle_rate_pid[i]);
    #endif
    }

    T_PWM = remo_pwm_get_pwm_reload_value()+1;  // 一个开关周期对应的计数值
    T_SQRT3 = ((int32_t)(T_PWM * 14189 / 8192)); // T * sqrt(3) = x * 1.732
    T_MAX = ((int32_t)(T_PWM * 32768));          // T * 32768
}

/**************************************************************************************************
 * 函数名称: remo_esc_ctrl_update
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 电调主控制程序
**************************************************************************************************/
void remo_esc_ctrl_update(void)
{
    static uint16_t alignment_count[ESC_CTRL_MOTOR_NUM] = {0};
    static uint16_t el_angle_sum_count[ESC_CTRL_MOTOR_NUM] = {0};
    static int32_t alignment_el_angle_sum[ESC_CTRL_MOTOR_NUM] = {0};
    static uint16_t wait_delay_count[ESC_CTRL_MOTOR_NUM] = {0};
    static uint8_t rate_ctrl_init_flag[ESC_CTRL_MOTOR_NUM] = {0};
    int16_t rate_pid_output = 0;

    for (uint8_t i = 0; i < ESC_CTRL_MOTOR_NUM; i++)
    {
    #ifdef ESC_SPEED_CTRL    
        if (angle_rate_pid[i].ctrl_flag)
        {
            if (!rate_ctrl_init_flag[i])
            {
                angle_hall_reset_speed_fit(i);
                rate_ctrl_init_flag[i] = 1;
                esc_ctrl[i].esc_state = ESC_RUN;
            }

            angle_hall_speed_fit(IMU_ODR__VEL_CTRL__FREQ, i);
            angle_rate_pid[i].err = angle_rate_pid[i].ref - angle_rate_pid[i].mea;
            rate_pid_output = ctrl_pid_controller(&angle_rate_pid[i]);
            if (angle_rate_pid[i].ctrl_state == 2)
            {
                esc_ctrl[i].torque = rate_pid_output;
                esc_ctrl[i].torque = filter_low_pass_2nd(esc_ctrl[i].torque, &angle_rate_pid[i].out_butterworth_2nd);
            }

        }
        else
        {
            angle_rate_pid[i].ctrl_state = 0;
            angle_rate_pid[i].ki_out = 0;
            angle_rate_pid[i].err_last = 0;
            rate_ctrl_init_flag[i] = 0;
        }
    #endif
            
            
        remo_trig_func(*esc_ctrl[i].el_angle_ptr, &esc_ctrl[i].el_angle_trig);

        if (esc_ctrl[i].esc_test_flag)
        {
            esc_ctrl[i].esc_test_flag = false;
            esc_ctrl[i].esc_state = esc_ctrl[i].esc_test_state;
        }
        switch (esc_ctrl[i].esc_state)
        {
            case ESC_STOP:
                // stop下关闭pwm
                esc_ctrl[i].volt_dq_ref.d = 0;
                esc_ctrl[i].volt_dq_ref.q = 0;
                break;
            case ESC_IDLE:
                // idle下开启pwm
                // idle下一段时间后，若收到电机run的指令，则进入wait状态
                if (wait_delay_count[i] < 20)
                {
                        wait_delay_count[i] ++;
                }
                else if (esc_ctrl[i].fault_type == NO_FAULT)
                {
                        esc_ctrl[i].esc_state = ESC_WAIT;
                        wait_delay_count[i] = 0;
                }
                esc_ctrl[i].volt_dq_ref.d = 0;
                esc_ctrl[i].volt_dq_ref.q = 0;
                break;
            case ESC_WAIT:
                esc_ctrl[i].volt_dq_ref.d = 0;
                esc_ctrl[i].volt_dq_ref.q = 0;
                if (esc_ctrl[i].esc_align_start_flag) esc_ctrl[i].esc_state = ESC_ALIGNMENT;
                else if(esc_ctrl[i].esc_already_align_flag) esc_ctrl[i].esc_state = ESC_RUN;
                break;
            case ESC_ALIGNMENT:
                esc_ctrl[i].el_angle_offset = 0;
                esc_ctrl[i].el_angle_trig.sin = 0;
                esc_ctrl[i].el_angle_trig.cos = CONST_1_Q14;
                alignment_count[i] ++;
                if (alignment_count[i] <= ALIGNMENT_NUM1)
                {
                        esc_ctrl[i].volt_dq_ref.d = 0;
                        esc_ctrl[i].volt_dq_ref.q = esc_ctrl[i].aligment_torque;
                }
                else if (alignment_count[i] <= ALIGNMENT_NUM2)
                {
                        if (esc_ctrl[i].volt_dq_ref.d < esc_ctrl[i].aligment_torque)
                        {
                                esc_ctrl[i].volt_dq_ref.d += 100;
                        }
                        esc_ctrl[i].volt_dq_ref.q = 0;
                }
                else if (alignment_count[i] <= ALIGNMENT_NUM3)
                {
                        esc_ctrl[i].volt_dq_ref.q = 0;
                        el_angle_sum_count[i] ++;
                        if (el_angle_sum_count[i] == 40)
                        {
                                alignment_el_angle_sum[i] += *esc_ctrl[i].el_angle_ptr;
                                el_angle_sum_count[i] = 0;
                        }
                }
                else if (alignment_count[i] > ALIGNMENT_NUM3)
                {
                        alignment_count[i] = 0;
                        esc_ctrl[i].volt_dq_ref.d = 0;
                        esc_ctrl[i].volt_dq_ref.q = 0;
                        esc_ctrl[i].el_angle_offset = alignment_el_angle_sum[i] >> 7;
                        alignment_el_angle_sum[i] = 0;
                        el_angle_sum_count[i] = 0;
                        esc_ctrl[i].esc_state = ESC_STOP;
                        esc_ctrl[i].esc_align_start_flag = 0;
                        esc_ctrl[i].esc_already_align_flag = 1;
                }
                break;
            case ESC_RUN:
                if (esc_ctrl[i].esc_align_start_flag) esc_ctrl[i].esc_state = ESC_ALIGNMENT;
                if (mytest_d_state == 1)
                {
                        esc_ctrl[i].volt_dq_ref.d = mytest_d_ref;
                }
                else 
                {
                        esc_ctrl[i].volt_dq_ref.d = 0;
                }
                esc_ctrl[i].volt_dq_ref.q = esc_ctrl[i].torque;
                
                if (esc_ctrl[i].volt_dq_ref.q - esc_ctrl[i].volt_dq_ref_last.q > DELTA_DQ_TH)
                {
                        esc_ctrl[i].volt_dq_ref.q = esc_ctrl[i].volt_dq_ref_last.q + DELTA_DQ_TH;
                }
                else if (esc_ctrl[i].volt_dq_ref.q - esc_ctrl[i].volt_dq_ref_last.q < -DELTA_DQ_TH)
                {
                        esc_ctrl[i].volt_dq_ref.q = esc_ctrl[i].volt_dq_ref_last.q - DELTA_DQ_TH;
                }
                esc_ctrl[i].volt_dq_ref_last.q = esc_ctrl[i].volt_dq_ref.q;
                
                if (esc_ctrl[i].volt_dq_ref.d - esc_ctrl[i].volt_dq_ref_last.d > DELTA_DQ_TH)
                {
                        esc_ctrl[i].volt_dq_ref.d = esc_ctrl[i].volt_dq_ref_last.d + DELTA_DQ_TH;
                }
                else if (esc_ctrl[i].volt_dq_ref.d - esc_ctrl[i].volt_dq_ref_last.d < -DELTA_DQ_TH)
                {
                        esc_ctrl[i].volt_dq_ref.d = esc_ctrl[i].volt_dq_ref_last.d - DELTA_DQ_TH;
                }
                esc_ctrl[i].volt_dq_ref_last.d = esc_ctrl[i].volt_dq_ref.d;

                break;
            case ESC_TEST:
                break;
            case ESC_ERR:
            default:
                esc_ctrl[i].volt_dq_ref.d = 0;
                esc_ctrl[i].volt_dq_ref.q = 0;
                break;
        }
        
        if (esc_ctrl[i].esc_state != ESC_RUN)
        {
            esc_ctrl[i].volt_dq_ref_last.d = 0;
            esc_ctrl[i].volt_dq_ref_last.q = 0;
        }
        
        if (esc_ctrl[i].volt_dq_ref.d > VOLT_DQ_LIMIT)  esc_ctrl[i].volt_dq_ref.d = VOLT_DQ_LIMIT;
        else if (esc_ctrl[i].volt_dq_ref.d < -VOLT_DQ_LIMIT)  esc_ctrl[i].volt_dq_ref.d = -VOLT_DQ_LIMIT;
        if (esc_ctrl[i].volt_dq_ref.q > VOLT_DQ_LIMIT)  esc_ctrl[i].volt_dq_ref.q = VOLT_DQ_LIMIT;
        else if (esc_ctrl[i].volt_dq_ref.q < -VOLT_DQ_LIMIT)  esc_ctrl[i].volt_dq_ref.q = -VOLT_DQ_LIMIT;
        
        remo_mc_math_rev_park(esc_ctrl[i].volt_dq_ref, esc_ctrl[i].el_angle_trig, &esc_ctrl[i].volt_alpha_beta_ref);
        remo_mc_math_calc_duty_cycles(i);
    
    }

}

/**************************************************************************************************
 * 函数名称: remo_mc_math_rev_park
 * 输入参数:
 * 返回结果:
 * 功能描述: 反park变换
 *             Zalfa = Zd*cos(angle) - Zq*sin(angle)
 *             Zbeta = Zd*sin(angle) + Zq*cos(angle)
**************************************************************************************************/
static void remo_mc_math_rev_park(d_q_t d_q_input, trig_func_q14_t vector_components, alpha_beta_t *alpha_beta_output)
{
    int32_t Z_tmp1, Z_tmp2;

    if (alpha_beta_output == (void *)0)
    {
        return;
    }

    // 三角函数是q14的，dq和αβ是q15的，此处要特别注意
    Z_tmp1 = d_q_input.d * vector_components.cos;
    Z_tmp1 = Z_tmp1 >> 14;

    Z_tmp2 = d_q_input.q * vector_components.sin;
    Z_tmp2 = Z_tmp2 >> 14;

    alpha_beta_output->alpha = Z_tmp1 - Z_tmp2;

    Z_tmp1 = d_q_input.d * vector_components.sin;
    Z_tmp1 = Z_tmp1 >> 14;

    Z_tmp2 = d_q_input.q * vector_components.cos;
    Z_tmp2 = Z_tmp2 >> 14;

    alpha_beta_output->beta = Z_tmp1 + Z_tmp2;
}


/**************************************************************************************************
 * 函数名称: remo_svpwm_calc_duty_cycles
 * 输入参数:
 * 返回结果:
 * 功能描述: 计算SVPWM，扇区划分如下
 *                     \  2 /
 *                   3  \  /  1
 *                  ------------
 *                   4  /  \  6
 *                     /  5 \
**************************************************************************************************/
static void remo_mc_math_calc_duty_cycles(uint8_t index)
{
    int32_t t0 = 0, t1 = 0, t2 = 0, t12 = 0, alpha = 0, beta = 0;
    uint16_t phase_a_cnt = 0, phase_b_cnt = 0, phase_c_cnt = 0;

    alpha = esc_ctrl[index].volt_alpha_beta_ref.alpha * T_SQRT3;
    beta = esc_ctrl[index].volt_alpha_beta_ref.beta * T_PWM;

    if (beta > alpha)
    {
        if (beta < 0) // 扇区4
        {
            t1 = (beta - alpha) >> 1;
            t2 = -beta;
            t12 = t1 + t2;
            if (t12 > T_MAX)
            {
                t1 = t1 * T_MAX / t12;
                t2 = t2 * T_MAX / t12;
            }
            t0 = T_MAX - t1 - t2;

            phase_a_cnt = t0 >> 16; // (t0 >> 1) >> 15
            phase_b_cnt = phase_a_cnt + (t1 >> 15);
            phase_c_cnt = phase_b_cnt + (t2 >> 15);
        }
        else if ((beta + alpha) < 0) // 扇区3
        {
            t1 = beta;
            t2 = (-beta - alpha) >> 1;
            t12 = t1 + t2;
            if (t12 > T_MAX)
            {
                t1 = t1 * T_MAX / t12;
                t2 = t2 * T_MAX / t12;
            }
            t0 = T_MAX - t1 - t2;

            phase_a_cnt = t0 >> 16;
            phase_c_cnt = phase_a_cnt + (t2 >> 15);
            phase_b_cnt = phase_c_cnt + (t1 >> 15);
        }
        else // 扇区2
        {
            t1 = (alpha + beta) >> 1;
            t2 = (beta - alpha) >> 1;
            t12 = t1 + t2;
            if (t12 > T_MAX)
            {
                t1 = t1 * T_MAX / t12;
                t2 = t2 * T_MAX / t12;
            }
            t0 = T_MAX - t1 - t2;

            phase_c_cnt = t0 >> 16;
            phase_a_cnt = phase_c_cnt + (t1 >> 15);
            phase_b_cnt = phase_a_cnt + (t2 >> 15);
        }
    }
    else
    {
        if (beta > 0) // 扇区1
        {
            t1 = (alpha - beta) >> 1;
            t2 = beta;
            t12 = t1 + t2;
            if (t12 > T_MAX)
            {
                t1 = t1 * T_MAX / t12;
                t2 = t2 * T_MAX / t12;
            }
            t0 = T_MAX - t1 - t2;

            phase_c_cnt = t0 >> 16;
            phase_b_cnt = phase_c_cnt + (t2 >> 15);
            phase_a_cnt = phase_b_cnt + (t1 >> 15);
        }
        else if ((alpha + beta) > 0) // 扇区6
        {
            t1 = -beta;
            t2 = (alpha + beta) >> 1;
            t12 = t1 + t2;
            if (t12 > T_MAX)
            {
                t1 = t1 * T_MAX / t12;
                t2 = t2 * T_MAX / t12;
            }
            t0 = T_MAX - t1 - t2;

            phase_b_cnt = t0 >> 16;
            phase_c_cnt = phase_b_cnt + (t1 >> 15);
            phase_a_cnt = phase_c_cnt + (t2 >> 15);
        }
        else // 扇区5
        {
            t1 = (-alpha - beta) >> 1;
            t2 = (alpha - beta) >> 1;
            t12 = t1 + t2;
            if (t12 > T_MAX)
            {
                t1 = t1 * T_MAX / t12;
                t2 = t2 * T_MAX / t12;
            }
            t0 = T_MAX - t1 - t2;

            phase_b_cnt = t0 >> 16;
            phase_a_cnt = phase_b_cnt + (t2 >> 15);
            phase_c_cnt = phase_a_cnt + (t1 >> 15);
        }
    }

    esc_ctrl[index].timer_pwm_output_ptr[0] = phase_a_cnt;
    esc_ctrl[index].timer_pwm_output_ptr[1] = phase_b_cnt;
    esc_ctrl[index].timer_pwm_output_ptr[2] = phase_c_cnt;
}

// ---------------START ESC SPEED CTRL--------------------------
#ifdef ESC_SPEED_CTRL  
void esc_ctrl_set_angle_deg(float angle_deg, uint8_t index)
{
    vel_lr_by_angle[index].angle_deg = angle_deg;
}

void esc_ctrl_set_angle_ctrl_flag(uint8_t flag, uint8_t index)
{
    angle_rate_pid[index].ctrl_flag = flag;
}

void esc_ctrl_set_angle_ctrl_state(uint8_t state, uint8_t index)
{
	angle_rate_pid[index].ctrl_state = state;
}

void esc_ctrl_set_rate_ref(float ref, uint8_t index)
{
	angle_rate_pid[index].ref = ref;
}

float mytest_dps_th = 500.0f;
float mytest_dps_data[100] = {0.0f};
uint8_t mytest_dps_data_cnt = 0;
static void angle_hall_speed_fit(uint16_t freq, uint8_t index)
{
    uint8_t i = 0, temp = 0;
    float rotor_angle_corr = 0.0f;
    float delta_angle = 0.0f;
	float temp_f;
    
    // -180°/180°的连续处理
    delta_angle = vel_lr_by_angle[index].rotor_angle_last - vel_lr_by_angle[index].angle_deg;
    if (delta_angle > 180.0f) 
    {
        vel_lr_by_angle[index].rotor_angle_corr_cnt = 1;
    }
    else if (delta_angle < -180.f) 
    {
        vel_lr_by_angle[index].rotor_angle_corr_cnt = -1;
    }
    rotor_angle_corr = vel_lr_by_angle[index].angle_deg + vel_lr_by_angle[index].rotor_angle_corr_cnt * 360.0f;
    vel_lr_by_angle[index].rotor_angle_last = vel_lr_by_angle[index].angle_deg;

    // 判断是否数据全部越过-180°/180°的分界线
    temp = 0;
    for (i = 0; i < vel_lr_by_angle[index].vel_lr_num; i++)
    {
        if (fabs(vel_lr_by_angle[index].vel_lr.Y[i]) > 180.0f) temp++;
        else break;
    }

    if (temp >= vel_lr_by_angle[index].vel_lr_num)
    {
        linear_regression_offset(&vel_lr_by_angle[index].vel_lr, rotor_angle_corr, vel_lr_by_angle[index].rotor_angle_corr_cnt * 360, \
		vel_lr_by_angle[index].vel_lr_num);
        vel_lr_by_angle[index].rotor_angle_corr_cnt = 0;
    }
    else 
    {
        linear_regression_offset(&vel_lr_by_angle[index].vel_lr, rotor_angle_corr, 0, vel_lr_by_angle[index].vel_lr_num);
    }

    vel_lr_by_angle[index].speed_dps = vel_lr_by_angle[index].vel_lr.a * freq;  // 2000Hz更新频率
    
    if (++mytest_dps_data_cnt >= 100) mytest_dps_data_cnt = 0;
    mytest_dps_data[mytest_dps_data_cnt] = vel_lr_by_angle[index].speed_dps;
    if (fabs(vel_lr_by_angle[index].speed_dps -vel_lr_by_angle[index].speed_dps_last) > mytest_dps_th)
    {
        vel_lr_by_angle[index].speed_dps = vel_lr_by_angle[index].speed_dps;
    }
    vel_lr_by_angle[index].speed_dps_last = vel_lr_by_angle[index].speed_dps;

    // 低通滤波. 二阶滤波没有一阶滤波效果好 
    vel_lr_by_angle[index].speed_dps += vel_lr_by_angle[index].speed_lpf_alpha * (vel_lr_by_angle[index].speed_dps_last - vel_lr_by_angle[index].speed_dps);  
    
	// 直接采用imu数据
	angle_rate_pid[index].mea = vel_lr_by_angle[index].speed_dps;
}


/**************************************************************************************************
 * 函数名称: linear_regression
 * 输入参数: x：输入
 * 返回结果: float
 * 功能描述: 线性回归函数
**************************************************************************************************/
static void linear_regression_offset(linear_reg_t *linear_reg, float y, float offset, uint8_t len)
{
    // 随着迭代的进行，当x值不断增加时，拟合数据逐渐远离原点，截断值会随着斜率变化的幅度会增大
    float temp_den = 1.0f, temp_num_a = 0, temp_num_b = 0;
    uint8_t i = 0;
    uint16_t index = linear_reg->index;  // index:最后一个数

    linear_reg->Y[index] = y;
    linear_reg->Y[index] -= offset;
    linear_reg->sum_y = linear_reg->Y[index];
    linear_reg->sum_xy = (len-1)*linear_reg->Y[index];
    for (i = 0; i < len-1; i++)
    {
        if (++index >= len ) index = 0;  // 从第一个数开始
		if (index >= 7)
		{
			index = 0;
		}
		linear_reg->Y[index] -= offset;
        linear_reg->sum_y += linear_reg->Y[index];
		linear_reg->sum_xy += linear_reg->Y[index] * i;
    }

    if (++linear_reg->index >= len) linear_reg->index = 0;

    if (linear_reg->len < len - 1)
    {
            linear_reg->len ++;
            return;
    }

    temp_den = linear_reg->sum_x2 * len - linear_reg->sum_x * linear_reg->sum_x;
    temp_num_a = linear_reg->sum_xy * len - linear_reg->sum_x * linear_reg->sum_y;
    temp_num_b = linear_reg->sum_x2 * linear_reg->sum_y - linear_reg->sum_xy * linear_reg->sum_x;

    linear_reg->a = temp_num_a / temp_den;
    linear_reg->b = temp_num_b / temp_den;
}


static void angle_hall_reset_speed_fit(uint8_t index)
{
    vel_lr_by_angle[index].vel_lr.Y = &vel_lr_data[index][0];
    vel_lr_by_angle[index].speed_lpf_alpha = 0.85f;
    vel_lr_by_angle[index].vel_lr_num = ENCODER_VEL_LR_FIT_NUM;
    
    vel_lr_by_angle[index].speed_dps_last = 0.0f;
    vel_lr_by_angle[index].rotor_angle_last = 0.0f;
    vel_lr_by_angle[index].rotor_angle_corr_cnt = 0;
    vel_lr_by_angle[index].vel_lr.sum_x = vel_lr_by_angle[index].vel_lr_num * (vel_lr_by_angle[index].vel_lr_num-1) * 0.5f;
    vel_lr_by_angle[index].vel_lr.sum_y = 0.0f;
    vel_lr_by_angle[index].vel_lr.sum_xy = 0.0f;
    vel_lr_by_angle[index].vel_lr.sum_x2 = vel_lr_by_angle[index].vel_lr_num*(vel_lr_by_angle[index].vel_lr_num-1)*\
                                                (2*vel_lr_by_angle[index].vel_lr_num-1)/6;
    vel_lr_by_angle[index].vel_lr.a = 0.0f;
    vel_lr_by_angle[index].vel_lr.b = 0.0f;
    vel_lr_by_angle[index].vel_lr.index = 0;
    vel_lr_by_angle[index].vel_lr.len = 0;
    vel_lr_by_angle[index].rotor_angle_last = 0.0f;
}

static void filter_butterworth_lpf_2nd_init(uint16_t cutoff_freq, uint16_t smp_freq,
                                          low_pass_filter_2nd_t *butterworth_2nd)
{
    float fr = 0.0f, ohm = 0.0f, c = 0.0f;

    butterworth_2nd->cutoff_freq = cutoff_freq;
    butterworth_2nd->sample_freq = smp_freq;

    if(cutoff_freq == 0)
    {
        butterworth_2nd->b[0] = 1.0f;
        butterworth_2nd->b[1] = 0.0f;
        butterworth_2nd->b[2] = 0.0f;
        butterworth_2nd->a[0] = 0.0f;
        butterworth_2nd->a[1] = 0.0f;
    }

    fr = (float)((float)cutoff_freq / (float)smp_freq);
    ohm = tanf(3.14159265f * fr);
    c = 1.0f + 2.0f * cosf(3.14159265f / 4.0f) * ohm + ohm * ohm;

    butterworth_2nd->b[0] = ohm * ohm / c;
    butterworth_2nd->b[1] = 2.0f * butterworth_2nd->b[0];
    butterworth_2nd->b[2] = butterworth_2nd->b[0];
    butterworth_2nd->a[0] = 2.0f * (ohm * ohm - 1.0f) / c;
    butterworth_2nd->a[1] = (1.0f - 2.0f * cosf(3.14159265f / 4.0f) * ohm + ohm * ohm) / c;

    // 状态量清0
    butterworth_2nd->de[0] = 0.0f;
    butterworth_2nd->de[1] = 0.0f;
    butterworth_2nd->de[2] = 0.0f;
}


float filter_low_pass_2nd(float input, low_pass_filter_2nd_t *filter_param)
{
    float h;

    filter_param->de[0] = input - filter_param->de[1] * filter_param->a[0] -
                          filter_param->de[2] * filter_param->a[1];
    h = filter_param->de[0] * filter_param->b[0] + filter_param->de[1] * filter_param->b[1] +
        filter_param->de[2] * filter_param->b[2];

    filter_param->de[2] = filter_param->de[1];
    filter_param->de[1] = filter_param->de[0];

    return h;
}

static void nonlinear_unit_init(float of, float am, float sl, nonlinear_unit_t *nl_unit)
{
    nl_unit->offset = (of >= 0) ? of : nl_unit->offset;
    nl_unit->amplitude = ((of - am) <= 0) ? 0 : am;
    nl_unit->slope = (sl >= 0) ? sl : nl_unit->slope;
}

static float nonlinear_func(nonlinear_unit_t nl_uint, float input_x)
{
    return nl_uint.offset - nl_uint.amplitude / (1.0f + nl_uint.slope * input_x * input_x); // offset - amplitude / (1.0 + slope * x * x)
}


/**************************************************************************************************
 * 函数名称: ctrl_pid_controller
 * 输入参数: pid_para->pid控制器参数
 * 返回结果: pid控制器的输出结果
 * 功能描述: pid控制器
**************************************************************************************************/
static float ctrl_pid_controller(ctrl_pid_t *pid)
{
    float error_diff, error_filter;
    float temp1, temp2;
    float sum_err1, sum_err2;

    temp1 = pid->err * pid->err;

    // 计算非线性参数 kp = offset - amplitude / (1.0 + slope * error * error)
    // 误差越大，kp越大，最大为offset；误差越小，kp越小，最小为offset-amplitude
    temp2 = 1.0f + pid->kp_nl_unit.slope * temp1;
    temp2 = pid->kp_nl_unit.amplitude / temp2;
    pid->kp = pid->kp_nl_unit.offset - temp2;

    // 计算ki值，同kp
    temp2 = 1.0f + pid->ki_nl_unit.slope * temp1;
    temp2 = pid->ki_nl_unit.amplitude / temp2;
    pid->ki = pid->ki_nl_unit.offset - temp2;

    // 计算kd值，同kp
    temp2 = 1.0f + pid->kd_nl_unit.slope * temp1;
    temp2 = pid->kd_nl_unit.amplitude / temp2;
    pid->kd = pid->kd_nl_unit.offset - temp2;

    // 比例项
    pid->kp_out = pid->kp * pid->err;

#if (PID_DISCRETE_METHOD == BACKWARD_EULER_METHOD)
    // 积分项
    pid->ki_out = pid->ki_out + pid->ki * pid->err;
    // 微分项，待补充
    pid->kd_out = 0.0f;
#elif (PID_DISCRETE_METHOD == BILINEAR_METHOD)
    // 积分项
     pid->ki_out = pid->ki_out +
                  0.5f * pid->ki * (pid->err + pid->err_last);
    // 微分项，待补充
    pid->kd_out = 0.0f;
#else
#error "Wrong PID discrete method!"
#endif

    // 积分退饱和
    if (((pid->ki_out > 0.0f) && (pid->err < 0.0f)) ||
        ((pid->ki_out < 0.0f) && (pid->err > 0.0f)))
    {
        pid->ki_out *= pid->ki_dec;
    }

    // 积分项限幅
    if (pid->ki_out < -pid->ki_limit) pid->ki_out = -pid->ki_limit;
    else if (pid->ki_out > pid->ki_limit) pid->ki_out = pid->ki_limit;

    if (pid->kd == 0)
    {
        pid->kd_out = 0.0f;
    }
    else
    {
        // 误差微分，滤波处理
        // error_filter = filter_low_pass_2nd(pid->error, &(pid->butterworth_2nd));
        // error_diff = error_filter - pid->error_last;
//        error_diff = pid->err - pid->err_last;
        sum_err1 = 0;
        sum_err2 = 0;
        pid->err_fit_data[pid->err_fit_index] = pid->err;
        for (uint8_t i = 0, index = pid->err_fit_index; i < DIFF_FIT_NUM; i++)
        {
            if (++index >= DIFF_FIT_NUM) index = 0;
            sum_err1 += pid->err_fit_data[index];
            sum_err2 += pid->err_fit_data[index] * i;
            
        }
        if (++pid->err_fit_index >= DIFF_FIT_NUM) pid->err_fit_index = 0;
        temp1 = DIFF_FIT_NUM * sum_err2 - pid->err_fit_sumx * sum_err1;
        error_diff = temp1 / pid->err_fit_delta;
        
        error_diff = filter_low_pass_2nd(error_diff, &(pid->err_butterworth_2nd));
        // 微分项
        pid->kd_out = pid->kd * error_diff;
    }

    // PID输出
    pid->pid_out = pid->kp_out + pid->ki_out + pid->kd_out;

    // PID输出限幅
    if (pid->pid_out < -pid->pid_limit) pid->pid_out = -pid->pid_limit;
    else if (pid->pid_out > pid->pid_limit) pid->pid_out = pid->pid_limit;


    // 状态更新
    pid->err_last = pid->err;

    return pid->pid_out;
}

static void ctrl_pid_controller_init_err_fit(ctrl_pid_t *pid)
{
    float temp = 0.0f;
    pid->err_fit_sumx = 0.0f;
    for (uint8_t i = 0; i < DIFF_FIT_NUM; i++)
    {
        pid->err_fit_sumx += i;
        temp += i*i;
        pid->err_fit_data[i] = 0.0f;
    }
    pid->err_fit_delta =  DIFF_FIT_NUM*temp - (pid->err_fit_sumx*pid->err_fit_sumx);
    pid->err_fit_index = 0;
}
#endif // ---------------END ESC SPEED CTRL--------------------------

/**************************************************************************************************
 * 函数名称: remo_motor_send_torque_cmd
 * 输入参数: 
 * 返回结果: void
 * 功能描述: 电机控制，发送转矩指令
**************************************************************************************************/
void remo_esc_set_torque_cmd(int16_t torque, uint8_t index)
{
#ifdef ESC_SPEED_CTRL
	if (angle_rate_pid[index].ctrl_flag)
	{
		return;
	}
#endif
	esc_ctrl[index].torque = torque;
}

void remo_esc_set_el_angle_ptr(const int16_t *ptr, uint8_t index)
{
    if (index > 2)  return;
    esc_ctrl[index].el_angle_ptr = ptr;
}

void remo_esc_set_el_angle_offset(int16_t offset, uint8_t index)
{
    if (index > 2)  return;
    esc_ctrl[index].el_angle_offset = offset;
}

void remo_esc_set_esc_status(esc_state_t state, uint8_t index)
{
    if (index > 2)  return;
    if (state == ESC_ALIGNMENT || state == ESC_RUN)
    {
        if (state == ESC_ALIGNMENT)  
        {
            if (esc_ctrl[index].esc_state == ESC_STOP || esc_ctrl[index].esc_state == ESC_RUN)
            {
                esc_ctrl[index].esc_align_start_flag = 1;
                esc_ctrl[index].esc_already_align_flag = 0;
                esc_ctrl[index].esc_state = ESC_IDLE;
            }
        }
        else 
        {
            esc_ctrl[index].esc_already_align_flag = 1;
            esc_ctrl[index].esc_state = ESC_IDLE;
        }
    }
    else esc_ctrl[index].esc_state = state;
	
}

void remo_esc_set_motors_stop(void)
{
    remo_esc_set_esc_status(ESC_STOP, ROLL);
    remo_esc_set_esc_status(ESC_STOP, PITCH);
    remo_esc_set_esc_status(ESC_STOP, YAW);
}

void remo_esc_set_motors_run(void)
{
    remo_esc_set_esc_status(ESC_RUN, ROLL);
    remo_esc_set_esc_status(ESC_RUN, PITCH);
    remo_esc_set_esc_status(ESC_RUN, YAW);
}

void remo_esc_set_esc_align_already_flag(uint8_t flag, uint8_t index)
{
    if (index > 2)  return;
    esc_ctrl[index].esc_already_align_flag = flag;
}

void remo_esc_set_pwm_output_ptr(uint16_t *ptr, uint8_t index)
{
    if (index > 2)  return;
    esc_ctrl[index].timer_pwm_output_ptr = ptr;
}

int16_t *remo_esc_get_el_angle_offset_ptr(uint8_t index)
{
    return &esc_ctrl[index].el_angle_offset;
}

int16_t remo_esc_get_el_angle_offset(uint8_t index)
{
    return esc_ctrl[index].el_angle_offset;
}

esc_state_t remo_esc_get_esc_status(uint8_t index)
{
    return esc_ctrl[index].esc_state;
}

fault_type_t remo_esc_get_fault_type(uint8_t index)
{
    return esc_ctrl[index].fault_type;
}

uint8_t remo_esc_get_align_start_flag(uint8_t index)
{
    return esc_ctrl[index].esc_align_start_flag;
}

uint8_t remo_esc_get_align_already_flag(uint8_t index)
{
    return esc_ctrl[index].esc_already_align_flag;
}

motor_state_t remo_esc_get_motors_state(void)
{
    if (esc_ctrl[PITCH].fault_type != NO_FAULT || esc_ctrl[YAW].fault_type != NO_FAULT)
    {
            return MOTOR_ERR;
    }
    else if (esc_ctrl[PITCH].esc_state == ESC_RUN && esc_ctrl[YAW].esc_state == ESC_RUN)
    {
            return MOTOR_RUN;
    }
    else if (esc_ctrl[PITCH].esc_state == ESC_STOP && esc_ctrl[YAW].esc_state == ESC_STOP)
    {
            return MOTOR_STOP;
    }
    else 
    {
            return MOTOR_OTHERS;
    }
    return  0;
}
