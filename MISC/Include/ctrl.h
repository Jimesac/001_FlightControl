#ifndef TEST_CTRL_H
#define TEST_CTRL_H

#include "filter.h"
#include "misc.h"

#define CTRL_ERR_DIFF_FIT_NUM   (5)

// PID参数类型
typedef enum
{
    CTRL_PID_KP = 0,
    CTRL_PID_KI,
    CTRL_PID_KD,
    CTRL_PID_KIDEC,
    CTRL_PID_I_LIMIT,
    CTRL_PID_PID_LIMIT,
} ctrl_pid_param_t;

typedef struct
{
    // 超前滞后补偿器的参数
    float sample_time;
    uint16_t wz; // 零点
    uint16_t wp; // 极点

    // 超前滞后补偿器离散化后的系数
    float a[1]; // 分母项系数
    float b[2]; // 分子项系数

    float de[1];
} ctrl_leadlag_unit_t;

// 滞后超前控制器
typedef struct
{
    ctrl_leadlag_unit_t lead;
    ctrl_leadlag_unit_t lag;

    float upper_limit;
    float lower_limit;
} ctrl_leadlag_t;

// PID控制器
// NOTE 对于PID控制器：Kp+Ki/s；它离散后对应的参数为：kp=Kp；ki=Ki*Ts；
// Ts是控制器的周期。因此，注意参数的取值.
typedef struct
{
    float ref;
    float mea;
    
    float kp;
    nonlinear_unit_t kp_nl_unit; // Kp非线性参数

    float err;
    float err_last;
	float delta_err;
    float err_limit;

    low_pass_filter_2nd_t butterworth_2nd;

    float kp_out;
    float p_out;

    float p_limit;
} ctrl_p_t;

typedef struct
{
    float ref;
    float mea;
    
    float kp;
    float ki;
    float ki_dec; // 积分项退饱和系数

    nonlinear_unit_t kp_nl_unit; // Kp非线性参数
    nonlinear_unit_t ki_nl_unit; // Ki非线性参数

    float err;
    float err_last;
	float delta_err;
    float err_limit;

    low_pass_filter_2nd_t butterworth_2nd;

    float kp_out;
    float ki_out;
    float pi_out;

    float ki_limit;

    float pi_limit;
} ctrl_pi_t;

typedef struct
{
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
	float delta_err;
	float sum_err;
	float sum_err_filter;
	float sum_sqr_err;
	float sum_qr_err_filter;
    float err_limit;
    
    float err_fit_data[CTRL_ERR_DIFF_FIT_NUM];
    uint8_t err_fit_index;
    float err_fit_sumx;
    float err_fit_delta;
    low_pass_filter_2nd_t butterworth_2nd;

    float kp_out;
    float ki_out;
    float kd_out;
    float pid_out;

    float ki_limit;

    float pid_limit;
} ctrl_pid_t;

// 超前滞后系数初始化
void ctrl_leadlag_unit_init(float sample_time, uint16_t wz, uint16_t wp,
                                 ctrl_leadlag_unit_t *leadlag_unit);
// 设置PID控制器输出范围限制
void ctrl_leadlag_set_out_limit(float lower_limit, float upper_limit,
                                     ctrl_leadlag_t *leadlag);
// 超前滞后控制器函数
float ctrl_leadlag_controller(float input, ctrl_leadlag_t *leadlag);

// 复位pid控制器的参数（将参数全部清0）
void ctrl_pid_reset_param(ctrl_pid_t *pid);
// 设置PID控制器中的Kp、Ki和Kd的非线性参数
void ctrl_pid_set_nonlinear_unit(float of, float am, float sl, nonlinear_unit_t *nl_unit);
// 设置I控制器退耦系数
void ctrl_pid_set_ki_dec(float ki_dec, ctrl_pid_t *pid);
// P控制器函数
float ctrl_p_controller(ctrl_p_t *p);
// PID控制器函数
float ctrl_pid_controller(ctrl_pid_t *pid);

void ctrl_pid_controller_init_err_fit(ctrl_pid_t *pid);

#endif // TEST_CTRL_H
