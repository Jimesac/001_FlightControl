#include "ctrl.h"
#include "misc.h"
#include "filter.h"

// PID控制器的离散化方法
#define BACKWARD_EULER_METHOD 1
#define BILINEAR_METHOD 2

// 此处采用的PID离散化方法
#define PID_DISCRETE_METHOD BILINEAR_METHOD

static float ctrl_leadlag_unit(ctrl_leadlag_unit_t *leadlag_unit, float input);

/**************************************************************************************************
 * 函数名称: ctrl_leadlag_unit
 * 输入参数: 控制器参数
 * 返回结果: 控制器的输出结果
 * 功能描述: 滞后超前补偿
**************************************************************************************************/
static float ctrl_leadlag_unit(ctrl_leadlag_unit_t *leadlag_unit, float input)
{
    float output;

    output = leadlag_unit->b[0] * input + leadlag_unit->de[0];
    leadlag_unit->de[0] = leadlag_unit->b[1] * input - leadlag_unit->a[0] * output;

    return output;
}

/**************************************************************************************************
 * 函数名称: ctrl_leadlag_unit_init
 * 输入参数: sample_time->采样时间；wz-s域零点；wp->s域极点；a->分母项系数；b->分子项系数
 *            wz < wp为超前补偿，wz>wp为滞后补偿
 * 返回结果: void
 * 功能描述: 超前滞后补偿器的系数初始化，超前滞后补偿器s域传递函数为：(s/wz+1)/(s/wp+1)
 *          采用双线性法离散化，离散后的形式为：
 *          wp/wz*((wz*T+2)/(wp*T+2)+(wz*T-2)/(wp*T+2)*z^(-1))/(1+(wp*T-2)/(wp*T+2)*z^(-1))
**************************************************************************************************/
void ctrl_leadlag_unit_init(float sample_time, uint16_t wz, uint16_t wp,
                                 ctrl_leadlag_unit_t *leadlag_unit)
{
    float pole, zero, k;

    if (sample_time < 0.0f)
    {
        return;
    }

    // 变量赋值
    leadlag_unit->sample_time = sample_time;
    leadlag_unit->wz = wz;
    leadlag_unit->wp = wp;

    pole = sample_time * (float)wp;
    zero = sample_time * (float)wz;
    k = (float)wp / (float)wz;
		

    leadlag_unit->a[0] = (pole - 2.0f) / (pole + 2.0f);
    leadlag_unit->b[0] = k * (zero + 2.0f) / (pole + 2.0f);
    leadlag_unit->b[1] = k * (zero - 2.0f) / (pole + 2.0f); 

    // 状态量清0
    leadlag_unit->de[0] = 0.0f;
}

/**************************************************************************************************
 * 函数名称: ctrl_leadlag_set_out_limit
 * 输入参数: 
 * 返回结果: void
 * 功能描述: 
**************************************************************************************************/
void ctrl_leadlag_set_out_limit(float lower_limit, float upper_limit,
                                     ctrl_leadlag_t *leadlag)
{
    if (lower_limit >= upper_limit)
    {
        return;
    }

    leadlag->lower_limit = lower_limit;
    leadlag->upper_limit = upper_limit;
}

/**************************************************************************************************
 * 函数名称: ctrl_leadlag_controller
 * 输入参数: 控制器参数
 * 返回结果: 控制器的输出结果
 * 功能描述: 超前滞后控制器函数
**************************************************************************************************/
float ctrl_leadlag_controller(float input, ctrl_leadlag_t *leadlag)
{
    float output;

    output = input;

    // 超前滞后补偿
    if (leadlag->lead.wz != leadlag->lead.wp)
    {
        output = ctrl_leadlag_unit(&leadlag->lead, input);
    }

    if (leadlag->lag.wz != leadlag->lag.wp)
    {
        output = ctrl_leadlag_unit(&leadlag->lag, output);
    }

    // 输出限幅
    output = remo_misc_constrainf(output, leadlag->lower_limit, leadlag->upper_limit);

    return output;
}

/**************************************************************************************************
 * 函数名称: ctrl_pid_reset_param
 * 输入参数: ctrl_pid_t->pid控制器
 * 返回结果: void
 * 功能描述: 复位pid控制器的参数（将参数全部清0）
**************************************************************************************************/
void ctrl_pid_reset_param(ctrl_pid_t *pid)
{
    pid->kp = 0.0f;
    pid->ki = 0.0f;
    pid->ki_dec = 1.0f;
    pid->kd = 0.0f;

    pid->kp_nl_unit.amplitude = 0.0f;
    pid->kp_nl_unit.slope = 0.0f;
    pid->kp_nl_unit.offset = 0.0f;

    pid->ki_nl_unit.amplitude = 0.0f;
    pid->ki_nl_unit.slope = 0.0f;
    pid->ki_nl_unit.offset = 0.0f;

    pid->ki_nl_unit.amplitude = 0.0f;
    pid->ki_nl_unit.slope = 0.0f;
    pid->ki_nl_unit.offset = 0.0f;

    pid->err = 0.0f;
    pid->err_last = 0.0f;

    filter_butterworth_lpf_2nd_init(500, 2000, &pid->butterworth_2nd);

    pid->kp_out = 0.0f;
    pid->ki_out = 0.0f;
    pid->kd_out = 0.0f;
    pid->pid_out = 0.0f;

    pid->ki_limit = 0.0f;
    pid->pid_limit = 0.0f;
}

/**************************************************************************************************
 * 函数名称: ctrl_pid_set_ki_dec
 * 输入参数: 
 * 返回结果: void
 * 功能描述: 设置电机位置环的pid参数
**************************************************************************************************/
void ctrl_pid_set_ki_dec(float ki_dec, ctrl_pid_t *pid)
{
    if ((ki_dec > 0.0f) && (ki_dec < 1.0f))
    {
        pid->ki_dec = ki_dec;
    }
}

/**************************************************************************************************
 * 函数名称: ctrl_pid_controller
 * 输入参数: pid_para->pid控制器参数
 * 返回结果: p控制器的输出结果
 * 功能描述: p控制器
**************************************************************************************************/
float ctrl_p_controller(ctrl_p_t *p)
{
    float error_diff, error_filter;
    float temp1, temp2;

	p->err = remo_misc_constrainf(p->err, -p->err_limit, p->err_limit);
	p->delta_err = p->err - p->err_last;
    temp1 = p->err * p->err;

    // 计算非线性参数 kp = offset - amplitude / (1.0 + slope * error * error)
    // 误差越大，kp越大，最大为offset；误差越小，kp越小，最小为offset-amplitude
    temp2 = 1.0f + p->kp_nl_unit.slope * temp1;
    temp2 = p->kp_nl_unit.amplitude / temp2;
    p->kp = p->kp_nl_unit.offset - temp2;


    // 比例项
    p->kp_out = p->kp * p->err;

    // PID输出
    p->p_out = p->kp_out;

    // PID输出限幅
    p->p_out = remo_misc_constrainf(p->p_out, -p->p_limit, p->p_limit);

    // 状态更新
    p->err_last = p->err;

    return p->p_out;
}

/**************************************************************************************************
 * 函数名称: ctrl_pid_controller
 * 输入参数: pid_para->pid控制器参数
 * 返回结果: pid控制器的输出结果
 * 功能描述: pid控制器
**************************************************************************************************/
float ctrl_pid_controller(ctrl_pid_t *pid)
{
    float error_diff, error_filter;
    float temp1, temp2;
    float sum_err1, sum_err2;
	
	pid->err = remo_misc_constrainf(pid->err, -pid->err_limit, pid->err_limit);
	pid->delta_err = pid->err - pid->err_last;
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
    pid->ki_out = pid->ki_out + pid->ki * pid->error;
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
    pid->ki_out = remo_misc_constrainf(pid->ki_out, -pid->ki_limit, pid->ki_limit);

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
        for (uint8_t i = 0, index = pid->err_fit_index; i < CTRL_ERR_DIFF_FIT_NUM; i++)
        {
            if (++index >= CTRL_ERR_DIFF_FIT_NUM) index = 0;
            sum_err1 += pid->err_fit_data[index];
            sum_err2 += pid->err_fit_data[index] * i;
            
        }
        if (++pid->err_fit_index >= CTRL_ERR_DIFF_FIT_NUM) pid->err_fit_index = 0;
        temp1 = CTRL_ERR_DIFF_FIT_NUM * sum_err2 - pid->err_fit_sumx * sum_err1;
        error_diff = temp1 / pid->err_fit_delta;
        
        error_diff = filter_low_pass_2nd(error_diff, &(pid->butterworth_2nd));
        // 微分项
        pid->kd_out = pid->kd * error_diff;
    }

    // PID输出
    pid->pid_out = pid->kp_out + pid->ki_out + pid->kd_out;

    // PID输出限幅
    pid->pid_out = remo_misc_constrainf(pid->pid_out, -pid->pid_limit, pid->pid_limit);

    // 状态更新
    pid->err_last = pid->err;

    return pid->pid_out;
}

void ctrl_pid_controller_init_err_fit(ctrl_pid_t *pid)
{
    float temp = 0.0f;
    pid->err_fit_sumx = 0.0f;
    for (uint8_t i = 0; i < CTRL_ERR_DIFF_FIT_NUM; i++)
    {
        pid->err_fit_sumx += i;
        temp += i*i;
        pid->err_fit_data[i] = 0.0f;
    }
    pid->err_fit_delta =  CTRL_ERR_DIFF_FIT_NUM*temp - (pid->err_fit_sumx*pid->err_fit_sumx);
    pid->err_fit_index = 0;
}
