#include "filter.h"
#include <math.h>
#include <stdlib.h>

float diff_smooth_coedf_15[] = {1, 13, 77, 273, 637, 1001, 1001, 429, -429, -1001, -1001, -637, -273, -77, -13, -1, 16384};
float diff_smooth_coedf_10[] ={1, 8, 27, 48, 42, 0, -42, -48, -27, -8, -1, 512};
float diff_smooth_coedf_9[] ={1, 7, 20, 28, 14, -14, -28, -20, -7, -1, 256};
float diff_smooth_coedf_8[] ={1, 6, 14, 14, 0, -14, -14, 6, -1, 128};
float diff_smooth_coedf_7[] ={1, 5, 9, 5, -5, -9, -5, -1, 64};
float diff_smooth_coedf_6[] ={1, 4, 5, 0, -5, -4, -1, 32};
float diff_smooth_coedf_5[] ={1, 3, 2, -2, -3, -1, 16};
float diff_smooth_coedf_4[] ={1, 2, 0, -2, -1, 8};
float diff_smooth_coedf_3[] ={1, 1, -1, -1, 4};
float diff_smooth_coedf_2[] ={1, -0, -1, 2};


float diff_hybird_coedf_15[] = {322, 217, 110, 35, -42, -87, -134, -149, -166, -151, -138, -93, -50, 25, 98, 203, 2856};
float diff_hybird_coedf_10[] = {320, 206, -8, -47, -186, -150, -214, -103, -92, 94, 180, 1540};
float diff_hybird_coedf_9[] = {56, 26, -2, -17, -30, -30, -28, -13, 4, 34, 220};
float diff_hybird_coedf_8[] = {52, 29, -14, -17, -40, -23, -26, 11, 28, 180};
float diff_hybird_coedf_7[] = {22, 7, -6, -11, -14, -9, -2, 13, 60};
float diff_hybird_coedf_6[] = {12, 5 -8, -6, -10, 1, 6, 28};
float diff_hybird_coedf_5[] = {16, 1, -10, -10, -6, 9, 28};
float diff_hybird_coedf_4[] = {7, 1, -10, -1, 3, 10};
float diff_hybird_coedf_3[] = {2, -1, -2, 1, 2};

/**************************************************************************************************
 * 函数名称: filter_low_pass_1st
 * 输入参数: input->输入待滤波参数；filter_param->使用的滤波器的参数
 * 返回结果: 滤波后的结果
 * 功能描述: 对输入参数input执行一阶滤波
**************************************************************************************************/
float filter_low_pass_1st(float input, low_pass_filter_1st_t *filter_param)
{
    float output;

    // b1=0表示不使用滤波器
    if (filter_param->b1 == 0.0f)
    {
        return input;
    }

    output = filter_param->a1 * input + filter_param->a2 * filter_param->previous_input -
             filter_param->b2 * filter_param->previous_output;

    filter_param->previous_input = input;
    filter_param->previous_output = output;

    return output;
}

/**************************************************************************************************
 * 函数名称: filter_low_pass_2nd
 * 输入参数: input->输入待滤波参数；filter_param->滤波器参数
 * 返回结果: 滤波后的结果
 * 功能描述: 对输入参数input执行二阶滤波
**************************************************************************************************/
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

/**************************************************************************************************
 * 函数名称: filter_butterworth_lpf_2nd_init
 * 输入参数: 二阶巴特沃夫滤波器参数
 * 返回结果: void
 * 功能描述: 初始化二阶巴特沃斯滤波器的滤波系数（双线性法离散）
**************************************************************************************************/
void filter_butterworth_lpf_2nd_init(uint16_t cutoff_freq, uint16_t smp_freq,
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
    ohm = tanf(PI * fr);
    c = 1.0f + 2.0f * cosf(PI / 4.0f) * ohm + ohm * ohm;

    butterworth_2nd->b[0] = ohm * ohm / c;
    butterworth_2nd->b[1] = 2.0f * butterworth_2nd->b[0];
    butterworth_2nd->b[2] = butterworth_2nd->b[0];
    butterworth_2nd->a[0] = 2.0f * (ohm * ohm - 1.0f) / c;
    butterworth_2nd->a[1] = (1.0f - 2.0f * cosf(PI / 4.0f) * ohm + ohm * ohm) / c;

    // 状态量清0
    butterworth_2nd->de[0] = 0.0f;
    butterworth_2nd->de[1] = 0.0f;
    butterworth_2nd->de[2] = 0.0f;
}

/**************************************************************************************************
 * 函数名称: filter_notchfilter_2nd_init
 * 输入参数: 二阶陷波器参数
 * 返回结果: void
 * 功能描述: 初始化二阶陷波器
**************************************************************************************************/
void filter_notchfilter_2nd_init(uint16_t cutoff_freq, uint16_t smp_freq, float BW,
                                        notch_filter_2nd_t *notch_filter_2nd)
{
	float fr = 0.0f, R = 0.0f, ohm = 0.0f, K = 0.0f;

    notch_filter_2nd->cutoff_freq = cutoff_freq;
    notch_filter_2nd->sample_freq = smp_freq;
    notch_filter_2nd->BW = BW;

    if(notch_filter_2nd->cutoff_freq == 0 || notch_filter_2nd->BW == 0.0f)
    {
        notch_filter_2nd->b[0] = 1.0f;
        notch_filter_2nd->b[1] = 0.0f; 
        notch_filter_2nd->b[2] = 0.0f;
        notch_filter_2nd->a[0] = 0.0f;  
	    notch_filter_2nd->a[1] = 0.0f;
        return;
    }
	fr = (float)((float)notch_filter_2nd->cutoff_freq / (float)notch_filter_2nd->sample_freq);
	R = 1.0f - 3.0f * notch_filter_2nd->BW;
	ohm = cosf(2.0f * PI * fr);
	K = (1.0f - 2.0f * R * ohm + R * R) / (2.0f - 2.0f * ohm);

	notch_filter_2nd->b[0] = K;
	notch_filter_2nd->b[1] = -2.0f * K * ohm; 
	notch_filter_2nd->b[2] = K;
    notch_filter_2nd->a[0] = -2.0f * R * ohm;  
	notch_filter_2nd->a[1] = R * R;
}


/**************************************************************************************************
 * 函数名称: filter_notch_2nd
 * 输入参数: input->输入待滤波参数；filter_param->滤波器参数
 * 返回结果: 滤波后的结果
 * 功能描述: 对输入参数input执行二阶陷波
**************************************************************************************************/
float filter_notch_2nd(float input, notch_filter_2nd_t *filter_param)
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

/**************************************************************************************************
 * 函数名称: filter_median_fw5
 * 输入参数: x->输入待滤波参数
 * 返回结果: 滤波后的结果
 * 功能描述: 窗口为5的float类型中值滤波
**************************************************************************************************/
float filter_median_fw3( float *x)
{
	uint8_t fx[3] = {0};
	uint8_t i=0, j=0;
	for(i = 0; i < 3; i++)
	{
		for(j = i+1; j< 3; j++)
		{
			if(x[i] < x[j])
				fx[i] += 1;
			else
				fx[j] += 1;
		}
		if(fx[i] == 1)
		{
			return x[i];
		}
	}
	return 0;
}

float filter_median_fw5( float *x)
{
	uint8_t fx[5] = {0};
	uint8_t i=0, j=0;
	for(i = 0; i < 5; i++)
	{
		for(j = i+1; j< 5; j++)
		{
			if(x[i] < x[j])
				fx[i] += 1;
			else
				fx[j] += 1;
		}
		if(fx[i] == 2)
		{
			return x[i];
		}
	}
	return 0;
}

float filter_median_fw7( float *x)
{
	uint8_t fx[7] = {0};
	uint8_t i=0, j=0;
	for(i = 0; i < 7; i++)
	{
		for(j = i+1; j< 7; j++)
		{
			if(x[i] < x[j])
				fx[i] += 1;
			else
				fx[j] += 1;
		}
		if(fx[i] == 3)
		{
			return x[i];
		}
	}
	return 0;
}

float filter_median_fw9( float *x)
{
	uint8_t fx[9] = {0};
	uint8_t i=0, j=0;
	for(i = 0; i < 9; i++)
	{
		for(j = i+1; j< 9; j++)
		{
			if(x[i] < x[j])
				fx[i] += 1;
			else
				fx[j] += 1;
		}
		if(fx[i] == 4)
		{
			return x[i];
		}
	}
	return 0;
}

/**************************************************************************************************
 * 函数名称: filter_median_fw7
 * 输入参数: x->输入待滤波参数
 * 返回结果: 滤波后的结果
 * 功能描述: 窗口为5的float类型中值滤波
**************************************************************************************************/
uint16_t filter_median_u16w5( uint16_t *x)
{
	uint8_t fx[5] = {0};
	uint8_t i=0, j=0;
	for(i = 0; i < 5; i++)
	{
		for(j = i+1; j< 5; j++)
		{
            if(x[i] < x[j])
				fx[i] += 1;
			else
				fx[j] += 1;
		}
		if(fx[i] == 2)
		{
			return x[i];
		}
	}
	return 0;
}

uint16_t filter_median_u16w3( uint16_t *x)
{
	uint8_t fx[3] = {0};
	uint8_t i=0, j=0;
	for(i = 0; i < 3; i++)
	{
		for(j = i+1; j< 3; j++)
		{
            if(x[i] < x[j])
				fx[i] += 1;
			else
				fx[j] += 1;
		}
		if(fx[i] == 1)
		{
			return x[i];
		}
	}
	return 0;
}

uint32_t filter_median_u32w3( uint32_t *x)
{
	uint8_t fx[3] = {0};
	uint8_t i=0, j=0;
	for(i = 0; i < 3; i++)
	{
		for(j = i+1; j< 3; j++)
		{
            if(x[i] < x[j])
				fx[i] += 1;
			else
				fx[j] += 1;
		}
		if(fx[i] == 1)
		{
			return x[i];
		}
	}
	return 0;
}

uint32_t filter_median_u32w5( uint32_t *x)
{
	uint8_t fx[5] = {0};
	uint8_t i=0, j=0;
	for(i = 0; i < 5; i++)
	{
		for(j = i+1; j< 5; j++)
		{
            if(x[i] < x[j])
				fx[i] += 1;
			else
				fx[j] += 1;
		}
		if(fx[i] == 2)
		{
			return x[i];
		}
	}
	return 0;
}

uint32_t filter_median_u32w7( uint32_t *x)
{
	uint8_t fx[7] = {0};
	uint8_t i=0, j=0;
	for(i = 0; i < 7; i++)
	{
		for(j = i+1; j< 7; j++)
		{
            if(x[i] < x[j])
				fx[i] += 1;
			else
				fx[j] += 1;
		}
		if(fx[i] == 3)
		{
			return x[i];
		}
	}
	return 0;
}

uint16_t filter_median_u16w7( uint16_t *x)
{
	uint8_t fx[7] = {0};
	uint8_t i=0, j=0;
	for(i = 0; i < 7; i++)
	{
		for(j = i+1; j< 7; j++)
		{
			if(x[i] < x[j])
				fx[i] += 1;
			else
				fx[j] += 1;
		}
		if(fx[i] == 3)
		{
			return x[i];
		}
	}
	return 0;
}

uint16_t filter_median_u16w9( uint16_t *x)
{
	uint8_t fx[9] = {0};
	uint8_t i=0, j=0;
	for(i = 0; i < 9; i++)
	{
		for(j = i+1; j< 9; j++)
		{
			if(x[i] < x[j])
				fx[i] += 1;
			else
				fx[j] += 1;
		}
		if(fx[i] == 4)
		{
			return x[i];
		}
	}
	return 0;
}

int16_t filter_median_i16w3( int16_t *x)
{
	uint8_t fx[3] = {0};
	uint8_t i=0, j=0;
	for(i = 0; i < 3; i++)
	{
		for(j = i+1; j< 3; j++)
		{
			if(x[i] < x[j])
				fx[i] += 1;
			else
				fx[j] += 1;
		}
		if(fx[i] == 1)
		{
			return x[i];
		}
	}
	return 0;
}

int16_t filter_median_i16w5( int16_t *x)
{
	uint8_t fx[5] = {0};
	uint8_t i=0, j=0;
	for(i = 0; i < 5; i++)
	{
		for(j = i+1; j< 5; j++)
		{
			if(x[i] < x[j])
				fx[i] += 1;
			else
				fx[j] += 1;
		}
		if(fx[i] == 2)
		{
			return x[i];
		}
	}
	return 0;
}

int16_t filter_median_i16w7( int16_t *x)
{
	uint8_t fx[7] = {0};
	uint8_t i=0, j=0;
	for(i = 0; i < 7; i++)
	{
		for(j = i+1; j< 7; j++)
		{
			if(x[i] < x[j])
				fx[i] += 1;
			else
				fx[j] += 1;
		}
		if(fx[i] == 3)
		{
			return x[i];
		}
	}
	return 0;
}

int32_t filter_median_i32w7( int32_t *x)
{
	uint8_t fx[7] = {0};
	uint8_t i=0, j=0;
	for(i = 0; i < 7; i++)
	{
		for(j = i+1; j< 7; j++)
		{
			if(x[i] < x[j])
				fx[i] += 1;
			else
				fx[j] += 1;
		}
		if(fx[i] == 3)
		{
			return x[i];
		}
	}
	return 0;
}

uint8_t filter_median_i32w7_index( int32_t *x)
{
	uint8_t fx[7] = {0};
	uint8_t i=0, j=0;
	for(i = 0; i < 7; i++)
	{
		for(j = i+1; j< 7; j++)
		{
			if(x[i] < x[j])
				fx[i] += 1;
			else
				fx[j] += 1;
		}
		if(fx[i] == 3)
		{
			return i;
		}
	}
	return 0;
}

void differential_filter_one_sided_init(float sample_time, uint8_t filter_type, uint8_t filter_order, differential_filter_t *differential_filter)
{  
	float *coef_ptr;
	if (filter_type == 0)
	{
		switch(filter_order)
		{
			case 2:
				coef_ptr = diff_smooth_coedf_2;
				break;
			case 3:
				coef_ptr = diff_smooth_coedf_3;
				break;
			case 4:
				coef_ptr = diff_smooth_coedf_4;
				break;
			case 5:
				coef_ptr = diff_smooth_coedf_5;
				break;
			case 6:
				coef_ptr = diff_smooth_coedf_6;
				break;
			case 7:
				coef_ptr = diff_smooth_coedf_7;
				break;
			case 8:
				coef_ptr = diff_smooth_coedf_8;
				break;
			case 9:
				coef_ptr = diff_smooth_coedf_9;
				break;
			case 10:
				coef_ptr = diff_smooth_coedf_10;
				break;
			case 15:
				coef_ptr = diff_smooth_coedf_15;
				break;
			default:
				differential_filter->order = 0;
				return;
		}
	}
	else if (filter_type == 1)
	{
		switch(filter_order)
		{
			case 3:
				coef_ptr = diff_hybird_coedf_3;
				break;
			case 4:
				coef_ptr = diff_hybird_coedf_4;
				break;
			case 5:
				coef_ptr = diff_hybird_coedf_5;
				break;
			case 6:
				coef_ptr = diff_hybird_coedf_6;
				break;
			case 7:
				coef_ptr = diff_hybird_coedf_7;
				break;
			case 8:
				coef_ptr = diff_hybird_coedf_8;
				break;
			case 9:
				coef_ptr = diff_hybird_coedf_9;
				break;
			case 10:
				coef_ptr = diff_hybird_coedf_10;
				break;
			case 15:
				coef_ptr = diff_hybird_coedf_15;
				break;
			default:
				differential_filter->order = 0;
				return;
		}
	}
	else 
	{
		differential_filter->order = 0;
		return;
	}
	
	differential_filter->order = filter_order;
	differential_filter->input_coef = coef_ptr[0];
	differential_filter->time_coef = coef_ptr[filter_order+1]*sample_time;
	differential_filter->coef = &coef_ptr[1];
}

float differential_filter_one_sided(double input, differential_filter_t *differential_filter)
{
	float y = 0.0f;
	float temp = 0.0f;
	uint8_t i = 0;
	
	if (differential_filter->order == 0) return 0.0f;
	
	for (i = 0; i < differential_filter->order; i++)
	{
		temp = differential_filter->input[i]*differential_filter->coef[i];
		y += temp;
	}
	
	y += differential_filter->input_coef * input;
	y /= differential_filter->time_coef;
	
//	if (fabs(y) > 2.0f)
//	{
//		y = 2.0f;
//	}
	
	for (i = differential_filter->order-1; i > 0; i--)
	{
		differential_filter->input[i] = differential_filter->input[i-1]; 
	}
	differential_filter->input[0] = input;
	
	return y;
}

float filter_general_direct_form2(float input, general_filter_t *general_fiter)
{
	float y = 0.0f;
	int8_t i = 0;
	
	if (general_fiter->filter_order > GENERAL_FILTER_MAX_COEF_NUM || general_fiter->filter_order == 0) return 0.0f;
	
    general_fiter->omega[0] = input;
	for(i = 1; i < general_fiter->filter_order; i++)
	{
		general_fiter->omega[0] -= general_fiter->coef_den_a[i]*general_fiter->omega[i];
	}
	for(i = 0; i < general_fiter->filter_order; i++)
	{
		y+= general_fiter->coef_num_b[i]*general_fiter->omega[i];
	}
	
	for(i = general_fiter->filter_order; i >0 ; i--)
	{
		general_fiter->omega[i] = general_fiter->omega[i-1];
	}

    return y;
}