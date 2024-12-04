#ifndef TEST_FILTER_H
#define TEST_FILTER_H

#include <stdlib.h>
#include <stdint.h>

#include "diff_filter.h"

#define PI  3.14159265f

// 一阶低通滤波器参数
// 滤波器暂定为：(a1*z+a2)/(b1*z+b2)
typedef struct
{
    float b1; // 取值为1
    float b2;
    float a1;
    float a2;

    float previous_input;  // 上一次输入
    float previous_output; // 上一次输出
} low_pass_filter_1st_t;

// 二阶低通滤波器参数
typedef struct
{
    uint16_t sample_freq; // 采样频率
    uint16_t cutoff_freq; // 截止频率

    float a[2];  // 分母：1+a[0]*z^(-1)+a[1]*z^(-2)
    float b[3];  // 分子：b[0]+b[1]*z^(-1)+b[2]*z^(-2)
    float de[3]; // 延时元素，delay_element
} low_pass_filter_2nd_t;    //32+4

// 二阶陷波器参数
typedef struct
{
    uint16_t sample_freq; // 采样频率
    uint16_t cutoff_freq; // 截止频率
    float BW;             // 陷波带宽比例

    float a[2];  // 分母：1+a[0]*z^(-1)+a[1]*z^(-2)
    float b[3];  // 分子：b[0]+b[1]*z^(-1)+b[2]*z^(-2)
    float de[3]; // 延时元素，delay_element
} notch_filter_2nd_t;


#define GENERAL_FILTER_MAX_COEF_NUM   (8)   // 最大8阶
typedef enum{
	LOWPASS_FILTER = 0,
	HIGHPASS_FILTER,
	DERIVATIVE_FILTER,
	NOTCH_FILTER,
}filter_type_t;
typedef struct{
	float coef_num_b[GENERAL_FILTER_MAX_COEF_NUM+1];
	float coef_den_a[GENERAL_FILTER_MAX_COEF_NUM+1];
	
	float omega[GENERAL_FILTER_MAX_COEF_NUM+1];
	
	filter_type_t filter_type;
	uint8_t filter_order;
}general_filter_t;


float filter_low_pass_1st(float input, low_pass_filter_1st_t *filter_param);
float filter_low_pass_2nd(float input, low_pass_filter_2nd_t *filter_param);

void filter_butterworth_lpf_2nd_init(uint16_t cutoff_freq, uint16_t smp_freq,
                                          low_pass_filter_2nd_t *butterworth_2nd);

void filter_notchfilter_2nd_init(uint16_t cutoff_freq, uint16_t smp_freq, float BW,
                                        notch_filter_2nd_t *notch_filter_2nd);
float filter_notch_2nd(float input, notch_filter_2nd_t *filter_param);

float filter_median_fw3( float *x);
float filter_median_fw5( float *x);
float filter_median_fw7( float *x);
float filter_median_fw9( float *x);
uint16_t filter_median_u16w3( uint16_t *x);
uint16_t filter_median_u16w5( uint16_t *x);
uint32_t filter_median_u32w3( uint32_t *x);
uint32_t filter_median_u32w5( uint32_t *x);
uint32_t filter_median_u32w7( uint32_t *x);
uint16_t filter_median_u16w7( uint16_t *x);
uint16_t filter_median_u16w9( uint16_t *x);
int16_t filter_median_i16w3( int16_t *x);
int16_t filter_median_i16w5( int16_t *x);
int16_t filter_median_i16w7( int16_t *x);
int32_t filter_median_i32w7( int32_t *x);
uint8_t filter_median_i32w7_index( int32_t *x);

void differential_filter_one_sided_init(float sample_time, uint8_t filter_type, uint8_t filter_order, differential_filter_t *differential_filter);
float differential_filter_one_sided(double input, differential_filter_t *differential_filter);
float filter_general_direct_form2(float input, general_filter_t *general_fiter);

#endif   // TEST_FILTER_H
