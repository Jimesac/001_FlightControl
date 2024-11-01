#include "fft.h"
#include "math.h"
#include "stdlib.h"
#include "bsp_timer.h"
#include "fixed_point.h"
#include "imu.h"
#include "misc.h"
#include "user_config.h"
// #define FFT_TRIG_USE_FUN

// 以2为基底的fft变化每一个stage的计算时间随着stage的增大而增大
#define FFT_RADIX_NUM    (2)

#define FFT_SAMPLE_SBN           ((uint16_t)8) // radix = 2:1,2,...12; radix = 4:1,2,...,6; radix = 8:1,2,...,4.
#define FFT_SAMPLE_NUM           ((uint16_t)1<<FFT_SAMPLE_SBN)

#define FFT_SAMPLE_FRE           ((uint16_t)SCHEDULE_SECTION_FREQ)

struct {
    uint16_t sample_index;
    bool sample_finished_flag;
	uint8_t sample_data_type;

    uint8_t stage;
    bool calc_finished_flag;

    uint16_t freq[3];
    uint32_t magnitude[3];
    uint8_t res_index;

    int16_t dc;
}fft_status = {
    .sample_index = 0,
    .sample_finished_flag = false,
	.sample_data_type = PITCH,

    .stage = 0,
    .calc_finished_flag = false,
    .freq = {0},
    .magnitude = {0},
    .res_index = 0,
    .dc = 0
};


fft_complex_t mytest_complex[FFT_SAMPLE_NUM] = {0};

struct{
    uint16_t fre[2];
    float amp[2];
    float pha[2];
}fft_test_signal = {
    .fre = {100, 16},
    .amp = {100, 300},
    .pha = {0, 0}
};

static void mytest_fft_sample(int16_t data);
static void mytest_fft_permutate(fft_complex_t complex[], uint8_t radix_exp_v);
static void mytest_fft_forward_r2_sx(fft_complex_t complex[], uint8_t radix_exp_v, uint8_t stage_i);
static void mytest_fft_forward_r2_se1(fft_complex_t complex[], uint8_t radix_exp_v);
static void mytest_fft_forward_r2_se2(fft_complex_t complex[], uint8_t radix_exp_v);
static void mytest_fft_forward_r2_se3(fft_complex_t complex[], uint8_t radix_exp_v);
static void mytest_fft_cal_magnitude(fft_complex_t complex[], uint8_t radix_exp_v);


uint32_t mytest_fft_time[12][2] = {0};
void fft_test(void)
{ 

	float temp = 0.0f;
    // 生成信号
    for (uint16_t i = 0; i < FFT_SAMPLE_NUM; i ++)
    {
		temp = (float)(i*fft_test_signal.fre[0]);
        mytest_complex[i].r = fft_test_signal.amp[0]*sinf(temp/(float)FFT_SAMPLE_FRE * 6.28318f + fft_test_signal.pha[0]);
		temp = (float)(i*fft_test_signal.fre[1]);
        mytest_complex[i].r += fft_test_signal.amp[1]*sinf(temp/(float)FFT_SAMPLE_FRE * 6.28318f + fft_test_signal.pha[1]);

    }

    
    mytest_fft_time[0][0] =  bsp_timer_clock_start_0p1us();
    mytest_fft_permutate(mytest_complex, FFT_SAMPLE_SBN);
    mytest_fft_time[0][1] =  bsp_timer_clock_end_0p1us(mytest_fft_time[0][0]);


    for (uint8_t i = 1; i <= FFT_SAMPLE_SBN - 3; i ++)
	{
		mytest_fft_time[i][0] =  bsp_timer_clock_start_0p1us();
		mytest_fft_forward_r2_sx(mytest_complex, FFT_SAMPLE_SBN, i);
		mytest_fft_time[i][1] =  bsp_timer_clock_end_0p1us(mytest_fft_time[i][0]);
	}

    mytest_fft_time[FFT_SAMPLE_SBN-2][0] =  bsp_timer_clock_start_0p1us();
    mytest_fft_forward_r2_se3(mytest_complex, FFT_SAMPLE_SBN);
    mytest_fft_time[FFT_SAMPLE_SBN-2][1] =  bsp_timer_clock_end_0p1us(mytest_fft_time[FFT_SAMPLE_SBN-2][0]);

    mytest_fft_time[FFT_SAMPLE_SBN-1][0] =  bsp_timer_clock_start_0p1us();
    mytest_fft_forward_r2_se2(mytest_complex, FFT_SAMPLE_SBN);
    mytest_fft_time[FFT_SAMPLE_SBN-1][1] =  bsp_timer_clock_end_0p1us(mytest_fft_time[FFT_SAMPLE_SBN-1][0]);

    mytest_fft_time[FFT_SAMPLE_SBN][0] =  bsp_timer_clock_start_0p1us();
    mytest_fft_forward_r2_se1(mytest_complex, FFT_SAMPLE_SBN);
    mytest_fft_time[FFT_SAMPLE_SBN][1] =  bsp_timer_clock_end_0p1us(mytest_fft_time[FFT_SAMPLE_SBN][0]);

    mytest_fft_time[FFT_SAMPLE_SBN + 1][0] =  bsp_timer_clock_start_0p1us();
    mytest_fft_cal_magnitude(mytest_complex, FFT_SAMPLE_SBN);
    mytest_fft_time[FFT_SAMPLE_SBN + 1][1] =  bsp_timer_clock_end_0p1us(mytest_fft_time[FFT_SAMPLE_SBN + 1][0]);




    // fft_permutate(mytest_complex, FFT_SAMPLE_SBN);
    // mytest_fft_forward_r2(mytest_complex, FFT_SAMPLE_SBN);
    // mytest_fft_time[1][1] =  test_clock_timing_end_us(mytest_fft_time[1][0]);
}

void mytest_fft_update(void)
{
    const float *gyro_corr = remo_imu_get_gyro_corr();
    if (!fft_status.sample_finished_flag)
    {
        mytest_fft_sample(gyro_corr[fft_status.sample_data_type]*50);
        fft_status.calc_finished_flag = false;
        fft_status.stage = 0;
    }
    else if (!fft_status.calc_finished_flag)
    {
        switch(fft_status.stage)
        {
            case 0:
                mytest_fft_permutate(mytest_complex, FFT_SAMPLE_SBN);
                fft_status.freq[0] = 0;
                fft_status.freq[1] = 0;
                fft_status.freq[2] = 0;
                fft_status.magnitude[0] = 0;
                fft_status.magnitude[1] = 0;
                fft_status.magnitude[2] = 0;
                fft_status.dc = 0;
                break;
            case 1:
            case 2:
            case 3:
            case 4:
            case 5:
                mytest_fft_forward_r2_sx(mytest_complex, FFT_SAMPLE_SBN, fft_status.stage);
                break;
            case 6:
                mytest_fft_forward_r2_se3(mytest_complex, FFT_SAMPLE_SBN);
                break;
            case 7:
                mytest_fft_forward_r2_se2(mytest_complex, FFT_SAMPLE_SBN);
                break;
            case 8:
                mytest_fft_forward_r2_se1(mytest_complex, FFT_SAMPLE_SBN);
                break;
            case 9:
                mytest_fft_cal_magnitude(mytest_complex, FFT_SAMPLE_SBN);
                fft_status.calc_finished_flag = true;
                fft_status.sample_finished_flag = false;
				if (fft_status.sample_data_type == PITCH) fft_status.sample_data_type = YAW;
				else if (fft_status.sample_data_type == YAW) fft_status.sample_data_type = PITCH;
                break;
            default:
                break;
        }
        fft_status.stage ++;
    }
}

static void mytest_fft_sample(int16_t data)
{
    int32_t scaled_data;
    int8_t sign = 0;

    sign = (data > 0) ? 1:-1;
    scaled_data = 512 + abs(data);
    scaled_data = 300 - 153600/scaled_data;

    if (fft_status.sample_index < FFT_SAMPLE_NUM)
    {
        mytest_complex[fft_status.sample_index].r = scaled_data * sign;
        mytest_complex[fft_status.sample_index].i = 0;
        fft_status.sample_finished_flag = false;
        fft_status.sample_index ++;
        if (fft_status.sample_index == FFT_SAMPLE_NUM)
        {
            fft_status.sample_finished_flag = true;
		    fft_status.sample_index = 0;
        }
    }
}

static void mytest_fft_permutate(fft_complex_t complex[], uint8_t radix_exp_v)
{
    uint32_t i = 0, j = 0, result = 0;
    uint16_t num = 1 << radix_exp_v, num_half = 1 << (radix_exp_v -1);
    fft_complex_t complex_temp;

    for (i = 0; i < num; i=i+1)
    {
        // j = RBITS(i, radix_exp_v);
        // RBITS(W, BITS) (rbit(W) >> (32 - (BITS)))

        j = i;
        j = (((j & 0xaaaaaaaa) >> 1) | ((j & 0x55555555) << 1));
        j = (((j & 0xcccccccc) >> 2) | ((j & 0x33333333) << 2));
        j = (((j & 0xf0f0f0f0) >> 4) | ((j & 0x0f0f0f0f) << 4));
        j = (((j & 0xff00ff00) >> 8) | ((j & 0x00ff00ff) << 8));
        j = (j >> 16) | (j << 16);

        j = j >> (32 - (radix_exp_v));

        if (j >= i)
        {
            complex_temp.r = complex[i].r;
            complex[i].r = complex[j].r;
            complex[j].r = complex_temp.r;
        }

        i += 1;
        j += num_half;
        if (j >= i)
        {
            complex_temp.r = complex[i].r;
            complex[i].r = complex[j].r;
            complex[j].r = complex_temp.r;
        }
    }
}

static void mytest_fft_cal_magnitude(fft_complex_t complex[], uint8_t radix_exp_v)
{
    uint16_t size = 1 << (radix_exp_v - 1);
    uint16_t freq_hHz = FFT_SAMPLE_FRE * 100 / (size << 1); // 每一个数据表示的频率，单位100*hz
    uint16_t i = 0;

    uint32_t magnitude_temp = 0; 
    uint16_t freq_temp = 0;

    for (i = 0; i < size-3;)
    {
        magnitude_temp = complex[i].r * complex[i].r + complex[i].i * complex[i].i;
        complex[i].i = freq_temp = freq_hHz * i;
        complex[i++].r = magnitude_temp >> 16; 
        magnitude_temp = complex[i].r * complex[i].r + complex[i].i * complex[i].i;
        complex[i].i = freq_temp = freq_hHz * i;
        complex[i++].r = magnitude_temp >> 16; 
        magnitude_temp = complex[i].r * complex[i].r + complex[i].i * complex[i].i;
        complex[i].i = freq_temp = freq_hHz * i;
        complex[i++].r = magnitude_temp >> 16; 
        magnitude_temp = complex[i].r * complex[i].r + complex[i].i * complex[i].i;
        complex[i].i = freq_temp = freq_hHz * i;
        complex[i++].r = magnitude_temp >> 16; 
    }

    for (i = 1; i < size-1; i++)
    {
        if (complex[i].r >= complex[i-1].r && complex[i].r > complex[i-2].r && \
            complex[i].r >= complex[i+1].r && complex[i].r > complex[i+2].r)
        {
            if (complex[i].r > fft_status.magnitude[0])
            {
                fft_status.freq[2] = fft_status.freq[1];
                fft_status.magnitude[2] = fft_status.magnitude[1];
                fft_status.freq[1] = fft_status.freq[0];
                fft_status.magnitude[1] = fft_status.magnitude[0];
                fft_status.freq[0] = complex[i].i;
                fft_status.magnitude[0] = complex[i].r;
            }
            else if (complex[i].r > fft_status.magnitude[1])
            {
                fft_status.freq[2] = fft_status.freq[1];
                fft_status.magnitude[2] = fft_status.magnitude[1];
                fft_status.freq[1] = complex[i].i;
                fft_status.magnitude[1] = complex[i].r;
            }
            else if (complex[i].r > fft_status.magnitude[2])
            {
                fft_status.freq[2] = complex[i].i;
                fft_status.magnitude[2] = complex[i].r;
            }
        }
    }

    fft_status.dc = complex[1].r - complex[0].r;
}


static void mytest_fft_forward_r2_sx(fft_complex_t complex[], uint8_t radix_exp_v, uint8_t stage_i)
{
    // uint16_t W_N = 0;
    uint16_t k_size = 0, k_i = 0;

    uint16_t bf_block_elem_size = 0;         // 一个蝶形块元素的数目
    uint16_t bf_block_size = 0;
    uint16_t bf_pair_interval = 0;

    uint16_t elem_index1 = 0, elem_index2 = 0;

    int32_t W_N_ki_cos = 0, W_N_ki_sin = 0;

    uint16_t i = 0;
    uint8_t trig_div_num = stage_i - 2;

    fft_complex_t complex_temp;
    uint8_t section;
    uint16_t sintable_index;
    const int16_t *sintable = fp_get_sintable();

    // 从stage3开始迭代计算
    // W_N = 1 << stage_i;                  // W_N_k乘子的N的值，从2开始，以2为系数的等比数列,2、4、8、16...
    k_size = 1 << (stage_i - 1);            // W_N_k乘子的k的范围[0, k_size)，从0开始，其值为W_N/2 - 1，0、2、4、8...

    bf_pair_interval = k_size;      // 蝶形运算对的间隔，从1开始，其值为W_N/2，1、2、4、8...
    bf_block_elem_size = k_size << 1;   // 一个蝶形块的元素数目
    bf_block_size = 1 << (radix_exp_v - stage_i);   // 蝶形运算块的数量

    // 每一个蝶形块的W_N_k的第一个值(k_i=0)为1，是一个固定的值，此处单独处理以提高效率
    // 为啥单独处理效率还更低了
    elem_index1 = 0;
    for (i = 0; i < bf_block_size; i ++)
    {   
        // 每一个W_N_k乘子对应的值都包含在一个蝶形运算对里面
        elem_index2 = elem_index1 + bf_pair_interval;
        complex[elem_index1].r = complex[elem_index1].r + complex[elem_index2].r;
        complex[elem_index1].i = complex[elem_index1].i + complex[elem_index2].i;
        complex[elem_index2].r = complex[elem_index1].r - (complex[elem_index2].r << 1);
        complex[elem_index2].i = complex[elem_index1].i - (complex[elem_index2].i << 1);

        elem_index1 += bf_block_elem_size;
    }
    
    // 每个stage的数据与W_N_k相乘后更新值。
    // k_size = (radix_exp_v == stage_i)? (k_size >> 1):k_size;
    for (k_i = 1; k_i < k_size; k_i ++)   
    {

        // 注意这里的指数形式是e^(2*pi*k/N),实际应该是e^(-2*pi*k/N)
        section = k_i >> trig_div_num;  
        sintable_index = (k_i << (8 - trig_div_num)) & SIN_0TO90_INDEX_MASK;
        switch(section)
        {
            case 0:
                W_N_ki_sin  = sintable[sintable_index];
                W_N_ki_cos = sintable[0xff - sintable_index];
                break;
            case 1:
                W_N_ki_sin = sintable[0xff - sintable_index];
                W_N_ki_cos = -sintable[sintable_index];
                break;
            case 2:
                W_N_ki_sin = -sintable[sintable_index];
                W_N_ki_cos = -sintable[0xff - sintable_index];
                break;
            case 3:
                W_N_ki_sin = -sintable[0xff - sintable_index];
                W_N_ki_cos = sintable[sintable_index];
                break;
            default:
                break;
        }

        elem_index1 = k_i;
        for (i = 0; i < bf_block_size; i ++)
        {   
            elem_index2 = elem_index1 + bf_pair_interval;

            // 注意这里的复数是cosα - jsinα
            complex_temp.r = (W_N_ki_sin * complex[elem_index2].i + W_N_ki_cos * complex[elem_index2].r) >> 14;
            complex_temp.i = (W_N_ki_cos * complex[elem_index2].i - W_N_ki_sin * complex[elem_index2].r) >> 14;

            // 每一个W_N_k乘子对应的值都包含在一个蝶形运算对里面
            complex[elem_index1].r = complex[elem_index1].r + complex_temp.r;
            complex[elem_index1].i = complex[elem_index1].i + complex_temp.i;
            complex[elem_index2].r = complex[elem_index1].r - (complex_temp.r << 1);// - complex[factor_index].r;
            complex[elem_index2].i = complex[elem_index1].i - (complex_temp.i << 1);//- complex[factor_index].i;

            elem_index1 += bf_block_elem_size;
        }
    }
}


static void mytest_fft_forward_r2_se3(fft_complex_t complex[], uint8_t radix_exp_v)
{
    // uint16_t W_N = 0;
    uint16_t k_size = 0, k_i = 0;

    uint16_t bf_block_elem_size = 0;         // 一个蝶形块元素的数目
    uint16_t bf_pair_interval = 0;

    uint16_t elem_index1 = 0, elem_index2 = 0;

    int32_t W_N_ki_cos = 0, W_N_ki_sin = 0;
    uint8_t trig_div_num = radix_exp_v - 4;

    uint16_t i = 0, j = 0;

    fft_complex_t complex_temp;

#ifndef FFT_TRIG_USE_FUN
    uint8_t section;
    uint16_t sintable_index;
    const int16_t *sintable = fp_get_sintable();
#endif

    // 从stage3开始迭代计算
    // W_N = 1 << stage_i;                  // W_N_k乘子的N的值，从2开始，以2为系数的等比数列,2、4、8、16...
    k_size = 1 << (radix_exp_v - 3);            // W_N_k乘子的k的范围[0, k_size)，从0开始，其值为W_N/2 - 1，0、2、4、8...

    bf_pair_interval = k_size;      // 蝶形运算对的间隔，从1开始，其值为W_N/2，1、2、4、8...
    bf_block_elem_size = k_size << 1;   // 一个蝶形块的元素数目

    // 每个stage的数据与W_N_k相乘后更新值。
    for (k_i = 0; k_i < k_size; k_i ++)   
    {

        // 注意这里的指数形式是e^(2*pi*k/N),实际应该是e^(-2*pi*k/N)
        section = k_i >> trig_div_num;  
        sintable_index = (k_i << (8 - trig_div_num)) & SIN_0TO90_INDEX_MASK;
        switch(section)
        {
            case 0:
                W_N_ki_sin  = sintable[sintable_index];
                W_N_ki_cos = sintable[0xff - sintable_index];
                break;
            case 1:
                W_N_ki_sin = sintable[0xff - sintable_index];
                W_N_ki_cos = -sintable[sintable_index];
                break;
            case 2:
                W_N_ki_sin = -sintable[sintable_index];
                W_N_ki_cos = -sintable[0xff - sintable_index];
                break;
            case 3:
                W_N_ki_sin = -sintable[0xff - sintable_index];
                W_N_ki_cos = sintable[sintable_index];
                break;
            default:
                break;
        }
		
        elem_index1 = k_i;
        elem_index2 = elem_index1 + bf_pair_interval;
        // 注意这里的复数是cosα - jsinα
        complex_temp.r = (W_N_ki_sin * complex[elem_index2].i + W_N_ki_cos * complex[elem_index2].r) >> 14;
        complex_temp.i = (W_N_ki_cos * complex[elem_index2].i - W_N_ki_sin * complex[elem_index2].r) >> 14;
        // 每一个W_N_k乘子对应的值都包含在一个蝶形运算对里面
        complex[elem_index1].r = complex[elem_index1].r + complex_temp.r;
        complex[elem_index1].i = complex[elem_index1].i + complex_temp.i;
        complex[elem_index2].r = complex[elem_index1].r - (complex_temp.r << 1);// - complex[factor_index].r;
        complex[elem_index2].i = complex[elem_index1].i - (complex_temp.i << 1);//- complex[factor_index].i;

        elem_index1 += bf_block_elem_size;
        elem_index2 = elem_index1 + bf_pair_interval;
        // 注意这里的复数是cosα - jsinα
        complex_temp.r = (W_N_ki_sin * complex[elem_index2].i + W_N_ki_cos * complex[elem_index2].r) >> 14;
        complex_temp.i = (W_N_ki_cos * complex[elem_index2].i - W_N_ki_sin * complex[elem_index2].r) >> 14;
        // 每一个W_N_k乘子对应的值都包含在一个蝶形运算对里面
        complex[elem_index1].r = complex[elem_index1].r + complex_temp.r;
        complex[elem_index1].i = complex[elem_index1].i + complex_temp.i;
        complex[elem_index2].r = complex[elem_index1].r - (complex_temp.r << 1);// - complex[factor_index].r;
        complex[elem_index2].i = complex[elem_index1].i - (complex_temp.i << 1);//- complex[factor_index].i;

        elem_index1 += bf_block_elem_size;
        elem_index2 = elem_index1 + bf_pair_interval;
        // 注意这里的复数是cosα - jsinα
        complex_temp.r = (W_N_ki_sin * complex[elem_index2].i + W_N_ki_cos * complex[elem_index2].r) >> 14;
        complex_temp.i = (W_N_ki_cos * complex[elem_index2].i - W_N_ki_sin * complex[elem_index2].r) >> 14;
        // 每一个W_N_k乘子对应的值都包含在一个蝶形运算对里面
        complex[elem_index1].r = complex[elem_index1].r + complex_temp.r;
        complex[elem_index1].i = complex[elem_index1].i + complex_temp.i;
        complex[elem_index2].r = complex[elem_index1].r - (complex_temp.r << 1);// - complex[factor_index].r;
        complex[elem_index2].i = complex[elem_index1].i - (complex_temp.i << 1);//- complex[factor_index].i;

        elem_index1 += bf_block_elem_size;
        elem_index2 = elem_index1 + bf_pair_interval;
        // 注意这里的复数是cosα - jsinα
        complex_temp.r = (W_N_ki_sin * complex[elem_index2].i + W_N_ki_cos * complex[elem_index2].r) >> 14;
        complex_temp.i = (W_N_ki_cos * complex[elem_index2].i - W_N_ki_sin * complex[elem_index2].r) >> 14;
        // 每一个W_N_k乘子对应的值都包含在一个蝶形运算对里面
        complex[elem_index1].r = complex[elem_index1].r + complex_temp.r;
        complex[elem_index1].i = complex[elem_index1].i + complex_temp.i;
        complex[elem_index2].r = complex[elem_index1].r - (complex_temp.r << 1);// - complex[factor_index].r;
        complex[elem_index2].i = complex[elem_index1].i - (complex_temp.i << 1);//- complex[factor_index].i;

    }
}

static void mytest_fft_forward_r2_se2(fft_complex_t complex[], uint8_t radix_exp_v)
{
    // uint16_t W_N = 0;
    uint16_t k_size = 0, k_i = 0;

    uint16_t bf_block_elem_size = 0;         // 一个蝶形块元素的数目
    uint16_t bf_pair_interval = 0;

    uint16_t elem_index1 = 0, elem_index2 = 0;

    int32_t W_N_ki_cos = 0, W_N_ki_sin = 0;
    uint8_t trig_div_num = radix_exp_v - 3;

    fft_complex_t complex_temp;

#ifndef FFT_TRIG_USE_FUN
    uint8_t section;
    uint16_t sintable_index;
    const int16_t *sintable = fp_get_sintable();
#endif

    // 从stage3开始迭代计算
    // W_N = 1 << stage_i;                  // W_N_k乘子的N的值，从2开始，以2为系数的等比数列,2、4、8、16...
    k_size = 1 << (radix_exp_v - 2);            // W_N_k乘子的k的范围[0, k_size)，从0开始，其值为W_N/2 - 1，0、2、4、8...

    bf_pair_interval = k_size;      // 蝶形运算对的间隔，从1开始，其值为W_N/2，1、2、4、8...
    bf_block_elem_size = k_size << 1;   // 一个蝶形块的元素数目
    
    // 每个stage的数据与W_N_k相乘后更新值。
    for (k_i = 0; k_i < k_size; k_i ++)   
    {

        // 注意这里的指数形式是e^(2*pi*k/N),实际应该是e^(-2*pi*k/N)
        section = k_i >> trig_div_num;  
        sintable_index = (k_i << (8 - trig_div_num)) & SIN_0TO90_INDEX_MASK;
        switch(section)
        {
            case 0:
                W_N_ki_sin  = sintable[sintable_index];
                W_N_ki_cos = sintable[0xff - sintable_index];
                break;
            case 1:
                W_N_ki_sin = sintable[0xff - sintable_index];
                W_N_ki_cos = -sintable[sintable_index];
                break;
            case 2:
                W_N_ki_sin = -sintable[sintable_index];
                W_N_ki_cos = -sintable[0xff - sintable_index];
                break;
            case 3:
                W_N_ki_sin = -sintable[0xff - sintable_index];
                W_N_ki_cos = sintable[sintable_index];
                break;
            default:
                break;
        }

        elem_index1 = k_i;
        elem_index2 = elem_index1 + bf_pair_interval;

        // 注意这里的复数是cosα - jsinα
        complex_temp.r = (W_N_ki_sin * complex[elem_index2].i + W_N_ki_cos * complex[elem_index2].r) >> 14;
        complex_temp.i = (W_N_ki_cos * complex[elem_index2].i - W_N_ki_sin * complex[elem_index2].r) >> 14;

        // 每一个W_N_k乘子对应的值都包含在一个蝶形运算对里面
        complex[elem_index1].r = complex[elem_index1].r + complex_temp.r;
        complex[elem_index1].i = complex[elem_index1].i + complex_temp.i;
        complex[elem_index2].r = complex[elem_index1].r - (complex_temp.r << 1);// - complex[factor_index].r;
        complex[elem_index2].i = complex[elem_index1].i - (complex_temp.i << 1);//- complex[factor_index].i;

        elem_index1 += bf_block_elem_size;
        elem_index2 = elem_index1 + bf_pair_interval;

        // 注意这里的复数是cosα - jsinα
        complex_temp.r = (W_N_ki_sin * complex[elem_index2].i + W_N_ki_cos * complex[elem_index2].r) >> 14;
        complex_temp.i = (W_N_ki_cos * complex[elem_index2].i - W_N_ki_sin * complex[elem_index2].r) >> 14;

        // 每一个W_N_k乘子对应的值都包含在一个蝶形运算对里面
        complex[elem_index1].r = complex[elem_index1].r + complex_temp.r;
        complex[elem_index1].i = complex[elem_index1].i + complex_temp.i;
        complex[elem_index2].r = complex[elem_index1].r - (complex_temp.r << 1);// - complex[factor_index].r;
        complex[elem_index2].i = complex[elem_index1].i - (complex_temp.i << 1);//- complex[factor_index].i;

    }
}

static void mytest_fft_forward_r2_se1(fft_complex_t complex[], uint8_t radix_exp_v)
{
    // uint16_t W_N = 0;
    uint16_t k_size = 0;

    uint16_t bf_block_elem_size = 0;         // 一个蝶形块元素的数目
    uint16_t bf_pair_interval = 0;

    uint16_t elem_index1 = 0, elem_index2 = 0;

    int32_t W_N_ki_cos = 0, W_N_ki_sin = 0;

#ifndef FFT_TRIG_USE_FUN
    uint8_t section;
    uint16_t sintable_index;
    const int16_t *sintable = fp_get_sintable();
#endif


    // 从stage3开始迭代计算
    // W_N = 1 << stage_i;                  // W_N_k乘子的N的值，从2开始，以2为系数的等比数列,2、4、8、16...
    k_size = 1 << (radix_exp_v - 1);            // W_N_k乘子的k的范围[0, k_size)，从0开始，其值为W_N/2 - 1，0、2、4、8...

    bf_pair_interval = k_size;      // 蝶形运算对的间隔，从1开始，其值为W_N/2，1、2、4、8...
    bf_block_elem_size = k_size << 1;   // 一个蝶形块的元素数目

    // 每一个蝶形块的W_N_k的第一个值(k_i=0)为1，是一个固定的值，此处单独处理以提高效率
    // 为啥单独处理效率还更低了
    elem_index1 = 0; 
    // 每一个W_N_k乘子对应的值都包含在一个蝶形运算对里面
    elem_index2 = elem_index1 + bf_pair_interval;
    complex[elem_index1].r = complex[elem_index1].r + complex[elem_index2].r;
    complex[elem_index1].i = complex[elem_index1].i + complex[elem_index2].i;
    complex[elem_index2].r = complex[elem_index1].r - (complex[elem_index2].r << 1);
    complex[elem_index2].i = complex[elem_index1].i - (complex[elem_index2].i << 1);
    
    radix_exp_v -= 2;
    // 每个stage的数据与W_N_k相乘后更新值。
    for (elem_index1 = 1; elem_index1 < k_size; elem_index1 ++)   
    {

        // 注意这里的指数形式是e^(2*pi*k/N),实际应该是e^(-2*pi*k/N)
        section = elem_index1 >> radix_exp_v;  
        sintable_index = (elem_index1 << (8 - radix_exp_v)) & SIN_0TO90_INDEX_MASK;

        switch(section)
        {
            case 0:
                W_N_ki_sin  = sintable[sintable_index];
                W_N_ki_cos = sintable[0xff - sintable_index];
                break;
            case 1:
                W_N_ki_sin = sintable[0xff - sintable_index];
                W_N_ki_cos = -sintable[sintable_index];
                break;
            case 2:
                W_N_ki_sin = -sintable[sintable_index];
                W_N_ki_cos = -sintable[0xff - sintable_index];
                break;
            case 3:
                W_N_ki_sin = -sintable[0xff - sintable_index];
                W_N_ki_cos = sintable[sintable_index];
                break;
            default:
                break;
        }

        // elem_index1 = k_i; 
        elem_index2 = elem_index1 + bf_pair_interval;

        // 注意这里的复数是cosα - jsinα
        complex[elem_index1].r += (W_N_ki_sin * complex[elem_index2].i + W_N_ki_cos * complex[elem_index2].r) >> 14;
        complex[elem_index1].i += (W_N_ki_cos * complex[elem_index2].i - W_N_ki_sin * complex[elem_index2].r) >> 14;

        // 每一个W_N_k乘子对应的值都包含在一个蝶形运算对里面
        // complex[elem_index1].r = complex[elem_index1].r + complex_temp.r;
        // complex[elem_index1].i = complex[elem_index1].i + complex_temp.i;
        // complex[elem_index2].r = complex[elem_index1].r - (complex_temp.r << 1);// - complex[factor_index].r;
        // complex[elem_index2].i = complex[elem_index1].i - (complex_temp.i << 1);//- complex[factor_index].i;
    }
}

uint16_t mytest_fft_get_freq(void)
{
    if(++fft_status.res_index > 2) fft_status.res_index = 0;
    return fft_status.freq[fft_status.res_index];
}

uint16_t mytest_fft_get_magnitude(void)
{
    return fft_status.magnitude[fft_status.res_index];
}
