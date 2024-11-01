#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include "fixed_point.h"
#include "fixed_point_table.h"

// 角度和角速度定标采用int32,Q14的格式, 值范围为-131072~131071.999938965，
// 值的精度为1/2^14 = 0.00006103515625, 小于0.0001, 满足一些函数中参数值的精度要求

// 三角函数查表用的
#define SIN_MASK 0x0300
#define U0_90 0x0000
#define U90_180 0x0100
#define U180_270 0x0200
#define U270_360 0x0300

// 反正弦查表用
#define SIN_DEG_40_Q14 10531   // sin(40/180*PI) * 2^14 = 10531
#define SIN_DEG_70_Q14 15396   // sin(70/180*PI) * 2^14 = 15396
#define SIN_DEG_90_Q14 16384   // sin(90/180*PI) * 2^14 = 16384


/**************************************************************************************************
 * 函数名称: remo_trig_functions
 * 输入参数: angle，采用的是14/7位小数，角度范围是-180~180度
 * 返回结果: trig，用来返回计算结果
 * 功能描述: 三角函数查表，三角函数定标采用的是s32.14
**************************************************************************************************/
#ifdef SIN_TABLE_INPUT_ANGLE_Q14 
void remo_trig_func(int32_q14_t angle_deg, trig_func_t *trig)
#elif defined(SIN_TABLE_INPUT_ANGLE_Q7)
void remo_trig_func(int16_q7_t angle_deg, trig_func_q14_t *trig)
#endif
{
    uint16_t index = 0;
    uint16_t range = 0;

	uint32_t angle_deg_temp = 0;

    if (trig == (void *)0)
    {
        return;
    }

#ifdef SIN_TABLE_INPUT_ANGLE_Q14 
#if SIN_TABLE_SIZE == 256 
    // angle / 2^14 * 1024 / 360 = angle * 91 / 2^19
    // 修正成0~1024，其中0~90:0~255，90~180:256~511，180~270:512~767，270~360:768~1023
	if (angle_deg < 0)
    {
        angle_deg += DEG_360_Q14;
    }
    index = (uint32_t)(((angle_deg) * 91) >> 19);
    range = index & SIN_MASK;
#endif
#elif defined(SIN_TABLE_INPUT_ANGLE_Q7)
#if SIN_TABLE_SIZE == 256 
    // angle / 2^7 * 1024 / 360 = angle * 91 / 2^12
    // 修正成0~1024，其中0~90:0~255，90~180:256~511，180~270:512~767，270~360:768~1023
	angle_deg_temp = (angle_deg < 0) ? ((int32_t)angle_deg + DEG_360_Q7):(angle_deg);
    index = (angle_deg * 91) >> 12;
    range = index & SIN_MASK;
#endif
#endif

    index = index & 0x00FF;

    switch (range)
    {
    case U0_90:
        trig->sin = (int32_q14_t)(sin_table_q14[index]); // 强制类型转换，只保留低8位
        trig->cos = (int32_q14_t)(sin_table_q14[(0xff - index)]);
        break;

    case U90_180:
        trig->sin = (int32_q14_t)(sin_table_q14[(0xff - index)]);
        trig->cos = (int32_q14_t)(-sin_table_q14[index]);
        break;

    case U180_270:
        trig->sin = (int32_q14_t)(-sin_table_q14[index]);
        trig->cos = (int32_q14_t)(-sin_table_q14[(0xff - index)]);
        break;

    case U270_360:
        trig->sin = (int32_q14_t)(-sin_table_q14[(0xff - index)]);
        trig->cos = (int32_q14_t)(sin_table_q14[index]);
        break;

    default:
        break;
    }
}

const int16_t *fp_get_sintable(void)
{
    return sin_table_q14;
}
