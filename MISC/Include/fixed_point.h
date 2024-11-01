#ifndef REMO_FIXED_POINT_H
#define REMO_FIXED_POINT_H

#include <stdint.h>

#define SIN_TABLE_INPUT_ANGLE_Q7
// #define SIN_TABLE_INPUT_ANGLE_Q14

#define PTR_NULL  ((void*)0)

// 注意事项：
// 1. 两个int32变量x1、x2相乘得到一个int64y的值时至少要先将一个int32转换成int64，
//    如y = (int64)x1 * (int64)x2或y = (int64)x1 * x2，否则会出现错误。
// 2. 两个int变量x1,x2相乘后得到的数值y移位的操作下面第一种方式计算用时少：
//    ① y = (x1 * x2) >> N;
//    ② y = x1 * x2; y >>= N。
// 3. int类型求平方根没有专用的函数，用sqrtf函数也挺快的，1/sqrtf(x)比Q_rsqrt(x)要快,
//    int类型用sqrtf后要将计算结果强制转换成int类型，然后再用int类型去除，
//    sqrtf返回的是float类型，直接除的话就是float类型的除法，显然要比int类型除法要慢
// 4. 在一条指令行中int和uint相乘再移位必须要把uint先转换成int，不然都会当做uint相乘

#define CONST_1_Q14          (16384)        // 1 * 2^14 = 16384
#define CONST_2_Q14          (32768)        // 2 * 2^14 = 16384
#define CONST_3_Q14          (49152)        // 3* 2^14 = 16384
#define CONST_1_SQUARE_Q14   (268435456)    // 1 * 2^28 = 16384
#define CONST_NEG_1_Q14      ((-16384))       // -1 * 2^14 = -16384
#define Q14_SHIFT_BIT_NUM    (14)         // 整数转换为q14格式需要的移位数
#define Q14_SQRT_SHIFT_BIT_NUM  (7)     // q14格式数据开平方根之后需要的移位数
#define Q14_ROUND_VALUE      (8192)         // 2^13 = 8192, q14格式数据归一化后由截断误差变为四舍五入

#define CONST_1_Q7          (128)
#define CONST_2_Q7          (256)
#define CONST_1_SQUARE_Q7   (16384)
#define Q7_SHIFT_BIT_NUM    (7)          // 整数转换为q7格式需要的移位数；主要用在控制器中，精度精确到1/2^7 = 0.0078
#define Q7_ROUND_VALUE      (64)         // 2^6 = 64, q7格式数据归一化后由截断误差变为四舍五入

#define CONST_1_Q10          (1024)
#define CONST_1_Q23          (8388608)        // 1 * 2^23 = 16384
#define Q4_SHIFT_BIT_NUM     (0)          // 整数转换为q4格式需要的移位数
#define Q10_SHIFT_BIT_NUM    (10)         // 整数转换为q10格式需要的移位数
#define Q17_SHIFT_BIT_NUM    (17)         // 整数转换为q17格式需要的移位数
#define Q23_SHIFT_BIT_NUM    (23)         // 整数转换为q23格式需要的移位数；
                                            // 主要用于姿态更新中，静止不不动时一个更新周期内角度变化量很小很小，需要高精度表示
#define Q14_TO_Q17_SHIFT_BIT_NUM   (Q17_SHIFT_BIT_NUM - Q14_SHIFT_BIT_NUM)   // q14转换成q23需要的移位数
#define Q14_TO_Q23_SHIFT_BIT_NUM   (Q23_SHIFT_BIT_NUM - Q14_SHIFT_BIT_NUM)   // q14转换成q23需要的移位数
#define Q14_TO_Q23_ROUND_VALUE     (256)   // 由q14到q23运算由截断误差变为四舍五入需要的值
#define Q14_TO_Q7_SHIFT_BIT_NUM    (Q14_SHIFT_BIT_NUM - Q7_SHIFT_BIT_NUM)    // q14转换成q7需要的移位数
#define Q14_TO_Q7_ROUND_VALUE     (64)    // 由q7到q14运算比例系数,2^(14-7-1)
#define Q14_TO_Q7_SCALE_VALUE     (128)    // 由q7到q14运算比例系数,2^(14-7)
#define Q14_TO_Q10_SHIFT_BIT_NUM   (Q14_SHIFT_BIT_NUM - Q10_SHIFT_BIT_NUM)   // q14转换成q10需要的移位数

// q14格式数据归一化需要用到的数值1 * 2^14 * 2^x，保证值的小数后一位的精度0.1*2^3 >1,所以x = 14+3=17
#define Q14_NORM_VALUE            (2147483648)    // 1 * 2^14 * 2^17 = 2147483648
#define Q14_NORM_SHIFT_BIT_NUM    (17)          // 17

#define Q12_TO_Q14_SHIFT_BIT_NUM   (2)  // 14-12
#define Q12_TO_Q14_SCALE_VALUE     (4)  // 2^2
#define Q12_TO_Q14_ROUND_VALUE     (2)  // 2^2 / 2

// 角度值的q14数据格式
#define DEG_360_Q14    (5898240)    // 360 * 2^14 = 5898240
#define DEG_180_Q14    (2949120)    // 180 * 2^14 = 2949120
#define DEG_90_Q14     (1474560)    // 90 * 2^14 = 1474560
#define DEG_135_Q14    (2211840)    // 135 * 2^14 = 2211840
#define DEG_75_Q14     (1228800)    // 75 * 2^14 = 1228800
#define DEG_60_Q14     (983040)     // 60 * 2^14 = 983040
#define DEG_45_Q14     (737280)     // 45 * 2^14 = 737280
#define PI1_4_Q14  (12868)   // PI1_4 * 2^14 = 12868
#define PI1_2_Q14  (25736)   // PI1_2 * 2^14 = 25736
#define PI_Q14     (51472)   // PI * 2^14 = 51472
#define PI2_Q14    (102944)  // PI2 * 2^14 = 102944

#define DEG_360_Q7    ((uint16_t)46080)    // 360 * 2^7 = 46080
#define DEG_200_Q7    ((uint16_t)25600)    // 200 * 2^7 = 25600
#define DEG_180_Q7    ((uint16_t)23040)    // 180 * 2^7 = 23040
#define DEG_90_Q7     ((uint16_t)11520)    // 90 * 2^7 = 11520
#define DEG_82_Q7     ((uint16_t)10496)    // 82 * 2^7 = 10496
#define DEG_80_Q7     ((uint16_t)10240)    // 80 * 2^7 = 10240
#define DEG_76_Q7     ((uint16_t)9728)    // 80 * 2^7 = 10240

#define DEG_360_Q10    ((int32_t)368640)    // 360 * 2^10 = 368640
#define DEG_180_Q10    ((int32_t)184320)    // 180 * 2^10 = 184320

#define DEG_360_Q13    ((int32_t)2949120)    // 360 * 2^13 = 2949120
#define DEG_180_Q13    ((int32_t)1474560)    // 180 * 2^13 = 1474560

// 角度之间的单位转换
#define RAD_TO_DEG_Q14  (938734)   // 180/PI * 2^14 = 938734
#define DEG_TO_RAD_Q14  (286)      // PI/180 * 2^14 = 286

// 百分数转换为q7格式，128/100 = 41/2^5
#define CENT_TO_Q7_SCALE_VALUE            ((uint8_t)41)
#define CENT_TO_Q7_ROUND_VALUE            ((uint8_t)16)
#define CENT_TO_Q7_SHIFT_BIT_NUM          ((uint8_t)5)   
// q7格式转换为百分数，100/128 = 25/2^5
#define Q7_TO_CENT_SCALE_VALUE            ((uint8_t)25)
#define Q7_TO_CENT_ROUND_VALUE            ((uint8_t)16)
#define Q7_TO_CENT_SHIFT_BIT_NUM          ((uint8_t)5)   

// 百分数转换为q14格式，163874/100 = 1311/2^3
#define Q14_CENT_VALUE            (1311)
#define Q14_CENT_SHIFT_BIT_NUM    (3)   

// 0~90°的插值区间被细分成2^TRIG_INTERP_0TO90_DIVISION_SBN份
#define TRIG_INTERP_0TO90_DIVISION_SBN          ((uint16_t)14)
// 0~360°被细分在0~90°中的mask
#define TRIG_INTERP_0TO90_DIVISION_INDEX_MASK   ((uint16_t)(1 << TRIG_INTERP_0TO90_DIVISION_SBN) - 1) 
// sintable中插值比例所用到移位数
#define TRIG_INTERP_0TO90_LINEAR_SBN            ((uint16_t)(TRIG_INTERP_0TO90_DIVISION_SBN - SIN_TABLE_SBN))
// sintable中插值比例所用到的mask
#define TRIG_INTERP_0TO90_LINEAR_MASK           ((uint16_t)(1 << TRIG_INTERP_0TO90_LINEAR_SBN) - 1)
// 0~360°在0~90°sintable中的mask
#define SIN_0TO90_INDEX_MASK           ((uint16_t)(1 << 8) - 1)    // table的长度为2^8


typedef int32_t  int32_q7_t;
typedef uint32_t uint32_q7_t;
typedef int32_t  int32_q14_t;
typedef uint32_t uint32_q14_t;
typedef int32_t  int32_q17_t;
typedef uint32_t uint32_q17_t;
typedef int32_t  int32_q20_t;
typedef int32_t  int32_q23_t;
typedef int32_t  int32_q10_t;
typedef uint32_t uint32_q10_t;

typedef int16_t  int16_q7_t;
typedef uint16_t uint16_q7_t;
typedef uint16_t uint16_q10_t;
typedef uint16_t uint16_q17_t;
typedef int16_t  int16_q14_t;
typedef uint16_t uint16_q14_t;

typedef int16_t  int16_q4_t;
typedef int64_t  int64_q14_t;

typedef struct
{
    int16_q14_t sin;
    int16_q14_t cos;
} trig_func_q14_t;


#ifdef SIN_TABLE_INPUT_ANGLE_Q14 
void remo_trig_func(int32_q14_t angle_deg, trig_func_t *trig);
#elif defined(SIN_TABLE_INPUT_ANGLE_Q7)
void remo_trig_func(int16_q7_t angle_deg, trig_func_q14_t *trig);
#endif

const int16_t *fp_get_sintable(void);

#endif // REMO_FIXED_POINT_H
