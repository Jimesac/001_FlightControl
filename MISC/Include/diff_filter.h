#ifndef DIFF_FILTER_H
#define DIFF_FILTER_H

#include <stdlib.h>
#include <stdint.h>

extern float diff_smooth_coedf_15[];
extern float diff_smooth_coedf_10[];
extern float diff_smooth_coedf_9[];
extern float diff_smooth_coedf_8[];
extern float diff_smooth_coedf_7[];
extern float diff_smooth_coedf_6[];
extern float diff_smooth_coedf_5[];
extern float diff_smooth_coedf_4[];
extern float diff_smooth_coedf_3[];
extern float diff_smooth_coedf_2[];


extern float diff_hybird_coedf_15[];
extern float diff_hybird_coedf_10[];
extern float diff_hybird_coedf_9[];
extern float diff_hybird_coedf_8[];
extern float diff_hybird_coedf_7[];
extern float diff_hybird_coedf_6[];
extern float diff_hybird_coedf_5[];
extern float diff_hybird_coedf_4[];
extern float diff_hybird_coedf_3[];

// 微分滤波器参数
typedef struct
{
	uint8_t order;
	float *coef;
	
	float *input;
	float input_coef;
	
	float time_coef;
} differential_filter_t;



#endif   // DIFF_FILTER_H
