#include <math.h>
#include <stdlib.h>
#include "misc.h"

/**************************************************************************************************
 * 函数名称: remo_misc_matrix_multiply
 * 输入参数: 
 * 返回结果: void
 * 功能描述: 矩阵乘法 matrixC      =     matrixA        X      matrixB
 *                aRows x bCols   aRows x aCols_bRows    aCols_bRows x bCols
**************************************************************************************************/
void remo_misc_matrix_multiply(uint8_t aRows, uint8_t aCols_bRows, uint8_t bCols, float matrixC[], \
							   const float matrixA[], const float matrixB[])
{
	uint8_t i, j, k;

	for (i = 0; i < aRows * bCols; i++)
	{
		matrixC[i] = 0.0;
	}

	for (i = 0; i < aRows; i++)
	{
		for (j = 0; j < aCols_bRows; j++)
		{
			for (k = 0; k < bCols; k++)
			{
				matrixC[i * bCols + k] += matrixA[i * aCols_bRows + j] * matrixB[j * bCols + k];
			}
		}
	}
}

/**************************************************************************************************
 * 函数名称: remo_misc_constrainf
 * 输入参数: a->输入值，lower->限幅下界，upper->限幅上界
 * 返回结果: 限幅结果
 * 功能描述: 对输入值进行上下边界的限幅
**************************************************************************************************/
float remo_misc_constrainf(float a, float lower, float upper)
{
	if (a < lower)
	{
		return lower;
	}
	else if (a > upper)
	{
		return upper;
	}
	else
	{
		return a;
	}
}

int16_t remo_misc_constrain_i16(int16_t a, int16_t lower, int16_t upper)
{
	if (a < lower)
	{
		return lower;
	}
	else if (a > upper)
	{
		return upper;
	}
	else
	{
		return a;
	}
}

/**************************************************************************************************
 * 函数名称: remo_misc_matrix_inversion
 * 输入参数: A->输入值，n->方阵维数，AInverse->逆矩阵
 * 返回结果: 求逆是否成功
 * 功能描述: 采用高斯消元法对方阵A求逆得到逆矩阵AInverse
**************************************************************************************************/
uint8_t remo_misc_matrix_inversion(float* A, int n, float* AInverse)
{
	int i, j, iPass, imx, icol, irow;
  	float det, temp, pivot, factor;
  	float* ac = (float*)malloc(n*n*sizeof(float));
	if(ac == NULL)
	{
		return 0;
	}
  	det = 1;
  	for(i = 0; i < n; i++)
  	{
    	for (j = 0; j < n; j++)
    	{
      		AInverse[n*i+j] = 0;
      		ac[n*i+j] = A[n*i+j];
    	}
    	AInverse[n*i+i] = 1;
  	}
  	for(iPass = 0; iPass < n; iPass++)
  	{
    	imx = iPass;
    	for (irow = iPass; irow < n; irow++)
    	{
      		if (fabs(A[n*irow+iPass]) > fabs(A[n*imx+iPass])) imx = irow;
    	}
    
    
		if (imx != iPass)
	    {
			for (icol = 0; icol < n; icol++)
	      	{
	        	temp = AInverse[n*iPass+icol];
	        	AInverse[n*iPass+icol] = AInverse[n*imx+icol];
	        	AInverse[n*imx+icol] = temp;
	        	if (icol >= iPass)
	        	{
	          		temp = A[n*iPass+icol];
	          		A[n*iPass+icol] = A[n*imx+icol];
	          		A[n*imx+icol] = temp;
	        	}
	      	}
	    }
	    
	    pivot = A[n*iPass+iPass];
	    det = det * pivot;
	    if (det == 0)
	    {
			free(ac);
	      	return 0;
	    }
	    
	    for (icol = 0; icol < n; icol++)
	    {
	      
			AInverse[n*iPass+icol] = AInverse[n*iPass+icol] / pivot;
	      	if (icol >= iPass) A[n*iPass+icol] = A[n*iPass+icol] / pivot;
	    }
	    
	    for (irow = 0; irow < n; irow++)
	    {
	      
			if (irow != iPass) factor = A[n*irow+iPass];
	      	for (icol = 0; icol < n; icol++)
	      	{
	        	if (irow != iPass)
	        	{
	          		AInverse[n*irow+icol] -= factor * AInverse[n*iPass+icol];
	          		A[n*irow+icol] -= factor * A[n*iPass+icol];
	        	}
	      	}
	    }
  	}
  
  	free(ac);
  	return 1;
}

/**************************************************************************************************
 * 函数名称: ctrl_pid_set_nonlinear_unit
 * 输入参数: am->S型函数幅值；sl->S型函数非线性的严重程度；of->S型函数偏移。
 *           am控制非线性增减，大于0为增函数，小于0为减函数，等于0为常值
 *           最后函数输出值必须大于0，即am + of >= 0
 * 返回结果: void
 * 功能描述: 非线性参数
 *          y = offset - amplitude / (1 + slop *x*x)，初始值为offset-amplitude，终值为offset
**************************************************************************************************/
void nonlinear_unit_init(float of, float am, float sl, nonlinear_unit_t *nl_unit)
{
    nl_unit->offset = (of >= 0) ? of : nl_unit->offset;
    nl_unit->amplitude = ((of - am) <= 0) ? 0 : am;
    nl_unit->slope = (sl >= 0) ? sl : nl_unit->slope;
}


/**************************************************************************************************
 * 函数名称: remo_fp_nonlinear_func_
 * 输入参数: nl_uint->非线性系数，input_x->输入
 * 返回结果: 非线性值
 * 功能描述: 计算q14格式非线性参数 y = offset - amplitude / (1.0 + slope * x * x)
 * 			x越大，y越大，最大为offset；x越小，y越小，最小为offset-amplitude
**************************************************************************************************/
inline float nonlinear_func(nonlinear_unit_t nl_uint, float input_x)
{
    return nl_uint.offset - nl_uint.amplitude / (1.0f + nl_uint.slope * input_x * input_x); // offset - amplitude / (1.0 + slope * x * x)
}

/**************************************************************************************************
 * 函数名称: Q_rsqrt
 * 输入参数: number->输入
 * 返回结果: void
 * 功能描述: 快速求开平方倒数
**************************************************************************************************/
float Q_rsqrt( float number )
{
	int32_t i;
	float x2, y;
	const float threehalfs = 1.5F;

	x2 = number * 0.5F;
	y  = number;
	i  = * ( int32_t * ) &y;            // evil floating point bit level hacking
	i  = 0x5f3759df - ( i >> 1 ); // what the fuck?
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
	//y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed

	return y;
}


inline float Rajan_FastArcTan(float x) {
  return 0.785398f*x - x*(fabs(x) - 1)*(0.2447 + 0.0663*fabs(x));
}


//inline float Rajan_FastArcTan2(float y, float x) {

//  uint8_t qCode;
//  const float pi_2 = 1.570796f;
//  float q;
//  float z;
//  float fabsx = fabs(x);
//  float fabsy = fabs(y);

//  // 6 us
//  uint8_t swap45 = (fabs(y) > fabs(x));
    
//  // 22us
//  if ((y >= 0) && (x >= 0)) { qCode = 0; }
//  if ((y >= 0) && (x <= 0)) { qCode = 1; }
//  if ((y <= 0) && (x <= 0)) { qCode = 2; }
//  if ((y <= 0) && (x >= 0)) { qCode = 3; }

//  // 54 us
//  if (swap45) {
//    q = x / y;
//  } else {
//    q = y / x;
//  }

//  // 92 us
//  z = Rajan_FastArcTan(q);

//  if (swap45) {
//    switch (qCode) {
//      case 0: z = 1.570796 - z;  break;
//      case 1: z = 1.570796 - z;  break;
//      case 2: z = -1.570796 - z; break;
//      case 3: z = -1.570796 - z; break;
//    }
//  } else {
//    switch (qCode) {    
//      case 0: z = z;         break;
//      case 1: z = 3.14159265 + z;    break;
//      case 2: z = -3.14159265 + z;   break;
//      case 3: z = z;         break;
//    }
//  }
  
//  return z;
//}

inline float Rajan_FastArcTan2(float y, float x) {

  uint8_t qCode;
  float q, fabsq;
  float z;
  float fabsx = fabs(x);
  float fabsy = fabs(y);

  // 6 us
  uint8_t swap45 = (fabsy > fabsx);
    
  // 22us
  if ((y >= 0) && (x >= 0)) { qCode = 0; }
  else if ((y >= 0) && (x <= 0)) { qCode = 1; }
  else if ((y <= 0) && (x <= 0)) { qCode = 2; }
  else if ((y <= 0) && (x >= 0)) { qCode = 3; }

  // 54 us
  if (swap45) {
    q = x / y;
    fabsq = fabs(q);
    z = 0.785398f*q - q*(fabsq - 1)*(0.2447f + 0.0663f*fabsq);
    switch (qCode) {
      case 0: z = 1.570796f - z;  break;
      case 1: z = 1.570796f - z;  break;
      case 2: z = -1.570796f - z; break;
      case 3: z = -1.570796f - z; break;
    }
  } else {
    q = y / x;
    fabsq = fabs(q);
    z = 0.785398f*q - q*(fabsq - 1)*(0.2447f + 0.0663f*fabsq);
    switch (qCode) {    
      case 0: z = z;         break;
      case 1: z = 3.14159265f + z;    break;
      case 2: z = -3.14159265f + z;   break;
      case 3: z = z;         break;
    }
  }
  
  return z;
}




/**************************************************************************************************
 * 函数名称: remo_safe_asinf
 * 输入参数: x：输入
 * 返回结果: float
 * 功能描述: 防止输入非法值产生错误
**************************************************************************************************/
float safe_asinf(float x)
{
	if(x > 1.0f)
	{
		x = 1.0f;
	}
	else if(x < -1.0f)
	{
		x = -1.0f;
	}
	
	return asinf(x);
}

/**************************************************************************************************
 * 函数名称: remo_rand_q14
 * 输入参数: row->矩阵行，column->矩阵列，rand_M->随机数矩阵
 * 返回结果: void
 * 功能描述: 随机生成一个0~1之间的row * column的随机矩阵
 * **************************************************************************************************/
void remo_rand(uint8_t row, uint8_t column, float *rand_matrix)
{
	uint16_t length = row * column;
	uint16_t i = 0;	

	for(i = 0; i < length; i++)
	{
		rand_matrix[i] = ((uint16_t)rand());
	}
}

/**************************************************************************************************
 * 函数名称: sys_indent_chirp_signal_generator
 * 输入参数: amp->扫描信号幅值，sample_freq->采样频率，sample_num->采样数量，start_freq_hz->开始频率,t = 0，
 *          time1_ms/tmie1_freq_hz->t1频率和对应时间，sample_data->采样点
 * 返回结果: void
 * 功能描述: 生成扫频信号
 *          x(t) = A*cos(2*π*(k/2*t + f0)*t), k = (f1 - f0)/T
 * https://www.gaussianwaves.com/2014/07/chirp-signal-frequency-sweeping-fft-and-power-spectral-density-matlab-python/
**************************************************************************************************/
float chirp_signal_generator(uint16_t amp, uint16_t sample_freq, float start_freq_hz, \
                            uint16_t time1_ms, float tmie1_freq_hz, uint16_t count)
{
    float t = 0.0f;
    float k_half =  0.5f * (tmie1_freq_hz - start_freq_hz) * 1000 / (float)time1_ms;

    t = count / (float)sample_freq;
    return (amp * cosf(6.28318531f * (k_half * t + start_freq_hz) * t));
}

void Ellipsoid_LSQ_4p(const int16_t *data_x, const int16_t *data_y, uint16_t len, float *params)
{
	int i=0,j=0,k=0,sum=0,flag_inv;
	float XTX[4][4] = {0.0f};
	float XInv[4][4] = {0.0f};
	float XTF[4]={0.0f};
    float u[4] = {0.0f};
	float A=0.0f;
    float temp1 = 0.0f, temp2 = 0.0f;
    
    if (len < 4) return;
	
    return;
	//X=[x^2  x y^2 y]
    for(i=0; i<len; i++)
    {
        if (i%100 == 0) WDT_RefreshCounter();
        temp1 = data_x[i]*data_x[i];
        temp2 = data_y[i]*data_y[i];
        
        XTX[0][0] += temp1*temp1;
        XTX[0][1] += temp1*data_x[i];
        XTX[0][2] += temp1*data_y[i]*data_y[i];
        XTX[0][3] += temp1*data_y[i];
        
        XTX[1][1] += temp1;
        XTX[1][2] += data_x[i]*temp2;
        XTX[1][3] += data_x[i]*data_y[i];
        
        XTX[2][2] += temp2*temp2;
        XTX[2][3] += temp2*data_y[i];
        
        XTX[3][3] += temp2;
    }
    XTX[1][0] = XTX[0][1];
    XTX[2][0] = XTX[0][2];
    XTX[3][0] = XTX[0][3];
    XTX[2][1] = XTX[1][2];
    XTX[3][1] = XTX[1][3];
    XTX[3][2] = XTX[2][3];
    
    XTF[0] = XTX[1][1];
    XTF[2] = XTX[3][3];
    for(i=0; i<len; i++)
    {
        if (i%100 == 0) WDT_RefreshCounter();
        XTF[1] += data_x[i];
        XTF[3] += data_y[i];
    }
 
	flag_inv=remo_misc_matrix_inversion(&XTX[0][0], 4, &XInv[0][0]);   // inv(X'*X)
    WDT_RefreshCounter();
	if(flag_inv == 1)
    {
		u[0] = XInv[0][0]*XTF[0] + XInv[0][1]*XTF[1] + XInv[0][2]*XTF[2] + XInv[0][3]*XTF[3];
        u[1] = XInv[1][0]*XTF[0] + XInv[1][1]*XTF[1] + XInv[1][2]*XTF[2] + XInv[1][3]*XTF[3];
        u[2] = XInv[2][0]*XTF[0] + XInv[2][1]*XTF[1] + XInv[2][2]*XTF[2] + XInv[2][3]*XTF[3];
        u[3] = XInv[3][0]*XTF[0] + XInv[3][1]*XTF[1] + XInv[3][2]*XTF[2] + XInv[3][3]*XTF[3];
    }
	else return;
	
	A = 1+u[1]*u[1]/(4*u[0])+u[3]*u[3]/(4*u[2]);
	*(params )  = sqrt(u[0]/A);  
	*(params+1) = u[1]/(2**(params ));
	*(params+2) = sqrt(u[2]/A); 
	*(params+3) = u[3]/(2**(params+2));
}

/**************************************************************************************************
 * 函数名称: remo_misc_vector_norm
 * 输入参数: data->指向向量的数据指针；len->数据的长度
 * 返回结果: float
 * 功能描述: 计算数据data的平方和，然后开方并返回结果（欧几里得范数，常用于计算向量的长度）
**************************************************************************************************/
float remo_misc_vector_norm(const float *data, uint8_t len)
{
	uint8_t i;
	float sum = 0, norm;

	for (i = 0; i < len; i++)
	{
		sum += data[i] * data[i];
	}

	norm = sqrtf(sum);
	return norm;
}


