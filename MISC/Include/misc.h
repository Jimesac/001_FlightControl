#ifndef REMO_MISC_H
#define REMO_MISC_H

#include <stdint.h>
#include <stdbool.h>

#define RAD_TO_DEG 57.29578f
#define DEG_TO_RAD 0.0174533f

typedef struct
{
    float amplitude;
    float slope;
    float offset;
}nonlinear_unit_t;

typedef struct
{
    float sin;
    float cos;
} trig_func_f_t;


float Q_rsqrt( float number );
float remo_misc_constrainf(float a, float lower, float upper);
int16_t remo_misc_constrain_i16(int16_t a, int16_t lower, int16_t upper);

void remo_misc_matrix_multiply(uint8_t aRows, uint8_t aCols_bRows, uint8_t bCols, float matrixC[], \
                               const float matrixA[], const float matrixB[]);
float remo_misc_constrainf(float a, float lower, float upper);
uint8_t remo_misc_matrix_inversion(float *A, int n, float *AInverse);
void nonlinear_unit_init(float of, float am, float sl, nonlinear_unit_t *nl_unit);
float nonlinear_func(nonlinear_unit_t nl_uint, float input_x);
float safe_asinf(float x);
float Q_rsqrt( float number );
float Rajan_FastArcTan(float x);
float Rajan_FastArcTan2(float y, float x);
void remo_rand(uint8_t row, uint8_t column, float *rand_matrix);

float chirp_signal_generator(uint16_t amp, uint16_t sample_freq, float start_freq_hz, \
                            uint16_t time1_ms, float tmie1_freq_hz, uint16_t count);

void Ellipsoid_LSQ_4p(const int16_t *data_x, const int16_t *data_y, uint16_t len, float *params);

float remo_misc_vector_norm(const float *data, uint8_t len);
#endif // REMO_MISC_FP_H
