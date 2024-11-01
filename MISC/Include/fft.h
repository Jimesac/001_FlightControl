#ifndef __FFT_TEST_H__
#define __FFT_TEST_H__

#include "stdint.h"

// Fixed-point data type
typedef int16_t fft_t;

// Complex number type
typedef struct {
  fft_t r, i;
} fft_complex_t;


void fft_test(void);

void mytest_fft_update(void);

uint16_t mytest_fft_get_freq(void);
uint16_t mytest_fft_get_magnitude(void);
#endif

