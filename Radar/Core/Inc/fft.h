#ifndef FFT_H_
#define FFT_H_
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <math.h>
#include <complex.h>

#ifdef __cplusplus
extern "C" {
#endif

//float PI;
typedef float complex cplx;
void fft(float complex data[], int n);
void _fft(float complex out[], int n, int step);
void rfft(uint16_t data[], float complex outI[], float complex outQ[], int size);
void fft_test(uint16_t data[], float complex out[], int size);
void show(const char *s, float complex buf[]);
void cplxtoAbs(float complex buf1[],float complex buf2[], float out1[],float out2[], int n);
void maxAbs(float complex buf1[], float complex buf2[], float out1[], float out2[], int *x, float *sum, int n);
#ifdef __cplusplus
}
#endif

#endif
