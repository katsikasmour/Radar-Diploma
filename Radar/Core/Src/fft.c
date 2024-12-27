#include "../Inc/fft.h"
#include "../Inc/utilities.h"
#include "main.h"


// Function to perform the bit reversal of the given input data
void bit_reverse(float complex data[], int n) {
    int j = 0;
    for (int i = 0; i < n; ++i) {
        if (i < j) {
            float complex temp = data[i];
            data[i] = data[j];
            data[j] = temp;
        }
        int m = n / 2;
        while (m >= 1 && j >= m) {
            j -= m;
            m /= 2;
        }
        j += m;
    }
}

// Optimized FFT function /// 21/07 CORRECT - CHECK WITH PYTHON SCRIPT
void fft(float complex data[], int n) {
    // Bit-reversal reordering
    bit_reverse(data, n);

    // FFT computation using the Cooley-Tukey algorithm
    for (int len = 2; len <= n; len *= 2) {
        int half_len = len / 2;
        float complex exp_base = cexp(-2.0 * I * M_PI / len);
        for (int i = 0; i < n; i += len) {
            float complex w = 1.0 + 0.0 * I;
            for (int j = 0; j < half_len; ++j) {
                float complex t = w * data[i + j + half_len];
                float complex u = data[i + j];
                data[i + j] = u + t;
                data[i + j + half_len] = u - t;
                w *= exp_base;
            }
        }
    }
}
//// Recursive FFT function
//void _fft(float complex data[], int n, int step) {
//    if (step >= n)
//        return;
//
//    // Perform the FFT on the even and odd indices
//    _fft(data, n, step * 2);
//    _fft(data + step, n, step * 2);
//
//    // Precompute the twiddle factors
//    for (int i = 0; i < n; i += 2 * step) {
//        float complex t = cexp(-I * M_PI * i / n) * data[i + step];
//        float complex u = data[i];
//
//        data[i] = u + t;
//        data[i + step] = u - t;
//    }
//}


//void _fft(float complex data[], int n, int step) {
//	if (step >= n)
//		return;
//
//	_fft(data, n, step * 2);
//	_fft(data + step, n, step * 2);
//
//	for (int i = 0; i < n; i += 2 * step) {
//		float complex t = cexp(-I * M_PI * i / n) * data[i + step];
//		data[i + step] = data[i] - t;
//		data[i] = data[i] + t;
//	}
//}

void rfft(uint16_t data[], float complex outI[], float complex outQ[],
		int size) {
	const float scale = 3.3 / 4095;
	int halfSize = size / 2;
	float complex *outI_ptr = outI;
	float complex *outQ_ptr = outQ;
	uint16_t *data_ptr = data;

	// Loop unrolling for better performance
	for (int i = 0; i < halfSize - 3; i += 4) {
		*outI_ptr++ = *data_ptr++ * scale + 0.0 * I;
		*outQ_ptr++ = *data_ptr++ * scale + 0.0 * I;

		*outI_ptr++ = *data_ptr++ * scale + 0.0 * I;
		*outQ_ptr++ = *data_ptr++ * scale + 0.0 * I;

		*outI_ptr++ = *data_ptr++ * scale + 0.0 * I;
		*outQ_ptr++ = *data_ptr++ * scale + 0.0 * I;

		*outI_ptr++ = *data_ptr++ * scale + 0.0 * I;
		*outQ_ptr++ = *data_ptr++ * scale + 0.0 * I;
	}

	// Handle remaining elements
	for (int i = (halfSize & ~3); i < halfSize; ++i) {
		outI[i] = data[2 * i] * scale + 0.0 * I;
		outQ[i] = data[2 * i + 1] * scale + 0.0 * I;
	}

	fft(outI, halfSize);
	fft(outQ, halfSize);
}

//void rfft(uint16_t data[], float complex outI[], float complex outQ[],
//		int size) {
//	for (int i = 0; i < size / 2; ++i) {
//		outI[i] = (3.3 * data[2 * i] / 4095) + 0.0 * I;
//		outQ[i] = (3.3 * data[2 * i + 1] / 4095) + 0.0 * I;
//	}
//
//	_fft(outI, size / 2, 1);
//	_fft(outQ, size / 2, 1);
//}

void cplxtoAbs(float complex buf1[], float complex buf2[], float out1[],
		float out2[], int n) {
	for (int i = 0; i < n; i++) {
		out1[i] = cabs(buf1[i]);
		out2[i] = cabs(buf2[i]);
//		out[i] = round(absolute * 100000.0) / 100000.0;
	}
}

void maxAbs(float complex buf1[], float complex buf2[], float out1[],
		float out2[], int *x, float *sum, int n) {
	float max = 0;
	*sum = 0;
	for (int i = 0; i < n; i++) {
		out1[i] = cabs(buf1[i]);
		out2[i] = cabs(buf2[i]);
		//clear abs_I[0] - DC VOLTAGE
		if (i == 0) {
			out1[i] = 0;
		}
		*sum = *sum + out1[i];
		if ((out1[i] > max) && (i != 0)) {
			max = out1[i];
			*x = i;
//			printf("max bin is: %d", x);
		}
	}
}

//void _fft(float complex out[], float complex tmp[], int n, int step) {
////	float PI = atan2(1, 1) * 4;
//
//	if (step < n) {
//		_fft(tmp, out, n, step * 2);
//		_fft(tmp + step, out + step, n, step * 2);
//
//		for (int i = 0; i < n; i += 2 * step) {
//			float complex t = cexp(-I * M_PI * i / n) * tmp[i + step];
//			out[i / 2] = tmp[i] + t;
//			out[(i + n) / 2] = tmp[i] - t;
//		}
//	}
//}

//void fft(float complex buf[], int n, int is_inverse) {
//	if (n <= 1)
//		return;
//
//	float complex even[n / 2];
//	float complex odd[n / 2];
//
//	for (int i = 0; i < n / 2; ++i) {
//		even[i] = buf[2 * i];
//		odd[i] = buf[2 * i + 1];
//	}
//
//	fft(even, n / 2, is_inverse);
//	fft(odd, n / 2, is_inverse);
//
//	float angle = (is_inverse ? 1 : -1) * 2 * M_PI / n;
//	float complex w = cexp(I * angle);
//
//	for (int k = 0; k < n / 2; ++k) {
//		float complex t = w * odd[k];
//		buf[k] = even[k] + t;
//		buf[k + n / 2] = even[k] - t;
//		w *= cexp(I * angle * (is_inverse ? -1 : 1));
//	}
//}

//float complex tmpI[SAMPLE_NUM];
//float complex tmpQ[SAMPLE_NUM];

//void rfft(uint16_t data[], float complex outI[], float complex outQ[], int size) {
//
//
//	for (int i = 0; i < size / 2; ++i) {
//		tmpI[i] = (3.3 * data[2 * i] / 4095) + 0.0 * I;
////		tmpI[i] = data[2 * i];
//		outI[i] = tmpI[i];
//		tmpQ[i] = (3.3 * data[2 * i + 1] / 4095) + 0.0 * I;
//		outQ[i] = tmpQ[i];
//	}
//
//	_fft(outI, tmpI, size / 2, 1);
//	_fft(outQ, tmpQ, size / 2, 1);
//
//}

//void fft_test(uint16_t data[], float complex out[], int size) {
//
//	float complex tmp[size];
//	for (int i = 0; i < size; ++i) {
//		out[i] = data[i] + 0.0 * I;
//		tmp[i] = data[i] + 0.0 * I;
//	}
//
//	_fft(out, tmp, size, 1);
//}
//void fft(float buf[], float complex outI[], float complex outQ[], int n) {
//	float complex tmpI[n / 2];
//	float complex tmpQ[n / 2];
//	int x = 0;
//	int j = 0;
//	for (int i = 0; i < n; i++) {
//		if (i % 4 == 0) {
//			tmpI[x] = 3.3 * buf[i] / 4095;
//			outI[x] = tmpI[x];
//			x++;
//		} else {
//			tmpQ[j] = 3.3 * buf[i] / 4095;
//			outQ[j] = tmpQ[j];
//			j++;
//		}
//
//	}
////	_fft(outI, tmpI, n / 2, 1);
////	_fft(outQ, tmpQ, n / 2, 1);
//}

void show(const char *s, float complex buf[]) {
	printf("%s", s);
	for (int i = 0; i < 8; i++)
		if (!cimag(buf[i]))
			printf("%g ", creal(buf[i]));
		else
			printf("(%g, %g) ", creal(buf[i]), cimag(buf[i]));
}
