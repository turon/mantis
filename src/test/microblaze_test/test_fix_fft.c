
/*
  Short test program to accompany fix_fft.c
*/

#define DEBUG 1
#define SPECTRUM 0

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "fix_fft.h"

#define FFT_SIZE  64
#define log2FFT   6
#define N         (2 * FFT_SIZE)
#define log2N     (log2FFT + 1)
#define FREQUENCY 5
#define AMPLITUDE 12288
#define TOTAL_CHARS 20
extern void show_amp(int index, int i, unsigned int amp);

void do_fft_test()
{
	int i, scale;
	unsigned diff;
	short x[N], fx[N];

    printf("Samples: \n");

	for (i=0; i<N; i++){
        /*
		x[i] = AMPLITUDE*cos(i*FREQUENCY*(2*3.1415926535)/N);
		if (i & 0x01)
			fx[(N+i)>>1] = x[i];
		else
			fx[i>>1] = x[i];
        */
#if DEBUG
        show_amp(i,x[i], AMPLITUDE);
#endif
	}

	fix_fftr(fx, log2N, 0);
printf("----------------------------------------------\n");
printf("-  Frequencies                               -\n");
printf("----------------------------------------------\n");
for (i=0; i<N/2; i++) show_amp(i, fx[i], AMPLITUDE);


#if SPECTRUM
for (i=0; i<N/2; i++) show_amp(i, fx[i], AMPLITUDE);
return;
#endif
    printf("----------------------------------------------\n");
    printf("-  Scaled                                    -\n");
    printf("----------------------------------------------\n");
	scale = fix_fftr(fx, log2N, 1);
//fprintf(stderr, "scale = %d\n", scale);

	for (i=0,diff=0; i<N; i++) {
		int sample;
		if (i & 0x01)
			sample = fx[(N+i)>>1] << scale;
		else
			sample = fx[i>>1] << scale;
#if DEBUG
        show_amp(i, sample, AMPLITUDE);
#endif
		diff += abs(x[i]-sample);
	}
//	fprintf(stderr, "sum(abs(diffs)))/N = %g\n", diff/(double)N);

	return;
}

