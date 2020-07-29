#include "ap_cint.h"
#include "img_hist.h"

void get_hist(
		uint8 image[N_ROWS*N_COLS],
		int hist[256]) {
#pragma HLS INTERFACE s_axilite port=return
#pragma HLS INTERFACE s_axilite port=hist
#pragma HLS INTERFACE s_axilite port=image

	int i=0;

	for (i=0; i < 256; i++) {
#pragma HLS PIPELINE
		hist[i] = 0;
	}

	for (i=0; i < N_ROWS*N_COLS; i++) {
#pragma HLS PIPELINE
		hist[image[i]]++;
	}
}
