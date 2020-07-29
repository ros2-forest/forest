#include "ap_int.h"
#include "hls_stream.h"
#include "ap_axi_sdata.h"
#include "img_hist_stream.h"

typedef ap_axiu<8,1,1,1> AXI_UINT8;
typedef hls::stream<AXI_UINT8> STREAM_UINT8;

void get_hist_stream(
		STREAM_UINT8 &image,
		int hist[256]) {
#pragma HLS INTERFACE s_axilite port=return
#pragma HLS INTERFACE axis port=image
#pragma HLS INTERFACE s_axilite port=hist

	for (int i=0; i<256; i++) {
#pragma HLS PIPELINE
		hist[i]=0;
	}

	for (int i=0; i < N_ROWS*N_COLS; i++) {
#pragma HLS PIPELINE
		AXI_UINT8 image_stream = image.read();
		hist[image_stream.data]++;
	}
}
