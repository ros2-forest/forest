#include "ap_int.h"
#include "hls_stream.h"
#include "ap_axi_sdata.h"
#include "contrast_stretch.h"

typedef struct {
	ap_uint<64> data;
	ap_uint<1> last;
} AXI_UINT64_STRUCT;
typedef hls::stream<AXI_UINT64_STRUCT> STREAM_UINT64;
typedef ap_uint<8> uint8;
typedef ap_uint<15> uint15;
typedef ap_uint<64> uint64;

void do_stretch(
		STREAM_UINT64 &image_in,
		STREAM_UINT64 &image_out) {
#pragma HLS INTERFACE s_axilite port=return
#pragma HLS INTERFACE axis port=image_in
#pragma HLS INTERFACE axis port=image_out

	AXI_UINT64_STRUCT image[N_ROWS*N_COLS/8];
	// Images are 128x128, thus we need to store values that are at most
	// 128*128 = 2^14 in the hist array
	uint15 hist[256];
	uint15 cumulative_hist, low_perc_count, hi_perc_count;
	int i, j;
	int c = 0;
	int d = 255;
	uint8 data_in;
	uint8 data_out[8];
#pragma ARRAY_PARTITION variable=data_out complete
	uint64 in_str_data;
	uint64 out_str_data;

	// 1. Compute the histogram

	clean_hist: for (i=0; i<256; i++){
#pragma HLS PIPELINE
		hist[i] = 0;
	}

	build_hist: for (i=0; i < N_ROWS*N_COLS/8; i++) {
#pragma HLS PIPELINE
		AXI_UINT64_STRUCT image_in_str = image_in.read();
		write_hist: for (j = 0; j < 8; j++) {
			data_in = image_in_str.data.range((j+1)*8-1, j*8);
			hist[data_in]++;
		}
		image[i] = image_in_str;
	}

	// 2. Find the 1st and 99th percentile values

	low_perc_count = uint15((1 * N_ROWS*N_COLS)/100);
	hi_perc_count = uint15((99 * N_ROWS*N_COLS)/100);
	cumulative_hist = 0;
	find_c: for (i=0; i < 256; i++) {
#pragma HLS PIPELINE
		cumulative_hist = hist[i] + cumulative_hist;
		if (cumulative_hist > low_perc_count) {
			c = i;
			break;
		}
	}

	cumulative_hist = N_ROWS*N_COLS;
	find_d: for (i=255; i > 0; i--) {
#pragma HLS PIPELINE
		cumulative_hist = cumulative_hist - hist[i];
		if (cumulative_hist < hi_perc_count) {
			d = i;
			break;
		}
	}

	// 3. Do linear contrast stretching

	do_stretch: for (i=0; i < N_ROWS*N_COLS/8; i++) {
#pragma HLS PIPELINE
		out_str_data = image[i].data;
		scale: for (j = 0; j < 8; j++){
			data_out[j] = out_str_data.range((j+1)*8-1, j*8);
			if (data_out[j] < c) {
				data_out[j] = 0;
			}
			else if(data_out[j] > d) {
				data_out[j] = 255;
			}
			else{
				data_out[j] = ((data_out[j]-c)*255)/(d-c);
			}
		}
		image[i].data = (data_out[7], data_out[6], data_out[5], data_out[4], data_out[3], data_out[2], data_out[1], data_out[0]);
		image_out.write(image[i]);
	}
}
