#include "ap_cint.h"

void mult_float_to_list(
		double double_num,
		double double_list_in[3],
		double double_list_out[3]) {
#pragma HLS INTERFACE s_axilite port=return
#pragma HLS INTERFACE s_axilite port=double_num
#pragma HLS INTERFACE s_axilite port=double_list_in
#pragma HLS INTERFACE s_axilite port=double_list_out

	int i=0;

	for (i=0; i < 3; i++) {
#pragma HLS PIPELINE
		double_list_out[i] = double_num * double_list_in[i];
	}

}
