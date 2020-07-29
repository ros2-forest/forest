#include "ap_cint.h"

void comp_sum(
		uint8 int_list[10],
		int sum_val[1]) {
#pragma HLS INTERFACE s_axilite port=return
#pragma HLS INTERFACE s_axilite port=sum_val
#pragma HLS INTERFACE s_axilite port=int_list

	int sum=0;
	for (int i=0; i < 10; i++) {
#pragma HLS PIPELINE
		sum+=int_list[i];
	}
	sum_val[0]=sum;

}
