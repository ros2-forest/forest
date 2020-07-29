#include "ap_cint.h"

void add(
		int a,
		int b,
		int c[1]) {
#pragma HLS INTERFACE s_axilite port=a
#pragma HLS INTERFACE s_axilite port=b
#pragma HLS INTERFACE s_axilite port=c
#pragma HLS INTERFACE s_axilite port=return

c[0]= a+b;

}
