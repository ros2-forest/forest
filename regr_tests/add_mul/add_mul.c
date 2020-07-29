#include "ap_cint.h"

void add_mul(
		int a,
		int b,
		int c[2],
		double d,
		double e,
		double f[2]) {
#pragma HLS INTERFACE s_axilite port=f
#pragma HLS INTERFACE s_axilite port=e
#pragma HLS INTERFACE s_axilite port=e
#pragma HLS INTERFACE s_axilite port=d
#pragma HLS INTERFACE s_axilite port=a
#pragma HLS INTERFACE s_axilite port=b
#pragma HLS INTERFACE s_axilite port=c
#pragma HLS INTERFACE s_axilite port=return

c[0]=a+b;
c[1]=a*b;

f[0]=d+e;
f[1]=d*e;

}
