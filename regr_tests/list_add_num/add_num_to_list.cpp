#include "ap_int.h"
#include "hls_stream.h"
#include "ap_axi_sdata.h"

typedef ap_axiu<8,1,1,1> AXI_UINT8;
typedef hls::stream<AXI_UINT8> STREAM_UINT8;
typedef ap_uint<8> UINT8;

void add_num_to_list(
		UINT8 num_to_add,
		int list[5],
		STREAM_UINT8 &result
		) {
#pragma HLS INTERFACE s_axilite port=return
#pragma HLS INTERFACE axis port=result
#pragma HLS INTERFACE s_axilite port=list
#pragma HLS INTERFACE s_axilite port=num_to_add

	for (int i=0; i < 5; i++) {
#pragma HLS PIPELINE
		AXI_UINT8 result_stream;
		result_stream.data = list[i] + num_to_add;
		result_stream.keep=1;
		result_stream.strb=1;
		result_stream.user=1;
		result_stream.id=0;
		result_stream.dest=0;
		if (i < 4) {
			result_stream.last=0;
		} else {
			result_stream.last=1;
		}
		result.write(result_stream);
	}
}
