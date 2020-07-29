#include "ap_int.h"
#include "hls_stream.h"
#include "ap_axi_sdata.h"

typedef ap_axis<32,1,1,1> AXI_INT32;
typedef hls::stream<AXI_INT32> STREAM_INT32;

void get_stream_sum(
		STREAM_INT32 &inStream,
		STREAM_INT32 &outStream,
		int num_to_add) {
#pragma HLS INTERFACE s_axilite port=return
#pragma HLS INTERFACE axis port=inStream
#pragma HLS INTERFACE axis port=outStream
#pragma HLS INTERFACE s_axilite port=num_to_add

	int i = 0;

	for (i=0; i<100; i++) {
#pragma HLS PIPELINE
		AXI_INT32 dataIn = inStream.read();
		AXI_INT32 dataOut = dataIn;

		dataOut.data = dataIn.data + num_to_add;

		outStream.write(dataOut);
	}
}
