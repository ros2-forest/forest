#include "mnist_bnn.h"

static uint1 conv1_krns[C1_KRN_SZ][C1_KRN_SZ][C1_N_KRNS] = {
	#include "c1_w.h"
};
static fixed18 bn_1_qp[2][C1_N_KRNS] = {
	#include "bn_1_qp.h"
};
static uint1 conv2_krns[C2_KRN_SZ][C2_KRN_SZ][C1_N_KRNS][C2_N_KRNS] = {
	#include "c2_w.h"
};
static fixed18 bn_2_qp[2][C2_N_KRNS] = {
	#include "bn_2_qp.h"
};
static uint1 conv3_krns[C3_KRN_SZ][C3_KRN_SZ][C2_N_KRNS][C3_N_KRNS] = {
	#include "c3_w.h"
};
static fixed18 bn_3_qp[2][C3_N_KRNS] = {
	#include "bn_3_qp.h"
};
static uint1 fc_1_w[F_OUT_SZ][FC1_SZ] = {
	#include "fc_1_w.h"
};
static fixed18 bn_4_qp[2][FC1_SZ] = {
	#include "bn_4_qp.h"
};
static uint1 fc_2_w[FC1_SZ][FC2_SZ] = {
	#include "fc_2_w.h"
};
static fixed18 bn_5_qp[2][FC2_SZ] = {
	#include "bn_5_qp.h"
};

void read_stream(STREAM_UINT8 &image_in, fixed18 image[BATCH_SZ][N_ROWS][N_COLS]){
	// Read the input stream and put the values in the image array
	// Apply quantization to image here

#pragma HLS INLINE off

	int b, r, c;
	uint8 data_in;
	fixed18 norm_data_in;

	batch_rd: for(b=0; b<BATCH_SZ; b++){
		row_rd: for (r=0; r<N_ROWS; r++) {
			col_rd: for (c=0; c<N_COLS; c++){
#pragma HLS PIPELINE
				data_in = image_in.read().data;
				norm_data_in = (fixed18)((data_in)/(127.5)) - 1;
				image[b][r][c] = norm_data_in;
			}
		}
	}

	return;
}

void conv1(fixed18 image[BATCH_SZ][N_ROWS][N_COLS], fixed18 conv1_out[BATCH_SZ][C1_OUT_N_ROWS][C1_OUT_N_COLS][C1_N_KRNS]) {
	// First conv layer. Filter consists of 32 3x3 kernels
	// Input shape: (1,28,28)
	// Output shape: (1,26,26,32)

#pragma HLS INLINE off

	int b, c1_x, c1_y, k, k_x, k_y;
	int stride = 1;
	fixed18 tmp_sum;
	fixed18 pos, neg;

	c1_batch: for (b=0; b<BATCH_SZ; b++){
		c1_row: for (c1_x=0; c1_x<C1_OUT_N_ROWS; c1_x++){
			c1_col: for (c1_y=0; c1_y<C1_OUT_N_COLS; c1_y++){
				c1_krn: for (k=0; k<C1_N_KRNS; k++) {
#pragma HLS PIPELINE
					conv1_out[b][c1_x][c1_y][k] = 0;
					c1_krn_x: for (k_x=0; k_x<C1_KRN_SZ; k_x++){
						c1_krn_y: for (k_y=0; k_y<C1_KRN_SZ; k_y++){
							pos=image[b][stride*c1_x+k_x][stride*c1_y+k_y];
							neg=(-1)*pos;
							tmp_sum = ((conv1_krns[k_x][k_y][k] == (uint1)0) ? pos:neg);
							conv1_out[b][c1_x][c1_y][k]+=tmp_sum;
						}
					}
				}
			}
		}
	}

	return;
}

void pool1(fixed18 conv1_out[BATCH_SZ][C1_OUT_N_ROWS][C1_OUT_N_COLS][C1_N_KRNS], uint1 pool1_out[BATCH_SZ][P1_OUT_N_ROWS][P1_OUT_N_COLS][C1_N_KRNS]){
	// Max pooling layer. Batch normalization and quantization
	// also happen here
	// Input shape: (1,26,26,32)
	// Output shape: (1,13,13,32)

#pragma HLS INLINE off

	int b, p1_x, p1_y, k, k_x, k_y;
	fixed18 max, c1_val;
	int stride = 2;

	p1_batch: for (b=0; b<BATCH_SZ; b++){
		p1_row: for (p1_x=0; p1_x<P1_OUT_N_ROWS; p1_x++){
			p1_col: for (p1_y=0; p1_y<P1_OUT_N_COLS; p1_y++){
				p1_krn: for (k=0; k<C1_N_KRNS; k++) {
#pragma HLS PIPELINE
					max = conv1_out[b][stride*p1_x][stride*p1_y][k];
					p1_krn_x: for (k_x=0; k_x<P1_KRN_SZ; k_x++){
						p1_krn_y: for (k_y=0; k_y<P1_KRN_SZ; k_y++){
							c1_val = conv1_out[b][stride*p1_x+k_x][stride*p1_y+k_y][k];
							if (c1_val > max){
								max = c1_val;
							}
						}
					}
					pool1_out[b][p1_x][p1_y][k] = ((bn_1_qp[0][k]*(max+bn_1_qp[1][k])) >= 0 ? 0:1);
				}
			}
		}
	}

	return;
}

void conv2(uint1 pool1_out[BATCH_SZ][P1_OUT_N_ROWS][P1_OUT_N_COLS][C1_N_KRNS], int8 conv2_out[BATCH_SZ][C2_OUT_N_ROWS][C2_OUT_N_COLS][C2_N_KRNS]) {
	// Second conv layer. Filter consists of 64 3x3 kernels
	// Input shape: (1,13,13,32)
	// Output shape: (1,11,11,64)

#pragma HLS INLINE off

	int b, c2_x, c2_y, k, ch, k_x, k_y;
	int stride = 1;
	int2 tmp_sum;

	c2_batch: for (b=0; b<BATCH_SZ; b++){
		c2_row: for (c2_x=0; c2_x<C2_OUT_N_ROWS; c2_x++){
			c2_col: for (c2_y=0; c2_y<C2_OUT_N_COLS; c2_y++){
				c2_krn: for (k=0; k<C2_N_KRNS; k++) {
#pragma HLS PIPELINE
					conv2_out[b][c2_x][c2_y][k] = 0;
					c2_in_ch: for (ch=0; ch<C1_N_KRNS; ch++){
						c2_krn_x: for (k_x=0; k_x<C2_KRN_SZ; k_x++){
							c2_krn_y: for (k_y=0; k_y<C2_KRN_SZ; k_y++){
								tmp_sum = ((conv2_krns[k_x][k_y][ch][k] ^ pool1_out[b][stride*c2_x+k_x][stride*c2_y+k_y][ch]), (uint1)1);
								conv2_out[b][c2_x][c2_y][k] += tmp_sum;
							}
						}
					}
				}
			}
		}
	}


	return;
}

void pool2(int8 conv2_out[BATCH_SZ][C2_OUT_N_ROWS][C2_OUT_N_COLS][C2_N_KRNS], uint1 pool2_out[BATCH_SZ][P2_OUT_N_ROWS][P2_OUT_N_COLS][C2_N_KRNS]){
	// Max pooling layer. Batch normalization and quantization
	// also happen here
	// Input shape: (1,11,11,64)
	// Output shape: (1,5,5,64)

#pragma HLS INLINE off

	int b, p2_x, p2_y, k, k_x, k_y;
	int8 max, c2_val;
	int stride = 2;

	p2_batch: for (b=0; b<BATCH_SZ; b++){
		p2_row: for (p2_x=0; p2_x<P2_OUT_N_ROWS; p2_x++){
			p2_col: for (p2_y=0; p2_y<P2_OUT_N_COLS; p2_y++){
				p2_krn: for (k=0; k<C2_N_KRNS; k++) {
#pragma HLS PIPELINE
					max = conv2_out[b][stride*p2_x][stride*p2_y][k];
					p2_krn_x: for (k_x=0; k_x<P2_KRN_SZ; k_x++){
						p2_krn_y: for (k_y=0; k_y<P2_KRN_SZ; k_y++){
							c2_val = conv2_out[b][stride*p2_x+k_x][stride*p2_y+k_y][k];
							if (c2_val > max){
								max = c2_val;
							}
						}
					}
					pool2_out[b][p2_x][p2_y][k] = ((bn_2_qp[0][k]*(max+bn_2_qp[1][k])) >= 0 ? 0:1);
				}
			}
		}
	}

	return;
}

void conv3(uint1 pool2_out[BATCH_SZ][P2_OUT_N_ROWS][P2_OUT_N_COLS][C2_N_KRNS], uint1 conv3_out[BATCH_SZ][C3_OUT_N_ROWS][C3_OUT_N_COLS][C3_N_KRNS]) {
	// Third conv layer. Filter consists of 64 3x3 kernels
	// Batch normalization and quantization
	// also happen here
	// Input shape: (1,5,5,64)
	// Output shape: (1,3,3,64)

#pragma HLS INLINE off

	int b, c3_x, c3_y, k, ch, k_x, k_y;
	int8 sum_val;
	int2 tmp_sum;
	int stride = 1;

	c3_batch: for (b=0; b<BATCH_SZ; b++){
		c3_row: for (c3_x=0; c3_x<C3_OUT_N_ROWS; c3_x++){
			c3_col: for (c3_y=0; c3_y<C3_OUT_N_COLS; c3_y++){
				c3_krn: for (k=0; k<C3_N_KRNS; k++) {
#pragma HLS PIPELINE
					sum_val = 0;
					c3_in_ch: for (ch=0; ch<C2_N_KRNS; ch++){
						c3_krn_x: for (k_x=0; k_x<C3_KRN_SZ; k_x++){
							c3_krn_y: for (k_y=0; k_y<C3_KRN_SZ; k_y++){
								tmp_sum = ((conv3_krns[k_x][k_y][ch][k] ^ pool2_out[b][stride*c3_x+k_x][stride*c3_y+k_y][ch]), (uint1)1);
								sum_val += tmp_sum;
							}
						}
					}
					conv3_out[b][c3_x][c3_y][k] = ((bn_3_qp[0][k]*(sum_val+bn_3_qp[1][k])) >= 0 ? 0:1);
				}
			}
		}
	}

	return;
}

void flatten(uint1 conv3_out[BATCH_SZ][C3_OUT_N_ROWS][C3_OUT_N_COLS][C3_N_KRNS], uint1 flat_out[BATCH_SZ][F_OUT_SZ]){
	// Reshape an array from size [a][b][c][d] to [a][b*c*d]
	// Input shape: (1,3,3,64)
	// Output shape: (1,576)

#pragma HLS INLINE off

	int b, c3_x, c3_y, ch;

	flat_batch: for (b=0; b<BATCH_SZ; b++){
		flat_row: for (c3_x=0; c3_x<C3_OUT_N_ROWS; c3_x++){
			flat_col: for (c3_y=0; c3_y<C3_OUT_N_COLS; c3_y++){
				flat_in_ch: for (ch=0; ch<C3_N_KRNS; ch++){
#pragma HLS PIPELINE
					flat_out[b][ch+c3_y*C3_N_KRNS+c3_x*C3_OUT_N_COLS*C3_N_KRNS] = conv3_out[b][c3_x][c3_y][ch];
				}
			}
		}
	}

	return;
}

void fc1(uint1 flat_out[BATCH_SZ][F_OUT_SZ], uint1 fc1_out[BATCH_SZ][FC1_SZ]){
	// Fully-connected layer. Batch normalization and quantization
	// also happen here
	// Input shape: (1,576)
	// Output shape: (1,64)

#pragma HLS INLINE off

	int b, i_node, o_node;
	int8 sum_val;
	int2 tmp_sum;

	fc1_batch: for (b=0; b<BATCH_SZ; b++){
		fc1_outnodes: for (o_node=0; o_node<FC1_SZ; o_node++){
			sum_val = 0;
			fc1_innodes: for (i_node=0; i_node<F_OUT_SZ; i_node++){
#pragma HLS PIPELINE
				tmp_sum = ((flat_out[b][i_node] ^ fc_1_w[i_node][o_node]), (uint1)1);
				sum_val+=tmp_sum;
			}
			fc1_out[b][o_node] = ((bn_4_qp[0][o_node]*(sum_val+bn_4_qp[1][o_node])) >= 0 ? 0:1);
		}
	}

	return;
}

void fc2(uint1 fc1_out[BATCH_SZ][FC1_SZ], fixed18 fc2_out[BATCH_SZ][FC2_SZ]){
	// Fully-connected layer. The Batch normalization
	// and softmax computation also happen here
	// Input shape: (1,64)
	// Output shape: (1,10)

#pragma HLS INLINE off

	int b, i_node, o_node;

	float sum = 0.0;
	float prob[FC2_SZ];
	float bn_sum;
	int8 sum_val;
	int2 tmp_sum;

	fc2_batch: for (b=0; b<BATCH_SZ; b++){
		sum = 0.0;
		fc2_onodes: for (o_node = 0; o_node < FC2_SZ; o_node++) {
#pragma HLS PIPELINE
			sum_val = 0;
			fc2_inodes: for (i_node = 0; i_node < FC1_SZ; i_node++) {
				tmp_sum = ((fc1_out[b][i_node] ^ fc_2_w[i_node][o_node]), (uint1)1);
				sum_val += tmp_sum;
			}
			bn_sum = (bn_5_qp[0][o_node]*(sum_val+bn_5_qp[1][o_node]));
			prob[o_node] = bn_sum;
		}

		fc2_sum: for (o_node = 0; o_node < FC2_SZ; o_node++) {
#pragma HLS PIPELINE
			sum += expf(prob[o_node]);
		}
		fc2_softmax: for (o_node = 0; o_node < FC2_SZ; o_node++) {
#pragma HLS PIPELINE
			fc2_out[b][o_node] = (fixed18)(expf(prob[o_node]) / sum);
		}
	}

	return;
}

void predict(fixed18 fc2_out[BATCH_SZ][FC2_SZ], uint8 digit[BATCH_SZ]){
	// Return the number with greatest probability computed by the CNN

#pragma HLS INLINE off

	int b, i;
	uint8 curr_res;
	fixed18 curr_prob;
	pred_batch: for (b=0; b<BATCH_SZ; b++){
		curr_res=0;
		curr_prob=0;
		pred_nodes: for (i=0; i<FC2_SZ; i++){
			if (fc2_out[b][i] > curr_prob){
				curr_res = i;
				curr_prob = fc2_out[b][i];
			}
		}
		digit[b] = curr_res;
	}
}

void write_stream(uint8 digit[BATCH_SZ], STREAM_UINT8 &digit_out){

#pragma HLS INLINE off

	int b;

	wr_batch: for(b=0; b<BATCH_SZ; b++){
		AXI_UINT8_STRUCT out_str;
		out_str.data = digit[b];
		if(b==BATCH_SZ-1){
			out_str.last=1;
		} else{
			out_str.last=0;
		}
		digit_out.write(out_str);
	}
}

void bnn_top(
		STREAM_UINT8 &image_in,
		STREAM_UINT8 &digit_out) {
#pragma HLS INTERFACE s_axilite port=return
#pragma HLS INTERFACE axis port=image_in
#pragma HLS INTERFACE axis port=digit_out

#pragma HLS DATAFLOW

#pragma HLS ARRAY_PARTITION variable=fc_1_w complete dim=2

	fixed18 image[BATCH_SZ][N_ROWS][N_COLS];
#pragma HLS ARRAY_PARTITION variable=image complete dim=3
	fixed18 conv1_out[BATCH_SZ][C1_OUT_N_ROWS][C1_OUT_N_COLS][C1_N_KRNS];
#pragma HLS ARRAY_PARTITION variable=conv1_out complete dim=4
	uint1 pool1_out[BATCH_SZ][P1_OUT_N_ROWS][P1_OUT_N_COLS][C1_N_KRNS];
#pragma HLS ARRAY_PARTITION variable=pool1_out complete dim=4
	int8 conv2_out[BATCH_SZ][C2_OUT_N_ROWS][C2_OUT_N_COLS][C2_N_KRNS];
#pragma HLS ARRAY_PARTITION variable=conv2_out complete dim=4
	uint1 pool2_out[BATCH_SZ][P2_OUT_N_ROWS][P2_OUT_N_COLS][C2_N_KRNS];
#pragma HLS ARRAY_PARTITION variable=pool2_out complete dim=4
	uint1 conv3_out[BATCH_SZ][C3_OUT_N_ROWS][C3_OUT_N_COLS][C3_N_KRNS];
#pragma HLS ARRAY_PARTITION variable=conv3_out complete dim=4
	uint1 flat_out[BATCH_SZ][F_OUT_SZ];
#pragma HLS ARRAY_PARTITION variable=flat_out block factor=32 dim=2
	uint1 fc1_out[BATCH_SZ][FC1_SZ];
#pragma HLS ARRAY_PARTITION variable=fc1_out block factor=32 dim=2
	fixed18 fc2_out[BATCH_SZ][FC2_SZ];
	uint8 digit[BATCH_SZ];

	read_stream(image_in, image);

	conv1(image, conv1_out);

	pool1(conv1_out, pool1_out);

	conv2(pool1_out, conv2_out);

	pool2(conv2_out, pool2_out);

	conv3(pool2_out, conv3_out);

	flatten(conv3_out, flat_out);

	fc1(flat_out, fc1_out);

	fc2(fc1_out, fc2_out);

	predict(fc2_out, digit);

	write_stream(digit, digit_out);

	return;
}
