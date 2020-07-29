#include "mnist_cnn.h"

static fixed16 conv1_krns[C1_N_KRNS][C1_KRN_SZ][C1_KRN_SZ] = {
	#include "c1_w.h"
};
static fixed16 conv1_biases[C1_N_KRNS] = {
	#include "c1_b.h"
};
static fixed16 conv2_krns[C2_KRN_SZ][C2_KRN_SZ][C1_N_KRNS][C2_N_KRNS] ={
	#include "c2_w.h"
};
static fixed16 conv2_biases[C2_N_KRNS] = {
	#include "c2_b.h"
};
static fixed16 fc_1_w[FC1_SZ][F_OUT_SZ] ={
	#include "fc_1_w.h"
};
static fixed16 fc_1_b[FC1_SZ] = {
	#include "fc_1_b.h"
};
static fixed16 fc_2_w[FC2_SZ][FC1_SZ] ={
	#include "fc_2_w.h"
};
static fixed16 fc_2_b[FC2_SZ] = {
	#include "fc_2_b.h"
};

void read_stream(STREAM_UINT8 &image_in, fixed16 image[BATCH_SZ][N_ROWS][N_COLS]){
	// Read the input stream and put the values in the image array

#pragma HLS INLINE off

	int b, r, c;
	uint8 data_in;
	fixed16 norm_data_in;

	batch_rd: for(b=0; b<BATCH_SZ; b++){
		row_rd: for (r=0; r<N_ROWS; r++) {
			col_rd: for (c=0; c<N_COLS; c++){
#pragma HLS PIPELINE
				data_in = image_in.read().data;
				norm_data_in = (fixed16)((data_in)/(255.0));
				image[b][r][c] = norm_data_in;
			}
		}
	}

	return;
}

void conv1(fixed16 image[BATCH_SZ][N_ROWS][N_COLS], fixed16 conv1_out[BATCH_SZ][C1_OUT_N_ROWS][C1_OUT_N_COLS][C1_N_KRNS]) {
	// First conv layer. Filter consists of 16 3x3 kernels
	// Input shape: (1,28,28)
	// Output shape: (1,26,26,16)

#pragma HLS INLINE off

	int b, c1_x, c1_y, k, k_x, k_y;
	int stride = 1;

	c1_batch: for (b=0; b<BATCH_SZ; b++){
		c1_row: for (c1_x=0; c1_x<C1_OUT_N_ROWS; c1_x++){
			c1_col: for (c1_y=0; c1_y<C1_OUT_N_COLS; c1_y++){
				c1_krn: for (k=0; k<C1_N_KRNS; k++) {
#pragma HLS PIPELINE
					conv1_out[b][c1_x][c1_y][k] = conv1_biases[k];
					c1_krn_x: for (k_x=0; k_x<C1_KRN_SZ; k_x++){
						c1_krn_y: for (k_y=0; k_y<C1_KRN_SZ; k_y++){
							conv1_out[b][c1_x][c1_y][k]+=conv1_krns[k][k_x][k_y]*image[b][stride*c1_x+k_x][stride*c1_y+k_y];
						}
					}
				}
			}
		}
	}

	return;
}

void pool1(fixed16 conv1_out[BATCH_SZ][C1_OUT_N_ROWS][C1_OUT_N_COLS][C1_N_KRNS], fixed16 pool1_out[BATCH_SZ][P1_OUT_N_ROWS][P1_OUT_N_COLS][C1_N_KRNS]){
	// Max pooling layer
	// The ReLU behaviour is also incorporated here. If none of the
	// values are bigger than 0, the max pool will pick 0
	// Input shape: (1,26,26,16)
	// Output shape: (1,13,13,16)

#pragma HLS INLINE off

	int b, p1_x, p1_y, k, k_x, k_y;
	fixed16 max, c1_val;
	int stride = 2;

	p1_batch: for (b=0; b<BATCH_SZ; b++){
		p1_row: for (p1_x=0; p1_x<P1_OUT_N_ROWS; p1_x++){
			p1_col: for (p1_y=0; p1_y<P1_OUT_N_COLS; p1_y++){
				p1_krn: for (k=0; k<C1_N_KRNS; k++) {
#pragma HLS PIPELINE
					max = 0;
					p1_krn_x: for (k_x=0; k_x<P1_KRN_SZ; k_x++){
						p1_krn_y: for (k_y=0; k_y<P1_KRN_SZ; k_y++){
							c1_val = conv1_out[b][stride*p1_x+k_x][stride*p1_y+k_y][k];
							if (c1_val > max){
								max = c1_val;
							}
						}
					}
					pool1_out[b][p1_x][p1_y][k] = max;
				}
			}
		}
	}

	return;
}

void conv2(fixed16 pool1_out[BATCH_SZ][P1_OUT_N_ROWS][P1_OUT_N_COLS][C1_N_KRNS], fixed16 conv2_out[BATCH_SZ][C2_OUT_N_ROWS][C2_OUT_N_COLS][C2_N_KRNS]) {
	// Second conv layer. Filter consists of 32 3x3 kernels
	// Input shape: (1,13,13,16)
	// Output shape: (1,11,11,32)

#pragma HLS INLINE off

	int b, c2_x, c2_y, k, ch, k_x, k_y;
	int stride = 1;

	c2_batch: for (b=0; b<BATCH_SZ; b++){
		c2_row: for (c2_x=0; c2_x<C2_OUT_N_ROWS; c2_x++){
			c2_col: for (c2_y=0; c2_y<C2_OUT_N_COLS; c2_y++){
				c2_krn: for (k=0; k<C2_N_KRNS; k++) {
#pragma HLS PIPELINE
					conv2_out[b][c2_x][c2_y][k] = conv2_biases[k];
					c2_in_ch: for (ch=0; ch<C1_N_KRNS; ch++){
						c2_krn_x: for (k_x=0; k_x<C2_KRN_SZ; k_x++){
							c2_krn_y: for (k_y=0; k_y<C2_KRN_SZ; k_y++){
								conv2_out[b][c2_x][c2_y][k]+=conv2_krns[k_x][k_y][ch][k]*pool1_out[b][stride*c2_x+k_x][stride*c2_y+k_y][ch];
							}
						}
					}
				}
			}
		}
	}

	return;
}

void pool2(fixed16 conv2_out[BATCH_SZ][C2_OUT_N_ROWS][C2_OUT_N_COLS][C2_N_KRNS], fixed16 pool2_out[BATCH_SZ][P2_OUT_N_ROWS][P2_OUT_N_COLS][C2_N_KRNS]){
	// Max pooling layer
	// The ReLU behaviour is also incorporated here. If none of the
	// values are bigger than 0, the max pool will pick 0
	// Input shape: (1,11,11,32)
	// Output shape: (1,5,5,32)

#pragma HLS INLINE off

	int b, p2_x, p2_y, k, k_x, k_y;
	fixed16 max, c2_val;
	int stride = 2;

	p2_batch: for (b=0; b<BATCH_SZ; b++){
		p2_row: for (p2_x=0; p2_x<P2_OUT_N_ROWS; p2_x++){
			p2_col: for (p2_y=0; p2_y<P2_OUT_N_COLS; p2_y++){
				p2_krn: for (k=0; k<C2_N_KRNS; k++) {
#pragma HLS PIPELINE
					max = 0;
					p2_krn_x: for (k_x=0; k_x<P2_KRN_SZ; k_x++){
						p2_krn_y: for (k_y=0; k_y<P2_KRN_SZ; k_y++){
							c2_val = conv2_out[b][stride*p2_x+k_x][stride*p2_y+k_y][k];
							if (c2_val > max){
								max = c2_val;
							}
						}
					}
					pool2_out[b][p2_x][p2_y][k] = max;
				}
			}
		}
	}

	return;
}

void flatten(fixed16 pool2_out[BATCH_SZ][P2_OUT_N_ROWS][P2_OUT_N_COLS][C2_N_KRNS], fixed16 flat_out[BATCH_SZ][F_OUT_SZ]){
	// Reshape an array from size [a][b][c][d] to [a][b*c*d]
	// Input shape: (1,5,5,32)
	// Output shape: (1,800)

#pragma HLS INLINE off

	int b, p2_x, p2_y, ch;

	flat_batch: for (b=0; b<BATCH_SZ; b++){
		flat_row: for (p2_x=0; p2_x<P2_OUT_N_ROWS; p2_x++){
			flat_col: for (p2_y=0; p2_y<P2_OUT_N_COLS; p2_y++){
				flat_in_ch: for (ch=0; ch<C2_N_KRNS; ch++){
#pragma HLS PIPELINE
					flat_out[b][ch+p2_y*C2_N_KRNS+p2_x*P2_OUT_N_COLS*C2_N_KRNS] = pool2_out[b][p2_x][p2_y][ch];
				}
			}
		}
	}

	return;
}

void fc1(fixed16 flat_out[BATCH_SZ][F_OUT_SZ], fixed16 fc1_out[BATCH_SZ][FC1_SZ]){
	// Fully-connected layer. The ReLU behaviour is
	// implemented in this function as well
	// Input shape: (1,800)
	// Output shape: (1,64)

#pragma HLS INLINE off

	int b, i_node, o_node;

	fc1_batch: for (b=0; b<BATCH_SZ; b++){
		fc1_outnodes: for (o_node=0; o_node<FC1_SZ; o_node++){
			fc1_out[b][o_node]=fc_1_b[o_node];
			fc1_innodes: for (i_node=0; i_node<F_OUT_SZ; i_node++){
#pragma HLS UNROLL factor=32
				fc1_out[b][o_node]+=flat_out[b][i_node]*fc_1_w[o_node][i_node];
			}
			fc1_out[b][o_node] = ((fc1_out[b][o_node] > (fixed16)0) ? fc1_out[b][o_node] : (fixed16)0);
		}
	}

	return;
}

void fc2(fixed16 fc1_out[BATCH_SZ][FC1_SZ], fixed16 fc2_out[BATCH_SZ][FC2_SZ]){
	// Fully-connected layer. The softmax computation
	// also happens here
	// Input shape: (1,64)
	// Output shape: (1,10)

#pragma HLS INLINE off

	int b, i_node, o_node;

	float sum = 0.0;
	float prob[FC2_SZ];

	fc2_batch: for (b=0; b<BATCH_SZ; b++){
		sum = 0.0;
		fc2_onodes: for (o_node = 0; o_node < FC2_SZ; o_node++) {
			prob[o_node] = fc_2_b[o_node];
			fc2_inodes: for (i_node = 0; i_node < FC1_SZ; i_node++) {
#pragma HLS UNROLL factor=2
				prob[o_node]+=((float)fc1_out[b][i_node]*(float)fc_2_w[o_node][i_node]);
			}
		}
		fc2_sum: for (o_node = 0; o_node < FC2_SZ; o_node++) {
			sum += expf(prob[o_node]);
		}
		fc2_softmax: for (o_node = 0; o_node < FC2_SZ; o_node++) {
			fc2_out[b][o_node] = (fixed16)(expf(prob[o_node]) / sum);
		}
	}

	return;
}

void predict(fixed16 fc2_out[BATCH_SZ][FC2_SZ], uint8 digit[BATCH_SZ]){
	// Return the number with greatest probability computed by the CNN

#pragma HLS INLINE off

	int b, i;
	uint8 curr_res;
	fixed16 curr_prob;
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

void cnn_top(
		STREAM_UINT8 &image_in,
		STREAM_UINT8 &digit_out) {
#pragma HLS INTERFACE s_axilite port=return
#pragma HLS INTERFACE axis port=image_in
#pragma HLS INTERFACE axis port=digit_out

#pragma HLS DATAFLOW

	fixed16 image[BATCH_SZ][N_ROWS][N_COLS];
	fixed16 conv1_out[BATCH_SZ][C1_OUT_N_ROWS][C1_OUT_N_COLS][C1_N_KRNS];
	fixed16 pool1_out[BATCH_SZ][P1_OUT_N_ROWS][P1_OUT_N_COLS][C1_N_KRNS];
#pragma HLS array_partition variable=pool1_out complete dim=4
	fixed16 conv2_out[BATCH_SZ][C2_OUT_N_ROWS][C2_OUT_N_COLS][C2_N_KRNS];
	fixed16 pool2_out[BATCH_SZ][P2_OUT_N_ROWS][P2_OUT_N_COLS][C2_N_KRNS];
	fixed16 flat_out[BATCH_SZ][F_OUT_SZ];
	fixed16 fc1_out[BATCH_SZ][FC1_SZ];
	fixed16 fc2_out[BATCH_SZ][FC2_SZ];
	uint8 digit[BATCH_SZ];

	read_stream(image_in, image);

	conv1(image, conv1_out);

	pool1(conv1_out, pool1_out);

	conv2(pool1_out, conv2_out);

	pool2(conv2_out, pool2_out);

	flatten(pool2_out, flat_out);

	fc1(flat_out, fc1_out);

	fc2(fc1_out, fc2_out);

	predict(fc2_out, digit);

	write_stream(digit, digit_out);

	return;
}
