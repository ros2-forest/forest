#include "cartpole_rl.h"

static fixed22 fc_1_w[FC1_SZ][N_STA] ={
	#include "fc_1_w.h"
};
static fixed22 fc_1_b[FC1_SZ] = {
	#include "fc_1_b.h"
};
static fixed22 fc_2_w[FC2_SZ][FC1_SZ] ={
	#include "fc_2_w.h"
};
static fixed22 fc_2_b[FC2_SZ] = {
	#include "fc_2_b.h"
};
static fixed22 fc_3_w[FC3_SZ][FC2_SZ] ={
	#include "fc_3_w.h"
};
static fixed22 fc_3_b[FC3_SZ] = {
	#include "fc_3_b.h"
};
static fixed22 fc_4_w[FC4_SZ][FC3_SZ] ={
	#include "fc_4_w.h"
};
static fixed22 fc_4_b[FC4_SZ] = {
	#include "fc_4_b.h"
};

void read_stream(STREAM_INT &state_in, fixed22 state[N_STA]){
	// Read the input stream and put the values in the state array

#pragma HLS INLINE off

	int s;
	union_t data_in;

	str_rd: for (s=0; s<N_STA; s++){
#pragma HLS PIPELINE
		data_in.i = state_in.read().data;
		state[s] = (fixed22)data_in.f;
	}

	return;
}

void fc1(fixed22 state[N_STA], fixed22 fc1_out[FC1_SZ]){
	// Fully-connected layer. The ReLU behaviour is
	// implemented in this function as well
	// Input shape: (1,4)
	// Output shape: (1,128)

#pragma HLS INLINE off

	int i_node, o_node;

	fc1_outnodes: for (o_node=0; o_node<FC1_SZ; o_node++){
#pragma HLS PIPELINE
		fc1_out[o_node]=fc_1_b[o_node];
		fc1_innodes: for (i_node=0; i_node<N_STA; i_node++){
			fc1_out[o_node]+=state[i_node]*fc_1_w[o_node][i_node];
		}
		fc1_out[o_node] = ((fc1_out[o_node] > (fixed22)0) ? fc1_out[o_node] : (fixed22)0);
	}

	return;
}

void fc2(fixed22 fc1_out[FC1_SZ], fixed22 fc2_out[FC2_SZ]){
	// Fully-connected layer. The ReLU behaviour is
	// implemented in this function as well
	// Input shape: (1,128)
	// Output shape: (1,32)

#pragma HLS INLINE off

	int i_node, o_node;

	fc2_outnodes: for (o_node=0; o_node<FC2_SZ; o_node++){
#pragma HLS PIPELINE
		fc2_out[o_node]=fc_2_b[o_node];
		fc2_innodes: for (i_node=0; i_node<FC1_SZ; i_node++){
			fc2_out[o_node]+=fc1_out[i_node]*fc_2_w[o_node][i_node];
		}
		fc2_out[o_node] = ((fc2_out[o_node] > (fixed22)0) ? fc2_out[o_node] : (fixed22)0);
	}

	return;
}

void fc3(fixed22 fc2_out[FC2_SZ], fixed22 fc3_out[FC3_SZ]){
	// Fully-connected layer. The ReLU behaviour is
	// implemented in this function as well
	// Input shape: (1,32)
	// Output shape: (1,8)

#pragma HLS INLINE off

	int i_node, o_node;

	fc3_outnodes: for (o_node=0; o_node<FC3_SZ; o_node++){
#pragma HLS PIPELINE
		fc3_out[o_node]=fc_3_b[o_node];
		fc3_innodes: for (i_node=0; i_node<FC2_SZ; i_node++){
			fc3_out[o_node]+=fc2_out[i_node]*fc_3_w[o_node][i_node];
		}
		fc3_out[o_node] = ((fc3_out[o_node] > (fixed22)0) ? fc3_out[o_node] : (fixed22)0);
	}

	return;
}

void fc4(fixed22 fc3_out[FC3_SZ], float fc4_out[FC4_SZ]){
	// Fully-connected layer. The ReLU behaviour is
	// implemented in this function as well
	// Input shape: (1,8)
	// Output shape: (1,2)

#pragma HLS INLINE off

	int i_node, o_node;

	fc4_outnodes: for (o_node=0; o_node<FC4_SZ; o_node++){
#pragma HLS PIPELINE
		fc4_out[o_node]=(float)fc_4_b[o_node];
		fc4_innodes: for (i_node=0; i_node<FC3_SZ; i_node++){
			fc4_out[o_node]+=(float) (fc3_out[i_node]*fc_4_w[o_node][i_node]);
		}
	}

	return;
}

void write_stream(float fc4_out[FC4_SZ], STREAM_INT &q_vals_out){

#pragma HLS INLINE off

	int q;
	union_t data_out;
	AXI_INT_STRUCT out_str;

	wr_q: for(q=0; q<FC4_SZ; q++){
		data_out.f = (float)fc4_out[q];
		out_str.data = data_out.i;
		if(q==FC4_SZ-1){
			out_str.last=1;
		} else{
			out_str.last=0;
		}
		q_vals_out.write(out_str);
	}
}

void cartpole_qn(
		STREAM_INT &state_in,
		STREAM_INT &q_vals_out) {
#pragma HLS INTERFACE s_axilite port=return
#pragma HLS INTERFACE axis port=state_in
#pragma HLS INTERFACE axis port=q_vals_out

#pragma HLS DATAFLOW

	fixed22 state[N_STA];
#pragma HLS ARRAY_PARTITION variable=state complete dim=1
	fixed22 fc1_out[FC1_SZ];
#pragma HLS ARRAY_PARTITION variable=fc1_out block factor=32 dim=1
	fixed22 fc2_out[FC2_SZ];
#pragma HLS ARRAY_PARTITION variable=fc2_out block factor=32 dim=1
	fixed22 fc3_out[FC3_SZ];
#pragma HLS ARRAY_PARTITION variable=fc3_out complete dim=1
	float fc4_out[FC4_SZ];
#pragma HLS ARRAY_PARTITION variable=fc4_out complete dim=1

	read_stream(state_in, state);

	fc1(state, fc1_out);

	fc2(fc1_out, fc2_out);

	fc3(fc2_out, fc3_out);

	fc4(fc3_out, fc4_out);

	write_stream(fc4_out, q_vals_out);

	return;
}
