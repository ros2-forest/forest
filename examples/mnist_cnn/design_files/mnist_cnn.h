#include <ap_int.h>
#include <hls_stream.h>
#include <ap_axi_sdata.h>
#include <ap_fixed.h>
#include <math.h>

// Macros

#define N_ROWS 28
#define N_COLS 28
#define BATCH_SZ 1

#define C1_KRN_SZ 3
#define C1_N_KRNS 16
#define C1_OUT_N_ROWS (N_ROWS-C1_KRN_SZ+1) //26
#define C1_OUT_N_COLS (N_COLS-C1_KRN_SZ+1) //26

#define P1_KRN_SZ 2
#define P1_OUT_N_ROWS ((int) (C1_OUT_N_ROWS/2)) //13
#define P1_OUT_N_COLS ((int) (C1_OUT_N_COLS/2)) //13

#define C2_KRN_SZ 3
#define C2_N_KRNS 32
#define C2_OUT_N_ROWS (P1_OUT_N_ROWS-C2_KRN_SZ+1) //11
#define C2_OUT_N_COLS (P1_OUT_N_COLS-C2_KRN_SZ+1) //11

#define P2_KRN_SZ 2
#define P2_OUT_N_ROWS ((int) (C2_OUT_N_ROWS/2)) //5
#define P2_OUT_N_COLS ((int) (C2_OUT_N_COLS/2)) //5

#define F_OUT_SZ (P2_OUT_N_ROWS*P2_OUT_N_COLS*C2_N_KRNS) //800

#define FC1_SZ 64

#define FC2_SZ 10

// Type definitions

typedef struct {
	ap_uint<8> data;
	ap_uint<1> last;
} AXI_UINT8_STRUCT;
typedef hls::stream<AXI_UINT8_STRUCT> STREAM_UINT8;
typedef ap_uint<8> uint8;
typedef ap_fixed<16,6> fixed16;
