#include <ap_int.h>
#include <hls_stream.h>
#include <ap_axi_sdata.h>
#include <ap_fixed.h>

// Macros

#define N_STA 4

#define FC1_SZ 128

#define FC2_SZ 32

#define FC3_SZ 8

#define FC4_SZ 2

// Type definitions

typedef struct {
	int data;
	ap_uint<1> last;
} AXI_INT_STRUCT;
typedef hls::stream<AXI_INT_STRUCT> STREAM_INT;
typedef ap_fixed<22,8> fixed22;
typedef union {
	int i;
	float f;
} union_t;
