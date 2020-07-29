# Tutorial - Getting Started with Forest

In this tutorial, we will present an example of utilizing the ROS2-Forest tool to incorporate FPGA logic to a ROS2 system.

You will learn:

1. How to generate a template config.forest file and how to populate it with the appropriate information from your FPGA design.

2. How to run the ROS2-Forest tool and generate a ROS2 package that has drivers to communicate with the FPGA logic.

3. How to generate simple talker and listener nodes to verify the communication between FPGA and software is working.

4. Limitations and design considerations when using Forest.

## Introduction

In this tutorial, we will make use of a very simple HLS design, the add module, shown below. We wish to incorporate the
add module into a ROS2 system by making use of the Forest tool.

```C
void add(int a, int b, int c[1]) {

#pragma HLS INTERFACE s_axilite port=a
#pragma HLS INTERFACE s_axilite port=b
#pragma HLS INTERFACE s_axilite port=c
#pragma HLS INTERFACE s_axilite port=return

  c[0]= a+b;

}
```

This design consists of an adder module which adds two integer variables. The result is placed in the `int` array c.

The first step to use this design with Forest is to follow the traditional Vivado HLS design process. The HLS design 
must be synthesized and exported as an IP to Vivado, where it is integrated with the PS part of the Zynq, just like 
any PYNQ overlay. If you are not familiar with creating PYNQ overlays, you can refer to the 
[PYNQ docs](https://pynq.readthedocs.io/en/latest/overlay_design_methodology.html).

After integrating the HLS IP with the Zynq PS and generating the bitstream, you will have a .bit and a .hwh file
which will be needed by PYNQ (and thus Forest). You will have to copy these files over to your PYNQ system, where 
the Forest tool will be running.

## Configuration File - config.forest

The next step is to provide Forest with a configuration file that contains some information about your system, this
file is the **config.forest** file.

You can generate a template config.forest file for this design by going to the directory where the forest.py script is 
located and running:

```
python3 forest.py -g -i 2 -o 1
```

The `-g` flag tells forest to generate the template config.forest. The values beside `-i` and `-o` tell the tool how many
inputs and how many outputs the design has, respectively. The adder module has 2 inputs (`int a` and `int b`) and 1 output
(`int c[1]`).

After running this command, you will have a config.forest file in the same directory as the forest.py script.

Now we need to populate the config.forest file with information about the design. The finalized config.forest file
is shown below:

```
**** 1- Setup Information ****

Forest project name:simple_add

Absolute ROS2 dev_ws path:/home/xilinx/work/dev_ws

Absolute FPGA .bit file path:/home/xilinx/work/vivado_prjs/simple_add/overlays/add.bit

User IP name:add_0

**** 2- Input definitions ****

// Input 1

Input name:a

Protocol:lite

Type:int32

Address Offset (if AXI-Lite):16

// Input 2

Input name:b

Protocol:lite

Type:int32

Address Offset (if AXI-Lite):24

**** 3- Output definitions ****

// Output 1

Output name:sum

Protocol:lite

Type:int32[1]

Address Offset (if AXI-Lite):32
```

The **Forest project name** value is completely chosen by the user. It will be used by the tool to name the generated ROS2 packages
that contain the drivers for the FPGA logic. The generated packages will be called forest\_\<Forest project name\>\_fpga\_node and
forest\_\<Forest project name\>\_interface. Please avoid having spaces in this field.

The **Absolute ROS2 dev_ws path** value tells the tool the (absolute) location of the ROS2 dev_ws where you want the generated packages
to be located.

**Absolute FPGA .bit file path** tells the tool the (absolute) location of the .bit file for the design. Note that as with any other PYNQ 
overlay, the .hwh file must be located in the same location as your .bit file.

**User IP name** is the name of your HLS IP. This can be seen in the Vivado block design, as shown in the image below:

![User IP name](https://github.com/ros2-forest/forest/blob/master/docs/tutorials/ip_name.png)

Next are the input and output definitions. You will need to fill in these fields for each I/O signal in your design.

**Input/Output name** is freely chosen by the user. The value will be used, for instance, in the .msg files created by the tool. It may be 
a good idea to keep the naming consistent with name of the corresponding signal in the HLS module.

**Protocol** indicates the protocol used to send/receive the signal to/from the PS to the FPGA. The allowed values are "lite" for AXI-Lite or 
"stream" for AXI-Stream. Note that every signal in your design must be sent either by AXI-Lite or AXI-Stream, and that only certain data types 
are supported for each protocol (see the Considerations and Limitations section below).

**Type** tells the tool the data type of the signal. The type must be given according to ROS2 data types (not C/C++). The mapping between ROS2
and C/C++ data types can be seen [here](https://index.ros.org/doc/ros2/Concepts/About-ROS-Interfaces/). Note that only certain data types are 
supported (see the Considerations and Limitations section below).

**Address Offset (if AXI-Lite)** provides the address offset of the variable. This value can be found in the .hwh file. For the `int a` input 
signal, for instance, under the `add_0` module in the .hwh, it can be seen that:

```XML
<REGISTER NAME="a">
              <PROPERTY NAME="DESCRIPTION" VALUE="Data signal of a"/>
              <PROPERTY NAME="ADDRESS_OFFSET" VALUE="16"/>
              <PROPERTY NAME="SIZE" VALUE="32"/>
              <PROPERTY NAME="ACCESS" VALUE="write-only"/>
              <PROPERTY NAME="IS_ENABLED" VALUE="true"/>
              <PROPERTY NAME="RESET_VALUE" VALUE="0"/>
              <FIELDS>

```

Thus the address offset value is 16. Note that only AXI-Lite variables have address offset values. AXI-Stream variables are not memory mapped.
For them, this field can be left blank. For array variables the value of this field should be the base address of the array, i.e., the address
of the first element in the array. Similarly, for 64-bit types (e.g. `double`), the base address of the lower 32-bits should be provided. 

## Running the Forest tool

Now that the config.forest file is completed, the forest tool can be run:

```
python3 forest.py -t
```

The tool will run for a few minutes and will create and build two ROS2 packages in the requested dev_ws location. They will be called 
forest\_\<Forest project name\>\_fpga\_node and forest\_\<Forest project name\>\_interface. The fpga\_node package contains a ROS2 node that communicates
with the FPGA and the optional talker and listener nodes, while the interface package contains the message definitions for the FPGA node in the
msg/FpgaIn.msg (input signal definitions) and msg/FpgaOut.msg (output signal definitions) files. The files inside these packages will be created 
based on Jinja 2 template files located in the templates folder. Running the tool will also create an output/ folder, which will hold the 
final version of the Jinja2 templates used in the ROS2 packages.

The -t (test) flag is optional and tells the tool to generate two ROS2 scripts that are used for testing the communication between PS and the 
FPGA logic through ROS2. The nodes are called "talker" and "listener". The talker node publishes random data of the appropriate type to the FPGA 
node's input topic, and the listener node subscribes to the FPGA node's output topic. If we run the talker node, fpga\_node, and listener node at 
the same time, we should be able to see some output in the listener node. This indicates that we are able to send data to the FPGA and receive 
outputs from it. However it does not necessarily indicate that the functionality of the FPGA module is behaving as expected.

## Running the Forest-Generated ROS2-FPGA Node

After the tool finishes running, we can open three different terminals to verify that the module is working as expected, one for the talker node, one 
for the ROS2-FPGA node, and one for the listener node.

- ROS2-FPGA node:

```
cd <dev_ws location>
. install/setup.bash
ros2 run forest_simple_add_fpga_node fpga_node
```

- Listener node:

```
cd <dev_ws location>
. install/setup.bash
ros2 run forest_simple_add_fpga_node listener
```

- Talker node:

```
cd <dev_ws location>
. install/setup.bash
ros2 run forest_simple_add_fpga_node talker
```

## Considerations and Limitations

This section lists some limitations of the Forest tool, and considerations that must be taken when designing HLS modules which will be provided as inputs to Forest.

### Type Support

Only some of the [ROS2 primitive data types](https://index.ros.org/doc/ros2/Concepts/About-ROS-Interfaces/) are supported by Forest. The type support
is as follows:

1. For AXI-Lite variables:

- Input variable types: `uint8, int32, float64, uint8[N], int32[N], float64[N]`

- Output variable types: `uint8[N], int32[N], float64[N]`

2. For AXI-Stream variables:

- Input variable types: `uint8[N], int32[N], uint64[N], float32[N]`

- Output variable types: `uint8[N], int32[N], uint64[N], float32[N]`

All output variables must be arrays. An output variable of size 1 can be created by using an array of size 1, as in the add.c example.

In cases where it is not supported, `float32` (float) can be used, for example, by sending/receiving an `int32` and making use of a union data structure
in the HLS code.

All input and output arrays must have a fixed, bounded size.

### AXI-Stream and DMA

AXI-Stream transfers are assumed to be of constant size (i.e. AXI-Stream variables are fixed-size arrays).

Forest only supports designs with at most 1 DMA. Only at most 1 input variable and 1 output variable can make use of the "stream" protocol.

The DMA is assumed to be called "axi_dma_0". This is the default name of the DMA IP in Vivado IP integrator. If you change the name of the DMA,
then you must also change "axi_dma_0" to the name of your DMA in the `ros_fpga_lib.py` file, which is located in the forest\_\<Forest project name\>\_fpga\_node
package.
  
### Top-Level HLS Module

The top-level HLS module for the design must have return type `void` and have an AXI-Lite control interface.

I.e., the pragma

```
#pragma HLS INTERFACE s_axilite port=return
```

must be present.
