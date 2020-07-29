# Forest Example Project - Linear Contrast Stretch

This example project illustrates an application of a Forest-generated ROS2-FPGA node to image processing. The task is to perform a linear contrast strech operation. You can read more about this operation here [1].

## Introduction and Design Files

The contrast stretch operation was performed in three different grayscale image sizes: 128x128, 256x256, and 512x512. You can find the design files and images used in each of these examples in the cs_128_64b/, cs_256_64b/, and cs_512_64b/ directories. The images were obtained from online image databases [2,3].

In order to improve the FPGA latency, the images were sent in arrays of 64-bit unsigned integers, where each `uint64` represents 8 consecutive pixel values. This reduces the number of AXI-Stream transactions needed to transfer the image to and from the FPGA. So a 512x512 image is sent through an array of `uint64[32768]`, where 32768 = 512\*512\/8.

The setup for this example project consists of three ROS2 nodes. The first ROS2 node runs on a PC and publishes and plots the input image. The second node is the Forest-generated ROS2-FPGA node, which runs on the Zynq board and performs the contrast stretch operation. The third node runs on the PC and receives the output image which was processed by the ROS2-FPGA node and plots it.

Each of the three subdirectories (cs_128_64b/, cs_256_64b/, and cs_512_64b/) contain three directories: design_files/, imgs/, and ros2_packages/, which are explained below:

### design_files/

This directory contains the HLS files for the project (contrast_stretch.cpp and contrast_stretch.h), as well as the config.forest file that is provided to Forest. Note that some information in the config.forest file might need to be changed if you are using it in your own workspace, such as the Absolute ROS2 dev_ws path, Absolute FPGA .bit file path, and User IP name. 

### imgs/

This directory contains the images that are sent to the FPGA to be contrast stretched.

### ros2_packages/

This directory contains two ROS2 packages that shoudl be built ***on your PC*** (not on the Zynq board). They are the packages that define the nodes that send and receive image data to/from the FPGA. There is one change that has to be done for files in this directory. In the send_img.py file (https://github.com/ros2-forest/forest/blob/master/examples/contrast_stretch/cs_512_64b/ros2_packages/cs_512_64b/cs_512_64b/send_img.py), the line:

```Python
image_tiff = Image.open('/home/danielpi/Desktop/work/case_studies/contrast/imgs/test/512/' + self.images[self.i])
```

Must be changed to the location of the imgs/ folder in your file system.

The node send_img.py puts the image in `uint64` format, publishes it to the fpga_in topic and plots it. The node receive_img.py subscribes to the fpga_out topic, and plots the output image. There is also a pc_sw_node.py that performs the contrast stretch operation in software. You can use it to compare the time taken to perform the computation in SW and in the FPGA HW.

## Running the Example Project

This is a step-by-step of how to run the linear contrast stretch example project.

1. Create a new Vivado HLS project and include the contrast_stretch.cpp and contrast_stretch.h files from the design_files/ folder. Synthesize the design and export it as an IP to Vivado.

2. Create a new block design in a new Vivado project and include the IP exported in Step 1. Also include the Zynq IP and a Vivado AXI Direct Memory Access (DMA) IP. Integrate the IPs and compile the design to generate a bitstream.

3. Move the generated bitstream and .hwh files, as well as the config.forest file in the design_files/ directory to your PYNQ system. The config.forest must be placed in the same folder as your forest.py file.

4. Update the config.forest file to account for the location of the ROS2 dev_ws directory and of the .bit and .hwh files in your PYNQ file system.

5. Run Forest to generate the ROS2-FPGA node `python3 forest.py -t`

6. Move the two folders inside ros2_packages/ to the dev_ws/src location in your PC and build them after making the change to the send_img.py file.

7. In your PC, run the send_img and receive_img nodes in two different terminals.

```
ros2 run cs_512_64b receive_img

ros2 run cs_512_64b send_img
```

8. On PYNQ, run the ROS2-FPGA node.

```
ros2 run forest_cs_512_64b_fpga_node fpga_node
```

## Results

### Demo

- ADD VIDEO

### Latency Results

The following table shows the average observed latency to perform the contrast stretch operation on the Digilent Zybo Z7-20 FPGA, 
on an Intel Core i7-6500U processor, and on the Digilent Z7-20 ARM Cortex A9 processor on the 512x512 images:

| Device | Device Latency (ms) | Ratio (Device Latency:FPGA Latency) |
| :---:         |     ---:      |          ---: |
| Intel CPU     | 9832.9      | 36.3:1      |
| FPGA   | 270.7     | 1:1    |
| ARM     | 71237.3      | 262.2:1      |

### Hardware Resource Usage

The following images show the hardware resource usage reported by Vivado HLS for the 128x128, 256x256, and 512x512 contrast stretch designs, respectively.

- 128x128

![usage_128](https://github.com/ros2-forest/forest/blob/master/examples/contrast_stretch/cs_128_64b_usage.png)

- 256x256

![usage_256](https://github.com/ros2-forest/forest/blob/master/examples/contrast_stretch/cs_256_64b_usage.png)

- 512x512

![usage_512](https://github.com/ros2-forest/forest/blob/master/examples/contrast_stretch/cs_512_64b_usage.png)

## References

[1] - R. Fisher, S. Perkins, A. Walker, and E. Wolfart, “Contrast Stretching,” Point Operations - Contrast Stretching. Online. Available: https://homepages.inf.ed.ac.uk/rbf/HIPR2/stretch.htm. Accessed: 23-Jul-2020.

[2] - SIPI Image Database. Online. Available: http://sipi.usc.edu/database/. Accessed: 23-Jul-2020.

[3] - “CVG - UGR - Image database,” UGR. Online. Available: http://decsai.ugr.es/cvg/dbimagenes/. Accessed: 23-Jul-2020.
