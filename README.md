# ROS2 - Forest

FOrEST (FPGA-Oriented Easy Synthesizer Tool) is a tool that allows an easy and seamless integration of HLS-generated FPGA logic into [ROS2](https://index.ros.org/doc/ros2/) systems. It automatically generates a ROS2-FPGA node, which can be used to accelerate versatile and robust real-world robot applications.

Below you can see an example of a ROS2 node that makes use of FPGA hardware to perform the MNIST digit recognition task through convolutional neural networks running. The 
output of the FPGA CNN hardware (FPGA prediction) is printed to the screen by another ROS2 node.

![mnist_bnn_gif](docs/mnist_bnn_gif.gif)

## Environment

For the installation of PYNQ and ROS2 Eloquent on the Zynq boards, please refer to the step-by-step [setup guide](https://github.com/ros2-forest/forest/tree/master/docs/setup_guide).

#### PYNQ

Forest runs on [PYNQ version 2.5](https://pynq.readthedocs.io/en/v2.5/index.html).

#### Python

Tested with Python 3.6.

#### Required Python Packages

The following Python packages are required:

`pip3 install random time struct numpy subprocess sys os getopt jinja2`

#### ROS2

[ROS2 Eloquent](https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Development-Setup/), built from source.

#### Vivado

Tested with designs from Vivado 2019.1 and Vivado HLS 2019.1.

## Installation

Go through the [Setup Guide](https://github.com/ros2-forest/forest/tree/master/docs/setup_guide) to install PYNQ v2.5 and ROS2 Eloquent, and then get the Forest tool in your Zynq board by cloning the Github repository.

`git clone https://github.com/ros2-forest/forest.git`

## Usage

```
python3 forest.py [-h] [-t] [-g -i ninputs -o noutputs]

-h or --help: Prints the usage statement for the script

-t or --test: Generates simple talker and listener nodes along with the FPGA ROS node

-g or --genconfig: Generates a template config file to be used by the script

-i or --ninputs: Number of input signals for the template config file

-o or --noutputs: Number of output signals for the template config file
```

## Tutorial

See the [Getting Started](https://github.com/ros2-forest/forest/tree/master/docs/tutorials) tutorial.

## Examples

1. [Image Processing - Contrast Stretching](https://github.com/ros2-forest/forest/tree/master/examples/contrast_stretch)

2. [Machine Learning - BNN for MNIST digit recognition](https://github.com/ros2-forest/forest/tree/master/examples/mnist_bnn)

3. [Machine Learning - CNN for MNIST digit recognition](https://github.com/ros2-forest/forest/tree/master/examples/mnist_cnn)
