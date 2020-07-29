# Forest Setup Guide

This guide will help you to install the PYNQ 2.5 environment, and ROS2 Eloquent on PYNQ. These are required in order to use the Forest tool.

## PYNQ 2.5 Installation

### Requirements

- Xilinx Zynq board

- If you are building the PYNQ image, you will need Ubuntu 18.04, 
[Vivado 2019.1](https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/vivado-design-tools/2019-1.html), 
and [Petalinux 2019.1](https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/embedded-design-tools/2019-1.html)

- SD Card for the PYNQ image. At least 16GB is recommended

### Procedure 

If you use one of the boards which are officially supported by PYNQ (Pynq-Z1, Pynq-Z2, ZCU104, ZCU111), you can simply
download a PYNQ image, pass it to your SD Card, and you are done! Refer to the instructions [here](http://www.pynq.io/board.html)
and [here](https://pynq.readthedocs.io/en/v2.5/getting_started.html) for the supported boards.

If you are not using one of these boards, you will need to build PYNQ yourself. In this case, you can refer to 
[this tutorial](https://wasa-labo.com/wp/?p=612) (In Japanese, but Google Translate worked well), while keeping 
in mind a few differences for the PYNQ 2.5 version:

1. Need Ubuntu 18.04, Vivado 2019.1, Petalinux 2019.1. No need for SDx

2. At the step of running make, there are two additional useful switches:

- PREBUILT allows you to specify a pre-built image for the rootfs (board-agnostic step), which saves time.
The v2.5 pre-built image for 32-bit ARM can be downloaded [here](http://bit.ly/33fftBw)

- PYNQ_SDIST. See [this post](https://discuss.pynq.io/t/pynq-build-on-custom-board-freezes/1236).
The necessary .tar.gz file can be downloaded [here](https://github.com/Xilinx/PYNQ/releases/download/v2.5/pynq-2.5.tar.gz)

- So the final make command was:

`make PREBUILT=/home/user/Desktop/work/bionic.arm.2.5.img 
PYNQ_SDIST=/home/user/Desktop/work/PYNQ/sdbuild/pynq-2.5.tar.gz
BOARDDIR=/home/user/Desktop/work/PYNQ/myboards BOARDS=Z7-20`

Then transfer the output .img file to your SD Card and boot the board in SD mode.

You can also refer to the [official PYNQ guide](https://pynq.readthedocs.io/en/v2.5/pynq_sd_card.html).

## ROS2 Eloquent Installation on the PYNQ v2.5 system

Refer to the official ROS2 Eloquent installation instructions [here](https://index.ros.org/doc/ros2/Installation/Eloquent/). I built ROS2 Eloquent [from source](https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Development-Setup/).

Follow the official guide. There are a few extra things to do during the installation which are different from the official guide.

1. Before starting, do `export ROS_OS_OVERRIDE=ubuntu:bionic`

2. In the "Add ROS2 apt repository" step, change 'lsb_release -cs' to 'bionic'

3. In the "Install Dependencies using rosdep" step, specify the OS:
`rosdep install --from-paths src --ignore-src --rosdistro eloquent --os ubuntu:bionic -y --skip-keys "console_bridge fastcdr 
fastrtps libopensplice67 libopensplice69 rti-connext-dds-5.3.1 urdfdom_headers"`

4. Export the HOME env var, as ROS2 needs it for logging, but it is not exported by default on PYNQ

5. You may want to leave out certain ROS2 visualization tools (rviz, rqt, etc.) from the installation, to save space/installation time. You can see how to do this in the "Build the code in the workspace" step
