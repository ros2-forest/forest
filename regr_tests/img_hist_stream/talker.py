import rclpy
from rclpy.node import Node

import random
import struct
import numpy as np

from forest_img_hist_stream_interface.msg import FpgaIn

class TalkerNode(Node):

    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(FpgaIn, 'fpga_in_topic', 10)
        timer_period = 2  # Frequency of publishing
        self.timer = self.create_timer(timer_period, self.timer_callback)
        with open('/home/xilinx/work/utils/mnist/train-images-idx3-ubyte','rb') as f:
            magic, size = struct.unpack(">II", f.read(8))
            nrows, ncols = struct.unpack(">II", f.read(8))
            data = np.fromfile(f, dtype=np.dtype(np.uint8).newbyteorder('>'))
            data_plot = data.reshape((size, nrows, ncols))
            self.data_ros = data.reshape((size, nrows*ncols))

    def timer_callback(self):
        msg = FpgaIn()
        #for i in range(784):
            #msg.image[i] = random.randint(0, 5)
        i = random.randint(0, 60000)
        msg.image = self.data_ros[i]
        self.publisher_.publish(msg)
        #self.get_logger().info("Publishing: {}".format(msg))
        sw_result = self.get_hist_sw(self.data_ros[i])
        self.get_logger().info("SW Result: {}".format(sw_result))

    def get_hist_sw(self, img):
        hist_sw = [0]*256
        for i in range(len(img)):
            hist_sw[img[i]] += 1
        return hist_sw

def main(args=None):
    rclpy.init(args=args)

    talker = TalkerNode()

    rclpy.spin(talker)

    talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
