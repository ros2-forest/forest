import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import time

from forest_cs_128_64b_interface.msg import FpgaOut

class RecImgNode(Node):

    def __init__(self):
        super().__init__('receive_img')
        profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE, durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE)
        self.subscription = self.create_subscription(FpgaOut,'fpga_out_topic',self.read_node_callback, profile)
        self.subscription
        self.N_ROWS=128
        self.N_COLS=128
        self.i = 0

    def read_node_callback(self, msg):
        self.t4 = time.time()
        image_out = np.zeros((self.N_ROWS*self.N_COLS,), dtype=np.uint8)
        for i in range(self.N_ROWS*self.N_COLS//8):
            for j in range(8):
                image_out[8*i+j] = np.uint8(np.right_shift(msg.image_out[i],np.uint64(8*j)))
        if self.i == 0:
            self.fig, self.ax = plt.subplots(1,1)
            self.im  = self.ax.imshow(image_out.reshape(self.N_ROWS,self.N_COLS), cmap='gray')
            plt.title("Output Image")
            plt.ion()
            plt.show()
        else:
            self.im.set_data(image_out.reshape(self.N_ROWS,self.N_COLS))
            plt.draw()
            plt.pause(1e-3)
        self.i+=1
        self.get_logger().info("idx:{} t4:{}".format(msg.idx, self.t4))

def main(args=None):
    rclpy.init(args=args)

    receive_img = RecImgNode()

    rclpy.spin(receive_img)

    receive_img.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
