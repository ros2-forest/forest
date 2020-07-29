import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

import random
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import time

from forest_cs_128_64b_interface.msg import FpgaIn

class SendImgNode(Node):

    def __init__(self):
        super().__init__('send_img')
        profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE, durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE)
        self.publisher_ = self.create_publisher(FpgaIn, 'fpga_in_topic', profile)
        timer_period = 7  # Frequency of publishing
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.N_ROWS=128
        self.N_COLS=128
        self.images = ['1023.tiff', '1073.tiff', '1097.tiff', '175.tiff', '210.tiff']
        self.i=0

    def timer_callback(self):
        if self.i < 50:
            msg = FpgaIn()
            # Open Image
            image_tiff = Image.open('/home/danielpi/Desktop/work/case_studies/contrast/imgs/test/128/' + self.images[self.i%5])
            # Transform into Numpy array
            im_array = np.array(image_tiff, dtype=np.uint8)
            im_list = im_array.flatten()
            # Prepare image data for publishing
            for i in range(self.N_ROWS*self.N_COLS//8):
                msg.image_in[i] = (im_list[8*i+7] << (64-1*8)) | (im_list[8*i+6] << (64-2*8)) | (im_list[8*i+5] << (64-3*8)) | (im_list[8*i+4] << (64-4*8)) | (im_list[8*i+3] << (64-5*8)) | (im_list[8*i+2] << (64-6*8)) | (im_list[8*i+1] << (64-7*8)) | (im_list[8*i])
            msg.idx = self.i
            # Publish image data
            self.t1 = time.time()
            self.publisher_.publish(msg)
            # Plot image
            if self.i == 0:
                self.fig, self.ax = plt.subplots(1,1)
                self.im = self.ax.imshow(im_array, cmap='gray')
                plt.title("Input Image")
                plt.ion()
                plt.show()
            else:
                self.im.set_data(im_array)
                plt.draw()
                plt.pause(1e-3)
            self.get_logger().info("idx:{} t1:{}".format(self.i, self.t1))
            self.i+=1
        
def main(args=None):
    rclpy.init(args=args)

    send_img = SendImgNode()

    rclpy.spin(send_img)

    send_img.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
