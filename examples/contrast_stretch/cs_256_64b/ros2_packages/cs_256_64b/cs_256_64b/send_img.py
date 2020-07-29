import rclpy
from rclpy.node import Node

import random
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import time

from forest_cs_256_64b_interface.msg import FpgaIn

class SendImgNode(Node):

    def __init__(self):
        super().__init__('send_img')
        self.publisher_ = self.create_publisher(FpgaIn, 'fpga_in_topic', 10)
        timer_period = 20  # Frequency of publishing
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.N_ROWS=256
        self.N_COLS=256
        self.images = ['5.1.09.tiff', '5.1.10.tiff', '5.1.11.tiff', '5.1.12.tiff', '5.1.14.tiff']
        self.i=0

    def timer_callback(self):
        if self.i < 5:
            msg = FpgaIn()
            # Open Image
            image_tiff = Image.open('/home/danielpi/Desktop/work/case_studies/contrast/imgs/test/256/' + self.images[self.i])
            # Transform into Numpy array
            im_array = np.array(image_tiff, dtype=np.uint8)
            im_list = im_array.flatten()
            # Prepare image data for publishing
            for i in range(self.N_ROWS*self.N_COLS//8):
                msg.image_in[i] = (im_list[8*i+7] << (64-1*8)) | (im_list[8*i+6] << (64-2*8)) | (im_list[8*i+5] << (64-3*8)) | (im_list[8*i+4] << (64-4*8)) | (im_list[8*i+3] << (64-5*8)) | (im_list[8*i+2] << (64-6*8)) | (im_list[8*i+1] << (64-7*8)) | (im_list[8*i])
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
            self.i+=1
        self.get_logger().info("t1:{}".format(self.t1))

def main(args=None):
    rclpy.init(args=args)

    send_img = SendImgNode()

    rclpy.spin(send_img)

    send_img.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
