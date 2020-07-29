import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

import random
import numpy as np
import matplotlib.pyplot as plt
import time
import struct
import random

from tensorflow import keras
from tensorflow.keras.datasets import mnist
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Dropout, Flatten
from tensorflow.keras.layers import Conv2D, MaxPooling2D
from tensorflow.keras import backend as K
from keras.models import model_from_json
import os

from forest_mnist_cnn_first_interface.msg import FpgaIn

class SendNode(Node):

    def __init__(self):
        super().__init__('send')
        profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE, durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE)
        self.publisher_ = self.create_publisher(FpgaIn, 'fpga_in_topic', profile)
        timer_period = 0.5  # Frequency of publishing
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.N_ROWS=28
        self.N_COLS=28
        self.i=0
        # Suppress warnings
        os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
        num_classes=10
        # input image dimensions
        img_rows, img_cols = 28, 28
        (x_train, y_train), (x_test, self.y_test) = mnist.load_data()

        if K.image_data_format() == 'channels_first':
            self.x_test = x_test.reshape(x_test.shape[0], 1, img_rows, img_cols)
        else:
            self.x_test = x_test.reshape(x_test.shape[0], img_rows, img_cols, 1)

    def timer_callback(self):
        if self.i < 100:
            msg = FpgaIn()
            # Open Image
            img_plt = self.x_test[self.i][:,:,0]
            img_data = img_plt.flatten()
            label = int(self.y_test[self.i])
            # Prepare image data for publishing
            msg.image_in = img_data
            msg.label = label
            self.t1 = time.time()
            self.publisher_.publish(msg)
            # Plot image
            '''if self.i == 0:
                self.fig, self.ax = plt.subplots(1,1)
                self.im = self.ax.imshow(img_plt, cmap='gray')
                plt.title("Input Image")
                plt.ion()
                plt.show()
            else:
                self.im.set_data(img_plt)
                plt.draw()
                plt.pause(1e-3)'''
            self.get_logger().info("{} - t1:{}".format(self.i, self.t1))
            self.i+=1
        else:
            self.get_logger().info("Done!")
            
def main(args=None):
    rclpy.init(args=args)

    send = SendNode()

    rclpy.spin(send)

    send.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
