# Import ROS python and Node functionalities
import rclpy
from rclpy.node import Node

import numpy as np
import time

# Import the messages defined for the ROS FPGA Node interface
from forest_cs_256_64b_interface.msg import FpgaOut
from forest_cs_256_64b_interface.msg import FpgaIn

class PcSwNode(Node):

    def __init__(self):
        super().__init__('pc_sw_node')
        # Publish to the fpga_out_topic topic, message type is FpgaOut
        self.publisher_ = self.create_publisher(FpgaOut, 'fpga_out_topic', 10)
        # Subscribe to the fpga_in_topic topic, message type is FpgaIn
        self.subscription = self.create_subscription(FpgaIn, 'fpga_in_topic', self.sw_sub_callback, 10)
        self.subscription
        self.N_ROWS = 256
        self.N_COLS = 256
        self.out_img = np.zeros((self.N_ROWS*self.N_COLS,), dtype=np.uint8)

    def sw_pub_callback(self):
        msg = FpgaOut()
        for i in range(self.N_ROWS*self.N_COLS//8):
            out_hi = np.uint32(np.left_shift(self.out_img[8*i+7], 32-1*8) | np.left_shift(self.out_img[8*i+6], 32-2*8) | np.left_shift(self.out_img[8*i+5], 32-3*8) | self.out_img[8*i+4]) 
            out_lo = np.uint32(np.left_shift(self.out_img[8*i+3], 32-1*8) | np.left_shift(self.out_img[8*i+2], 32-2*8) | np.left_shift(self.out_img[8*i+1], 32-3*8) | self.out_img[8*i]) 
            msg.image_out[i] = np.uint64(np.left_shift(out_hi, 32) | out_lo)
        self.publisher_.publish(msg)
        self.t3 = time.time()
        self.get_logger().info("t2:{} t3:{}".format(self.t2, self.t3))

    def sw_sub_callback(self, msg):
        self.t2 = time.time()
        image_in = np.zeros((self.N_ROWS*self.N_COLS,), dtype=np.uint8)
        for i in range(self.N_ROWS*self.N_COLS//8):
            for j in range(8):
                image_in[8*i+j] = np.uint8(np.right_shift(msg.image_in[i],np.uint64(8*j)))
        in_hist = self.histogram(image_in)
        c, d = self.get_c_d(in_hist)
        self.do_stretch(c, d, image_in)
        self.sw_pub_callback()

    def histogram(self, img_data):
        hist = [0]*256
        for i in range(len(img_data)):
            hist[img_data[i]]+=1
        return hist
    
    def get_c_d(self, hist):
        c = 0
        d = 255
        low_perc_count = int((1*self.N_ROWS*self.N_COLS)/100)
        hi_perc_count = int((99*self.N_ROWS*self.N_COLS)/100)

        cumulative_hist = 0   
        for i in range(0, 256):
            cumulative_hist += hist[i]
            if (cumulative_hist > low_perc_count):
                c = i
                break

        cumulative_hist = self.N_ROWS*self.N_COLS
        for i in range(255, 0, -1):
            cumulative_hist -= hist[i]
            if (cumulative_hist < hi_perc_count):
                d = i
                break
        
        return c, d

    def do_stretch(self, c, d, img_data):
        for i in range(self.N_ROWS*self.N_COLS):
            if (img_data[i] < c):
                self.out_img[i] = 0
            elif (img_data[i] > d):
                self.out_img[i] = 255
            else:
                self.out_img[i] = np.uint8((img_data[i]-c)*255/(d-c))

def main(args=None):
    rclpy.init(args=args)

    pc_sw_node = PcSwNode()

    rclpy.spin(pc_sw_node)

    pc_sw_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
