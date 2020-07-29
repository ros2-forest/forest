import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

import numpy as np
import time

from forest_mnist_cnn_first_interface.msg import FpgaOut

class RecNode(Node):

    def __init__(self):
        super().__init__('receive')
        profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE, durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE)
        self.subscription = self.create_subscription(FpgaOut,'fpga_out_topic',self.read_node_callback, profile)
        self.subscription
        self.i = 0
        self.correct = 0

    def read_node_callback(self, msg):
        self.t4 = time.time()
        self.get_logger().info("Prediction:{}".format(msg.digit[0]))
        #if msg.label == msg.digit[0]:
            #self.correct+=1
        #self.get_logger().info("{} - t4:{}".format(self.i, self.t4))
        self.i+=1
        #self.get_logger().info("Accuracy: {}/{} ({:.2f}%)".format(self.correct, self.i, (self.correct/self.i)*100))


def main(args=None):
    rclpy.init(args=args)

    receive = RecNode()

    rclpy.spin(receive)

    receive.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
