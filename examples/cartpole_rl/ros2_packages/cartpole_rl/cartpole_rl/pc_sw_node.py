# Import ROS python and Node functionalities
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from tensorflow import keras
from tensorflow.keras.datasets import mnist
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Dropout, Flatten
from tensorflow.keras.layers import Conv2D, MaxPooling2D
from tensorflow.keras import backend as K
from keras.models import model_from_json
import os
import numpy as np
import time

# Import the messages defined for the ROS FPGA Node interface
from forest_cartpole_rl_interface.msg import FpgaOut
from forest_cartpole_rl_interface.msg import FpgaIn

class PcSwNode(Node):

    def __init__(self):
        super().__init__('pc_sw_node')
        profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE, durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE)
        # Publish to the fpga_out_topic topic, message type is FpgaOut
        self.publisher_ = self.create_publisher(FpgaOut, 'fpga_out_topic', profile)
        # Subscribe to the fpga_in_topic topic, message type is FpgaIn
        self.subscription = self.create_subscription(FpgaIn, 'fpga_in_topic', self.sw_sub_callback, profile)
        self.subscription
        os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
        json_file = open('/home/danielpi/Desktop/work/case_studies/cartpole_rl/scripts/architecture.json','r')
        loaded_model_json = json_file.read()
        json_file.close()
        self.model = model_from_json(loaded_model_json)
        self.model.load_weights("/home/danielpi/Desktop/work/case_studies/cartpole_rl/scripts/weights.h5")

    def sw_pub_callback(self):
        msg = FpgaOut()
        msg.q_vals = self.q_vals
        self.t3 = time.time()
        self.publisher_.publish(msg)
        self.get_logger().info("t_hls:{}".format(self.t3-self.t2))

    def sw_sub_callback(self, msg):
        self.t2 = time.time()
        state = np.reshape(msg.state, [1, 4])
        self.q_vals = self.model.predict(state)[0]
        self.sw_pub_callback()

def main(args=None):
    rclpy.init(args=args)

    pc_sw_node = PcSwNode()

    rclpy.spin(pc_sw_node)

    pc_sw_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
