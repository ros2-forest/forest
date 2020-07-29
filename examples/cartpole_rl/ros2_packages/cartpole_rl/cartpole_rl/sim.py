import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile


import numpy as np
import matplotlib.pyplot as plt
import time
import gym
import random
import os

from forest_cartpole_rl_interface.msg import FpgaIn
from forest_cartpole_rl_interface.msg import FpgaOut

import time

class SimNode(Node):

    def __init__(self):
        super().__init__('sim')
        profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE, durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE)
        self.publisher_ = self.create_publisher(FpgaIn, 'fpga_in_topic', profile)
        self.subscription = self.create_subscription(FpgaOut,'fpga_out_topic',self.sub, profile)
        self.subscription
        self.ep = 0
        self.t = 0
        self.render_env()

    def render_env(self):
        if self.ep == 0 and self.t == 0:
            self.env = gym.make('CartPole-v1')
        
        if self.ep == 1:
            self.env.close()
            print("Simulation done!")
        else:
            if self.t == 0:
                self.state = self.env.reset()
            self.t+=1
            self.env.render()
            self.update_env()

    def update_env(self):
        msg = FpgaIn()
        msg.state = np.reshape(self.state, (4,)).astype('float32')
        self.t1 = time.time()
        self.publisher_.publish(msg)
        #self.get_logger().info("t1:{}".format(self.t1))

    def sub(self, msg):
        self.t4 = time.time()
        action = np.argmax(msg.q_vals)
        self.state, reward, done, info = self.env.step(action)
        #self.get_logger().info("t4:{}".format(self.t4))
        if done:
            print("Episode finished after {} timesteps".format(self.t))
            self.t = 0
            self.ep+=1
        self.render_env()
        

def main(args=None):
    rclpy.init(args=args)

    sim = SimNode()

    rclpy.spin(sim)

    sim.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
