import rclpy
from rclpy.node import Node

from forest_img_hist_interface.msg import FpgaOut
from forest_img_hist_interface.msg import FpgaIn


class ListenerNode(Node):

    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(FpgaOut,'fpga_out_topic',self.read_node_callback, 10)
        self.subscription

    def read_node_callback(self, msg):
        self.get_logger().info("HW result: {}".format(msg))

    def read_node_callback_sw(self, msg):
        sw_result = self.get_hist_sw(msg.image)
        print("SW result: {}".format(sw_result))

    def get_hist_sw(self, img):
        hist_sw = [0]*256
        for i in range(len(img)):
            hist_sw[img[i]] += 1
        return hist_sw


def main(args=None):
    rclpy.init(args=args)

    listener = ListenerNode()

    rclpy.spin(listener)

    listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
