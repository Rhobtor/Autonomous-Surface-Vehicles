
import sys
import rclpy
from rclpy.node import Node
import time
from datetime import datetime
import random
from math import exp, sin, cos

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class Sonar_node(Node):

    def parameters(self):

        self.declare_parameter('debug', True)
        self.DEBUG = self.get_parameter('debug').get_parameter_value().bool_value
        

    def declare_topics(self):

        qos_profile_BEF = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

    def __init__(self):
        super().__init__("sonar_service")

        self.parameters()

        self.declare_topics()

def main(args=None):
    #init ROS2
    rclpy.init(args=args)
    
    sonar_node = Sonar_node()
    rclpy.spin(sonar_node)

    # Destroy the node explicitly
    sonar_node.free_resources()
    rclpy.shutdown()


if __name__ == '__main__':
    main()