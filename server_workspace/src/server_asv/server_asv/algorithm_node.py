#ros libraries
import rclpy #main librarie
from rclpy.node import Node #for defining a node
from mavros_msgs.msg import State, WaypointReached, GlobalPositionTarget, Waypoint
from sensor_msgs.msg import BatteryState, NavSatFix
from mavros_msgs.srv import SetMode, WaypointClear, WaypointPush
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from time import sleep #delay

import queue # for queueing the WP


class Algorithm_node(Node):

    def initialize_parameters(self):

        # Get parameters from ROSPARAM
        self.declare_parameter('debug', True)
        self.get_parameter('debug').get_parameter_value().bool_value

    def declare_services(self):

        # This node connects to the following services:
        pass

    def declare_topics(self):

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        qos_profile_BEF = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

    def __init__(self):
        super().__init__('algorithm_node')
        
        # Declare some parameters #
        self.initialize_parameters()

        # Declare Services, Action and Subscribers
        self.declare_services()
        self.declare_topics()



def main(args=None):
    #init ROS2
    rclpy.init(args=args)
    
    #start a class that servers the services
    algorithm_node = Algorithm_node()
    rclpy.spin(algorithm_node)

    #after close connection shut down ROS2
    rclpy.shutdown()


if __name__ == '__main__':
    main()
