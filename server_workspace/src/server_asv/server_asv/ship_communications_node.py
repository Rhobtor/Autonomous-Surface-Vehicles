
#TODO: Dame tiene que reformarlo


#ros libraries
import rclpy #main librarie
from rclpy.node import Node #for defining a node
from .submodulos.MQTT import MQTT
from .submodulos.call_service import call_service,call_service_extern

from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import GlobalPositionTarget
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float64  
# Import function to transform quaternion to euler
from .submodulos.quaternion_to_euler import quaternion_to_euler
import numpy
from datetime import datetime
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import json 
import rclpy
from time import sleep 
import traceback


class ShipCommunicationNode(Node):

    def initialize_parameters(self):
        # Declare some parameters #
        self.declare_parameter('internet_loss_timeout', 30.0)
        self.internet_loss_timeout = self.get_parameter('internet_loss_timeout').get_parameter_value().integer_value
        self.declare_parameter('mqtt_addr', "adress")
        self.mqtt_addr = self.get_parameter('mqtt_addr').get_parameter_value().string_value
        self.declare_parameter('mqtt_user', "user")
        self.mqtt_user = 'server'
        self.declare_parameter('mqtt_password', "password")
        self.mqtt_password = self.get_parameter('mqtt_password').get_parameter_value().string_value

    def declare_topics(self):

        # This node connects to the following topics
        # 1) The state of the vehicle /mavros/state
        # 2) The battery /mavros/battery
        # 3) The start_asv topic /start_asv
        # 4) The wp_target topic /wp_target
        # 5) The position of the ASV /asv_position
        # 6) The wp_clear topic /wp_clear

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
        qos_profile_REL=rclpy.qos.QoSProfile(
			depth=1,
			reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
			durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
			history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST
			)

        # Subscriptions

        # Publications


    def declare_services(self):
        """ Services that this node offers and subscribes to."""
        pass



    def __init__(self):
        super().__init__('communication_node')

        # Get parameters
        self.initialize_parameters()
        # Declare subscribers
        self.declare_topics()
        # Declare services
        self.declare_services()


        # Declare MQTT topics
        topics = [

        ]
        
        for topic in topics:
            self.get_logger().info(f"Subscribing to {topic}")

        try:
            self.mqttConnection = MQTT(name="server", 
                                        addr=self.mqtt_addr, 
                                        user=self.mqtt_user, 
                                        password=self.mqtt_password, 
                                        on_message=self.on_message, 
                                        on_disconnect=self.on_disconnect,
                                        topics2suscribe=topics
                                        )
        except ConnectionRefusedError:
            self.get_logger().error(f"Connection to MQTT server was refused")
            self.get_logger().fatal("MQTT module is dead")
            self.destroy_node()
        except OSError:
            self.get_logger().error(f"MQTT server was not found")
            self.get_logger().fatal("MQTT module is dead")
            self.destroy_node()
        except TimeoutError:
            self.get_logger().error(f"MQTT was busy, timeout error")
            self.get_logger().fatal("MQTT module is dead")
            self.destroy_node()
        except:
            error = traceback.format_exc()
            self.get_logger().fatal(f"MQTT connection failed, unknown error:\n {error}")
            self.get_logger().fatal("MQTT module is dead")
            self.destroy_node()

        # Declare timer for publishing data
        self.publishing_timer = self.create_timer(0.5, self.pub_timer_callback)


    def pub_timer_callback(self):
        pass

    def on_disconnect(self,  client,  __, _):

        sleep(1)
        self.get_logger().error("connection to server was lost")

        if not ping_google():
            time_without_internet = 0

            while not ping_google():

                self.get_logger().error("no internet connection, waiting for internet",once=True)
                sleep(1)
                time_without_internet += 1

                if time_without_internet >= self.internet_loss_timeout:
                    self.processing == True
                    self.call_msg.asv_mode = 3 #if we waited for too long, change into manual mode

            self.get_logger().info("Internet connection regained")
        else:
            self.get_logger().error("There is internet, unknown reason, retrying to connect to MQTT")

    def on_message(self, _client, _, msg):
        # This function is called when a message is received

        # Get the topic
        topic = msg.topic
        # Get the payload
        payload = msg.payload.decode("utf-8")

        # Check the topic


def main(args=None):
    #init ROS2
    rclpy.init(args=args)

    #start a class that servers the services
    ship_comms_node = ShipCommunicationNode()
    rclpy.spin(ship_comms_node)

    #after close connection shut down ROS2
    rclpy.shutdown()


if __name__ == '__main__':
    main()
