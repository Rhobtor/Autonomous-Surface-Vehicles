#ros libraries
import rclpy #main librarie
from rclpy.node import Node #for defining a node

from .submodulos.MQTT import MQTT
from .submodulos.terminal_handler import ping_google
from .submodulos.asv_identity import get_asv_identity
from zed_interfaces.srv import StartSvoRec
from std_srvs.srv import Trigger
from .submodulos.call_service import call_service,call_service_extern

from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import GlobalPositionTarget
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float64  
from asv_interfaces.msg import SensorMsg
from asv_interfaces.msg import SonarMsg
from asv_interfaces.msg import TrashMsg
# Import function to transform quaternion to euler
from .submodulos.quaternion_to_euler import quaternion_to_euler
import numpy
from datetime import datetime
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import json 
import rclpy
from time import sleep 
import traceback


class ServerCommunicationNode(Node):

    def initialize_parameters(self):

        # Declare some parameters #
        self.declare_parameter('internet_loss_timeout', 30.0)
        self.internet_loss_timeout = self.get_parameter('internet_loss_timeout').get_parameter_value().integer_value
        self.declare_parameter('mqtt_addr', "adress")
        self.mqtt_addr = self.get_parameter('mqtt_addr').get_parameter_value().string_value
        self.declare_parameter('mqtt_user', "user")
        self.vehicle_id = get_asv_identity()
        self.mqtt_user = 'asv' + str(get_asv_identity())
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
        self.asv_state_subscription = self.create_subscription(State, '/mavros/state', self.asv_state_callback, qos_profile)
        self.asv_battery_subscription = self.create_subscription(BatteryState, '/mavros/battery', self.asv_battery_callback, qos_profile_BEF)
        self.asv_position_subscription = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.asv_position_callback, qos_profile_BEF)
        self.asv_orientation_subscription = self.create_subscription(Float64, '/mavros/global_position/compass_hdg', self.asv_orientation_callback, qos_profile_BEF)

        # Subscriptions to the sensors
        self.wqp_sensor_subscription = self.create_subscription(SensorMsg, '/wqp_measurements', self.wqp_sensor_callback, qos_profile_BEF)
        # Subscriptions to the sonar
        self.sonar_sensor_subscription = self.create_subscription(SonarMsg, '/sonar_measurements', self.sonar_sensor_callback, qos_profile_BEF)
        # Subscriptions to trash point
        self.trash_point_subscription = self.create_subscription(TrashMsg, '/zed2i_trash_detections/trash_localization', self.trash_point_callback, qos_profile_REL)

        # Publications
        self.start_asv_publisher = self.create_publisher(Bool, '/start_asv', qos_profile)
        self.wp_target_publisher = self.create_publisher(GlobalPositionTarget, '/wp_target', qos_profile)
        self.wp_clear_publisher = self.create_publisher(Bool, '/wp_clear', qos_profile)

    def declare_services(self):
        """ Services that this node offers and subscribes to."""

        # Service to change the mode of the ASV
        self.set_mode_srv = self.create_client(SetMode, '/mavros/set_mode')
        self.camera_recording_client = self.create_client(StartSvoRec, '/zed/zed_node/start_svo_rec')
        self.camera_stop_recording_client = self.create_client(Trigger, '/zed/zed_node/stop_svo_rec')

    def __init__(self):
        super().__init__('communication_node')

        # Get parameters
        self.initialize_parameters()
        # Declare subscribers
        self.declare_topics()
        # Declare services
        self.declare_services()

        self.topic_identity = 'asv/asv' + str(self.vehicle_id)

        self.asv_mode = "MANUAL"
        self.battery = -1
        self.asv_position = {'latitude': 0, 'longitude': 0, 'heading': 0}


        # Declare MQTT topics
        topics = [self.topic_identity + '/start_asv', 
                self.topic_identity + '/wp_clear', 
                self.topic_identity + '/wp_target',
                self.topic_identity + '/asv_mode',
                self.topic_identity + '/camerarecord_on',
                self.topic_identity + '/camerarecord_off']
        
        for topic in topics:
            self.get_logger().info(f"Subscribing to {topic}")

        try:
            self.mqttConnection = MQTT(name=self.topic_identity, 
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

        # Spin forever
        while rclpy.ok():
            rclpy.spin_once(self)
            sleep(0.1)


    def pub_timer_callback(self):
        # When the timer is triggered, publish the data

        # Publish the complete state of the ASV
        self.mqttConnection.send_new_msg(json.dumps({
            'mode': self.asv_mode,
            'battery': self.battery,
            'Latitude': self.asv_position['latitude'],
            'Longitude': self.asv_position['longitude'],
            'Heading': self.asv_position['heading'],
            'veh_num': self.vehicle_id,
            'date': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }), topic = self.topic_identity + '/asv_state')

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
        if topic == self.topic_identity + '/start_asv':
            # If the topic is /asv_start, send the start command trough the topic
            msg = Bool()
            msg.data = payload == 1
            self.start_asv_publisher.publish(msg)
            self.get_logger().info("Start ASV command received")

        elif topic == self.topic_identity + '/wp_clear':
            # If the topic is /wp_clear, send the clear command trough the topic
            msg = Bool()
            msg.data = payload == 1
            self.wp_clear_publisher.publish(msg)
            self.get_logger().info("Clean WPs command received")

        elif topic == self.topic_identity + '/wp_target':
            # If the topic is /wp_target, send the WP command trough the topic
            # Format the payload to a dict (From json)
            payload = json.loads(payload)
            self.get_logger().info("New WP received: " + str(payload))
            msg = GlobalPositionTarget()
            try:
                msg.latitude = payload['latitude']
                msg.longitude = payload['longitude']
                # Publish the WP
                self.wp_target_publisher.publish(msg)
            except KeyError:
                self.get_logger().error("The payload of the requested WP is not correct")

        elif topic == self.topic_identity + '/asv_mode':
            # Call the service to change the mode of the ASV
            # Check if the mode is correct
            # Create the request
            request = SetMode.Request()
            request.custom_mode = payload
            # Call the service
            self.set_mode_srv.call_async(request)
            self.get_logger().info("Change mode command received: " + payload)
        
        elif topic == self.topic_identity + '/camerarecord_on':
            current_date = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            # Construct the filename with the current date
            filename = f"/root/CameraRecord/ASV_{current_date}.svo"
            self.message_zed = {
                    'svo_filename': filename,
                    'compression_mode': 2,
                    'target_framerate': 30,
                    'bitrate': 6000
                }
            self.get_logger().info("start record")
            call_service_extern(self, self.camera_recording_client, self.message_zed)
        
        elif topic == self.topic_identity + '/camerarecord_off':
            self.message_stop={}
            self.get_logger().info("stop record")
            call_service_extern(self, self.camera_stop_recording_client, self.message_stop)
        else:
            self.get_logger().error("The topic " + topic + " is not recognized")
            self.get_logger().error("The payload is " + str(payload))

    def asv_state_callback(self, msg):
        # This function is called when the state topic is updated
        self.asv_mode = msg.mode
    
    def asv_battery_callback(self, msg):
        # This function is called when the battery topic is updated
        self.battery = msg.percentage

    def asv_position_callback(self, msg):

        self.asv_position['latitude'] = msg.latitude
        self.asv_position['longitude'] =  msg.longitude

    def asv_orientation_callback(self, msg):
         
        self.asv_position['heading'] = msg.data

    def wqp_sensor_callback(self, msg):
        # This function is called when the wqp_sensor topic is updated
        # Check if the message is correct
        if msg.success:
            # If the message is correct, send the message to the MQTT server

            # Format the json msg
            json_msg = json.dumps({
                'conductivity': msg.conductivity,
                'temperature_ct': msg.temperature_ct,
                'turbidity': msg.turbidity,
                'ph': msg.ph,
                'vbat': msg.vbat,
                'lat': msg.lat,
                'lon': msg.lon,
                'date': msg.date,
                'veh_num': self.vehicle_id
            })

            # Send the message
            self.mqttConnection.send_new_msg(json_msg, topic = '/database/wqp')
        else:
            self.get_logger().error("The message received from the WQP sensor is not correct")

    def sonar_sensor_callback(self, msg):
        # This function is called when the sonar_sensor topic is updated
        if msg.success:
            
            json_msg = json.dumps({
                'measurement': msg.distance,
                'Latitude': msg.lat,
                'Longitude': msg.lon,
                'veh_num': self.vehicle_id,
                'date': msg.date
            })

            self.mqttConnection.send_new_msg(json_msg, topic = '/database/sonar')
        else:
            self.get_logger().error("The message received from the sonar sensor is not correct")

    def trash_point_callback(self, msg):
        # This function is called when the sonar_sensor topic is updated
        if msg.success:
            if not isinstance(msg.object_lat,(int, float)) or numpy.isnan(msg.object_lat) :
                return 

            json_msg = json.dumps({
                'Latitude_Drone': msg.drone_lat,
                'Longitude_Drone': msg.drone_lon,
                'Latitude_Obj': msg.object_lat,
                'Longitude_Obj':msg.object_lon,
                'Heading_Drone':msg.drone_heading,
                'Class':msg.trash,
                'Heading_Obj':msg.object_heading,
                'veh_num': self.vehicle_id,
                'date': msg.date
            })

            self.mqttConnection.send_new_msg(json_msg, topic = '/database/trash')
        else:
            self.get_logger().error("The message received from the sonar sensor is not correct")

def main(args=None):
    #init ROS2
    rclpy.init(args=args)

    #start a class that servers the services
    server_comms_node = ServerCommunicationNode()
    server_comms_node.destroy_node()

    #after close connection shut down ROS2
    rclpy.shutdown()


if __name__ == '__main__':
    main()
