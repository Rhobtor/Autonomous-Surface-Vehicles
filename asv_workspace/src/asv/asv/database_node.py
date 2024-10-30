import rclpy #main librarie
from rclpy.node import Node #for defining a node

from .submodulos.database import MYSQL
from asv_interfaces.msg import SensorMsg
from asv_interfaces.msg import SonarMsg
from asv_interfaces.msg import TrashMsg

import numpy
from datetime import datetime
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
 
import rclpy
from time import sleep 


class DatabaseComunicationNode(Node):
    def initialize_parameters(self):
        self.declare_parameter('db_host', "db_host")
        self.host_addr = self.get_parameter('db_host').get_parameter_value().string_value
        self.declare_parameter('database', "database")
        self.database = self.get_parameter('database').get_parameter_value().string_value
        self.declare_parameter('db_user', "db_user")
        self.db_user = self.get_parameter('db_user').get_parameter_value().string_value
        self.declare_parameter('db_password', "db_password")
        self.db_password = self.get_parameter('db_password').get_parameter_value().string_value
        self.declare_parameter('db_port', "db_port")
        self.db_port = self.get_parameter('db_port').get_parameter_value().string_value

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
        qos_profile_REL=rclpy.qos.QoSProfile(
			depth=1,
			reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
			durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
			history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST
			)



        # Subscriptions to the sensors
        self.wqp_sensor_subscription = self.create_subscription(SensorMsg, '/wqp_measurements', self.wqp_sensor_callback, qos_profile_BEF)
        # Subscriptions to the sonar
        self.sonar_sensor_subscription = self.create_subscription(SonarMsg, '/sonar_measurements', self.sonar_sensor_callback, qos_profile_BEF)
        # Subscriptions to trash point
        self.trash_point_subscription = self.create_subscription(TrashMsg, '/zed2i_trash_detections/trash_localization', self.trash_point_callback, qos_profile_REL)



    def __init__(self):
            super().__init__('database_node')

            # Get parameters
            self.initialize_parameters()
            # Declare subscribers
            # Declare subscribers
            self.declare_topics()

            try:
                    # Establish database connection instance
                    self.database_connection = MYSQL(
                        host=self.host_addr,
                        database=self.database,
                        user=self.db_user,
                        password=self.db_password,
                        port=self.db_port
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

            self.vehicle_id = 1

            # Spin forever
            while rclpy.ok():
                rclpy.spin_once(self)
                sleep(0.1)

    # def asv_position_callback(self, msg):

    #     self.asv_position['latitude'] = msg.lat
    #     self.asv_position['longitude'] =  msg.lon

    def wqp_sensor_callback(self, msg):
      
        if msg.success:
            date = msg.date  # Ensure this is in the expected format
            parsed_date = datetime.strptime(date, '%d/%m/%Y %H:%M:%S')
            formatted_date = parsed_date.strftime('%Y-%m-%d %H:%M:%S')
            if msg.ph !=0:

                query = ("INSERT INTO WQP (Date, Latitude, Longitude, Sensor, ASV, Data) "
                        "VALUES (%s, %s, %s, %s, %s, %s)")
                data = (formatted_date, msg.lat, msg.lon, "Ph", self.vehicle_id, msg.ph)
                self.database_connection.insert_data_to_db(query, data)

            if msg.vbat !=0:

                query = ("INSERT INTO WQP (Date, Latitude, Longitude, Sensor, ASV, Data) "
                        "VALUES (%s, %s, %s, %s, %s, %s)")
                data = (formatted_date, msg.lat, msg.lon, "Batery", self.vehicle_id, msg.vbat)
                self.database_connection.insert_data_to_db(query, data)

            if msg.turbidity !=0:

                query = ("INSERT INTO WQP (Date, Latitude, Longitude, Sensor, ASV, Data) "
                        "VALUES (%s, %s, %s, %s, %s, %s)")
                data = (formatted_date, msg.lat, msg.lon, "Turbidity", self.vehicle_id, msg.turbidity)
                self.database_connection.insert_data_to_db(query, data)

            if msg.temperature_ct !=0:

                query = ("INSERT INTO WQP (Date, Latitude, Longitude, Sensor, ASV, Data) "
                        "VALUES (%s, %s, %s, %s, %s, %s)")
                data = (formatted_date, msg.lat, msg.lon, "Temperature_ct", self.vehicle_id, msg.temperature_ct)
                self.database_connection.insert_data_to_db(query, data)

            if msg.conductivity !=0 :

                query = ("INSERT INTO WQP (Date, Latitude, Longitude, Sensor, ASV, Data) "
                        "VALUES (%s, %s, %s, %s, %s, %s)")
                data = (formatted_date, msg.lat, msg.lon, "Conductivity", self.vehicle_id, msg.conductivity)
                self.database_connection.insert_data_to_db(query, data)



    def sonar_sensor_callback(self, msg):
        if msg.success:
            date = msg.date  # Ensure this is in the expected format
            parsed_date = datetime.strptime(date, '%d/%m/%Y %H:%M:%S')
            formatted_date = parsed_date.strftime('%Y-%m-%d %H:%M:%S')

            query = ("INSERT INTO WQP (Date, Latitude, Longitude, Sensor, ASV, Data) "
                    "VALUES (%s, %s, %s, %s, %s, %s)")
            data = (formatted_date, msg.lat, msg.lon, "Sonar", self.vehicle_id, msg.distance)
            self.database_connection.insert_data_to_db(query, data)
            #self.get_logger().info('Data inserted successfully into the database.')


    def trash_point_callback(self, msg):
        if msg.success and isinstance(msg.object_lat, (int, float)) and not numpy.isnan(msg.object_lat):
       
            query = ("INSERT INTO TRASH (Date, Latitude_Drone, Longitude_Drone, Latitude_Obj, Longitude_Obj, "
                    "Heading_Drone, Heading_Obj, ASV, Class) "
                    "VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s)")
            data = (msg.date, msg.drone_lat, msg.drone_lon, msg.object_lat, msg.object_lon,
                    msg.drone_heading, msg.object_heading, self.vehicle_id,msg.trash)
            self.database_connection.insert_data_to_db(query, data)
            #self.get_logger().info('Data inserted successfully into the database.')

def main(args=None):
    #init ROS2
    rclpy.init(args=args)

    #start a class that servers the services
    server_comms_node = DatabaseComunicationNode()
    server_comms_node.destroy_node()

    #after close connection shut down ROS2
    rclpy.shutdown()


if __name__ == '__main__':
    main()


