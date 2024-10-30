import rclpy
from rclpy.node import Node
import os
import numpy as np

class PathPlannerNode(Node):

	def parameters(self):

		self.declare_parameter('debug', True)
		self.debug = self.get_parameter('debug').get_parameter_value().bool_value
					

	def declare_services(self):
		pass

	def declare_topics(self):

		pass

	def __init__(self):
		super().__init__('path_planner_node')

		# Declare parameters, services and topics
		self.parameters()
		self.declare_services()	
		self.declare_topics()
		
def main(args=None):
	rclpy.init(args=args)
	node = PathPlannerNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()
