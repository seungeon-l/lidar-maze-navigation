#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan


class ExampleLaser(Node):
	def __init__(self):
		super().__init__('example_laser')
		self.subscription = self.create_subscription(
			LaserScan,
			'scan_filtered',
			self.scan_callback,
			10,
		)

	def scan_callback(self, msg: LaserScan):
		distance_front = msg.ranges[len(msg.ranges) // 2]
		self.get_logger().info(f"Distance front: {distance_front}")


def main(args=None):
	rclpy.init(args=args)
	node = ExampleLaser()
	try:
		rclpy.spin(node)
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
