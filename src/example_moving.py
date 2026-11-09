#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

from create_fundamentals.msg import SensorPacket
from create_fundamentals.srv import DiffDrive, ResetEncoders


class ExampleMoving(Node):
    def __init__(self):
        super().__init__("example_moving")

        self.encoder_l = 0.0
        self.encoder_r = 0.0

        self.create_subscription(SensorPacket, "sensor_packet", self.sensor_packet_callback, 10)

        self.get_logger().info("Waiting for create driver ...")
        self.diff_drive_client = self.create_client(DiffDrive, "diff_drive")
        self.reset_encoders_client = self.create_client(ResetEncoders, "reset_encoders")

        while not self.diff_drive_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for diff_drive service...")
        while not self.reset_encoders_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for reset_encoders service...")

        self.get_logger().info("Create driver is ready!")

    def sensor_packet_callback(self, msg: SensorPacket):
        self.encoder_l = msg.encoder_left
        self.encoder_r = msg.encoder_right

    def call_reset_encoders(self):
        request = ResetEncoders.Request()
        future = self.reset_encoders_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def call_diff_drive(self, left, right):
        request = DiffDrive.Request()
        request.left = float(left)
        request.right = float(right)
        future = self.diff_drive_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    node = ExampleMoving()

    node.get_logger().info("Differential drive demo ...")

    time.sleep(1.0)
    node.call_reset_encoders()
    time.sleep(1.0)
    node.get_logger().info(f"encoder_l: {node.encoder_l}, encoder_r: {node.encoder_r}")

    node.call_diff_drive(10.0, 0.0)
    time.sleep(1.0)
    node.call_diff_drive(0.0, 0.0)
    time.sleep(1.0)
    node.get_logger().info(f"encoder_l: {node.encoder_l}, encoder_r: {node.encoder_r}")

    node.call_diff_drive(0.0, 10.0)
    time.sleep(1.0)
    node.call_diff_drive(0.0, 0.0)
    time.sleep(1.0)
    node.get_logger().info(f"encoder_l: {node.encoder_l}, encoder_r: {node.encoder_r}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
