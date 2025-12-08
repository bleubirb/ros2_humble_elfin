#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rclpy
from rclpy.node import Node
from pns_driver import PNS_Driver

class GripTestNode(Node):
    def __init__(self):
        super().__init__("grip_test_node")

        ip = self.declare_parameter("ip", "192.168.1.1").get_parameter_value().string_value
        port = self.declare_parameter("port", "502").get_parameter_value().string_value

        self.driver = PNS_Driver(self, ip, port)
        self.get_logger().info(f"Connected to RG2FT at {ip}:{port}")

        self.create_timer(2.0, self.run_test)

        self.test_started = False

    def run_test(self):
        if self.test_started:
            return
        
        self.test_started = True

        force = 0.0
        self.get_logger().info(f"Initializing grip test with force {force}")
        self.driver.set_fd(force)
        time.sleep(2.0)

        force = 0.5
        self.get_logger().info(f"Starting grip test with force {force}")
        self.driver.set_fd(force)

        while not self.driver.get_done():
            self.get_logger().info("Gripping...")
            time.sleep(0.2)
        
        self.get_logger().info("Grip test completed.")

        self.get_logger().info("Releasing grip.")
        self.driver.set_fd(0.0)
        while not self.driver.get_done():
            self.get_logger().info("Releasing...")
            time.sleep(0.2)

        self.get_logger().info("Release completed.")
        self.driver.stop()
        rclpy.shutdown()

def main():
    rclpy.init()
    node = GripTestNode()
    rclpy.spin(node)

if __name__ == "__main__":
    main()