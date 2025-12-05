#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import threading
import time
from concurrent.futures import Future

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose
from rclpy.node import Node
from scipy.spatial.transform import Rotation

from hybrid_msgs.msg import PosOri

OBSERVE_LOC = Point(x=0.156, y=0.221, z=0.326)  # in m
DROP_LOC = Point(x=-0.300, y=0.00, z=0.200)

# gripper is rotated 22 deg wrt horizontal when picking berries
BERRY_ROT = Point(x=-90.0, y=68.0, z=0.0)  # in degrees
BERRY_OBS_ROT = Point(x=-90.0, y=-18.0, z=0.0)
DROP_ROT = Point(x=-179.5, y=0.0, z=0.0)


class Hybrid(Node):
    def __init__(self):
        super().__init__("hybrid_node")

        self.dance_pub = self.create_publisher(
            PosOri,
            "/cmd_move/dance",
            10,
        )

        self.pose_sub = self.create_subscription(
            Pose,
            "/cmd_move/robot_pose",
            self.pose_callback,
            10,
        )

        self.position = None
        self.orientation = None

        self.get_logger().info("Hybrid node initialized.")

    def send_dance_command(self, msg: PosOri):
        self.dance_pub.publish(msg)
        self.get_logger().info(f"Sent dance command: {msg}")

        while self.position is None or self.orientation is None:
            time.sleep(0.1)

        self.get_logger().info("Waiting for robot to reach target pose...")

        while not np.allclose(
            self.position,
            [msg.position.x, msg.position.y, msg.position.z],
            atol=0.01,
        ) or not np.allclose(
            self.orientation,
            [msg.orientation.x, msg.orientation.y, msg.orientation.z],
            atol=2.0,
        ):
            time.sleep(0.1)

        self.get_logger().info("Robot reached target pose.")

    def pose_callback(self, msg: Pose):
        self.position = [msg.position.x, msg.position.y, msg.position.z]
        quat = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ]

        self.orientation = Rotation.from_quat(quat).as_euler("xyz", degrees=True)

        # self.get_logger().info(
        #     f"Current Position: x={self.position[0]:.3f}, y={self.position[1]:.3f}, z={self.position[2]:.3f}, Current Orientation: roll={self.orientation[0]:.2f}, pitch={self.orientation[1]:.2f}, yaw={self.orientation[2]:.2f}"
        # )


def main():
    rclpy.init()

    hybrid = Hybrid()

    # rclpy.spin(hybrid)

    future = Future()
    spin_thread = threading.Thread(
        target=rclpy.spin_until_future_complete,
        args=(hybrid, future),
        daemon=True,
    )
    spin_thread.start()

    time.sleep(5)  # wait for everything to initialize

    dance_msg = PosOri()
    dance_msg.position.x = DROP_LOC.x
    dance_msg.position.y = DROP_LOC.y
    dance_msg.position.z = DROP_LOC.z
    dance_msg.orientation.x = DROP_ROT.x
    dance_msg.orientation.y = DROP_ROT.y
    dance_msg.orientation.z = DROP_ROT.z
    hybrid.send_dance_command(dance_msg)

    hybrid.get_logger().info("Dance sequence complete. Shutting down.")

    future.set_result(True)
    spin_thread.join()
    hybrid.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
