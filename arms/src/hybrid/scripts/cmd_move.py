#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from move_solver import (
    JOINT_MAX_LIMITS,
    JOINT_MIN_LIMITS,
)
from rclpy.node import Node
from sensor_msgs.msg import JointState

from hybrid_msgs.msg import PosOri
from hybrid_msgs.srv import MoveRequest, PoseRequest

DISABLE_MOVE_LOGGING = False


class CmdMove(Node):
    def __init__(self):
        super().__init__("cmd_move")

        self.joints_sub = self.create_subscription(
            JointState,
            "joint_states",
            self.joints_callback,
            10,
        )
        self.dance_sub = self.create_subscription(
            PosOri,
            "/cmd_move/dance",
            self.dance_callback,
            10,
        )

        self.joints_pub = self.create_publisher(JointState, "joint_goal", 1)
        self.pose_pub = self.create_publisher(Pose, "/cmd_move/robot_pose", 1)

        self.move_solver_client = self.create_client(MoveRequest, "move_solver")
        while not self.move_solver_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("move_solver service not available, waiting...")

        self.move_solver_request = MoveRequest.Request()

        self.pose_solver_client = self.create_client(PoseRequest, "pose_solver")
        while not self.pose_solver_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("pose_solver service not available, waiting...")

        self.pose_solver_request = PoseRequest.Request()

        self.joint_state = None
        self.joint_orders = None

        self.pose_futures = []
        self.solver_futures = []

        self.get_logger().info("CmdMove node initialized.")

    def log(self, message):
        if not DISABLE_MOVE_LOGGING:
            self.get_logger().info(message)

    def joints_callback(self, data: JointState):
        # self.get_logger().info(f"Received joint state: {data}")

        if self.joint_orders is None:
            self.joint_orders = [0] * 6
            for i, name in enumerate(data.name):
                self.joint_orders[i] = int(name.replace("elfin_joint", "")) - 1

        tmp_state = [0] * 6
        for i, pos in enumerate(data.position):
            tmp_state[self.joint_orders[i]] = pos

        self.joint_state = tmp_state

        # self.get_logger().info(f"Processed joint state: {self.joint_state}")

        # pos, ori = self.move_solver.pose_from_joints(self.joint_state)
        self.pose_solver_request.joints = self.joint_state
        self.pose_futures.append(
            self.pose_solver_client.call_async(self.pose_solver_request)
        )

    def dance_callback(self, data: PosOri):
        self.get_logger().info(f"Received dance command: {data}")

        self.move_solver_request.location = data
        self.move_solver_request.starting_joint_state = (
            self.joint_state if self.joint_state is not None else [0.0] * 6
        )

        future = self.move_solver_client.call_async(self.move_solver_request)
        self.solver_futures.append(future)

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            incomplete_pose_futures = []
            for f in self.pose_futures:
                if f.done():
                    response: PoseRequest.Response = f.result()
                    # self.get_logger().info(f"Got response: {response}")

                    self.pose_pub.publish(response.location)

                    # self.get_logger().info(f"Published pose: {pose_msg}")
                else:
                    incomplete_pose_futures.append(f)
            self.pose_futures = incomplete_pose_futures

            incomplete_solver_futures = []
            for f in self.solver_futures:
                if f.done():
                    response: MoveRequest.Response = f.result()
                    self.log(
                        f"Move solver response: valid={response.valid}, joints={response.joints}"
                    )
                    if response.valid:
                        success = self.move_to_joints(response.joints)
                        if success:
                            self.log("Movement successful")
                        else:
                            self.log("Movement failed: could not reach goal")
                    else:
                        self.log("Movement failed: invalid joint solution")
                else:
                    incomplete_solver_futures.append(f)
            self.solver_futures = incomplete_solver_futures

    def move_to_joints(self, joints: list[float]) -> bool:
        self.log(f"Attempted joint state: {joints}")
        joints = np.mod(joints, 2 * np.pi)
        joints = np.array([(j - 2 * np.pi) if j > np.pi else j for j in joints])
        joints = np.clip(joints, JOINT_MIN_LIMITS, JOINT_MAX_LIMITS)
        self.log(f"Final joint state: {joints}")

        js = JointState()
        js.name = [
            "elfin_joint1",
            "elfin_joint2",
            "elfin_joint3",
            "elfin_joint4",
            "elfin_joint5",
            "elfin_joint6",
        ]
        js.position = [float(j) for j in joints]
        js.header.stamp = self.get_clock().now().to_msg()
        self.joints_pub.publish(js)

        self.joint_state = None  # reset joint state to wait for new update
        count = 0
        while count < 1000:
            if self.joint_state is not None:
                joint_state_np = np.array(self.joint_state)
                if np.allclose(joint_state_np, joints, atol=1e-1):
                    self.log("Goal reached")
                    return True
                else:
                    if count % 10 == 0:
                        self.log("Waiting to reach goal...")
                    count += 1

            time.sleep(0.01)
        return False


def main():
    rclpy.init()

    cmd_move = CmdMove()

    cmd_move.spin()

    cmd_move.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
