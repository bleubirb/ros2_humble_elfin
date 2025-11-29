#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time

import numpy as np
from geometry_msgs.msg import Pose
from move_solver import (
    JOINT_MAX_LIMITS,
    JOINT_MIN_LIMITS,
)
from sensor_msgs.msg import JointState

DISABLE_MOVE_LOGGING = False


class CmdMove(object):
    def __init__(self, node, move_solver):
        self.node = node
        self.move_solver = move_solver

        self.joints_sub = node.create_subscription(
            JointState,
            "/joint_states",
            self.joints_callback,
            10,
        )
        self.joints_pub = node.create_publisher(JointState, "joint_goal", 1)
        self.pose_pub = node.create_publisher(Pose, "hybrid/robot_pose", 1)

        self.joint_state = None
        self.joint_orders = None

    def log(self, message):
        if not DISABLE_MOVE_LOGGING:
            self.node.get_logger().info(message)

    def joints_callback(self, data):
        if self.joint_orders is None:
            self.joint_orders = [0] * 6
            for i, name in enumerate(data.name):
                self.joint_orders[i] = int(name.replace("elfin_joint", "")) - 1

        tmp_state = [0] * 6
        for i, pos in enumerate(data.position):
            tmp_state[self.joint_orders[i]] = pos

        self.joint_state = tmp_state

        pos, ori = self.move_solver.pose_from_joints(self.joint_state)

        # self.log(f"Current end-effector pose: pos={pos}, ori={ori}")

        pose_msg = Pose()
        pose_msg.position.x = pos[0]
        pose_msg.position.y = pos[1]
        pose_msg.position.z = pos[2]

        pose_msg.orientation.x = ori[0]
        pose_msg.orientation.y = ori[1]
        pose_msg.orientation.z = ori[2]
        pose_msg.orientation.w = ori[3]

        self.pose_pub.publish(pose_msg)

        # self.log(f"Received joint state: {self.joint_state}")

    def dance(self, joints):
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
        js.header.stamp = self.node.get_clock().now().to_msg()
        # time.sleep(0.5)
        self.joints_pub.publish(js)
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
                        # self.log(f"Current joint state: {joint_state_np}")
                        # self.log(f"Target joint state: {joints}")
                    count += 1

            time.sleep(0.01)
        # time.sleep(0.5)
        return False
