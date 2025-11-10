#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import threading
import time
from asyncio import Future

import rclpy
from cmd_move import CmdMove
from move_solver import (
    Action,
    JointAction,
    MoveSolver,
)
from pns_driver import PNS_Driver

from vision_msgs.msg import Berries
from vision_msgs.srv import Ready

OBSERVE_LOC = [0.053, 0.500, 0.4546]  # in m
DROP_LOC = [-0.300, 0.00, 0.200]

# gripper is rotated 22 deg wrt horizontal when picking berries
BERRY_ROT = [-90, 68, 0]  # in degrees
DROP_ROT = [-179.5, 0, 0]


def pick_seq(position: list[float]) -> list[JointAction]:
    actions = []
    offset = 0.1  # meters

    force = 0.8  # for berry
    # force = 3.0 # for ball

    # approach (-y)
    actions.append(
        JointAction(
            Action.MOVE,
            position=[position[0], position[1] - offset, position[2]],
            orientation=BERRY_ROT,
        )
    )
    actions.append(JointAction(Action.MOVE, position=position, orientation=BERRY_ROT))

    # grip
    actions.append(JointAction(Action.GRIP, force=force))

    # pull down (-z) and back (-y)
    actions.append(
        JointAction(
            Action.MOVE,
            position=[position[0], position[1], position[2] - offset],
            orientation=BERRY_ROT,
        )
    )
    actions.append(
        JointAction(
            Action.MOVE,
            position=[position[0], position[1] - offset, position[2] - offset],
            orientation=BERRY_ROT,
        )
    )

    return actions


def drop_seq(position: list[float]) -> list[JointAction]:
    actions = []

    raised_pos = [position[0], position[1], position[2] + 0.15]
    force = 0.0  # release

    # approach from raised pos
    actions.append(JointAction(Action.MOVE, position=raised_pos, orientation=DROP_ROT))
    actions.append(JointAction(Action.MOVE, position=position, orientation=DROP_ROT))

    # drop
    actions.append(JointAction(Action.GRIP, force=force))

    # return to raised pos
    actions.append(JointAction(Action.MOVE, position=raised_pos, orientation=DROP_ROT))

    return actions


def handle_action(action: JointAction, ms: MoveSolver, cm: CmdMove):
    log_str = ""
    if action.action == Action.MOVE:  # MOVE action
        node.get_logger().info(
            f"Moving to {action.position} with orientation {action.orientation}"
        )

        # log the move action, either as all zeros or with the specified position and orientation
        if action.position is None:
            log_str = f"{time.time()},{Action.MOVE},{','.join(['0']*6)},0\n"
        else:
            log_str = f"{time.time()},{Action.MOVE},{','.join([str(x) for x in action.position])},{','.join([str(x) for x in action.orientation])},0\n"

        joints = ms.move(
            action.position,
            action.orientation,
            cm.joint_state or [0, 0, 0, 0, 0, 0],
        )

        # execute the joint action
        cm.dance(joints)
    else:  # GRIP action
        node.get_logger().info(f"Setting desired force to {action.force} for {i+1}")
        log_str = f"{time.time()},{Action.GRIP},{','.join(['0']*6)},{action.force}\n"

        driver.set_fd(action.force)

        count = 0
        while not driver.get_done():
            if count % 10 == 0:
                node.get_logger().info(
                    f"Waiting for gripper to reach desired force for {i+1}"
                )
            count += 1
            time.sleep(0.5)

    return log_str


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node("hybrid_node")
    ip = node.declare_parameter("ip", "192.168.1.1").get_parameter_value().string_value
    port = node.declare_parameter("port", "502").get_parameter_value().string_value

    ms = MoveSolver(node)
    cm = CmdMove(node, ms)
    driver = PNS_Driver(node, ip, port)

    # spin required for subscriptions to work
    future = Future()
    spin_thread = threading.Thread(
        target=rclpy.spin_until_future_complete, args=(node, future), daemon=True
    )
    spin_thread.start()

    time.sleep(1)

    HOME = JointAction(Action.MOVE, position=None, orientation=None)

    actions: list[JointAction] = [
        JointAction(Action.GRIP, force=0),
        HOME,
        JointAction(Action.MOVE, position=OBSERVE_LOC, orientation=[-90, 68, 0]),
        # *pick_seq(berry_loc),
        # *drop_seq(drop_loc),
        # HOME,
    ]

    if not os.path.exists("data"):
        os.makedirs("data")
    f_idx = 0
    while os.path.exists(f"data/hybrid_state_{f_idx}.csv"):
        f_idx += 1
    log_file = open(f"data/hybrid_state_{f_idx}.csv", "w")
    log_file.write("Time,State,X,Y,Z,RX,RY,RZ,F\n")

    for i, action in enumerate(actions):
        node.get_logger().info(f"Executing action {i+1}/{len(actions)}")

        log_file.write(handle_action(action, ms, cm))

        node.get_logger().info(f"Finished action {i+1}/{len(actions)}")

    node.create_client(Ready, "/vision/set_ready").call_async(
        Ready.Request(ready_req=True)
    )

    def save_poses(msg: Berries):
        node.poses = msg.berries

    poses_sub = node.create_subscription(
        Berries,
        "/vision/detected_poses",
        save_poses,
        1,
    )

    count = 0
    while not hasattr(node, "poses"):
        if count % 10 == 0:
            node.get_logger().info("Waiting for berry poses...")
        count += 1
        time.sleep(0.5)

    for j, berry in enumerate(node.poses):
        berry_loc = [
            berry.pose.x,
            berry.pose.y,
            berry.pose.z,
        ]
        node.get_logger().info(f"Berry {j+1}: {berry_loc}")

        pick_actions = pick_seq(berry_loc)
        drop_actions = drop_seq(DROP_LOC)

        for k, action in enumerate(pick_actions + drop_actions):
            node.get_logger().info(f"Executing action {k+1}/{len(actions)}")

            log_file.write(handle_action(action, ms, cm))

            node.get_logger().info(f"Finished action {k+1}/{len(actions)}")

    log_file.close()
    node.get_logger().info(f"Data saved to data/hybrid_state_{f_idx}.csv")
    driver.stop()
    node.get_logger().info("It worked!")

    # while True:
    #     time.sleep(1)

    # cleanup
    future.set_result(True)
    spin_thread.join()
    rclpy.shutdown()
