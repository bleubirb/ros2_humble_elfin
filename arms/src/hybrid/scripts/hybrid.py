#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import threading
import time
from asyncio import Future

import rclpy
from move_solver import (
    Action,
    JointAction,
    MoveSolver,
)
from cmd_move import CmdMove
from pns_driver import PNS_Driver

def pick_seq(position: list[float]) -> list[JointAction]:
    actions = []
    offset = 0.1  # meters

    # orientation = [-90, -112, 0]
    orientation = [-90, 68, 0]
    force = 0.8  # for berry
    # force = 3.0 # for ball

    # approach (-y)
    actions.append(
        JointAction(
            Action.MOVE,
            position=[position[0], position[1] - offset, position[2]],
            orientation=orientation,
        )
    )
    # actions.append(JointAction(Action.MOVE, position=position, orientation=orientation))

    # grip
    # actions.append(JointAction(Action.GRIP, force=force))

    # pull down (-z) and back (-y)
    # actions.append(
    #     JointAction(
    #         Action.MOVE,
    #         position=[position[0], position[1], position[2] - offset],
    #         orientation=orientation,
    #     )
    # )
    # actions.append(
    #     JointAction(
    #         Action.MOVE,
    #         position=[position[0], position[1] - offset, position[2] - offset],
    #         orientation=orientation,
    #     )
    # )

    return actions


def drop_seq(position: list[float]) -> list[JointAction]:
    actions = []

    raised_pos = [position[0], position[1], position[2] + 0.15]
    orientation = [-179.5, 0, 0]
    force = 0.0  # release

    # approach from raised pos
    actions.append(
        JointAction(Action.MOVE, position=raised_pos, orientation=orientation)
    )
    actions.append(JointAction(Action.MOVE, position=position, orientation=orientation))

    # drop
    # actions.append(JointAction(Action.GRIP, force=force))

    # return to raised pos
    actions.append(
        JointAction(Action.MOVE, position=raised_pos, orientation=orientation)
    )

    return actions


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

    berry_loc = [0.053, 0.720, 0.4546]  # in mm
    drop_loc = [-0.300, 0.00, 0.200]

    HOME = JointAction(Action.MOVE, position=None, orientation=None)

    actions: list[JointAction] = [
        JointAction(Action.GRIP, force=0),
        # HOME,
        *pick_seq(berry_loc),
        # *drop_seq(drop_loc),
        # HOME,
    ]

    joint_actions = dict()

    def compute_thread():
        last_idx = -1
        for i, action in enumerate(actions):
            if action.action == Action.MOVE:
                node.get_logger().info(f"Requesting joints for {i}")

                if last_idx == -1:
                    while cm.joint_state is None:
                        node.get_logger().info(
                            "Waiting for initial joint state to be received..."
                        )
                        time.sleep(0.5)
                    start_state = cm.joint_state
                else:
                    start_state = joint_actions[last_idx]

                joint_actions[i] = ms.move(
                    action.position,
                    action.orientation,
                    start_state,
                )
                node.get_logger().info(f"Received joints for {i}")
                last_idx = i

    thread = threading.Thread(target=compute_thread)
    thread.start()

    if not os.path.exists("data"):
        os.makedirs("data")
    f_idx = 0
    while os.path.exists(f"data/hybrid_state_{f_idx}.csv"):
        f_idx += 1
    log_file = open(f"data/hybrid_state_{f_idx}.csv", "w")
    log_file.write("Time,State,X,Y,Z,RX,RY,RZ,F\n")

    for i, action in enumerate(actions):
        node.get_logger().info(f"Executing action {i+1}/{len(actions)}")
        if action.action == Action.MOVE:  # MOVE action
            node.get_logger().info(
                f"Moving to {i+1}: {action.position} with orientation {action.orientation}"
            )

            # log the move action, either as all zeros or with the specified position and orientation
            if action.position is None:
                log_file.write(f"{time.time()},{Action.MOVE},{','.join(['0']*6)},0\n")
            else:
                log_file.write(
                    f"{time.time()},{Action.MOVE},{','.join([str(x) for x in action.position])},{','.join([str(x) for x in action.orientation])},0\n"
                )

            # wait for the joint action to be computed
            while i not in joint_actions:
                if count % 10 == 0:
                    node.get_logger().info(f"Waiting for joint action for {i+1}")
                count += 1
                time.sleep(0.1)

            # execute the joint action
            cm.dance(joint_actions[i])
        else:  # GRIP action
            node.get_logger().info(f"Setting desired force to {action.force} for {i+1}")
            log_file.write(
                f"{time.time()},{Action.GRIP},{','.join(['0']*6)},{action.force}\n"
            )

            driver.set_fd(action.force)

            count = 0
            while not driver.get_done():
                if count % 10 == 0:
                    node.get_logger().info(
                        f"Waiting for gripper to reach desired force for {i+1}"
                    )
                count += 1
                time.sleep(0.5)

        node.get_logger().info(f"Finished action {i+1}/{len(actions)}")

    log_file.close()
    node.get_logger().info(f"Data saved to data/hybrid_state_{f_idx}.csv")
    driver.stop()
    node.get_logger().info("It worked!")

    while True:
        time.sleep(1)

    # cleanup
    # future.set_result(True)
    # spin_thread.join()
    # rclpy.shutdown()
