#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import threading
import time
from concurrent.futures import Future, ThreadPoolExecutor

import rclpy
from cmd_move import CmdMove
from helpers import Action, JointAction
from letters import LETTER_POSITIONS
from move_solver import MoveSolver

KEYBOARD_ROT = [-179.5, 0.0, 90.0]
APPROACH_OFFSET_Z = 0.02  # in m

json_path = "/home/adminhsl/Desktop/pns_ros2/arms/src/hybrid/scripts/hybrid_config.json"
json_data = json.load(open(json_path, "r"))

raised_actions = json_data.get("raised", {})
lowered_actions = json_data.get("lowered", {})


def letter_seq(
    char: str,
    ms: MoveSolver,
    cm: CmdMove,
    executor: ThreadPoolExecutor,
    node: rclpy.node.Node,
) -> list[JointAction]:
    actions = []
    x, y, z = LETTER_POSITIONS[char]

    x /= 1000  # convert mm to m
    y /= 1000  # convert mm to m
    z /= 1000  # convert mm to m

    node.get_logger().info(f"Typing letter '{char}' at position ({x}, {y}, {z})")

    # approach (+z)
    if char in raised_actions:
        raised_joints = raised_actions[char]
        raised = JointAction(
            Action.MOVE,
            position=[x, y, z + APPROACH_OFFSET_Z],
            orientation=KEYBOARD_ROT,
            joints=raised_joints,
            letter=char,
            raised=True,
        )
    else:
        raised = JointAction(
            Action.MOVE,
            position=[x, y, z + APPROACH_OFFSET_Z],
            orientation=KEYBOARD_ROT,
            ms=ms,
            cm=cm,
            executor=executor,
            letter=char,
            raised=True,
        )

    actions.append(raised)

    if char in lowered_actions:
        lowered_joints = lowered_actions[char]
        lowered = JointAction(
            Action.MOVE,
            position=[x, y, z],
            orientation=KEYBOARD_ROT,
            joints=lowered_joints,
            letter=char,
            raised=False,
        )
    else:
        lowered = JointAction(
            Action.MOVE,
            position=[x, y, z],
            orientation=KEYBOARD_ROT,
            ms=ms,
            cm=cm,
            executor=executor,
            letter=char,
            raised=False,
        )

    actions.append(lowered)

    # retreat (+z)
    actions.append(raised)  # reuse the approach action to retreat

    return actions


def handle_action(
    action: JointAction,
    ms: MoveSolver,
    cm: CmdMove,
    # driver: PNS_Driver,
    node: rclpy.node.Node,
) -> str:
    log_str = ""
    if action.action == Action.MOVE:  # MOVE action
        node.get_logger().info(
            f"Moving to {action.position} with orientation {action.orientation}"
        )

        # log the move action, either as all zeros or with the specified position and orientation
        if action.position is None:
            log_str = f"{time.time()},{Action.MOVE},{','.join(['0']*6)},0"
        else:
            log_str = f"{time.time()},{Action.MOVE},{','.join([str(x) for x in action.position])},{','.join([str(x) for x in action.orientation])},0"

        if action.future is not None:
            valid, joints = action.future.result()
        elif action.joints is not None:
            valid = True
            joints = action.joints
        else:
            valid, joints = ms.move(
                action.position,
                action.orientation,
                cm.joint_state or [0, 0, 0, 0, 0, 0],
            )
        
        if action.raised is not None and action.letter is not None:
            if action.raised:
                raised_actions[action.letter] = joints
            else:
                lowered_actions[action.letter] = joints
            # save to json
            with open(json_path, "w") as f:
                json.dump(
                    {"raised": raised_actions, "lowered": lowered_actions},
                    f,
                    indent=4,
                )

        log_str += f",{int(valid)}\n"
        if not valid:
            node.get_logger().error("No valid joint solution found!")
            return log_str

        # execute the joint action
        cm.dance(joints)

        # time.sleep(2)  # delay between motions
    # elif action.action == Action.GRIP:  # GRIP action
    #     node.get_logger().info(f"Setting desired force to {action.force} for {i+1}")
    #     log_str = f"{time.time()},{Action.GRIP},{','.join(['0']*6)},{action.force},1\n"

    #     driver.set_fd(action.force)

    #     count = 0
    #     while not driver.get_done():
    #         if count % 10 == 0:
    #             node.get_logger().info(
    #                 f"Waiting for gripper to reach desired force for {i+1}"
    #             )
    #         count += 1
    #         time.sleep(0.5)

    return log_str


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node("hybrid_node")
    ip = node.declare_parameter("ip", "192.168.1.1").get_parameter_value().string_value
    port = node.declare_parameter("port", "502").get_parameter_value().string_value

    ms = MoveSolver(node)
    cm = CmdMove(node, ms)
    # driver = PNS_Driver(node, ip, port)
    executor = ThreadPoolExecutor(max_workers=5)

    # spin required for subscriptions to work
    future: Future = Future()
    spin_thread = threading.Thread(
        target=rclpy.spin_until_future_complete, args=(node, future), daemon=True
    )
    spin_thread.start()

    time.sleep(1)

    if not os.path.exists("data"):
        os.makedirs("data")
    f_idx = 0
    while os.path.exists(f"data/keyboard_state_{f_idx}.csv"):
        f_idx += 1
    log_file = open(f"data/keyboard_state_{f_idx}.csv", "w", encoding="utf-8")
    log_file.write("Time,State,X,Y,Z,RX,RY,RZ,F,Valid\n")

    # # close gripper on a stylus
    # node.get_logger().info("Gripping...")

    # log_file.write(
    #     handle_action(JointAction(Action.GRIP, force=0.5), ms, cm, driver, node)
    # )

    # node.get_logger().info("Reached grip!")

    # state to track caps lock
    caps_state = False

    while True:
        # get text input
        text = input("Enter text to type: ")
        actions = []

        for char in text:
            # check if character is valid
            if char.lower() not in LETTER_POSITIONS:
                node.get_logger().error(f"Character '{char}' not found on keyboard!")
                continue

            # if letter is uppercase, caps lock needed
            need_caps = char != char.lower()

            # if desired caps state is different from current, toggle caps lock
            if need_caps != caps_state:
                actions.extend(letter_seq("caps", ms, cm, executor, node))
                caps_state = need_caps

            # type the letter
            actions.extend(letter_seq(char.lower(), ms, cm, executor, node))

        if caps_state:  # reset caps lock if needed
            actions.extend(letter_seq("caps", ms, cm, executor, node))
            caps_state = False

        for i, action in enumerate(actions):
            node.get_logger().info(f"Executing typing action {i+1}/{len(actions)}")

            log_file.write(handle_action(action, ms, cm, node))

            node.get_logger().info(f"Finished typing action {i+1}/{len(actions)}")

    executor.shutdown(wait=True)
    log_file.close()
    node.get_logger().info(f"Data saved to data/keyboard_state_{f_idx}.csv")
    # driver.stop()
    node.get_logger().info("It worked!\n")

    # cleanup
    future.set_result(True)
    spin_thread.join()
    rclpy.shutdown()
