#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import threading
import time
from concurrent.futures import Future, ThreadPoolExecutor

import rclpy
from cmd_move import CmdMove
from helpers import Action, JointAction
from move_solver import MoveSolver
from pns_driver import PNS_Driver

from vision_msgs.msg import Berries
from vision_msgs.srv import Ready

# OBSERVE_LOC = [0.156, 0.221, 0.326]  # in m
DROP_LOC = [-0.300, 0.00, 0.200]

# gripper is rotated 22 deg wrt horizontal when picking berries
BERRY_ROT = [-90, 68, 0]  # in degrees
BERRY_OBS_ROT = [-90, -18, 0]
DROP_ROT = [-179.5, 0, 0]

OBS_ROT_OFFSET_X = 0.08  # in m
CLOSEUP_OFFSET_Y = 0.15  # in m
APPROACH_OFFSET_Y = 0.05  # in m


def rotated_seq(
    position: list[float], ms: MoveSolver, cm: CmdMove, executor: ThreadPoolExecutor
) -> list[JointAction]:
    actions = []

    # close-up view (-y, -z)
    actions.append(
        JointAction(
            Action.MOVE,
            position=[
                position[0] + OBS_ROT_OFFSET_X,
                position[1] - CLOSEUP_OFFSET_Y,
                position[2],
            ],
            orientation=BERRY_OBS_ROT,
            ms=ms,
            cm=cm,
            executor=executor,
        )
    )
    actions.append(JointAction(Action.FIND))

    return actions


def pick_seq(
    position: list[float], ms: MoveSolver, cm: CmdMove, executor: ThreadPoolExecutor
) -> list[JointAction]:
    actions = []

    force = 0.5  # for berry
    # force = 3.0 # for ball

    # approach (-y)
    actions.append(
        JointAction(
            Action.MOVE,
            position=[position[0], position[1] - APPROACH_OFFSET_Y, position[2]],
            orientation=BERRY_ROT,
            ms=ms,
            cm=cm,
            executor=executor,
        )
    )
    actions.append(
        JointAction(
            Action.MOVE,
            position=position,
            orientation=BERRY_ROT,
            ms=ms,
            cm=cm,
            executor=executor,
        )
    )

    # grip
    # actions.append(JointAction(Action.GRIP, force=force))

    # pull down (-z) and back (-y)
    actions.append(
        JointAction(
            Action.MOVE,
            position=[position[0], position[1], position[2] - APPROACH_OFFSET_Y],
            orientation=BERRY_ROT,
            ms=ms,
            cm=cm,
            executor=executor,
        )
    )
    actions.append(
        JointAction(
            Action.MOVE,
            position=[
                position[0],
                position[1] - APPROACH_OFFSET_Y,
                position[2] - APPROACH_OFFSET_Y,
            ],
            orientation=BERRY_ROT,
            ms=ms,
            cm=cm,
            executor=executor,
        )
    )

    return actions


def drop_seq(
    position: list[float], ms: MoveSolver, cm: CmdMove, executor: ThreadPoolExecutor
) -> list[JointAction]:
    actions = []

    raised_pos = [position[0], position[1], position[2] + 0.15]
    force = 0.0  # release

    # approach from raised pos
    actions.append(
        JointAction(
            Action.MOVE,
            position=raised_pos,
            orientation=DROP_ROT,
            ms=ms,
            cm=cm,
            executor=executor,
        )
    )
    actions.append(
        JointAction(
            Action.MOVE,
            position=position,
            orientation=DROP_ROT,
            ms=ms,
            cm=cm,
            executor=executor,
        )
    )

    # drop
    actions.append(JointAction(Action.GRIP, force=force))

    # return to raised pos
    actions.append(
        JointAction(
            Action.MOVE,
            position=raised_pos,
            orientation=DROP_ROT,
            ms=ms,
            cm=cm,
            executor=executor,
        )
    )

    return actions


def handle_action(
    action: JointAction,
    ms: MoveSolver,
    cm: CmdMove,
    driver: PNS_Driver,
    ready_client: rclpy.client.Client,
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
        else:
            valid, joints = ms.move(
                action.position,
                action.orientation,
                cm.joint_state or [0, 0, 0, 0, 0, 0],
            )

        log_str += f",{int(valid)}\n"
        if not valid:
            node.get_logger().error("No valid joint solution found!")
            return log_str

        # execute the joint action
        cm.dance(joints)

        time.sleep(2)  # delay between motions
    elif action.action == Action.GRIP:  # GRIP action
        node.get_logger().info(f"Setting desired force to {action.force} for {i+1}")
        log_str = f"{time.time()},{Action.GRIP},{','.join(['0']*6)},{action.force},1\n"

        driver.set_fd(action.force)

        count = 0
        while not driver.get_done():
            if count % 10 == 0:
                node.get_logger().info(
                    f"Waiting for gripper to reach desired force for {i+1}"
                )
            count += 1
            time.sleep(0.5)
    else:  # FIND action
        node.poses = None

        ready_client.call_async(Ready.Request(ready_req=True))

        count = 0
        while getattr(node, "poses", None) is None:
            if count % 10 == 0:
                node.get_logger().info("Waiting for berry poses...")
            count += 1
            time.sleep(0.5)

    return log_str


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node("hybrid_node")
    ip = node.declare_parameter("ip", "192.168.1.1").get_parameter_value().string_value
    port = node.declare_parameter("port", "502").get_parameter_value().string_value

    ready_client = node.create_client(Ready, "/vision/set_ready")

    def save_poses(msg: Berries):
        node.poses = msg.berries
        node.get_logger().info(f"poses: {node.poses}")

    poses_sub = node.create_subscription(
        Berries,
        "/vision/detected_poses",
        save_poses,
        1,
    )

    ms = MoveSolver(node)
    cm = CmdMove(node, ms)
    driver = PNS_Driver(node, ip, port)
    executor = ThreadPoolExecutor(max_workers=5)

    # spin required for subscriptions to work
    future = Future()
    spin_thread = threading.Thread(
        target=rclpy.spin_until_future_complete, args=(node, future), daemon=True
    )
    spin_thread.start()

    time.sleep(1)

    if not os.path.exists("data"):
        os.makedirs("data")
    f_idx = 0
    while os.path.exists(f"data/hybrid_state_{f_idx}.csv"):
        f_idx += 1
    log_file = open(f"data/hybrid_state_{f_idx}.csv", "w")
    log_file.write("Time,State,X,Y,Z,RX,RY,RZ,F,Valid\n")

    estimated_data = []

    observe_locs = [
        [0.205, 0.2, 0.49],
        [0.156, 0.221, 0.326],  # hardcode more positions here
        # [-0.023, 0.127, 0.387],
        [0.219, 0.299, 0.459],
        [0.217, 0.328, 0.393],
        [0.310, 0.312, 0.411]
    ]

    for observe_loc in observe_locs:
        node.get_logger().info(f"Using observe location: {observe_loc}")

        observe_act = JointAction(
            Action.MOVE,
            position=observe_loc,
            orientation=BERRY_ROT,
            ms=ms,
            cm=cm,
            executor=executor,
        )

        actions: list[JointAction] = [
            JointAction(Action.GRIP, force=0),
            observe_act,
            JointAction(Action.FIND),
        ]

        for i, action in enumerate(actions):
            node.get_logger().info(f"Executing action {i+1}/{len(actions)}")

            log_file.write(handle_action(action, ms, cm, driver, ready_client, node))

            node.get_logger().info(f"Finished action {i+1}/{len(actions)}")

        # BEGIN BERRY SEQUENCE

        # +x: from room to wall
        # +y: away from arm, toward other arm
        # +z: up?

        OBSERVE_X_OFFSET = -0.050
        OBSERVE_Y_OFFSET = -0.095
        OBSERVE_Z_OFFSET = 0.08  # TODO: calibrate

        ROTATED_X_OFFSET = -0.040

        orig_berry_locs = node.poses.copy()

        move_to_loc = None
        revised_move_to_loc = None

        for j, berry in enumerate(orig_berry_locs):
            orig_berry_loc = [
                berry.pose.x,
                berry.pose.y,
                berry.pose.z,
            ]
            berry_loc = [
                orig_berry_loc[0] + OBSERVE_X_OFFSET,
                orig_berry_loc[1] + OBSERVE_Y_OFFSET,
                orig_berry_loc[2] + OBSERVE_Z_OFFSET,
            ]
            move_to_loc = [
                observe_loc[0] + berry_loc[0],
                observe_loc[1] + berry_loc[1],
                observe_loc[2] + OBSERVE_Z_OFFSET,
            ]
            node.get_logger().info(
                f"Berry {j+1}: {orig_berry_loc} → {berry_loc} → {move_to_loc}"
            )

            rotated_actions = rotated_seq(move_to_loc, ms, cm, executor)
            for k, action in enumerate(rotated_actions):
                node.get_logger().info(
                    f"Executing close-up action {k+1}/{len(rotated_actions)}"
                )

                log_file.write(
                    handle_action(action, ms, cm, driver, ready_client, node)
                )

                node.get_logger().info(
                    f"Finished close-up action {k+1}/{len(rotated_actions)}"
                )

            if not node.poses:
                node.get_logger().error("No berry poses found during close-up!")
                continue

            _pose = min(node.poses, key=lambda p: abs(p.pose.z))

            close_berry_loc = [
                _pose.pose.x,
                _pose.pose.y,
                _pose.pose.z,
            ]

            revised_move_to_loc = [
                move_to_loc[0],
                move_to_loc[1],
                move_to_loc[2] + close_berry_loc[0] + ROTATED_X_OFFSET,
            ]
            node.get_logger().info(
                f"Revised Berry {j+1} location: {close_berry_loc} → z: {close_berry_loc[0] + ROTATED_X_OFFSET} → {revised_move_to_loc}"
            )

        if move_to_loc is not None and revised_move_to_loc is not None:
            estimated_data.append((observe_loc, move_to_loc, revised_move_to_loc))

    est_file = open(f"data/hybrid_estimated_{f_idx}.csv", "w")
    est_file.write(
        "Observe_X,Observe_Y,Observe_Z,Est_X,Est_Y,Est_Z,Rev_X,Rev_Y,Rev_Z\n"
    )
    for data in estimated_data:
        observe_loc, est_loc, rev_loc = data
        est_file.write(
            f"{observe_loc[0]},{observe_loc[1]},{observe_loc[2]},"
            f"{est_loc[0]},{est_loc[1]},{est_loc[2]},"
            f"{rev_loc[0]},{rev_loc[1]},{rev_loc[2]}\n"
        )
    est_file.close()

    # END BERRY SEQUENCE

    executor.shutdown(wait=True)
    log_file.close()
    node.get_logger().info(f"Data saved to data/hybrid_state_{f_idx}.csv")
    driver.stop()
    node.get_logger().info("It worked!\n")

    # cleanup
    future.set_result(True)
    spin_thread.join()
    rclpy.shutdown()
