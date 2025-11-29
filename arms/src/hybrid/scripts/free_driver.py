#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import time
from concurrent.futures import Future, ThreadPoolExecutor

import rclpy
from cmd_move import CmdMove
from move_solver import MoveSolver
from pns_driver import PNS_Driver
from std_srvs.srv import SetBool

if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node("hybrid_node")
    ip = node.declare_parameter("ip", "192.168.1.1").get_parameter_value().string_value
    port = node.declare_parameter("port", "502").get_parameter_value().string_value

    ms = MoveSolver(node)
    cm = CmdMove(node, ms)
    driver = PNS_Driver(node, ip, port)
    executor = ThreadPoolExecutor(max_workers=5)

    # spin required for subscriptions to work
    future: Future = Future()
    spin_thread = threading.Thread(
        target=rclpy.spin_until_future_complete, args=(node, future), daemon=True
    )
    spin_thread.start()

    time.sleep(1)

    enable_req = SetBool.Request()
    enable_req.data = True

    clients = []
    for joint_id in range(1, 6):
        clients.append(
            node.create_client(SetBool, f"/elfin_module_close_brake_slave{joint_id}")
        )

    for client in clients:
        while not client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info(
                f"Service {client.srv_name} not available, waiting..."
            )
        client.call_async(enable_req)

    node.get_logger().info("Press Enter to continue...")

    # input()

    # clients = []
    # for joint_id in range(1, 6):
    #     clients.append(node.create_client(SetBool, f"/elfin_module_close_brake_slave{joint_id}"))

    # for client in clients:
    #     while not client.wait_for_service(timeout_sec=1.0):
    #         node.get_logger().info(f"Service {client.srv_name} not available, waiting...")
    #     client.call_async(enable_req)

    executor.shutdown(wait=True)
    driver.stop()
    node.get_logger().info("It worked!\n")

    # cleanup
    future.set_result(True)
    spin_thread.join()
    rclpy.shutdown()
