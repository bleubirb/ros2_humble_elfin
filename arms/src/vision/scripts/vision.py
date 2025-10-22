#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import torch
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

class Vision:
    def __init__(self, node: Node):
        self.node: Node = node
        self.frame: int = 0

        self.processed_pub: Publisher = node.create_publisher(Image, "/vision/processed_image", 1)
        self.img_sub: Subscription = node.create_subscription(
            Image, "/stereo/left/image_rect_color", self.process_image, 1
        )

        self.node.get_logger().info("Vision node initialized and subscribed to /stereo/left/image_rect_color")

        self.br: CvBridge = CvBridge()

    def process_image(self, img_msg: Image) -> None:
        ts: float = float(img_msg.header.stamp.sec) + float(img_msg.header.stamp.nanosec) * 1e-9
        self.frame += 1

        # Convert ROS Image message to OpenCV image
        cv_image = self.br.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        height, width, _ = cv_image.shape

        self.node.get_logger().info(f"Processing frame {self.frame} at time {ts:.3f}s with resolution {width}x{height}")

        # Stream processed image back to publisher
        processed_img_msg: Image = self.br.cv2_to_imgmsg(cv_image, encoding='bgr8')
        processed_img_msg.header.stamp = img_msg.header.stamp
        self.processed_pub.publish(processed_img_msg)


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node("vision_node")

    Vision(node)

    rclpy.spin(node)

    # python infer_rcv_blueberries_v5.py
    # --ip 169.254.4.196
    # --checkpoint ./checkpoints/best_model.pth
    # --names blueberry
    # --score-thresh 0.80
    # --min-size 1200
    # --max-size 2000
    # --tile-grid 2x2
    # --tile-overlap 0.2
    # --tile-nms 0.5
    # --no-depth
    # --csv blueberries_tiled.csv
    # --display

    # rclpy.spin(node)
