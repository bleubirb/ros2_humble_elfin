#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import importlib
import os
from typing import Optional, Tuple, List

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import torch
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

class Vision:
    def __init__(self, node: Node):
        self.node = node

        self.processed_pub = node.create_publisher(Image, "/vision/processed_image", 1)
        self.img_sub = node.create_subscription(Image, "/stereo/left/image_rect_color", self.process_image, 1)

    def process_image(self, img_msg: Image) -> None:
        pass

if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node("vision_node")
    
    # rclpy.spin(node)