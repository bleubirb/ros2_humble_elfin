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


def draw_box(img: np.ndarray, box, color=(0, 255, 0), label: Optional[str] = None, score: Optional[float] = None):
    x1, y1, x2, y2 = [int(v) for v in box]
    cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
    if label is not None or score is not None:
        text = label if label is not None else ""
        if score is not None:
            text = f"{text} {score:.2f}".strip()
        cv2.putText(img, text, (x1, max(0, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2, cv2.LINE_AA)


class ModelVizNode(Node):
    def __init__(self):
        super().__init__("pth_model_viz")

        # Parameters
        self.declare_parameter("model_path", "")
        self.declare_parameter("model_is_torchscript", True)
        self.declare_parameter("model_class_module", "")   # e.g. "my_pkg.models.detector"
        self.declare_parameter("model_class_name", "")     # e.g. "MyDetector"
        self.declare_parameter("weights_key", "")          # e.g. "state_dict" if checkpoint wraps it
        self.declare_parameter("device", "cuda" if torch.cuda.is_available() else "cpu")
        self.declare_parameter("output_type", "detection")  # detection | segmentation | classification
        self.declare_parameter("input_topic", "/camera/color/image_raw")
        self.declare_parameter("output_image_topic", "/model/overlay")
        self.declare_parameter("marker_topic", "/model/markers")
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("resize_width", 0)   # 0 keeps original size
        self.declare_parameter("resize_height", 0)
        self.declare_parameter("class_names_path", "")  # optional newline-separated labels

        self.bridge = CvBridge()
        self.model = None
        self.device = self.get_parameter("device").get_parameter_value().string_value
        self.output_type = self.get_parameter("output_type").get_parameter_value().string_value
        self.conf_threshold = self.get_parameter("conf_threshold").get_parameter_value().double_value
        self.class_names = self._load_class_names(self.get_parameter("class_names_path").value)

        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        is_ts = self.get_parameter("model_is_torchscript").get_parameter_value().bool_value
        module_path = self.get_parameter("model_class_module").get_parameter_value().string_value
        class_name = self.get_parameter("model_class_name").get_parameter_value().string_value
        weights_key = self.get_parameter("weights_key").get_parameter_value().string_value

        if not os.path.isfile(model_path):
            self.get_logger().error(f"model_path does not exist: {model_path}")
        else:
            try:
                if is_ts:
                    self.model = torch.jit.load(model_path, map_location=self.device)
                    self.get_logger().info("Loaded TorchScript model")
                else:
                    if not module_path or not class_name:
                        raise RuntimeError("Non-TorchScript requires model_class_module and model_class_name")
                    mod = importlib.import_module(module_path)
                    cls = getattr(mod, class_name)
                    self.model = cls()  # if your ctor needs args, add parameters and pass them
                    ckpt = torch.load(model_path, map_location=self.device)
                    state_dict = ckpt[weights_key] if weights_key and weights_key in ckpt else ckpt
                    self.model.load_state_dict(state_dict)
                    self.get_logger().info("Loaded state_dict model")
                self.model.to(self.device)
                self.model.eval()
            except Exception as e:
                self.get_logger().error(f"Failed to load model: {e}")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        out_img_topic = self.get_parameter("output_image_topic").get_parameter_value().string_value
        marker_topic = self.get_parameter("marker_topic").get_parameter_value().string_value

        self.sub = self.create_subscription(Image, input_topic, self.on_image, qos)
        self.pub_img = self.create_publisher(Image, out_img_topic, 10)
        self.pub_markers = self.create_publisher(MarkerArray, marker_topic, 10)

        self.resize_w = int(self.get_parameter("resize_width").value)
        self.resize_h = int(self.get_parameter("resize_height").value)

        self.get_logger().info(f"Subscribed to {input_topic}; publishing overlay to {out_img_topic} and markers to {marker_topic}")

    def _load_class_names(self, path: str) -> Optional[List[str]]:
        if not path:
            return None
        if not os.path.isfile(path):
            self.get_logger().warn(f"class_names_path not found: {path}")
            return None
        with open(path, "r") as f:
            return [line.strip() for line in f if line.strip()]

    def preprocess(self, img_bgr: np.ndarray) -> Tuple[torch.Tensor, Tuple[int, int]]:
        h, w = img_bgr.shape[:2]
        if self.resize_w > 0 and self.resize_h > 0:
            img_bgr = cv2.resize(img_bgr, (self.resize_w, self.resize_h), interpolation=cv2.INTER_LINEAR)
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        img = img_rgb.astype(np.float32) / 255.0
        tensor = torch.from_numpy(img).permute(2, 0, 1).unsqueeze(0).to(self.device)
        return tensor, (w, h)

    def on_image(self, msg: Image):
        if self.model is None:
            return
        try:
            cv_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")
            return

        inp, (orig_w, orig_h) = self.preprocess(cv_bgr)

        with torch.no_grad():
            try:
                output = self.model(inp)
            except Exception as e:
                self.get_logger().warn(f"inference failed: {e}")
                return

        overlay = cv_bgr.copy()
        markers = MarkerArray()

        if self.output_type == "detection":
            overlay, markers = self.handle_detection(overlay, output, orig_w, orig_h)
        elif self.output_type == "segmentation":
            overlay = self.handle_segmentation(overlay, output, orig_w, orig_h)
        elif self.output_type == "classification":
            overlay = self.handle_classification(overlay, output)
        else:
            self.get_logger().warn(f"Unknown output_type: {self.output_type}")

        # Publish overlay
        out_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        out_msg.header = msg.header
        self.pub_img.publish(out_msg)
        # Publish markers
        if markers.markers:
            self.pub_markers.publish(markers)

    def handle_detection(self, overlay: np.ndarray, output, orig_w: int, orig_h: int) -> Tuple[np.ndarray, MarkerArray]:
        # Try torchvision style: list[dict] with boxes, scores, labels
        boxes = None
        scores = None
        labels = None
        if isinstance(output, (list, tuple)) and len(output) > 0 and isinstance(output[0], dict):
            od = output[0]
            boxes = od.get("boxes", None)
            scores = od.get("scores", None)
            labels = od.get("labels", None)
            if torch.is_tensor(boxes):
                boxes = boxes.detach().cpu().numpy()
            if torch.is_tensor(scores):
                scores = scores.detach().cpu().numpy()
            if torch.is_tensor(labels):
                labels = labels.detach().cpu().numpy()
        elif torch.is_tensor(output):
            arr = output.detach().cpu().numpy()
            if arr.ndim == 2 and arr.shape[1] >= 6:
                # Nx6: x1,y1,x2,y2,score,class
                boxes = arr[:, 0:4]
                scores = arr[:, 4]
                labels = arr[:, 5].astype(int)
        elif isinstance(output, np.ndarray) and output.ndim == 2 and output.shape[1] >= 6:
            boxes = output[:, 0:4]
            scores = output[:, 4]
            labels = output[:, 5].astype(int)

        marray = MarkerArray()
        if boxes is None:
            self.get_logger().warn("Detection output not understood; expecting torchvision dict or Nx6 array")
            return overlay, marray

        # Draw and create markers
        for i in range(len(boxes)):
            score = float(scores[i]) if scores is not None else 1.0
            if score < self.conf_threshold:
                continue
            cls_id = int(labels[i]) if labels is not None else -1
            label = None
            if self.class_names and 0 <= cls_id < len(self.class_names):
                label = self.class_names[cls_id]
            elif cls_id >= 0:
                label = f"id{cls_id}"
            draw_box(overlay, boxes[i], (0, 255, 0), label, score)

            # Marker (2D box as LINE_LIST in image pixel frame; RViz can show in ImageOverlay plugin or just skip)
            # Here we publish as text markers at the top-left corner, which are visible in RViz "Markers".
            marker = Marker()
            marker.header.frame_id = "camera_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "detections"
            marker.id = i
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.scale.z = 0.05  # text height in meters
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            # Place text at a fixed depth, project roughly to camera; for full correctness, use camera intrinsics
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.5
            marker.text = label if label else f"{cls_id}:{score:.2f}"
            marray.markers.append(marker)

        return overlay, marray

    def handle_segmentation(self, overlay: np.ndarray, output, orig_w: int, orig_h: int) -> np.ndarray:
        # Expect HxW logits (class 1) or CxHxW with argmax over C
        if torch.is_tensor(output):
            out = output.detach().cpu().squeeze().numpy()
        else:
            out = np.squeeze(output)
        if out.ndim == 2:
            mask = out
        elif out.ndim == 3:
            mask = np.argmax(out, axis=0)
        else:
            self.get_logger().warn("Segmentation output not understood")
            return overlay

        mask = cv2.resize(mask.astype(np.float32), (overlay.shape[1], overlay.shape[0]), interpolation=cv2.INTER_NEAREST)
        heat = cv2.applyColorMap(np.uint8(255 * (mask - mask.min()) / (mask.ptp() + 1e-6)), cv2.COLORMAP_JET)
        blended = cv2.addWeighted(overlay, 0.6, heat, 0.4, 0)
        return blended

    def handle_classification(self, overlay: np.ndarray, output) -> np.ndarray:
        # Expect logits vector
        if torch.is_tensor(output):
            logits = output.detach().cpu().squeeze().numpy()
        else:
            logits = np.squeeze(output)
        cls_id = int(np.argmax(logits))
        score = float(torch.softmax(torch.from_numpy(logits), dim=0)[cls_id].item())
        label = str(cls_id)
        if self.class_names and 0 <= cls_id < len(self.class_names):
            label = self.class_names[cls_id]
        text = f"{label}: {score:.2f}"
        cv2.rectangle(overlay, (0, 0), (int(8 * len(text)), 30), (0, 0, 0), -1)
        cv2.putText(overlay, text, (5, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
        return overlay


def main():
    rclpy.init()
    node = ModelVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()