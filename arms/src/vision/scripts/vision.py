#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import os
import threading
import time

import cv2
import numpy as np
import rclpy
import requests
from cv_bridge import CvBridge
from detector import ExactDetector, run_detector_tiled
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from sensor_msgs.msg import CameraInfo, Image

# from stereo_msgs.msg import Image
from tracker import CentroidTracker
import subprocess
import json

CHECKPOINT = os.path.join(os.path.abspath(os.path.dirname(__file__)), "bestestboy.pth")
NAME = "blueberry"
ANCHORS = "64,128,256,384,512"
RATIOS = "0.75,1.0,1.5"
MIN_SIZE = 1200
MAX_SIZE = 2000
SCORE_THRESH = 0.95

# tile settings
TILE_ROWS = 2
TILE_COLS = 2
TILE_OVERLAP = 0.2
TILE_NMS = 0.5

# stream content
NO_DEPTH = False
MAX_FPS = 0  # 0=unlimited

# output
CSV_FN = "blueberries_tiled"
DATA_PATH = "data_cam"
TRACK_MAX_DIST = 60

IMAGE_TS_DELTA_THRESH = 0.05  # seconds


class Vision:
    def __init__(self, node: Node):
        self.node: Node = node
        self.frame: int = 0

        self.processed_pub: Publisher = node.create_publisher(
            Image, "/vision/processed_image", 1
        )

        self.poses_pub: Publisher = node.create_publisher(
            Pose, "/vision/detected_poses", 1
        )

        self.img_sub: Subscription = node.create_subscription(
            Image, "/stereo/left/image_rect_color", self.store_color_img, 1
        )

        # self.depth_sub: Subscription = node.create_subscription(
        #     Image, "/stereo/depth", self.store_depth_img, 1
        # )

        # self.info_sub: Subscription = node.create_subscription(
        #     CameraInfo, "/stereo/left/camera_info", self.store_camera_info, 1
        # )

        self.pose_sub: Subscription = node.create_subscription(
            Pose, "/hybrid/robot_pose", self.store_robot_pose, 1
        )
        self.robot_pose: Pose | None = None

        self.br: CvBridge = CvBridge()

        self.detector = ExactDetector(
            CHECKPOINT,
            names=NAME,
            anchors=ANCHORS,
            ratios=RATIOS,
            min_size=MIN_SIZE,
            max_size=MAX_SIZE,
        )

        self.tracker = CentroidTracker(max_dist_px=TRACK_MAX_DIST)

        if not os.path.exists(DATA_PATH):
            os.makedirs(DATA_PATH)
        f_idx = 0
        while os.path.exists(f"{DATA_PATH}/{CSV_FN}_{f_idx}.csv"):
            f_idx += 1

        self.csv_file = open(f"{DATA_PATH}/{CSV_FN}_{f_idx}.csv", "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            [
                "frame",
                "timestamp_sec",
                "track_id",
                "label",
                "score",
                "u",
                "v",
                "bbox_x1",
                "bbox_y1",
                "bbox_x2",
                "bbox_y2",
                "X_m",
                "Y_m",
                "Z_m",
            ]
        )

        self.last_tick: float = 0.0

        self.color_dict: dict[float, np.ndarray] = {}

        self.process_frames_thread = threading.Thread(
            target=self.process_frames_threaded
        )
        self.process_frames_thread.start()

    def store_color_img(self, img_msg: Image) -> None:
        ts: float = (
            float(img_msg.header.stamp.sec) + float(img_msg.header.stamp.nanosec) * 1e-9
        )
        # Convert ROS Image message to OpenCV image
        cv_image = self.br.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")

        self.color_dict[ts] = cv_image.copy()

        # self.node.get_logger().info(f"Stored color image at timestamp {ts:.3f}s")

    def store_robot_pose(self, pose_msg: Pose) -> None:
        self.robot_pose = pose_msg
        # self.node.get_logger().info(f"Updated robot pose: {pose_msg}")

    def process_frames(self) -> None:
        if not self.color_dict:
            self.node.get_logger().info("Waiting for color image...")
            time.sleep(0.1)
            return

        processed = False # set processed to true when all berries in a frame have been localized

        while not processed and rclpy.ok():
            # find newest color timestamp
            color_ts = max(self.color_dict.keys())

            # pop images and parameters from dicts
            cv_image = self.color_dict.pop(color_ts)

            # detection (tiled)
            dets_xywh, det_scores, _ = run_detector_tiled(
                self.detector,
                cv_image,
                SCORE_THRESH,
                TILE_ROWS,
                TILE_COLS,
                TILE_OVERLAP,
                nms_thresh=TILE_NMS,
            )
            ids = self.tracker.update(dets_xywh) if dets_xywh else []

            for i, xywh in enumerate(dets_xywh):
                self.node.get_logger().info(f"Detection {i}: xywh={xywh}, score={det_scores[i]:.4f}")
                x, y, w, h = map(int, xywh)
                u, v = x + 0.5 * w, y + 0.5 * h
                tid = ids[i] if i < len(ids) else -1
                score = det_scores[i]
                coords = {"x": float("nan"), "y": float("nan"), "z": float("nan")}

                if self.robot_pose is not None:
                    try:
                        payload_args = {
                            "pose_frame": "external",
                            "region_of_interest_2d": {
                                "offset_x": int(x),
                                "offset_y": int(y),
                                "width": int(w),
                                "height": int(h),
                            },
                            "cell_count": {"x": 1, "y": 1},
                            "robot_pose": {
                                "position": {
                                    "x": float(self.robot_pose.position.x),
                                    "y": float(self.robot_pose.position.y),
                                    "z": float(self.robot_pose.position.z),
                                },
                                "orientation": {
                                    "x": float(self.robot_pose.orientation.x),
                                    "y": float(self.robot_pose.orientation.y),
                                    "z": float(self.robot_pose.orientation.z),
                                    "w": float(self.robot_pose.orientation.w),
                                },
                            },
                        }
                        payload = {"args": payload_args}
                        payload_json = json.dumps(payload)
                        url = "http://192.168.1.104/api/v2/pipelines/0/nodes/rc_measure/services/measure_depth"

                        # self.node.get_logger().info(f"Depth request payload: {payload}")

                        cmd = [
                            "curl",
                            "-s",
                            "-X",
                            "PUT",
                            url,
                            "-H",
                            "Content-Type: application/json",
                            "-d",
                            payload_json,
                            "-w",
                            "\n%{http_code}",
                        ]
                        proc = subprocess.run(
                            cmd, capture_output=True, text=True, timeout=5.0
                        )

                        if proc.stderr:
                            self.node.get_logger().debug(
                                f"curl stderr: {proc.stderr.strip()}"
                            )

                        stdout = proc.stdout or ""
                        body, sep, status = stdout.rpartition("\n")
                        if not sep:  
                            body = stdout
                            status_code = None
                        else:
                            status_code = int(status) if status.isdigit() else None

                        class _Resp:
                            def __init__(self, status_code, text):
                                self.status_code = status_code
                                self.text = text

                            def json(self):
                                return json.loads(self.text)

                        resp = _Resp(status_code, body)
                    except subprocess.TimeoutExpired as exc:
                        self.node.get_logger().error(f"Depth request timed out: {exc}")
                        resp = None
                    except Exception as exc:
                        self.node.get_logger().error(f"Depth request failed: {exc}")
                        resp = None
                    if resp is not None:
                        try:
                            self.node.get_logger().info(f"Depth response status: {resp.status_code}")
                            # self.node.get_logger().info(f"Depth response body: {resp.text}")
                            depth_info = resp.json()
                        except ValueError:
                            self.node.get_logger().error("Depth response not JSON")
                            depth_info = None

                        # parse depth_info and set coords
                        if depth_info is not None:
                            resp_body = depth_info.get("response", depth_info)

                            overall = resp_body.get("overall", {}) or {}
                            mean_overall = overall.get("mean_z") or {}

                            # self.node.get_logger().info(f"Mean overall depth: {mean_overall}")

                            mz_x = float(mean_overall.get("x"))
                            mz_y = float(mean_overall.get("y"))
                            mz_z = float(mean_overall.get("z"))

                            coords["x"] = mz_x
                            coords["y"] = mz_z # robot coords
                            coords["z"] = mz_y

                            self.node.get_logger().info(
                                f"Detection {i} (ID {tid}): 3D coords: X={coords['x']:.4f} Y={coords['y']:.4f} Z={coords['z']:.4f}"
                            )
                        else:
                            self.node.get_logger().error("No depth data received")
            processed = True

            self.annotate_image(cv_image, x, y, w, h, tid, NAME, score, coords)
            self.csv_writer.writerow(
                [
                    self.frame,
                    f"{color_ts:.6f}",
                    tid,
                    NAME,
                    f"{score:.4f}",
                    f"{u:.2f}",
                    f"{v:.2f}",
                    x,
                    y,
                    x + w,
                    y + h,
                    f"{coords['x']:.4f}",
                    f"{coords['y']:.4f}",
                    f"{coords['z']:.4f}",
                ]
            )

        fps = 1.0 / max(1e-9, (color_ts - self.last_tick))
        self.last_tick = color_ts

        self.node.get_logger().info(
            f"Frame {self.frame}: Detected {len(dets_xywh)} objects, FPS: {fps:.2f}"
        )
        self.frame += 1

        # Stream processed image back to publisher
        processed_img_msg: Image = self.br.cv2_to_imgmsg(cv_image, encoding="bgr8")
        processed_img_msg.header.stamp.sec = int(color_ts)
        processed_img_msg.header.stamp.nanosec = int((color_ts - int(color_ts)) * 1e9)
        self.processed_pub.publish(processed_img_msg)

    def process_frames_threaded(self) -> None:
        while rclpy.ok():
            start_time = time.time()
            self.process_frames()
            elapsed = time.time() - start_time
            if MAX_FPS > 0:
                sleep_time = max(0.0, (1.0 / MAX_FPS) - elapsed)
                time.sleep(sleep_time)

    def annotate_image(self, cv_image, x, y, w, h, tid, name, score, coords) -> None:
        color = (0, 220, 0)
        cv2.rectangle(cv_image, (x, y), (x + w, y + h), color, 2)
        label = f"id {tid}  {name} {score:.2f} [{coords['x']:.4f}, {coords['y']:.4f}, {coords['z']:.4f}]"
        cv2.putText(
            cv_image,
            label,
            (x, max(0, y - 7)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
        )

    def cleanup(self) -> None:
        self.csv_file.close()


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node("vision_node")

    v = Vision(node)

    try:
        rclpy.spin(node)
    except Exception as e:
        v.cleanup()
        if isinstance(e, KeyboardInterrupt):
            print("Exiting...")
        else:
            raise e
