#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import json
import os
import time

import cv2
import rclpy
import requests
from cv_bridge import CvBridge
from detector import ExactDetector, run_detector_tiled
from geometry_msgs.msg import Point, Pose
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from sensor_msgs.msg import Image
from tracker import CentroidTracker

from vision_msgs.msg import Berries, BerryPose
from vision_msgs.srv import Ready

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
            Berries, "/vision/detected_poses", 1
        )

        self.img_sub: Subscription = node.create_subscription(
            Image, "/stereo/left/image_rect_color", self.process_frames, 1
        )

        self.pose_sub: Subscription = node.create_subscription(
            Pose, "/cmd_move/robot_pose", self.store_robot_pose, 1
        )
        self.robot_pose: Pose | None = None

        self.ready_srv = node.create_service(Ready, "/vision/set_ready", self.set_ready)
        self.ready = False

        self.br: CvBridge = CvBridge()

        self.detector = ExactDetector(
            CHECKPOINT,
            names=NAME,
            anchors=ANCHORS,
            ratios=RATIOS,
            min_size=MIN_SIZE,
            max_size=MAX_SIZE,
        )

        self.tracker = None

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

        self.frame_count: int = 0

    def store_robot_pose(self, pose_msg: Pose) -> None:
        self.robot_pose = pose_msg
        # self.node.get_logger().info(f"Updated robot pose: {pose_msg}")

    def set_ready(self, req, resp) -> None:
        self.ready = req.ready_req
        resp.ready_resp = self.ready
        self.node.get_logger().info(f"Set ready to {self.ready}")
        return resp

    def process_frames(self, img_msg: Image) -> None:
        if not self.ready:
            return

        ts: float = (
            float(img_msg.header.stamp.sec) + float(img_msg.header.stamp.nanosec) * 1e-9
        )
        # Convert ROS Image message to OpenCV image
        cv_image = self.br.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")

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

        # if nothing found, try again
        if not dets_xywh:
            dets_xywh, det_scores, _ = run_detector_tiled(
                self.detector,
                cv_image,
                SCORE_THRESH,
                TILE_ROWS,
                TILE_COLS,
                TILE_OVERLAP,
                nms_thresh=TILE_NMS,
            )

        if self.tracker is None:
            self.tracker = CentroidTracker(max_dist_px=TRACK_MAX_DIST)

        ids = self.tracker.update(dets_xywh) if dets_xywh else []

        self.frame_count += 1

        berries = []

        for i, xywh in enumerate(dets_xywh):
            self.node.get_logger().info(
                f"Detection {i}: xywh={xywh}, score={det_scores[i]:.4f}"
            )
            x, y, w, h = map(int, xywh)
            u, v = x + 0.5 * w, y + 0.5 * h
            tid = ids[i] if i < len(ids) else -1
            score = det_scores[i]
            coords = {"x": float("nan"), "y": float("nan"), "z": float("nan")}

            payload_args = {
                "pose_frame": "camera",
                "region_of_interest_2d": {
                    "offset_x": int(x),
                    "offset_y": int(y),
                    "width": int(w),
                    "height": int(h),
                },
                "cell_count": {"x": 1, "y": 1},
            }
            payload = {"args": payload_args}
            payload_json = json.dumps(payload)
            url = "http://192.168.1.104/api/v2/pipelines/0/nodes/rc_measure/services/measure_depth"

            for _ in range(3):
                resp = requests.put(
                    url,
                    headers={"Content-Type": "application/json"},
                    data=payload_json,
                    timeout=5.0,
                )

                try:
                    # self.node.get_logger().info(
                    #     f"Depth response status: {resp.status_code}"
                    # )
                    resp.raise_for_status()
                    # self.node.get_logger().info(f"Depth response body: {resp.text}")
                    depth_info = resp.json()
                    # self.node.get_logger().info(f"Depth response body: {depth_info}")
                except:
                    self.node.get_logger().error(
                        "Depth response not successful or not JSON"
                    )
                    depth_info = None

                # parse depth_info and set coords
                if depth_info is not None:
                    resp_body = depth_info.get("response", depth_info)

                    overall = resp_body.get("overall", {})
                    min_overall = overall.get("min_z", {})

                    # self.node.get_logger().info(f"min overall depth: {min_overall}")

                    mz_x = float(min_overall.get("x"))
                    mz_y = float(min_overall.get("y"))
                    mz_z = float(min_overall.get("z"))

                    coords["x"] = mz_x
                    coords["y"] = mz_z  # robot coords
                    coords["z"] = mz_y

                    self.node.get_logger().info(
                        f"Detection {i} (ID {tid}): 3D coords: X={coords['x']:.4f} Y={coords['y']:.4f} Z={coords['z']:.4f}"
                    )
                    if coords["x"] != 0.0 and coords["y"] != 0.0 and coords["z"] != 0.0:
                        break  # valid data received, exit retry loop
                else:
                    self.node.get_logger().error("No depth data received")

                time.sleep(0.5)  # wait before retrying

            if (
                coords["x"] != float("nan")
                and coords["x"] != 0.0
                and coords["y"] != 0.0
                and coords["z"] != 0.0
                and self.frame_count % 4 == 0
            ):
                berries.append(
                    BerryPose(
                        id=tid, pose=Point(x=coords["x"], y=coords["y"], z=coords["z"])
                    )
                )

            self.annotate_image(cv_image, x, y, w, h, tid, NAME, score, coords)
            self.csv_writer.writerow(
                [
                    self.frame,
                    f"{ts:.6f}",
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

        fps = 1.0 / max(1e-9, (ts - self.last_tick))
        self.last_tick = ts

        self.frame += 1
        self.node.get_logger().info(
            f"Frame {self.frame}: Detected {len(dets_xywh)} objects, FPS: {fps:.2f}"
        )

        # Stream processed image back to publisher
        processed_img_msg: Image = self.br.cv2_to_imgmsg(cv_image, encoding="bgr8")
        processed_img_msg.header.stamp.sec = int(ts)
        processed_img_msg.header.stamp.nanosec = int((ts - int(ts)) * 1e9)
        self.processed_pub.publish(processed_img_msg)

        # after 5 frames
        if self.frame_count % 4 == 0:
            # Publish detected poses
            berries_msg = Berries(berries=berries)
            self.poses_pub.publish(berries_msg)

            # don't run again unless commanded
            self.ready = False
            self.tracker = None
            self.frame_count = 0

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
