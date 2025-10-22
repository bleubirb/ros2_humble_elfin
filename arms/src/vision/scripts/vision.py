#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import os
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from detector import ExactDetector, run_detector_tiled
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from sensor_msgs.msg import Image
from tracker import CentroidTracker

CHECKPOINT = os.path.join(os.path.abspath(os.path.dirname(__file__)), "bestestboy.pth")
NAME = "blueberry"
ANCHORS = "64,128,256,384,512"
RATIOS = "0.75,1.0,1.5"
MIN_SIZE = 1200
MAX_SIZE = 2000
SCORE_THRESH = 0.8

# tile settings
TILE_ROWS = 2
TILE_COLS = 2
TILE_OVERLAP = 0.2
TILE_NMS = 0.5

# stream content
NO_DEPTH = True
MAX_FPS = 0

# output
CSV_FN = "blueberries_tiled"
DATA_PATH = "data_cam"
TRACK_MAX_DIST = 60


class Vision:
    def __init__(self, node: Node):
        self.node: Node = node
        self.frame: int = 0

        self.processed_pub: Publisher = node.create_publisher(
            Image, "/vision/processed_image", 1
        )
        self.img_sub: Subscription = node.create_subscription(
            Image, "/stereo/left/image_rect_color", self.process_image, 1
        )

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
                "depth_method",
            ]
        )

        self.last_tick: float = 0.0

    def process_image(self, img_msg: Image) -> None:
        ts: float = (
            float(img_msg.header.stamp.sec) + float(img_msg.header.stamp.nanosec) * 1e-9
        )
        now = time.time()
        self.frame += 1

        # Convert ROS Image message to OpenCV image
        cv_image = self.br.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        height, width, _ = cv_image.shape

        self.node.get_logger().info(
            f"Processing frame {self.frame} at time {ts:.3f}s with resolution {width}x{height}"
        )

        # detection (tiled or single)
        dets_xywh, det_scores, det_labels = run_detector_tiled(
            self.detector,
            cv_image,
            SCORE_THRESH,
            TILE_ROWS,
            TILE_COLS,
            TILE_OVERLAP,
            nms_thresh=TILE_NMS,
        )
        ids = self.tracker.update(dets_xywh) if dets_xywh else []

        # TODO: depth estimation using stereo disparity (right now, this is difficult because disparity is a separate topic)
        # has_disp = (not NO_DEPTH) and (imageset.disparity.image.data != b"")
        # Zmap = None
        # if has_disp:
        #     disp = imageset.disparity
        #     disp_img = to_numpy_image(disp.image)  # mono16
        #     Zmap = disparity_to_depth_m(
        #         disp_img,
        #         disp.scale,
        #         disp.offset,
        #         left.focal_length,
        #         disp.baseline,
        #         disp.invalid_data_value,
        #     )

        for i, xywh in enumerate(dets_xywh):
            x, y, w, h = map(int, xywh)
            u, v = x + 0.5 * w, y + 0.5 * h
            tid = ids[i] if i < len(ids) else -1
            score = det_scores[i]

            # TODO: depth estimation
            X = Y = Zm = None
            depth_method = ""
            # if Zmap is not None:
            #     cx, cy = int(round(u)), int(round(v))
            #     wx = max(1, int(round(w * 0.2)))
            #     wy = max(1, int(round(h * 0.2)))
            #     x0 = max(0, cx - wx)
            #     x1 = min(width, cx + wx)
            #     y0 = max(0, cy - wy)
            #     y1 = min(height, cy + wy)
            #     patch = Zmap[y0:y1, x0:x1]
            #     if patch.size > 0:
            #         Zm = float(np.nanmedian(patch))
            #         if np.isfinite(Zm) and Zm > 0:
            #             fx = float(left.focal_length)
            #             cx0 = float(left.principal_point_u)
            #             cy0 = float(left.principal_point_v)
            #             X = (u - cx0) * Zm / fx
            #             Y = (v - cy0) * Zm / fx
            #             depth_method = "stereo"

            self.annotate_image(cv_image, x, y, w, h, tid, NAME, score, Zm)
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
                    "" if X is None else f"{X:.4f}",
                    "" if Y is None else f"{Y:.4f}",
                    "" if Zm is None else f"{Zm:.4f}",
                    depth_method,
                ]
            )

        fps = 1.0 / max(1e-6, (ts - self.last_tick))
        self.last_tick = ts

        if self.last_tick > 0.0:
            self.node.get_logger().info(
                f"Frame {self.frame}: Detected {len(dets_xywh)} objects, FPS: {fps:.2f}"
            )

        # Stream processed image back to publisher
        processed_img_msg: Image = self.br.cv2_to_imgmsg(cv_image, encoding="bgr8")
        processed_img_msg.header.stamp = img_msg.header.stamp
        self.processed_pub.publish(processed_img_msg)

        if MAX_FPS > 0:
            elapsed = time.time() - now
            sleep_time = max(0.0, (1.0 / MAX_FPS) - elapsed)
            time.sleep(sleep_time)

    # TODO: depth estimation
    # def disparity_to_depth_m(
    #     disp_u16: np.ndarray,
    #     scale: float,
    #     offset: float,
    #     focal_px: float,
    #     baseline_m: float,
    #     invalid_value: float,
    # ) -> np.ndarray:
    #     d = (disp_u16.astype(np.float32) / float(scale)) + float(offset)
    #     d[d <= max(1e-9, float(invalid_value))] = np.nan
    #     return (float(focal_px) * float(baseline_m)) / d

    def annotate_image(self, cv_image, x, y, w, h, tid, name, score, Zm=None) -> None:
        color = (0, 220, 0) if Zm is not None else (80, 80, 80)
        cv2.rectangle(cv_image, (x, y), (x + w, y + h), color, 2)
        label = f"id {tid}  {name} {score:.2f}" + (
            f"  Z={Zm:.2f}m" if Zm is not None else ""
        )
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
