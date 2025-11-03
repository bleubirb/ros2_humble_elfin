#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import os
import time
import threading
import bisect

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from stereo_msgs.msg import DisparityImage

from detector import ExactDetector, run_detector_tiled
from tracker import CentroidTracker

# ---------------- Config (kept like your original) ----------------
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

# pairing
IMAGE_TS_DELTA_THRESH = 0.20  # allow up to 200ms drift/delay

# detector working resolution (kept as your original)
DET_W = 640
DET_H = 480


class Vision:
    def __init__(self, node: Node):
        self.node: Node = node
        self.frame: int = 0

        self.processed_pub: Publisher = node.create_publisher(
            Image, "/vision/processed_image", 1
        )

        # --- Subscriptions (QoS tuned for sensors) ---
        sensor_qos = qos_profile_sensor_data

        self.img_sub: Subscription = node.create_subscription(
            Image, "/stereo/left/image_rect_color", self.store_color_img, sensor_qos
        )
        self.disparity_sub: Subscription = node.create_subscription(
            DisparityImage, "/stereo/disparity", self.store_disparity_img, sensor_qos
        )
        self.info_sub: Subscription = node.create_subscription(
            CameraInfo, "/stereo/left/camera_info", self.store_camera_info, sensor_qos
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

        # --- Robustness & smoothing ---
        self.Z_PATCH_FRAC = 0.20      # fraction of bbox for Z patch
        self.MAD_K = 2.5              # median ± K*MAD inlier band
        self.MIN_INLIERS = 25         # minimal inlier pixels after filtering
        self.MAD_MAX = 0.004          # m; patch must be this tight (or better)

        self.EMA_ALPHA = 0.80         # per-track EMA (higher=steadier)
        self.MAX_DZ_PER_FRAME = 0.08  # m; clamp ΔZ per frame (None to disable)

        # anti-drift gates
        self.Z_ABS_GATE = 0.01        # m: ignore smaller absolute changes
        self.Z_REL_GATE = 0.03        # fraction: ignore smaller relative changes
        self.Z_CONFIRM = 2            # need N consecutive frames to accept a bigger change

        # stillness gating in image space (px)
        self.UV_STILL_PX = 0.5        # if center moves less than this, hold Z

        # per-track state
        self._z_ema = {}              # tid -> (z_smooth, last_frame_idx)
        self._z_last = {}             # tid -> last valid smoothed Z (sticky reuse)
        self._z_pending = {}          # tid -> {"z": float, "count": int} for confirm
        self._uv_last = {}            # tid -> (u,v) last center for stillness

        # --- CSV output (kept) ---
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

        # --- Buffers (plus sorted index for pairing) ---
        self.color_dict: dict[float, np.ndarray] = {}
        self.color_times = []  # sorted timestamps for nearest matching

        # disparity tuple: (ts, disp_img_32f, f_px, T_m, min_d, roi)
        self.disparity_data: tuple[float, np.ndarray, float, float, float, object] | None = None

        # intrinsics (original rectified frame)
        self.fx = None
        self.cx = None
        self.cy = None
        self.orig_w = None
        self.orig_h = None

        # retain your original names for readability
        self.principal_point_u = None
        self.principal_point_v = None

        # first-arrival flags
        self._seen_color = False
        self._seen_disp = False
        self._seen_info = False

        # processing thread (kept)
        self.process_frames_thread = threading.Thread(target=self.process_frames_threaded, daemon=True)
        self.process_frames_thread.start()

    # ------------------------ Callbacks ------------------------

    def store_camera_info(self, info_msg: CameraInfo) -> None:
        # keep your original fields/logs
        self.principal_point_u = info_msg.k[2]
        self.principal_point_v = info_msg.k[5]
        self.fx = float(info_msg.k[0])
        self.cx = float(info_msg.k[2])
        self.cy = float(info_msg.k[5])
        self.node.get_logger().info(
            f"Camera principal point: u={self.principal_point_u}, v={self.principal_point_v}"
        )
        if not self._seen_info:
            self.node.get_logger().info(
                f"Camera intrinsics (orig): fx={self.fx:.3f}, cx={self.cx:.3f}, cy={self.cy:.3f}"
            )
            self._seen_info = True

        # close like your original
        self.node.destroy_subscription(self.info_sub)
        self.node.get_logger().info("Camera info received and subscription closed.")

    def store_color_img(self, img_msg: Image) -> None:
        ts: float = float(img_msg.header.stamp.sec) + float(img_msg.header.stamp.nanosec) * 1e-9
        cv_image = self.br.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")

        # store + index
        self.color_dict[ts] = cv_image
        bisect.insort(self.color_times, ts)

        if not self._seen_color:
            self._seen_color = True

        # set original size
        if self.orig_w is None or self.orig_h is None:
            self.orig_h, self.orig_w = cv_image.shape[:2]

        # trim stale vs latest disparity
        if self.disparity_data is not None:
            disp_ts = self.disparity_data[0]
            cutoff = disp_ts - IMAGE_TS_DELTA_THRESH
            while self.color_times and self.color_times[0] < cutoff:
                old_ts = self.color_times.pop(0)
                self.color_dict.pop(old_ts, None)

    def store_disparity_img(self, disp_msg: DisparityImage) -> None:
        ts: float = float(disp_msg.header.stamp.sec) + float(disp_msg.header.stamp.nanosec) * 1e-9

        # If our earliest color is newer than this disparity by too much, skip it
        if self.color_times and self.color_times[0] > ts + IMAGE_TS_DELTA_THRESH:
            return

        disp_image = self.br.imgmsg_to_cv2(disp_msg.image, desired_encoding="32FC1")
        f_px = float(disp_msg.f)
        if hasattr(disp_msg, "t"):
            T_m = float(disp_msg.t)
        elif hasattr(disp_msg, "T"):
            T_m = float(disp_msg.T)
        else:
            self.node.get_logger().error("DisparityImage has no baseline attribute 't' or 'T'; skipping frame.")
            return
        min_d = float(disp_msg.min_disparity)
        roi = disp_msg.valid_window

        # quick validity check
        try:
            has_valid = np.nanmax(disp_image) > max(0.0, min_d)
        except Exception:
            has_valid = False
        if not self._seen_disp:
            self._seen_disp = True
        if not has_valid:
            self.node.get_logger().warning(
                "Disparity image appears empty (no values > min_disparity). "
                "If this persists, ensure the 'Disparity' component is enabled on rc_visard."
            )

        self.disparity_data = (ts, disp_image.copy(), f_px, T_m, min_d, roi)

        # trim stale colors
        cutoff = ts - IMAGE_TS_DELTA_THRESH
        while self.color_times and self.color_times[0] < cutoff:
            old_ts = self.color_times.pop(0)
            self.color_dict.pop(old_ts, None)

    # ------------------------ Core Processing ------------------------

    def _nearest_color(self, t: float):
        if not self.color_times:
            return None, None
        i = bisect.bisect_left(self.color_times, t)
        candidates = []
        if i < len(self.color_times):
            candidates.append(self.color_times[i])
        if i > 0:
            candidates.append(self.color_times[i - 1])
        ts_best = min(candidates, key=lambda x: abs(x - t))
        return ts_best, self.color_dict.get(ts_best)

    def disparity_to_depth_m(
        self,
        disp: np.ndarray,
        f_px: float,
        T_m: float,
        min_d: float,
        roi,
    ) -> np.ndarray:
        # mask ROI and invalids, then Z = fT/d
        d = disp.astype(np.float32).copy()

        # mask ROI (on disparity native size)
        if roi and (roi.width > 0 and roi.height > 0):
            mask = np.zeros_like(d, dtype=bool)
            y0, y1 = roi.y_offset, roi.y_offset + roi.height
            x0, x1 = roi.x_offset, roi.x_offset + roi.width
            mask[y0:y1, x0:x1] = True
            d[~mask] = np.nan

        thr = max(0.0, float(min_d))
        d[~np.isfinite(d)] = np.nan
        d[d <= thr] = np.nan

        Z = (float(f_px) * float(T_m)) / d
        Z[~np.isfinite(Z)] = np.nan
        return Z

    def rescaled_intrinsics_for_detector(
        self, fx_orig: float, cx_orig: float, cy_orig: float,
        orig_w: int, orig_h: int, det_w: int, det_h: int
    ):
        sx = det_w / float(orig_w)
        sy = det_h / float(orig_h)
        fx_r = fx_orig * sx
        cx_r = cx_orig * sx
        cy_r = cy_orig * sy
        return fx_r, cx_r, cy_r

    def _robust_patch_depth(self, patch: np.ndarray, mad_k: float, min_inliers: int):
        p = patch[np.isfinite(patch)]
        if p.size == 0:
            return None, 0, (np.nan, np.nan)
        med = np.median(p)
        mad = np.median(np.abs(p - med))
        if not np.isfinite(mad) or mad == 0.0:
            inliers = p
        else:
            lo = med - mad_k * mad
            hi = med + mad_k * mad
            inliers = p[(p >= lo) & (p <= hi)]
        if inliers.size < min_inliers:
            return None, inliers.size, (med, mad)
        z_hat = float(np.median(inliers))
        return z_hat, inliers.size, (med, mad)

    def _nearest_finite_z(self, Zmap, cx, cy, r_max=40):
        """Search outward for the nearest finite Z within a growing radius (pixels)."""
        h, w = Zmap.shape[:2]
        for r in range(3, r_max + 1, 3):
            x0 = max(0, cx - r); x1 = min(w, cx + r + 1)
            y0 = max(0, cy - r); y1 = min(h, cy + r + 1)
            ring = Zmap[y0:y1, x0:x1]
            if ring.size == 0:
                continue
            finite = np.isfinite(ring)
            if finite.any():
                return float(np.nanmedian(ring[finite]))
        return None

    def _ema_smooth(self, tid: int, z_now: float):
        if tid is None or tid < 0 or not np.isfinite(z_now):
            return z_now
        prev = self._z_ema.get(tid)
        if prev is None:
            self._z_ema[tid] = (z_now, self.frame)
            return z_now
        z_prev, _ = prev
        z_in = z_now
        if self.MAX_DZ_PER_FRAME is not None and np.isfinite(z_prev):
            dz = np.clip(z_now - z_prev, -self.MAX_DZ_PER_FRAME, self.MAX_DZ_PER_FRAME)
            z_in = z_prev + dz
        alpha = self.EMA_ALPHA
        z_smooth = alpha * z_prev + (1.0 - alpha) * z_in
        self._z_ema[tid] = (z_smooth, self.frame)
        return z_smooth

    def _z_change_significant(self, z_last: float, z_new: float) -> bool:
        dz = abs(z_new - z_last)
        return (dz >= self.Z_ABS_GATE) and (dz >= self.Z_REL_GATE * max(1e-9, z_last))

    def process_frames(self) -> None:
        if not self.disparity_data or not self.color_times:
            self.node.get_logger().info("Waiting for both color and disparity images...")
            time.sleep(0.05)
            return

        disp_ts, disp_image, f_px, baseline_m, min_d, roi = self.disparity_data

        if disp_ts == self.last_tick:
            time.sleep(0.01)
            return

        # nearest color pairing
        color_ts, cv_image = self._nearest_color(disp_ts)
        if color_ts is None or cv_image is None:
            time.sleep(0.02)
            return

        dt = abs(color_ts - disp_ts)
        if dt > IMAGE_TS_DELTA_THRESH:
            self.node.get_logger().error(
                f"Timestamp mismatch: color {color_ts:.3f}s vs disparity {disp_ts:.3f}s"
            )
            if color_ts > disp_ts:
                return
            # drop stale color and try next
            self.color_dict.pop(color_ts, None)
            try:
                self.color_times.remove(color_ts)
            except ValueError:
                pass
            return

        # consume matched color
        self.color_dict.pop(color_ts, None)
        try:
            self.color_times.remove(color_ts)
        except ValueError:
            pass

        # resize color to working size
        resize_img = cv2.resize(cv_image, (DET_W, DET_H))
        height, width = DET_H, DET_W
        self.node.get_logger().info(f"disp image shape: {disp_image.shape}, cv_image shape: {resize_img.shape}")

        # detection (tiled)
        dets_xywh, det_scores, _ = run_detector_tiled(
            self.detector,
            resize_img,
            SCORE_THRESH,
            TILE_ROWS,
            TILE_COLS,
            TILE_OVERLAP,
            nms_thresh=TILE_NMS,
        )
        ids = self.tracker.update(dets_xywh) if dets_xywh else []

        Zmap = None
        if not NO_DEPTH:
            self.node.get_logger().info("Computing depth map from disparity...")
            Z_orig = self.disparity_to_depth_m(
                disp_image,
                f_px,
                baseline_m,
                min_d,
                roi,
            )
            # scene snapshot
            finite = np.isfinite(Z_orig)
            if np.any(finite):
                p = np.percentile(Z_orig[finite], [5, 50, 95]).astype(float)
                self.node.get_logger().info(f"Z m p5/50/95: {p[0]:.3f}/{p[1]:.3f}/{p[2]:.3f}")
            else:
                self.node.get_logger().info("Depth frame is all-NaN after masking.")

            Zmap = cv2.resize(Z_orig, (width, height), interpolation=cv2.INTER_NEAREST)
            self.node.get_logger().info(f"zmap: {Zmap.shape}")

            # (Optional) log valid_window mapped to detector coords
            if roi and (roi.width > 0 and roi.height > 0):
                H0, W0 = disp_image.shape[:2]
                sx = width / float(W0)
                sy = height / float(H0)
                rx0 = int(round(roi.x_offset * sx))
                ry0 = int(round(roi.y_offset * sy))
                rx1 = int(round((roi.x_offset + roi.width) * sx))
                ry1 = int(round((roi.y_offset + roi.height) * sy))
                self.node.get_logger().info(
                    f"valid_window(detector coords): x=[{rx0},{rx1}) y=[{ry0},{ry1})"
                )
                # Visualize ROI box on output image (thin blue line)
                cv2.rectangle(resize_img, (rx0, ry0), (rx1, ry1), (255, 0, 0), 1)

        # rescale intrinsics to detector space for XY
        no_intrinsics = (
            self.fx is None or self.cx is None or self.cy is None or
            self.orig_w is None or self.orig_h is None
        )
        if not no_intrinsics:
            fx_r, cx_r, cy_r = self.rescaled_intrinsics_for_detector(
                self.fx, self.cx, self.cy, self.orig_w, self.orig_h, width, height
            )
        else:
            fx_r = cx_r = cy_r = None

        # iterate detections
        for i, xywh in enumerate(dets_xywh):
            x, y, w, h = map(int, xywh)
            u, v = x + 0.5 * w, y + 0.5 * h
            tid = ids[i] if i < len(ids) else -1
            score = det_scores[i]

            X = Y = None
            Zs = None
            depth_method = ""

            # is center inside ROI?
            inside = True
            rx0 = ry0 = rx1 = ry1 = None
            if (Zmap is not None) and (roi and (roi.width > 0 and roi.height > 0)):
                H0, W0 = disp_image.shape[:2]
                sx = width / float(W0); sy = height / float(H0)
                rx0 = int(round(roi.x_offset * sx)); ry0 = int(round(roi.y_offset * sy))
                rx1 = int(round((roi.x_offset + roi.width) * sx))
                ry1 = int(round((roi.y_offset + roi.height) * sy))
                inside = (rx0 <= u < rx1) and (ry0 <= v < ry1)
                self.node.get_logger().info(
                    f"center ({u:.1f},{v:.1f}) insideROI={inside}"
                )

            if Zmap is not None:
                self.node.get_logger().info(f"Extracting depth for detection {i} (tid={tid})")

                # optionally clamp center into ROI to avoid holes at the edge
                cxp, cyp = int(round(u)), int(round(v))
                if rx0 is not None:
                    cxp = int(np.clip(cxp, rx0 + 1, rx1 - 2))
                    cyp = int(np.clip(cyp, ry0 + 1, ry1 - 2))

                wx = max(1, int(round(w * self.Z_PATCH_FRAC)))
                wy = max(1, int(round(h * self.Z_PATCH_FRAC)))
                x0 = max(0, cxp - wx)
                x1 = min(width, cxp + wx)
                y0 = max(0, cyp - wy)
                y1 = min(height, cyp + wy)
                patch = Zmap[y0:y1, x0:x1]

                Zm = None
                ninliers = 0
                mad_all = np.nan

                if patch.size > 0:
                    z_hat, ninliers, (med_all, mad_all) = self._robust_patch_depth(
                        patch, self.MAD_K, self.MIN_INLIERS
                    )
                    if z_hat is None:
                        # fallback to plain median
                        med = np.nanmedian(patch)
                        if np.isfinite(med):
                            Zm = float(med)
                        self.node.get_logger().info(
                            f"Depth patch fallback ninliers={ninliers} med={np.float32(med_all):.3f} mad={np.float32(mad_all):.3f}"
                        )
                    else:
                        Zm = z_hat
                        self.node.get_logger().info(
                            f"Depth patch robust ninliers={ninliers} med={np.float32(med_all):.3f} mad={np.float32(mad_all):.3f}"
                        )

                # if patch is invalid, try nearest finite around the center
                if (Zm is None) or (not np.isfinite(Zm)):
                    nf = self._nearest_finite_z(Zmap, cxp, cyp, r_max=40)
                    if nf is not None and np.isfinite(nf):
                        Zm = nf
                        self.node.get_logger().info(
                            f"Depth center-nearest fallback used (tid={tid}): Z={Zm:.4f}m"
                        )

                # ---- stillness + anti-drift gating + sticky Z ----
                # stillness check
                still = False
                if tid in self._uv_last:
                    u_last, v_last = self._uv_last[tid]
                    du = u - u_last
                    dv = v - v_last
                    still = (du * du + dv * dv) ** 0.5 < self.UV_STILL_PX
                self._uv_last[tid] = (u, v)

                # patch quality
                quality_ok = (ninliers >= self.MIN_INLIERS) and np.isfinite(mad_all) and float(mad_all) <= self.MAD_MAX

                # default: no update yet
                if Zm is not None and np.isfinite(Zm) and Zm > 0 and quality_ok:
                    if tid in self._z_last:
                        z_last = self._z_last[tid]
                        if still:
                            # object visually still -> hold last value
                            Zs = z_last
                            depth_method = "stereo(held-still)"
                        else:
                            # apply anti-drift gate
                            if not self._z_change_significant(z_last, Zm):
                                Zs = z_last
                                depth_method = "stereo(held)"
                            else:
                                # require confirmation across frames
                                pend = self._z_pending.get(tid, {"z": Zm, "count": 0})
                                if np.isfinite(pend["z"]) and abs(pend["z"] - Zm) < max(self.Z_ABS_GATE, self.Z_REL_GATE * max(1e-9, Zm)):
                                    pend["count"] += 1
                                else:
                                    pend = {"z": Zm, "count": 1}
                                self._z_pending[tid] = pend

                                if pend["count"] >= self.Z_CONFIRM:
                                    Zs = self._ema_smooth(tid, Zm)
                                    if np.isfinite(Zs) and Zs > 0:
                                        self._z_last[tid] = float(Zs)
                                    self._z_pending.pop(tid, None)
                                    depth_method = "stereo"
                                else:
                                    Zs = z_last
                                    depth_method = "stereo(hold-confirm)"
                    else:
                        # first time for this tid
                        Zs = self._ema_smooth(tid, Zm)
                        if np.isfinite(Zs) and Zs > 0:
                            self._z_last[tid] = float(Zs)
                        self._z_pending.pop(tid, None)
                        depth_method = "stereo(first)"
                else:
                    # no valid/quality measurement this frame -> sticky reuse if available
                    if tid in self._z_last:
                        Zs = float(self._z_last[tid])
                        depth_method = "stereo(reuse)"

                # compute X,Y when we have Zs and intrinsics
                if Zs is not None and np.isfinite(Zs) and Zs > 0 and not no_intrinsics:
                    X = ((u - cx_r) * Zs) / fx_r
                    Y = ((v - cy_r) * Zs) / fx_r
                    if depth_method == "":
                        depth_method = "stereo"
                    self.node.get_logger().info(
                        f"Detection {i} (tid={tid}): X={X:.4f}m, Y={Y:.4f}m, Z={Zs:.4f}m"
                    )
            else:
                # no Zmap → optionally keep previous Z
                if tid in self._z_last:
                    Zs = float(self._z_last[tid])
                    depth_method = "stereo(reuse)"
                    if not no_intrinsics:
                        X = ((u - cx_r) * Zs) / fx_r
                        Y = ((v - cy_r) * Zs) / fx_r
                    self.node.get_logger().info(
                        f"Reusing last Z for detection {i} (tid={tid}) without new depth: Z={Zs:.4f}m"
                    )

            # annotate like your original
            self.annotate_image(resize_img, x, y, w, h, tid, NAME, score, Zs)

            # write CSV
            self.csv_writer.writerow(
                [
                    self.frame,
                    f"{disp_ts:.6f}",
                    tid,
                    NAME,
                    f"{score:.4f}",
                    f"{u:.2f}",
                    f"{v:.2f}",
                    x,
                    y,
                    x + w,
                    y + h,
                    "None" if X is None else f"{X:.4f}",
                    "None" if Y is None else f"{Y:.4f}",
                    "None" if Zs is None else f"{Zs:.4f}",
                    depth_method,
                ]
            )

        # FPS line (kept)
        fps = 1.0 / max(1e-9, (disp_ts - self.last_tick))
        self.last_tick = disp_ts
        self.node.get_logger().info(
            f"Frame {self.frame}: Detected {len(dets_xywh)} objects, FPS: {fps:.2f}"
        )
        self.frame += 1

        # publish processed image (kept)
        processed_img_msg: Image = self.br.cv2_to_imgmsg(resize_img, encoding="bgr8")
        processed_img_msg.header.stamp.sec = int(disp_ts)
        processed_img_msg.header.stamp.nanosec = int((disp_ts - int(disp_ts)) * 1e9)
        self.processed_pub.publish(processed_img_msg)

    def process_frames_threaded(self) -> None:
        while rclpy.ok():
            start_time = time.time()
            self.process_frames()
            elapsed = time.time() - start_time
            if MAX_FPS > 0:
                sleep_time = max(0.0, (1.0 / MAX_FPS) - elapsed)
                time.sleep(sleep_time)

    # ------------------------ Helpers (kept look) ------------------------

    def annotate_image(self, cv_image, x, y, w, h, tid, name, score, Zm=None) -> None:
        color = (0, 220, 0) if Zm is not None and np.isfinite(Zm) else (80, 80, 80)
        cv2.rectangle(cv_image, (x, y), (x + w, y + h), color, 2)
        label = f"id {tid}  {name} {score:.2f}" + (
            f"  Z={Zm:.2f}m" if Zm is not None and np.isfinite(Zm) else "None"
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
        try:
            self.csv_file.close()
        except Exception:
            pass


def main():
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
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
