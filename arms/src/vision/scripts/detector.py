import math
from typing import List, Tuple

import cv2
import numpy as np
import torch
from model import get_model


def parse_anchor_sizes(s: str):
    if "|" in s:
        return tuple(
            tuple(int(x) for x in p.split(",") if x.strip()) for p in s.split("|")
        )
    nums = [int(x) for x in s.split(",") if x.strip()]
    return tuple((n,) for n in nums)


def parse_anchor_ratios(s: str, n_levels: int):
    ratios = tuple(float(x) for x in s.split(",") if x.strip())
    return tuple([ratios] * n_levels)


class ExactDetector:
    def __init__(
        self,
        ckpt_path,
        names="blueberry",
        anchors="64,128,256,384,512",
        ratios="0.75,1.0,1.5",
        min_size=800,
        max_size=1333,
        device=None,
    ):

        self.device = torch.device(
            device or ("cuda" if torch.cuda.is_available() else "cpu")
        )
        self.class_names = [s.strip() for s in names.split(",") if s.strip()]
        num_classes = len(self.class_names) + 1
        anchor_sizes = parse_anchor_sizes(anchors)
        anchor_ratios = parse_anchor_ratios(ratios, n_levels=len(anchor_sizes))
        print(
            f"[model] anchors: sizes={anchor_sizes} | ratios={anchor_ratios} | levels={len(anchor_sizes)}"
        )

        self.model = get_model(
            num_classes=num_classes,
            anchor_sizes=anchor_sizes,
            anchor_ratios=anchor_ratios,
        ).to(self.device)
        self.model.transform.min_size = (int(min_size),)
        self.model.transform.max_size = int(max_size)

        # robust checkpoint loader
        ckpt = torch.load(ckpt_path, map_location=self.device)
        if isinstance(ckpt, dict):
            for k in ("model_state", "state_dict", "model", "net", "network"):
                if k in ckpt and isinstance(ckpt[k], dict):
                    ckpt = ckpt[k]
                    break
            else:
                ckpt = {k: v for k, v in ckpt.items() if hasattr(v, "shape")}
        state = {(k[7:] if k.startswith("module.") else k): v for k, v in ckpt.items()}
        res = self.model.load_state_dict(state, strict=False)
        try:
            print(
                f"[load] missing={len(getattr(res,'missing_keys',[]))} "
                f"unexpected={len(getattr(res,'unexpected_keys',[]))}"
            )
        except Exception:
            pass

        # hard-force eval
        self.model.eval()
        self.model.train(False)
        print(f"Checkpoint loaded from {ckpt_path}")

    @torch.inference_mode()
    def __call__(self, bgr: np.ndarray, score_thresh: float = 0.5):
        if getattr(self.model, "training", True):
            self.model.eval()
            self.model.train(False)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        ten = (
            torch.from_numpy(rgb)
            .float()
            .div(255.0)
            .permute(2, 0, 1)
            .to(self.device, non_blocking=True)
        )
        out = self.model([ten])[0]
        boxes = out["boxes"].detach().cpu().numpy()
        scores = out["scores"].detach().cpu().numpy()
        labels = out["labels"].detach().cpu().numpy()
        dets = []
        for b, s, lab in zip(boxes, scores, labels):
            if s < score_thresh:
                continue
            x1, y1, x2, y2 = b.astype(int)
            dets.append(((x1, y1, x2 - x1, y2 - y1), float(s), int(lab)))
        return dets


def nms_xyxy(
    boxes: np.ndarray, scores: np.ndarray, iou_thresh: float = 0.5
) -> List[int]:
    if boxes.size == 0:
        return []
    x1, y1, x2, y2 = boxes.T
    areas = (x2 - x1 + 1) * (y2 - y1 + 1)
    order = scores.argsort()[::-1]
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(int(i))
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        w = np.maximum(0.0, xx2 - xx1 + 1)
        h = np.maximum(0.0, yy2 - yy1 + 1)
        inter = w * h
        iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-9)
        inds = np.where(iou <= iou_thresh)[0]
        order = order[inds + 1]
    return keep


def tile_boxes(
    W: int, H: int, rows: int, cols: int, overlap_frac: float
) -> List[Tuple[int, int, int, int]]:
    tw = math.ceil(W / cols)
    th = math.ceil(H / rows)
    ov = int(round(min(tw, th) * max(0.0, min(0.49, overlap_frac))))
    xs = []
    x = 0
    while x < W:
        xs.append(x)
        if x + tw >= W:
            break
        x = x + tw - ov
    ys = []
    y = 0
    while y < H:
        ys.append(y)
        if y + th >= H:
            break
        y = y + th - ov
    tiles = []
    for y0 in ys:
        for x0 in xs:
            x1 = min(W, x0 + tw)
            y1 = min(H, y0 + th)
            tiles.append((x0, y0, x1, y1))
    return tiles


def run_detector_tiled(
    detector: ExactDetector,
    bgr: np.ndarray,
    score_thresh: float,
    rows: int,
    cols: int,
    overlap_frac: float,
    nms_thresh: float = 0.5,
):
    H, W = bgr.shape[:2]
    tiles = tile_boxes(W, H, rows, cols, overlap_frac)
    all_xyxy = []
    all_scores = []
    all_labels = []
    for x0, y0, x1, y1 in tiles:
        crop = bgr[y0:y1, x0:x1]
        dets = detector(crop, score_thresh=score_thresh)
        for (x, y, w, h), s, lab in dets:
            # to full-image coords
            all_xyxy.append([x0 + x, y0 + y, x0 + x + w, y0 + y + h])
            all_scores.append(s)
            all_labels.append(lab if lab >= 1 else 1)
    if not all_xyxy:
        return [], [], []

    boxes = np.array(all_xyxy, dtype=np.float32)
    scores = np.array(all_scores, dtype=np.float32)
    labels = np.array(all_labels, dtype=np.int32)

    keep = nms_xyxy(boxes, scores, iou_thresh=nms_thresh)
    boxes = boxes[keep]
    scores = scores[keep]
    labels = labels[keep]

    dets_xywh = [
        (int(b[0]), int(b[1]), int(b[2] - b[0]), int(b[3] - b[1])) for b in boxes
    ]
    det_scores = scores.tolist()
    det_labels = labels.tolist()
    return dets_xywh, det_scores, det_labels
