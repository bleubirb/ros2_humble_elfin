from dataclasses import dataclass
from typing import List, Tuple

import numpy as np


@dataclass
class Track:
    tid: int
    cx: float
    cy: float


class CentroidTracker:
    def __init__(self, max_dist_px=60, next_id_start=1):
        self.maxd = float(max_dist_px)
        self.next_id = int(next_id_start)
        self.tracks: List[Track] = []

    def update(self, boxes_xywh: List[Tuple[int, int, int, int]]) -> List[int]:
        if not boxes_xywh:
            return []
        centers = np.array(
            [[x + w / 2.0, y + h / 2.0] for (x, y, w, h) in boxes_xywh],
            dtype=np.float32,
        )
        if centers.ndim != 2 or centers.shape[0] == 0:
            return []
        ids = [-1] * len(boxes_xywh)
        if not self.tracks:
            for i in range(len(boxes_xywh)):
                ids[i] = self.next_id
                self.tracks.append(Track(ids[i], centers[i, 0], centers[i, 1]))
                self.next_id += 1
            return ids
        T = np.array([[t.cx, t.cy] for t in self.tracks], dtype=np.float32)
        if T.ndim != 2 or T.shape[0] == 0:
            for i in range(len(boxes_xywh)):
                ids[i] = self.next_id
                self.tracks.append(Track(ids[i], centers[i, 0], centers[i, 1]))
                self.next_id += 1
            return ids
        D = np.linalg.norm(T[:, None, :] - centers[None, :, :], axis=2)
        used_t, used_n = set(), set()
        while True:
            mi = mj = -1
            mv = np.inf
            for i in range(D.shape[0]):
                if i in used_t:
                    continue
                for j in range(D.shape[1]):
                    if j in used_n:
                        continue
                    if D[i, j] < mv:
                        mi, mj, mv = i, j, D[i, j]
            if mi < 0:
                break
            if mv <= self.maxd:
                ids[mj] = self.tracks[mi].tid
                self.tracks[mi].cx, self.tracks[mi].cy = centers[mj, 0], centers[mj, 1]
                used_t.add(mi)
                used_n.add(mj)
            else:
                break
        for j in range(len(boxes_xywh)):
            if ids[j] == -1:
                ids[j] = self.next_id
                self.tracks.append(Track(ids[j], centers[j, 0], centers[j, 1]))
                self.next_id += 1
        return ids
