"""A simulated vehicle detector: the ground-truth boxes with the errors a real one makes.

It misses some vehicles (more often when trees hide part of them), puts noise on every box edge
and now and then reports a box where there is nothing. This keeps the synthetic runs honest: the
tracker downstream never sees perfect data.
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class Detection:
    box: tuple       # (x1, y1, x2, y2) pixels
    score: float     # confidence in [0, 1]


class SimulatedDetector:
    def __init__(self, width, height, seed=0, miss_rate=0.05, box_sigma_px=1.0,
                 box_sigma_frac=0.04, false_per_frame=0.05, min_visible=0.4, min_in_frame=0.5):
        self.width = width
        self.height = height
        self.rng = np.random.default_rng(seed)
        self.miss_rate = miss_rate              # for a vehicle in clear view
        self.box_sigma_px = box_sigma_px        # edge noise: sigma_px + sigma_frac x box size
        self.box_sigma_frac = box_sigma_frac
        self.false_per_frame = false_per_frame  # mean number of false boxes per frame
        self.min_visible = min_visible          # less visible than this is never detected
        self.min_in_frame = min_in_frame        # less of the box in the image: never detected

    def detection_probability(self, truth):
        if truth.box is None or truth.in_frame < self.min_in_frame:
            return 0.0
        # Falls linearly from the clear-view rate at 90 % visible to zero at min_visible.
        share = (truth.visible - self.min_visible) / (0.9 - self.min_visible)
        return (1 - self.miss_rate) * float(np.clip(share, 0.0, 1.0))

    def detect(self, truths):
        detections = []
        for truth in truths:
            if self.rng.random() >= self.detection_probability(truth):
                continue
            x1, y1, x2, y2 = truth.box
            size = np.array([x2 - x1, y2 - y1, x2 - x1, y2 - y1])
            noisy = np.array(truth.box) + self.rng.normal(
                0.0, self.box_sigma_px + self.box_sigma_frac * size)
            x1, x2 = np.clip(np.sort(noisy[[0, 2]]), 0, self.width - 1)
            y1, y2 = np.clip(np.sort(noisy[[1, 3]]), 0, self.height - 1)
            if x2 - x1 < 2 or y2 - y1 < 2:
                continue
            score = float(np.clip(0.55 + 0.4 * truth.visible + self.rng.normal(0, 0.05), 0.5, 0.99))
            detections.append(Detection((float(x1), float(y1), float(x2), float(y2)), score))

        for _ in range(self.rng.poisson(self.false_per_frame)):
            w, h = self.rng.uniform(15, 45), self.rng.uniform(8, 25)
            x1 = self.rng.uniform(0, self.width - 1 - w)
            y1 = self.rng.uniform(0, self.height - 1 - h)
            detections.append(Detection((x1, y1, x1 + w, y1 + h), float(self.rng.uniform(0.3, 0.6))))
        return detections
