"""A real vehicle detector: a YOLO model exported to ONNX, run with onnxruntime on the GPU.

The model is Ultralytics YOLO26n-obb, trained on DOTA: aerial images taken from straight above,
as this camera sees the ground (models trained on COCO photos taken from the side found fewer
than half of the cars in a top-down clip). It finds oriented boxes, and its export has the box
selection built in, so its output is a fixed list of rows (centre x, centre y, width, height,
score, class, angle) in the input image's pixels. Only the two vehicle classes are kept, each as
the upright box around it, which is what the tracker works with. onnxruntime is an optional
dependency (`pip install -e ".[gpu]"`), imported only when a detector is made.
"""

from pathlib import Path

import cv2
import numpy as np

from aegislink.uav.detector import Detection

DEFAULT_MODEL = Path("models") / "yolo26n-obb.onnx"
VEHICLE_CLASSES = {9, 10}        # DOTA large vehicle, small vehicle
PAD_VALUE = 114                  # the grey Ultralytics pads with in training


def letterbox(frame, size):
    """BGR frame -> (1, 3, h, w) float RGB in 0..1 for an input of `size` (h, w): scaled to fit,
    centred, grey around it. Returns the blob, the scale and the (x, y) padding."""
    height, width = size
    scale = min(width / frame.shape[1], height / frame.shape[0])
    resized = cv2.resize(frame, (round(frame.shape[1] * scale), round(frame.shape[0] * scale)),
                         interpolation=cv2.INTER_LINEAR)
    pad_x = (width - resized.shape[1]) // 2
    pad_y = (height - resized.shape[0]) // 2
    canvas = np.full((height, width, 3), PAD_VALUE, np.uint8)
    canvas[pad_y:pad_y + resized.shape[0], pad_x:pad_x + resized.shape[1]] = resized
    blob = canvas[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255
    return blob, scale, (pad_x, pad_y)


def decode(rows, scale, pad, image_size, min_score):
    """Model output rows -> Detections in the frame's pixels (`image_size` is (width, height))."""
    width, height = image_size
    detections = []
    for cx, cy, w, h, score, cls, angle in rows:
        if score < min_score or int(cls) not in VEHICLE_CLASSES:
            continue
        c, s = abs(np.cos(angle)), abs(np.sin(angle))
        half_x, half_y = (w * c + h * s) / 2, (w * s + h * c) / 2
        x1, x2 = np.clip((np.array([cx - half_x, cx + half_x]) - pad[0]) / scale, 0, width - 1)
        y1, y2 = np.clip((np.array([cy - half_y, cy + half_y]) - pad[1]) / scale, 0, height - 1)
        if x2 - x1 < 2 or y2 - y1 < 2:
            continue
        detections.append(Detection((float(x1), float(y1), float(x2), float(y2)), float(score)))
    return detections


class YoloDetector:
    def __init__(self, model_path=DEFAULT_MODEL, min_score=0.25):
        import onnxruntime as ort

        # The CUDA and cuDNN libraries come as pip packages; this finds and loads them.
        ort.preload_dlls()
        # Errors only: it warns about ops it keeps on the CPU and about one CUDA kernel.
        ort.set_default_logger_severity(3)
        options = ort.SessionOptions()
        options.log_severity_level = 3
        self.session = ort.InferenceSession(
            str(model_path), options, providers=["CUDAExecutionProvider", "CPUExecutionProvider"])
        model_input = self.session.get_inputs()[0]
        self.input_name = model_input.name
        self.input_size = tuple(model_input.shape[2:4])   # (h, w)
        self.min_score = min_score

    @property
    def provider(self):
        """The execution provider in use: CUDAExecutionProvider on the GPU."""
        return self.session.get_providers()[0]

    def detect(self, frame):
        blob, scale, pad = letterbox(frame, self.input_size)
        rows = self.session.run(None, {self.input_name: blob})[0][0]
        return decode(rows, scale, pad, (frame.shape[1], frame.shape[0]), self.min_score)
