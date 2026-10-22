"""The YOLO detector: image preparation and output decoding everywhere, the model itself only where
onnxruntime runs on a GPU and the model and a clip are in models/ and data/ (never on CI)."""

import time
from pathlib import Path

import cv2
import numpy as np
import pytest

from aegislink.uav.yolo_detector import DEFAULT_MODEL, decode, letterbox

ROOT = Path(__file__).resolve().parent.parent
CLIP = ROOT / "data" / "roundabout_tuzla.mp4"


def test_letterbox_keeps_the_aspect_and_centres_the_image():
    frame = np.full((720, 1280, 3), 200, np.uint8)
    frame[:, :, 0] = 10  # blue in BGR
    blob, scale, (pad_x, pad_y) = letterbox(frame, (384, 640))
    assert blob.shape == (1, 3, 384, 640) and blob.dtype == np.float32
    assert scale == 0.5 and (pad_x, pad_y) == (0, 12)
    # RGB order, 0..1, grey 114 in the padding.
    assert blob[0, 2, 100, 100] == pytest.approx(10 / 255)
    assert blob[0, 0, 100, 100] == pytest.approx(200 / 255)
    assert blob[0, 0, 5, 100] == pytest.approx(114 / 255)
    assert blob[0, 0, 384 - 6, 100] == pytest.approx(114 / 255)


def test_decode_maps_rotated_boxes_back_and_keeps_confident_vehicles():
    # Rows: centre x, centre y, width, height, score, class, angle (radians), in input pixels.
    rows = np.array([
        [120, 122, 40, 20, 0.9, 10, 0.0],             # small vehicle
        [230, 127, 60, 30, 0.6, 9, np.pi / 2],        # large vehicle, turned a quarter
        [300, 112, 20, 10, 0.2, 10, 0.0],             # too unsure
        [400, 132, 40, 40, 0.95, 12, 0.0],            # a roundabout: not a vehicle
        [5, 15, 50, 30, 0.8, 10, 0.0],                # partly in the padding, off the left edge
        [0, 0, 0, 0, 0.0, 0, 0.0],                    # an empty slot of the fixed-size output
    ], np.float32)
    detections = decode(rows, scale=0.5, pad=(0, 12), image_size=(1280, 720), min_score=0.3)
    assert [d.score for d in detections] == pytest.approx([0.9, 0.6, 0.8])
    assert detections[0].box == pytest.approx((200, 200, 280, 240))
    assert detections[1].box == pytest.approx((430, 170, 490, 290), abs=1e-3)
    assert detections[2].box == pytest.approx((0, 0, 60, 36))


def test_decode_takes_the_upright_box_around_a_turned_one():
    side = 20 * np.sqrt(2)
    rows = np.array([[100, 112, side, side, 0.9, 10, np.pi / 4]], np.float32)
    (detection,) = decode(rows, scale=1.0, pad=(0, 12), image_size=(1280, 720), min_score=0.3)
    assert detection.box == pytest.approx((80, 80, 120, 120), abs=1e-3)


def test_decode_drops_boxes_that_end_up_empty():
    rows = np.array([[800, 122, 200, 20, 0.9, 10, 0.0]], np.float32)  # right of the image
    assert decode(rows, scale=0.5, pad=(0, 12), image_size=(1280, 720), min_score=0.3) == []


def _gpu_detector():
    ort = pytest.importorskip("onnxruntime")
    if "CUDAExecutionProvider" not in ort.get_available_providers():
        pytest.skip("onnxruntime has no CUDA provider")
    if not (ROOT / DEFAULT_MODEL).is_file() or not CLIP.is_file():
        pytest.skip("model or clip not downloaded")
    from aegislink.uav.yolo_detector import YoloDetector
    return YoloDetector(ROOT / DEFAULT_MODEL)


def test_yolo_finds_the_cars_on_the_gpu():
    detector = _gpu_detector()
    assert detector.provider == "CUDAExecutionProvider"
    capture = cv2.VideoCapture(str(CLIP))
    ok, frame = capture.read()
    capture.release()
    assert ok
    frame = cv2.resize(frame, (1280, 720), interpolation=cv2.INTER_AREA)
    detector.detect(frame)  # the first run sets up the GPU
    times = []
    for _ in range(20):
        started = time.perf_counter()
        detections = detector.detect(frame)
        times.append(time.perf_counter() - started)
    # The first frame of the clip shows about 25 cars (8 of them parked); the model finds 19.
    assert len(detections) >= 15
    for d in detections:
        x1, y1, x2, y2 = d.box
        assert 0 <= x1 < x2 <= 1279 and 0 <= y1 < y2 <= 719
        assert 20 < max(x2 - x1, y2 - y1) < 200   # car-sized at this altitude
    assert np.median(times) < 0.015
