"""Image encoding on the UAV and decoding at the GCS."""

import cv2
import numpy as np
import pytest

from aegislink.uav import encoder


@pytest.fixture(scope="module")
def frame():
    # Textured like the terrain: smooth colour plus fine noise.
    rng = np.random.default_rng(1)
    base = cv2.resize(rng.integers(0, 255, (9, 16, 3), dtype=np.uint8), (1280, 720))
    noise = rng.integers(-20, 20, (720, 1280, 3))
    return np.clip(base.astype(int) + noise, 0, 255).astype(np.uint8)


def test_keyframe_is_gray_and_downscaled(frame):
    data = encoder.encode_keyframe(frame)
    assert data[:4] == b"RIFF" and data[8:12] == b"WEBP"
    assert encoder.decode_image(data, gray=True).shape == (270, 480)
    # Gray: decoded in colour, the channels differ only by WebP's chroma rounding.
    image = encoder.decode_image(data).astype(int)
    assert image.shape == (270, 480, 3)
    assert np.abs(image[..., 0] - image[..., 2]).max() <= 3


def test_patch_box_is_padded_and_clipped():
    assert encoder.patch_box((100.4, 200.6, 140.2, 220.0), 1280, 720) == (92, 193, 148, 228)
    assert encoder.patch_box((-10, 700, 30, 730), 1280, 720) == (0, 692, 38, 720)
    assert encoder.patch_box((1300, 10, 1320, 30), 1280, 720) is None


def test_patch_round_trip_keeps_the_crop(frame):
    box, data = encoder.encode_patch(frame, (600, 300, 640, 320))
    assert box == (592, 292, 648, 328)
    image = encoder.decode_image(data)
    crop = frame[292:328, 592:648]
    assert image.shape == crop.shape
    # Lossy, but close to the original.
    assert np.abs(image.astype(int) - crop).mean() < 12
    assert len(data) < 1200


def test_patch_outside_the_image_is_none(frame):
    assert encoder.encode_patch(frame, (2000, 10, 2040, 30)) is None


def test_full_frame_is_jpeg(frame):
    data = encoder.encode_full_frame(frame)
    assert data[:2] == b"\xff\xd8"
    assert encoder.decode_image(data).shape == frame.shape


def test_garbage_decodes_to_none():
    assert encoder.decode_image(b"not an image") is None
    assert encoder.decode_image(b"") is None
