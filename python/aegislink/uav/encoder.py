"""Image encoding for the two streams.

The foveated stream sends the background as a small gray keyframe and each target as a padded
colour crop, both WebP (3-4x smaller than JPEG at this size). The baseline sends whole frames as
JPEG.
"""

import cv2
import numpy as np

KEYFRAME_WIDTH = 480
KEYFRAME_QUALITY = 25
PATCH_PAD = 8
PATCH_QUALITY = 70
FULL_FRAME_QUALITY = 50


def encode_keyframe(frame, width=KEYFRAME_WIDTH, quality=KEYFRAME_QUALITY):
    height = round(frame.shape[0] * width / frame.shape[1])
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    small = cv2.resize(gray, (width, height), interpolation=cv2.INTER_AREA)
    return _encode(".webp", small, [cv2.IMWRITE_WEBP_QUALITY, quality])


def patch_box(box, width, height, pad=PATCH_PAD):
    """The padded crop around a track box, in whole pixels and clipped to the image; None if
    nothing of it is inside."""
    x1 = max(0, round(box[0]) - pad)
    y1 = max(0, round(box[1]) - pad)
    x2 = min(width, round(box[2]) + pad)
    y2 = min(height, round(box[3]) + pad)
    if x2 <= x1 or y2 <= y1:
        return None
    return x1, y1, x2, y2


def encode_patch(frame, box, pad=PATCH_PAD, quality=PATCH_QUALITY):
    """Returns (crop box, WebP bytes), or None if the box is outside the frame."""
    crop_box = patch_box(box, frame.shape[1], frame.shape[0], pad)
    if crop_box is None:
        return None
    x1, y1, x2, y2 = crop_box
    return crop_box, _encode(".webp", frame[y1:y2, x1:x2], [cv2.IMWRITE_WEBP_QUALITY, quality])


def encode_full_frame(frame, quality=FULL_FRAME_QUALITY):
    return _encode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])


def decode_image(data, gray=False):
    """A BGR (or gray) image, or None if the bytes are not a decodable image (they come off the
    network)."""
    if not data:
        return None
    flag = cv2.IMREAD_GRAYSCALE if gray else cv2.IMREAD_COLOR
    return cv2.imdecode(np.frombuffer(data, dtype=np.uint8), flag)


def _encode(ext, image, params):
    ok, buf = cv2.imencode(ext, image, params)
    if not ok:
        raise RuntimeError(f"OpenCV could not encode {ext}")
    return buf.tobytes()
