"""Image encoding for the two streams.

The foveated stream sends the background as a small gray keyframe and each target as a padded
colour crop, both WebP (3-4x smaller than JPEG at this size). The baseline sends whole frames as
JPEG.
"""

import cv2
import numpy as np

KEYFRAME_SIZE = (480, 270)
KEYFRAME_QUALITY = 25
PATCH_PAD = 8
PATCH_QUALITY = 70
FULL_FRAME_QUALITY = 50


def encode_keyframe(frame, size=KEYFRAME_SIZE, quality=KEYFRAME_QUALITY):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    small = cv2.resize(gray, size, interpolation=cv2.INTER_AREA)
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


def decode_image(data, gray=False, size=None):
    """A BGR (or gray) image, or None if the bytes are not a decodable image (they come off the
    network). With `size` (width, height) the image must be WebP or JPEG of exactly that size,
    checked in its header first: a few hundred bytes can claim a 16000 x 16000 image, which
    would take seconds and gigabytes to decode."""
    if not data:
        return None
    if size is not None and image_size(data) != tuple(size):
        return None
    flag = cv2.IMREAD_GRAYSCALE if gray else cv2.IMREAD_COLOR
    try:
        image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), flag)
    except cv2.error:   # for example a header over OpenCV's own pixel limit
        return None
    if image is not None and size is not None and image.shape[1::-1] != tuple(size):
        return None
    return image


def image_size(data):
    """(width, height) from a WebP or JPEG header without decoding; None for anything else."""
    data = bytes(data[:65536])
    if len(data) >= 30 and data[:4] == b"RIFF" and data[8:12] == b"WEBP":
        chunk = data[12:16]
        if chunk == b"VP8 " and data[23:26] == b"\x9d\x01\x2a":    # lossy
            return (int.from_bytes(data[26:28], "little") & 0x3FFF,
                    int.from_bytes(data[28:30], "little") & 0x3FFF)
        if chunk == b"VP8L" and data[20] == 0x2F:   # lossless: two 14-bit (size - 1)
            bits = int.from_bytes(data[21:25], "little")
            return (bits & 0x3FFF) + 1, ((bits >> 14) & 0x3FFF) + 1
        if chunk == b"VP8X":                        # extended: two 24-bit (size - 1)
            return (int.from_bytes(data[24:27], "little") + 1,
                    int.from_bytes(data[27:30], "little") + 1)
        return None
    if data[:2] != b"\xff\xd8":
        return None
    # JPEG: walk the marker segments to the first start of frame (SOF0-SOF15, except the DHT,
    # JPG and DAC markers that share that range).
    i = 2
    while i + 9 <= len(data) and data[i] == 0xFF:
        marker = data[i + 1]
        if marker == 0xFF:                          # fill byte
            i += 1
        elif 0xC0 <= marker <= 0xCF and marker not in (0xC4, 0xC8, 0xCC):
            height = int.from_bytes(data[i + 5:i + 7], "big")
            return int.from_bytes(data[i + 7:i + 9], "big"), height
        elif 0xD0 <= marker <= 0xD9 or marker == 0x01:  # markers without a length
            i += 2
        else:
            i += 2 + int.from_bytes(data[i + 2:i + 4], "big")
    return None


def _encode(ext, image, params):
    ok, buf = cv2.imencode(ext, image, params)
    if not ok:
        raise RuntimeError(f"OpenCV could not encode {ext}")
    return buf.tobytes()
