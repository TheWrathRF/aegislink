"""The foveated view at the GCS: the newest background keyframe moved with the camera since it was
taken, and each track's newest patch pasted on the track's newest box.

Every piece arrives in the pixels of the frame it was captured in. The view is drawn from the
camera pose of the newest telemetry, and each piece is mapped into it with the camera's motion
between its frame and the view's frame (`NadirCamera.motion_matrix`, one affine map for a nadir
camera over flat ground). Positions come from the telemetry (`PoseHistory`); altitude and heading
come with each image. Without a known position for either frame a piece is not moved.
"""

import cv2
import numpy as np

from aegislink.common import config
from aegislink.uav.camera import NadirCamera
from aegislink.uav.encoder import KEYFRAME_SIZE, PATCH_PAD

CAMERA = NadirCamera()
W, H = config.FRAME_WIDTH, config.FRAME_HEIGHT
# The patch's border fades into the background over this many pixels. The crop is padded by
# PATCH_PAD (8) around the box, so the vehicle itself stays opaque.
FEATHER_PX = 6
GREEN, AMBER = (0, 255, 0), (0, 200, 255)
# A piece whose camera motion scales it beyond this (an altitude ratio, possible only from bad
# telemetry) is not drawn: it would be meaningless, and slow to warp.
MAX_SCALE = 4.0


def _scale(sx, sy):
    """3x3 matrix that scales an image by (sx, sy), with pixel centres on integer coordinates."""
    return np.array([[sx, 0, 0.5 * sx - 0.5], [0, sy, 0.5 * sy - 0.5], [0, 0, 1.0]])


def _motion(view, frame, alt, heading_deg):
    """3x3 map from pixels of `frame`, seen at `alt` and `heading_deg`, to pixels of the view, or
    None when the motion is not usable (see MAX_SCALE)."""
    t = view.telemetry
    if t is None:
        return np.eye(3)
    view_pose = view.poses.pose(t.frame, t.alt_m, t.heading_deg)
    pose = view.poses.pose(frame, alt, heading_deg)
    if view_pose is None or pose is None:
        return np.eye(3)
    try:
        m = np.vstack([CAMERA.motion_matrix(pose, view_pose), [0, 0, 1.0]])
    except (ValueError, np.linalg.LinAlgError):
        return None
    scale = np.sqrt(abs(np.linalg.det(m[:2, :2])))
    if not (np.isfinite(m).all() and 1 / MAX_SCALE <= scale <= MAX_SCALE):
        return None
    return m


def _apply(m, points):
    return np.asarray(points, float) @ m[:2, :2].T + m[:2, 2]


def _view_boxes(view):
    """[(track, (x1, y1, x2, y2))] with the boxes moved into the view's frame (camera pixels)."""
    if not view.tracks:
        return []
    t = view.telemetry
    # Track lists carry no altitude or heading; they are a frame or two from the view at most,
    # so the view's own are used and only the position changes.
    m = np.eye(3) if t is None else _motion(view, view.tracks_frame, t.alt_m, t.heading_deg)
    if m is None:
        return []
    boxes = []
    for track in view.tracks:
        x1, y1, x2, y2 = track.box
        corners = _apply(m, [(x1, y1), (x2, y1), (x1, y2), (x2, y2)])
        boxes.append((track, (*corners.min(axis=0), *corners.max(axis=0))))
    return boxes


def track_boxes(view, scale):
    """The track boxes in the view, in pixels of an image `scale` times the camera's size."""
    s = _scale(scale, scale)
    result = []
    for track, (x1, y1, x2, y2) in _view_boxes(view):
        (a, b), (c, d) = _apply(s, [(x1, y1), (x2, y2)])
        result.append((track, (a, b, c, d)))
    return result


def _feather(width, height):
    x = np.arange(width)
    y = np.arange(height)
    edge = np.minimum.outer(np.minimum(y, y[::-1]), np.minimum(x, x[::-1]))
    return np.clip((edge + 0.5) / FEATHER_PX, 0, 1).astype(np.float32)


def _box_centre_in_crop(lo, hi, box_size, limit):
    """Where the track box's centre was inside a crop [lo, hi) along one axis. The crop is the
    box padded by PATCH_PAD and cut at the image edge (0 or `limit`), so when only one side was
    cut, the other side still sits PATCH_PAD from the box's edge."""
    cut_lo, cut_hi = lo <= 0, hi >= limit
    if cut_hi and not cut_lo:
        return lo + PATCH_PAD + (box_size - 1) / 2
    if cut_lo and not cut_hi:
        return hi - PATCH_PAD - 1 - (box_size - 1) / 2
    return (lo + hi - 1) / 2


def _paste(canvas, patch, box, view, out):
    """Pastes a patch so that its track's box lands on `box` (view pixels), turned and scaled
    with the camera's motion since the patch was taken."""
    msg = patch.msg
    motion = _motion(view, msg.frame, msg.alt_m, msg.heading_deg)
    if motion is None:
        return
    a = motion[:2, :2]
    x1, y1, x2, y2 = msg.box
    bx1, by1, bx2, by2 = box
    centre_in_crop = np.array([_box_centre_in_crop(x1, x2, bx2 - bx1, W),
                               _box_centre_in_crop(y1, y2, by2 - by1, H)])
    centre_in_view = np.array([(bx1 + bx2 - 1) / 2, (by1 + by2 - 1) / 2])
    # Patch pixel (i, j) is camera pixel (x1 + i, y1 + j) of its frame.
    to_view = np.eye(3)
    to_view[:2, :2] = a
    to_view[:2, 2] = centre_in_view + a @ (np.array([x1, y1]) - centre_in_crop)
    m = out @ to_view

    ph, pw = patch.image.shape[:2]
    corners = _apply(m, [(-0.5, -0.5), (pw - 0.5, -0.5), (-0.5, ph - 0.5), (pw - 0.5, ph - 0.5)])
    rx1, ry1 = np.maximum(np.floor(corners.min(axis=0)).astype(int), 0)
    rx2 = min(canvas.shape[1], int(np.ceil(corners[:, 0].max())) + 1)
    ry2 = min(canvas.shape[0], int(np.ceil(corners[:, 1].max())) + 1)
    if rx2 <= rx1 or ry2 <= ry1:
        return
    m[0, 2] -= rx1
    m[1, 2] -= ry1
    size = (rx2 - rx1, ry2 - ry1)
    image = cv2.warpAffine(patch.image, m[:2], size, flags=cv2.INTER_LINEAR,
                           borderMode=cv2.BORDER_CONSTANT)
    alpha = cv2.warpAffine(_feather(pw, ph), m[:2], size, flags=cv2.INTER_LINEAR,
                           borderMode=cv2.BORDER_CONSTANT)[:, :, None]
    roi = canvas[ry1:ry2, rx1:rx2]
    canvas[ry1:ry2, rx1:rx2] = (roi * (1 - alpha) + image * alpha + 0.5).astype(np.uint8)


def render_fovea(view, size, boxes=True):
    """The foveated view as a BGR image of `size` (width, height)."""
    w, h = size
    out = _scale(w / W, h / H)
    k = view.keyframe
    motion = None if k is None else _motion(view, k.msg.frame, k.msg.alt_m, k.msg.heading_deg)
    if motion is not None:
        to_camera = _scale(W / KEYFRAME_SIZE[0], H / KEYFRAME_SIZE[1])
        m = out @ motion @ to_camera
        gray = cv2.warpAffine(k.image, m[:2], (w, h), flags=cv2.INTER_LINEAR,
                              borderMode=cv2.BORDER_CONSTANT)
        canvas = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    else:
        canvas = np.zeros((h, w, 3), np.uint8)

    view_boxes = _view_boxes(view)
    for track, box in view_boxes:
        patch = view.patches.get(track.id)
        if patch is not None:
            _paste(canvas, patch, box, view, out)
    if boxes:
        for track, box in view_boxes:
            corners = np.clip(_apply(out, [box[:2], box[2:]]), -10 * w, 10 * w)
            (x1, y1), (x2, y2) = np.round(corners).astype(int)
            color = GREEN if track.seen else AMBER
            cv2.rectangle(canvas, (x1 - 2, y1 - 2), (x2 + 2, y2 + 2), color, 2)
            cv2.putText(canvas, str(track.id), (x1, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        color, 1, cv2.LINE_AA)
    return canvas


def render_baseline(image, size):
    """The baseline frame scaled to `size`, or black before the first one."""
    w, h = size
    if image is None:
        return np.zeros((h, w, 3), np.uint8)
    if (image.shape[1], image.shape[0]) == (w, h):
        return image
    return cv2.resize(image, (w, h), interpolation=cv2.INTER_AREA)
