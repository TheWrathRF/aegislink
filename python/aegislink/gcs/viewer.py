"""A minimal OpenCV view of what the GCS has received: the baseline next to the foveated stream.

The foveated view is the newest background keyframe (gray, scaled up) with each track's newest
patch pasted at the track's newest box, so a target moves at the track rate even when its patch
is old. The background is not moved with the camera here, so it slides out of line as the UAV
flies on until the next keyframe arrives.
"""

import cv2
import numpy as np

from aegislink._core import MsgType
from aegislink.common import config
from aegislink.uav.encoder import decode_image

PANE_WIDTH, PANE_HEIGHT = config.FRAME_WIDTH // 2, config.FRAME_HEIGHT // 2
HUD_HEIGHT = 44
GREEN, AMBER, WHITE = (0, 255, 0), (0, 200, 255), (255, 255, 255)


def render_fovea(station):
    w, h = config.FRAME_WIDTH, config.FRAME_HEIGHT
    if station.keyframe is not None:
        background = cv2.resize(station.keyframe.image, (w, h), interpolation=cv2.INTER_LINEAR)
        canvas = cv2.cvtColor(background, cv2.COLOR_GRAY2BGR)
    else:
        canvas = np.zeros((h, w, 3), np.uint8)
    for track in station.tracks:
        patch = station.patches.get(track.id)
        x1, y1, x2, y2 = track.box
        if patch is not None:
            _paste(canvas, patch.image, (x1 + x2) / 2, (y1 + y2) / 2)
        color = GREEN if track.seen else AMBER
        cv2.rectangle(canvas, (x1 - 2, y1 - 2), (x2 + 2, y2 + 2), color, 2)
        cv2.putText(canvas, str(track.id), (x1, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1,
                    cv2.LINE_AA)
    return canvas


def _paste(canvas, image, cx, cy):
    """Pastes `image` centred on (cx, cy), cut at the canvas edges."""
    ph, pw = image.shape[:2]
    x1, y1 = round(cx - pw / 2), round(cy - ph / 2)
    cx1, cy1 = max(0, x1), max(0, y1)
    cx2, cy2 = min(canvas.shape[1], x1 + pw), min(canvas.shape[0], y1 + ph)
    if cx2 <= cx1 or cy2 <= cy1:
        return
    canvas[cy1:cy2, cx1:cx2] = image[cy1 - y1:cy2 - y1, cx1 - x1:cx2 - x1]


def render_baseline(station):
    image = decode_image(station.full_frame.image) if station.full_frame else None
    if image is None or image.shape[:2] != (config.FRAME_HEIGHT, config.FRAME_WIDTH):
        return np.zeros((config.FRAME_HEIGHT, config.FRAME_WIDTH, 3), np.uint8)
    return image


def hud_lines(station, now_s):
    fovea = station.link_rate["fovea"].rate(now_s) * 8 / 1000
    baseline = station.link_rate["baseline"].rate(now_s) * 8 / 1000
    share = f"{fovea / baseline:.1%} of baseline" if baseline > 0 else "baseline silent"
    rates = {t: station.message_rate[t].rate(now_s) for t in station.message_rate}
    keyframe_age = "none"
    if station.keyframe is not None and station.telemetry is not None:
        age = (station.telemetry.frame - station.keyframe.msg.frame) / config.FPS
        keyframe_age = f"{max(0.0, age):.1f} s old"
    left = [f"{baseline:.1f} kbps", f"{rates[MsgType.FULL_FRAME]:.1f} frames/s"]
    right = [f"{fovea:.1f} kbps ({share})",
             f"tracks {rates[MsgType.TRACKS]:.1f} Hz  patches {rates[MsgType.ROI_PATCH]:.1f}/s  "
             f"keyframe {keyframe_age}"]
    return left, right


def compose(station, now_s):
    """Both panes side by side at half size, with two lines of numbers under each."""
    panes = [cv2.resize(render_baseline(station), (PANE_WIDTH, PANE_HEIGHT),
                        interpolation=cv2.INTER_AREA),
             cv2.resize(render_fovea(station), (PANE_WIDTH, PANE_HEIGHT),
                        interpolation=cv2.INTER_AREA)]
    image = np.zeros((PANE_HEIGHT + HUD_HEIGHT, 2 * PANE_WIDTH, 3), np.uint8)
    image[:PANE_HEIGHT, :PANE_WIDTH] = panes[0]
    image[:PANE_HEIGHT, PANE_WIDTH:] = panes[1]
    titles = ("BASELINE (full-frame JPEG)", "AEGISLINK (foveated)")
    for i, (title, lines) in enumerate(zip(titles, hud_lines(station, now_s))):
        x = i * PANE_WIDTH + 8
        cv2.putText(image, title, (x, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, WHITE, 1, cv2.LINE_AA)
        for j, line in enumerate(lines):
            cv2.putText(image, line, (x, PANE_HEIGHT + 18 + 18 * j), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, WHITE, 1, cv2.LINE_AA)
    return image
