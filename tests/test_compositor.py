"""The GCS compositor: the background warped by the camera's motion, and patches pasted on the
newest track boxes, checked against where the camera model says things are."""

import numpy as np
import pytest

from aegislink.common import config, geo, messages
from aegislink.gcs import compositor
from aegislink.gcs.poses import PoseHistory
from aegislink.gcs.receiver import Keyframe, Patch, View
from aegislink.uav.camera import NadirCamera, Pose
from aegislink.uav.encoder import KEYFRAME_SIZE

ORIGIN = (46.0, 7.0)
CAMERA = NadirCamera()
W, H = config.FRAME_WIDTH, config.FRAME_HEIGHT


def telemetry(frame, east, north, alt=150.0, heading=0.0):
    lat, lon = geo.enu_to_latlon(east, north, ORIGIN)
    return messages.TelemetryMsg(frame, float(lat), float(lon), alt, 10.0, heading, 0, 0)


def history(*items):
    poses = PoseHistory()
    for t in items:
        poses.add(t)
    return poses


def test_positions_are_relative_to_the_first_telemetry_and_interpolated():
    poses = history(telemetry(10, 100.0, 50.0), telemetry(12, 104.0, 46.0))
    assert poses.position(10) == pytest.approx((0.0, 0.0), abs=0.01)
    assert poses.position(11) == pytest.approx((2.0, -2.0), abs=0.01)
    assert poses.position(12) == pytest.approx((4.0, -4.0), abs=0.01)
    assert poses.position(9) is None and poses.position(13) is None
    pose = poses.pose(11, alt=120.0, heading_deg=30.0)
    assert (pose.alt, pose.heading_deg) == (120.0, 30.0)


def test_history_keeps_ten_seconds_and_can_be_cleared():
    poses = history(*(telemetry(f, f * 0.1, 0.0) for f in range(400)))
    assert poses.position(400 - 10 * config.FPS - 1) is None
    assert poses.position(399) == pytest.approx((39.9, 0.0), abs=0.01)
    copy = poses.copy()
    poses.clear()
    assert poses.position(399) is None and copy.position(399) is not None


def marked_keyframe(pixel):
    """A dark keyframe with one bright 2x2 dot at `pixel` (keyframe coordinates)."""
    image = np.full((KEYFRAME_SIZE[1], KEYFRAME_SIZE[0]), 40, np.uint8)
    x, y = pixel
    image[y:y + 2, x:x + 2] = 255
    return image


def brightest(image):
    """The centre of the brightest spot."""
    gray = image[:, :, 1].astype(float)
    ys, xs = np.nonzero(gray >= 0.8 * gray.max())
    return np.array([xs.mean(), ys.mean()])


@pytest.mark.parametrize("move", [(0, 0, 0, 0), (12, -7, 0, 0), (0, 0, 20, 0), (5, 3, -15, 10)])
def test_background_lands_where_the_camera_sees_its_ground_point(move):
    de, dn, dheading, dalt = move
    key_pose = Pose(0.0, 0.0, 150.0, 10.0)
    view_pose = Pose(de, dn, 150.0 + dalt, 10.0 + dheading)
    # The dot's ground point, seen from the keyframe's pose.
    key_pixel = (240, 135)
    full_pixel = (np.array(key_pixel) + 0.5 + 0.5) * W / KEYFRAME_SIZE[0] - 0.5
    ground = CAMERA.pixel_to_ground(key_pose, full_pixel)
    expected = CAMERA.ground_to_pixel(view_pose, ground)

    poses = history(telemetry(1, 0.0, 0.0), telemetry(5, de, dn))
    view = View(
        telemetry=telemetry(5, de, dn, view_pose.alt, view_pose.heading_deg),
        tracks_frame=5, tracks=[],
        keyframe=Keyframe(messages.KeyframeMsg(1, 150.0, 10.0, b""), marked_keyframe(key_pixel)),
        patches={}, poses=poses)
    image = compositor.render_fovea(view, (W, H))
    assert image.shape == (H, W, 3)
    assert np.abs(brightest(image) - expected).max() <= 1.0


def test_without_poses_the_background_is_only_scaled():
    view = View(
        telemetry=None, tracks_frame=None, tracks=[],
        keyframe=Keyframe(messages.KeyframeMsg(1, 150.0, 0.0, b""), marked_keyframe((100, 50))),
        patches={}, poses=PoseHistory())
    image = compositor.render_fovea(view, (W // 2, H // 2))
    expected = (np.array([100.5, 50.5]) + 0.5) * (W / 2) / KEYFRAME_SIZE[0] - 0.5
    assert np.abs(brightest(image) - expected).max() <= 0.5


def colour_patch(width, height, colour=(0, 0, 255)):
    return np.full((height, width, 3), colour, np.uint8)


def view_with_patch(crop, box, patch_frame=5, view_frame=5, patch_heading=0.0,
                    view_heading=0.0, image=None):
    x1, y1, x2, y2 = crop
    image = colour_patch(x2 - x1, y2 - y1) if image is None else image
    patch = Patch(messages.PatchMsg(patch_frame, 150.0, patch_heading, 3, crop, b""), image)
    poses = history(telemetry(patch_frame, 0.0, 0.0), telemetry(view_frame, 0.0, 0.0))
    return View(
        telemetry=telemetry(view_frame, 0.0, 0.0, heading=view_heading),
        tracks_frame=view_frame, tracks=[messages.TrackBox(3, box, True)], keyframe=None,
        patches={3: patch}, poses=poses)


def test_patch_of_this_frame_lands_on_its_crop():
    box = (600, 300, 640, 320)
    crop = (592, 292, 648, 328)
    image = compositor.render_fovea(view_with_patch(crop, box), (W, H), boxes=False)
    # Opaque in the middle, feathered to the background at its edge.
    assert tuple(image[310, 620]) == (0, 0, 255)
    assert tuple(image[293, 593])[2] < 255
    assert image[291, 620].max() == 0 and image[310, 591].max() == 0
    assert tuple(image[310, 647])[2] > 0


def test_patch_follows_the_newest_box():
    crop = (592, 292, 648, 328)
    image = compositor.render_fovea(
        view_with_patch(crop, (700, 400, 740, 420), patch_frame=2), (W, H), boxes=False)
    assert tuple(image[410, 720]) == (0, 0, 255)
    assert image[310, 620].max() == 0


def test_patch_turns_with_the_camera():
    # A patch whose left half is red and right half is blue, seen after the camera turned 90
    # degrees: image right (east at heading 0) is then image down... or up.
    image = colour_patch(40, 40, (255, 0, 0))
    image[:, :20] = (0, 0, 255)
    view = view_with_patch((600, 300, 640, 340), (612, 312, 628, 328), patch_frame=2,
                           view_heading=90.0, image=image)
    out = compositor.render_fovea(view, (W, H), boxes=False)
    m = CAMERA.motion_matrix(Pose(0, 0, 150, 0), Pose(0, 0, 150, 90))
    left_dir = m[:, :2] @ np.array([-1.0, 0.0])     # where "left in the patch" points now
    centre = np.array([619.5, 319.5])
    red_at = np.round(centre + 10 * left_dir).astype(int)
    blue_at = np.round(centre - 10 * left_dir).astype(int)
    assert tuple(out[red_at[1], red_at[0]]) == (0, 0, 255)
    assert tuple(out[blue_at[1], blue_at[0]]) == (255, 0, 0)


def test_patch_cut_at_the_image_edge_keeps_its_uncut_side_on_the_box():
    # The box runs past the right edge, so the crop was cut there: its left side (box edge - 8)
    # is the one to trust.
    box = (1250, 300, 1290, 320)
    crop = (1242, 292, 1280, 328)
    image = colour_patch(38, 36)
    image[:, 8:12] = (0, 255, 0)        # a green stripe at the box's left edge
    out = compositor.render_fovea(view_with_patch(crop, box, image=image), (W, H), boxes=False)
    assert tuple(out[310, 1251]) == (0, 255, 0)


def test_boxes_from_an_older_frame_move_with_the_camera():
    poses = history(telemetry(4, 0.0, 0.0), telemetry(5, 6.0, 0.0))
    view = View(
        telemetry=telemetry(5, 6.0, 0.0), tracks_frame=4,
        tracks=[messages.TrackBox(1, (600, 340, 640, 380), True)], keyframe=None, patches={},
        poses=poses)
    m = CAMERA.motion_matrix(Pose(0, 0, 150, 0), Pose(6, 0, 150, 0))
    x1, y1, x2, y2 = compositor.track_boxes(view, 1.0)[0][1]
    assert (x1, y1) == pytest.approx(m @ [600, 340, 1], abs=1)
    assert (x2, y2) == pytest.approx(m @ [640, 380, 1], abs=1)
    out = compositor.render_fovea(view, (W, H))
    assert out[:, :, 1].max() == 255        # the box is drawn in green


def test_render_without_anything_is_black_and_baseline_scales():
    view = View(None, None, [], None, {}, PoseHistory())
    assert compositor.render_fovea(view, (640, 360)).max() == 0
    assert compositor.render_baseline(None, (640, 360)).shape == (360, 640, 3)
    frame = np.full((H, W, 3), 99, np.uint8)
    assert compositor.render_baseline(frame, (640, 360))[100, 100, 0] == 99


def test_telemetry_repeating_a_frame_keeps_the_history():
    poses = history(*(telemetry(f, f * 1.0, 0.0) for f in range(100, 131)))
    poses.add(telemetry(130, 131.0, 0.0))     # positions are relative to frame 100 (100 m east)
    assert poses.position(100) == pytest.approx((0.0, 0.0), abs=0.01)
    assert poses.position(130) == pytest.approx((31.0, 0.0), abs=0.01)


@pytest.mark.parametrize("alt, jump", [(0.01, 1e6), (1e-6, 10.0), (3e38, 0.0), (0.0, 0.0)])
def test_extreme_poses_do_not_break_or_slow_the_view(alt, jump):
    import time
    poses = history(telemetry(4, 0.0, 0.0), telemetry(5, jump, 0.0, alt=alt))
    image = colour_patch(56, 36)
    view = View(
        telemetry=telemetry(5, jump, 0.0, alt=alt), tracks_frame=4,
        tracks=[messages.TrackBox(3, (600, 300, 640, 320), True)],
        keyframe=Keyframe(messages.KeyframeMsg(4, 150.0, 0.0, b""), marked_keyframe((10, 10))),
        patches={3: Patch(messages.PatchMsg(4, 150.0, 0.0, 3, (592, 292, 648, 328), b""), image)},
        poses=poses)
    start = time.perf_counter()
    out = compositor.render_fovea(view, (W, H))
    assert out.shape == (H, W, 3) and time.perf_counter() - start < 0.01


def estimate(track_id, state, mean, cov, path=()):
    from aegislink.gcs.ghost_tracks import Estimate
    return Estimate(track_id, state, 2.5 if state == "GHOST" else 0.0, np.array(mean, float),
                    np.array(cov, float), np.zeros(2), np.array(path, float).reshape(-1, 2),
                    (0.0, 0.0, 0.0))


def test_a_ghost_moves_its_box_and_patch_to_the_predicted_position():
    box = (600, 300, 640, 320)
    crop = (592, 292, 648, 328)
    view = view_with_patch(crop, box)
    # The box's centre is pixel (620, 310); the filter now puts the vehicle 10 m east of it.
    centre = CAMERA.pixel_to_ground(Pose(0, 0, 150, 0), (620, 310))
    view.ghosts = [estimate(3, "GHOST", centre + (10.0, 0.0), np.eye(2) * 0.5)]
    out = compositor.render_fovea(view, (W, H), boxes=False)
    shift = 10.0 / CAMERA.metres_per_pixel(150.0)
    assert tuple(out[310, round(620 + shift)]) == (0, 0, 255)
    assert out[310, 620].max() == 0
    # Still LIVE: the patch stays on its box.
    view.ghosts = [estimate(3, "LIVE", centre + (10.0, 0.0), np.eye(2) * 0.5)]
    out = compositor.render_fovea(view, (W, H), boxes=False)
    assert tuple(out[310, 620]) == (0, 0, 255)


def colour_mask(image, colour, tolerance=40):
    """Pixels close to `colour` (lines are anti-aliased)."""
    return np.all(np.abs(image.astype(int) - colour) <= tolerance, axis=2)


def test_ellipse_path_and_label_are_drawn_where_the_filter_says():
    view = View(telemetry(5, 0.0, 0.0), 5, [], None, {}, history(telemetry(5, 0.0, 0.0)))
    cov = np.diag([16.0, 4.0])      # semi-axes sqrt(5.991 x 16) = 9.8 m east, 4.9 m north
    view.ghosts = [estimate(8, "GHOST", (0.0, 0.0), cov, path=[(5.0, 0.0), (10.0, 0.0)])]
    out = compositor.render_fovea(view, (W, H))
    mpp = CAMERA.metres_per_pixel(150.0)
    ring = colour_mask(out, compositor.GHOST_COLOR)
    rows, cols = np.nonzero(ring)
    assert cols.max() - CAMERA.cx == pytest.approx(9.8 / mpp, abs=2)
    assert CAMERA.cy - rows.min() > 4.9 / mpp + 5     # the label sits above the ellipse
    # The path runs from 5 to 10 m east along the centre row.
    column = out[357:363, round(CAMERA.cx + 7.5 / mpp)].astype(int)   # anti-aliased: dimmer
    assert column[:, 0].max() > 150 and column[:, 2].max() > 150 and column[:, 1].max() < 100
    # LIVE: no ellipse, and the path only from 4.5 m out, clear of the vehicle.
    path = [(d, 0.0) for d in range(1, 21)]
    view.ghosts = [estimate(8, "LIVE", (0.0, 0.0), cov, path=path)]
    out = compositor.render_fovea(view, (W, H))
    live = colour_mask(out, compositor.LIVE_COLOR)
    assert not colour_mask(out, compositor.GHOST_COLOR).any()
    _, cols = np.nonzero(live)
    assert cols.min() - CAMERA.cx == pytest.approx(5 / mpp, abs=2)
    assert cols.max() - CAMERA.cx == pytest.approx(20 / mpp, abs=2)


def test_ghosts_are_not_drawn_without_a_view_pose():
    view = View(None, None, [], None, {}, PoseHistory())
    view.ghosts = [estimate(1, "GHOST", (0.0, 0.0), np.eye(2))]
    assert compositor.render_fovea(view, (320, 180)).max() == 0
