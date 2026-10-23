"""The GCS end of the links: receive both streams, keep the newest of everything, follow every
track with a Kalman filter, and send the receiver reports that set the UAV's budget."""

import json
import math
import select
import socket
import threading
import time
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from aegislink._core import MsgType, Packetizer
from aegislink.common import config, messages
from aegislink.common.net import LinkReceiver, RateMeter
from aegislink.gcs.ghost_tracks import GHOST, GhostTracks, semi_axes
from aegislink.gcs.poses import PoseHistory
from aegislink.gcs.spoof_detector import SpoofDetector
from aegislink.uav.encoder import KEYFRAME_SIZE, PATCH_PAD, decode_image, image_size

REPORT_INTERVAL_S = 0.25
TRACK_LOG_INTERVAL_S = 1 / config.FPS
# A keyframe is paced out at the budget rate: about 3 s for 5 KB at 50 kbps.
FOVEA_TIMEOUT_US = 4_000_000
# Sent the moment they are captured, so their delay is the link's delay.
CONTROL_TYPES = {MsgType.TELEMETRY, MsgType.TRACKS}
# Telemetry whose frame number jumps (more than JUMP_FRAMES below the held one: the UAV started
# again; or further ahead than the UAV can be by now) is followed only once JUMP_CONFIRM
# telemetry messages in a row continue the jump, so one forged packet cannot reset the station.
JUMP_FRAMES = 10 * config.FPS
JUMP_CONFIRM = 30
# How far a frame may be ahead of the UAV clock: arrival jitter (the UAV never runs faster).
AHEAD_FRAMES = config.FPS

USED, OLD, BAD = "used", "old", "bad"
# Altitudes outside this range (and any value that is not finite) cannot be drawn: the camera
# model divides by the altitude. They can only come from a broken or forged message.
MIN_ALT_M, MAX_ALT_M = 1.0, 10_000.0


def _sane_pose(alt, heading_deg):
    return MIN_ALT_M <= alt <= MAX_ALT_M and math.isfinite(heading_deg)


@dataclass
class Patch:
    msg: messages.PatchMsg
    image: np.ndarray       # BGR, the size of msg.box


@dataclass
class Keyframe:
    msg: messages.KeyframeMsg
    image: np.ndarray       # gray, downscaled


@dataclass
class View:
    """What the station held at one moment: everything the foveated view is drawn from."""
    telemetry: messages.TelemetryMsg | None
    tracks_frame: int | None
    tracks: list
    keyframe: Keyframe | None
    patches: dict               # track id -> Patch
    poses: PoseHistory = field(default_factory=PoseHistory)
    ghosts: list = field(default_factory=list)      # ghost_tracks.Estimate, predicted to now


class Measurement:
    """What arrived between its start and stop times: the summary of a run, headless or with the
    window."""

    def __init__(self):
        self.start_s = None
        self.stop_s = None
        self.bytes = Counter()          # link name -> datagram bytes
        self.messages = Counter()       # MsgType -> messages
        self.frames = {MsgType.TELEMETRY: set(), MsgType.TRACKS: set(), MsgType.FULL_FRAME: set()}

    @property
    def active(self):
        return self.start_s is not None and self.stop_s is None

    def summary(self):
        seconds = self.stop_s - self.start_s
        fovea = self.bytes["fovea"] * 8 / seconds / 1000
        baseline = self.bytes["baseline"] * 8 / seconds / 1000
        result = {"seconds": round(seconds, 2), "fovea_kbps": round(fovea, 1),
                  "baseline_kbps": round(baseline, 1),
                  "fovea_share": round(fovea / baseline, 4) if baseline else None}
        for msg_type, name in ((MsgType.TELEMETRY, "telemetry"), (MsgType.TRACKS, "tracks")):
            frames = self.frames[msg_type]
            span = max(frames) - min(frames) + 1 if frames else 0
            result[f"{name}_hz"] = round(len(frames) / seconds, 2)
            result[f"{name}_delivered"] = round(len(frames) / span, 4) if span else 0.0
        # The frames the UAV made, from the telemetry's frame numbers: the baseline's own span
        # would call one lone frame that got through a full delivery.
        telemetry = self.frames[MsgType.TELEMETRY]
        first, last = (min(telemetry), max(telemetry)) if telemetry else (0, -1)
        made = last - first + 1
        result["uav_fps"] = round(made / seconds, 2)
        result["patches_per_s"] = round(self.messages[MsgType.ROI_PATCH] / seconds, 2)
        result["keyframes"] = self.messages[MsgType.BG_KEYFRAME]
        result["baseline_fps"] = round(self.messages[MsgType.FULL_FRAME] / seconds, 2)
        # A baseline frame arrives later than its telemetry, so the first one measured may be
        # older than the first telemetry measured.
        arrived = sum(1 for f in self.frames[MsgType.FULL_FRAME] if first <= f <= last)
        result["baseline_delivered"] = round(arrived / made, 4) if made else 0.0
        return result


def _g(x):
    return float(f"{x:.6g}")


class TrackLog:
    """The tracks as the station predicts them, one JSON object per line, 30 times a second: the
    UAV time, how long the foveated link has been silent, the origin of the ground frame
    (latitude, longitude), and per track its state, time since the last fix, mean, covariance
    (xx, xy, yy) and the filtered position at the last fix (t, east, north). `nav` is the
    newest telemetry's frame, its GPS position, the position used for it and whether the GPS
    spoofing alarm is on."""

    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.path.open("w", encoding="utf-8", newline="\n")

    def write(self, t, silent_s, origin, estimates, nav=None):
        tracks = [{"id": e.id, "state": e.state, "tslf": round(e.tslf_s, 4),
                   "mean": [round(float(v), 3) for v in e.mean],
                   "cov": [_g(e.cov[0, 0]), _g(e.cov[0, 1]), _g(e.cov[1, 1])],
                   "fix": [round(e.fix[0], 4), round(float(e.fix[1]), 3),
                           round(float(e.fix[2]), 3)]}
                  for e in estimates]
        line = {"t": round(t, 4), "silent": None if silent_s is None else round(silent_s, 3),
                "origin": list(origin), "tracks": tracks, "nav": nav}
        self.file.write(json.dumps(line, separators=(",", ":")) + "\n")
        # A scenario run may stop this process at any moment: every line is complete on disk.
        self.file.flush()

    def close(self):
        self.file.close()


class GroundStation:
    def __init__(self, fovea_port, baseline_port, report_port, motion_model="cv",
                 track_log=None):
        self.sockets = {}
        for name, port in (("fovea", fovea_port), ("baseline", baseline_port)):
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
            s.bind(("127.0.0.1", port))
            s.setblocking(False)
            self.sockets[name] = s
        self.report_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.report_addr = ("127.0.0.1", report_port)
        self.report_packetizer = Packetizer()
        self.next_report_s = None
        self.links = {"fovea": LinkReceiver(FOVEA_TIMEOUT_US, delay_types=CONTROL_TYPES),
                      "baseline": LinkReceiver()}

        self.telemetry = None           # newest TelemetryMsg
        self.telemetry_arrived_s = None
        self.jump = None                # (frame, count) of telemetry that jumps, until confirmed
        self.poses = PoseHistory()      # positions of the recent frames
        self.tracks_frame = None
        self.tracks = []                # newest list of TrackBox
        self.ghosts = GhostTracks(motion_model)
        self.unmeasured = None          # (frame, tracks) that arrived before their telemetry
        self.track_log = None if track_log is None else TrackLog(track_log)
        self.next_log_s = None
        self.keyframe = None            # newest Keyframe
        self.spoof = SpoofDetector()    # checks the GPS against the keyframes' optical flow
        self.patches = {}               # track id -> newest Patch
        self.full_frame = None          # newest FullFrameMsg (decoded only when shown)
        self.bad_messages = 0           # complete messages that did not decode
        self.first_fovea_s = None
        self.last_fovea_s = None
        self.link_rate = {name: RateMeter() for name in self.sockets}       # bytes per second
        self.message_rate = {t: RateMeter() for t in MsgType}               # messages per second
        self.measurement = Measurement()

    def close(self):
        for s in self.sockets.values():
            s.close()
        self.report_tx.close()
        if self.track_log is not None:
            self.track_log.close()

    def poll(self, timeout_s):
        """Waits up to `timeout_s` for datagrams and handles all that are waiting."""
        self.read(self.wait(timeout_s))

    def wait(self, timeout_s):
        """Waits up to `timeout_s` for datagrams; returns the sockets that have some."""
        readable, _, _ = select.select(list(self.sockets.values()), [], [], timeout_s)
        return readable

    def read(self, readable):
        for name, s in self.sockets.items():
            if s not in readable:
                continue
            while True:
                try:
                    datagram = s.recv(65535)
                except BlockingIOError:
                    break
                self._receive(name, datagram, time.monotonic())

    def _receive(self, name, datagram, now_s):
        if name == "fovea":
            if self.first_fovea_s is None:
                self.first_fovea_s = now_s
            self.last_fovea_s = now_s
        self.link_rate[name].add(now_s, len(datagram))
        if self.measurement.active:
            self.measurement.bytes[name] += len(datagram)
        message = self.links[name].push(datagram, int(now_s * 1e6))
        if message is None:
            return
        try:
            body = messages.decode(message.type, message.data)
        except ValueError:
            self.bad_messages += 1
            return
        result = self._apply(message.type, body, now_s)
        if result == BAD:
            self.bad_messages += 1
        if result != USED:
            return
        self.message_rate[message.type].add(now_s)
        if self.measurement.active:
            self.measurement.messages[message.type] += 1
            if message.type in self.measurement.frames:
                frame = body[0] if message.type == MsgType.TRACKS else body.frame
                self.measurement.frames[message.type].add(frame)

    def _plausible(self, frame, now_s):
        """Whether `frame` fits the held telemetry: at most JUMP_FRAMES older, and not ahead of
        where the UAV clock says the UAV can be by now. Anything fits before any telemetry."""
        if self.telemetry is None:
            return True
        return (self.telemetry.frame - JUMP_FRAMES <= frame
                <= self.uav_time(now_s) * config.FPS + AHEAD_FRAMES)

    def _newer(self, frame, held, now_s):
        """Whether `frame` replaces the held one of its kind. A held frame that no longer fits
        the telemetry (forged, or from before a restart) gives way to any that does."""
        if not self._plausible(frame, now_s):
            return False
        if held is None or frame >= held:
            return True
        if self.telemetry is None:
            return held - frame > JUMP_FRAMES
        return not self._plausible(held, now_s)

    def _jump_confirmed(self, frame):
        """Counts telemetry that continues a jump; True for the JUMP_CONFIRM-th in a row."""
        follows = self.jump is not None and 0 < frame - self.jump[0] <= JUMP_FRAMES
        count = self.jump[1] + 1 if follows else 1
        self.jump = None if count >= JUMP_CONFIRM else (frame, count)
        return count >= JUMP_CONFIRM

    def _apply(self, msg_type, body, now_s=None):
        """Keeps the body if it is newer than what is held: USED, OLD (ignored) or BAD."""
        now_s = time.monotonic() if now_s is None else now_s
        if msg_type in (MsgType.TELEMETRY, MsgType.ROI_PATCH, MsgType.BG_KEYFRAME):
            if not _sane_pose(body.alt_m, body.heading_deg):
                return BAD
        if msg_type == MsgType.TELEMETRY:
            held = self.telemetry.frame if self.telemetry else None
            if held is not None and not self._plausible(body.frame, now_s):
                if not self._jump_confirmed(body.frame):
                    return OLD
                if body.frame < held:       # the UAV started again
                    self._forget()
            # The UAV sends one telemetry per frame, so the held frame again is a copy (a replay
            # would turn the UAV clock back to when the original arrived).
            elif held is not None and body.frame <= held:
                return OLD
            else:
                self.jump = None            # the stream goes on: a pending jump was forged
            self.telemetry = body
            self.telemetry_arrived_s = now_s
            self.poses.add(body)
            if self.unmeasured is not None:
                self._measure(*self.unmeasured)
        elif msg_type == MsgType.TRACKS:
            frame, tracks = body
            if not self._newer(frame, self.tracks_frame, now_s):
                return OLD
            self.tracks_frame, self.tracks = frame, tracks
            ids = {t.id for t in tracks}
            self.ghosts.keep_only(ids)
            self._measure(frame, tracks)
            self.patches = {tid: p for tid, p in self.patches.items() if tid in ids}
        elif msg_type == MsgType.ROI_PATCH:
            x1, y1, x2, y2 = body.box
            if not (0 <= x1 < x2 <= config.FRAME_WIDTH and 0 <= y1 < y2 <= config.FRAME_HEIGHT):
                return BAD
            # A patch is its box padded by PATCH_PAD; one much larger costs time on every frame.
            track = next((t for t in self.tracks if t.id == body.track_id), None)
            if track is not None:
                bx1, by1, bx2, by2 = track.box
                if (x2 - x1 > 2 * (bx2 - bx1) + 2 * PATCH_PAD + 32
                        or y2 - y1 > 2 * (by2 - by1) + 2 * PATCH_PAD + 32):
                    return BAD
            image = decode_image(body.image, size=(x2 - x1, y2 - y1))
            if image is None:
                return BAD
            old = self.patches.get(body.track_id)
            # Only for a track in the newest list (sent just before it), so patches stay bounded.
            if (body.track_id not in {t.id for t in self.tracks}
                    or not self._newer(body.frame, old.msg.frame if old else None, now_s)):
                return OLD
            self.patches[body.track_id] = Patch(body, image)
        elif msg_type == MsgType.BG_KEYFRAME:
            # Checked before decoding: a keyframe also costs an optical-flow pass.
            if not self._newer(body.frame, self.keyframe.msg.frame if self.keyframe else None,
                               now_s):
                return OLD
            image = decode_image(body.image, gray=True, size=KEYFRAME_SIZE)
            if image is None:
                return BAD
            self.keyframe = Keyframe(body, image)
            self.spoof.add_keyframe(body.frame, image, body.alt_m, body.heading_deg,
                                    self.poses.gps_position(body.frame))
            self.poses.fallback = self.spoof.fallback
        elif msg_type == MsgType.FULL_FRAME:
            # Decoded only when shown, but its size is checked now.
            if image_size(body.image) != (config.FRAME_WIDTH, config.FRAME_HEIGHT):
                return BAD
            if not self._newer(body.frame, self.full_frame.frame if self.full_frame else None,
                               now_s):
                return OLD
            self.full_frame = body
        return USED

    def _measure(self, frame, tracks):
        """Feeds a track list to the ghost tracks once the telemetry of its frame is held (the
        two can arrive in either order)."""
        pose = self.poses.telemetry_pose(frame)
        if pose is None:
            newest = self.telemetry.frame if self.telemetry else None
            self.unmeasured = (frame, tracks) if newest is None or frame > newest else None
            return
        self.unmeasured = None
        self.ghosts.add_boxes(frame, tracks, pose)

    def _forget(self):
        """Drops the foveated state of an earlier run of the UAV."""
        self.telemetry = None
        self.telemetry_arrived_s = None
        self.jump = None
        self.poses.clear()
        self.tracks_frame, self.tracks = None, []
        self.keyframe = None
        self.spoof.clear()
        self.patches = {}
        self.ghosts.clear()
        self.unmeasured = None

    def uav_time(self, now_s):
        """The UAV's time now (frame / FPS), from its newest telemetry and the time since that
        arrived, or None before any telemetry. During a blackout it keeps running."""
        if self.telemetry is None:
            return None
        return self.telemetry.frame / config.FPS + max(0.0, now_s - self.telemetry_arrived_s)

    def ghost_estimates(self, now_s, path=True):
        """The tracks that are not lost, predicted to now."""
        t = self.uav_time(now_s)
        if t is None:
            return []
        self.ghosts.prune(t)
        return self.ghosts.estimates(t, path)

    def view(self, now_s=None):
        """A copy of what the foveated view needs, safe to draw from while receiving goes on;
        with `now_s`, also the tracks predicted to then."""
        ghosts = [] if now_s is None else self.ghost_estimates(now_s)
        return View(self.telemetry, self.tracks_frame, list(self.tracks), self.keyframe,
                    dict(self.patches), self.poses.copy(), ghosts)

    def log_tracks_if_due(self, now_s):
        if self.track_log is None or self.telemetry is None:
            return
        if self.next_log_s is None:
            self.next_log_s = now_s
        if now_s < self.next_log_s:
            return
        self.next_log_s += TRACK_LOG_INTERVAL_S
        if self.next_log_s < now_s:
            self.next_log_s = now_s + TRACK_LOG_INTERVAL_S
        silent = None if self.last_fovea_s is None else now_s - self.last_fovea_s
        frame = self.telemetry.frame
        nav = {"frame": frame,
               "gps": [round(float(v), 3) for v in self.poses.gps_position(frame)],
               "used": [round(float(v), 3) for v in self.poses.position(frame)],
               "alarm": self.spoof.alarm}
        self.track_log.write(self.uav_time(now_s), silent, self.poses.origin,
                             self.ghost_estimates(now_s, path=False), nav)

    def stats(self, now_s, estimates=None):
        """The numbers the dashboard shows: rates over the last second, ages, and the ghost
        tracks (from `estimates` when the caller has them for `now_s` already)."""
        rate = {t: self.message_rate[t].rate(now_s) for t in MsgType}
        if estimates is None:
            t = self.uav_time(now_s)
            estimates = [] if t is None else self.ghosts.estimates(t, path=False)
        ghosts = [e for e in estimates if e.state == GHOST]
        areas = [math.pi * a * b for a, b in (semi_axes(e.cov) for e in ghosts)]
        keyframe_age = None
        if self.keyframe is not None and self.telemetry is not None:
            keyframe_age = max(0, self.telemetry.frame - self.keyframe.msg.frame) / config.FPS
        return {
            "fovea_kbps": self.link_rate["fovea"].rate(now_s) * 8 / 1000,
            "baseline_kbps": self.link_rate["baseline"].rate(now_s) * 8 / 1000,
            "telemetry_hz": rate[MsgType.TELEMETRY],
            "tracks_hz": rate[MsgType.TRACKS],
            "patches_per_s": rate[MsgType.ROI_PATCH],
            "baseline_fps": rate[MsgType.FULL_FRAME],
            "keyframe_age_s": keyframe_age,
            "fovea_silent_s": None if self.last_fovea_s is None else now_s - self.last_fovea_s,
            "bad_packets": self.links["fovea"].bad + self.links["baseline"].bad,
            "bad_messages": self.bad_messages,
            "expired": self.links["fovea"].reassembler.stats.expired,
            "filtered_tracks": len(estimates),
            "ghost_tracks": len(ghosts),
            "max_tslf_s": max((e.tslf_s for e in ghosts), default=None),
            "max_ellipse_m2": max(areas, default=None),
            "gps_alarm": self.spoof.alarm,
            "gps_alarms": self.spoof.alarms,
            "spoof_d2": self.spoof.d2,
            "gps_speed": self.spoof.gps_speed(),
            "flow_speed": self.spoof.flow_speed(),
        }

    def start_measuring_if_due(self, now_s, warmup_s):
        """Starts the measurement `warmup_s` after the first foveated datagram."""
        m = self.measurement
        if m.start_s is None and self.first_fovea_s is not None:
            if now_s >= self.first_fovea_s + warmup_s:
                m.start_s = now_s

    def summary(self, now_s):
        """Stops the measurement and returns what arrived, or None if it never started."""
        m = self.measurement
        if m.start_s is None:
            return None
        if m.active:
            m.stop_s = now_s
        result = m.summary()
        fovea = self.links["fovea"]
        result["bad_packets"] = fovea.bad + self.links["baseline"].bad
        result["bad_messages"] = self.bad_messages
        result["expired"] = fovea.reassembler.stats.expired
        result["gps_alarms"] = self.spoof.alarms
        result["undecided_keyframe_pairs"] = self.spoof.undecided
        return result

    def send_report_if_due(self, now_s):
        if self.next_report_s is None:
            self.next_report_s = now_s
        if now_s < self.next_report_s:
            return
        self.next_report_s += REPORT_INTERVAL_S
        if self.next_report_s < now_s:  # the loop was held up; do not send a burst
            self.next_report_s = now_s + REPORT_INTERVAL_S
        now_us = int(now_s * 1e6)
        body = messages.encode_report(self.links["fovea"].report(now_us))
        for datagram in self.report_packetizer.packetize(MsgType.RECEIVER_REPORT, now_us, body):
            self.report_tx.sendto(datagram, self.report_addr)


class ReceiverThread:
    """Runs a GroundStation in its own thread, so that drawing never holds up receiving or the
    receiver reports. The UI reads the station only through `snapshot`, under the same lock."""

    def __init__(self, station, poll_s=0.005, warmup_s=0.0):
        self.station = station
        self.poll_s = poll_s
        self.warmup_s = warmup_s    # the measurement starts this long after the first datagram
        self.lock = threading.Lock()
        self.error = None           # the exception that stopped the thread, if any
        self._stopping = threading.Event()
        self._thread = threading.Thread(target=self._run, name="gcs-receiver", daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stopping.set()
        self._thread.join(timeout=2.0)

    def is_alive(self):
        return self._thread.is_alive()

    def _run(self):
        try:
            while not self._stopping.is_set():
                readable = self.station.wait(self.poll_s)     # without the lock
                with self.lock:
                    self.station.read(readable)
                    now = time.monotonic()
                    self.station.start_measuring_if_due(now, self.warmup_s)
                    self.station.send_report_if_due(now)
                    self.station.log_tracks_if_due(now)
        except Exception as e:      # shown by the UI instead of dying silently
            self.error = e

    def snapshot(self):
        """(View, newest FullFrameMsg or None, stats) as of now."""
        with self.lock:
            s = self.station
            now = time.monotonic()
            view = s.view(now)
            return view, s.full_frame, s.stats(now, view.ghosts)
