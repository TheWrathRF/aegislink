"""Scores the ground station's tracks against the UAV's ground truth.

    python -m aegislink.tools.evaluate --truth logs/ground_truth.jsonl --tracks logs/tracks.jsonl

The track log (`gcs.main --track-log`) holds each track's predicted position and covariance 30
times a second. Each track is matched to the true vehicle nearest to its filtered position at its
last fix; then every sample is checked against that vehicle's true position at the same moment:
it lies inside the 95 % ellipse when its squared Mahalanobis distance is at most 5.991.

Samples taken while the foveated link was silent (a blackout) and the track was a GHOST show
whether the ellipse can be trusted when it matters: a consistent filter holds the truth in about
95 % of them. Samples with the link up are counted apart.
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np

from aegislink.common import geo
from aegislink.gcs.ghost_tracks import CHI2_95, GHOST, GHOST_AFTER_S

# A track is matched to a vehicle only if that vehicle was this close to its filtered position at
# the track's last fix.
MATCH_M = 3.0


def read_jsonl(path):
    with Path(path).open(encoding="utf-8") as f:
        return [json.loads(line) for line in f if line.strip()]


class Truth:
    """Every vehicle's position at every frame of a ground-truth log, in the map frame."""

    def __init__(self, lines):
        header = lines[0]
        self.fps = header["fps"]
        self.origin = tuple(header["origin_latlon"])
        frames = lines[1:]
        self.first = frames[0]["frame"]
        self.traffic = np.array([f["traffic"] for f in frames], float)  # frames x vehicles x 2

    @classmethod
    def read(cls, path):
        return cls(read_jsonl(path))

    def at(self, t):
        """(vehicles, 2) positions at time `t` (seconds, frame / fps), interpolated, or None
        outside the log."""
        x = t * self.fps - self.first
        if not 0 <= x <= len(self.traffic) - 1 or len(self.traffic) < 2:
            return None
        i = min(math.floor(x), len(self.traffic) - 2)
        w = x - i
        return self.traffic[i] * (1 - w) + self.traffic[i + 1] * w

    def offset(self, origin):
        """Where `origin` (lat, lon) lies in the map frame: added to a position measured from
        `origin`, it gives the map position."""
        east, north = geo.latlon_to_enu(origin[0], origin[1], self.origin)
        return np.array([float(east), float(north)])


def _share(values):
    return round(float(np.mean(values)), 4) if values else None


def ghost_consistency(truth, samples):
    """How often the true position lies inside the 95 % ellipse: during blackouts (link silent,
    track GHOST), overall and per second since the last fix, and with the link up."""
    match = {}                  # (track id, fix time) -> vehicle index or None
    blackout, link_up = [], []
    by_second = {}              # whole seconds since the last fix -> [inside, ...]
    blackouts = []              # (start t, end t) of each silent stretch
    unmatched = 0
    silent_since = None
    for sample in samples:
        silent = sample["silent"] is not None and sample["silent"] >= GHOST_AFTER_S
        if silent and silent_since is None:
            silent_since = sample["t"]
        elif not silent and silent_since is not None:
            blackouts.append((silent_since, sample["t"]))
            silent_since = None
        if sample["origin"] is None:
            continue
        offset = truth.offset(sample["origin"])
        positions = truth.at(sample["t"])
        if positions is None:
            continue
        for track in sample["tracks"]:
            key = (track["id"], track["fix"][0])
            if key not in match:
                match[key] = _match(truth, track["fix"], offset)
            vehicle = match[key]
            if vehicle is None:
                unmatched += 1
                continue
            xx, xy, yy = track["cov"]
            cov = np.array([[xx, xy], [xy, yy]])
            error = positions[vehicle] - (np.array(track["mean"]) + offset)
            try:
                inside = bool(error @ np.linalg.solve(cov, error) <= CHI2_95)
            except np.linalg.LinAlgError:
                inside = False
            if silent and track["state"] == GHOST:
                blackout.append(inside)
                by_second.setdefault(int(track["tslf"]), []).append(inside)
            elif not silent:
                link_up.append(inside)
    if silent_since is not None:
        blackouts.append((silent_since, samples[-1]["t"]))
    return {
        "blackouts": len(blackouts),
        "blackout_s": [round(end - start, 2) for start, end in blackouts],
        "blackout_samples": len(blackout),
        "blackout_inside": _share(blackout),
        "blackout_inside_by_tslf_s": {str(k): _share(v) for k, v in sorted(by_second.items())},
        "link_up_samples": len(link_up),
        "link_up_inside": _share(link_up),
        "tracks_matched": len({k[0] for k, v in match.items() if v is not None}),
        "unmatched_samples": unmatched,
    }


def _match(truth, fix, offset):
    """The vehicle nearest to a track's filtered position at its last fix, or None if none is
    within MATCH_M."""
    t, east, north = fix
    positions = truth.at(t)
    if positions is None:
        return None
    distance = np.linalg.norm(positions - (np.array([east, north]) + offset), axis=1)
    nearest = int(np.argmin(distance))
    return nearest if distance[nearest] <= MATCH_M else None


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--truth", type=Path, required=True, help="the UAV's ground-truth log")
    parser.add_argument("--tracks", type=Path, required=True, help="the GCS's track log")
    args = parser.parse_args()
    result = ghost_consistency(Truth.read(args.truth), read_jsonl(args.tracks))
    print(json.dumps(result))


if __name__ == "__main__":
    main()
