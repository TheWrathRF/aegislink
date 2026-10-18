"""The ground station: receive the UAV's streams and show them side by side.

    python -m aegislink.gcs.main                       # OpenCV window (q or Esc quits)
    python -m aegislink.gcs.main --no-view --seconds 10 --summary

It receives the foveated stream and the baseline from the proxy (or, with --direct, from the
UAV) and sends receiver reports back 4 times per second. --summary prints, as one JSON line, what
arrived between --warmup seconds after the first foveated datagram and --seconds after that.
"""

import argparse
import json
import sys
import time
from pathlib import Path

import cv2

from aegislink.common import config
from aegislink.gcs import viewer
from aegislink.gcs.receiver import GroundStation

RENDER_INTERVAL_S = 1 / 30
NOTHING_ARRIVES_S = 15.0


def run(station, seconds=None, warmup_s=0.0, view=True, snapshot=None):
    """Receives until `seconds` of measurement are over (or forever), or q / Esc in the window."""
    started = time.monotonic()
    next_render = started
    image = None
    m = station.measurement
    try:
        while True:
            station.poll(0.002)
            now = time.monotonic()
            station.send_report_if_due(now)
            if station.first_fovea_s is None:
                if seconds is not None and now - started > NOTHING_ARRIVES_S:
                    break
            elif m.start_s is None and now >= station.first_fovea_s + warmup_s:
                m.start_s = now
            elif seconds is not None and m.active and now >= m.start_s + seconds:
                m.stop_s = now
                break
            if (view or snapshot) and now >= next_render:
                next_render = max(next_render + RENDER_INTERVAL_S, now)
                image = viewer.compose(station, now)
                if view:
                    cv2.imshow("AegisLink GCS", image)
                    if cv2.waitKey(1) in (ord("q"), 27):
                        break
    finally:
        if m.active:
            m.stop_s = time.monotonic()
        if view:
            cv2.destroyAllWindows()
        if snapshot is not None and image is not None:
            Path(snapshot).parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(snapshot), image)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--no-view", action="store_true", help="no window")
    parser.add_argument("--seconds", type=float, help="measure this long, then quit")
    parser.add_argument("--warmup", type=float, default=0.0,
                        help="seconds after the first foveated datagram before measuring")
    parser.add_argument("--summary", action="store_true", help="print what arrived as JSON")
    parser.add_argument("--snapshot", type=Path, help="save the last view as an image")
    parser.add_argument("--direct", action="store_true", help="report to the UAV, not the proxy")
    parser.add_argument("--fovea-port", type=int, default=config.FOVEA_TO_GCS)
    parser.add_argument("--baseline-port", type=int, default=config.BASELINE_TO_GCS)
    parser.add_argument("--report-port", type=int,
                        help="where receiver reports go (default: the proxy's report link)")
    args = parser.parse_args()
    report_port = args.report_port or (config.REPORTS_TO_UAV if args.direct
                                       else config.REPORTS_TO_PROXY)
    station = GroundStation(args.fovea_port, args.baseline_port, report_port)
    try:
        run(station, args.seconds, args.warmup, not args.no_view, args.snapshot)
    except KeyboardInterrupt:
        pass
    finally:
        station.close()
    m = station.measurement
    if args.summary:
        if m.start_s is None:
            print(json.dumps({"error": "no foveated stream arrived"}))
            sys.exit(1)
        result = m.summary()
        fovea = station.links["fovea"]
        result["bad_packets"] = fovea.bad + station.links["baseline"].bad
        result["bad_messages"] = station.bad_messages
        result["expired"] = fovea.reassembler.stats.expired
        print(json.dumps(result))


if __name__ == "__main__":
    main()
