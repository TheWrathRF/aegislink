"""The ground station: receive the UAV's streams and show them side by side.

    python -m aegislink.gcs.main                       # the dashboard window
    python -m aegislink.gcs.main --no-view --seconds 10 --summary

It receives the foveated stream and the baseline from the proxy (or, with --direct, from the
UAV) and sends receiver reports back 4 times per second. The window's EW deck sets the proxy's
links through its control port. When the window closes, its frame times are printed as one JSON
line. Without a window, --summary prints, as one JSON line, what arrived between --warmup seconds
after the first foveated datagram and --seconds after that. Every track is followed by a Kalman
filter in ground metres (--motion-model); --track-log writes its predictions 30 times a second.
"""

import argparse
import json
import sys
import time
from pathlib import Path

from aegislink.common import config
from aegislink.gcs.ghost_tracks import MODELS
from aegislink.gcs.receiver import GroundStation

NOTHING_ARRIVES_S = 15.0


def run_headless(station, seconds=None, warmup_s=0.0):
    """Receives until `seconds` of measurement are over (or forever)."""
    started = time.monotonic()
    m = station.measurement
    try:
        while True:
            station.poll(0.002)
            now = time.monotonic()
            station.send_report_if_due(now)
            station.log_tracks_if_due(now)
            if station.first_fovea_s is None:
                if seconds is not None and now - started > NOTHING_ARRIVES_S:
                    break
            elif m.start_s is None and now >= station.first_fovea_s + warmup_s:
                m.start_s = now
            elif seconds is not None and m.active and now >= m.start_s + seconds:
                m.stop_s = now
                break
    finally:
        if m.active:
            m.stop_s = time.monotonic()


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--no-view", action="store_true", help="no window")
    parser.add_argument("--seconds", type=float,
                        help="without a window: measure this long; with one: close after this long")
    parser.add_argument("--warmup", type=float, default=0.0,
                        help="seconds after the first foveated datagram before measuring")
    parser.add_argument("--summary", action="store_true",
                        help="print what arrived as JSON (needs --no-view)")
    parser.add_argument("--snapshot", type=Path, help="save the window as an image when it closes")
    parser.add_argument("--direct", action="store_true", help="report to the UAV, not the proxy")
    parser.add_argument("--fovea-port", type=int, default=config.FOVEA_TO_GCS)
    parser.add_argument("--baseline-port", type=int, default=config.BASELINE_TO_GCS)
    parser.add_argument("--report-port", type=int,
                        help="where receiver reports go (default: the proxy's report link)")
    parser.add_argument("--proxy-control", type=int, default=config.PROXY_CONTROL,
                        help="the proxy's control port, for the EW deck")
    parser.add_argument("--motion-model", choices=sorted(MODELS), default="cv",
                        help="the tracks' Kalman filter: constant velocity or acceleration")
    parser.add_argument("--track-log", type=Path,
                        help="write the predicted tracks here (JSON lines, 30 per second)")
    args = parser.parse_args()
    if args.summary and not args.no_view:
        parser.error("--summary needs --no-view")
    if args.snapshot and args.no_view:
        parser.error("--snapshot needs the window")
    report_port = args.report_port or (config.REPORTS_TO_UAV if args.direct
                                       else config.REPORTS_TO_PROXY)
    station = GroundStation(args.fovea_port, args.baseline_port, report_port, args.motion_model,
                            args.track_log)
    try:
        if args.no_view:
            run_headless(station, args.seconds, args.warmup)
        else:
            # Only the window needs Qt; the headless station runs without it.
            from aegislink.gcs import dashboard
            dashboard.run(station, None if args.direct else args.proxy_control, args.seconds,
                          args.snapshot)
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
        result["gps_alarms"] = station.spoof.alarms
        result["undecided_keyframe_pairs"] = station.spoof.undecided
        print(json.dumps(result))


if __name__ == "__main__":
    main()
