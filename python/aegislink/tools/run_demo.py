"""Runs the whole demo: the chaos proxy, the ground station window and the UAV.

All three stop when the window is closed. The exit code is the window's, or 1 if the UAV
stopped early with an error.

    python -m aegislink.tools.run_demo
    python -m aegislink.tools.run_demo --seconds 30 --snapshot logs/demo.png

The ports are the defaults from `common/config.py` (5600-5605, control 5700), or the same
layout from --base-port.
"""

import argparse
import shutil
import subprocess
import sys
import sysconfig
from pathlib import Path

from aegislink.common import config
from aegislink.common.proxy_control import ProxyControl

# Without --seconds the UAV flies this long; its ground-truth log grows by about 3.9 MB a minute.
UAV_SECONDS = 3600


def find_proxy():
    """The installed ew_chaos_proxy, or None. pip puts it next to the Python scripts, which may
    not be on PATH when no virtual environment is activated."""
    name = "ew_chaos_proxy"
    return shutil.which(name, path=sysconfig.get_path("scripts")) or shutil.which(name)


def ports(base):
    offset = base - config.FOVEA_TO_PROXY
    names = ("FOVEA_TO_PROXY", "FOVEA_TO_GCS", "BASELINE_TO_PROXY", "BASELINE_TO_GCS",
             "REPORTS_TO_PROXY", "REPORTS_TO_UAV", "PROXY_CONTROL")
    return {name.lower(): getattr(config, name) + offset for name in names}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seed", type=int, default=1, help="world and detector seed")
    parser.add_argument("--seconds", type=float, help="close the window after this long")
    parser.add_argument("--snapshot", type=Path, help="save the window as an image when it closes")
    parser.add_argument("--record", type=Path,
                        help="save the window as numbered JPEGs in this folder, 10 per second")
    parser.add_argument("--log", type=Path, help="the UAV's ground-truth log (default: logs/...)")
    parser.add_argument("--video", type=Path,
                        help="fly over this clip with YOLO instead of the synthetic world")
    parser.add_argument("--alt0", type=float, help="altitude in metres at the clip's first frame")
    parser.add_argument("--base-port", type=int, default=config.FOVEA_TO_PROXY,
                        help="first of the six link ports; control is this + 100")
    args = parser.parse_args()
    p = ports(args.base_port)
    proxy_path = find_proxy()
    if proxy_path is None:
        sys.exit("ew_chaos_proxy not found: run 'python -m pip install -e .' first")

    processes = []
    try:
        processes.append(subprocess.Popen(
            [proxy_path, "--link", f"{p['fovea_to_proxy']}:{p['fovea_to_gcs']}",
             "--link", f"{p['baseline_to_proxy']}:{p['baseline_to_gcs']}",
             "--link", f"{p['reports_to_proxy']}:{p['reports_to_uav']}",
             "--control", str(p["proxy_control"]), "--seed", str(args.seed)],
            stdout=subprocess.DEVNULL))
        with ProxyControl(p["proxy_control"]) as control:
            control.wait_ready()
        gcs_args = ["--fovea-port", str(p["fovea_to_gcs"]),
                    "--baseline-port", str(p["baseline_to_gcs"]),
                    "--report-port", str(p["reports_to_proxy"]),
                    "--proxy-control", str(p["proxy_control"])]
        if args.seconds is not None:
            gcs_args += ["--seconds", str(args.seconds)]
        if args.snapshot is not None:
            gcs_args += ["--snapshot", str(args.snapshot)]
        if args.record is not None:
            gcs_args += ["--record", str(args.record)]

        def start_gcs():
            gcs = subprocess.Popen([sys.executable, "-m", "aegislink.gcs.main", *gcs_args])
            processes.append(gcs)
            return gcs

        if args.video is None:
            gcs = start_gcs()
        uav_seconds = UAV_SECONDS if args.seconds is None else args.seconds + 10
        uav_args = [] if args.log is None else ["--log", str(args.log)]
        if args.video is not None:
            uav_args += ["--video", str(args.video)]
            if args.alt0 is not None:
                uav_args += ["--alt0", str(args.alt0)]
        uav = subprocess.Popen(
            [sys.executable, "-m", "aegislink.uav.main", "--no-preview", "--send",
             "--seed", str(args.seed), "--seconds", str(uav_seconds),
             "--fovea-port", str(p["fovea_to_proxy"]),
             "--baseline-port", str(p["baseline_to_proxy"]),
             "--report-port", str(p["reports_to_uav"]), *uav_args],
            stdout=subprocess.DEVNULL if args.video is None else subprocess.PIPE, text=True)
        processes.append(uav)
        if args.video is not None:
            # Measuring the clip takes seconds; the window (and its --seconds and --record)
            # starts when the UAV says it is ready, so it does not open on an empty view.
            for line in uav.stdout:
                if line.startswith("clip:"):
                    print(line.strip(), flush=True)
                    break
            else:
                sys.exit(f"the UAV stopped before the clip was ready (exit code {uav.wait()})")
            gcs = start_gcs()
        gcs.wait()
        return_code = gcs.returncode
        # Without the UAV the window only showed an empty view: that run failed.
        if uav.poll() is not None and uav.returncode != 0:
            print(f"the UAV stopped early (exit code {uav.returncode})", file=sys.stderr)
            return_code = return_code or 1
    except KeyboardInterrupt:
        return_code = 130
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
    sys.exit(return_code)


if __name__ == "__main__":
    main()
