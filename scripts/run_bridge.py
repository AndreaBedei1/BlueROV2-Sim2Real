#!/usr/bin/env python
"""Run the lockstep bridge between ArduSub SITL and HoloOcean (or the mock).

Examples:
    python scripts/run_bridge.py --backend holoocean --out results/run1
    python scripts/run_bridge.py --backend mock --out results/mock1

Order of operations: refuse a dirty git tree (unless --allow-dirty), bind the
UDP port (fails before any engine is started if the port is taken), start at
most one HoloOcean instance, write ``ready`` and ``engine.pid``, run. SIGTERM
and SIGHUP are turned into an orderly stop so that the HoloOcean engine is
always closed and ``summary.json`` is written.
"""
import argparse
import hashlib
import json
import platform
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))

from bluerov2_sim2real.backends import HoldSchedule, HoloOceanBackend, MockBackend  # noqa: E402
from bluerov2_sim2real.bridge import Bridge, BridgeConfig, open_bridge_socket  # noqa: E402
from bluerov2_sim2real.thrusters import LogicalVehicle  # noqa: E402


def _sh(*cmd):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, cwd=REPO, timeout=30).stdout.strip()
    except Exception as e:  # noqa: BLE001
        return f"unavailable: {e}"


def _sha256(path: Path):
    try:
        return hashlib.sha256(path.read_bytes()).hexdigest()
    except OSError:
        return None


def git_state():
    diff = _sh("git", "diff", "HEAD")
    return {"commit": _sh("git", "rev-parse", "HEAD"), "branch": _sh("git", "rev-parse", "--abbrev-ref", "HEAD"),
            "status_porcelain": _sh("git", "status", "--porcelain"),   # untracked files count as dirty
            "diff_sha256": hashlib.sha256(diff.encode()).hexdigest(), "diff": diff}


def holoocean_info():
    import holoocean
    info = {"client_version": holoocean.__version__}
    try:
        import importlib.metadata as md
        dist = md.distribution("holoocean")
        du = dist.read_text("direct_url.json")
        info["client_source"] = json.loads(du) if du else None
    except Exception as e:  # noqa: BLE001
        info["client_source"] = f"unavailable: {e}"
    from holoocean import util
    world = Path(util.get_holoocean_path()) / "worlds" / "Ocean"
    info["world_package_dir"] = str(world)
    try:
        info["world_package_config"] = {k: v for k, v in json.loads((world / "config.json").read_text()).items()
                                        if k in ("name", "version")}
    except OSError:
        info["world_package_config"] = None
    info["engine_binary_sha256"] = _sha256(world / "Linux" / "Holodeck" / "Binaries" / "Linux" / "Holodeck")
    return info


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", choices=["holoocean", "mock"], default="holoocean")
    ap.add_argument("--out", required=True)
    ap.add_argument("--rate", type=int, default=200, help="physics rate (Hz) = SITL loop rate")
    ap.add_argument("--instance", type=int, default=7, help="SITL instance (port 9002+10*I)")
    ap.add_argument("--boot-hold", type=float, default=25.0,
                    help="sim seconds of kinematic hold (surface, then descent to --hold-depth)")
    ap.add_argument("--hold-depth", type=float, default=1.0)
    ap.add_argument("--no-drag-compensation", action="store_true")
    ap.add_argument("--horizontal-angle", type=float, default=30.0)
    ap.add_argument("--max-sim", type=float, default=None)
    ap.add_argument("--max-wall", type=float, default=None)
    ap.add_argument("--allow-dirty", action="store_true", help="run with uncommitted changes (recorded)")
    ap.add_argument("--sim-sensors", action="store_true",
                    help="Phase 3: camera (H.264/RTP) + Ping1D + DISTANCE_SENSOR + camera tilt (off by default)")
    ap.add_argument("--ffmpeg", default=None, help="ffmpeg with libx264 for the video stream")
    args = ap.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    git = git_state()
    if git["status_porcelain"] and not args.allow_dirty:
        sys.exit("refusing to run with uncommitted changes (use --allow-dirty to record them anyway)")

    port = 9002 + 10 * args.instance
    try:
        sock = open_bridge_socket("127.0.0.1", port)
    except OSError as e:
        sys.exit(f"UDP 127.0.0.1:{port} not available ({e}); choose another --instance")

    def _terminate(signum, _frame):
        raise KeyboardInterrupt(f"signal {signum}")
    signal.signal(signal.SIGTERM, _terminate)
    signal.signal(signal.SIGHUP, _terminate)

    schedule = HoldSchedule(depth=args.hold_depth) if args.boot_hold > 0 else None
    import numpy
    manifest = {
        "date_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "argv": sys.argv, "backend": args.backend, "rate_hz": args.rate, "bind": f"127.0.0.1:{port}",
        "drag_compensation": args.backend == "holoocean" and not args.no_drag_compensation,
        "horizontal_angle_deg": args.horizontal_angle, "boot_hold_s": args.boot_hold,
        "hold_schedule": schedule.__dict__ if schedule else None,
        "git": {k: v for k, v in git.items() if k != "diff"},
        "python": platform.python_version(), "numpy": numpy.__version__,
        "host": {"platform": platform.platform(), "node_sha256_8": hashlib.sha256(socket.gethostname().encode()).hexdigest()[:8],
                 "gpus": _sh("nvidia-smi", "--query-gpu=name,driver_version", "--format=csv,noheader")},
    }
    if git["diff"]:
        (out / "uncommitted.diff").write_text(git["diff"])
    backend = None
    try:
        if args.backend == "holoocean":
            manifest["holoocean"] = holoocean_info()
            hub, extra = None, ()
            if args.sim_sensors:
                from bluerov2_sim2real.sensors.sim_hub import CAMERA, RANGEFINDER, SimSensorHub
                hub = SimSensorHub(out / "sensors", args.ffmpeg)
                extra = (CAMERA, RANGEFINDER)
                manifest["sim_sensors"] = {"camera": CAMERA, "rangefinder": RANGEFINDER, "video_udp": 5610,
                                           "ping1d_udp": 19090, "distance_sensor_component": 194,
                                           "ffmpeg": _sh(args.ffmpeg, "-version").splitlines()[0] if args.ffmpeg else None,
                                           "ffmpeg_sha256": _sha256(Path(args.ffmpeg)) if args.ffmpeg else None}
            backend = HoloOceanBackend(tps=args.rate, start_location=(0.0, 0.0, 0.0),
                                       drag_compensation=not args.no_drag_compensation, hold=schedule,
                                       sensor_hub=hub, extra_sensors=extra)
            (out / "engine.pid").write_text(f"{backend.engine_pid}\n")
        else:
            backend = MockBackend(tps=args.rate, start_depth=0.0, hold=schedule is not None)
        (out / "bridge_manifest.json").write_text(json.dumps(manifest, indent=2))
        cfg = BridgeConfig(bind_port=port, out_dir=out, max_sim_s=args.max_sim,
                           max_wall_s=args.max_wall, boot_hold_s=args.boot_hold)
        bridge = Bridge(backend, cfg, LogicalVehicle(horizontal_angle_deg=args.horizontal_angle), sock=sock)
        (out / "ready").write_text(f"{port}\n")
        summary = bridge.run()
        print(json.dumps(summary, indent=2, default=float))
    except KeyboardInterrupt:
        print("bridge stopped by signal before the main loop", file=sys.stderr)
    finally:
        signal.signal(signal.SIGTERM, signal.SIG_IGN)   # do not interrupt the engine shutdown
        if backend is not None:
            try:
                if getattr(backend, "sensor_hub", None) is not None:
                    hub_stats = {"frames_in": backend.sensor_hub.frames_in,
                                 "frames_out": backend.sensor_hub.frames_out,
                                 "ping1d_requests": backend.sensor_hub.ping.requests,
                                 "last_tilt_deg": backend.sensor_hub.tilt_deg,
                                 "errors": backend.sensor_hub.errors,
                                 "note": "sensor-on runs are NOT valid for Phase 2 timing metrics "
                                         "(10 Hz camera renders exceed the 5 ms tick budget)"}
                    (out / "sensors" / "hub_stats.json").write_text(json.dumps(hub_stats))
                    backend.sensor_hub.close()
            finally:
                backend.close()          # the engine is always closed, even if the hub fails


if __name__ == "__main__":
    main()
