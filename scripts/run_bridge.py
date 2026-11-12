#!/usr/bin/env python
"""Run the lockstep bridge between ArduSub SITL and HoloOcean (or the mock).

Examples:
    python scripts/run_bridge.py --backend holoocean --out results/run1 --boot-hold 20
    python scripts/run_bridge.py --backend mock --out results/mock1
"""
import argparse
import json
import platform
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from bluerov2_sim2real.backends import HoloOceanBackend, MockBackend  # noqa: E402
from bluerov2_sim2real.bridge import Bridge, BridgeConfig  # noqa: E402
from bluerov2_sim2real.thrusters import LogicalVehicle  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", choices=["holoocean", "mock"], default="holoocean")
    ap.add_argument("--out", required=True)
    ap.add_argument("--rate", type=int, default=200, help="physics rate (Hz) = SITL loop rate")
    ap.add_argument("--instance", type=int, default=7, help="SITL instance (port 9002+10*I)")
    ap.add_argument("--boot-hold", type=float, default=20.0,
                    help="sim seconds holding the vehicle at the surface while SITL boots")
    ap.add_argument("--no-drag-compensation", action="store_true")
    ap.add_argument("--horizontal-angle", type=float, default=30.0)
    ap.add_argument("--max-sim", type=float, default=None)
    ap.add_argument("--max-wall", type=float, default=None)
    args = ap.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    hold = args.boot_hold > 0
    if args.backend == "holoocean":
        import holoocean
        backend = HoloOceanBackend(tps=args.rate, start_location=(0.0, 0.0, 0.0),
                                   drag_compensation=not args.no_drag_compensation,
                                   hold_pose=((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)) if hold else None)
        holo_version = holoocean.__version__
    else:
        backend = MockBackend(tps=args.rate, start_depth=0.0, hold=hold)
        holo_version = None
    manifest = {
        "date_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "argv": sys.argv, "backend": args.backend, "holoocean_client": holo_version,
        "python": platform.python_version(), "rate_hz": args.rate,
        "bind": f"127.0.0.1:{9002 + 10 * args.instance}",
        "drag_compensation": args.backend == "holoocean" and not args.no_drag_compensation,
        "horizontal_angle_deg": args.horizontal_angle, "boot_hold_s": args.boot_hold,
    }
    (out / "bridge_manifest.json").write_text(json.dumps(manifest, indent=2))
    cfg = BridgeConfig(bind_port=9002 + 10 * args.instance, out_dir=out, max_sim_s=args.max_sim,
                       max_wall_s=args.max_wall, boot_hold_s=args.boot_hold)
    try:
        summary = Bridge(backend, cfg, LogicalVehicle(horizontal_angle_deg=args.horizontal_angle)).run()
    finally:
        backend.close()
    print(json.dumps(summary, indent=2, default=float))


if __name__ == "__main__":
    main()
