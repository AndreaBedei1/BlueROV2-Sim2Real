#!/usr/bin/env python
"""One autonomy implementation, two interchangeable environments.

A small depth-survey behaviour: check the vehicle state, arm, hold depth with
ArduSub's own ALT_HOLD controller, visit a list of depths, look down with the
camera, switch the lights on, then return to MANUAL and disarm. The logic uses
only ``VehicleClient`` and never inspects which environment it runs in: the
profile is chosen on the command line, outside the logic.

    python examples/depth_survey_autonomy.py --profile simulation
    python examples/depth_survey_autonomy.py --profile real_readonly --listen 0.0.0.0:14550
    python examples/depth_survey_autonomy.py --profile replay --tlog run/router/traffic.tlog --sysid 17

In ``real_readonly`` and ``replay`` every command is rejected locally (nothing
is transmitted); the behaviour still runs, observes the vehicle and records
which decisions it would have taken.
"""
import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
import bluerov2_sim2real.interface  # noqa: E402,F401
from bluerov2_sim2real.interface.client import VehicleClient  # noqa: E402

DEPTHS_M = [1.5, 2.5, 1.5]
TOLERANCE_M = 0.10
STEP_TIMEOUT_S = 40.0


def run(vehicle: VehicleClient, log) -> dict:
    """The autonomy logic. Identical for every environment."""
    if not vehicle.wait(lambda o: o.get("mode") is not None and o.get("depth_m") is not None, 60):
        log("abort: no vehicle telemetry")
        return {"completed": False, "reason": "no telemetry"}
    log(f"vehicle: mode={vehicle.obs.get('mode')} armed={vehicle.obs.get('armed')} "
        f"depth={vehicle.obs.get('depth_m')} m link_ok={vehicle.link_ok()}")
    decisions = []

    def act(result):
        decisions.append({"command": result.command, "status": result.status.value, "detail": result.detail})
        log(f"{result.command}: {result.status.value} {result.detail}")
        return result.ok

    act(vehicle.set_mode("MANUAL"))
    act(vehicle.arm())
    hold_ok = act(vehicle.set_mode("ALT_HOLD"))
    if hold_ok:
        # Explicit operational step, NOT a fix of the known D2 failure: ArduSub 4.1.2 initialises its
        # vertical integrator wrongly when ALT_HOLD is entered from MANUAL (Phase 2, D2). The behaviour
        # waits until the vertical speed has settled before commanding depths.
        def settled(o):
            return o.get("climb_rate_mps") is not None and abs(o.get("climb_rate_mps")) < 0.02
        t_wait = time.monotonic()
        vehicle.wait(lambda o: not settled(o), 5)                      # the transient starts
        ok = vehicle.wait(settled, 60) and (time.sleep(10) or settled(vehicle.obs))
        log(f"ALT_HOLD entry settled={ok} after {time.monotonic() - t_wait:.1f} s, "
            f"depth={vehicle.obs.get('depth_m')} m")
    reached = []
    for target in DEPTHS_M:
        sent = hold_ok and act(vehicle.set_depth_target(target))
        if not sent:
            decisions.append({"command": f"wait_depth_{target}", "status": "skipped", "detail": "target not sent"})
            continue
        def near(o):
            d = o.get("depth_m", max_age_s=1.0)              # decide on fresh data only
            return d is not None and abs(d - target) < TOLERANCE_M
        ok = vehicle.wait(near, STEP_TIMEOUT_S)
        time.sleep(15.0)                                  # within tolerance after settling (Phase 2: <= 12 s)
        ok = ok and near(vehicle.obs)
        reached.append({"target_m": target, "reached": ok, "depth_m": vehicle.obs.get("depth_m")})
        log(f"depth target {target} m reached={ok} depth={vehicle.obs.get('depth_m')} link_ok={vehicle.link_ok()}")
    act(vehicle.camera_tilt("down", hold_s=1.5))
    act(vehicle.lights(1, True))
    time.sleep(1.0)
    log(f"camera tilt (MOUNT_STATUS) = {vehicle.obs.get('mount_tilt_deg')} deg, "
        f"lights1 level = {vehicle.obs.get('lights1_level')}")
    act(vehicle.camera_tilt("center", hold_s=0.3))
    act(vehicle.set_mode("MANUAL"))
    act(vehicle.disarm())
    return {"completed": True, "decisions": decisions, "depth_steps": reached,
            "observed_final": {k: vehicle.obs.get(k) for k in
                               ("mode", "armed", "depth_m", "mount_tilt_deg", "lights1_level", "gripper_cmd")}}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--profile", choices=["simulation", "real_readonly", "replay"], default="simulation")
    ap.add_argument("--listen", help="real_readonly: host:port where the vehicle stream arrives")
    ap.add_argument("--tlog", help="replay: recorded tlog")
    ap.add_argument("--sysid", type=int, default=None)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    if a.profile == "simulation":
        vehicle = VehicleClient.simulation()
    elif a.profile == "real_readonly":
        if not a.listen:
            sys.exit("real_readonly needs an explicit --listen host:port (never selected automatically)")
        host, port = a.listen.rsplit(":", 1)
        vehicle = VehicleClient.real_readonly((host, int(port)), autopilot_sysid=a.sysid or 1)
    else:
        vehicle = VehicleClient.replay(Path(a.tlog), autopilot_sysid=a.sysid or 1, realtime=True)
    t0 = time.time()

    def log(msg):
        print(f"[autonomy {a.profile} {time.time() - t0:7.1f}s] {msg}", flush=True)

    try:
        result = run(vehicle, log)
    finally:
        vehicle.close()
    result.update(profile=a.profile, rejected_locally=vehicle.rejected_locally)
    if a.out:
        Path(a.out).parent.mkdir(parents=True, exist_ok=True)
        Path(a.out).write_text(json.dumps(result, indent=2, default=str))
    print(json.dumps({k: result[k] for k in ("profile", "completed", "rejected_locally")}))


if __name__ == "__main__":
    main()
