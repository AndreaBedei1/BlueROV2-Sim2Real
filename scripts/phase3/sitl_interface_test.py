#!/usr/bin/env python
"""End-to-end interface test against ArduSub 4.1.2 SITL through the Phase 3 router.

Runs inside ``run_stack.sh`` (RUN_DIR set). Everything goes through the common
VehicleClient in the ``simulation`` profile, i.e. exactly the MAVLink messages
QGroundControl or an autonomy program would send. Writes ``interface_test.json``
with one PASS/FAIL entry per check and ``timeline.csv`` with observations.
"""
import csv
import json
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
import bluerov2_sim2real.interface  # noqa: E402,F401  (MAVLink 2 dialect)
from bluerov2_sim2real.interface.client import CommandStatus, VehicleClient  # noqa: E402

RUN = Path(os.environ.get("RUN_DIR", "."))
OUT = RUN / "interface_test"
OUT.mkdir(parents=True, exist_ok=True)
checks = []
tl = open(OUT / "timeline.csv", "w", newline="", buffering=1)
tw = csv.writer(tl)
tw.writerow(["wall", "event", "mode", "armed", "depth_m", "roll", "pitch", "yaw", "yawrate",
             "servo9", "servo13", "servo14", "servo16", "mount_tilt_deg"])


def snap(c, event=""):
    o = c.obs
    att = o.get("attitude_rpy_rad") or (None,) * 3
    rates = o.get("rates_frd_rad_s") or (None,) * 3
    sv = o.get("servo_us") or (None,) * 16
    tw.writerow([f"{time.time():.3f}", event, o.get("mode"), o.get("armed"), o.get("depth_m"), *att, rates[2],
                 sv[8], sv[12], sv[13], sv[15], o.get("mount_tilt_deg")])


def check(name, ok, **detail):
    checks.append({"check": name, "pass": bool(ok), **detail})
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}", flush=True)


def observe(c, seconds, event):
    t_end = time.time() + seconds
    vals = []
    while time.time() < t_end:
        snap(c, event)
        vals.append(dict(c.obs.fields))
        time.sleep(0.1)
    return vals


def series(vals, name, idx=None):
    out = []
    for v in vals:
        x = v.get(name)
        if x is not None and x.value is not None:
            out.append(x.value[idx] if idx is not None else x.value)
    return out


def main():
    c = VehicleClient.simulation()
    ok = c.wait(lambda o: o.get("mode") is not None and o.get("depth_m") is not None
                and o.get("servo_us") is not None, 120)
    check("telemetry_received", ok, mode=c.obs.get("mode"), depth=c.obs.get("depth_m"))
    # wait for the bridge's kinematic boot hold to end (vehicle released at 1 m, at rest)
    c.wait(lambda o: (o.get("depth_m") or 0) > 0.95, 120)
    time.sleep(5)
    observe(c, 2, "initial")
    check("initial_state_defined", abs(c.obs.get("depth_m") - 1.0) < 0.1 and c.obs.get("armed") is False,
          depth=c.obs.get("depth_m"))

    r = c.set_mode("MANUAL")
    check("mode_manual", r.status is CommandStatus.ACCEPTED, result=r.status.value)
    r = c.arm()
    check("arm", r.status is CommandStatus.ACCEPTED and c.obs.get("armed") is True, result=r.status.value)
    c.set_sticks()
    observe(c, 2, "armed_neutral")

    # --- teleoperation axes (MANUAL: pilot input mixed directly by ArduSub) ------------
    def axis(label, sticks, measure):
        d0, a0 = c.obs.get("depth_m"), c.obs.get("attitude_rpy_rad")
        c.set_sticks(**sticks)
        vals = observe(c, 3.0, label)
        c.set_sticks()
        observe(c, 4.0, label + "_release")
        return measure(vals, d0, a0)

    vh = series  # alias
    res = axis("ascend", dict(z=900), lambda v, d0, a0: vh(v, "depth_m")[-1] - d0)
    check("teleop_ascend_depth_decreases", res < -0.05, depth_change_m=round(res, 3))
    res = axis("descend", dict(z=100), lambda v, d0, a0: vh(v, "depth_m")[-1] - d0)
    check("teleop_descend_depth_increases", res > 0.05, depth_change_m=round(res, 3))
    res = axis("yaw_right", dict(r=400), lambda v, d0, a0: max(vh(v, "rates_frd_rad_s", 2)))
    check("teleop_yaw_right_positive_rate", res > 0.2, max_yaw_rate=round(res, 3))
    res = axis("yaw_left", dict(r=-400), lambda v, d0, a0: min(vh(v, "rates_frd_rad_s", 2)))
    check("teleop_yaw_left_negative_rate", res < -0.2, min_yaw_rate=round(res, 3))
    # forward/lateral: no horizontal position/velocity is observable through MAVLink on the
    # vehicle (no DVL/GPS); the autopilot-side evidence is the motor mix in SERVO_OUTPUT_RAW
    def mix(v, d0, a0):
        s = vh(v, "servo_us")[-1]
        return s[0:4]
    m = axis("forward", dict(x=400), mix)
    check("teleop_forward_motor_mix", m[0] < 1500 and m[1] < 1500 and m[2] > 1500 and m[3] > 1500,
          servo1_4=list(m), expected="M1,M2 < 1500 < M3,M4 (forward factors -1,-1,+1,+1)")
    m = axis("lateral_right", dict(y=400), mix)
    check("teleop_lateral_motor_mix", m[0] > 1500 and m[1] < 1500 and m[2] > 1500 and m[3] < 1500,
          servo1_4=list(m), expected="lateral factors +1,-1,+1,-1")

    # --- accessories via joystick buttons -----------------------------------------------
    def servo(i):
        return (c.obs.get("servo_us") or [0] * 16)[i - 1]

    c.gripper("open")
    observe(c, 1.0, "gripper_open")
    check("gripper_open_output9_max", servo(9) == 1900, servo9=servo(9), state=c.obs.get("gripper_cmd"))
    c.gripper("close")
    observe(c, 1.0, "gripper_close")
    check("gripper_close_output9_min", servo(9) == 1100, servo9=servo(9), state=c.obs.get("gripper_cmd"))

    l0 = servo(13)
    for _ in range(4):
        c.lights(1, True)
    observe(c, 1.0, "lights1_up")
    l1 = servo(13)
    for _ in range(2):
        c.lights(1, False)
    observe(c, 1.0, "lights1_down")
    l2 = servo(13)
    check("lights1_brighter_then_dimmer_output13", l1 > l0 and l2 < l1, pwm_sequence=[l0, l1, l2],
          level=c.obs.get("lights1_level"))
    m0 = servo(14)
    for _ in range(3):
        c.lights(2, True)
    observe(c, 1.0, "lights2_up")
    check("lights2_brighter_output14", servo(14) > m0, pwm_sequence=[m0, servo(14)],
          note="shift pairing of lights2 buttons is an assumption (parameter backup pending)")

    t0 = servo(16)
    c.camera_tilt("up", hold_s=2.0)
    observe(c, 1.0, "tilt_up")
    t_up, mt_up = servo(16), c.obs.get("mount_tilt_deg")
    c.camera_tilt("down", hold_s=2.0)
    observe(c, 1.0, "tilt_down")
    t_down, mt_down = servo(16), c.obs.get("mount_tilt_deg")
    c.camera_tilt("center", hold_s=0.3)
    observe(c, 2.0, "tilt_center")
    t_c = servo(16)
    check("camera_tilt_up_down_center_output16",
          t_up < t0 and t_down > t_up and abs(t_c - 1500) <= 10,
          pwm=[t0, t_up, t_down, t_c], mount_status_deg=[mt_up, mt_down, c.obs.get("mount_tilt_deg")],
          decoded_deg=[c.obs.get("camera_tilt_deg_from_servo")],
          note="SERVO16_REVERSED=1: tilt up lowers the PWM")

    # --- mode transitions (D2 transient is a known open FAIL, measured not hidden) -----
    r = c.set_mode("STABILIZE")
    check("mode_stabilize", r.status is CommandStatus.ACCEPTED, result=r.status.value)
    observe(c, 5, "stabilize")
    d_entry = c.obs.get("depth_m")
    r = c.set_mode("ALT_HOLD")
    check("mode_alt_hold", r.status is CommandStatus.ACCEPTED, result=r.status.value)
    vals = observe(c, 25, "alt_hold_entry")
    dev = max(abs(d - d_entry) for d in series(vals, "depth_m"))
    check("D2_alt_hold_entry_transient_le_0.10m", dev <= 0.10, max_depth_deviation_m=round(dev, 3),
          note="known open FAIL from Phase 2 (ArduSub 4.1.2 integrator init); reported, not compensated")
    r = c.set_depth_target(d_entry + 0.5)
    vals = observe(c, 30, "depth_step")
    err = series(vals, "depth_m")[-1] - (d_entry + 0.5)
    check("depth_target_via_client", r.ok and abs(err) < 0.05, final_error_m=round(err, 3))
    r = c.set_mode("MANUAL")
    check("mode_manual_again", r.status is CommandStatus.ACCEPTED)
    r = c.disarm()
    observe(c, 1.0, "disarmed")
    check("disarm", r.status is CommandStatus.ACCEPTED and c.obs.get("armed") is False, result=r.status.value)
    c.close()
    summary = {"checks": checks, "passed": sum(x["pass"] for x in checks), "total": len(checks)}
    (OUT / "interface_test.json").write_text(json.dumps(summary, indent=2, default=str))
    print(f"interface test: {summary['passed']}/{summary['total']} passed")


if __name__ == "__main__":
    main()
