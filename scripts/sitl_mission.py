#!/usr/bin/env python
"""MAVLink scenarios for the Phase 2 tests, against ArduSub SITL only.

    python scripts/sitl_mission.py axes  --out results/runC   # Test C (motor test + pilot axes)
    python scripts/sitl_mission.py depth --out results/runE   # Tests D and E (stability, depth control)
    python scripts/sitl_mission.py soak  --out results/runF --minutes 30   # Test F (long run)

Safety: the connection string is fixed to 127.0.0.1 and the script checks that
the autopilot reports the SITL system id (17) before sending any command.

The script is single-threaded: it reads MAVLink, sends MANUAL_CONTROL at
10 Hz (ArduSub disarms after FS_PILOT_TIMEOUT without pilot input) and a GCS
heartbeat at 1 Hz. Time references use the autopilot ``time_boot_ms`` (SITL
simulated time), never the wall clock.
"""
import argparse
import csv
import json
import time
from pathlib import Path

from pymavlink import mavutil

SITL_SYSID = 17
MODES = {"STABILIZE": 0, "ALT_HOLD": 2, "MANUAL": 19}
LOG_TYPES = ["HEARTBEAT", "ATTITUDE", "VFR_HUD", "GLOBAL_POSITION_INT", "SERVO_OUTPUT_RAW",
             "SCALED_PRESSURE", "SCALED_PRESSURE2", "SYS_STATUS", "STATUSTEXT", "SYSTEM_TIME",
             "COMMAND_ACK", "RAW_IMU", "SCALED_IMU2", "LOCAL_POSITION_NED", "EKF_STATUS_REPORT"]
MOTOR_TEST_ORDER_BOARD = 2      # MAVLink MOTOR_TEST_ORDER: 0 default, 1 sequence, 2 board
MOTOR_TEST_THROTTLE_PWM = 1


class Mission:
    def __init__(self, instance: int, out: Path):
        port = 5760 + 10 * instance
        self.out = out
        out.mkdir(parents=True, exist_ok=True)
        self.m = mavutil.mavlink_connection(f"tcp:127.0.0.1:{port}", source_system=255,
                                            source_component=190, autoreconnect=False)
        self.sticks = dict(x=0, y=0, z=500, r=0)
        self.t_boot_ms = 0
        self.armed = False
        self.mode = None
        self._t_mc = 0.0
        self._t_hb = 0.0
        self._log = open(out / "mavlink.jsonl", "w", buffering=1)
        self._marks = open(out / "marks.csv", "w", newline="", buffering=1)
        self._mw = csv.writer(self._marks)
        self._mw.writerow(["time_boot_ms", "event", "value"])

    # -- plumbing -----------------------------------------------------------
    def mark(self, event, value=""):
        self._mw.writerow([self.t_boot_ms, event, value])
        print(f"[mission] t={self.t_boot_ms / 1e3:8.2f}s {event} {value}", flush=True)

    def _periodic(self):
        now = time.monotonic()
        if now - self._t_mc >= 0.1:
            self._t_mc = now
            s = self.sticks
            self.m.mav.manual_control_send(SITL_SYSID, s["x"], s["y"], s["z"], s["r"], 0)
        if now - self._t_hb >= 1.0:
            self._t_hb = now
            self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def spin(self, seconds_sim=None, until=None, timeout_wall=600.0):
        """Process messages until `seconds_sim` of autopilot time elapsed or until(msg) is true."""
        t_end = None if seconds_sim is None else self.t_boot_ms + seconds_sim * 1e3
        w0 = time.monotonic()
        while True:
            self._periodic()
            msg = self.m.recv_match(blocking=True, timeout=0.05)
            if msg is not None and msg.get_srcSystem() == SITL_SYSID:
                t = msg.get_type()
                if hasattr(msg, "time_boot_ms"):
                    self.t_boot_ms = msg.time_boot_ms
                if t == "HEARTBEAT" and msg.get_srcComponent() == 1:
                    self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.mode = msg.custom_mode
                if t in LOG_TYPES:
                    d = msg.to_dict()
                    d["_t_boot_ms"] = self.t_boot_ms
                    self._log.write(json.dumps(d) + "\n")
                if t == "STATUSTEXT":
                    print(f"[sitl] {msg.text}", flush=True)
                if until is not None and until(msg):
                    return msg
            if t_end is not None and self.t_boot_ms >= t_end:
                return None
            if time.monotonic() - w0 > timeout_wall:
                raise TimeoutError("no progress from SITL")

    # -- commands -------------------------------------------------------------
    def connect(self):
        hb = self.m.wait_heartbeat(timeout=120)
        if hb is None or hb.get_srcSystem() != SITL_SYSID:
            raise RuntimeError(f"unexpected autopilot (sysid {hb and hb.get_srcSystem()}), refusing to command")
        self.mark("connected", f"sysid={hb.get_srcSystem()}")
        self.m.mav.request_data_stream_send(SITL_SYSID, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
        self.spin(1.0)

    def set_mode(self, name):
        self.m.mav.set_mode_send(SITL_SYSID, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, MODES[name])
        self.spin(until=lambda msg: msg.get_type() == "HEARTBEAT" and msg.custom_mode == MODES[name], timeout_wall=30)
        self.mark("mode", name)

    def arm(self, arm=True):
        for _ in range(10):
            self.m.mav.command_long_send(SITL_SYSID, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                         1 if arm else 0, 0, 0, 0, 0, 0, 0)
            self.spin(1.0)
            if self.armed == arm:
                self.mark("armed" if arm else "disarmed")
                return
        raise RuntimeError("arming state change refused (see STATUSTEXT in mavlink.jsonl)")

    def wait_boot_time(self, t_s):
        self.mark("wait_until", t_s)
        self.spin(until=lambda msg: self.t_boot_ms >= t_s * 1e3, timeout_wall=600)

    def depth_target(self, depth_m):
        mask = 0b110111111000   # use position only
        self.m.mav.set_position_target_global_int_send(
            0, SITL_SYSID, 1, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, mask,
            0, 0, -float(depth_m), 0, 0, 0, 0, 0, 0, 0, 0)

    def hold_depth(self, depth_m, seconds):
        self.mark("depth_target", depth_m)
        t_end = self.t_boot_ms + seconds * 1e3
        while self.t_boot_ms < t_end:
            self.depth_target(depth_m)
            self.spin(1.0)

    def motor_test(self, motor, pwm, seconds):
        self.mark("motor_test_start", f"{motor}:{pwm}")
        t_end = self.t_boot_ms + seconds * 1e3
        while self.t_boot_ms < t_end:
            # ArduSub 4.1.2: param1 is the 0-based output index (AP_MotorsMatrix::output_test_num),
            # verified in SITL: sending m drove output m+1.
            self.m.mav.command_long_send(SITL_SYSID, 1, mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 0,
                                         motor - 1, MOTOR_TEST_THROTTLE_PWM, pwm, 1, 0, MOTOR_TEST_ORDER_BOARD, 0)
            self.spin(0.2)
        self.mark("motor_test_end", motor)

    def pilot(self, axis, value, seconds, settle=4.0):
        self.mark("pilot_start", f"{axis}:{value}")
        self.sticks[axis] = value
        self.spin(seconds)
        self.sticks = dict(x=0, y=0, z=500, r=0)
        self.mark("pilot_end", axis)
        self.spin(settle)


def scenario_axes(ms: Mission, a):
    ms.wait_boot_time(a.start_after)
    ms.set_mode("MANUAL")
    for motor in range(1, 9):
        ms.arm(True)
        ms.spin(2.0)
        ms.motor_test(motor, 1600, 1.5)
        ms.spin(11.0)          # motor test times out -> disarm; 10 s cooldown before the next one
    ms.arm(True)
    for axis, val in [("x", 400), ("x", -400), ("y", 400), ("y", -400), ("z", 900), ("z", 100), ("r", 400), ("r", -400)]:
        ms.pilot(axis, val, 3.0)
    ms.arm(False)


def scenario_depth(ms: Mission, a):
    ms.wait_boot_time(a.start_after)
    ms.set_mode("MANUAL")
    ms.arm(True)
    ms.mark("test_D_start")
    ms.spin(a.neutral_s)                       # Test D: armed, neutral sticks, no controller action
    ms.mark("test_D_end")
    ms.set_mode("ALT_HOLD")
    ms.mark("test_E_start")
    ms.spin(10.0)
    for d in a.depths:
        ms.hold_depth(d, a.hold_s)
    ms.mark("test_E_end")
    ms.arm(False)


def scenario_soak(ms: Mission, a):
    ms.wait_boot_time(a.start_after)
    ms.set_mode("ALT_HOLD")
    ms.arm(True)
    ms.mark("test_F_start")
    t_end = ms.t_boot_ms + a.minutes * 60e3
    cycle = [1.0, 2.0, 1.5, 2.5]
    i = 0
    while ms.t_boot_ms < t_end:
        ms.hold_depth(cycle[i % len(cycle)], min(60.0, (t_end - ms.t_boot_ms) / 1e3))
        i += 1
    ms.mark("test_F_end")
    ms.arm(False)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scenario", choices=["axes", "depth", "soak"])
    ap.add_argument("--out", required=True)
    ap.add_argument("--instance", type=int, default=7)
    ap.add_argument("--start-after", type=float, default=40.0, help="autopilot seconds before starting")
    ap.add_argument("--neutral-s", type=float, default=30.0)
    ap.add_argument("--hold-s", type=float, default=45.0)
    ap.add_argument("--depths", type=float, nargs="+", default=[1.0, 2.0, 1.5, 3.0, 1.0])
    ap.add_argument("--minutes", type=float, default=30.0)
    a = ap.parse_args()
    ms = Mission(a.instance, Path(a.out))
    ms.connect()
    {"axes": scenario_axes, "depth": scenario_depth, "soak": scenario_soak}[a.scenario](ms, a)
    ms.mark("done")


if __name__ == "__main__":
    main()
