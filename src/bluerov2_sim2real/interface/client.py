"""Common vehicle client: one API for the simulated and the physical BlueROV2.

The autonomy code talks only to :class:`VehicleClient`. The *profile* chooses
the transport; commands and observations keep the meaning fixed in
``contract.py``. ArduSub (SITL or vehicle) remains the controller: the client
only sends the MAVLink messages a ground station or companion would send.

Profiles
--------
``simulation``     UDP listen 127.0.0.1:14570, where the Phase 3 router pushes the
                   SITL stream (like BlueOS' GCS link); replies go back to the
                   router. Only autopilot system id 17 (the SITL) is accepted.
``real_readonly``  passive listener for a vehicle stream (e.g. BlueOS GCS link to
                   the topside UDP port). It never transmits: every command is
                   rejected locally and the transport has no write path. Must be
                   requested explicitly with an explicit listen address.
``replay``         reads a recorded .tlog; read-only by construction.

Commands return a :class:`CommandResult` (never raise for a refused or
read-only command), so the autonomy logic is identical in every profile.
"""
from __future__ import annotations

import socket
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Callable, Dict, Optional, Tuple

from pymavlink import mavutil

from . import contract as C
from .tlog import read_tlog

mavlink = mavutil.mavlink


class ReadOnlyViolation(RuntimeError):
    """Raised by a read-only transport if anything tries to write to it."""


class CommandStatus(str, Enum):
    ACCEPTED = "accepted"            # autopilot acknowledged / state confirmed
    SENT = "sent"                    # message without acknowledgement in MAVLink (e.g. MANUAL_CONTROL)
    DENIED = "denied"                # autopilot refused (COMMAND_ACK result != ACCEPTED)
    TIMEOUT = "timeout"              # no acknowledgement / confirmation in time
    REJECTED_LOCALLY = "rejected_locally"   # read-only profile or invalid request: nothing transmitted


@dataclass
class CommandResult:
    command: str
    status: CommandStatus
    detail: str = ""

    @property
    def ok(self) -> bool:
        return self.status in (CommandStatus.ACCEPTED, CommandStatus.SENT)


# --- transports ------------------------------------------------------------------

class UdpListenTransport:
    """Bind a local UDP port, receive MAVLink, reply to the last sender (if writable)."""

    def __init__(self, host: str, port: int, writable: bool):
        self.writable = writable
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.sock.settimeout(0.1)
        self.peer: Optional[Tuple[str, int]] = None

    def recv(self) -> bytes:
        try:
            data, addr = self.sock.recvfrom(65536)
        except socket.timeout:
            return b""
        self.peer = addr
        return data

    def send(self, data: bytes):
        if not self.writable:
            raise ReadOnlyViolation("read-only transport: transmission blocked before the network")
        if self.peer is None:
            raise ConnectionError("no autopilot stream received yet")
        self.sock.sendto(data, self.peer)

    def close(self):
        self.sock.close()


class TlogReplayTransport:
    """Feed a recorded tlog as if it arrived live (optionally paced). Never writes."""

    writable = False

    def __init__(self, path: Path, realtime: bool = False):
        self.msgs = list(read_tlog(path))
        self.i = 0
        self.realtime = realtime
        self.t0_wall = time.monotonic()
        self.t0_log = self.msgs[0][0] if self.msgs else 0.0

    def recv(self) -> bytes:
        if self.i >= len(self.msgs):
            time.sleep(0.05)
            return b""
        t, m = self.msgs[self.i]
        if self.realtime:
            wait = (t - self.t0_log) - (time.monotonic() - self.t0_wall)
            if wait > 0:
                time.sleep(min(wait, 0.1))
                return b""
        self.i += 1
        return bytes(m.get_msgbuf())

    @property
    def exhausted(self) -> bool:
        return self.i >= len(self.msgs)

    def send(self, data: bytes):
        raise ReadOnlyViolation("replay transport is read-only")

    def close(self):
        pass


class _Writer:
    def __init__(self, transport):
        self.t = transport

    def write(self, data: bytes):
        self.t.send(bytes(data))


# --- observations ------------------------------------------------------------------

@dataclass
class Value:
    value: object
    t_boot_ms: Optional[int]     # autopilot time of the source message (None if it has none)
    t_rx: float                  # client monotonic receive time


@dataclass
class Observation:
    """Latest vehicle state; every field is None until observed (never invented)."""
    fields: Dict[str, Value] = field(default_factory=dict)

    def get(self, name: str, max_age_s: Optional[float] = None):
        v = self.fields.get(name)
        if v is None:
            return None
        if max_age_s is not None and time.monotonic() - v.t_rx > max_age_s:
            return None
        return v.value

    def age_s(self, name: str) -> Optional[float]:
        v = self.fields.get(name)
        return None if v is None else time.monotonic() - v.t_rx


# --- client --------------------------------------------------------------------------

class VehicleClient:
    def __init__(self, profile: str, transport, autopilot_sysid: int, source_component: int = 191):
        if profile not in ("simulation", "real_readonly", "replay"):
            raise ValueError(profile)
        self.profile = profile
        self.transport = transport
        self.sysid = autopilot_sysid
        self.read_only = profile != "simulation"
        if self.read_only and getattr(transport, "writable", False):
            raise ValueError("read-only profile requires a non-writable transport")
        self.mav_in = mavlink.MAVLink(None)
        self.mav_in.robust_parsing = True
        self.mav_out = mavlink.MAVLink(_Writer(transport), srcSystem=C_GCS_SYSID, srcComponent=source_component)
        self.obs = Observation()
        self.rejected_locally = 0
        self.foreign_messages = 0
        self._lock = threading.RLock()
        self._listeners = []
        self._sticks = dict(C.MANUAL_CONTROL_NEUTRAL)
        self._buttons = 0
        self._pilot_thread = None
        self._stop = threading.Event()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()
        if not self.read_only:
            # Like QGroundControl with a joystick: GCS HEARTBEAT at 1 Hz and MANUAL_CONTROL at 20 Hz
            # (neutral unless commanded) for the whole session. ArduSub 4.1.2 otherwise falls back to
            # RC inputs and, with the vehicle's FS_GCS_ENABLE=2 / FS_PILOT_INPUT=2, to its failsafes.
            self._ensure_pilot_stream()

    # -- constructors --------------------------------------------------------------
    @classmethod
    def simulation(cls, port: int = 14570):
        return cls("simulation", UdpListenTransport("127.0.0.1", port, writable=True), autopilot_sysid=17)

    @classmethod
    def real_readonly(cls, listen: Tuple[str, int], autopilot_sysid: int = 1):
        """Explicit, passive listener for the physical vehicle. Never transmits."""
        return cls("real_readonly", UdpListenTransport(listen[0], listen[1], writable=False),
                   autopilot_sysid=autopilot_sysid)

    @classmethod
    def replay(cls, tlog: Path, autopilot_sysid: int, realtime: bool = False):
        return cls("replay", TlogReplayTransport(Path(tlog), realtime), autopilot_sysid=autopilot_sysid)

    def close(self):
        self._stop.set()
        self._rx_thread.join(1.0)
        if self._pilot_thread is not None:
            self._pilot_thread.join(1.0)
        self.transport.close()

    # -- receive ---------------------------------------------------------------------
    def _rx_loop(self):
        while not self._stop.is_set():
            try:
                data = self.transport.recv()
            except OSError:
                if self._stop.is_set():
                    return          # socket closed by close()
                raise
            if not data:
                continue
            for m in self.mav_in.parse_buffer(data) or []:
                self._handle(m)

    def _set(self, name, value, tb):
        self.obs.fields[name] = Value(value, tb, time.monotonic())

    def _handle(self, m):
        t = m.get_type()
        if t == "BAD_DATA":
            return
        src, comp = m.get_srcSystem(), m.get_srcComponent()
        if src != self.sysid:
            self.foreign_messages += 1        # e.g. other GCS heartbeats: ignored, counted
            return
        tb = getattr(m, "time_boot_ms", None)
        with self._lock:
            if t == "HEARTBEAT" and comp == 1:
                self._set("armed", bool(m.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED), None)
                self._set("mode", C.MODE_NAMES.get(m.custom_mode, f"UNKNOWN_{m.custom_mode}"), None)
                self._set("system_status", m.system_status, None)
            elif t == "ATTITUDE":
                self._set("attitude_rpy_rad", (m.roll, m.pitch, m.yaw), tb)
                self._set("rates_frd_rad_s", (m.rollspeed, m.pitchspeed, m.yawspeed), tb)
            elif t == "GLOBAL_POSITION_INT":
                self._set("depth_m", -m.relative_alt / 1000.0, tb)
                self._set("climb_rate_mps", -m.vz / 100.0, tb)
            elif t == "SCALED_PRESSURE2":
                self._set("pressure_hpa", m.press_abs, tb)
                self._set("water_temp_c", m.temperature / 100.0, tb)
            elif t == "SYS_STATUS":
                self._set("battery_v", m.voltage_battery / 1000.0, None)
            elif t == "SERVO_OUTPUT_RAW":
                servos = tuple(getattr(m, f"servo{i}_raw", 0) or 0 for i in range(1, 17))
                tbs = m.time_usec // 1000
                self._set("servo_us", servos, tbs)
                self._set("gripper_cmd", C.decode_gripper(servos[C.ACCESSORY_OUTPUTS["gripper"] - 1]), tbs)
                self._set("lights1_level", C.decode_level(servos[C.ACCESSORY_OUTPUTS["lights1"] - 1]), tbs)
                self._set("lights2_level", C.decode_level(servos[C.ACCESSORY_OUTPUTS["lights2"] - 1]), tbs)
                self._set("camera_tilt_deg_from_servo",
                          C.decode_tilt_deg(servos[C.ACCESSORY_OUTPUTS["camera_tilt"] - 1]), tbs)
            elif t == "MOUNT_STATUS":
                self._set("mount_tilt_deg", m.pointing_a / 100.0, None)
            elif t == "DISTANCE_SENSOR" and comp == 194:      # Ping1D service component only
                valid = m.min_distance <= m.current_distance <= m.max_distance and \
                    getattr(m, "signal_quality", 0) != 1
                self._set("rangefinder_m", m.current_distance / 100.0 if valid else None, tb)
                self._set("rangefinder_quality", getattr(m, "signal_quality", None), tb)
            elif t == "STATUSTEXT":
                self._set("last_statustext", m.text, None)
            for cb in list(self._listeners):
                cb(m)

    def wait(self, predicate: Callable[[Observation], bool], timeout_s: float) -> bool:
        t_end = time.monotonic() + timeout_s
        while time.monotonic() < t_end:
            if predicate(self.obs):
                return True
            time.sleep(0.02)
        return False

    # -- transmit ----------------------------------------------------------------------
    def _reject(self, name, why="read-only profile: nothing transmitted") -> CommandResult:
        self.rejected_locally += 1
        return CommandResult(name, CommandStatus.REJECTED_LOCALLY, why)

    def _send(self, fn, *args):
        with self._lock:
            fn(*args)

    def _command_long(self, name, cmd, params, timeout_s=3.0) -> CommandResult:
        if self.read_only:
            return self._reject(name)
        ack = {}

        def cb(m):
            if m.get_type() == "COMMAND_ACK" and m.command == cmd:
                ack["r"] = m.result
        self._listeners.append(cb)
        try:
            p = list(params) + [0] * (7 - len(params))
            self._send(self.mav_out.command_long_send, self.sysid, 1, cmd, 0, *p)
            t_end = time.monotonic() + timeout_s
            while time.monotonic() < t_end and "r" not in ack:
                time.sleep(0.02)
        finally:
            self._listeners.remove(cb)
        if "r" not in ack:
            return CommandResult(name, CommandStatus.TIMEOUT)
        if ack["r"] == mavlink.MAV_RESULT_ACCEPTED:
            return CommandResult(name, CommandStatus.ACCEPTED)
        return CommandResult(name, CommandStatus.DENIED, f"MAV_RESULT {ack['r']}")

    def arm(self) -> CommandResult:
        r = self._command_long("arm", mavlink.MAV_CMD_COMPONENT_ARM_DISARM, [1])
        if r.status is CommandStatus.ACCEPTED and not self.wait(lambda o: o.get("armed") is True, 3):
            return CommandResult("arm", CommandStatus.TIMEOUT, "ACK received but HEARTBEAT not armed")
        return r

    def disarm(self) -> CommandResult:
        return self._command_long("disarm", mavlink.MAV_CMD_COMPONENT_ARM_DISARM, [0])

    def set_mode(self, mode: str, timeout_s: float = 3.0) -> CommandResult:
        if mode not in C.MODES:
            return CommandResult("set_mode", CommandStatus.REJECTED_LOCALLY, f"unknown mode {mode}")
        if self.read_only:
            return self._reject("set_mode")
        self._send(self.mav_out.set_mode_send, self.sysid, mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, C.MODES[mode])
        if self.wait(lambda o: o.get("mode") == mode, timeout_s):
            return CommandResult("set_mode", CommandStatus.ACCEPTED, mode)
        return CommandResult("set_mode", CommandStatus.TIMEOUT, mode)

    def set_depth_target(self, depth_m: float) -> CommandResult:
        if self.read_only:
            return self._reject("set_depth_target")
        if self.obs.get("mode") != "ALT_HOLD":
            return CommandResult("set_depth_target", CommandStatus.REJECTED_LOCALLY,
                                 "ArduSub 4.1.2 uses depth targets only in ALT_HOLD")
        mask = 0b110111111000
        self._send(self.mav_out.set_position_target_global_int_send, 0, self.sysid, 1,
                   mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, mask, 0, 0, -float(depth_m),
                   0, 0, 0, 0, 0, 0, 0, 0)
        return CommandResult("set_depth_target", CommandStatus.SENT, f"{depth_m} m (no ACK in MAVLink)")

    # -- pilot input (MANUAL_CONTROL stream) -----------------------------------------
    def set_sticks(self, x=0, y=0, z=500, r=0) -> CommandResult:
        if self.read_only:
            return self._reject("manual_control")
        vals = {"x": x, "y": y, "z": z, "r": r}
        for k, (lo, hi) in C.MANUAL_CONTROL_RANGES.items():
            if not lo <= vals[k] <= hi:
                return CommandResult("manual_control", CommandStatus.REJECTED_LOCALLY,
                                     f"{k}={vals[k]} outside {lo}..{hi} (ArduSub 4.1.2 convention)")
        with self._lock:
            self._sticks = {k: int(v) for k, v in vals.items()}
        self._ensure_pilot_stream()
        return CommandResult("manual_control", CommandStatus.SENT)

    def _ensure_pilot_stream(self):
        if self._pilot_thread is None:
            self._pilot_thread = threading.Thread(target=self._pilot_loop, daemon=True)
            self._pilot_thread.start()

    def _pilot_loop(self, rate_hz: float = 20.0):
        t_hb = 0.0
        while not self._stop.is_set():
            with self._lock:
                s, b = dict(self._sticks), self._buttons
            try:
                if time.monotonic() - t_hb >= 1.0:
                    t_hb = time.monotonic()
                    self._send(self.mav_out.heartbeat_send, mavlink.MAV_TYPE_GCS,
                               mavlink.MAV_AUTOPILOT_INVALID, 0, 0, mavlink.MAV_STATE_ACTIVE)
                self._send(self.mav_out.manual_control_send, self.sysid, s["x"], s["y"], s["z"], s["r"], b)
            except (ConnectionError, OSError):
                pass            # no autopilot stream yet: nothing to answer to
            time.sleep(1.0 / rate_hz)

    def press_button(self, name: str, hold_s: float = 0.3) -> CommandResult:
        """Press a joystick button as the vehicle's QGC joystick would (MANUAL_CONTROL.buttons)."""
        if name not in C.BUTTONS:
            return CommandResult("button", CommandStatus.REJECTED_LOCALLY, f"unknown button {name}")
        if self.read_only:
            return self._reject(f"button:{name}")
        bit, shifted = C.BUTTONS[name]
        self._ensure_pilot_stream()
        shift = 1 << C.SHIFT_BUTTON
        seq = ([shift, shift | (1 << bit), shift, 0] if shifted else [1 << bit, 0])
        durs = ([0.3, hold_s, 0.3, 0.3] if shifted else [hold_s, 0.3])
        for mask, d in zip(seq, durs):
            with self._lock:
                self._buttons = mask
            time.sleep(d)
        return CommandResult(f"button:{name}", CommandStatus.SENT)

    # accessory helpers (logical commands; physical effect awaiting verification)
    def gripper(self, action: str) -> CommandResult:
        return self.press_button({"open": "gripper_open", "close": "gripper_close"}[action])

    def lights(self, which: int, brighter: bool) -> CommandResult:
        return self.press_button(f"lights{which}_{'brighter' if brighter else 'dimmer'}")

    def camera_tilt(self, direction: str, hold_s: float = 1.0) -> CommandResult:
        name = {"up": "mount_tilt_up", "down": "mount_tilt_down", "center": "mount_center"}[direction]
        return self.press_button(name, hold_s)


C_GCS_SYSID = 255          # ArduSub accepts pilot input only from SYSID_MYGCS (255 on the vehicle)
