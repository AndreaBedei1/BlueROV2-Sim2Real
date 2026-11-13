"""Versioned vehicle interface contract (Phase 3).

One contract, two execution environments. The contract fixes the *meaning*
of every command and observation the autonomy software may use; the
environment (simulation or physical vehicle) only changes the transport.

The reference is the laboratory vehicle as documented in Phase 1:
ArduSub 4.1.2 on Navigator, BlueOS 1.3.1, eight thrusters (VECTORED_6DOF,
horizontals at 30 deg), accessories on outputs 9/13/14/16.

Every interface carries a status:
  VERIFIED_EQUIVALENT        same messages/semantics shown in SITL with the
                             vehicle's firmware version and parameters
  FUNCTIONAL_APPROXIMATION   same interface, simplified or different behaviour
                             behind it (e.g. simulated sensor values)
  REPLAY_ONLY                only recorded real data can be served
  UNSUPPORTED                not available in simulation (or not at all)
  AWAITING_PHYSICAL          cannot be verified without the physical vehicle
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Dict, List, Optional

CONTRACT_VERSION = "1.0.0"
ARDUSUB_REFERENCE = "ArduSub 4.1.2 (2dd0bb7d), BlueOS 1.3.1"


class Status(str, Enum):
    VERIFIED_EQUIVALENT = "verified_equivalent"
    FUNCTIONAL_APPROXIMATION = "functional_approximation"
    REPLAY_ONLY = "replay_only"
    UNSUPPORTED = "unsupported"
    AWAITING_PHYSICAL = "awaiting_physical_verification"


# ArduSub 4.1.2 custom modes (ArduSub/defines.h)
MODES: Dict[str, int] = {"STABILIZE": 0, "ACRO": 1, "ALT_HOLD": 2, "AUTO": 3, "GUIDED": 4,
                         "CIRCLE": 7, "SURFACE": 9, "POSHOLD": 16, "MANUAL": 19}
MODE_NAMES = {v: k for k, v in MODES.items()}

# MANUAL_CONTROL ranges in ArduSub 4.1.2 (joystick.cpp): x, y, r in -1000..1000,
# z in 0..1000 with 500 = no vertical input. Buttons = bitmask, bit n -> BTNn_*.
MANUAL_CONTROL_RANGES = {"x": (-1000, 1000), "y": (-1000, 1000), "z": (0, 1000), "r": (-1000, 1000)}
MANUAL_CONTROL_NEUTRAL = {"x": 0, "y": 0, "z": 500, "r": 0}

# Joystick buttons of the vehicle backup (configs/sitl/phase3_accessories.parm)
BUTTONS = {
    "mount_center": (7, False), "mount_tilt_up": (9, False), "mount_tilt_down": (10, False),
    "lights1_dimmer": (13, False), "lights1_brighter": (14, False),
    "gripper_close": (3, True), "gripper_open": (4, True),          # servo_1 min / max (shifted)
    "lights2_brighter": (13, True), "lights2_dimmer": (14, True),  # pairing ASSUMED, see parm file
}
SHIFT_BUTTON = 5          # BTN5_FUNCTION = 1 (shift) in ArduSub's sub.parm; vehicle value not in the inventory

# Accessory outputs (SERVOn, 1-based) and decoding
ACCESSORY_OUTPUTS = {"gripper": 9, "lights1": 13, "lights2": 14, "camera_tilt": 16}
PWM_MIN, PWM_MAX = 1100, 1900
TILT_RANGE_DEG = (-45.0, 45.0)      # MNT_ANGMIN/MAX_TIL
TILT_SERVO_REVERSED = True          # SERVO16_REVERSED = 1
RANGE_MIN_QUALITY = 50              # BlueOS sends signal_quality = max(1, Ping1D confidence %)


@dataclass
class InterfaceItem:
    name: str
    physical_reference: str
    simulation: str
    status: Status
    evidence: str = ""
    notes: str = ""


@dataclass
class Contract:
    version: str = CONTRACT_VERSION
    reference: str = ARDUSUB_REFERENCE
    frames: Dict[str, str] = field(default_factory=lambda: {
        "earth": "NED (north, east, down); depth = +down in metres below the surface reference",
        "body": "FRD (forward, right, down)",
        "attitude": "ATTITUDE roll/pitch/yaw [rad], ArduPilot Z-Y-X, yaw from north clockwise",
        "rates": "ATTITUDE rollspeed/pitchspeed/yawspeed [rad/s], body FRD",
    })
    time: Dict[str, str] = field(default_factory=lambda: {
        "autopilot": "time_boot_ms of the autopilot (SITL: simulated time; vehicle: since ArduSub boot). "
                     "Use it for control timing.",
        "receive": "wall-clock receive time on the client (monotonic), for latency/age only",
        "utc": "not available: SYSTEM_TIME.time_unix_usec = 0 on the vehicle (no RTC/NTP) and in SITL",
    })
    observations: Dict[str, str] = field(default_factory=lambda: {
        "armed": "HEARTBEAT.base_mode & MAV_MODE_FLAG_SAFETY_ARMED (component 1)",
        "mode": "HEARTBEAT.custom_mode -> MODES",
        "attitude": "ATTITUDE",
        "depth_m": "-GLOBAL_POSITION_INT.relative_alt/1000 (EKF depth from the external pressure sensor). "
                   "VFR_HUD.alt is NOT used (constant on the vehicle, Phase 1)",
        "climb_rate_mps": "climb (up-positive) = -GLOBAL_POSITION_INT.vz/100 (vz is NED, down-positive)",
        "pressure_hpa": "SCALED_PRESSURE2.press_abs (external depth sensor)",
        "battery_v": "SYS_STATUS.voltage_battery/1000 (simulated value in SITL)",
        "servo_us": "SERVO_OUTPUT_RAW servo1..16; outputs 9/13/14/16 accessories. Outputs 1-8 (motors) are a "
                    "FUNCTIONAL APPROXIMATION: SITL uses MOT_n_DIRECTION=+1, the vehicle -1 on motors 1,2,5,8, "
                    "so motor PWM signs differ between environments; use the logical mix, not raw motor PWM",
        "mount_tilt_deg": "MOUNT_STATUS.pointing_a/100 (mount body-frame output angle, AP_Mount_Servo)",
        "rangefinder_m": "DISTANCE_SENSOR from the Ping1D service (component 194, sysid of the vehicle "
                         "assumed) only, signal_quality >= RANGE_MIN_QUALITY; 0 (unknown) / 1 (invalid) -> None",
        "link_ok": "autopilot HEARTBEAT within 3 s and receive thread alive; decide only on fresh data",
        "horizontal_position": "NOT AVAILABLE: no DVL/USBL/GPS underwater; EKF horizontal position is "
                               "dead reckoning and must not be used (ROVL unverified)",
    })
    commands: Dict[str, str] = field(default_factory=lambda: {
        "arm/disarm": "COMMAND_LONG MAV_CMD_COMPONENT_ARM_DISARM, result from COMMAND_ACK",
        "set_mode": "SET_MODE (custom mode), confirmed by HEARTBEAT",
        "manual_control": "MANUAL_CONTROL x,y,r -1000..1000, z 0..1000 (500 neutral), buttons; sent "
                          "continuously by ONE pilot source (QGC ~25 Hz, client 20 Hz; a design choice). RC "
                          "overrides expire after 3 s; GCS failsafe 2.5 s when armed. FS_PILOT_INPUT is "
                          "compiled out of SITL (awaiting physical verification)",
        "depth_target": "SET_POSITION_TARGET_GLOBAL_INT, position only, alt = -depth, ALT_HOLD only; ArduSub "
                        "uses alt as z relative to the EKF origin and ignores frame/lat/lon; equals depth only "
                        "when the EKF origin is at the surface; no ACK exists (observe depth)",
        "accessories": "MANUAL_CONTROL buttons mapped by BTNn_FUNCTION (gripper, lights, camera tilt)",
    })
    items: List[InterfaceItem] = field(default_factory=list)

    def to_json(self) -> str:
        d = asdict(self)
        return json.dumps(d, indent=2, default=lambda o: o.value if isinstance(o, Enum) else str(o))


def decode_tilt_deg(pwm: Optional[int]) -> Optional[float]:
    """Camera tilt angle (deg, + up) from the output-16 PWM (reversed servo)."""
    if pwm is None or pwm < PWM_MIN or pwm > PWM_MAX:
        return None
    if TILT_SERVO_REVERSED:
        pwm = PWM_MIN + PWM_MAX - pwm
    lo, hi = TILT_RANGE_DEG
    return lo + (pwm - PWM_MIN) * (hi - lo) / (PWM_MAX - PWM_MIN)


def decode_level(pwm: Optional[int]) -> Optional[float]:
    """Light level 0..1 from a passthrough output (1100 = off, 1900 = full)."""
    if pwm is None or pwm < PWM_MIN or pwm > PWM_MAX:
        return None
    return (pwm - PWM_MIN) / (PWM_MAX - PWM_MIN)


def decode_gripper(pwm: Optional[int]) -> Optional[str]:
    """Logical gripper command from output 9 (vehicle config: 1900 open, 1100 close; L5 presence)."""
    if pwm is None or pwm == 0:
        return None
    if pwm >= 1800:
        return "open"
    if pwm <= 1200:
        return "close"
    return "neutral"
