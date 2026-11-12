"""ArduPilot SITL JSON backend protocol, as implemented in ArduSub 4.1.2.

Verified against ``libraries/SITL/SIM_JSON.{h,cpp}`` at tag ``ArduSub-4.1.2``
(commit 2dd0bb7d):

* SITL -> simulator: one UDP datagram per physics frame, little-endian struct
  ``<HHI16H`` = magic 18458, frame_rate (Hz), frame_count, pwm[16] (us).
  4.1.2 always sends 16 channels (the 32-channel variant appeared in 4.5).
* simulator -> SITL: one text line terminated by ``\\n`` and sent back to the
  datagram's source address. SITL parses only the last complete line.
* The 4.1.2 parser is not a JSON parser: it locates ``key`` with ``strstr``
  and reads the value exactly two characters after the end of the key
  (skipping ``":``), and vectors with ``sscanf("[%f, %f, %f]")``. Separators
  therefore must be compact (``"key":value``) with no space after the colon.
  Mandatory fields: timestamp, imu.gyro, imu.accel_body, position, velocity,
  plus attitude or quaternion. A decreasing timestamp is treated as a reset.
* SITL blocks waiting for the reply and resends the same frame after ~1 s
  without an answer (``recv_fdm``: 100 ms timeout, resend when > 1000 ms).
"""
from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import Sequence, Tuple

SERVO_MAGIC = 18458
SERVO_STRUCT = struct.Struct("<HHI16H")
SERVO_PACKET_SIZE = SERVO_STRUCT.size  # 40 bytes


class ProtocolError(ValueError):
    """Raised for a datagram that is not a valid 4.1.2 servo packet."""


@dataclass(frozen=True)
class ServoPacket:
    frame_rate: int
    frame_count: int
    pwm: Tuple[int, ...]  # 16 values, microseconds; 0 = channel not output


def parse_servo_packet(data: bytes) -> ServoPacket:
    if len(data) != SERVO_PACKET_SIZE:
        raise ProtocolError(f"servo packet has {len(data)} bytes, expected {SERVO_PACKET_SIZE}")
    fields = SERVO_STRUCT.unpack(data)
    if fields[0] != SERVO_MAGIC:
        raise ProtocolError(f"bad magic {fields[0]}, expected {SERVO_MAGIC}")
    return ServoPacket(frame_rate=fields[1], frame_count=fields[2], pwm=tuple(fields[3:]))


def build_servo_packet(frame_rate: int, frame_count: int, pwm: Sequence[int]) -> bytes:
    """Inverse of :func:`parse_servo_packet` (used by tests and the mock SITL)."""
    pwm = list(pwm) + [0] * (16 - len(pwm))
    return SERVO_STRUCT.pack(SERVO_MAGIC, frame_rate, frame_count, *pwm[:16])


def _num(x: float) -> str:
    if not math.isfinite(x):
        raise ProtocolError(f"non-finite value {x!r} cannot be sent to SITL")
    return repr(float(x))


def _vec(v: Sequence[float]) -> str:
    return "[" + ",".join(_num(x) for x in v) + "]"


@dataclass(frozen=True)
class FdmState:
    """State returned to SITL. Units and frames are those expected by SITL."""
    timestamp_s: float                     # simulation time, s, monotonic
    gyro_frd: Sequence[float]              # body angular rate, rad/s, FRD
    accel_body_frd: Sequence[float]        # specific force, m/s^2, FRD (~[0,0,-9.8] at rest)
    position_ned: Sequence[float]          # m, NED, relative to the SITL origin
    velocity_ned: Sequence[float]          # m/s, NED
    quaternion_wxyz: Sequence[float]       # rotation body(FRD) -> earth(NED)


def encode_fdm(state: FdmState) -> bytes:
    """Encode a state as the compact one-line JSON accepted by ArduSub 4.1.2."""
    body = (
        '{"timestamp":' + _num(state.timestamp_s)
        + ',"imu":{"gyro":' + _vec(state.gyro_frd)
        + ',"accel_body":' + _vec(state.accel_body_frd) + "}"
        + ',"position":' + _vec(state.position_ned)
        + ',"quaternion":' + _vec(state.quaternion_wxyz)
        + ',"velocity":' + _vec(state.velocity_ned)
        + "}"
    )
    return ("\n" + body + "\n").encode("ascii")


# --- Re-implementation of the 4.1.2 parser, used to test our encoder ----------

_KEYTABLE = [
    # section, key, kind, required
    ("", "timestamp", "double", True),
    ("imu", "gyro", "vec3", True),
    ("imu", "accel_body", "vec3", True),
    ("", "position", "vec3", True),
    ("", "attitude", "vec3", False),
    ("", "quaternion", "quat", False),
    ("", "velocity", "vec3", True),
]


def _scan_floats(s: str, n: int):
    """Mimic sscanf("[%f, %f, ...]"): literal '[' first, then n comma-separated floats."""
    if not s.startswith("["):
        return None
    inner = s[1:]
    out = []
    for i in range(n):
        inner = inner.lstrip()
        end = 0
        while end < len(inner) and inner[end] in "0123456789+-.eE":
            end += 1
        if end == 0:
            return None
        try:
            out.append(float(inner[:end]))
        except ValueError:
            return None
        inner = inner[end:]
        if i < n - 1:
            inner = inner.lstrip()
            if not inner.startswith(","):
                return None
            inner = inner[1:]
    return out


def ardusub412_parse(line: str) -> dict:
    """Parse a line the way ``JSON::parse_sensors`` (4.1.2) does.

    Stricter than SITL on purpose: raises ProtocolError on any missing
    mandatory field or vector parse failure. (SITL 4.1.2 rejects a line only
    when the bitmask is 0 or has no attitude; after a vector parse failure it
    may accept a partial update with stale fields.) Used to check that our
    encoder never relies on that lenient behaviour.
    """
    result = {}
    for section, key, kind, required in _KEYTABLE:
        p = line.find(section)          # strstr(json, section); "" matches at 0
        if p < 0:
            if required:
                raise ProtocolError(f"Failed to find {section}")
            continue
        p += len(section) + 1            # as in SIM_JSON.cpp, also for an empty section
        q = line.find(key, p)
        if q < 0:
            if required:
                raise ProtocolError(f"Failed to find key {section}/{key}")
            continue
        value = line[q + len(key) + 2:]
        if kind == "double":
            num = ""
            for ch in value.lstrip():
                if ch in "0123456789+-.eE":
                    num += ch
                else:
                    break
            result[key] = float(num) if num else 0.0
        else:
            n = 4 if kind == "quat" else 3
            vals = _scan_floats(value, n)
            if vals is None:
                raise ProtocolError(f"Failed to parse vector for {section}/{key}")
            result[key] = vals
    if "attitude" not in result and "quaternion" not in result:
        raise ProtocolError("Did not receive attitude or quaternion")
    return result
