"""Cerulean Surveyor 240-16: .svlog parsing and byte-exact replay.

Status in Phase 3: REPLAY ONLY. There is no simulated Surveyor: HoloOcean's
imaging sonar is a different instrument and fabricating ATOF detections from it
would claim a fidelity that has not been measured.

Decoded (Cerulean public API docs): ATOF_POINT_DATA 3012, ATTITUDE_REPORT 504,
DEVICE_INFORMATION 4, JSON_WRAPPER 10, WATER_STATS 118. Everything else in a log
(e.g. 3009, 3010 seen in the laboratory logs, MAVLINK_WRAPPER 150) is kept and
replayed byte-for-byte but not interpreted.

The replay server listens on TCP (loopback, configurable port; the device uses
62312) and streams recorded frames unchanged, paced by the recorded
``pwr_up_msec`` time stamps. By default only frames whose source id is the
device (src 1) are sent; SonarView-host frames in the log (JSON session
metadata src 0, MAVLINK_WRAPPER src 2) are dropped unless include_host_frames.
So the stream is byte-exact to the *device frames of the log*, not a capture of
the TCP stream itself. Device frames 3009/3010/14 are passed through without
interpretation. Streaming starts when the client sends SET_PING_PARAMETERS with
ping_enable = true (the documented way to start pinging) or immediately with
``autostart=True``; afterwards client commands are ignored (no stop, no
parameter changes, no request/response emulation). Pacing follows the time
stamp of the last timed frame (ATTITUDE frames precede the ATOF of the same ping
by up to ~0.3 s, so frames go out in small bursts).
"""
from __future__ import annotations

import json
import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import List, Optional

from . import ping_protocol as pp

ATOF_POINT_DATA = 3012
YZ_POINT_DATA = 3011
ATTITUDE_REPORT = 504
WATER_STATS = 118
DEVICE_INFORMATION = 4
JSON_WRAPPER = 10
SET_PING_PARAMETERS = 3023


@dataclass
class AtofPing:
    pwr_up_msec: int
    utc_msec: int
    listening_sec: float
    sos_mps: float
    ping_number: int
    ping_hz: int
    pulse_sec: float
    angles_rad: List[float]          # positive to port
    tof_s: List[float]

    def points_yz(self):
        """(y port+, z up+) in metres, per Cerulean's formulas."""
        import math
        out = []
        for a, t in zip(self.angles_rad, self.tof_s):
            d = 0.5 * self.sos_mps * t
            out.append((d * math.sin(a), -d * math.cos(a)))
        return out


def decode_atof(payload: bytes) -> AtofPing:
    pwr, utc, listen, sos, num, hz, pulse, _flags, n, _r = struct.unpack_from("<IQffIIfIHH", payload, 0)
    angles, tofs = [], []
    for k in range(n):
        a, t = struct.unpack_from("<ff", payload, 40 + 16 * k)
        angles.append(a)
        tofs.append(t)
    return AtofPing(pwr, utc, listen, sos, num, hz, pulse, angles, tofs)


def decode_attitude(payload: bytes):
    ux, uy, uz = struct.unpack_from("<fff", payload, 0)
    utc, pwr = struct.unpack_from("<QI", payload, 24)
    import math
    return {"up_vec": (ux, uy, uz), "pitch_rad": math.asin(max(-1.0, min(1.0, ux))),
            "roll_rad": math.atan2(uy, uz), "utc_msec": utc, "pwr_up_msec": pwr}


def frame_time_ms(f: pp.Frame) -> Optional[int]:
    """Device power-up time of a frame, when the message carries one."""
    if f.message_id == ATOF_POINT_DATA and len(f.payload) >= 4:
        return struct.unpack_from("<I", f.payload, 0)[0]
    if f.message_id == ATTITUDE_REPORT and len(f.payload) >= 36:
        return struct.unpack_from("<I", f.payload, 32)[0]
    return None


def summarize(path) -> dict:
    counts, sizes, pings, session = {}, {}, 0, None
    t_first = t_last = None
    for f in pp.iter_file(path):
        counts[f.message_id] = counts.get(f.message_id, 0) + 1
        sizes.setdefault(f.message_id, set()).add(len(f.payload))
        t = frame_time_ms(f)
        if t is not None:
            t_first = t if t_first is None else t_first
            t_last = t
        if f.message_id == ATOF_POINT_DATA:
            pings += 1
        if f.message_id == JSON_WRAPPER and session is None:
            try:
                session = json.loads(f.payload.rstrip(b"\x00").decode())
            except ValueError:
                session = None
    return {"frames_by_id": counts, "payload_sizes": {k: sorted(v)[:3] for k, v in sizes.items()},
            "atof_pings": pings, "duration_s": (t_last - t_first) / 1e3 if t_first is not None else None,
            "session_json_keys": sorted(session.keys()) if isinstance(session, dict) else None}


class ReplayServer:
    """Serve recorded Surveyor frames over TCP on loopback, byte-for-byte."""

    def __init__(self, path, port: int, autostart: bool = False, speed: float = 1.0, loop: bool = False,
                 include_host_frames: bool = False, device_src: int = 1):
        frames = list(pp.iter_file(path))
        self.frames = frames if include_host_frames else [f for f in frames if f.src == device_src]
        if speed > 0 and not any(frame_time_ms(f) is not None for f in self.frames):
            raise ValueError("log has no time-stamped device frames: cannot pace a replay")
        self.port, self.autostart, self.speed, self.loop = port, autostart, speed, loop
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", port))
        self.srv.listen(1)
        self.sent_frames = 0
        self.stop = threading.Event()

    def serve_one(self, timeout_s: float = 30.0):
        self.srv.settimeout(timeout_s)
        conn, _ = self.srv.accept()
        conn.settimeout(0.05)
        enabled = self.autostart
        parser = pp.Parser()
        try:
            while not self.stop.is_set():
                t0_wall, t0_dev = None, None          # pacing restarts at every pass (pwr_up_msec restarts)
                for f in self.frames:
                    while not enabled and not self.stop.is_set():
                        try:
                            data = conn.recv(4096)
                        except socket.timeout:
                            continue
                        if not data:
                            return
                        for cmd in parser.feed(data):
                            if cmd.message_id == SET_PING_PARAMETERS and len(cmd.payload) >= 20:
                                enabled = bool(cmd.payload[19])
                    t = frame_time_ms(f)
                    if t is not None and self.speed > 0:
                        if t0_dev is None:
                            t0_wall, t0_dev = time.monotonic(), t
                        wait = (t - t0_dev) / 1e3 / self.speed - (time.monotonic() - t0_wall)
                        if wait > 0:
                            time.sleep(wait)
                    conn.sendall(f.raw)
                    self.sent_frames += 1
                    if self.stop.is_set():
                        break
                if not self.loop:
                    break
        finally:
            conn.close()

    def close(self):
        self.stop.set()
        self.srv.close()


def set_ping_parameters(ping_enable: bool, end_mm: int = 0, enable_atof: bool = True) -> bytes:
    """SET_PING_PARAMETERS (3023) per the Cerulean API page (field order as documented)."""
    # layout as in brping's surveyor240 definition: iifhhHBBBBBBiHHf (36 bytes)
    payload = struct.pack("<iifhhHBBBBBBiHHf", 0, end_mm, 1500.0, -1, 0, 0, 0, int(ping_enable),
                          0, 0, 0, int(enable_atof), 240000, 400, 0, 1.5)
    return pp.encode(SET_PING_PARAMETERS, payload)
