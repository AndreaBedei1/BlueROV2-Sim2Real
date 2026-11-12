"""Simulated Ping1D (Blue Robotics) exposed through the native Ping protocol over UDP.

On the vehicle BlueOS bridges the Ping1D serial port to UDP 9090; clients such as
Ping Viewer or ``brping.Ping1D.connect_udp`` exchange Ping protocol frames.
This adapter answers the same requests with a distance taken from a simulated
down-looking range measurement (HoloOcean RangeFinderSensor, or any callable).

Status: FUNCTIONAL APPROXIMATION. The protocol is verified with Blue Robotics'
own client library; the *values* are a geometric ray distance with a simple
confidence model, not Ping1D acoustics. Phase 1 found no valid real Ping1D data
(records empty, in-air readings meaningless), so nothing is replayed and no
"typical" real values are invented: when no range is available the adapter
reports confidence 0, which consumers must treat as invalid.

Message ids and payload layouts come from the brping package definitions
(ping-protocol), so encoder and reference client share one source of truth.
"""
from __future__ import annotations

import socket
import struct
import threading
from typing import Callable, Optional

from . import ping_protocol as pp

GENERAL_REQUEST = 6
DEVICE_INFORMATION = 4
PROTOCOL_VERSION = 5
FIRMWARE_VERSION = 1200
DEVICE_ID = 1201
SPEED_OF_SOUND = 1203
RANGE = 1204
MODE_AUTO = 1205
PING_INTERVAL = 1206
GAIN_SETTING = 1207
TRANSMIT_DURATION = 1208
GENERAL_INFO = 1210
DISTANCE_SIMPLE = 1211
DISTANCE = 1212
PING_ENABLE = 1215
SET_PING_ENABLE = 1006

DEVICE_TYPE_PING1D = 1
MAX_RANGE_MM = 50_000      # Ping1D rated range


class Ping1DAdapter:
    """UDP server speaking Ping protocol. ``distance_source()`` returns metres or None."""

    def __init__(self, distance_source: Callable[[], Optional[float]], port: int, device_id: int = 1):
        self.source = distance_source
        self.device_id = device_id
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", port))
        self.sock.settimeout(0.2)
        self.ping_number = 0
        self.requests = 0
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)

    def start(self):
        self.thread.start()
        return self

    def _measure(self):
        d = self.source()
        self.ping_number += 1
        if d is None or not (0.0 <= d * 1000 <= MAX_RANGE_MM):
            return 0, 0                     # no echo: distance 0, confidence 0 (invalid)
        return int(round(d * 1000)), 100   # geometric hit: no acoustic confidence model

    def _reply(self, mid: int) -> Optional[bytes]:
        if mid == DEVICE_INFORMATION:
            # device_type, device_revision, firmware major/minor/patch, reserved
            p = struct.pack("<BBBBBB", DEVICE_TYPE_PING1D, 1, 0, 0, 0, 0)
        elif mid == PROTOCOL_VERSION:
            p = struct.pack("<BBBB", 1, 0, 0, 0)
        elif mid == FIRMWARE_VERSION:
            p = struct.pack("<BBHH", DEVICE_TYPE_PING1D, 1, 0, 0)    # device_type, device_model, major, minor
        elif mid == DEVICE_ID:
            p = struct.pack("<B", self.device_id)
        elif mid == SPEED_OF_SOUND:
            p = struct.pack("<I", 1_500_000)
        elif mid == RANGE:
            p = struct.pack("<II", 0, MAX_RANGE_MM)
        elif mid == MODE_AUTO:
            p = struct.pack("<B", 1)
        elif mid == PING_INTERVAL:
            p = struct.pack("<H", 100)
        elif mid == GAIN_SETTING:
            p = struct.pack("<I", 0)
        elif mid == TRANSMIT_DURATION:
            p = struct.pack("<H", 100)
        elif mid == PING_ENABLE:
            p = struct.pack("<B", 1)
        elif mid == DISTANCE_SIMPLE:
            d, c = self._measure()
            p = struct.pack("<IB", d, c)
        elif mid == DISTANCE:
            d, c = self._measure()
            p = struct.pack("<IHHIIII", d, c, 100, self.ping_number, 0, MAX_RANGE_MM, 0)
        elif mid == GENERAL_INFO:
            p = struct.pack("<HHHHBB", 0, 0, 5000, 100, 0, 1)   # fw major, minor, voltage_5 mV, ping_interval, gain, mode_auto
        else:
            return None
        return pp.encode(mid, p, src=self.device_id, dst=0)

    def run(self):
        parser = pp.Parser()
        while not self.stop.is_set():
            try:
                data, addr = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                return
            if addr[0] != "127.0.0.1":
                continue
            for f in parser.feed(data):
                self.requests += 1
                if f.message_id == GENERAL_REQUEST and len(f.payload) >= 2:
                    want = struct.unpack_from("<H", f.payload, 0)[0]
                elif not f.payload:
                    want = f.message_id          # legacy Ping1D request: empty message with the wanted id
                else:
                    continue                     # set_* commands are not emulated (read-only sensor model)
                r = self._reply(want)
                if r:
                    self.sock.sendto(r, addr)

    def close(self):
        self.stop.set()
        self.sock.close()
