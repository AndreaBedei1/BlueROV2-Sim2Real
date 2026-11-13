"""Read QGC-style .tlog files (8-byte big-endian usec timestamp + MAVLink frame)."""
from __future__ import annotations

import struct
from collections import Counter
from pathlib import Path
from typing import Iterator, Tuple

from pymavlink import mavutil


def read_tlog(path: Path) -> Iterator[Tuple[float, object]]:
    """Yield (unix_time_s, pymavlink message) for every complete frame."""
    mav = mavutil.mavlink.MAVLink(None)
    mav.robust_parsing = True
    data = Path(path).read_bytes()
    i = 0
    while i + 8 < len(data):
        (t_us,) = struct.unpack(">Q", data[i:i + 8])
        i += 8
        start = data[i]
        if start == 0xFD:
            if i + 10 > len(data):
                break
            plen = data[i + 1]
            incompat = data[i + 2]
            n = 10 + plen + 2 + (13 if incompat & 0x01 else 0)
        elif start == 0xFE:
            plen = data[i + 1]
            n = 6 + plen + 2
        else:
            raise ValueError(f"not a tlog frame at byte {i}")
        frame = data[i:i + n]
        i += n
        for msg in mav.parse_buffer(frame) or []:
            yield t_us * 1e-6, msg


def summarize(path: Path):
    """Counts of (src_system, src_component, type)."""
    c = Counter()
    for _, m in read_tlog(path):
        c[(m.get_srcSystem(), m.get_srcComponent(), m.get_type())] += 1
    return c
