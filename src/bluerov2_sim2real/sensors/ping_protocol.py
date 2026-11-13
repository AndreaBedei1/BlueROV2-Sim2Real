"""Blue Robotics / Cerulean "Ping protocol" framing (used by Ping1D and Surveyor 240-16).

Frame: 'B' 'R' | u16 payload_length | u16 message_id | u8 src | u8 dst | payload | u16 checksum,
little-endian, checksum = sum of all preceding bytes of the frame, truncated to 16 bits
(Cerulean "Universal Packet Format"; confirmed on real Surveyor .svlog files, where every
frame's checksum verifies with this rule).
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Iterator, Optional

HEADER = struct.Struct("<2sHHBB")


@dataclass(frozen=True)
class Frame:
    message_id: int
    src: int
    dst: int
    payload: bytes
    raw: bytes                   # exact bytes as received / recorded

    @property
    def checksum_ok(self) -> bool:
        return verify(self.raw)


def checksum(data: bytes) -> int:
    return sum(data) & 0xFFFF


def encode(message_id: int, payload: bytes = b"", src: int = 0, dst: int = 0) -> bytes:
    head = HEADER.pack(b"BR", len(payload), message_id, src, dst)
    body = head + payload
    return body + struct.pack("<H", checksum(body))


def verify(raw: bytes) -> bool:
    return len(raw) >= 10 and checksum(raw[:-2]) == struct.unpack("<H", raw[-2:])[0]


class Parser:
    """Incremental parser; resynchronises on 'BR'. Bad-checksum frames are counted and dropped."""

    def __init__(self):
        self.buf = bytearray()
        self.bad_checksum = 0
        self.resyncs = 0

    def feed(self, data: bytes) -> Iterator[Frame]:
        self.buf += data
        while True:
            i = self.buf.find(b"BR")
            if i < 0:
                del self.buf[:-1]
                return
            if i > 0:
                self.resyncs += 1
                del self.buf[:i]
            if len(self.buf) < 8:
                return
            _, plen, mid, src, dst = HEADER.unpack_from(self.buf, 0)
            n = 8 + plen + 2
            if len(self.buf) < n:
                return
            raw = bytes(self.buf[:n])
            if not verify(raw):
                self.bad_checksum += 1
                del self.buf[:2]
                continue
            del self.buf[:n]
            yield Frame(mid, src, dst, raw[8:8 + plen], raw)


def iter_file(path) -> Iterator[Frame]:
    p = Parser()
    with open(path, "rb") as f:
        while True:
            chunk = f.read(1 << 20)
            if not chunk:
                break
            yield from p.feed(chunk)


def first_or_none(frames, message_id) -> Optional[Frame]:
    for f in frames:
        if f.message_id == message_id:
            return f
    return None
