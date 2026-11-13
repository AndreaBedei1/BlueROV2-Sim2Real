"""Loopback MAVLink router for the simulation profile.

Plays the role of BlueOS' MAVLink router on the vehicle: one autopilot link,
several ground/companion endpoints, every complete MAVLink message forwarded
unchanged in both directions and recorded in a QGC-compatible ``.tlog``
(8-byte big-endian microsecond timestamp + raw message bytes).

Endpoints (all bound to 127.0.0.1):

* ``tcp_server``  - QGroundControl connects here (TCP client link).
* ``udp_out``     - like BlueOS "GCS Client Link" (udpout): the router sends to
  a fixed local port and accepts replies on the same socket.
* ``udp_in``      - like a BlueOS "udpin" link: local components (e.g. the
  Ping1D adapter publishing DISTANCE_SENSOR) send here; the router answers to
  the last sender.

Differences from the vehicle (documented, not hidden): transport ports and
the TCP link used by QGC in simulation (the vehicle pushes UDP to the topside
computer on 14550); no routing table by system/component id - every message
from one side goes to every endpoint of the other side. mavlink-router (BlueOS)
routes targeted messages only to endpoints that have seen the target, so e.g.
a companion's MANUAL_CONTROL reaches QGC here but not on the vehicle (not
verified on BlueOS).
"""
from __future__ import annotations

import selectors
import socket
import struct
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

from pymavlink import mavutil

LOOPBACK = "127.0.0.1"


class _Splitter:
    """Split a byte stream into complete MAVLink v1/v2 frames (raw bytes)."""

    def __init__(self):
        self.mav = mavutil.mavlink.MAVLink(None)
        self.mav.robust_parsing = True

    def feed(self, data: bytes) -> List[bytes]:
        out = []
        for msg in self.mav.parse_buffer(data) or []:
            if msg.get_type() != "BAD_DATA":
                out.append(bytes(msg.get_msgbuf()))
        return out


@dataclass
class RouterConfig:
    master: Tuple[str, int] = (LOOPBACK, 5830)          # SITL serial0 (instance 7)
    tcp_server_port: Optional[int] = 5880                # QGroundControl
    udp_out_ports: List[int] = field(default_factory=lambda: [14570])   # autonomy client
    udp_in_ports: List[int] = field(default_factory=lambda: [14580])    # sensor adapters
    tlog: Optional[Path] = None
    connect_timeout_s: float = 60.0


class MavlinkRouter:
    def __init__(self, cfg: RouterConfig):
        if cfg.master[0] != LOOPBACK:
            raise ValueError("simulation router only connects to a loopback autopilot")
        self.cfg = cfg
        self.sel = selectors.DefaultSelector()
        self.master = None
        self.master_split = _Splitter()
        self.tcp_clients = {}           # sock -> splitter
        self.udp_out = []               # (sock, (host, port), splitter)
        self.udp_in = []                # [sock, last_peer, splitter]
        self.tlog = open(cfg.tlog, "wb") if cfg.tlog else None
        self.counts = {"to_endpoints": 0, "to_master": 0}
        self._open_endpoints()

    # -- setup -----------------------------------------------------------------
    def _open_endpoints(self):
        if self.cfg.tcp_server_port:
            srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind((LOOPBACK, self.cfg.tcp_server_port))
            srv.listen(4)
            srv.setblocking(False)
            self.sel.register(srv, selectors.EVENT_READ, ("tcp_accept", None))
        for port in self.cfg.udp_out_ports:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.bind((LOOPBACK, 0))
            s.setblocking(False)
            ep = (s, (LOOPBACK, port), _Splitter())
            self.udp_out.append(ep)
            self.sel.register(s, selectors.EVENT_READ, ("udp_out", ep))
        for port in self.cfg.udp_in_ports:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.bind((LOOPBACK, port))
            s.setblocking(False)
            ep = [s, None, _Splitter()]
            self.udp_in.append(ep)
            self.sel.register(s, selectors.EVENT_READ, ("udp_in", ep))

    def connect_master(self):
        t0 = time.monotonic()
        while True:
            try:
                m = socket.create_connection(self.cfg.master, timeout=2.0)
                break
            except OSError:
                if time.monotonic() - t0 > self.cfg.connect_timeout_s:
                    raise
                time.sleep(0.5)
        m.settimeout(2.0)              # blocking sends with timeout: no partial-send crash
        m.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.master = m
        self.sel.register(m, selectors.EVENT_READ, ("master", None))

    # -- forwarding --------------------------------------------------------------
    def _log(self, frame: bytes):
        if self.tlog:
            self.tlog.write(struct.pack(">Q", int(time.time() * 1e6)) + frame)

    def _to_endpoints(self, frame: bytes):
        self._log(frame)
        self.counts["to_endpoints"] += 1
        for sock in list(self.tcp_clients):
            try:
                sock.sendall(frame)
            except OSError:
                self._drop_tcp(sock)
        for s, addr, _ in self.udp_out:
            try:
                s.sendto(frame, addr)
            except OSError:
                pass        # nobody listening yet: like udpout, silently dropped
        for ep in self.udp_in:
            if ep[1] is not None:
                try:
                    ep[0].sendto(frame, ep[1])
                except OSError:
                    pass

    def _to_master(self, frame: bytes, from_ep=None):
        self._log(frame)
        self.counts["to_master"] += 1
        self.master.sendall(frame)
        # BlueOS also lets companions see each other's messages (e.g. DISTANCE_SENSOR)
        for sock in list(self.tcp_clients):
            if sock is not from_ep:
                try:
                    sock.sendall(frame)
                except OSError:
                    self._drop_tcp(sock)
        for s, addr, _ in self.udp_out:
            if s is not from_ep:
                try:
                    s.sendto(frame, addr)
                except OSError:
                    pass

    def _drop_tcp(self, sock):
        try:
            self.sel.unregister(sock)
        except Exception:  # noqa: BLE001
            pass
        self.tcp_clients.pop(sock, None)
        sock.close()

    def run(self, duration_s: Optional[float] = None, stop_flag=None):
        if self.master is None:
            self.connect_master()
        t_end = None if duration_s is None else time.monotonic() + duration_s
        while (t_end is None or time.monotonic() < t_end) and not (stop_flag and stop_flag()):
            for key, _ in self.sel.select(timeout=0.2):
                kind, ep = key.data
                sock = key.fileobj
                if kind == "master":
                    data = sock.recv(65536)
                    if not data:
                        raise ConnectionError("autopilot closed the link")
                    for f in self.master_split.feed(data):
                        self._to_endpoints(f)
                elif kind == "tcp_accept":
                    c, addr = sock.accept()
                    if addr[0] != LOOPBACK:
                        c.close()
                        continue
                    c.settimeout(2.0)
                    self.tcp_clients[c] = _Splitter()
                    self.sel.register(c, selectors.EVENT_READ, ("tcp_client", None))
                elif kind == "tcp_client":
                    try:
                        data = sock.recv(65536)
                    except OSError:
                        data = b""
                    if not data:
                        self._drop_tcp(sock)
                        continue
                    for f in self.tcp_clients[sock].feed(data):
                        self._to_master(f, from_ep=sock)
                elif kind == "udp_out":
                    data, addr = sock.recvfrom(65536)
                    if addr[0] == LOOPBACK:
                        for f in ep[2].feed(data):
                            self._to_master(f, from_ep=sock)
                elif kind == "udp_in":
                    data, addr = sock.recvfrom(65536)
                    if addr[0] == LOOPBACK:
                        ep[1] = addr
                        for f in ep[2].feed(data):
                            self._to_master(f, from_ep=sock)

    def close(self):
        for key in list(self.sel.get_map().values()):
            try:
                key.fileobj.close()
            except Exception:  # noqa: BLE001
                pass
        if self.tlog:
            self.tlog.close()
