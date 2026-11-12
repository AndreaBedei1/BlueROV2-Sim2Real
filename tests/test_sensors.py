"""Ping-protocol, Surveyor replay and Ping1D adapter tests, checked against Blue Robotics' brping."""
import socket
import struct
import threading
import time
from pathlib import Path

import pytest

from bluerov2_sim2real.sensors import ping_protocol as pp
from bluerov2_sim2real.sensors import surveyor as sv
from bluerov2_sim2real.sensors.ping1d import Ping1DAdapter

REAL_SVLOG = Path("/data/Andrea/RobertoMiddleware/Registrazioni_Rover_2026-10-19/sonar/"
                  "LAGO_SONAR_TEST__2026-10-19-11-18.svlog")   # private lab data, never committed


def free_port(kind=socket.SOCK_STREAM):
    s = socket.socket(socket.AF_INET, kind)
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def test_frame_roundtrip_and_brping_compatibility():
    from brping import PingMessage
    raw = pp.encode(1211, struct.pack("<IB", 2500, 100), src=1)
    f = list(pp.Parser().feed(b"xx" + raw))[0]
    assert (f.message_id, f.src, f.payload, f.raw) == (1211, 1, struct.pack("<IB", 2500, 100), raw)
    m = PingMessage(msg_data=raw)                 # vendor parser accepts our frame
    assert m.message_id == 1211 and m.altitude_mm == 2500 and m.quality == 100


def test_parser_rejects_bad_checksum_and_resyncs():
    raw = bytearray(pp.encode(4, b"\x01\x02\x03\x04\x05\x06"))
    raw[-1] ^= 0xFF
    p = pp.Parser()
    assert list(p.feed(bytes(raw) + pp.encode(5, b"\x01\x00\x00\x00"))) [0].message_id == 5
    assert p.bad_checksum == 1


def _atof_frame(pwr_ms, n=3):
    pts = b"".join(struct.pack("<ffII", -0.5 + 0.5 * k, 0.004, 0, 0) for k in range(n))
    head = struct.pack("<IQffIIfIHH", pwr_ms, 0, 0.1, 1500.0, pwr_ms // 100, 240000, 1e-4, 0, n, 0)
    return pp.encode(sv.ATOF_POINT_DATA, head + pts)


def test_atof_decoding_matches_brping():
    from brping import PingMessage
    raw = _atof_frame(1234)
    ours = sv.decode_atof(list(pp.Parser().feed(raw))[0].payload)
    theirs = PingMessage(msg_data=raw)
    assert ours.pwr_up_msec == theirs.pwr_up_msec == 1234
    assert ours.sos_mps == pytest.approx(theirs.sos_mps) and len(ours.angles_rad) == theirs.num_points == 3
    y, z = ours.points_yz()[2]
    assert y == pytest.approx(0.5 * 1500 * 0.004 * __import__("math").sin(0.5)) and z < 0


def test_set_ping_parameters_matches_brping_layout():
    from brping import PingMessage
    raw = sv.set_ping_parameters(True)
    m = PingMessage(msg_data=raw)
    assert m.message_id == 3023 and m.ping_enable == 1 and m.enable_atof_data == 1
    assert len(raw) == 8 + 36 + 2 and raw[8 + 19] == 1


def test_replay_is_byte_exact_and_waits_for_ping_enable(tmp_path):
    frames = [pp.encode(sv.JSON_WRAPPER, b'{"session_id": 1}'), _atof_frame(1000),
              pp.encode(3009, bytes(range(200))), _atof_frame(1100)]     # 3009: unknown, kept raw
    log = tmp_path / "x.svlog"
    log.write_bytes(b"".join(frames))
    port = free_port()
    srv = sv.ReplayServer(log, port, speed=0)
    th = threading.Thread(target=srv.serve_one, daemon=True)
    th.start()
    c = socket.create_connection(("127.0.0.1", port))
    c.settimeout(0.5)
    with pytest.raises(socket.timeout):
        c.recv(10)                               # nothing before ping_enable
    c.sendall(sv.set_ping_parameters(True))
    got = b""
    while len(got) < len(log.read_bytes()):
        got += c.recv(65536)
    assert got == log.read_bytes() and srv.sent_frames == 4
    c.close()
    th.join(2)
    srv.close()


@pytest.mark.skipif(not REAL_SVLOG.exists(), reason="private laboratory recording not available")
def test_real_svlog_parses_and_replays_exactly(tmp_path):
    s = sv.summarize(REAL_SVLOG)
    assert s["frames_by_id"][sv.ATOF_POINT_DATA] > 1000 and s["atof_pings"] > 1000
    p = pp.Parser()
    n = 0
    with open(REAL_SVLOG, "rb") as f:
        for fr in p.feed(f.read(5_000_000)):
            if fr.message_id == sv.ATOF_POINT_DATA:
                a = sv.decode_atof(fr.payload)
                assert len(fr.payload) == 40 + 16 * len(a.angles_rad)
                n += 1
    assert p.bad_checksum == 0 and n > 0
    head = tmp_path / "head.svlog"
    head.write_bytes(open(REAL_SVLOG, "rb").read(2_000_000))
    port = free_port()
    srv = sv.ReplayServer(head, port, autostart=True, speed=0)
    th = threading.Thread(target=srv.serve_one, daemon=True)
    th.start()
    c = socket.create_connection(("127.0.0.1", port))
    expected = b"".join(fr.raw for fr in srv.frames)
    got = b""
    c.settimeout(2)
    while len(got) < len(expected):
        got += c.recv(1 << 20)
    assert got == expected
    c.close()
    th.join(2)
    srv.close()


def test_ping1d_adapter_with_official_client():
    from brping import Ping1D
    dist = {"m": 2.5}
    port = free_port(socket.SOCK_DGRAM)
    a = Ping1DAdapter(lambda: dist["m"], port).start()
    try:
        p = Ping1D()
        p.connect_udp("127.0.0.1", port)
        assert p.initialize() is True
        d = p.get_distance()
        assert d["distance"] == 2500 and d["confidence"] == 100
        dist["m"] = None                         # no echo -> invalid, never an invented value
        d = p.get_distance()
        assert d["confidence"] == 0
    finally:
        a.close()
