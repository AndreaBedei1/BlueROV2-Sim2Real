import json
import socket
import threading

import numpy as np
import pytest

from bluerov2_sim2real import drag
from bluerov2_sim2real.backends import MockBackend
from bluerov2_sim2real.bridge import Bridge, BridgeConfig
from bluerov2_sim2real.sitl_json import ardusub412_parse, build_servo_packet


def test_drag_compensation_algebra():
    v = np.array([0.7, -0.2, 0.3])
    total = drag.STOCK_FACTOR * drag.intended_drag(v) + drag.missing_drag_world(v)
    assert np.allclose(total, drag.intended_drag(v))
    # magnitude check: 0.5*997*0.8*0.45*|v|^2
    assert np.linalg.norm(drag.intended_drag([1.0, 0, 0])) == pytest.approx(179.46)
    assert drag.submerged_fraction_estimate(-0.1) == 0 and drag.submerged_fraction_estimate(1.0) == 1


def _free_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def test_bridge_lockstep_with_mock(tmp_path):
    """Test B core: protocol exchange, resend handling, lost frames, monotonic time."""
    port = _free_port()
    backend = MockBackend(tps=200, start_depth=1.0)
    cfg = BridgeConfig(bind_port=port, out_dir=tmp_path, recv_timeout_s=0.5, idle_exit_s=1.0)
    bridge = Bridge(backend, cfg)
    th = threading.Thread(target=bridge.run, daemon=True)
    th.start()

    cli = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cli.settimeout(2.0)
    replies = []
    sequence = list(range(1, 101)) + [100, 100] + list(range(103, 151))   # 2 resends, 2 lost
    for fc in sequence:
        pwm = [1500] * 4 + [1400] * 4 + [0] * 8                            # verticals: command "up"
        cli.sendto(build_servo_packet(200, fc, pwm), ("127.0.0.1", port))
        replies.append(cli.recv(4096).decode())
    cli.close()
    th.join(timeout=10)
    assert not th.is_alive()

    parsed = [ardusub412_parse(r.strip()) for r in replies]
    ts = [p["timestamp"] for p in parsed]
    assert ts[100] == ts[99] == ts[101]                       # resends answered with the same state
    steps = [t for i, t in enumerate(ts) if i not in (100, 101)]
    assert np.allclose(np.diff(steps), 0.005)                 # one dt per new frame, even across the gap
    assert parsed[-1]["position"][2] < 1.0                    # thrust "up" reduced depth in the mock

    summary = json.loads((tmp_path / "summary.json").read_text())
    assert summary["duplicates_resent"] == 2
    assert summary["lost_frames"] == 2
    assert summary["physics_steps"] == 148
    assert summary["bad_packets"] == 0
    rows = (tmp_path / "frames.csv").read_text().strip().splitlines()
    assert len(rows) == 1 + 148


def test_bridge_refuses_non_local_bind():
    from bluerov2_sim2real.bridge import open_bridge_socket
    with pytest.raises(ValueError):
        open_bridge_socket("0.0.0.0", _free_port())


def _run_bridge(tmp_path, backend, frames, cfg_kw=None, other_sender_at=None):
    port = _free_port()
    cfg = BridgeConfig(bind_port=port, out_dir=tmp_path, recv_timeout_s=0.3, idle_exit_s=0.6,
                       **(cfg_kw or {}))
    bridge = Bridge(backend, cfg)
    result = {}

    def target():
        try:
            result["summary"] = bridge.run()
        except Exception as e:  # noqa: BLE001
            result["error"] = e
    th = threading.Thread(target=target, daemon=True)
    th.start()
    cli = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cli.settimeout(0.5)
    other = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    replies = []
    for k, fc in enumerate(frames):
        if other_sender_at == k:
            other.sendto(build_servo_packet(200, fc, [1900] * 16), ("127.0.0.1", port))
        data = b"garbage" if fc is None else build_servo_packet(200, fc, [1500] * 8 + [0] * 8)
        cli.sendto(data, ("127.0.0.1", port))
        try:
            replies.append(cli.recv(4096))
        except socket.timeout:
            replies.append(None)
    th.join(timeout=10)
    cli.close()
    other.close()
    assert not th.is_alive()
    return result, json.loads((tmp_path / "summary.json").read_text()), replies


def test_bridge_stops_on_sitl_reset(tmp_path):
    _, s, _ = _run_bridge(tmp_path, MockBackend(), [1, 2, 3, 4, 1])
    assert s["stop_reason"] == "sitl_reset" and s["sitl_resets"] == 1 and s["physics_steps"] == 4


def test_bridge_ignores_bad_packets_and_other_senders(tmp_path):
    _, s, replies = _run_bridge(tmp_path, MockBackend(), [1, 2, None, 3, 4], other_sender_at=3)
    assert s["bad_packets"] == 1 and s["rejected_datagrams"] == 1 and s["physics_steps"] == 4
    assert replies[2] is None


def test_bridge_release_and_max_sim(tmp_path):
    b = MockBackend(hold=True)
    _, s, _ = _run_bridge(tmp_path, b, list(range(1, 60)), {"boot_hold_s": 0.1, "max_sim_s": 0.2})
    events = [json.loads(x)["kind"] for x in (tmp_path / "events.jsonl").read_text().splitlines()]
    assert "released" in events and b.hold is False
    assert s["stop_reason"] == "max_sim_s" and s["physics_steps"] == 40


class _ExplodingBackend(MockBackend):
    def step(self, wrench, nominal=None):
        if self.ticks >= 3:
            raise FloatingPointError("boom")
        return super().step(wrench, nominal)


def test_bridge_writes_summary_on_backend_error(tmp_path):
    r, s, _ = _run_bridge(tmp_path, _ExplodingBackend(), [1, 2, 3, 4, 5])
    assert isinstance(r.get("error"), FloatingPointError)
    assert s["stop_reason"] == "error:FloatingPointError"
    assert "exception" in (tmp_path / "events.jsonl").read_text()


class _SlowBackend(MockBackend):
    """Step 5 takes longer than a SITL resend interval."""
    def step(self, wrench, nominal=None):
        if self.ticks == 4:
            import time as _t
            _t.sleep(0.6)
        return super().step(wrench, nominal)


def test_resend_received_during_a_slow_step_is_dropped(tmp_path):
    """SITL-like client: resends frame 5 while the bridge is still stepping; must get ONE reply."""
    port = _free_port()
    cfg = BridgeConfig(bind_port=port, out_dir=tmp_path, recv_timeout_s=0.3, idle_exit_s=0.6)
    bridge = Bridge(_SlowBackend(), cfg)
    th = threading.Thread(target=bridge.run, daemon=True)
    th.start()
    cli = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cli.settimeout(1.5)
    for fc in range(1, 5):
        cli.sendto(build_servo_packet(200, fc, [1500] * 16), ("127.0.0.1", port))
        cli.recv(4096)
    cli.sendto(build_servo_packet(200, 5, [1500] * 16), ("127.0.0.1", port))
    import time as _t
    _t.sleep(0.2)
    cli.sendto(build_servo_packet(200, 5, [1500] * 16), ("127.0.0.1", port))   # resend while busy
    first = cli.recv(4096)
    cli.settimeout(0.4)
    with pytest.raises(socket.timeout):
        cli.recv(4096)                                       # no second, stale reply queued
    cli.sendto(build_servo_packet(200, 6, [1500] * 16), ("127.0.0.1", port))
    cli.settimeout(1.5)
    nxt = ardusub412_parse(cli.recv(4096).decode().strip())
    assert nxt["timestamp"] == pytest.approx(ardusub412_parse(first.decode().strip())["timestamp"] + 0.005)
    cli.close()
    th.join(timeout=10)
    s = json.loads((tmp_path / "summary.json").read_text())
    assert s["stale_duplicates_dropped"] == 1 and s["duplicates_resent"] == 0
