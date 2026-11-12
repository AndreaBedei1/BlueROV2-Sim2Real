import json
import socket
import threading

import numpy as np
import pytest

from bluerov2_sim2real import drag
from bluerov2_sim2real.backends import MockBackend
from bluerov2_sim2real.bridge import Bridge, BridgeConfig
from bluerov2_sim2real.sitl_json import ardusub412_parse, build_servo_packet


def test_drag_compensation_completes_the_intended_force():
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


def test_bridge_refuses_non_local_bind(tmp_path):
    with pytest.raises(ValueError):
        Bridge(MockBackend(), BridgeConfig(bind_host="0.0.0.0", out_dir=tmp_path))
