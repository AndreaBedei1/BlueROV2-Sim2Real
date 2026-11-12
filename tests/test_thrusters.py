import numpy as np
import pytest

from bluerov2_sim2real.thrusters import (
    ARDUSUB_412_VECTORED_6DOF, HOLO_MAX_THRUST_N, HoloAllocator, LogicalVehicle,
    MOTOR_TO_HOLO_POSITION, ThrusterModel, holo_geometry,
)

AXES = ["roll", "pitch", "yaw", "throttle", "forward", "lateral"]


def contribution(B, m):
    """Body wrench of motor m expressed on the ArduSub mixing axes."""
    Fx, Fy, Fz, Mx, My, Mz = B[:, m - 1]
    return {"roll": Mx, "pitch": My, "yaw": Mz, "throttle": -Fz, "forward": Fx, "lateral": Fy}


@pytest.mark.parametrize("angle", [30.0, 45.0])
def test_logical_signs_match_ardusub_mixing(angle):
    v = LogicalVehicle(horizontal_angle_deg=angle)
    for m, factors in ARDUSUB_412_VECTORED_6DOF.items():
        c = contribution(v.B, m)
        for axis, f in zip(AXES, factors):
            if f != 0:
                assert np.sign(c[axis]) == np.sign(f), (m, axis)
            else:
                assert abs(c[axis]) < 1e-12, (m, axis)


def test_horizontal_angle_is_30_deg():
    v = LogicalVehicle()
    for m in range(1, 5):
        d = v.directions[m - 1]
        assert np.degrees(np.arctan2(abs(d[1]), abs(d[0]))) == pytest.approx(30.0)
        assert np.linalg.norm(d) == pytest.approx(1.0)


def test_holo_geometry_matches_characterized_binary():
    # scripts/characterize_holoocean.py: index 0-3 push up (client +z), 4/6 forward-left, 5/7 forward-right
    pos, dirs = holo_geometry()
    for i in range(4):
        assert np.allclose(dirs[i], [0, 0, -1])
    s = 1 / np.sqrt(2)
    assert np.allclose(dirs[4], [s, -s, 0]) and np.allclose(dirs[6], [s, -s, 0])
    assert np.allclose(dirs[5], [s, s, 0]) and np.allclose(dirs[7], [s, s, 0])
    # index 4 is front starboard, index 0 front starboard vertical (FRD y > 0)
    assert pos[4][0] > 0 and pos[4][1] > 0 and pos[0][0] > 0 and pos[0][1] > 0


def test_motor_positions_follow_vectored_6dof_layout():
    v = LogicalVehicle()
    expected = {1: (+1, +1), 2: (+1, -1), 3: (-1, +1), 4: (-1, -1),
                5: (+1, +1), 6: (+1, -1), 7: (-1, +1), 8: (-1, -1)}   # (front?, starboard?)
    for m, (sx, sy) in expected.items():
        x, y, _ = v.positions[m - 1]
        assert np.sign(x) == sx and np.sign(y) == sy, m


def test_pwm_mapping():
    t = ThrusterModel()
    assert t.pwm_to_command(1500) == 0
    assert t.pwm_to_command(1900) == pytest.approx(1)
    assert t.pwm_to_command(1100) == pytest.approx(-1)
    assert t.pwm_to_command(2100) == pytest.approx(1)
    assert t.pwm_to_command(0) == 0                  # channel not output -> unpowered
    assert t.command_to_thrust(1.0) == pytest.approx(HOLO_MAX_THRUST_N)          # never above the engine clamp
    assert t.command_to_thrust(-1.0) == pytest.approx(-HOLO_MAX_THRUST_N * 39.9 / 51.4)


def test_full_vertical_command_never_saturates():
    v, a = LogicalVehicle(), HoloAllocator()
    for p in (1100, 1900):
        thr = v.pwm_to_thrusts([1500] * 4 + [p] * 4 + [0] * 8)
        assert not a.allocate(v.wrench(thr), nominal=v.nominal_holo_forces(thr)).saturated


def test_neutral_pwm_gives_zero_wrench():
    v = LogicalVehicle()
    assert np.allclose(v.wrench(v.pwm_to_thrusts([1500] * 8 + [0] * 8)), 0)
    assert np.allclose(v.wrench(v.pwm_to_thrusts([0] * 16)), 0)


def test_allocation_exact_when_unsaturated():
    a = HoloAllocator()
    rng = np.random.default_rng(0)
    for _ in range(200):
        w = rng.normal(scale=[8, 8, 8, 1, 1, 1])
        r = a.allocate(w)
        if not r.saturated:
            assert np.allclose(r.wrench_error, 0, atol=1e-9)


def test_allocation_flags_saturation():
    r = HoloAllocator().allocate(np.array([200.0, 0, 0, 0, 0, 0]))
    assert r.saturated and np.all(np.abs(r.forces) <= HOLO_MAX_THRUST_N + 1e-12)
    assert abs(r.wrench_error[0]) > 1


def test_vertical_commands_map_one_to_one():
    # verticals have identical geometry in both layouts: allocation = direct index mapping
    v, a = LogicalVehicle(), HoloAllocator()
    pwm = [1500] * 4 + [1450, 1600, 1520, 1380] + [0] * 8
    thr = v.pwm_to_thrusts(pwm)
    r = a.allocate(v.wrench(thr), nominal=v.nominal_holo_forces(thr))
    for m in range(5, 9):
        # logical verticals push down for a positive command, HoloOcean ones push up
        assert r.forces[MOTOR_TO_HOLO_POSITION[m]] == pytest.approx(-thr[m - 1], abs=1e-9)
    assert np.allclose(r.forces[4:8], 0, atol=1e-9)
    assert np.allclose(r.wrench_error, 0, atol=1e-9)


def test_allocation_with_nominal_is_exact_for_random_pwm():
    v, a = LogicalVehicle(), HoloAllocator()
    rng = np.random.default_rng(3)
    for _ in range(200):
        pwm = list(rng.integers(1380, 1620, size=8)) + [0] * 8
        thr = v.pwm_to_thrusts(pwm)
        r = a.allocate(v.wrench(thr), nominal=v.nominal_holo_forces(thr))
        if not r.saturated:
            assert np.allclose(r.wrench_error, 0, atol=1e-9)


def test_30deg_differs_from_stock_45deg_one_to_one():
    # pure surge request: a 30 deg vehicle produces more surge than the stock 45 deg layout
    v30, v45 = LogicalVehicle(30.0), LogicalVehicle(45.0)
    pwm = [1400, 1400, 1600, 1600] + [1500] * 4 + [0] * 8        # forward: M1,M2 factor -1, M3,M4 +1
    assert v30.wrench(v30.pwm_to_thrusts(pwm))[0] > v45.wrench(v45.pwm_to_thrusts(pwm))[0] > 0
