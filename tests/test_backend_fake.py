"""HoloOceanBackend logic against a fake holoocean module (no engine started)."""
import numpy as np
import pytest

from bluerov2_sim2real import drag
from bluerov2_sim2real.backends import GRAVITY, HoldSchedule, HoloOceanBackend


def rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


class FakeAgent:
    def __init__(self):
        self.calls = []

    def set_physics_state(self, loc, rot, vel, angvel):
        self.calls.append((list(loc), list(rot), list(vel), list(angvel)))


class FakeEnv:
    """Returns a programmable sensor state; records the forces it receives."""

    def __init__(self):
        self.agents = {"auv0": FakeAgent()}
        self.R = np.eye(3)
        self.p = np.array([0.0, 0.0, -2.0])
        self.v = np.zeros(3)
        self.a = np.zeros(3)
        self.gyro = np.array([0.01, -0.02, 0.03])
        self.imu_accel = np.array([9.0, 9.0, 9.0])     # deliberately different from the CoM value
        self.forces = []

    def step(self, forces):
        self.forces.append(np.asarray(forces, dtype=float))
        pose = np.eye(4)
        pose[0:3, 0:3], pose[0:3, 3] = self.R, self.p
        dyn = np.concatenate([self.a, self.v, self.p, np.zeros(3), np.zeros(3), np.zeros(3)])
        return {"PoseSensor": pose, "DynamicsSensor": dyn, "IMUSensor": np.vstack([self.imu_accel, self.gyro])}

    def __exit__(self, *a):
        pass


class FakeHolo:
    def __init__(self):
        self.env = FakeEnv()

    def make(self, **kw):
        self.cfg = kw["scenario_cfg"]
        return self.env


def backend(**kw):
    fh = FakeHolo()
    return HoloOceanBackend(tps=200, holoocean_module=fh, **kw), fh.env, fh


def test_scenario_is_stock_bluerov2_scheme0_unthrottled():
    _, _, fh = backend()
    a = fh.cfg["agents"][0]
    assert a["agent_type"] == "BlueROV2" and a["control_scheme"] == 0
    assert fh.cfg["ticks_per_sec"] == 200 and fh.cfg["frames_per_sec"] is False


def test_observation_frames_and_specific_force_at_com():
    b, env, _ = backend(drag_compensation=False)
    obs = b.step(np.zeros(6))
    assert obs.position_ned[2] == pytest.approx(2.0)                    # 2 m below the surface
    assert np.allclose(obs.accel_frd, [0, 0, -GRAVITY])                 # at rest, from the CoM (not the socket)
    assert np.allclose(obs.gyro_frd, env.gyro)                          # gyro = IMU row 1
    assert np.allclose(obs.extras["imu_socket_accel_frd"], env.imu_accel)
    env.a = np.array([0.0, 0.0, 1.0])                                    # accelerating up (NWU)
    obs = b.step(np.zeros(6))
    assert obs.accel_frd[2] == pytest.approx(-GRAVITY - 1.0)


def test_drag_compensation_opposes_motion_in_body_frame():
    b, env, _ = backend(drag_compensation=True)
    env.R = rz(np.radians(90))                     # yawed to port (NWU), heading = world +y
    env.v = np.array([0.0, 1.0, 0.0])              # moving forward along its own nose, 1 m/s
    b.step(np.zeros(6))                            # observe this state
    comp_body, comp_world = b.drag_compensation_body()
    assert np.allclose(comp_world, drag.missing_drag_world(env.v))
    # in body FRD the motion is +x, so the compensation must be -x with 0.99 of 179.46 N
    assert comp_body[0] == pytest.approx(-0.99 * 179.4600, rel=1e-4)
    assert abs(comp_body[1]) < 1e-9 and abs(comp_body[2]) < 1e-9


def test_hold_schedule_surface_then_descend_then_release():
    sch = HoldSchedule(surface_s=1.0, descent_rate=0.5, depth=1.0)
    b, env, _ = backend(hold=sch)
    env.v = np.array([0.0, 0.0, 0.3])              # solver residue must not be reported while held
    obs = None
    for _ in range(200):                           # 1 s at the surface
        obs = b.step(np.array([0, 0, 50.0, 0, 0, 0]))
    assert np.allclose(obs.velocity_ned, 0) and np.allclose(obs.gyro_frd, 0)
    assert env.agents["auv0"].calls[-1][0][2] == pytest.approx(0.0)
    for _ in range(200):                           # descending at 0.5 m/s
        obs = b.step(np.zeros(6))
    assert obs.velocity_ned[2] == pytest.approx(0.5)
    assert all(np.allclose(f, 0) for f in env.forces)                    # no thrust while held
    for _ in range(400):
        b.step(np.zeros(6))
    assert env.agents["auv0"].calls[-1][0][2] == pytest.approx(-1.0)     # held at 1 m
    b.release()
    n = len(env.agents["auv0"].calls)
    b.step(np.array([0, 0, 40.0, 0, 0, 0]))
    assert len(env.agents["auv0"].calls) == n                            # no more kinematic overrides
    assert np.any(env.forces[-1] != 0)


def test_non_finite_sensor_raises():
    b, env, _ = backend()
    env.v = np.array([np.nan, 0, 0])
    with pytest.raises(FloatingPointError):
        b.step(np.zeros(6))
