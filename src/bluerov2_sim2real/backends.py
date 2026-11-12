"""Physics backends returning the state needed by ArduSub SITL.

``HoloOceanBackend`` drives the stock HoloOcean 2.3.0 BlueROV2 agent (control
scheme 0) through ``HoloAllocator`` and adds the drag compensation.
``MockBackend`` is a small rigid-body integrator used to test the protocol and
the bridge logic without HoloOcean (Test B).

Both expose ``dt``, ``reset()``, ``step(wrench_frd) -> Observation``,
``hold`` (kinematic hold at a fixed pose, used during SITL boot so that the
SITL barometer is referenced at the water surface) and ``close()``.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from . import drag as dragmod
from .frames import T_FLIP, flu_to_frd, nwu_to_ned, quat_wxyz_from_matrix, rot_nwu_flu_to_ned_frd
from .thrusters import HoloAllocator

GRAVITY = 9.8   # HoloOcean world gravity (UE -980 cm/s^2), measured at rest


@dataclass
class Observation:
    t_sim: float
    position_ned: np.ndarray        # m, relative to the start x/y; z = depth of the CoM pose
    velocity_ned: np.ndarray        # m/s
    R_ned_frd: np.ndarray           # body -> earth
    gyro_frd: np.ndarray            # rad/s
    accel_frd: np.ndarray           # specific force, m/s^2
    extras: dict = field(default_factory=dict)

    @property
    def quaternion_wxyz(self) -> np.ndarray:
        return quat_wxyz_from_matrix(self.R_ned_frd)

    @property
    def depth_m(self) -> float:
        return float(self.position_ned[2])


def holo_scenario(tps: int, start_location, world: str = "SimpleUnderwater") -> dict:
    return {
        "name": "bluerov2_sim2real_phase2",
        "world": world,
        "package_name": "Ocean",
        "main_agent": "auv0",
        "ticks_per_sec": int(tps),
        "frames_per_sec": False,      # never throttled by the engine: SITL paces time
        "agents": [{
            "agent_name": "auv0",
            "agent_type": "BlueROV2",
            "sensors": [
                {"sensor_type": "PoseSensor", "socket": "COM"},
                {"sensor_type": "DynamicsSensor", "socket": "COM",
                 "configuration": {"UseCOM": True, "UseRPY": True}},
                {"sensor_type": "IMUSensor", "socket": "IMUSocket", "Hz": int(tps)},
            ],
            "control_scheme": 0,
            "location": list(start_location),
            "rotation": [0.0, 0.0, 0.0],
        }],
        "window_width": 320,
        "window_height": 240,
    }


class HoloOceanBackend:
    def __init__(self, tps: int = 200, start_location=(0.0, 0.0, 0.0), drag_compensation: bool = True,
                 world: str = "SimpleUnderwater", hold_pose: Optional[tuple] = None):
        import holoocean  # imported lazily: the mock path must not need it
        self.dt = 1.0 / tps
        self.tps = tps
        self.start_location = np.asarray(start_location, dtype=float)
        self.drag_compensation = drag_compensation
        self.allocator = HoloAllocator()
        self.hold = hold_pose is not None
        self.hold_pose = hold_pose or (tuple(start_location), (0.0, 0.0, 0.0))
        self.env = holoocean.make(scenario_cfg=holo_scenario(tps, start_location, world),
                                  show_viewport=False, verbose=False)
        self.agent = self.env.agents["auv0"]
        self.ticks = 0
        self._last_vel_world = np.zeros(3)
        self._last_R = np.eye(3)
        self._last_depth = -float(self.start_location[2])

    def close(self):
        if self.env is not None:
            self.env.__exit__(None, None, None)
            self.env = None

    def _observe(self, s, extras) -> Observation:
        pose = np.asarray(s["PoseSensor"], dtype=float)
        dyn = np.asarray(s["DynamicsSensor"], dtype=float)
        imu = np.asarray(s["IMUSensor"], dtype=float)
        if not (np.all(np.isfinite(pose)) and np.all(np.isfinite(dyn)) and np.all(np.isfinite(imu))):
            raise FloatingPointError("non-finite HoloOcean sensor output")
        R_nwu_flu = pose[0:3, 0:3]
        p_nwu = pose[0:3, 3]
        v_nwu = dyn[3:6]
        self._last_vel_world = v_nwu.copy()
        self._last_R = R_nwu_flu
        pos = nwu_to_ned(p_nwu - np.array([self.start_location[0], self.start_location[1], 0.0]))
        self._last_depth = float(pos[2])
        return Observation(
            t_sim=self.ticks * self.dt,
            position_ned=pos,
            velocity_ned=nwu_to_ned(v_nwu),
            R_ned_frd=rot_nwu_flu_to_ned_frd(R_nwu_flu),
            gyro_frd=imu[1].copy(),            # IMUSocket is FRD
            accel_frd=imu[0].copy(),           # specific force, FRD
            extras=extras,
        )

    def step(self, wrench_frd: np.ndarray, nominal: Optional[np.ndarray] = None) -> Observation:
        extras = {"held": self.hold}
        wrench = np.asarray(wrench_frd, dtype=float).copy()
        comp_world = np.zeros(3)
        if self.drag_compensation and not self.hold:
            frac = dragmod.submerged_fraction_estimate(self._last_depth)
            comp_world = dragmod.missing_drag_world(self._last_vel_world, frac)
            # world NWU force -> body FRD:  F_frd = T R_nwu_flu^T F_nwu
            wrench[0:3] += T_FLIP @ (self._last_R.T @ comp_world)
        alloc = self.allocator.allocate(wrench, nominal=nominal)
        extras.update(holo_forces=alloc.forces, saturated=alloc.saturated,
                      wrench_error=alloc.wrench_error, drag_comp_world_nwu=comp_world)
        if self.hold:
            loc, rpy = self.hold_pose
            self.agent.set_physics_state(list(loc), list(rpy), [0, 0, 0], [0, 0, 0])
            s = self.env.step(np.zeros(8))
        else:
            s = self.env.step(alloc.forces)
        self.ticks += 1
        return self._observe(s, extras)


class MockBackend:
    """Minimal 6-DoF rigid body: neutral buoyancy, linear damping, no drag defect.

    It is not a vehicle model; it only produces physically consistent states
    (position, velocity, attitude, rates, specific force) for protocol tests.
    """

    def __init__(self, tps: int = 200, mass: float = 11.5, inertia=(0.2, 0.25, 0.25),
                 lin_damping: float = 2.0, ang_damping: float = 1.5, start_depth: float = 0.0,
                 hold: bool = False):
        self.dt = 1.0 / tps
        self.tps = tps
        self.m = mass
        self.I = np.asarray(inertia, dtype=float)
        self.cl, self.ca = lin_damping, ang_damping
        self.hold = hold
        self.ticks = 0
        self.p = np.array([0.0, 0.0, start_depth])   # NED
        self.v = np.zeros(3)                          # NED
        self.R = np.eye(3)                            # NED <- FRD
        self.w = np.zeros(3)                          # FRD

    def close(self):
        pass

    def step(self, wrench_frd: np.ndarray, nominal: Optional[np.ndarray] = None) -> Observation:
        w = np.asarray(wrench_frd, dtype=float)
        F_ned = self.R @ w[0:3]
        a_ned = np.zeros(3) if self.hold else F_ned / self.m - self.cl * self.v
        alpha = np.zeros(3) if self.hold else w[3:6] / self.I - self.ca * self.w
        self.v = self.v + a_ned * self.dt
        self.p = self.p + self.v * self.dt
        self.w = self.w + alpha * self.dt
        wx, wy, wz = self.w * self.dt
        Om = np.array([[0, -wz, wy], [wz, 0, -wx], [-wy, wx, 0]])
        self.R = self.R @ (np.eye(3) + Om)
        u, _, vt = np.linalg.svd(self.R)
        self.R = u @ vt
        self.ticks += 1
        g_ned = np.array([0.0, 0.0, GRAVITY])
        accel_frd = self.R.T @ (a_ned - g_ned)
        return Observation(self.ticks * self.dt, self.p.copy(), self.v.copy(), self.R.copy(),
                           self.w.copy(), accel_frd, {"held": self.hold})
