"""Physics backends returning the state needed by ArduSub SITL.

``HoloOceanBackend`` drives the stock HoloOcean 2.3.0 BlueROV2 agent (control
scheme 0) through ``HoloAllocator`` and adds the drag compensation.
``MockBackend`` is a small rigid-body integrator used to test the protocol and
the bridge logic without HoloOcean (Test B).

Both expose ``dt``, ``ticks``, ``step(wrench_frd, nominal) -> Observation``,
``hold`` / ``release()`` (kinematic hold used during SITL boot, see
``HoldSchedule``) and ``close()``.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from . import drag as dragmod
from .frames import T_FLIP, nwu_to_ned, quat_wxyz_from_matrix, rot_nwu_flu_to_ned_frd
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


def holo_scenario(tps: int, start_location, world: str = "SimpleUnderwater", extra_sensors=()) -> dict:
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
            ] + list(extra_sensors),
            "control_scheme": 0,
            "location": list(start_location),
            "rotation": [0.0, 0.0, 0.0],
        }],
        "window_width": 320,
        "window_height": 240,
    }


@dataclass
class HoldSchedule:
    """Kinematic hold used while SITL boots (sim seconds from start).

    0 .. surface_s: CoM at the water surface, so that the SITL barometer is
    referenced at the surface, as on the real vehicle before a dive.
    Then a slow kinematic descent at ``descent_rate`` to ``depth`` and a hold
    there until the bridge releases the vehicle: the run starts fully
    submerged and at rest (a defined state) instead of from a half-submerged
    body sinking under the engine's partial-buoyancy model.
    """
    surface_s: float = 8.0
    descent_rate: float = 0.1     # m/s
    depth: float = 1.0            # m

    def at(self, t: float):
        """(depth, vertical speed down) of the held CoM at sim time t."""
        if t <= self.surface_s:
            return 0.0, 0.0
        d = (t - self.surface_s) * self.descent_rate
        if d >= self.depth:
            return self.depth, 0.0
        return d, self.descent_rate


class HoloOceanBackend:
    def __init__(self, tps: int = 200, start_location=(0.0, 0.0, 0.0), drag_compensation: bool = True,
                 world: str = "SimpleUnderwater", hold: Optional[HoldSchedule] = None, holoocean_module=None,
                 sensor_hub=None, extra_sensors=()):
        if holoocean_module is None:
            import holoocean as holoocean_module  # lazy: the mock path must not need it
        self.dt = 1.0 / tps
        self.tps = tps
        self.start_location = np.asarray(start_location, dtype=float)
        self.drag_compensation = drag_compensation
        self.allocator = HoloAllocator()
        self.schedule = hold
        self.hold = hold is not None
        self.sensor_hub = sensor_hub
        self._tilt_applied = None
        self.env = holoocean_module.make(scenario_cfg=holo_scenario(tps, start_location, world, extra_sensors),
                                         show_viewport=False, verbose=False)
        self.agent = self.env.agents["auv0"]
        self.ticks = 0
        self._last_vel_world = np.zeros(3)
        self._last_R = np.eye(3)
        self._last_depth = -float(self.start_location[2])

    @property
    def engine_pid(self) -> Optional[int]:
        proc = getattr(self.env, "_world_process", None)
        return getattr(proc, "pid", None)

    def release(self):
        self.hold = False

    def on_servo(self, pwm16):
        """Accessory outputs (Phase 3): only the camera tilt has a simulated effect."""
        if self.sensor_hub is None:
            return
        tilt = self.sensor_hub.tilt_from_pwm(pwm16)
        if tilt is not None and (self._tilt_applied is None or abs(tilt - self._tilt_applied) > 0.5):
            cam = self.agent.sensors.get("RGBCamera")
            if cam is not None:
                cam.rotate([0.0, -tilt, 0.0])     # sign convention checked in docs/phase3 (camera run)
                self._tilt_applied = tilt
                self.sensor_hub.tilt_deg = tilt

    def close(self):
        if self.env is not None:
            self.env.__exit__(None, None, None)
            self.env = None

    def _observe(self, s, extras, held_vel_down: Optional[float] = None) -> Observation:
        pose = np.asarray(s["PoseSensor"], dtype=float)
        dyn = np.asarray(s["DynamicsSensor"], dtype=float)
        imu = np.asarray(s["IMUSensor"], dtype=float)
        if not (np.all(np.isfinite(pose)) and np.all(np.isfinite(dyn)) and np.all(np.isfinite(imu))):
            raise FloatingPointError("non-finite HoloOcean sensor output")
        R_nwu_flu = pose[0:3, 0:3]
        p_nwu = pose[0:3, 3]
        a_nwu = dyn[0:3]                  # DynamicsSensor at COM: world, m/s^2, no gravity
        v_nwu = dyn[3:6]
        gyro = imu[1].copy()              # IMUSocket is FRD; angular rate is the same at any point
        if held_vel_down is not None:     # kinematic hold: report the imposed motion, not solver residue
            v_nwu = np.array([0.0, 0.0, -held_vel_down])
            a_nwu = np.zeros(3)
            gyro = np.zeros(3)
        self._last_vel_world = v_nwu.copy()
        self._last_R = R_nwu_flu
        pos = nwu_to_ned(p_nwu - np.array([self.start_location[0], self.start_location[1], 0.0]))
        self._last_depth = float(pos[2])
        # specific force at the CoM, body FRD: f = T R^T (a - g),  g_NWU = (0, 0, -9.8)
        f_frd = T_FLIP @ (R_nwu_flu.T @ (a_nwu - np.array([0.0, 0.0, -GRAVITY])))
        extras["imu_socket_accel_frd"] = imu[0].copy()
        return Observation(
            t_sim=self.ticks * self.dt,
            position_ned=pos,
            velocity_ned=nwu_to_ned(v_nwu),
            R_ned_frd=rot_nwu_flu_to_ned_frd(R_nwu_flu),
            gyro_frd=gyro,
            accel_frd=f_frd,
            extras=extras,
        )

    def drag_compensation_body(self) -> np.ndarray:
        """Missing drag (body FRD force) for the current step, from the last observed state."""
        frac = dragmod.submerged_fraction_estimate(self._last_depth)
        comp_world = dragmod.missing_drag_world(self._last_vel_world, frac)
        # world NWU force -> body FRD:  F_frd = T R_nwu_flu^T F_nwu
        return T_FLIP @ (self._last_R.T @ comp_world), comp_world

    def step(self, wrench_frd: np.ndarray, nominal: Optional[np.ndarray] = None) -> Observation:
        extras = {"held": self.hold}
        wrench = np.asarray(wrench_frd, dtype=float).copy()
        comp_world = np.zeros(3)
        if self.drag_compensation and not self.hold:
            comp_body, comp_world = self.drag_compensation_body()
            wrench[0:3] += comp_body
        if self.hold:
            depth, vdown = self.schedule.at((self.ticks + 1) * self.dt)
            loc = [self.start_location[0], self.start_location[1], -depth]
            self.agent.set_physics_state(loc, [0, 0, 0], [0, 0, -vdown], [0, 0, 0])
            s = self.env.step(np.zeros(8))
            extras.update(holo_forces=np.zeros(8), saturated=False, wrench_error=np.zeros(6),
                          drag_comp_world_nwu=comp_world)
            self.ticks += 1
            return self._observe(s, extras, held_vel_down=vdown)
        alloc = self.allocator.allocate(wrench, nominal=nominal)
        extras.update(holo_forces=alloc.forces, saturated=alloc.saturated,
                      wrench_error=alloc.wrench_error, drag_comp_world_nwu=comp_world)
        s = self.env.step(alloc.forces)
        self.ticks += 1
        obs = self._observe(s, extras)
        if self.sensor_hub is not None:
            self.sensor_hub.on_state(s)
        return obs


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

    def release(self):
        self.hold = False

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
