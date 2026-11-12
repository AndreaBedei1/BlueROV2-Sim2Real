"""Thruster model: ArduSub PWM -> per-motor thrust -> body wrench -> HoloOcean.

Two different eight-thruster layouts are involved and must not be confused:

1. The *logical simulated vehicle* commanded by ArduSub (``LOGICAL``):
   ArduSub 4.1.2 ``VECTORED_6DOF`` motors 1..8 (mixing factors below, copied
   from ``AP_Motors6DOF.cpp`` at tag ArduSub-4.1.2), with the four horizontal
   thrusters at the **30 deg** mounting angle verified on the laboratory
   vehicle. In this simulation a positive motor command produces force along
   the direction that makes each motor's contribution match the sign of its
   ArduSub mixing factors, with ``MOT_n_DIRECTION = 1`` for every motor.
   This is a *logical* correspondence. It does **not** describe the physical
   wiring, propeller handedness or ESC direction of the real vehicle, where
   ``MOT_1,2,5,8_DIRECTION = -1`` compensate for propeller handedness and the
   wiring is still unverified.

2. The *HoloOcean BlueROV2 agent* (``HOLO``), whose geometry is compiled into
   the engine (``BlueROV2.h/.cpp``, v2.3.0): verticals 0..3, horizontals 4..7
   fixed at 45 deg, each force clamped to +-28.75 N. The geometry below was
   checked against the binary by ``scripts/characterize_holoocean.py``.

Because the HoloOcean horizontals cannot be set to 30 deg without rebuilding
the engine, the bridge computes the 6-DoF body wrench of the logical vehicle
and allocates it to the eight HoloOcean thrusters (exact for unsaturated
commands, rank 6). Thruster lever arms of the real vehicle are not measured;
the logical vehicle provisionally reuses the HoloOcean thruster positions.

All vectors here are body FRD (x forward, y right, z down), metres, newtons,
about the centre of mass.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional, Sequence

import numpy as np

# ArduSub 4.1.2 VECTORED_6DOF: motor -> (roll, pitch, yaw, throttle, forward, lateral)
ARDUSUB_412_VECTORED_6DOF = {
    1: (0.0, 0.0, 1.0, 0.0, -1.0, 1.0),
    2: (0.0, 0.0, -1.0, 0.0, -1.0, -1.0),
    3: (0.0, 0.0, -1.0, 0.0, 1.0, 1.0),
    4: (0.0, 0.0, 1.0, 0.0, 1.0, -1.0),
    5: (1.0, -1.0, 0.0, -1.0, 0.0, 0.0),
    6: (-1.0, -1.0, 0.0, -1.0, 0.0, 0.0),
    7: (1.0, 1.0, 0.0, -1.0, 0.0, 0.0),
    8: (-1.0, 1.0, 0.0, -1.0, 0.0, 0.0),
}

PWM_NEUTRAL = 1500
PWM_HALF_RANGE = 400          # ArduSub: pwm = 1500 + thrust * 400 (AP_Motors6DOF::calc_thrust_to_pwm)

# HoloOcean BlueROV2 (engine v2.3.0, BlueROV2.h): UE cm, left-handed x fwd, y right, z up
HOLO_THRUSTER_LOCATIONS_UE_CM = np.array([
    [12.00, 21.81, 7.09], [12.00, -21.81, 7.09], [-12.00, -21.81, 7.09], [-12.00, 21.81, 7.09],
    [15.62, 9.88, -1.00], [15.62, -9.88, -1.00], [-15.62, -9.88, -1.00], [-15.62, 9.88, -1.00],
])
HOLO_MAX_THRUST_N = 10.0 * 11.5 / 4.0   # BR_MAX_THRUST = 28.75 N


def _holo_com_ue() -> np.ndarray:
    """BlueROV2::InitializeAgent with Perfect=true."""
    com = (HOLO_THRUSTER_LOCATIONS_UE_CM[0] + HOLO_THRUSTER_LOCATIONS_UE_CM[2]) / 2.0
    com[2] = HOLO_THRUSTER_LOCATIONS_UE_CM[7][2]
    return com


def _ue_cm_to_frd_m(v) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    return np.array([v[0], v[1], -v[2]]) / 100.0   # UE y is already "right"; z up -> down


def holo_geometry():
    """Positions (8x3) and unit force directions (8x3) of HoloOcean thrusters, FRD."""
    com = _holo_com_ue()
    pos = np.array([_ue_cm_to_frd_m(p - com) for p in HOLO_THRUSTER_LOCATIONS_UE_CM])
    s = 1.0 / np.sqrt(2.0)
    dirs = np.zeros((8, 3))
    dirs[0:4] = [0.0, 0.0, -1.0]                 # client +z (up) -> FRD -z
    for i in range(4, 8):
        # client FLU: even (F/sqrt2, +F/sqrt2, 0) forward-left; odd (F/sqrt2, -F/sqrt2, 0)
        dirs[i] = [s, -s, 0.0] if i % 2 == 0 else [s, s, 0.0]
    return pos, dirs


# ArduSub motor -> HoloOcean thruster index at the same nominal position
MOTOR_TO_HOLO_POSITION = {1: 4, 2: 5, 3: 7, 4: 6, 5: 0, 6: 1, 7: 3, 8: 2}


def wrench_matrix(pos: np.ndarray, dirs: np.ndarray) -> np.ndarray:
    """6xN matrix mapping per-thruster force (N) to body wrench [F; M] (FRD, about CoM)."""
    B = np.zeros((6, len(pos)))
    for i, (r, d) in enumerate(zip(pos, dirs)):
        B[0:3, i] = d
        B[3:6, i] = np.cross(r, d)
    return B


@dataclass
class ThrusterModel:
    """PWM -> thrust for one motor. Provisional, uncalibrated (Phase 4 calibrates).

    Linear in the normalised command u = (pwm - 1500) / 400, as ArduSub 4.1.2
    assumes when it maps thrust to PWM; no dead band, no dynamics, no voltage
    dependence. The Blue Robotics T200 at 16 V gives 51.4 N forward / 39.9 N
    reverse (Phase 1), but the HoloOcean agent clamps every thruster to
    28.75 N (BR_MAX_THRUST, compiled in the engine). Using the T200 values
    would make the bridge believe in forces that are silently clipped (seen in
    the first Test E run: 25 % of steps saturated). The T200 forward/reverse
    ratio is therefore kept and scaled to the simulator limit:
    28.75 N forward, 28.75 * 39.9 / 51.4 = 22.32 N reverse. This
    underestimates the real thrust authority (by ~0.56 at full command; the
    real T200 curve is nonlinear with a dead band, so the ratio varies).
    Positive command is assumed to be the strong (forward) side of every
    motor, an assumption about the logical vehicle only.

    Even with this scaling the 45 deg HoloOcean horizontals cannot deliver
    every 30 deg request (full logical surge 4*28.75*cos30 = 99.6 N vs
    4*28.75*cos45 = 81.3 N available); the allocator then reports saturation.

    PWM values outside [1100, 1900] us (ArduSub's MOT_PWM_MIN/MAX on the
    vehicle; SITL emits 1000 us on motor outputs before the motors are
    enabled) are treated as "no valid signal" -> unpowered, as an ESC would.
    """
    pwm_min: int = 1100
    pwm_max: int = 1900
    max_forward_n: float = HOLO_MAX_THRUST_N
    max_reverse_n: float = HOLO_MAX_THRUST_N * 39.9 / 51.4

    def pwm_to_command(self, pwm: float) -> float:
        if pwm < self.pwm_min or pwm > self.pwm_max:   # 0 = not output; 1000 = motors not enabled
            return 0.0
        return float(np.clip((pwm - PWM_NEUTRAL) / PWM_HALF_RANGE, -1.0, 1.0))

    def command_to_thrust(self, u: float) -> float:
        return u * (self.max_forward_n if u >= 0 else self.max_reverse_n)


@dataclass
class LogicalVehicle:
    horizontal_angle_deg: float = 30.0     # from the longitudinal (surge) axis; see docs
    thruster: ThrusterModel = field(default_factory=ThrusterModel)
    motor_channel: Dict[int, int] = field(default_factory=lambda: {m: m for m in range(1, 9)})

    def __post_init__(self):
        hpos, _ = holo_geometry()
        th = np.radians(self.horizontal_angle_deg)
        pos, dirs = np.zeros((8, 3)), np.zeros((8, 3))
        for m in range(1, 9):
            roll, pitch, yaw, thr, fwd, lat = ARDUSUB_412_VECTORED_6DOF[m]
            pos[m - 1] = hpos[MOTOR_TO_HOLO_POSITION[m]]
            if m <= 4:
                dirs[m - 1] = [fwd * np.cos(th), lat * np.sin(th), 0.0]
            else:
                dirs[m - 1] = [0.0, 0.0, -thr]   # throttle factor > 0 would mean "up" (-z FRD)
        self.positions, self.directions = pos, dirs
        self.B = wrench_matrix(pos, dirs)

    def pwm_to_thrusts(self, pwm16: Sequence[int]) -> np.ndarray:
        """Per-motor thrust (N), motors 1..8, from the 16 SITL output channels."""
        out = np.zeros(8)
        for m in range(1, 9):
            ch = self.motor_channel[m]
            out[m - 1] = self.thruster.command_to_thrust(self.thruster.pwm_to_command(pwm16[ch - 1]))
        return out

    def commands(self, pwm16: Sequence[int]) -> np.ndarray:
        return np.array([self.thruster.pwm_to_command(pwm16[self.motor_channel[m] - 1]) for m in range(1, 9)])

    def wrench(self, thrusts: np.ndarray) -> np.ndarray:
        return self.B @ thrusts

    def nominal_holo_forces(self, thrusts: np.ndarray) -> np.ndarray:
        """Per-index mapping motor m -> HoloOcean thruster at the same position.

        The sign accounts for the opposite positive directions of the two
        layouts (e.g. logical verticals push down for a positive command,
        HoloOcean verticals push up). Exact for the verticals; for the
        horizontals it ignores the 30/45 deg difference, which the allocator
        then corrects.
        """
        _, hdirs = holo_geometry()
        f0 = np.zeros(8)
        for m in range(1, 9):
            j = MOTOR_TO_HOLO_POSITION[m]
            f0[j] = np.sign(np.dot(self.directions[m - 1], hdirs[j])) * thrusts[m - 1]
        return f0


@dataclass
class AllocationResult:
    forces: np.ndarray          # 8 HoloOcean thruster forces actually sent (N)
    requested: np.ndarray       # forces before clamping
    saturated: bool
    wrench_error: np.ndarray    # achieved - desired wrench


class HoloAllocator:
    """Allocate a desired body wrench to the eight HoloOcean thrusters."""

    def __init__(self, max_thrust_n: float = HOLO_MAX_THRUST_N):
        self.pos, self.dirs = holo_geometry()
        self.B = wrench_matrix(self.pos, self.dirs)
        if np.linalg.matrix_rank(self.B) != 6:
            raise RuntimeError("HoloOcean thruster layout is not full rank")
        self.B_pinv = np.linalg.pinv(self.B)
        self.max_thrust = max_thrust_n

    def allocate(self, wrench: np.ndarray, nominal: Optional[np.ndarray] = None) -> AllocationResult:
        """Exact allocation closest to ``nominal`` (least-squares correction).

        ``f = f0 + pinv(B) (w - B f0)`` reproduces the wrench exactly (B has
        rank 6) and, among all exact solutions, is the one closest to the
        nominal per-index mapping ``f0``; with ``f0 = 0`` it is the
        minimum-norm solution. The four verticals have one null-space
        direction and the four horizontals another, so a nominal is needed to
        keep the vertical forces one-to-one with ArduSub motors 5..8.
        """
        f0 = np.zeros(8) if nominal is None else np.asarray(nominal, dtype=float)
        req = f0 + self.B_pinv @ (wrench - self.B @ f0)
        forces = req.copy()
        saturated = False
        for idx, n in self._groups:
            g = forces[idx]
            if np.max(np.abs(g)) <= self.max_thrust + 1e-9:
                continue
            # 1) move along the group's null space to minimise the largest force (wrench unchanged)
            g = g + _minmax_shift(g, n) * n
            # 2) if still above the clamp, scale the whole group: its partial wrench keeps its
            #    direction (verticals act only on Fz, Mx, My; horizontals only on Fx, Fy, Mz)
            peak = np.max(np.abs(g))
            if peak > self.max_thrust + 1e-9:
                g = g * (self.max_thrust / peak)
                saturated = True
            forces[idx] = g
        return AllocationResult(
            forces=forces, requested=req, saturated=saturated,
            wrench_error=self.B @ forces - wrench,
        )

    @property
    def _groups(self):
        """(indices, unit null-space vector) of the vertical and horizontal thruster groups."""
        if not hasattr(self, "_groups_cache"):
            groups = []
            for idx in (np.arange(0, 4), np.arange(4, 8)):
                _, s, vt = np.linalg.svd(self.B[:, idx])
                n = vt[-1]
                assert s[-1] < 1e-9 * s[0] or len(s) < 4
                assert np.allclose(self.B[:, idx] @ n, 0, atol=1e-9)
                groups.append((idx, n / np.linalg.norm(n)))
            self._groups_cache = groups
        return self._groups_cache


def _minmax_shift(f: np.ndarray, n: np.ndarray) -> float:
    """argmin_s max_i |f_i + s n_i| (convex, piecewise linear: optimum at a line crossing)."""
    lines = [(a, b) for fi, ni in zip(f, n) for a, b in ((ni, fi), (-ni, -fi))]
    cands = [0.0]
    for i in range(len(lines)):
        for j in range(i + 1, len(lines)):
            (a1, b1), (a2, b2) = lines[i], lines[j]
            if abs(a1 - a2) > 1e-12:
                cands.append((b2 - b1) / (a1 - a2))
    return float(min(cands, key=lambda s: np.max(np.abs(f + s * n))))
