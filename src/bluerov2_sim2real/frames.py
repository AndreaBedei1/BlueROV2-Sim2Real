"""Reference-frame conventions.

HoloOcean client (docs ``units-and-coordinates`` and engine source
``Conversion.cpp``/``PoseSensor.cpp``): metres, right-handed; world x
forward/"north", y left, z up (NWU); body x forward, y left, z up (FLU).
The water surface is at world z = SurfaceLevel (0 by default).

ArduPilot SITL JSON: earth NED, body FRD. Both changes of basis are the
same constant reflection T = diag(1, -1, -1), so

    v_NED = T v_NWU,   v_FRD = T v_FLU,   R_NED<-FRD = T R_NWU<-FLU T.

The HoloOcean ``IMUSocket`` of the BlueROV2 is documented as rotated 180 deg
about x (FRD). Evidence: at rest its accelerometer reads (0, 0, -9.8)
(characterisation), and the gyro x/y/z signs under single-thruster tests
match the expected roll/pitch/yaw (Test C). The bridge takes the gyro from
this socket and computes the specific force at the CoM.
"""
from __future__ import annotations

import numpy as np

T_FLIP = np.diag([1.0, -1.0, -1.0])


def nwu_to_ned(v) -> np.ndarray:
    return T_FLIP @ np.asarray(v, dtype=float)


def flu_to_frd(v) -> np.ndarray:
    return T_FLIP @ np.asarray(v, dtype=float)


def rot_nwu_flu_to_ned_frd(R_nwu_flu) -> np.ndarray:
    R = np.asarray(R_nwu_flu, dtype=float)
    return T_FLIP @ R @ T_FLIP


def quat_wxyz_from_matrix(R) -> np.ndarray:
    """Unit quaternion (w, x, y, z) of a rotation matrix, w >= 0."""
    R = np.asarray(R, dtype=float)
    tr = np.trace(R)
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        q = np.array([0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        q = np.array([(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s])
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        q = np.array([(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s])
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        q = np.array([(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s])
    q /= np.linalg.norm(q)
    return q if q[0] >= 0 else -q


def matrix_from_quat_wxyz(q) -> np.ndarray:
    w, x, y, z = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def euler_zyx_from_matrix(R) -> np.ndarray:
    """(roll, pitch, yaw) in radians, aerospace Z-Y-X convention (as ArduPilot)."""
    R = np.asarray(R, dtype=float)
    pitch = -np.arcsin(np.clip(R[2, 0], -1.0, 1.0))
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return np.array([roll, pitch, yaw])
