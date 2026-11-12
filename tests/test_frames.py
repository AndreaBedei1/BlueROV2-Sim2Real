import numpy as np
import pytest

from bluerov2_sim2real.frames import (
    euler_zyx_from_matrix, flu_to_frd, matrix_from_quat_wxyz, nwu_to_ned,
    quat_wxyz_from_matrix, rot_nwu_flu_to_ned_frd,
)


def rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def test_vectors():
    assert np.allclose(nwu_to_ned([1, 2, 3]), [1, -2, -3])   # up 3 m -> D = -3 (depth negative above origin)
    assert np.allclose(flu_to_frd([0, 0, 9.8]), [0, 0, -9.8])


def test_depth_sign():
    # 2 m below the surface in HoloOcean (z = -2) is D = +2 in NED
    assert nwu_to_ned([0, 0, -2.0])[2] == pytest.approx(2.0)


def test_yaw_left_in_nwu_is_negative_yaw_in_ned():
    R = rot_nwu_flu_to_ned_frd(rz(np.radians(30)))      # CCW from above in NWU = turning to port
    roll, pitch, yaw = np.degrees(euler_zyx_from_matrix(R))
    assert (roll, pitch) == pytest.approx((0, 0), abs=1e-9)
    assert yaw == pytest.approx(-30)


def test_nose_up_and_right_roll():
    # NWU/FLU: nose up = negative rotation about +y (left); right side down = positive about +x
    # (a positive x-rotation turns +y (left) towards +z (up), so the starboard side goes down)
    roll, pitch, yaw = np.degrees(euler_zyx_from_matrix(rot_nwu_flu_to_ned_frd(ry(np.radians(-10)))))
    assert pitch == pytest.approx(10) and roll == pytest.approx(0, abs=1e-9)
    roll, pitch, yaw = np.degrees(euler_zyx_from_matrix(rot_nwu_flu_to_ned_frd(rx(np.radians(15)))))
    assert roll == pytest.approx(15) and pitch == pytest.approx(0, abs=1e-9)


def test_quaternion_roundtrip_random():
    rng = np.random.default_rng(1)
    for _ in range(200):
        R = rz(rng.uniform(-3, 3)) @ ry(rng.uniform(-1.4, 1.4)) @ rx(rng.uniform(-3, 3))
        q = quat_wxyz_from_matrix(R)
        assert np.linalg.norm(q) == pytest.approx(1.0)
        assert np.allclose(matrix_from_quat_wxyz(q), R, atol=1e-9)


def test_body_to_earth_direction():
    # quaternion must rotate body FRD to earth NED: a vehicle yawed +90 (facing east) moving forward
    R = rot_nwu_flu_to_ned_frd(rz(np.radians(-90)))     # facing NWU -y = east
    assert np.allclose(R @ [1, 0, 0], [0, 1, 0], atol=1e-12)
