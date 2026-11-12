import math

import pytest

from bluerov2_sim2real.sitl_json import (
    SERVO_PACKET_SIZE, FdmState, ProtocolError, ardusub412_parse, build_servo_packet,
    encode_fdm, parse_servo_packet,
)


def test_servo_packet_roundtrip():
    pwm = [1500 + 10 * i for i in range(16)]
    data = build_servo_packet(200, 12345, pwm)
    assert len(data) == SERVO_PACKET_SIZE == 40
    p = parse_servo_packet(data)
    assert (p.frame_rate, p.frame_count, list(p.pwm)) == (200, 12345, pwm)


def test_servo_packet_rejects_bad_magic_and_size():
    data = bytearray(build_servo_packet(200, 1, [1500] * 16))
    data[0] ^= 0xFF
    with pytest.raises(ProtocolError):
        parse_servo_packet(bytes(data))
    with pytest.raises(ProtocolError):
        parse_servo_packet(b"\x1a\x48" + b"\x00" * 10)


def _state():
    return FdmState(12.345, [0.01, -0.02, 0.03], [0.1, -0.2, -9.8], [1.0, -2.0, 3.5],
                    [0.1, 0.2, -0.3], [0.9238795, 0.0, 0.0, 0.3826834])


def test_encoded_line_is_accepted_by_412_parser():
    line = encode_fdm(_state()).decode()
    assert line.startswith("\n") and line.endswith("\n")
    got = ardusub412_parse(line.strip())
    assert got["timestamp"] == pytest.approx(12.345)
    assert got["gyro"] == pytest.approx([0.01, -0.02, 0.03])
    assert got["accel_body"] == pytest.approx([0.1, -0.2, -9.8])
    assert got["position"] == pytest.approx([1.0, -2.0, 3.5])
    assert got["velocity"] == pytest.approx([0.1, 0.2, -0.3])
    assert got["quaternion"] == pytest.approx([0.9238795, 0.0, 0.0, 0.3826834])


def test_412_parser_rejects_space_after_colon():
    # standard json.dumps output (": [") is NOT accepted by the 4.1.2 sscanf parser
    bad = '{"timestamp": 1.0, "imu": {"gyro": [0, 0, 0], "accel_body": [0, 0, -9.8]}, ' \
          '"position": [0, 0, 0], "quaternion": [1, 0, 0, 0], "velocity": [0, 0, 0]}'
    with pytest.raises(ProtocolError):
        ardusub412_parse(bad)


def test_412_parser_requires_mandatory_fields():
    with pytest.raises(ProtocolError):
        ardusub412_parse('{"timestamp":1.0,"imu":{"gyro":[0,0,0],"accel_body":[0,0,-9.8]},"quaternion":[1,0,0,0]}')


def test_encoder_refuses_non_finite():
    s = _state()
    bad = FdmState(s.timestamp_s, [math.nan, 0, 0], s.accel_body_frd, s.position_ned, s.velocity_ned, s.quaternion_wxyz)
    with pytest.raises(ProtocolError):
        encode_fdm(bad)
