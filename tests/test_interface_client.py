"""Contract tests of the common vehicle client (no simulator needed)."""
import socket
import threading
import time

import pytest
from pymavlink import mavutil

from bluerov2_sim2real.interface import contract as C
from bluerov2_sim2real.interface.client import (
    CommandStatus, ReadOnlyViolation, TlogReplayTransport, UdpListenTransport, VehicleClient,
)

mavlink = mavutil.mavlink


def free_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


class FakeAutopilot:
    """Pushes telemetry to the client (like the router's udpout) and records what comes back."""

    def __init__(self, client_port, sysid=17, ack=True):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.05)
        self.dst = ("127.0.0.1", client_port)
        self.mav = mavlink.MAVLink(self, srcSystem=sysid, srcComponent=1)
        self.rx = mavlink.MAVLink(None)
        self.received = []
        self.raw_bytes = 0
        self.armed, self.mode, self.ack = False, 19, ack
        self.stop = threading.Event()
        self.th = threading.Thread(target=self.loop, daemon=True)
        self.th.start()

    def write(self, data):
        self.sock.sendto(bytes(data), self.dst)

    def loop(self):
        t_last = 0
        while not self.stop.is_set():
            if time.monotonic() - t_last > 0.05:
                t_last = time.monotonic()
                bm = mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (mavlink.MAV_MODE_FLAG_SAFETY_ARMED if self.armed else 0)
                self.mav.heartbeat_send(mavlink.MAV_TYPE_SUBMARINE, mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, bm, self.mode, 4)
                self.mav.global_position_int_send(1000, 0, 0, -2500, -2500, 0, 0, 30, 0)
                self.mav.attitude_send(1000, 0.1, -0.2, 0.3, 0, 0, 0)
                self.mav.servo_output_raw_send(1000000, 0, *[1500] * 8, 1900, 1500, 1500, 1500, 1500, 1100, 1500, 1300)
                self.mav.distance_sensor_send(1000, 20, 3000, 150, 0, 0, 25, 0)   # comp 1: must be ignored
            try:
                data = self.sock.recv(65536)
            except socket.timeout:
                continue
            self.raw_bytes += len(data)
            for m in self.rx.parse_buffer(data) or []:
                self.received.append(m)
                if m.get_type() == "COMMAND_LONG" and m.command == mavlink.MAV_CMD_COMPONENT_ARM_DISARM and self.ack:
                    self.armed = m.param1 == 1
                    self.mav.command_ack_send(m.command, mavlink.MAV_RESULT_ACCEPTED)
                elif m.get_type() == "SET_MODE":
                    self.mode = m.custom_mode

    def close(self):
        self.stop.set()
        self.th.join(1)
        self.sock.close()


@pytest.fixture
def sim():
    port = free_port()
    client = VehicleClient("simulation", UdpListenTransport("127.0.0.1", port, writable=True), autopilot_sysid=17)
    ap = FakeAutopilot(port)
    assert client.wait(lambda o: o.get("mode") is not None, 3)
    yield client, ap
    client.close()
    ap.close()


def test_observations_units_and_frames(sim):
    client, _ = sim
    assert client.wait(lambda o: o.get("depth_m") is not None and o.get("servo_us") is not None, 3)
    o = client.obs
    assert o.get("depth_m") == pytest.approx(2.5)            # relative_alt -2500 mm -> 2.5 m deep
    assert o.get("attitude_rpy_rad") == pytest.approx((0.1, -0.2, 0.3))
    assert o.get("mode") == "MANUAL" and o.get("armed") is False
    assert o.get("gripper_cmd") == "open"                     # output 9 at 1900
    assert o.get("lights2_level") == pytest.approx(0.0)       # output 14 at 1100
    assert o.get("camera_tilt_deg_from_servo") == pytest.approx(22.5)   # reversed output 16 at 1300
    assert o.get("rangefinder_m") is None                     # autopilot-internal DISTANCE_SENSOR ignored
    assert o.get("horizontal_position") is None               # never provided


def test_arm_mode_and_manual_control(sim):
    client, ap = sim
    r = client.arm()
    assert r.status is CommandStatus.ACCEPTED and client.obs.get("armed") is True
    assert client.set_mode("ALT_HOLD").status is CommandStatus.ACCEPTED
    assert client.set_depth_target(1.5).status is CommandStatus.SENT
    assert client.set_sticks(x=300, z=700).status is CommandStatus.SENT
    time.sleep(0.4)
    mc = [m for m in ap.received if m.get_type() == "MANUAL_CONTROL"]
    assert mc and (mc[-1].x, mc[-1].y, mc[-1].z, mc[-1].r) == (300, 0, 700, 0)
    assert mc[-1].target == 17 and mc[-1].get_srcSystem() == 255
    sp = [m for m in ap.received if m.get_type() == "SET_POSITION_TARGET_GLOBAL_INT"]
    assert sp and sp[-1].alt == pytest.approx(-1.5)


def test_manual_control_convention_enforced(sim):
    client, _ = sim
    r = client.set_sticks(z=-500)                 # generic -1000..1000 throttle is NOT ArduSub 4.1.2
    assert r.status is CommandStatus.REJECTED_LOCALLY and "0..1000" in r.detail


def test_depth_target_requires_alt_hold(sim):
    client, _ = sim
    assert client.set_depth_target(1.0).status is CommandStatus.REJECTED_LOCALLY


def test_shifted_button_sequence(sim):
    client, ap = sim
    client.gripper("open")
    masks = [m.buttons for m in ap.received if m.get_type() == "MANUAL_CONTROL"]
    shift, b4 = 1 << C.SHIFT_BUTTON, 1 << 4
    assert shift | b4 in masks
    i = masks.index(shift | b4)
    assert shift in masks[:i]                     # shift is held before the button
    assert masks[-1] == 0


def test_timeout_when_autopilot_does_not_ack():
    port = free_port()
    client = VehicleClient("simulation", UdpListenTransport("127.0.0.1", port, writable=True), autopilot_sysid=17)
    ap = FakeAutopilot(port, ack=False)
    try:
        assert client.wait(lambda o: o.get("mode") is not None, 3)
        r = client._command_long("arm", mavlink.MAV_CMD_COMPONENT_ARM_DISARM, [1], timeout_s=0.5)
        assert r.status is CommandStatus.TIMEOUT
    finally:
        client.close()
        ap.close()


def test_simulation_ignores_foreign_autopilot():
    port = free_port()
    client = VehicleClient("simulation", UdpListenTransport("127.0.0.1", port, writable=True), autopilot_sysid=17)
    ap = FakeAutopilot(port, sysid=1)            # a vehicle with sysid 1 (the real one's id)
    try:
        time.sleep(0.5)
        assert client.obs.get("mode") is None and client.foreign_messages > 0
    finally:
        client.close()
        ap.close()


def test_real_readonly_never_transmits():
    port = free_port()
    client = VehicleClient.real_readonly(("127.0.0.1", port), autopilot_sysid=1)
    ap = FakeAutopilot(port, sysid=1)
    try:
        assert client.wait(lambda o: o.get("depth_m") is not None, 3)       # telemetry is read
        results = [client.arm(), client.disarm(), client.set_mode("MANUAL"), client.set_sticks(x=100),
                   client.set_depth_target(1.0), client.gripper("open"), client.lights(1, True),
                   client.camera_tilt("up"), client.press_button("mount_center")]
        assert all(r.status is CommandStatus.REJECTED_LOCALLY for r in results), results
        with pytest.raises(ReadOnlyViolation):
            client.transport.send(b"\xfd")       # even a direct write is blocked before the socket
        time.sleep(0.5)
        assert ap.raw_bytes == 0 and ap.received == []
        assert client.rejected_locally == len(results)
    finally:
        client.close()
        ap.close()


def test_read_only_profile_refuses_writable_transport():
    with pytest.raises(ValueError):
        VehicleClient("real_readonly", UdpListenTransport("127.0.0.1", free_port(), writable=True), 1)


def test_replay_profile(tmp_path):
    # record a few frames in tlog format and replay them read-only
    import struct
    out = []

    class W:
        def write(self, d):
            out.append(struct.pack(">Q", int(time.time() * 1e6)) + bytes(d))
    m = mavlink.MAVLink(W(), srcSystem=1, srcComponent=1)
    m.heartbeat_send(mavlink.MAV_TYPE_SUBMARINE, mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 2, 4)
    m.global_position_int_send(5, 0, 0, 0, -1200, 0, 0, 0, 0)
    p = tmp_path / "x.tlog"
    p.write_bytes(b"".join(out))
    client = VehicleClient.replay(p, autopilot_sysid=1)
    try:
        assert client.wait(lambda o: o.get("depth_m") is not None, 2)
        assert client.obs.get("mode") == "ALT_HOLD" and client.obs.get("depth_m") == pytest.approx(1.2)
        assert client.arm().status is CommandStatus.REJECTED_LOCALLY
        assert isinstance(client.transport, TlogReplayTransport)
    finally:
        client.close()


def test_contract_decoders():
    assert C.decode_tilt_deg(1500) == pytest.approx(0)
    assert C.decode_tilt_deg(1100) == pytest.approx(45)      # reversed servo: low PWM = tilt up
    assert C.decode_tilt_deg(1900) == pytest.approx(-45)
    assert C.decode_level(1900) == 1 and C.decode_level(0) is None
    assert C.decode_gripper(1100) == "close" and C.decode_gripper(0) is None
    assert "1.0.0" in C.Contract().to_json()
