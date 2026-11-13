"""Simulated perception sensors exposed through the vehicle's native interfaces.

Optional and off by default: enabling it adds two HoloOcean sensors (front
RGBCamera on ``CameraSocket``, down-looking RangeFinderSensor at the CoM) but
never changes forces, physics or the lockstep reply. All work that could block
(video encoding, network) runs in threads fed with "latest value" slots, so
the bridge never waits for it.

Interfaces (all 127.0.0.1):
* video: H.264 over RTP/UDP, payload type 96, like the vehicle's BlueOS stream
  (1080p30 to topside UDP 5600); here 640x360 at 10 Hz to a dedicated port,
  with an SDP file. FUNCTIONAL APPROXIMATION: protocol/codec compatible,
  rendering is HoloOcean's, no underwater optics, lights or camera model.
* Ping1D: Ping protocol over UDP (vehicle: BlueOS bridge UDP 9090) fed with the
  down range (``Ping1DAdapter``), plus MAVLink DISTANCE_SENSOR from component
  194 through the router, as BlueOS' ping service does.
* camera tilt: output-16 PWM (MNT servo, reversed, +-45 deg) rotates the camera.
"""
from __future__ import annotations

import os
import subprocess
import threading
import time
from pathlib import Path
from typing import Optional

os.environ.setdefault("MAVLINK20", "1")

from ..interface import contract as C  # noqa: E402
from .ping1d import Ping1DAdapter  # noqa: E402

CAMERA = {"sensor_type": "RGBCamera", "socket": "CameraSocket", "Hz": 10,
          "configuration": {"CaptureWidth": 640, "CaptureHeight": 360}}
RANGEFINDER = {"sensor_type": "RangeFinderSensor", "socket": "COM", "Hz": 10,
               "configuration": {"LaserCount": 1, "LaserAngle": -90, "LaserMaxDistance": 50, "LaserDebug": False}}


class SimSensorHub:
    def __init__(self, out_dir: Path, ffmpeg: Optional[str], video_port: int = 5610, ping_port: int = 19090,
                 mavlink_port: int = 14580, sysid: int = 17):
        self.out = Path(out_dir)
        self.out.mkdir(parents=True, exist_ok=True)
        self.range_m: Optional[float] = None
        self.frames_in = self.frames_out = 0
        self.tilt_deg: Optional[float] = None
        self.errors = []
        self._range_seq = 0
        self._frame = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.ping = Ping1DAdapter(lambda: self.range_m, ping_port).start()
        self.ffmpeg = None
        if ffmpeg:
            sdp = self.out / "video.sdp"
            self.ffmpeg = subprocess.Popen(
                [ffmpeg, "-hide_banner", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgra",
                 "-s", "640x360", "-r", "10", "-i", "-", "-c:v", "libx264", "-preset", "ultrafast",
                 "-tune", "zerolatency", "-g", "10", "-bf", "0", "-pix_fmt", "yuv420p",
                 "-f", "rtp", "-payload_type", "96", f"rtp://127.0.0.1:{video_port}", "-sdp_file", str(sdp)],
                stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=self._open_log("ffmpeg.log"))
            threading.Thread(target=self._video_loop, daemon=True).start()
        threading.Thread(target=self._mavlink_loop, args=(mavlink_port, sysid), daemon=True).start()

    def _open_log(self, name):
        self._logf = open(self.out / name, "w")
        return self._logf

    # called by the backend after every physics step (must not block)
    def on_state(self, state):
        cam = state.get("RGBCamera")
        if cam is not None:
            with self._lock:
                self._frame = cam
            self.frames_in += 1
        rng = state.get("RangeFinderSensor")
        if rng is not None:
            v = float(rng[0])
            self.range_m = v if 0.0 < v < RANGEFINDER["configuration"]["LaserMaxDistance"] - 1e-3 else None
            self._range_seq += 1

    def tilt_from_pwm(self, pwm16) -> Optional[float]:
        return C.decode_tilt_deg(pwm16[C.ACCESSORY_OUTPUTS["camera_tilt"] - 1])

    def _video_loop(self):
        while not self._stop.is_set():
            with self._lock:
                f, self._frame = self._frame, None
            if f is None:
                time.sleep(0.01)
                continue
            try:
                self.ffmpeg.stdin.write(f.tobytes())
                self.frames_out += 1
            except (BrokenPipeError, ValueError) as e:
                self.errors.append(f"video thread stopped: {e!r}")
                return

    def _mavlink_loop(self, port, sysid):
        import socket
        from pymavlink import mavutil
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        class W:
            def write(self, d):
                s.sendto(bytes(d), ("127.0.0.1", port))
        mav = mavutil.mavlink.MAVLink(W(), srcSystem=sysid, srcComponent=194)   # BlueOS ping service id
        # Fields as BlueOS' ping service (core/services/ping/ping1d_mavlink.py): min 20 cm, max 12000 cm,
        # covariance 255, ULTRASOUND, PITCH_270, fov 0.52 rad, sent only for new measurements (<= 10 Hz).
        # Deviation: with no echo nothing is sent (BlueOS forwards the raw reading with low quality).
        t0, last = time.monotonic(), -1
        while not self._stop.is_set():
            seq, r = self._range_seq, self.range_m
            if seq != last and r is not None:
                last = seq
                try:
                    mav.distance_sensor_send(int((time.monotonic() - t0) * 1000), 20, 12000, int(round(r * 100)),
                                             mavutil.mavlink.MAV_DISTANCE_SENSOR_ULTRASOUND, 1,
                                             mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270, 255,
                                             0.52, 0.52, [0, 0, 0, 0], 100)
                except OSError as e:
                    self.errors.append(f"distance_sensor: {e!r}")
            time.sleep(0.05)

    def close(self):
        self._stop.set()
        self.ping.close()
        if self.ffmpeg:
            try:
                self.ffmpeg.stdin.close()
            except Exception:  # noqa: BLE001
                pass
            try:
                self.ffmpeg.wait(5)
            except subprocess.TimeoutExpired:
                self.ffmpeg.kill()
                self.ffmpeg.wait(5)
            self._logf.close()
