#!/usr/bin/env python
"""Integration check of the simulated perception interfaces (run inside run_stack.sh
with BRIDGE_ARGS="--sim-sensors --ffmpeg <ffmpeg>"). Writes sensor_check.json."""
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
import bluerov2_sim2real.interface  # noqa: E402,F401
from bluerov2_sim2real.interface.client import VehicleClient  # noqa: E402

RUN = Path(os.environ["RUN_DIR"])
FFMPEG = os.environ.get("FFMPEG", "ffmpeg")
OUT = RUN / "sensor_check"
OUT.mkdir(exist_ok=True)
SDP = RUN / "bridge" / "sensors" / "video.sdp"
checks = []


def check(name, ok, **kw):
    checks.append({"check": name, "pass": bool(ok), **kw})
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {kw}", flush=True)


def video_probe(seconds=6):
    r = subprocess.run([FFMPEG, "-hide_banner", "-protocol_whitelist", "file,udp,rtp", "-i", str(SDP),
                        "-t", str(seconds), "-f", "null", "-"], capture_output=True, text=True, timeout=60)
    frames = [int(x) for x in re.findall(r"frame=\s*(\d+)", r.stderr)]
    stream = re.search(r"Video: (\w+).*?(\d{3,4})x(\d{3,4})", r.stderr)
    return (frames[-1] if frames else 0), stream.groups() if stream else None


def snapshot(path):
    subprocess.run([FFMPEG, "-hide_banner", "-loglevel", "error", "-protocol_whitelist", "file,udp,rtp",
                    "-i", str(SDP), "-frames:v", "1", "-y", str(path)], timeout=60, check=True)
    from PIL import Image
    import numpy as np
    return float(np.asarray(Image.open(path).convert("L")).mean())


def main():
    c = VehicleClient.simulation()
    c.wait(lambda o: o.get("mode") is not None, 60)
    sdp_text = SDP.read_text() if SDP.exists() else ""
    check("video_sdp_h264_pt96", "RTP/AVP 96" in sdp_text and "H264/90000" in sdp_text, sdp=sdp_text.strip()[:300])
    n, stream = video_probe(6)
    check("video_stream_decodes_h264_640x360_about_10fps", stream is not None and stream[0] == "h264"
          and stream[1:] == ("640", "360") and 40 <= n <= 75, frames_in_6s=n, stream=stream)

    from brping import Ping1D
    p = Ping1D()
    p.connect_udp("127.0.0.1", 19090)
    check("ping1d_initialize_with_brping", p.initialize() is True)
    c.wait(lambda o: o.get("rangefinder_m") is not None, 10)
    samples = []
    c.set_mode("MANUAL")
    c.arm()
    for k, z in enumerate((500, 100, 500, 900, 500)):    # hold, descend, hold, ascend, hold (pilot sticks)
        phase = f"{'hold' if z == 500 else 'move'}{k}"
        c.set_sticks(z=z)
        t_phase = time.time()
        while time.time() < t_phase + 3:
            d = p.get_distance()
            samples.append({"phase": phase, "t_in_phase": round(time.time() - t_phase, 2),
                            "ping_mm": d["distance"] if d else None, "conf": d["confidence"] if d else None,
                            "depth_m": c.obs.get("depth_m"), "mav_range_m": c.obs.get("rangefinder_m")})
            time.sleep(0.25)
    c.set_sticks()
    c.disarm()
    ok = [s for s in samples if s["ping_mm"] and s["conf"] == 100 and s["depth_m"] is not None]
    # floor depth = vehicle depth + Ping1D altitude must be constant. Compare only the last
    # samples of each HOLD phase: during motion the telemetry depth, the rangefinder (10 Hz) and
    # the Ping request are not simultaneous (0.5 m/s x 0.3 s ~ 0.15 m).
    holds = [[s for s in samples if s["phase"] == h and s["t_in_phase"] >= 1.5] for h in ("hold0", "hold2", "hold4")]
    floor = [s["ping_mm"] / 1000 + s["depth_m"] for h in holds for s in h
             if s["ping_mm"] and s["conf"] == 100 and s["depth_m"] is not None]
    dep = [s["depth_m"] for h in holds for s in h if s["depth_m"] is not None]
    spread = max(floor) - min(floor) if floor else None
    check("ping1d_distance_tracks_vehicle_depth", len(floor) >= 15 and (max(dep) - min(dep)) > 0.5 and spread < 0.05,
          n=len(floor), depth_range_m=round(max(dep) - min(dep), 3) if dep else None,
          floor_depth_spread_m=round(spread, 4) if spread is not None else None,
          note="floor depth = depth + Ping1D altitude at three hold depths")
    mr = [s for s in ok if s["mav_range_m"] is not None]
    diff = max(abs(s["mav_range_m"] - s["ping_mm"] / 1000) for s in mr) if mr else None
    check("distance_sensor_component194_matches_ping", mr and diff < 0.3, max_diff_m=diff,
          note="DISTANCE_SENSOR at 5 Hz vs Ping1D on request: small lag expected")

    c.camera_tilt("up", hold_s=2.5)
    time.sleep(1.5)
    up = snapshot(OUT / "camera_tilt_up.png")
    c.camera_tilt("down", hold_s=5.0)
    time.sleep(1.5)
    down = snapshot(OUT / "camera_tilt_down.png")
    c.camera_tilt("center", hold_s=0.3)
    from PIL import Image
    import numpy as np
    a_up = np.asarray(Image.open(OUT / "camera_tilt_up.png").convert("L")).astype(float)
    a_dn = np.asarray(Image.open(OUT / "camera_tilt_down.png").convert("L")).astype(float)
    check("camera_tilt_changes_rendered_view", float(np.abs(a_up - a_dn).mean()) > 5,
          mean_abs_pixel_diff=round(float(np.abs(a_up - a_dn).mean()), 1),
          mount_tilt_deg=c.obs.get("mount_tilt_deg"),
          note="DIRECTION is verified by inspecting camera_tilt_up/down.png (luma is not a reliable cue)")
    c.close()
    (OUT / "sensor_check.json").write_text(json.dumps({"checks": checks, "samples": samples[:200]}, indent=1, default=str))
    print(f"sensor check: {sum(x['pass'] for x in checks)}/{len(checks)} passed")


if __name__ == "__main__":
    main()
