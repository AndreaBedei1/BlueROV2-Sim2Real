# Piloting the simulated BlueROV2 with QGroundControl

The simulated vehicle is ArduSub 4.1.2 SITL (the vehicle's firmware, with the
vehicle's parameters) driving the HoloOcean BlueROV2 through the Phase 2
bridge. QGroundControl talks to it exactly as to the real vehicle: same
MAVLink messages, same flight modes, same joystick handling. Only the
transport differs (see the table at the end).

## 1. One-time setup

* Phase 2 environment (conda env, ArduSub 4.1.2 SITL build with the repository
  patch, HoloOcean 2.3.0): [../phase2/REPRODUCE.md](../phase2/REPRODUCE.md).
* QGroundControl **4.2.8** (the version on the laboratory topside PC), Linux AppImage:

```bash
mkdir -p tools/qgc-4.2.8 && cd tools/qgc-4.2.8 && curl -LO https://github.com/mavlink/qgroundcontrol/releases/download/v4.2.8/QGroundControl.AppImage
```

```bash
chmod +x QGroundControl.AppImage && ./QGroundControl.AppImage --appimage-extract
```

  Reference SHA-256 of the AppImage used: `bee6f766d94d2ab410f0f018b3b9630db4cd0573762ab0ac68b572a01f5764f1`.

## 2. Start the simulation with QGroundControl on your desktop

```bash
export ARDUPILOT_DIR=/path/to/ardupilot-4.1.2 PYTHON=~/miniconda3/envs/bluerov2-sim2real/bin/python QGC_APPDIR=/path/to/tools/qgc-4.2.8/squashfs-root
```

```bash
scripts/phase3/run_stack.sh results/pilot --backend holoocean --qgc desktop
```

The script starts the bridge (one HoloOcean engine), ArduSub SITL with the
accessory/button/failsafe parameters, the loopback MAVLink router and an
**isolated** QGroundControl: it has its own settings folder inside the run
directory, every auto-connect source is disabled (including UDP 14550, on
which a topside computer receives the real vehicle) and its only link is a TCP
client to `127.0.0.1:5880`. Your normal QGroundControl settings are not read
or modified, and this instance cannot reach the physical vehicle. Stop with
Ctrl-C: only the processes started by the script are stopped.

The vehicle is held at the surface while SITL calibrates its pressure
reference, lowered to 1 m and released (about 25 s of simulated time).

Add `BRIDGE_ARGS="--sim-sensors --ffmpeg /path/to/ffmpeg"` to also get the
simulated front camera in QGroundControl's video widget (H.264 RTP on UDP
5610), the Ping1D on UDP 19090 and DISTANCE_SENSOR in the telemetry.

## 3. Joystick

QGroundControl handles the joystick exactly as with the real vehicle
(Vehicle Setup → Joystick: calibrate, enable, choose button functions).
ArduSub receives `MANUAL_CONTROL` with x/y/r in −1000…1000 and **z in 0…1000
(500 = no vertical input)**, and maps buttons through its `BTNn_FUNCTION`
parameters, which in this simulation are the vehicle's:

| Button | Function | Effect in simulation |
|---|---|---|
| 7 | mount center | camera tilt to 0° (output 16 = 1500 µs) |
| 9 / 10 | mount tilt up / down | output 16 (reversed servo, ±45°); rotates the simulated camera with `--sim-sensors` |
| 13 / 14 | lights 1 dimmer / brighter | output 13 level (logical only, no light rendering) |
| shift + 13 / 14 | lights 2 brighter / dimmer (pairing assumed) | output 14 level (logical only) |
| shift + 3 / 4 | gripper close / open (servo_1 min / max) | output 9 = 1100 / 1900 µs (logical only, no mechanics) |
| shift button | BTN5 (ArduSub default, vehicle value not recorded) | — |

Without a physical joystick, QGroundControl's on-screen *virtual joystick*
(Application Settings → General → Virtual Joystick) produces the same
`MANUAL_CONTROL` messages (verified: 25 Hz, z neutral 500, source 255/190).
A physical gamepad was not available on the test machine (device not
readable by the user account): see the interface status table.

## 4. Headless (automated) use

`--qgc private` runs QGroundControl on a private, cookie-protected headless X
display (`scripts/qgc/private_display.sh`), used by the automated operator
scenario `scripts/qgc/qgc_gui_scenario.py` (arm, 8 stick directions, mode
changes, disarm) and its analysis `scripts/qgc/analyze_qgc_session.py`.

## 5. Differences from the physical setup (transport only)

| | Physical vehicle | Simulation |
|---|---|---|
| Autopilot | ArduSub 4.1.2 on Navigator, sysid 1 | ArduSub 4.1.2 SITL, **sysid 17** |
| Link to QGC | BlueOS router pushes UDP to topside `:14550` | loopback router, QGC TCP client to `127.0.0.1:5880` |
| Companion / autonomy | BlueOS endpoints | router pushes UDP to `127.0.0.1:14570` |
| Video | RTP H.264 1080p30 to topside `:5600` | RTP H.264 640×360 10 fps to `127.0.0.1:5610` (optional) |
| Ping1D | BlueOS bridge UDP `9090` | UDP `127.0.0.1:19090` (optional) |
