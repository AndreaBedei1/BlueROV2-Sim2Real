# Phase 3 — Implementation report: one vehicle interface, two environments

Specification: [README.md](README.md). Operator guide: [QGC_GUIDE.md](QGC_GUIDE.md).
Future hardware replica: [HARDWARE_REPLICA_NOTE.md](HARDWARE_REPLICA_NOTE.md).
Interface contract: `src/bluerov2_sim2real/interface/contract.py` (version **1.0.0**).

**Interface parity is not physics parity.** Everything below concerns messages,
commands, units and protocols. Vehicle dynamics and sensor realism are Phase 4/5.

## 1. Architecture

```
  QGroundControl 4.2.8 (+ joystick)        autonomy program (examples/depth_survey_autonomy.py)
            |  MAVLink                                  |  VehicleClient (one API, contract v1.0.0)
            |                                           |
   ---------+------------------ profile ----------------+---------------------------------
   SIMULATION (default, loopback only)          | REAL_READONLY (explicit --listen, never default)
                                                |
   TCP 5880 / UDP 14570 / UDP 14580              |  BlueOS GCS link (UDP to topside)
   [loopback MAVLink router + tlog]              |  passive listener: no encoder, no send path,
            | TCP 5830                           |  every command REJECTED_LOCALLY
   ArduSub 4.1.2 SITL  (sysid 17, vehicle params)|
            | JSON/UDP 9072 (lockstep, 200 Hz)   |  ArduSub 4.1.2 on Navigator (sysid 1)
   Phase 2 bridge (unchanged numerical path)     |  thrusters, accessories, sensors
     |  thrusters 30° -> 45° agent, drag comp.   |
     |  accessories: output 16 -> camera tilt    |
     |  optional sensor hub:                     |
     |    RGB camera -> H.264 RTP UDP 5610       |  camera -> H.264 RTP UDP 5600
     |    down range -> Ping1D UDP 19090         |  Ping1D -> BlueOS UDP 9090
     |                 + DISTANCE_SENSOR c.194   |          + DISTANCE_SENSOR c.194
   HoloOcean 2.3.0 (BlueROV2 agent)              |  Surveyor 240-16 TCP 62312
                                                |    (Phase 3: REPLAY of recorded device frames)
```

ArduSub stays the controller in both environments: the client sends only what
a ground station sends (HEARTBEAT, MANUAL_CONTROL, COMMAND_LONG, SET_MODE,
SET_POSITION_TARGET_GLOBAL_INT) and reads standard telemetry.

## 2. Interface status

| Interface | Physical reference (Phase 1) | Simulation | Status |
|---|---|---|---|
| MAVLink message set / fields / units | ArduSub 4.1.2 | same firmware (SITL) | **Verified equivalent** (message level) |
| Transport to QGC / companions | BlueOS router, UDP 14550 push | loopback router, TCP 5880 / UDP 14570 | Functional approximation (transport only) |
| Arm / disarm / modes (MANUAL, STABILIZE, ALT_HOLD) via QGC and client | ArduSub 4.1.2 | same code | **Verified equivalent** |
| MANUAL_CONTROL sticks (x, y, r ±1000; z 0..1000, 500 neutral) | QGC + joystick | QGC virtual joystick and client | **Verified equivalent** (format, rate, effect in ArduSub) |
| Physical gamepad through QGC | QGC joystick setup | not testable on this machine | Awaiting physical verification |
| Joystick buttons → accessories | BTN functions of the backup | listed BTN functions set; others keep SITL `sub.parm` | Functional approximation (lights2 pairing and unlisted buttons unverified) |
| Depth control (ALT_HOLD + depth target) | ArduSub 4.1.2 | same code, vehicle gains | Verified equivalent (interface); D2 entry transient remains **FAIL** |
| Motor outputs 1–8 | MOT_n_DIRECTION −1 on 1,2,5,8; wiring unverified | logical +1 | Functional approximation |
| Gripper (output 9) | servo_1 buttons | output 9 = 1100/1900 µs, logical state only | Functional approximation (no mechanics); presence on vehicle L5 |
| Lights (outputs 13/14) | RC9/RC10 passthrough | output levels, no light rendering | Functional approximation |
| Camera tilt (output 16, ±45°, reversed) | mount servo | output 16 + MOUNT_STATUS + rotated simulated camera | Functional approximation |
| Video | H.264 1080p30 RTP/UDP 5600 | H.264 640×360 10 fps RTP/UDP 5610, SDP (optional) | Functional approximation (protocol/codec; no optics) |
| Ping1D | Ping protocol via BlueOS UDP 9090 + DISTANCE_SENSOR c.194 | protocol adapter fed by a ray-cast range | Functional approximation (no acoustics; placeholders documented) |
| Surveyor 240-16 | Cerulean protocol TCP 62312 | recorded device frames replayed byte-for-byte | **Replay only** (no simulated imaging sonar) |
| ROVL / horizontal position | unverified | none | Unsupported (never faked) |
| Pilot-input failsafe (FS_PILOT_INPUT) | active on vehicle | compiled out of SITL | Awaiting physical verification |
| real_readonly profile on the vehicle | BlueOS stream | tested with replay and a fake autopilot | Awaiting physical verification |

## 3. Test results (final batch: commit `ed5dcd5`; QGC session re-run on `be88f29` after the isolation fix)

All runs from a clean checkout (`repo_dirty=0`), one HoloOcean engine at a
time, recorded manifests. Evidence: [evidence/](evidence/).

| # | Test | Result | Evidence |
|---|---|---|---|
| 1 | Unit and contract tests (protocol, frames, thrusters, bridge, client incl. zero-byte read-only check, sensors vs `brping`) | **PASS** 60/60 (+1 private-data test skipped; passes locally on a real Surveyor log) | `pytest` |
| 2 | Phase 2 regression (D1, E, lockstep) on the Phase 3 code | **PASS**: D1 0.000 m; E 5/5, final errors ≤ 1.0 cm; 0 lost / 0 resent frames | [report_DE.json](evidence/phase2_regression/report_DE.json) |
| 3 | D2 ALT_HOLD entry transient (known Phase 2 failure) | **FAIL** (unchanged, reported): 0.577 m | same |
| 4 | QGC operator session through the GUI: arm (slide-confirm), 8 stick directions (virtual joystick), Stabilize → Depth Hold → Manual, disarm; MANUAL_CONTROL 255/190 → 17 at ~25 Hz, z 0..1000 with 500 neutral; ACKs accepted, no failsafe | **PASS** 13/13 | [qgc_session_report.json](evidence/qgc_session/qgc_session_report.json), screenshots |
| 5 | QGC isolation: only listener is the simulation video port (5610); no 14550 / 5600 | **PASS** (enforced at start-up) | run manifest |
| 6 | Client interface test in SITL: telemetry, defined initial state, modes, arm/disarm, teleop up/down/yaw, forward/lateral motor mix, gripper open/close (output 9), lights 1 and 2 (outputs 13/14), camera tilt up/down/center (output 16 = MOUNT_STATUS), depth target | **PASS** 20/21; the only FAIL is D2 | [interface_test.json](evidence/interface_test/interface_test.json) |
| 7 | Same autonomy program, `simulation` | **PASS**: 3/3 depth targets (1.550, 2.521, 1.513 m), tilt and lights commanded, disarmed | [autonomy_simulation.json](evidence/autonomy/autonomy_simulation.json) |
| 8 | Same autonomy program, `replay` of the recorded session (read-only) | **PASS**: completed, 8/8 commands rejected locally, nothing transmitted | [autonomy_replay.json](evidence/autonomy/autonomy_replay.json) |
| 9 | Video: SDP H.264/90000 PT 96; independent decoder: h264 640×360, 60 frames in 6 s | **PASS** | [sensor_check.json](evidence/sensors/sensor_check.json), [video.sdp](evidence/sensors/video.sdp) |
| 10 | Ping1D via Blue Robotics `brping` client; floor depth = depth + altitude constant within 8 mm over 1.56 m of vertical motion | **PASS** | same |
| 11 | DISTANCE_SENSOR component 194 vs Ping1D | **PASS** (max difference 0.048 m) | same |
| 12 | Camera tilt rotates the rendered view in the commanded direction | **PASS** (direction checked on the frames: up = open water, down = seafloor) | [up](evidence/sensors/camera_tilt_up.png), [down](evidence/sensors/camera_tilt_down.png) |
| 13 | Surveyor replay: real laboratory log, every checksum valid, device frames streamed byte-for-byte after SET_PING_PARAMETERS | **PASS** (local private data; test skipped in CI without it) | `tests/test_sensors.py` |
| 14 | real_readonly against the physical vehicle | **NOT RUN** — vehicle not connected (awaiting physical verification) | — |
| 15 | Physical gamepad through QGC | **NOT RUN** — no readable joystick device on the test machine | — |

Anomaly reported (not a test failure): yaw rates of 9–23 rad/s under full yaw
input are physically implausible; the HoloOcean BlueROV2 has no rotational
hydrodynamic damping (Phase 4 calibration).

With `--sim-sensors`, the 10 Hz camera renders make some ticks exceed the 5 ms
real-time budget (lockstep and simulated-time numerics unaffected); such runs
are not used for Phase 2 timing metrics.

## 4. Findings and corrections during Phase 3

* A client that does not stream MANUAL_CONTROL and a GCS heartbeat lets ArduSub
  fall back to RC inputs (the vehicle slowly sank): the client now behaves like
  QGC, and stops its stream if another pilot source appears.
* MAVLink 2 dialect is required (accessory outputs 9–16 are extension fields).
* ArduSub's shift button (BTN5) must be held before shifted functions (gripper).
* QGC toolbar positions change with the mode label width; the GUI scenario
  locates the menus from screenshots.
* Surveyor logs contain SonarView host frames (session JSON, MAVLink wrapper):
  only device frames are replayed by default.
* Camera tilt sign was first inverted (shown by the rendered frames).
* ALT_HOLD entry transient (D2) also affects autonomy: the example waits
  explicitly for the vertical speed to settle (documented, not a fix).
* The HoloOcean model's yaw rates under full yaw input (9–23 rad/s) are
  physically implausible (no rotational damping): reported as an anomaly.

## 5. Independent reviews

Three reviews (MAVLink/QGC parity, sensor adapters, safety/reproducibility)
were run on the code without giving them the expected conclusions. Blocking
items found and fixed: Phase 2 SITL launcher broken by an unset variable; the
GUI scenario could have driven a non-isolated QGC; silent fallback to the
desktop display; orphaned QGC process; inverted camera tilt. Major items fixed
or reclassified: single pilot source, button parity limits, failsafe wording,
data freshness, Ping validity, router send path, replay of host frames,
DISTANCE_SENSOR fields, test strictness. Remaining minor items: SET_MODE
COMMAND_ACK not used (mode confirmed by HEARTBEAT), no router unit tests,
cross-user engine-count race.

## 6. Open items

QGC listens for video on all interfaces (QGC behaviour, port 5610 only).
Physical gamepad test; full parameter backup (buttons, PSC_ACCZ_FF, lights2
pairing); vehicle Ping1D validity; real_readonly against the vehicle (needs a
connected vehicle, still read-only); physical wiring/propellers; video at the
vehicle's resolution and BlueOS camera discovery (mavlink-camera-manager);
Surveyor simulation (not attempted); D2 transient.
