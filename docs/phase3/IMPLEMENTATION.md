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

## 3. Test results (final batch, commit FINAL_COMMIT)

FINAL_RESULTS_TABLE

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

Physical gamepad test; full parameter backup (buttons, PSC_ACCZ_FF, lights2
pairing); vehicle Ping1D validity; real_readonly against the vehicle (needs a
connected vehicle, still read-only); physical wiring/propellers; video at the
vehicle's resolution and BlueOS camera discovery (mavlink-camera-manager);
Surveyor simulation (not attempted); D2 transient.
