# Phase 3 — One common interface for simulated and physical BlueROV2

**Status:** specification / implementation not yet performed.  
**Primary simulator:** HoloOcean 2.3.0.  
**Baseline:** Phase 2 on main (`ab1afe2`), tested with ArduSub 4.1.2 SITL.

## Objective

The same operator (QGroundControl and joystick) and the same autonomy software should use the *same documented vehicle-facing interfaces* on the simulated and physical BlueROV2 Heavy. Selecting the target changes the connection profile and transport configuration, **not the autonomy logic or the meanings of commands and observations**.

**Interface parity is distinct from physics parity.** Matching MAVLink messages, sensor packet formats and accessory behaviors does not establish that the real vehicle will follow the same trajectories. Vehicle-specific dynamics and sensor timing are Phase 4 work; quantitative comparison is Phase 5.

## Nonnegotiable architecture

```
QGroundControl / joystick / one autonomy implementation
                       |
       MAVLink + device-specific native sensor protocols
                       |
              target selection
               /            \
      SIMULATED             PHYSICAL
      SITL 4.1.2            BlueOS + ArduSub 4.1.2
         |                    Navigator + RPi4
   Phase 2 JSON bridge           ESC + actual sensors
         |
      HoloOcean
```

Do **not** replace ArduSub with a Python steering controller and do **not** reuse the unrelated desktop `Middleware` as the project's new API. The underlying MAVLink control and flight-mode behavior must remain owned by ArduSub.

No software should ever automatically select the physical vehicle when starting an experiment. Real-vehicle access is **read-only** during Phase 3 and must not send command, parameter, arming, servo or calibration messages.

## Interface inventory and parity levels

| Interface | Physical reference | Phase 3 target | Evidence standard |
|---|---|---|---|
| MAVLink transport | BlueOS MAVLink routing and ArduSub 4.1.2 | SITL MAVLink over isolated loopback endpoint | Message sequence, IDs, fields, ACK/timeout |
| Manual pilot control | QGC joystick, ArduSub MANUAL_CONTROL | Identical MANUAL_CONTROL packets consumed by SITL | Same axes, dead zones, mode and 4.1.2-specific throttle convention |
| Depth / modes | ArduSub MANUAL, ALT_HOLD and supported commands | Same autopilot code, matching parameter semantics | Telemetry/command contract, repeatable mode transitions |
| Thrusters | Eight motors, four horizontal at verified 30° | Logical 30° wrench mapped to fixed 45° HoloOcean agent | Phase 2 mapping plus documented physical wiring unknowns |
| Telemetry | HEARTBEAT, attitude, depth, servo output, health etc. | Same messages and units from SITL | Schema/field parity; distinguish unavailable positioning |
| Video | Real camera via BlueOS video service | HoloOcean image stream exposed through the protocol the consumers need, if supported | Codec/stream URL/timestamps/frequency documented, no claim of realistic underwater optics |
| Surveyor 240-16 | Cerulean packet protocol, optionally SonarLink WebSocket; raw svlog | **First implement recording/replay and a documented semantic adapter**; claim native protocol parity only when exact packet compliance is tested | Packet type, fields, beam geometry, validity, time and loss behavior |
| Ping1D | Ping protocol, current real acquisition unverified | Recorded-sample replay or simulated distance observation through equivalent adapter | No invented valid range samples; protocol and confidence/validity semantics |
| Gripper | Joystick servo_1 commands on output 9 per Phase 1 parameter audit | Simulated logical actuator state; contact/real object manipulation deferred | Commands, ACK/status if actually available; mark unmodeled mechanics |
| Lights | Outputs 13/14 / RCIN9/10 in Phase 1 backup | Simulated intensity state and visual effect only if supported | Channel/command/limits mapping |
| Camera tilt | Output 16 mount tilt per Phase 1 backup | Simulated camera orientation where supported | Tilt command, sign and range mapping |
| ROVL | Operational status/format not confirmed | NOT PRESENT until observed/verified; never fake valid global positioning | Explicit capability absent or unknown |

**Do not claim 1:1 coverage for unimplemented items.** The first milestone is the motion/telemetry/QGC core; video and sonars are subsequent Phase 3 milestones, and physical hydrodynamic identity is out of scope.

## Implementation milestones

### P3-A — Contract and profiles
- Read Phase 1 hardware/software/sensor audit and Phase 2 documentation/code.
- Freeze a versioned common contract with units, NED/FRD frames, modes, names, timestamps, message rates and validity.
- Document the real vehicle's observed services/ports from the *sanitized* records without publishing private addresses.
- Define explicit profiles `simulation` and `real_readonly`; default `simulation`.
- On the development workstation, `simulation` is loopback-only, with a dedicated SITL system ID (17) and nondefault ports. Real physical addresses are never silently routed into the simulator.

### P3-B — QGroundControl and joystick parity
- Reuse the Phase 2 SITL bridge and numerical path unchanged unless a verified defect requires correction.
- Connect a separate QGroundControl instance to the correct SITL MAVLink endpoint; never to the physical vehicle by accident.
- Prove joystick forward/back, lateral, vertical and yaw, arming/disarming in SITL, change of supported modes and corresponding outputs.
- Verify ArduSub 4.1.2's `MANUAL_CONTROL.z` convention **0..1000**; do not silently replace it with generic -1000..1000.
- The ALT_HOLD mode-entry transient is a known **FAIL (D2)**; surface it in tests and documentation, never suppress it with an undocumented special case.

### P3-C — Telemetry and accessory contracts
- Verify typical QGC/companion MAVLink telemetry messages, frequency, field meanings, missing data behavior and timestamp origin.
- Implement/test 8 motors + logical outputs for gripper, lights and camera tilt as separate modules.
- Test accessory behavior on simulation only. For physically absent or not-modeled behavior report `unsupported` or `unknown`, not fabricated successful operation.

### P3-D — Camera and sonars
- Document how the real 1.3.1 BlueOS video endpoint works and how consumer software subscribes, then evaluate an adapter for HoloOcean RGB frames.
- Distinguish raw sensor messages, processed measurements and transport. Preserve original device format when an exact packet-compatible adapter is feasible.
- Surveyor: inspect available `.svlog` or public Cerulean format specs. First implement a real-data parser/replay as a fixture source, with no new live hardware interaction. Do not use a generic MAVLink distance message as a substitute for 16-beam imaging/ranging data.
- Ping1D: expose only validated distance and validity semantics; no synthetic assumption of a successful sensor where the real records are empty or unreliable.
- Sonar rendering and measured timing/noise calibration belong to Phase 4 if they require real experiments.

### P3-E — One autonomy client, two targets
- Implement a thin shared software-facing client that uses existing MAVLink/device protocols; not a replacement controller for ArduSub.
- Run the same logic against `simulation` and `real_readonly`; in the physical profile telemetry access is permitted, **all actuation is rejected before any network write**.
- Run offline/replay tests against recorded input streams when available. The real execution may not be called fully validated without authorized underwater tests.

## Mandatory tests

1. Contract tests for MAVLink messages, units, frames, system IDs, time, transport and sensor validity.
2. QGC joystick manual movement tests using SITL only, with outputs compared to direct MAVLink command fixtures.
3. Mode transitions, especially the documented D2 transient: regression reported separately from Phase 2 acceptance.
4. Same autonomy program under simulated and offline-recorded observations; no branching on simulator-specific names inside the autonomy logic.
5. Accessory command mapping to the correct logical channels with signs and ranges; no claims about physical wiring yet.
6. On real-readonly profile, every attempted actuation (arm, disarm, motor, manual control, mode, servo, parameter writes) fails locally without transmitting.
7. Video and sonar adapters: exact packet/stream compatibility tests before using the phrase `1:1` for them. Unsupported features remain explicitly unsupported.
8. No interference with existing HoloOcean processes; dedicated ports, at most two engine instances, no stopping other users' jobs.
9. Regression of the Phase 2 suite and relevant 30-minute lockstep stability evidence if its numerical/timing path is touched.

## Required deliverables

- Versioned interface matrix (real, SITL, HoloOcean and adapter).
- System/command/observation architecture drawing and configuration profiles.
- Reproducible QGroundControl connection and joystick guide.
- Shared-client implementation with unit/integration tests.
- Clear feature-status grid: `verified parity`, `functional approximation`, `replay only`, `unavailable`, `unverified on hardware`.
- Traceable test evidence with source/version/commit hashes and a PASS/FAIL report.
- Updated README and separate future hardware-replica note (Raspberry Pi + optional Navigator); no hardware clone in this phase.

**Not part of Phase 3:** physical vehicle actuation, full BlueOS clone, hydrodynamic calibration, real sensor modeling from new measurements, full mission sim-to-real evaluation or publication claims of predictive physical fidelity.

## References

- [Phase 1 platform and interface inventory](../phase1/README.md)
- [Phase 2 verified SITL bridge and D2 finding](../phase2/README.md)
- [Phase 2 reproduction guide](../phase2/REPRODUCE.md)
- ArduSub pilot control: https://www.ardupilot.ardupilot.org/sub/docs/pilot-control.html
- BlueOS services: https://blueos.cloud/docs/stable/usage/advanced/
- Cerulean Surveyor interface: https://docs.ceruleansonar.com/c/surveyor-240-16/software-interface-guide
