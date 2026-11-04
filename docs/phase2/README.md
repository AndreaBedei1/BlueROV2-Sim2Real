# Phase 2 — Minimal HoloOcean ↔ ArduSub SITL connection

**Status: preparation / implementation not yet completed.**

This document follows the approved research plan, *Piano di ricerca e sviluppo — Integrazione e validazione di HoloOcean con il BlueROV2*, specifically its **Phase 2: Verifica del collegamento minimo**.

## Scope and starting conditions

- **HoloOcean is the primary simulator.**
- Begin on a **development PC**, using **ArduSub in Software-in-the-Loop (SITL)**.
- The first end-to-end maneuver is **depth control** of a BlueROV2 Heavy agent, not autonomous navigation or real vehicle control.
- Keep the **physical BlueROV2 disconnected from the control path**. Do not arm, command, reconfigure, calibrate or drive any physical actuator during this work.
- The previously existing desktop `Middleware` project is **out of scope** and must not be reused as an unverified baseline or modified.
- Do not interfere with other HoloOcean instances. Choose dedicated ports, avoid any existing/default HoloOcean port used by other processes, and run at most two HoloOcean instances concurrently.

## Verified physical reference

- Vehicle: **modified BlueROV2 Heavy, eight thrusters**.
- Four horizontal thrusters: **30° mounting angle, verified on the actual vehicle**.
- Four vertical thrusters.
- This confirms the **angle**, not the individual channel mapping, motor rotation, thrust polarity or physical mounting coordinates.
- The simulation must explicitly account for the **30° configuration**, rather than silently assuming stock 45° geometry.
- The onboard software recorded in the Phase 1 technical reconnaissance was BlueOS 1.3.1 and ArduSub 4.1.2. SITL source/build compatibility must be verified before implementation.

## In-scope milestones

### M0 — Reproducible, isolated environments

Record actual versions and hashes of ArduSub source, SITL build, HoloOcean Python package and its Unreal executable. Verify Windows/WSL networking without operating the real ROV. Check that a specific HoloOcean engine build has the correct drag implementation and correct physical clock behavior; do not conflate the stock and patched binaries.

Acceptance: documented environment manifest, no hidden port collision, and no dependencies on real hardware.

### M1 — Simulator interface contract

Study the **exact version-specific** ArduSub external-simulator protocol. Identify the output channel order, accepted message payload, frame conventions, units, timestamps, motor commands and expected simulator feedback. Implement a loopback/mock probe before attaching a physics engine.

Acceptance: deterministic parsing/serialization fixtures and a documented conversion table. Do not infer motor numbering from how a HoloOcean agent orders thruster inputs.

### M2 — Minimal bidirectional loop

Connect ArduSub SITL to HoloOcean: command the simulated eight-thruster agent using explicit channel mapping; translate simulator state and required observations into the SITL protocol. Preserve a single controlled advancement rule for simulation and autopilot time. Begin from stable neutral command and a known depth.

Acceptance: ArduSub motor outputs influence the simulated BlueROV2 and appropriate feedback returns to SITL, with message sequence/time traces and no uncontrolled real-world output.

### M3 — Physical/coordinate checks

Check the reference frames (including NED/ENU and body axes where relevant), units, angular conventions, actuator ordering and thrust signs; verify accelerometer treatment of gravity/specific force and depth sign. Confirm that simulated horizontal thruster geometry is configured for **30°**, not 45°. Do not assume the HoloOcean and ArduSub model have matching nominal PWM-to-thrust curves.

Acceptance: isolated tests that demonstrate direction/sign and transformation consistency. Distinguish known simulated mappings from as-yet-unverified physical ESC wiring.

### M4 — First closed-loop demonstration

Run a **simple depth-hold/change maneuver** in SITL and HoloOcean. Collect autopilot outputs, simulated depth, depth feedback, relevant attitude/IMU data, sim time, wall-clock time and command/update rate. Do not evaluate real vehicle tracking or claim sim-to-real fidelity.

Acceptance: appropriate response to a requested depth change, no unexplained direction inversion, and reproducible logs.

### M5 — Stability and repeatability

Run the integrated closed-loop system for **at least 30 minutes**. Record deadline misses, packet losses, sim-time/wall-time ratio, drift, jitter, CPU/GPU load and any time discontinuities. Isolate simulator ports and other concurrent experiments.

Acceptance: continuous operation without **unexplained loss of synchronization**; otherwise report a clear FAIL and root-cause evidence rather than declaring success.

## Deliverables

- Version and environment manifest (without secrets or private hardware details).
- Documented ArduSub ↔ HoloOcean interface contract.
- Explicit mapping of SITL channels to the eight HoloOcean thrusters, including 30° geometry assumptions.
- Unit tests for frames, units, gravity, actuator sign and protocol serialization.
- Reproducible launcher and integration steps for the development PC.
- Logs and a concise PASS/FAIL report for the depth maneuver and 30-minute stability test.

## Not part of Phase 2

- No real vehicle motion, ESC tests, firmware updates or changes to BlueOS on the actual ROV.
- No full shared autonomy API (Phase 3).
- No experimental hydrodynamic or sensor calibration (Phase 4).
- No physical mission comparisons or B0/B1/B2 claims (Phase 5).
- No Raspberry Pi/Navigator hardware clone yet; that idea belongs to a later, separately reviewed platform-fidelity activity.

## Technical cautions

The source literature and prior Phase 1 investigation establish architectural feasibility, **not** an already verified HoloOcean–ArduSub integration. An ArduSub SITL test on a separate computer is **SIL**, not Navigator HIL. External simulator JSON/UDP behavior and time stepping must be validated against the pinned software versions.

The physical vehicle's attachment hardware (sonars, lighting, gripper, tether) may alter mass distribution, buoyancy and hydrodynamics. No physical fidelity claim is warranted during the minimal software connection test.
