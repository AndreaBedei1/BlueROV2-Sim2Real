# Phase 1 — Platform and state-of-the-art reconnaissance

**Scope:** documentation and analysis only. No SITL bridge, autonomy interface, calibrated dynamic model or real-versus-simulated mission comparison was implemented in this phase.

## What has been established

### Physical platform

- The laboratory vehicle is a **modified BlueROV2 Heavy**, not a stock, unmodified vehicle.
- The vehicle has eight thrusters: **four horizontal thrusters in a verified 30-degree vectored mounting configuration**, plus **four vertical thrusters**. The 30-degree angle was checked on the physical vehicle by the project owner. Individual thruster positions, wiring, motor-channel mapping, propeller directions and force signs have not yet been independently documented.
- The platform includes a Raspberry Pi companion computer, a Navigator flight controller, BlueOS and ArduSub.
- Add-on components documented during reconnaissance include a forward camera, acoustic sensors (Cerulean Surveyor 240-16 and Ping1D), lighting, a front manipulator/gripper and the tether. Their exact installation details and influence on dynamics are not fully measured.
- ROVL/acoustic positioning was identified as relevant but availability of a calibrated independent underwater position reference is unresolved.

### Software and available records

- Prior on-vehicle read-only checks corroborated the onboard software stack and an inventory of **913 ArduSub parameters**.
- The integrity checks for **139 preserved backup files** passed. Backup integrity does not prove that every backed-up parameter still matches the active value.
- A prior Surveyor acquisition of approximately **40.7 MB** exists. Its missing/incomplete pings and host-side processing delays require careful interpretation.
- **87 onboard logs (~1.6 GB)** were inventoried but had not been downloaded during the initial reconnaissance.
- Existing HoloOcean experiments identified a drag-units problem in the specific engine build tested. A corrected build passed controlled numerical comparisons; the installed stock binary must not be confused with that corrected build.
- Simulation timestep behavior and thruster geometry require version-specific verification.

### Simulator review

**HoloOcean** was chosen as the primary research simulator because it provides a BlueROV2 Heavy agent and underwater sensing capabilities relevant to navigation and perception.

**Stonefish** remains an important alternative: published work demonstrates its combination with ArduSub SITL on BlueROV2 Heavy, and its articulated-body/contact support is valuable for a gripper and tether.

**Gazebo** and associated marine robotics frameworks provide additional examples of modular simulations and ArduSub integration.

**None of these choices establishes that one simulator is intrinsically more accurate for this modified physical vehicle.**

### Scientific literature

The Phase 1 literature survey examined **36 papers and technical publications** about underwater simulators, BlueROV2 dynamics, ArduSub SITL, sensors, calibration, SIL/HIL and simulation-to-reality evaluation.

Relevant precedents show that:
- External simulator integration with ArduSub is already established.
- BlueROV2 dynamic calibration has already been reported.
- HoloOcean has been used in SIL/HIL work with other underwater vehicles.
- Sonar and camera realism must be assessed for the actual sensor and operating conditions.

The potential scientific contribution is therefore a **controlled, reproducible evaluation of simulator predictive fidelity**, not merely connecting existing software components. This is a proposed direction, not a result achieved in Phase 1.

## Unresolved items

1. Confirm each thruster's physical position, propeller direction, control channel and output sign. **The 30-degree horizontal mounting angle is verified and no longer an open question.**
2. Identify exact wiring and geometry of all onboard sensors and add-on equipment, including the gripper.
3. Record vehicle mass, buoyancy arrangement and attachment geometry to support later modeling.
4. Verify sensor interfaces and whether ROVL can provide an independent underwater position reference.
5. Inventory/download the outstanding onboard logs through a separately authorized read-only workflow.
6. Verify timing and configuration consistency before using historical data in quantitative experiments.
7. Preserve an independently recoverable, access-controlled copy of the original platform configuration.
8. Confirm that each simulator baseline uses an auditable build and correctly implemented physics.

These are known follow-up checks and remain recorded after Phase 1. Under the original research plan, Phase 1 completion requires identifying integration prerequisites and open questions, not performing all calibration measurements. **Phase 2 may now begin on the development PC only.** Measuring hydrodynamic coefficients and executing real-versus-simulation trials belong to later phases.

## Public versus private records

This repository contains only curated, public-safe materials. Raw backup archives, full parameter dumps, internal network/device diagnostics, log captures, original acquisition files and any credentials remain outside the public Git repository. Any future data release requires a separate review.

## Reference reading

- Pérez-Edo et al. (2026): https://doi.org/10.3390/jmse14020198
- Ng and Krieg (2024): https://doi.org/10.3390/app14177453
- Orjales et al. (2025): https://doi.org/10.3390/app15137085
- von Benzon et al. (2022): https://doi.org/10.3390/jmse10121898
- Potokar et al. (2024): https://doi.org/10.1109/JOE.2024.3410290
- Meyers and Mangelson (2025): https://doi.org/10.23919/OCEANS59106.2025.11245188

The complete reviewed bibliography and editable report source are maintained separately until checked for safe publication.