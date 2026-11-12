# BlueROV2-Sim2Real

Research project on the **simulation-to-real characterization and validation of a modified BlueROV2 Heavy** using **HoloOcean** and **ArduSub**.

## Research goal

Study how faithfully an underwater simulation can reproduce the dynamics and sensor observations of a physical BlueROV2, and ultimately assess whether the same autonomous behavior can be evaluated in simulation and on the real vehicle.

**HoloOcean is the selected primary simulator.** Stonefish and Gazebo are included in the literature and technical comparison, but no second simulator implementation is currently planned.

## Current status

**Phase 1 — Platform reconnaissance and literature review: CLOSED (2026-11-12).** The plan's criterion — integration prerequisites and open questions identified — is met. Not all physical vehicle parameters were measured; the residual checks are tracked in [docs/phase1](docs/phase1/README.md). Part of the original literature material (LaTeX report, 36-entry BibTeX, paper review, CSV matrix) was lost; see the availability table there.

**Phase 2 — Minimal ArduSub SITL ↔ HoloOcean connection: implemented and tested on a development PC** (ArduSub 4.1.2 SITL in lockstep with HoloOcean 2.3.0, depth control, 30-minute stability run). Results, PASS/FAIL per test and open items: [docs/phase2](docs/phase2/README.md); how to repeat: [REPRODUCE](docs/phase2/REPRODUCE.md).

Work performed:
- Inventory of the modified BlueROV2 Heavy, its onboard software, sensors and accessories.
- Review of ArduSub and external simulation interfaces.
- Technical assessment of HoloOcean's BlueROV2 model and known physics issues.
- Comparative review of HoloOcean, Stonefish and Gazebo.
- Critical review of 36 relevant scientific publications (the review itself was lost; 6 key references are re-verified in [references.bib](docs/phase1/report/references.bib)).
- Identification of unresolved hardware measurements and configuration checks.

The HoloOcean–ArduSub bridge of Phase 2 lives in [`src/bluerov2_sim2real`](src/bluerov2_sim2real) (tests in [`tests`](tests), launch and analysis scripts in [`scripts`](scripts)). It was written from scratch for this project; earlier unrelated middleware is outside the scope of this repository. It never connects to the real vehicle.

## Vehicle and simulator

The physical platform is a modified BlueROV2 Heavy with eight thrusters, Raspberry Pi, Navigator and ArduSub, plus additional sensing and manipulation equipment. The **four horizontal thrusters are mounted at a verified 30° angle**. Individual thruster/channel mappings, actuator signs, sensor mounts and calibration parameters still require verification before assuming physical equivalence with a stock simulated vehicle.

The simulator is **HoloOcean** (2.3.0). The installed stock build applies hydrodynamic drag with a unit error (0.01 of the intended value) and fixes the horizontal thrusters at 45°; Phase 2 compensates both on the bridge side and documents the remaining model limits (no added mass, no rotational damping, uncalibrated thrust).

## Future research

The next milestone (Phase 3) is a common autonomy interface for simulation and vehicle; it has not been started. The longer-term plan includes a shared autonomy interface, calibration using real measurements and quantitative sim-to-real evaluation. These are **planned activities, not completed results**.

A possible future development is a **separate Raspberry Pi-based development replica** for software and integration studies. A companion-computer replica alone does not reproduce the Navigator flight controller, ESCs or physical sensors; those components must be treated separately.

## Research records and data

Public documentation should contain curated methods, diagrams, sources and non-sensitive results only. Full onboard backups, detailed hardware diagnostics, raw telemetry, credentials, private network configuration and original recordings are **not intended for public release by default**.

## Literature

Primary background includes:
- Pérez-Edo et al. (2026), Stonefish and ArduSub SITL on BlueROV2 Heavy. https://doi.org/10.3390/jmse14020198
- Ng and Krieg (2024), BlueROV2 SITL dynamics. https://doi.org/10.3390/app14177453
- Orjales et al. (2025), experimental BlueROV2 digital-twin calibration. https://doi.org/10.3390/app15137085
- Potokar et al. (2024), HoloOcean marine robotics simulator. https://doi.org/10.1109/JOE.2024.3410290

## Documentation

- [Phase 1 — findings, published technical notes and outstanding physical checks](docs/phase1/README.md)
- [Phase 2 — minimal SITL–HoloOcean connection: architecture, conversions, thruster mapping, test results](docs/phase2/README.md)
- [Phase 2 — reproduction guide](docs/phase2/REPRODUCE.md)

This repository is a research work in progress.