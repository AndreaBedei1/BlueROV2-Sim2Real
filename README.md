# BlueROV2-Sim2Real

Research project on the **simulation-to-real characterization and validation of a modified BlueROV2 Heavy** using **HoloOcean** and **ArduSub**.

## Research goal

Study how faithfully an underwater simulation can reproduce the dynamics and sensor observations of a physical BlueROV2, and ultimately assess whether the same autonomous behavior can be evaluated in simulation and on the real vehicle.

**HoloOcean is the selected primary simulator.** Stonefish and Gazebo are included in the literature and technical comparison, but no second simulator implementation is currently planned.

## Current status

**Phase 1 — Platform reconnaissance and literature review: CLOSED (2026-11-12).** The plan's criterion — integration prerequisites and open questions identified — is met. Not all physical vehicle parameters were measured; the residual checks are tracked in [docs/phase1](docs/phase1/README.md). Part of the original literature material (LaTeX report, 36-entry BibTeX, paper review, CSV matrix) was lost; see the availability table there.

**Phase 2 — Minimal ArduSub SITL ↔ HoloOcean verification: planned and documented, not yet implemented or tested.** See [Phase 2 plan](docs/phase2/README.md).

Work performed:
- Inventory of the modified BlueROV2 Heavy, its onboard software, sensors and accessories.
- Review of ArduSub and external simulation interfaces.
- Technical assessment of HoloOcean's BlueROV2 model and known physics issues.
- Comparative review of HoloOcean, Stonefish and Gazebo.
- Critical review of 36 relevant scientific publications (the review itself was lost; 6 key references are re-verified in [references.bib](docs/phase1/report/references.bib)).
- Identification of unresolved hardware measurements and configuration checks.

**No HoloOcean–ArduSub bridge has been implemented in this project.** Existing unrelated middleware is outside the scope of this repository.

## Vehicle and simulator

The physical platform is a modified BlueROV2 Heavy with eight thrusters, Raspberry Pi, Navigator and ArduSub, plus additional sensing and manipulation equipment. The **four horizontal thrusters are mounted at a verified 30° angle**. Individual thruster/channel mappings, actuator signs, sensor mounts and calibration parameters still require verification before assuming physical equivalence with a stock simulated vehicle.

The initial simulator target is **HoloOcean**, subject to appropriate validation of physics, timing, actuation and sensor representations.

## Future research

The next development milestone is a **PC-only minimal ArduSub SITL ↔ HoloOcean depth-control test**. The longer-term plan includes a shared autonomy interface, calibration using real measurements and quantitative sim-to-real evaluation. These are **planned activities, not completed results**.

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
- [Phase 2 — minimal SITL–HoloOcean connection and acceptance tests](docs/phase2/README.md)

This repository is a research work in progress.