# Phase 1 — Platform and state-of-the-art reconnaissance

**Status: CLOSED (2026-11-12) against the Phase 1 criterion of the research plan** — *the elements required for the
integration and the questions still to be verified have been identified.* Phase 1 did **not** measure all physical
parameters of the vehicle; the residual checks are listed in [open-items.it.md](open-items.it.md) and summarised below.

**Scope:** documentation and analysis only. No SITL bridge, autonomy interface, calibrated dynamic model or
real-versus-simulated mission comparison was implemented in this phase.

## Documents

The detailed Phase 1 notes are published in their original language (Italian), as validated in October 2026, with
private identifiers removed (workstation paths, internal IP/MAC addresses, host names, device serials, raw-evidence
indices). Two later corrections are marked in the text as *Aggiornamento 2026-11* (30° thruster angle).
Cross-references inside the notes still use the original file names; the table gives the correspondence.

| Topic | Public file | Original name used in cross-references |
|---|---|---|
| Consolidated Phase 1 report (13 Oct 2026) | [report/phase1-final-report.it.md](report/phase1-final-report.it.md) | `PHASE1_FINAL_REPORT.md` |
| Verified bibliography (partial, see below) | [report/references.bib](report/references.bib) | — |
| Vehicle inventory, thruster map, electronics, network/interfaces, parameters, backups | [hardware/hardware-inventory.it.md](hardware/hardware-inventory.it.md) | `hardware_inventory.md` |
| Existing real sensor data: inventory, timestamps, latency, noise, usability | [hardware/sensor-data-audit.it.md](hardware/sensor-data-audit.it.md) | `sensor_data_audit.md` |
| Software versions, toolchain, existing code | [software/software-inventory.it.md](software/software-inventory.it.md) | `software_inventory.md` |
| ArduSub SITL ↔ external simulator interface (JSON backend, 4.1.2 → master) | [software/sitl-interface-spec.it.md](software/sitl-interface-spec.it.md) | `sitl_interface_spec.md` |
| HoloOcean 2.3.0 BlueROV2 physics, frames, timestep, drag defect | [simulator-review/holoocean-physics-audit.it.md](simulator-review/holoocean-physics-audit.it.md) | `holoocean_physics_audit.md` |
| HoloOcean vs Stonefish vs Gazebo | [simulator-review/simulator-comparison.it.md](simulator-review/simulator-comparison.it.md) | `simulator_comparison.md` |
| Phase 2 architecture *proposal* written at the end of Phase 1 | [phase2-architecture-proposal.it.md](phase2-architecture-proposal.it.md) | `phase2_architecture.md` |
| 31 residual checks with priority and procedure | [open-items.it.md](open-items.it.md) | `phase1_todo.md` |

The proposal assumed a Windows host with WSL2 and the drag-corrected Windows HoloOcean build; the Phase 2
implementation runs on a Linux workstation and documents how it departs from the proposal
([Phase 2](../phase2/README.md)).

## Availability of the Phase 1 material

Inventory performed on 2026-11-12 on the Linux workstation that now hosts the project. Items marked *lost* are no
longer available anywhere known to the project owner and were **not** reconstructed from memory.

| Item | Status | Notes |
|---|---|---|
| Technical reconnaissance report | **available**, published | Markdown report of 13 Oct 2026 (supersedes the 12 Oct PDF) |
| Scientific report in LaTeX and PDF | **lost** | LaTeX compilation could therefore not be re-checked |
| BibTeX bibliography (36 entries) | **lost** | 6 entries rebuilt from doi.org/Crossref, all DOIs verified (see below) |
| Critical review of the 36 publications | **lost** | the published notes still contain the analysis of the key sources (Pérez-Edo et al. 2026, Stonefish, Gazebo, HoloOcean) |
| Bibliographic matrix (CSV) | **lost** | — |
| Hardware and software inventory | **available**, published | sanitised |
| ArduSub SITL interface analysis | **available**, published | — |
| HoloOcean physics and sensor analysis | **available**, published | — |
| HoloOcean / Stonefish / Gazebo comparison | **available**, published | — |
| List of open checks | **available**, published | — |
| Research plan (*Piano di ricerca e sviluppo*, PDF) | available, **kept private** | not published by default |
| Live device evidence and evidence manifest (13 Oct 2026) | available, **kept private** | raw device/network responses, file hashes |
| Rover recordings of 19 Oct 2026 (video, sonar) | available, **kept private** | original sensor acquisitions |
| `ROVerLago` folder (earlier rover notes) | **unreadable** | the copied archive is truncated (1.59 of 2.29 GB, end-of-archive index missing); contents not consulted |
| HoloOcean drag patch | **recoverable** | equals the uncommitted diff of the separate engine-build project against tag `v2.3.0`; reproduced in [Phase 2](../phase2/README.md) |

### Bibliography

[report/references.bib](report/references.bib) contains only the six publications already cited in the public
documents. Each record was downloaded from doi.org (Crossref content negotiation) on 2026-11-12; only month macros and
one citation key were normalised so that the file passes `bibtex` without warnings. Metadata checked:

- Pérez-Edo et al. (2026), *J. Mar. Sci. Eng.* 14(2):198 — Stonefish + ArduSub SITL on BlueROV2 Heavy. https://doi.org/10.3390/jmse14020198
- Ng and Krieg (2024), *Appl. Sci.* 14(17):7453 — ArduSub modifications for BlueROV SITL accuracy. https://doi.org/10.3390/app14177453
- Orjales et al. (2025), *Appl. Sci.* 15(13):7085 — evolutionary digital-twin calibration of open-frame vehicles. https://doi.org/10.3390/app15137085
- von Benzon et al. (2022), *J. Mar. Sci. Eng.* 10(12):1898 — open-source BlueROV2 benchmark simulator. https://doi.org/10.3390/jmse10121898
- Potokar et al. (2024), *IEEE J. Oceanic Eng.* 49(4):1322–1336 — HoloOcean. https://doi.org/10.1109/JOE.2024.3410290
- Meyers and Mangelson (2025), OCEANS 2025 Great Lakes — HoloOcean hardware-in-the-loop. https://doi.org/10.23919/OCEANS59106.2025.11245188

The remaining 30 entries of the original review cannot be listed without inventing or guessing them; a new
systematic search is required if the full review is needed for a publication.

## What has been established

### Physical platform

- The laboratory vehicle is a **modified BlueROV2 Heavy**, not a stock vehicle.
- Eight thrusters: **four horizontal thrusters mounted at 30°, verified on the physical vehicle**, and four vertical
  thrusters. ArduSub frame `VECTORED_6DOF` (`FRAME_CONFIG=2`), outputs 1–8 = motors 1–8.
- Raspberry Pi 4, Navigator, BlueOS 1.3.1, ArduSub 4.1.2 STABLE (`2dd0bb7d`, tag `ArduSub-4.1.2`), `SCHED_LOOP_RATE=200`.
- Front camera (H.264 1080p30), Cerulean Surveyor 240-16, Ping1D, lights, front gripper, tether. ROVL foreseen; its
  operating configuration is not confirmed.
- 913 ArduSub parameters inventoried; 139/139 backup files passed integrity checks (integrity does not prove that
  every backed-up value still matches the active one).

### Thruster configuration (what is and is not known)

| Aspect | Status |
|---|---|
| Number and type: 4 horizontal vectored + 4 vertical | verified |
| Horizontal mounting angle **30°** | **verified physically** (not an open question) |
| ArduSub 4.1.2 `VECTORED_6DOF` mixing factors per motor 1–8 | verified in source (`AP_Motors6DOF.cpp:156-166`) |
| `MOT_n_DIRECTION` = −1 on motors 1, 2, 5, 8 | verified in parameter backup |
| Surge/sway/yaw PWM signs of motors 1–4 | consistent with a pool test (Aug 2026) |
| Physical thruster ↔ Navigator output wiring | **open** |
| Propeller handedness and actual thrust direction of each thruster, verticals in particular | **open** |
| Thruster positions (lever arms), mass, CG/CB, ballast, accessory positions | **open** |

The stock HoloOcean BlueROV2 agent uses **45°** horizontal thrusters; this difference is handled explicitly in
Phase 2.

### Simulator review

HoloOcean was chosen as the primary research simulator (BlueROV2 Heavy agent, underwater sensing). Stonefish remains a
scientific reference (published ArduSub SITL integration on BlueROV2 Heavy, articulated bodies for gripper and tether);
Gazebo provides the reference lockstep JSON-SITL design. None of these choices establishes that one simulator is
intrinsically more accurate for this modified vehicle. The HoloOcean 2.3.0 stock binary applies hydrodynamic drag with
a unit error (factor 0.01); a separately built and verified corrected engine exists only as a Windows package.

### Scientific positioning

External simulator integration with ArduSub, BlueROV2 dynamic calibration and HoloOcean SIL/HIL are already reported
in the literature. The potential contribution is a **controlled, reproducible evaluation of simulator predictive
fidelity**, not merely connecting existing software components. This is a proposed direction, not a Phase 1 result.

## Residual checks carried into later phases

Full list with procedures: [open-items.it.md](open-items.it.md). Main ones:

1. Physical thruster position, wiring to Navigator outputs, propeller direction and thrust sign (verticals never tested in water).
2. Exact geometry and mass properties: lever arms, CG/CB, ballast, accessories (gripper, sonar, lights, tether).
3. Pressure-sensor reference (`BARO2_GND_PRESS` gives +0.52 m in air) and clock synchronisation (BlueOS clock ~7.7 days behind).
4. Download of the 87 onboard logs (~1.6 GB) through a separately authorised read-only workflow.
5. Sensor interfaces: Ping1D behaviour (it transmits in air), ROVL as independent position reference, camera latency.
6. Simulator baseline: auditable HoloOcean build with correct drag, verified timestep and sensor conventions (Phase 2).

## Public versus private records

This repository contains curated, public-safe material only. Raw backups, full parameter dumps, internal network and
device diagnostics, log captures, original acquisitions, private agent transcripts and credentials remain outside the
public Git repository. Any future data release requires a separate review.
