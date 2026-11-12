# Inventario software — Fase 1 (ricognizione) — HoloOcean ↔ BlueROV2/ArduSub

Data: 2026-10-13. Ricognizione **solo lettura**. Nessun repository modificato; solo comandi Git `--no-optional-locks` di lettura. Il motore HoloOcean non è stato avviato. Non c'è stato traffico di rete verso <ROV-subnet>.
Unico comando eseguito sul codice: `pytest -q` in `Middleware`, con `-p no:cacheprovider` e `PYTHONDONTWRITEBYTECODE=1` per non scrivere nulla nel repo. Prima ho verificato i test: tutti usano fake, `FakeEnvironment` e `http.client` mockato. Esito: **42 passed in 0.90 s**.

Le righe marcate (†) vengono da sotto-ricognizioni automatiche (agenti Explore). Hanno citazioni file:riga, ma non le ho ricontrollate riga per riga.

## 0. Sintesi

| Tema | Stato |
|---|---|
| Middleware v1 | Esiste una API Python comune (`VehicleBackend`) con tre backend: HoloOcean (scheme 0, servo PI interno a 30 Hz), BlueROV (sola telemetria GET mavlink2rest, attuazione bloccata) e Replay. 42 test offline passano. Validazione nel motore reale del 5/10, eseguita sul **binario ufficiale non patchato**. |
| Patch drag | Nel repo `Battery` (HoloVirtualBattery): `patches/holoocean-2.3-drag-units.patch`. Esistono due build compilate in `<HOB>\packages\{patched-after,rebuilt-before}` (UE 5.3.2 sorgente in `<UE-5.3>`). Il binario installato in `%LOCALAPPDATA%` è quello ufficiale non patchato. |
| ArduSub/MAVLink | C'è codice pymavlink reale per attuazione: `HoloObstacleAvoidance/scripts/real/rovlink.py`, `cmd_bridge.py` e `rov_real_bridge`. C'è un backend ROS-indipendente in `RovTest/.../mavlink_backend.py`. **Nessun codice SITL (JSON/FDM) e nessun sorgente o binario ArduPilot sul PC.** |
| Timing | Ci sono misure del passo fisico UE (`Battery/docs/holoocean_timestep_report.md`), RTF/wall-clock (`Battery/logs/generic_benchmark`, `BlueROV2SimMatch/results/env_verification.json`) e latenza reale MANUAL_CONTROL→PWM di 53–89 ms (HoloObstacleAvoidance). |
| Copie HoloOcean | Ci sono 5 copie del sorgente o del binario 2.3.0, descritte nel §3. Il client installato è identico all'upstream v2.3.0. |
| Spazio disco | Solo il drive C:, con **~15 GB liberi**. `<HOB>` occupa 34 GB, `<UE-5.3>` circa 20 GB. |

## 1. Ambiente host e toolchain

| Elemento | Valore | Evidenza |
|---|---|---|
| OS | Windows 11 Education 10.0.26200. Unico drive `C:` (~495 GB usati, ~15 GB liberi) | `Get-PSDrive` |
| Python di default in shell | `<python-3.8>` 3.8.3 (da **non** usare per HoloOcean) | `Middleware/README.md` |
| Conda env `holoocean_joystick` | Python 3.10.20, **holoocean 2.3.0** (installato da `file:///<user-home>/Downloads/HoloOcean-2.3.0/client`, cartella oggi non più presente), numpy 1.26.4, scipy 1.15.3, matplotlib 3.10.9, pytest 9.1.1, pyyaml 6.0.3, pygame 2.6.1, pywin32 312, holoenergy 0.1.0 (editable → `Desktop\Battery`). **Senza pymavlink** | `direct_url.json`, `pip list` |
| Conda env `ros2_lyrical` | Python 3.12.3, **pymavlink 2.4.49**, numpy 2.4.6 | `importlib.metadata` |
| Conda env `ros2` | Python 3.8.3, pymavlink 2.4.49 | idem |
| Altri env | altri ambienti di progetti non correlati, base miniconda 3.13.12 (torch 2.12). Non rilevanti | |
| `RovCode/.venv` | Python 3.12, pymavlink 2.4.49, opencv 4.11, numpy 1.26.4 (†) | |
| `Battery/.venv` | uv, Python 3.13.12 (dev/CI di holoenergy, senza HoloOcean) | `pyvenv.cfg` |
| ROS 2 | **Lyrical** (rclpy 10.0.10) in `<dev>/ros2_lyrical` e `<dev>/ros2-windows`. Nessun pacchetto mavros/ardupilot | `share/ros_environment` |
| Unreal Engine | Nessuna installazione da Epic Launcher. **UE 5.3.2 da sorgente** in `<UE-5.3>` (tag `5.3.2-release`, `072300df`, grafted), compilato solo per il target game. **Manca `UnrealEditor.exe`**, quindi non si possono fare cook delle mappe | `Build.version`, `ls Engine/Binaries/Win64` |
| Visual Studio | VS 2022 BuildTools con MSVC **14.44.35207** (lo stesso toolset del binario ufficiale), VS "18" Community, Windows SDK 10.0.22621.0 e 10.0.26100.0 | `Program Files (x86)` |
| WSL | `Ubuntu-24.04` (default, Stopped, v2) e un'altra distro non correlata. **Contenuto non ispezionato**: le distro non sono state avviate | `wsl -l -v` |
| Docker | Assente (non è nel PATH e non c'è in Program Files) | |
| ArduPilot / SITL | **Nessun sorgente, binario SITL, MAVProxy o `sim_vehicle.py`** su Windows. La ricerca ha coperto `C:\` a profondità 4 e `<user-home>` a profondità 5 | `find` |
| QGroundControl | 4.2.8 in `<Program Files>\QGroundControl`. Log in `Documents\QGroundControl` (Telemetry: 12 file) | VersionInfo |
| Altro | PingViewer, SonarView (Cerulean), usbipd-win, CMake, Git, nodejs | |

## 2. Repository e cartelle

### 2.1 Middleware — `<workstation>/Middleware` (**nessun repository Git**)

| Voce | Dettaglio |
|---|---|
| Stato | Cartella senza Git (`docs/middleware_design.md:3-4`). C'è una copia `build/lib/rov_middleware` identica a `src/` (`diff -rq` vuoto) e un wheel in `results/validation/packages/rov_middleware-0.1.0-py3-none-any.whl`. |
| Dipendenze | `pyproject.toml`: `rov-middleware 0.1.0`, `requires-python>=3.10`, `numpy>=1.23,<3`, extra `pytest`/`matplotlib`, marker `integration`. `requirements.txt`: numpy 1.26.4, matplotlib 3.10.9, pytest 9.1.1. `environment.yml`: env `rov_middleware`, py 3.10. Env reale: `holoocean_joystick`. |
| Config | `config/sim/default.json`: 30 Hz, SimpleUnderwater/Ocean, start NWU (0,0,-1.8), target depth 1.5 m, max 0.6/0.5 m/s e 0.6 rad/s, gain PI/PD, `max_thrust_n` 28.75, `sleep_dither_n` 0.02. `config/real/passive.json`: <ROV-IP>:6040, sys/comp 1/1, 2 Hz, `allow_actuation:false`. |
| API | `src/rov_middleware/api.py:47-113` definisce `VehicleBackend` (ABC). I setter `set_surge/sway/yaw_rate/velocity` sono normalizzati in [-1,1] con clip, e NaN/Inf vengono rifiutati (`api.py:11-19`). Ci sono anche `set_depth/hold_depth`, `stop` (mantiene la profondità), `update()` (un tick per chiamata, nessun thread) e il context manager. `VehicleState` (`api.py:30-44`) è in SI, NED/FRD, con tempo relativo. |
| Frame | `frames.py:1-31`: `S=diag(1,-1,-1)` per NWU→NED, yaw negato, `world_xy_to_body`. |
| Factory | `factory.py:5-16`: `create_vehicle("holoocean"|"bluerov"|"replay")`. |
| Backend HoloOcean | `backends/holoocean.py`. `_VelocityServo` (`:13-51`) è un PI di velocità con PD roll/pitch e allocazione pinv su 8 thruster; geometria a `:24-29` (fonte `BlueROV2.cpp` e BlueROV2SimMatch). Lo scenario (`:66-76`) ha `ticks_per_sec = update_rate_hz` (30), `frames_per_sec False`, scheme 0, sensori Pose/Velocity/IMU sul socket COM. In `connect` c'è il seed (`:78-100`). La depth PD in world è a `:126-140`. Il workaround per lo sleep di UE (dither ±0.02 N più teleport dopo 0.5 s fermo, `:146-161`) è a `:146-163`, e `env.step(forces)` a `:163`. |
| Backend BlueROV | `backends/bluerov.py`. **Solo GET** `http://<ROV-IP>:6040/mavlink/vehicles` (`:25-46`). Ogni attuatore, `stop` compreso, solleva `RuntimeError` prima di qualunque I/O (`:96-98`); `allow_actuation` diverso da `False` viene rifiutato (`:84-86`). La connessione richiede un contatore HEARTBEAT in avanzamento (`:152-170`). Ci sono freshness per messaggio, flag EKF `0x10`/`0x80` per `LOCAL_POSITION_NED` (`:176-184`), depth da `-relative_alt/1000` (`:207`) e mappa dei mode ArduSub (`:21-22`). |
| Backend Replay | `backends/replay.py:13-80`: playback del JSONL, attuazione rifiutata. |
| Controller e missioni | `waypoint.py:10-74` (P posizione/yaw solo su API, timeout, assestamento, verifica che il clock avanzi), `mission.py:8`, `motion_tests.py:10` (test A–E). |
| Log | `runlog.py:14-74`: CSV + JSONL con `schema_version 1`, manifest con versioni, frame e semantica del tempo. Non registra l'hash del binario HoloOcean. |
| Script | `scripts/probe_sim.py`, `run_motion_test.py`, `run_waypoint_mission.py`, `check_vehicle.py` (GET passivo), `replay_run.py`, `plot_results.py`, `compare_runs.py`, `_bootstrap.py`. |
| Test | `tests/test_api_sim.py` (FakeEnvironment), `test_real_safety.py` (Source fake e `http.client` patchato; verifica anche l'assenza di `pymavlink`/`send(` nel sorgente, `:76-83`), `test_waypoint.py` (KinematicVehicle), `test_replay_log.py`. **42 passed** oggi (0.90 s); 42 passed anche nel run del 5/10 (`docs/session_validation.md:12`). Nessun test lancia il motore: il marker `integration` è dichiarato ma non usato. |
| Risultati | `results/validation/motion/{A..E}`, `motion_repeat/A`, `waypoint`, `waypoint_repeat`, `real_passive`, `reproducibility_*.json` (differenza 0.0). Tutti creati il 5/10 con holoocean 2.3.0 a 30 Hz, wake_count 0. **Binario ufficiale, cioè drag a 1/100** (vedi §3): la dinamica validata non include il drag corretto. |
| Documentazione | `docs/middleware_design.md` (evidenze, architettura, frame, interlock), `session_validation.md` (tabelle A–E e waypoint; passivo reale: MANUAL/disarmed, depth 0.51 m in aria, EKF 0xA7 const_pos_mode, nessuna posizione), `water_test_handoff.md` (punti "TO BE VERIFIED", failsafe FS_PILOT_INPUT/FS_GCS). |
| Misure di tempo | Solo tempo simulato (`timestamp = ticks/30`, `holoocean.py:132`). **Nessuna misura wall-clock o RTF.** |

### 2.2 Battery (HoloVirtualBattery / `holoenergy`) — `<workstation>/Battery`

| Voce | Dettaglio |
|---|---|
| Git | remote `https://github.com/AndreaBedei1/HoloVirtualBattery.git`; branch `fix/verified-holoocean-drag-backend` allineato a origin; HEAD `fb7efaa74c8d` (2026-10-08 00:34 +0200) "docs: record push and CI results of the verified-backend phase"; `describe` `v0.1.0-36-gfb7efaa`. **Working tree pulito.** Altri branch: `main`, `feature/generic-marine-energy-framework` (62c18cf), `feature/holoenergy-battery-model` (d179584), `feature/scientific-consolidation` (a7a962b, tag v0.1.0), `investigation/holoocean-drag-units` (07deed7). |
| Dipendenze | `pyproject.toml`: py>=3.10, PyYAML; extra dev pytest/ruff/build/openpyxl; `uv.lock`. CLI `holoenergy-study` e `holoenergy-dashboard`. CI `.github/workflows/ci.yml` (8 job Ubuntu/Windows × 3.10–3.13, success). |
| Pacchetto | `holoenergy/` (modelli di batteria, propulsione T200, termico, `env_wrapper.py` con `EnergyAwareEnv`). |
| Tool HoloOcean | `tools/holoocean_backend.py:51-78` (`make_env`: avvia un **Holodeck.exe alternativo** con gli stessi parametri di `holoocean.make`; `resolve_binary` a `:27-48`), `check_holoocean_backend.py` (PASS/FAIL drag), `audit_holoocean_drag.py`, `audit_holoocean_timestep.py`, `build_patched_holoocean.py`, `run_verified_backend_campaign.py`, `benchmark_energy.py`. |
| Patch | Vedi §3.1. |
| Test | 22 file in `tests/`; `.pytest_cache/.../lastfailed` = `{}`. CI verde. La suite nativa 29/29 PASS è in `sources/verified_backend/native_suite.json`. Non rieseguiti. |
| Documentazione | `docs/holoocean_drag_units_report.md`, `verified_backend_report.md` (791 righe), `holoocean_patched_build.md`, `holoocean_timestep_report.md`, `holoocean_compatibility.md`, `holoocean_drag_issue_draft.md`, `validation_protocol_bluerov2.md`. |
| Misure di tempo | **Passo fisico**: `dt_fisico = min(1/tps, 1/30)`. A 20 Hz il rapporto fisica/clock è 0.667; da 30 a 200 Hz è 1.0. Raccomandazione: 100 Hz (`docs/holoocean_timestep_report.md:9-17, 68-76, 135-148`). Con il drag corretto, a 20–30 Hz l'Eulero esplicito inverte la velocità a 2.5 m/s (`:94-100`). **Wall-clock**: ~22.5 ms/step in free-run (`logs/generic_benchmark/performance.json`, `tools/benchmark_energy.py:64-70`; tps dello scenario non confermato). Demo real-time con pacing `perf_counter` a 20 Hz in `examples/realtime_energy_demo.py:66-93`. |
| Riferimenti esterni | Un clone metadata-only di upstream si trova in `build/holoocean-upstream`: partial `blob:none`, tag v2.3.0 `49e7055`, nessun checkout (1191 `D` in stage, normale per un clone senza checkout). C'è anche `build/ue-source-review/*.cpp/.h` (estratti UE per l'audit). |

### 2.3 BlueROV2SimMatch — `<workstation>/BlueROV2SimMatch` (**nessun repository Git**) (†)

| Voce | Dettaglio |
|---|---|
| Scopo | Replica in HoloOcean/ROS 2 delle prove in piscina del 16/08 (obstacle avoidance), con S3 calibration e parity ROS. |
| Struttura | `src/brsm/` (pipeline, `command_mapping.py`, `actuation.py`, `safety_interlock.py`), `src/sim_server/` (`runner.py`, `socket_server.py`, `protocol.py` TCP JSON 127.0.0.1:47654, `vehicle_dynamics.py`), `src/ros2_ws/` (`brsm_msgs`, `brsm_ros`), `scripts/`, `config/`, `results/`, `dataset/`. |
| Dipendenze | **Non fissate**: nessun requirements o pyproject. Usa l'env `holoocean_joystick` più ROS 2 Lyrical/`ros2_lyrical` (`scripts/_ros_env.bat:10-14`). `protocol.py:7-8` cita ancora un env `ocean` py3.9, non aggiornato. |
| HoloOcean | 2.3.0, 30 tps, `frames_per_sec False`, scheme 0 (`runner.py:57,126-133`). Lo sleep guard è a `runner.py:322-361`. La geometria dei thruster corretta dal C++ è in `vehicle_dynamics.py:32-99`: il docstring del client è sbagliato. |
| MAVLink | **Assente.** Solo il mapping Twist→MANUAL_CONTROL counts (`command_mapping.py:64-165`) e il modello di attuazione S3 (`actuation.py:58-178`). |
| Timing | `results/env_verification.json:77-92`: RTF 2.10 senza camera e 0.77 con camera da 512 px; `make` impiega 7.3 s. `results/probes/render_probe.json:48-58`. ROS closed loop a 20.05 Hz (`results/ros_closed_loop/K0_seed000.json`). |
| Test | 7 file, ~106 test; `lastfailed` `{}` (17/08). |
| Documentazione | `README.md`, `docs/AUDIT.md`, `results/real_vs_sim.md`. |

### 2.4 HoloObstacleAvoidance — `<workstation>/HoloObstacleAvoidance`

| Voce | Dettaglio |
|---|---|
| Git | remote `git@github.com:AndreaBedei1/HoloObstacleAvoidance.git`; branch `feature/scientific-sim-to-real-obstacle-avoidance`; HEAD `cca9b8e4207a` (2026-09-14 20:24 +0200) "Move BlueROV2 multimodal recorder to dedicated repository". |
| Scopo | Stack ROS 2 per obstacle avoidance da camera. Il confine autonomia↔veicolo è `/planner/cmd_vel_safe` (Twist), identico in sim e reale (†). |
| HoloOcean | Server `rov_obstacle_sim_bridge/holoocean_sim_server.py`, TCP 47654, scheme 0, 30 tps obbligatori (decisione D-010, `SCIENTIFIC_DECISIONS.md:165-179`). Il pacing real-time usa `timeBeginPeriod(1)`. C'è una pipeline di dinamica (`bluerov2_dynamics.py`) e un watchdog dei comandi di 1 s (†). |
| MAVLink (attuazione reale) | `scripts/real/rovlink.py:42-263`: `udpin:0.0.0.0:14550`, sysid 255/190, heartbeat GCS a 1 Hz, `manual_control_send`, `set_mode`, arm/disarm, `SET_MESSAGE_INTERVAL`; neutro e disarm all'uscita (†). `scripts/real/cmd_bridge.py`: Twist→MANUAL_CONTROL a 20 Hz in ALT_HOLD, timeout di 1 s verso neutro, interlock tramite variabili d'ambiente (†). `src/rov_real_bridge/rov_real_bridge/real_control_live_node.py` (`mavlink_url` parametrico, 20 Hz, senza arm/set_mode), `command_mapping.py`, `safety_interlock.py` (†). |
| Fatti sul veicolo | `docs/REAL_BLUEROV2_HARDWARE_INVENTORY.md`: ArduSub 4.1.2, FRAME_CONFIG=2 (Heavy), SERVO1-8 function 33–40, MOT_PWM 1100–1900. `docs/real_vehicle_reference/data/mavlink_inventory.json`: ATTITUDE ~12 Hz, HEARTBEAT 3.6 Hz (†). |
| Timing reale | `docs/REAL_POOL_EXPERIMENT_LOG.md:16-27`: **latenza MANUAL_CONTROL→PWM 53–89 ms**, t63 surge ~1.44 s e yaw ~1.04 s, deadband al 15–22 %. S3: `config/calibration/s3_vehicle.json` (gain joystick 0.20) (†). |
| Test | 39 file, ~458 test; nessun artefatto di run salvato (†). |

### 2.5 RovTest — `<workstation>/RovTest`

| Voce | Dettaglio |
|---|---|
| Git | remote `https://github.com/AndreaBedei1/RovTest.git`; branch `master` allineato; HEAD `2cfe1fec4d14` (2026-06-18 19:24 +0200) "Go on". |
| Contenuto | `RovLibrary/` è una copia di `RovCode`. `RovLibrary_ros2_ws/` (ROS 2 Lyrical, pymavlink==2.4.49) contiene `rov_mavlink_bridge/backend/mavlink_backend.py:29-492` (`BlueRovCommander`/`MavlinkBackend`: connect, arm, `set_flight_mode`, RC override, stato thread-safe; indipendente da ROS) e `mavlink_bridge_node.py` (telemetria a 5 Hz, servizi arm/mode) (†). |
| Test | 22 test; log colcon solo per `rov_msgs`, "3 passed" (†). |
| Documentazione | `RovLibrary_ros2_ws/docs/SCIENTIFIC_SIM_TO_REAL_ROADMAP.md` (†). |

### 2.6 RovCode — `<workstation>/RovCode` (**nessun repository Git**) (†)
Teleop per periferiche (luci, laser, gripper, tilt) su `udpin:0.0.0.0:14550`. Invia RC_CHANNELS_OVERRIDE, DO_SET_SERVO/RELAY e MOUNT; **non invia** arm, mode, heartbeat o MANUAL_CONTROL (`rovlib/mavlink_core.py:48-226`, `rovlib/pilot.py`). `.venv` con pymavlink 2.4.49. Rispetto a `RovTest/RovLibrary/rovlib` è quasi identico: differisce solo `video.py`, e RovCode è la copia più recente. Rate misurati in `output/session_20260218_114143/meta/session_summary.json` (ATTITUDE ~16 Hz) e `output/teleop_session_20260219_131029` (~101 msg/s). Nessun test.

### 2.7 RoverLago — `<workstation>/RoverLago` (**nessun repository Git**) (†)
Documentazione e strumenti del sistema reale. Documenti `docs/00..14`; `00_SYSTEM_INVENTORY.md:72-79` riporta BlueOS 1.3.1, **ArduSub 4.1.2 `2dd0bb7d`**, Navigator/Pi4, FRAME_CONFIG=2 e 913 parametri.
- **Endpoint MAVLink**: udpout <TOPSIDE-IP>:14550, tcpin 5777, MAVLink2Rest 6040.
- **Failsafe**: FS_GCS_ENABLE=2 (2.5 s), FS_PILOT_INPUT=2 con timeout 3 s, RC_OVERRIDE_TIME 3 (`06_ARDUSUB_POSITIONING.md:30-35`).
- **Stream rate**: valori SR0 nel `.parm`; QGC porta EXTRA1 a 20.
- **Strumenti**: `tools/mavlink_logger.py` (solo ricezione TCP 5777, tlog e timesync), `ardusub_backup.py`, `preflight_check.py`, `blueos_snapshot.py`, pymavlink 2.4.49 vendorizzato in `tools/vendor/`.
- **Backup**: `backups/2026-10-05_pre-change/` contiene parametri, BIN/tlog e le API di BlueOS.
- **Misure**: ~130 msg/s, latenza di rete ~10 ms, offset BlueOS−PC −3.77 s (`11_DATA_LOGGING.md:62,106-109`).

### 2.8 Altri

| Cartella | Git | Note |
|---|---|---|
| `BlueROV2MultimodalRecorder` | remote `https://github.com/AndreaBedei1/BlueROV2MultimodalRecorder.git`; branch `feature/final-multimodal-recorder` (senza upstream; stesso commit di `origin/feature/high-throughput-recorder`); HEAD `4164f4f0209875fe` (2026-09-16 17:21) "Simplify recorder to camera Ping1D and ROVL acquisition"; | Registra camera, Ping1D e ROVL, senza MAVLink. 47 test, `lastfailed` `{}` (†). |
| `BlueROVDepthDemo` | nessun repository Git | Filtro video e hotkey per tilt/gripper; importa `RovCode/rovlib`; `udpin:14550`; `tests/test_depth_demo.py` (†). Da non eseguire: tocca l'hardware. |
| `HoloOceanLibrary` | remote `git@github.com:AndreaBedei1/HoloOceanLibrary.git`; `main` allineato; HEAD `10d4cfe2c301` (2026-01-07) "fix library and complete"; tag `v1.0.0`; pulito | Builder di scenari e demo da tastiera; `environment.yml` py3.9 non aggiornato; versione holoocean non fissata; 30 tps, `frames_per_sec True` (†). |
| `HoloOceanDataset` | remote `git@github.com:AndreaBedei1/HoloOceanDataset.git`; `main` ; HEAD `12b75807f88d` (2025-12-27) "Add the code to check the crash"; | Generatore di dataset; loop `tick/step` con watchdog di timeout (`record_dataset_submarine.py:155-232`) (†). |
| `Holoocean2-3` | nessun repository Git | Contiene il sorgente `HoloOcean-2.3.0/HoloOcean-2.3.0` (vedi §3) e note di altri progetti non pertinenti (†). |

## 3. Copie di HoloOcean (duplicati e conflitti)

| # | Percorso | Tipo | Versione / hash | Note |
|---|---|---|---|---|
| A | `%LOCALAPPDATA%\holoocean\2.3.0\worlds\Ocean` | Package mondi **installato** (usato da `holoocean.make`) | `config.json` version 2.3.0; `Holodeck.exe` SHA256 **`8c206c9c…7602253`** (255 731 200 B, 06/08) | Binario ufficiale **non patchato**: drag a ×0.01. È l'unico package installato (solo Ocean). |
| B | `<HOB>\packages\patched-after\Ocean` | Package assemblato: exe ricompilato più contenuti cooked ufficiali (hard link) | `Holodeck.exe` SHA256 **`4d6c2d46be8703758fbe4e9b62a99d4037aec0e220cca3a60b22b488d441684a`** (07/10 22:27); `holoenergy_build_manifest.json` | **Build con patch del drag**. Per usarla serve `Battery/tools/holoocean_backend.make_env(binary=...)`, perché `holoocean.make` standard punta ad A. |
| C | `<HOB>\packages\rebuilt-before\Ocean` | Build di controllo senza patch | SHA256 `f52999de…6b524` | I dati sono bit-identici ad A secondo il report. |
| D | `<HOB>\src\before` / `<HOB>\src\after` | Checkout sorgente v2.3.0 `49e70552dfd9` | `before` pulito; `after` ha `M engine/.../HolodeckBuoyantAgent.cpp` (patch applicata) | Usati da `tools/build_patched_holoocean.py`. Anche `<HOB>\outputs\{patched-after,rebuilt-before}\Holodeck.exe/.pdb`. |
| E | `Desktop\Holoocean2-3\HoloOcean-2.3.0\HoloOcean-2.3.0` | Sorgente 2.3.0 (zip, senza Git) | `engine/Source` identico a D-before; nel `client/` **mancano setup.py, pyproject, tox, Dockerfile** | Non installabile così com'è. |
| F | `Battery\build\holoocean-upstream` | Clone Git partial, senza checkout | tag v2.3.0 | Solo metadati. |
| G | Client Python in `holoocean_joystick` | site-packages | 2.3.0, installato da `Downloads\HoloOcean-2.3.0\client` (cartella rimossa) | I file Python sono identici a upstream v2.3.0: differiscono solo README, `.lcm` e `pylintrc`. |

Conflitti: le copie del sorgente sono coerenti tra loro. Il conflitto reale è **di binario**: A (non patchato) contro B (patchato). Tutto il codice esistente (Middleware, SimMatch, ObstacleAvoidance) usa A di default. Middleware non registra l'hash del binario nel manifest (`runlog.py:35-46`).

### 3.1 Patch del drag (solo localizzazione; l'audit fisico è affidato a un altro agente)

| File | SHA256 | Contenuto |
|---|---|---|
| `Battery/patches/holoocean-2.3-drag-units.patch` | `00f1b303…ab488fa` (da `patches/README.md`) | Un solo hunk su `engine/Source/Holodeck/HolodeckCore/Private/HolodeckBuoyantAgent.cpp` @@ -90,6 +90,8 (`ApplyBuoyancyDragForce`): aggiunge `DragForce *= UEUnitsPerMeter;` dopo l'equazione `-0.5*rho*|v|²*Cd*A`. Nel sorgente E/D-before l'equazione è alle righe 86-96 e la patch va prima di `AddForceAtLocation` (riga ~93). |
| `Battery/patches/holoocean-develop-drag-units.patch` | `3f5508df…711c02e` | Proposta per upstream `develop` `f3d1230c`: la stessa riga in `ApplySurfaceBuoyancy`, `ApplyUnderwaterBuoyancy` e `ApplyWavelessForces`, più l'aspettativa di `test_currents.py` (0.0632→6.32 m/s²). **Mai compilata.** |
| `Battery/patches/README.md` | — | Bug, fix, evidenze (rapporto applicato/atteso 0.0099999997 prima, 1.0000000074 dopo su 153 casi), procedura di applicazione/revert e build. Issue upstream #368. |
| `Battery/build/drag-patch-verification.json` | — | Prima prova: `apply`/`reverse` ok, `after_runtime_tested:false`. Superata dalla campagna del 7/10 (`docs/verified_backend_report.md`, `sources/verified_backend/`). |
| `<HOB>\src\after\...\HolodeckBuoyantAgent.cpp` | — | Patch **applicata** nel checkout (righe 91-94 verificate). |

Note per la Fase 2: con il drag corretto il passo esplicito a 30 Hz diventa critico (`timestep_report §4`). Middleware gira a 30 Hz, mentre il report raccomanda 100 Hz. SimMatch e ObstacleAvoidance impongono 30 tps (D-010).

## 4. Codice ArduSub / MAVLink esistente

| File | Ruolo | Trasporto | Messaggi inviati | Sicurezza |
|---|---|---|---|---|
| `Middleware/src/rov_middleware/backends/bluerov.py:25-214` | Telemetria passiva | HTTP GET mavlink2rest :6040 | nessuno | Blocco totale dell'attuazione (`:84-98`), freshness, EKF |
| `HoloObstacleAvoidance/scripts/real/rovlink.py:42-263` (†) | Libreria GCS pymavlink | `udpin:0.0.0.0:14550` | HEARTBEAT 1 Hz, MANUAL_CONTROL, SET_MODE, ARM/DISARM, SET_MESSAGE_INTERVAL, RC9 per le luci | Neutro e disarm all'uscita; `newest()` scarta la telemetria vecchia |
| `HoloObstacleAvoidance/scripts/real/cmd_bridge.py` (†) | Twist→MANUAL_CONTROL a 20 Hz | via RovLink | idem | Timeout 1 s verso neutro; interlock tramite variabili d'ambiente; PID file |
| `HoloObstacleAvoidance/src/rov_real_bridge/rov_real_bridge/real_control_live_node.py` (†) | Nodo ROS live | `mavlink_url` parametrico | MANUAL_CONTROL 20 Hz, HEARTBEAT 1 Hz | Interlock a 3 flag, fault latch, 5× neutro allo shutdown; **senza arm/mode** |
| `HoloObstacleAvoidance/src/rov_real_bridge/rov_real_bridge/command_mapping.py` (†), copia in `BlueROV2SimMatch/src/brsm/command_mapping.py` | Twist→counts | — | — | Fail-closed su NaN; deadband affine |
| `RovTest/RovLibrary_ros2_ws/src/rov_mavlink_bridge/rov_mavlink_bridge/backend/mavlink_backend.py` (†) | Backend pymavlink indipendente da ROS | `udpin:14550` | ARM, SET_MODE, RC override, servo, mount | Lock, stato della connessione |
| `RovCode/rovlib/*` (†) | Periferiche | `udpin:14550` | RC override, DO_SET_SERVO/RELAY | Clamp PWM |
| `RoverLago/tools/mavlink_logger.py` (†) | Logger | TCP 5777 in sola ricezione | nessuno | — |

Assenze: nessun client del protocollo **SITL JSON** di ArduPilot (UDP 9002/9003, pacchetti servo verso stato JSON), nessun `SIM_STATE`/`HIL_*`, nessun binario ArduSub SITL. Nessun client mavlink2rest in scrittura (POST/WebSocket).

## 5. Misure di tempo e timestep disponibili

| Misura | Valore | Fonte |
|---|---|---|
| Passo fisico UE contro clock client | `min(1/tps, 1/30)`; 20 Hz → 0.667; 30–200 Hz → 1.0 | `Battery/docs/holoocean_timestep_report.md:57-70` |
| Stabilità del drag corretto | Errore al primo passo −168 % a 30 Hz, −15 % a 100 Hz, −3.9 % a 200 Hz | idem `:94-100` |
| Wall-clock per step HoloOcean (free-run, senza viewport) | ~22.5 ms/step, cioè ~44 step/s | `Battery/logs/generic_benchmark/performance.json` |
| RTF a 30 tps | 2.10 senza camera; 0.77 con camera da 512 px; `make` 7.3 s | `BlueROV2SimMatch/results/env_verification.json:77-92` (†) |
| Rate ROS closed loop | 20.05 Hz (max dt 91 ms) | `BlueROV2SimMatch/results/ros_closed_loop/K0_seed000.json` (†) |
| Latenza reale MANUAL_CONTROL→PWM | 53–89 ms | `HoloObstacleAvoidance/docs/REAL_POOL_EXPERIMENT_LOG.md:16-27` (†) |
| Rete e clock del veicolo | ~10 ms; offset −3.77 s; ~130 msg/s | `RoverLago/docs/11_DATA_LOGGING.md` (†) |
| Rate della telemetria reale | ATTITUDE 12–16 Hz, HEARTBEAT 2–3.6 Hz | `mavlink_inventory.json`, `RovCode/output/...` (†) |
| Middleware | Solo tempo simulato con dt 1/30 costante; nessun wall-clock | `docs/session_validation.md:24` |

## 6. Manifest dei repository (JSON-like)

*Section omitted from the public copy: it indexes raw private evidence (device responses, file paths, hashes) kept outside the repository.*

## 7. Codice riutilizzabile per la Fase 2

| Componente | File:riga | Perché |
|---|---|---|
| API comune sim/reale | `Middleware/src/rov_middleware/api.py:22-113` | Contratto `VehicleBackend`/`VehicleState` già testato; basta aggiungere un `ArduSubBackend` (SITL o reale). |
| Frame NWU↔NED | `Middleware/src/rov_middleware/frames.py:1-31` | Conversione verificata da test, necessaria per lo stato JSON verso SITL (NED). |
| Lettura dei sensori HoloOcean e stato | `Middleware/src/rov_middleware/backends/holoocean.py:57-136` | Scenario, sensori sul socket COM, estrazione di pose/vel/gyro. Base dei pacchetti di stato per SITL. |
| Workaround per lo sleep di UE | `Middleware/.../holoocean.py:146-161`; `BlueROV2SimMatch/src/sim_server/runner.py:322-361` | Indispensabile con comandi a forza (scheme 0) che arrivano da SITL. |
| Geometria degli 8 thruster | `Middleware/.../holoocean.py:20-26`; `BlueROV2SimMatch/src/sim_server/vehicle_dynamics.py:32-99`; `Holoocean2-3/.../engine/Source/Holodeck/Agents/Private/BlueROV2.cpp:77-109` (†) | Mappatura dei canali motore di ArduSub (Vectored 6DOF Heavy) sui thruster di HoloOcean; ordine e segni ancora da verificare. |
| Lancio di un binario patchato | `Battery/tools/holoocean_backend.py:27-78` | `make_env(binary=…)` per usare `<HOB>\packages\patched-after` senza toccare l'installazione. |
| Check del backend | `Battery/tools/check_holoocean_backend.py` | Gate PASS/FAIL sul drag prima di ogni campagna. |
| Logging versionato e replay | `Middleware/src/rov_middleware/runlog.py:14-74`, `backends/replay.py` | Lo schema CSV/JSONL va esteso con hash del binario, tps e RTF. |
| Telemetria passiva | `Middleware/.../bluerov.py:25-214` | Parsing della cache mavlink2rest, freshness ed EKF, utili anche contro SITL. |
| Libreria GCS pymavlink | `HoloObstacleAvoidance/scripts/real/rovlink.py:42-263` (†); `RovTest/.../backend/mavlink_backend.py:29-492` (†) | Heartbeat, mode, arm, MANUAL_CONTROL e message interval: già pronta per `tcp:127.0.0.1:5760` (SITL). |
| Mapping Twist→MANUAL_CONTROL | `HoloObstacleAvoidance/src/rov_real_bridge/rov_real_bridge/command_mapping.py` (†) | Puro Python, fail-closed. |
| Interlock | `.../rov_real_bridge/safety_interlock.py` (†); `Middleware/.../bluerov.py:84-98` | Modello di sicurezza a più flag e blocco prima dell'I/O. |
| Trasporto tra processi | `BlueROV2SimMatch/src/sim_server/protocol.py`; `HoloObstacleAvoidance/.../sim_bridge_protocol.py` (†) | TCP framed in sola stdlib, tra l'env HoloOcean (py3.10) e ROS/SITL. |
| Misura RTF | `BlueROV2SimMatch/scripts/verify_env.py:220-266` (†); `Battery/tools/benchmark_energy.py:64-70` | Harness wall-clock riutilizzabile per il lockstep con SITL. |
| Logger reale e parametri | `RoverLago/tools/mavlink_logger.py`, `ardusub_backup.py`, backup `.parm` da 913 parametri (†) | Per allineare i parametri SITL al veicolo reale e misurare le latenze. |
| Timestep e drag | `Battery/tools/audit_holoocean_timestep.py`, `audit_holoocean_drag.py` | Verifica del passo e del drag su qualunque build. |

## 8. Incognite e verifiche pendenti

1. **ArduSub SITL non presente.** Va deciso dove compilarlo (WSL Ubuntu-24.04, non ispezionato, oppure Docker, assente). La versione deve corrispondere a 4.1.2 `2dd0bb7d`, che non ha il backend JSON moderno: verificare se il SITL "JSON" esiste in 4.1.x o se serve una release più recente.
2. Contenuto delle distro WSL (ArduPilot, toolchain) non verificato: sarebbe necessario avviarle.
3. Spazio disco: ~15 GB liberi; `<HOB>` (34 GB) e `<UE-5.3>` (~20 GB) occupano molto. Una build ArduPilot o altre build UE potrebbero non entrarci.
4. Binario da usare in Fase 2: ufficiale (A) o patchato (B, `4d6c2d46…`)? Tutti i risultati di Middleware, SimMatch e ObstacleAvoidance sono stati prodotti con A (drag ×0.01). Inoltre `UnrealEditor` non esiste, quindi non si possono cuocere mappe nuove: la build B riusa i contenuti ufficiali.
5. Tick rate: Middleware e altri progetti usano 30 Hz; il report sul timestep raccomanda 100 Hz con il drag corretto. Il loop SITL di ArduSub (400 Hz o più) richiede lockstep o sub-stepping: da progettare e misurare (RTF a 100 Hz non misurato).
6. Ordine e segni dei motori ArduSub (SERVO1-8 = 33..40, FRAME_CONFIG=2 Heavy) rispetto all'ordine dei thruster in HoloOcean (VertFS, VertFP, VertBP, VertBS, AngFS, AngFP, AngBP, AngBS): verifica documentale e con test.
7. Curva PWM→spinta (T200, 1100–1900): esistono dati T200 in `Battery/sources/T200-Public-Performance-Data-10-20V-September-2019.xlsx`, non ancora collegati a HoloOcean.
8. L'installazione HoloOcean non è riproducibile dalla sorgente locale: la cartella `Downloads\HoloOcean-2.3.0` è stata rimossa e nel `Holoocean2-3/client` manca il packaging. Le sorgenti complete sono in `<HOB>\src\before`.
9. Il manifest di Middleware non registra hash del binario, tps effettivo né wall-clock: da aggiungere prima delle campagne sim↔SITL.
10. Conflitto di porta 14550 tra QGC e gli script `udpin`: per SITL usare TCP 5760 o 5777 dedicato.
11. Failsafe: FS_PILOT_INPUT 3 s e FS_GCS 2.5 s devono essere replicati nei parametri SITL, e il bridge deve mantenere heartbeat e MANUAL_CONTROL.
12. Le voci (†) vengono da sotto-ricognizione automatica: ricontrollare riga per riga le citazioni prima di usarle nel report finale.
13. Discrepanza nel profilo S3 di HoloObstacleAvoidance (surge− 0.00046 nel doc contro 0.000106 nel JSON) e `command_latency_s: null` (†).
14. (voce rimossa dalla copia pubblica: stato locale dei repository.)

---

> **Nota di riconciliazione (2026-10-13, report finale):** il punto 8.1 ipotizzava che ArduSub 4.1.2 non avesse il backend JSON. `sitl-interface-spec.it.md` §11 lo smentisce: `SIM_JSON.cpp` esiste al tag `ArduSub-4.1.2` (`2dd0bb7d`), con funzionalità ridotte (solo 16 canali, magic 18458; `position` obbligatorio; nessun `no_time_sync`/`no_lockstep`). Il punto 8.5 (loop ArduSub 400 Hz) va letto come segue: `SCHED_LOOP_RATE=200` sul veicolo reale (backup `params_20261005T101049Z.parm`), 400 è il default SITL.
