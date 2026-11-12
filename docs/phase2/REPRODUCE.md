# Phase 2 — How to reproduce the ArduSub SITL ↔ HoloOcean experiments

Everything runs on one Linux workstation; no real vehicle, no network beyond
the loopback interface. Times are for the reference machine (Ubuntu 24.04,
23 threads, RTX 6000 Ada); any x86-64 Linux with a Vulkan-capable NVIDIA GPU
supported by HoloOcean 2.3.0 should work.

## 1. Software versions (as used)

| Component | Version / identifier |
|---|---|
| OS | Ubuntu 24.04, kernel 7.0.0-38, gcc 13.3.0 |
| Python | 3.9.25 (conda env `bluerov2-sim2real`), packages pinned in [`requirements-lock.txt`](../../requirements-lock.txt) |
| HoloOcean client | 2.3.0, git `49e70552dfd97273b7dfbe755fbe65d7738b24b7` (tag `v2.3.0`) |
| HoloOcean world package | `Ocean` 2.3.0 for Linux, from the official BYU server via `holoocean.install("Ocean")`; engine binary `Holodeck` SHA-256 `077983e4aceed352f76f063c0071b3d50534c52fd694c7cba18f82f8e0e728b5` — **stock build, with the drag unit defect** |
| ArduPilot | tag `ArduSub-4.1.2`, commit `2dd0bb7d4c85ac48437f139d66df648fc0e1d4ae` (same firmware as the vehicle) + [`patches/ardupilot-ArduSub-4.1.2-sitl-loopback.patch`](../../patches/ardupilot-ArduSub-4.1.2-sitl-loopback.patch) (SITL HAL/SIM only, see §3) |
| pymavlink | 2.4.41 |

Each run records the exact state in `bridge/bridge_manifest.json`
(repository commit and diff hash, HoloOcean client source, world package,
engine binary hash, GPU/driver), `sitl/sitl_manifest.txt` (ArduSub binary
hash, ArduPilot commit, patch and parameter-file hashes, full command line),
`mission/params.json` (all 900+ parameters actually in effect in SITL) and
`run_manifest.txt`. The bridge refuses to run on a dirty git tree unless
`--allow-dirty` is given (the diff is then saved with the run).

## 2. Python environment

```bash
conda create -y -n bluerov2-sim2real python=3.9
```

```bash
~/miniconda3/envs/bluerov2-sim2real/bin/pip install -r requirements-lock.txt
```

```bash
~/miniconda3/envs/bluerov2-sim2real/bin/python -c "import holoocean; holoocean.install('Ocean')"
```

The world package (~5 GB download, 5.4 GB on disk) is installed in
`~/.local/share/holoocean/2.3.0/`. Accepting the download implies accepting
the Unreal Engine EULA (printed by HoloOcean).

## 3. ArduSub 4.1.2 SITL

```bash
git clone --branch ArduSub-4.1.2 --depth 1 https://github.com/ArduPilot/ardupilot.git ardupilot-4.1.2
```

```bash
cd ardupilot-4.1.2 && git submodule update --init --recursive --depth 1
```

```bash
git apply /path/to/BlueROV2-Sim2Real/patches/ardupilot-ArduSub-4.1.2-sitl-loopback.patch
```

```bash
PATH=~/miniconda3/envs/bluerov2-sim2real/bin:$PATH ./waf configure --board sitl && PATH=~/miniconda3/envs/bluerov2-sim2real/bin:$PATH ./waf sub
```

The 4.1.2 `waf` needs Python < 3.12 and `empy==3.3.4` (both in the env); the
build is native, no `install-prereqs` script and no root access are needed.

The patch changes only the simulator side of ArduPilot, never the ArduSub
vehicle code:

* every listening socket of SITL (MAVLink TCP, RC input) and the JSON physics
  socket are bound to 127.0.0.1 (upstream binds 0.0.0.0, exposing them on the
  workstation's networks);
* the simulated barometer uses the full-precision position (upstream
  truncates the altitude to 1 cm before synthesising the pressure);
* the JSON backend rounds the per-frame time increment to the microsecond
  instead of truncating it (upstream drifts ~8 ms per 30 min at 200 Hz).

## 4. Running the tests

Unit tests (no simulator, ~15 s):

```bash
~/miniconda3/envs/bluerov2-sim2real/bin/python -m pytest -q
```

HoloOcean characterisation (one engine, ~1 min):

```bash
~/miniconda3/envs/bluerov2-sim2real/bin/python scripts/characterize_holoocean.py --out results/characterization.json
```

Closed-loop runs (one engine each). Set the environment once:

```bash
export ARDUPILOT_DIR=/path/to/ardupilot-4.1.2 PYTHON=~/miniconda3/envs/bluerov2-sim2real/bin/python
```

```bash
scripts/run_experiment.sh results/testA_mock depth --backend mock
```

```bash
scripts/run_experiment.sh results/testC axes
```

```bash
scripts/run_experiment.sh results/testDE depth
```

```bash
scripts/run_experiment.sh results/testF soak --minutes 31
```

Then evaluate each run (writes `report.json` and `overview.png` in the run folder):

```bash
$PYTHON scripts/analyze_run.py results/testC --test C
```

```bash
$PYTHON scripts/analyze_run.py results/testDE --test DE
```

```bash
$PYTHON scripts/analyze_run.py results/testF --test F
```

## 5. Ports, processes and shared-machine rules

* SITL instance `INSTANCE` (default 7) uses TCP 5830/5832/5833 (MAVLink,
  5760/5762/5763 + 10·I), UDP 5571 (RC in) and sends physics frames to UDP
  127.0.0.1:9072 (9002 + 10·I), where the bridge listens. The defaults
  9002/9003/9005 and 5760 are deliberately avoided (on the reference machine
  9002/9003/9005 were already used by another project).
* HoloOcean 2.3.0 on Linux uses no network ports: client and engine talk
  through POSIX shared memory and semaphores named with a random UUID per
  instance (`/dev/shm/HOLODECK_MEM<uuid>_*`), so instances do not collide.
* `run_experiment.sh` takes a per-user lock (one experiment at a time, hence
  one engine), refuses to start if two HoloOcean engines already run on the
  machine (any user), checks every port before starting, and on exit stops
  only the processes it started (bridge → orderly engine shutdown; an
  orphaned engine is stopped only if its PID is the one the bridge recorded).
* The bridge and the MAVLink client accept only 127.0.0.1; the client refuses
  to command an autopilot whose system id is not the SITL one (17).
* The SITL parameter overlay is for simulation only and must never be loaded
  on the vehicle (e.g. `MOT_n_DIRECTION` differ from the vehicle's).

## 6. Data produced

`results/` is git-ignored. Per run: `bridge/frames.csv` (one row per physics
step: PWM, logical thrusts, HoloOcean forces, saturation, allocation error,
state in NED/FRD, drag compensation), `bridge/events.jsonl`,
`bridge/summary.json`, `mission/mavlink.jsonl` (all autopilot telemetry),
`mission/marks.csv` (test phases in autopilot time), `mission/params.json`,
`resources.csv` (CPU/RAM of our processes, machine-wide GPU), logs.
