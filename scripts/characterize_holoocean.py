#!/usr/bin/env python
"""Characterise the installed HoloOcean binary for the Phase 2 bridge.

Measures, with the stock BlueROV2 agent (control scheme 0, eight thruster
forces in newtons):

* per-tick wall-clock cost and achievable real-time factor at several rates;
* the physical integration step (from a known force applied from rest);
* the hydrodynamic drag actually applied (quadratic coefficient, compared with
  the value implied by the engine source, to detect the 0.01 unit defect);
* the IMU output at rest on the IMUSocket (specific-force convention);
* the force direction and the sign of the moment produced by each of the
  eight HoloOcean thruster indices.

The script never talks to ArduPilot or to any vehicle. It starts exactly one
HoloOcean instance at a time and closes it before the next one.

Usage:
    python scripts/characterize_holoocean.py --out results/characterization.json
"""
import argparse
import json
import platform
import time
from pathlib import Path

import numpy as np
import holoocean

MASS_KG = 11.5                       # BlueROV2.cpp
RHO = 997.0                          # HolodeckBuoyantAgent.h
CD, AREA = 0.8, 0.45                 # BlueROV2.cpp
K_QUAD_EXPECTED = 0.5 * RHO * CD * AREA / MASS_KG   # 1/m, for a correct drag


def scenario(tps, location=(0.0, 0.0, -5.0)):
    return {
        "name": "characterization",
        "world": "SimpleUnderwater",
        "package_name": "Ocean",
        "main_agent": "auv0",
        "ticks_per_sec": tps,
        "frames_per_sec": False,          # run as fast as possible
        "agents": [{
            "agent_name": "auv0",
            "agent_type": "BlueROV2",
            "sensors": [
                {"sensor_type": "DynamicsSensor", "socket": "COM",
                 "configuration": {"UseCOM": True, "UseRPY": True}},
                {"sensor_type": "IMUSensor", "socket": "IMUSocket"},
                {"sensor_type": "PoseSensor", "socket": "COM"},
                {"sensor_type": "DepthSensor", "socket": "DepthSocket"},
            ],
            "control_scheme": 0,
            "location": list(location),
            "rotation": [0.0, 0.0, 0.0],
        }],
        "window_width": 320,
        "window_height": 240,
    }


def dyn(state):
    d = np.asarray(state["DynamicsSensor"], dtype=float)
    return {"acc": d[0:3], "vel": d[3:6], "pos": d[6:9],
            "angacc": d[9:12], "angvel": d[12:15], "rpy": d[15:18]}


def reset_rest(env, loc):
    env.agents["auv0"].set_physics_state(list(loc), [0, 0, 0], [0, 0, 0], [0, 0, 0])
    s = None
    for _ in range(3):
        s = env.step(np.zeros(8))
    return s


def timing(tps, n_ticks):
    env = holoocean.make(scenario_cfg=scenario(tps), show_viewport=False, verbose=False)
    try:
        for _ in range(50):
            env.step(np.zeros(8))
        dts = np.empty(n_ticks)
        t_start = time.perf_counter()
        for i in range(n_ticks):
            t0 = time.perf_counter()
            env.step(np.zeros(8))
            dts[i] = time.perf_counter() - t0
        wall = time.perf_counter() - t_start
    finally:
        env.__exit__(None, None, None)
    sim = n_ticks / tps
    return {
        "ticks_per_sec": tps, "ticks": n_ticks, "wall_s": wall,
        "max_realtime_factor": sim / wall,
        "tick_ms_p50": float(np.percentile(dts, 50) * 1e3),
        "tick_ms_p95": float(np.percentile(dts, 95) * 1e3),
        "tick_ms_p99": float(np.percentile(dts, 99) * 1e3),
        "tick_ms_max": float(dts.max() * 1e3),
        "budget_ms": 1e3 / tps,
    }


def physics(tps):
    loc = (0.0, 0.0, -5.0)
    dt = 1.0 / tps
    out = {"ticks_per_sec": tps, "dt_client": dt}
    env = holoocean.make(scenario_cfg=scenario(tps, loc), show_viewport=False, verbose=False)
    try:
        agent = env.agents["auv0"]
        # 1. rest: neutral buoyancy, IMU convention
        s = reset_rest(env, loc)
        imu, pos0 = [], dyn(s)["pos"].copy()
        for _ in range(int(2 * tps)):
            s = env.step(np.zeros(8))
            imu.append(np.asarray(s["IMUSensor"], dtype=float))
        imu = np.asarray(imu)
        out["rest"] = {
            "duration_s": 2.0,
            "drift_m": (dyn(s)["pos"] - pos0).tolist(),
            "imu_accel_mean": imu[:, 0].mean(axis=0).tolist(),
            "imu_gyro_mean": imu[:, 1].mean(axis=0).tolist(),
            "depth_sensor": float(np.asarray(s["DepthSensor"]).ravel()[0]),
            "pose_z": float(np.asarray(s["PoseSensor"])[2, 3]),
        }
        # 2. integration step from a known vertical force applied from rest
        reset_rest(env, loc)
        f_each = 10.0
        s = env.step(np.array([f_each] * 4 + [0.0] * 4))
        dvz = dyn(s)["vel"][2]
        out["timestep"] = {
            "force_total_N": 4 * f_each,
            "dv_z": float(dvz),
            "dt_physical_est": float(dvz * MASS_KG / (4 * f_each)),
        }
        out["timestep"]["ratio_physical_over_client"] = out["timestep"]["dt_physical_est"] / dt
        # 3. drag: one free tick from an imposed surge velocity
        rows = []
        for v0 in (0.25, 0.5, 1.0, 1.5, 2.0):
            agent.set_physics_state(list(loc), [0, 0, 0], [v0, 0, 0], [0, 0, 0])
            s = env.step(np.zeros(8))
            v_before = dyn(s)["vel"][0]
            s = env.step(np.zeros(8))
            v_after = dyn(s)["vel"][0]
            rows.append((v_before, (v_before - v_after) / dt))
        rows = np.asarray(rows)
        # decel = c_lin * v + k_quad * v^2
        A = np.column_stack([rows[:, 0], rows[:, 0] ** 2])
        (c_lin, k_quad), *_ = np.linalg.lstsq(A, rows[:, 1], rcond=None)
        out["drag"] = {
            "samples_v_decel": rows.tolist(),
            "fit_linear_1_per_s": float(c_lin),
            "fit_quadratic_1_per_m": float(k_quad),
            "expected_quadratic_correct_drag": K_QUAD_EXPECTED,
            "applied_over_expected": float(k_quad / K_QUAD_EXPECTED),
        }
        # 4. per-thruster response from rest, one tick of 10 N
        thr = []
        for j in range(8):
            reset_rest(env, loc)
            u = np.zeros(8)
            u[j] = 10.0
            s = env.step(u)
            d = dyn(s)
            force_dir = d["vel"] * MASS_KG / (10.0 * dt)
            thr.append({
                "index": j,
                "force_per_N_world_flu": (force_dir).round(4).tolist(),
                "angvel_after_tick_deg_s": np.asarray(d["angvel"]).round(5).tolist(),
            })
        out["thrusters"] = thr
    finally:
        env.__exit__(None, None, None)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="results/characterization.json")
    ap.add_argument("--rates", default="30,100,200")
    ap.add_argument("--ticks", type=int, default=3000)
    args = ap.parse_args()
    rates = [int(r) for r in args.rates.split(",")]
    report = {
        "date_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "holoocean_client": holoocean.__version__,
        "python": platform.python_version(),
        "host": platform.platform(),
        "timing": [timing(r, args.ticks) for r in rates],
        "physics": [physics(r) for r in rates],
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
