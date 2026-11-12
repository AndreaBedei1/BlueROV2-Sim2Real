#!/usr/bin/env python
"""Evaluate a Phase 2 run (bridge frames + MAVLink log + marks) and write report.json and plots.

    python scripts/analyze_run.py results/testC --test C
    python scripts/analyze_run.py results/testE --test DE
    python scripts/analyze_run.py results/testF --test F
"""
import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from bluerov2_sim2real.thrusters import LogicalVehicle  # noqa: E402


def load_frames(run):
    with open(run / "bridge" / "frames.csv") as f:
        r = csv.reader(f)
        head = next(r)
        data = np.array([[float(x) for x in row] for row in r])
    return {h: data[:, i] for i, h in enumerate(head)}


def load_marks(run):
    with open(run / "mission" / "marks.csv") as f:
        return [(int(r["time_boot_ms"]) / 1e3, r["event"], r["value"]) for r in csv.DictReader(f)]


def load_mav(run, types):
    out = {t: [] for t in types}
    with open(run / "mission" / "mavlink.jsonl") as f:
        for line in f:
            d = json.loads(line)
            if d["mavpackettype"] in out:
                out[d["mavpackettype"]].append(d)
    return out


def window(F, t0, t1):
    return (F["t_sim"] >= t0) & (F["t_sim"] <= t1)


def body_velocity(F, idx):
    """NED velocity rotated into FRD using the logged Euler angles."""
    out = []
    for i in np.flatnonzero(idx):
        r, p, y = np.radians([F["roll_deg"][i], F["pitch_deg"][i], F["yaw_deg"][i]])
        cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
        R = np.array([[cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
                      [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
                      [-sp, sr * cp, cr * cp]])
        out.append(R.T @ [F["vn"][i], F["ve"][i], F["vd"][i]])
    return np.array(out)


def clock_check(F, mav):
    """Compare SITL SERVO_OUTPUT_RAW (time_usec = SITL sim time) with the bridge PWM log."""
    srv = mav["SERVO_OUTPUT_RAW"]
    # keep only samples where the outputs changed since the previous sample: a constant PWM proves nothing
    srv = [s for k, s in enumerate(srv) if k > 0 and any(s[f"servo{i}_raw"] != srv[k - 1][f"servo{i}_raw"] for i in range(1, 9))]
    t = F["t_sim"]
    pwm = np.column_stack([F[f"pwm{i}"] for i in range(1, 9)])
    dt = float(np.median(np.diff(t)))
    res = {}
    for offset_frames in (-3, -2, -1, 0, 1, 2, 3):
        ok = n = 0
        for s in srv:
            ts = s["time_usec"] / 1e6
            k = int(np.argmin(np.abs(t - ts))) + offset_frames
            if abs(t[int(np.argmin(np.abs(t - ts)))] - ts) <= 0.5 * dt and 0 <= k < len(t):
                n += 1
                ok += int(np.all(pwm[k] == [s[f"servo{i}_raw"] for i in range(1, 9)]))
        res[offset_frames] = (ok, n)
    top = max(v[0] for v in res.values())
    best = [o for o, v in res.items() if v[0] == top]
    n = res[0][1]
    return {"changing_samples": n, "offsets_with_best_match": best,
            "best_match_fraction": top / n if n else None,
            "note": "row t_sim is the time after the step that applied the row's PWM; SITL time_usec "
                    "of SERVO_OUTPUT_RAW is the time the output was computed, i.e. one frame earlier",
            "all_offsets": {str(k): v for k, v in res.items()}}


def eval_c(run, F, marks):
    v = LogicalVehicle()
    out = {"motor_test": [], "pilot": []}
    starts = {}
    for t, ev, val in marks:
        if ev in ("motor_test_start", "pilot_start"):
            starts[ev] = (t, val)
        elif ev in ("motor_test_end", "pilot_end"):
            t0, val0 = starts.pop(ev.replace("end", "start"))
            w = window(F, t0, t + 0.2)
            pwm = np.column_stack([F[f"pwm{i}"] for i in range(1, 9)])[w]
            vb = body_velocity(F, w)
            dv = vb.mean(axis=0) - vb[0]          # mean response relative to the state at command start
            g = np.column_stack([F["gx"], F["gy"], F["gz"]])[w]
            gmean = g.mean(axis=0)
            dd = F["d"][w][-1] - F["d"][w][0]
            if ev == "motor_test_end":
                m = int(val0.split(":")[0])
                active = [i + 1 for i in range(8) if np.any(pwm[:, i] != 1500)]
                # initial response: 0.3 s from the first frame in which the motor output is active,
                # before the induced rotation makes body-frame velocities hard to interpret
                widx = np.flatnonzero(w)
                on = widx[np.any(np.column_stack([F[f"pwm{i}"] for i in range(1, 9)])[widx] != 1500, axis=1)]
                if on.size:
                    w = window(F, F["t_sim"][on[0]], F["t_sim"][on[0]] + 0.3)
                    vb = body_velocity(F, w)
                    dv = vb[-1] - vb[0]
                    g = np.column_stack([F["gx"], F["gy"], F["gz"]])[w]
                    gmean = g.mean(axis=0)
                    dd = F["d"][w][-1] - F["d"][w][0]
                col = v.B[:, m - 1]
                exp_dir = v.directions[m - 1]
                checks = {"only_channel_m_active": active == [m]}
                if m <= 4:
                    horiz = dv[0:2]
                    cosang = float(np.dot(horiz, exp_dir[0:2]) / (np.linalg.norm(horiz) * np.linalg.norm(exp_dir[0:2]) + 1e-12))
                    checks["velocity_along_expected_direction"] = cosang > 0.9
                    checks["yaw_rate_sign"] = bool(np.sign(gmean[2]) == np.sign(col[5]))
                    detail = {"dv_body_xy": horiz.tolist(), "expected_dir_xy": exp_dir[0:2].tolist(),
                              "cos_angle": cosang, "mean_gyro_z": float(gmean[2]), "expected_yaw_sign": float(np.sign(col[5]))}
                else:
                    checks["depth_increases"] = dd > 0      # logical verticals push down for a positive command
                    checks["roll_rate_sign"] = bool(np.sign(gmean[0]) == np.sign(col[3]))
                    checks["pitch_rate_sign"] = bool(np.sign(gmean[1]) == np.sign(col[4]))
                    detail = {"depth_change": float(dd), "mean_gyro": gmean.tolist(),
                              "expected_roll_sign": float(np.sign(col[3])), "expected_pitch_sign": float(np.sign(col[4]))}
                out["motor_test"].append({"motor": m, "active_channels": active, "checks": checks, **detail,
                                          "pass": all(checks.values())})
            else:
                axis, val = val0.split(":")
                val = int(val)
                if axis == "x":
                    ok, meas = (dv[0] > 0) == (val > 0), float(dv[0])
                elif axis == "y":
                    ok, meas = (dv[1] > 0) == (val > 0), float(dv[1])
                elif axis == "z":
                    ok, meas = (dd < 0) == (val > 500), float(dd)          # stick up -> shallower
                else:
                    ok, meas = (gmean[2] > 0) == (val > 0), float(gmean[2])
                out["pilot"].append({"axis": axis, "stick": val, "measured": meas, "pass": bool(ok)})
    gz = np.abs(F["gz"])
    out["max_abs_yaw_rate_rad_s"] = float(gz.max())
    out["note"] = ("expected directions come from the same logical vehicle model the bridge uses: the test "
                   "verifies channel order, allocation and sign plumbing end-to-end, not the vehicle model. "
                   "Rotation magnitudes are a HoloOcean model property (no rotational hydrodynamic damping).")
    out["pass"] = all(x["pass"] for x in out["motor_test"]) and all(x["pass"] for x in out["pilot"]) \
        and len(out["motor_test"]) == 8 and len(out["pilot"]) == 8
    return out


E_CRITERIA = {   # fixed before the final runs (2026-11-12, after the independent reviews)
    "settle_band_m": "max(0.05 m, 5 % of the step)",
    "max_settling_s": 20.0, "max_overshoot": "max(0.05 m, 25 % of the step)",
    "max_final_abs_mean_error_m": 0.02, "max_abs_roll_pitch_deg": 2.0, "max_saturated_fraction": 0.01,
}


def eval_de(run, F, marks):
    ev = {e: t for t, e, _ in marks if e in ("test_D_start", "test_D_end", "test_E_start", "test_E_end")}
    dt = float(np.median(np.diff(F["t_sim"])))
    res = {}
    w = window(F, ev["test_D_start"], ev["test_D_end"])
    res["D1_manual_neutral"] = {
        "duration_s": float(F["t_sim"][w][-1] - F["t_sim"][w][0]),
        "depth_start": float(F["d"][w][0]), "depth_end": float(F["d"][w][-1]),
        "max_abs_horizontal_drift_m": float(np.max(np.hypot(F["n"][w] - F["n"][w][0], F["e"][w] - F["e"][w][0]))),
        "max_abs_depth_change_m": float(np.max(np.abs(F["d"][w] - F["d"][w][0]))),
        "max_abs_roll_deg": float(np.max(np.abs(F["roll_deg"][w]))),
        "max_abs_pitch_deg": float(np.max(np.abs(F["pitch_deg"][w]))),
        "yaw_change_deg": float(np.ptp(F["yaw_deg"][w])),
        "pwm_all_neutral": bool(all(np.all(F[f"pwm{i}"][w] == 1500) for i in range(1, 9))),
        "meaning": "link adds no spurious force when ArduSub outputs neutral (cannot detect controller issues)",
    }
    d1 = res["D1_manual_neutral"]
    d1["pass"] = d1["pwm_all_neutral"] and d1["max_abs_depth_change_m"] < 0.01 \
        and d1["max_abs_horizontal_drift_m"] < 0.01 and d1["max_abs_roll_deg"] < 0.5 and d1["max_abs_pitch_deg"] < 0.5
    targets = [(t, float(v)) for t, e, v in marks if e == "depth_target"]
    t_first = targets[0][0] if targets else ev["test_E_end"]
    w = window(F, ev["test_E_start"], t_first)
    d0 = F["d"][w][0]
    res["D2_althold_entry_hold"] = {
        "duration_s": float(F["t_sim"][w][-1] - F["t_sim"][w][0]), "depth_at_entry": float(d0),
        "max_abs_depth_deviation_m": float(np.max(np.abs(F["d"][w] - d0))),
        "final_depth_deviation_m": float(F["d"][w][-1] - d0),
        "max_abs_roll_deg": float(np.max(np.abs(F["roll_deg"][w]))),
        "max_abs_pitch_deg": float(np.max(np.abs(F["pitch_deg"][w]))),
        "yaw_change_deg": float(np.ptp(F["yaw_deg"][w])),
        "meaning": "closed-loop hold right after mode entry, no target command",
    }
    d2 = res["D2_althold_entry_hold"]
    d2["pass"] = d2["max_abs_depth_deviation_m"] < 0.10 and d2["max_abs_roll_deg"] < 2 and d2["max_abs_pitch_deg"] < 2
    res["D"] = {"pass": d1["pass"] and d2["pass"]}
    steps = []
    for k, (t0, d) in enumerate(targets):
        t1 = targets[k + 1][0] if k + 1 < len(targets) else ev["test_E_end"]
        w = window(F, t0, t1)
        t, depth = F["t_sim"][w] - t0, F["d"][w]
        d_start = depth[0]
        amp = abs(d - d_start)
        err = depth - d
        band = max(0.05, 0.05 * amp)
        outside = np.flatnonzero(np.abs(err) > band)
        settle = None
        if outside.size == 0:
            settle = 0.0
        elif outside[-1] < len(err) - 1:
            settle = float(t[outside[-1] + 1])
        last = t > t[-1] - 10.0
        overshoot = float(max(0.0, np.max((depth - d) * np.sign(d - d_start)))) if amp > 0 else 0.0
        k5 = min(len(depth) - 1, int(round(5.0 / dt)))
        s = {
            "target_m": d, "start_depth_m": float(d_start), "duration_s": float(t[-1]),
            "moved_in_correct_direction": bool(np.sign(depth[k5] - d_start) == np.sign(d - d_start)) if amp > 0.05 else True,
            "settling_time_s": settle, "overshoot_m": overshoot,
            "final10s_mean_error_m": float(err[last].mean()), "final10s_rms_error_m": float(np.sqrt((err[last] ** 2).mean())),
            "max_abs_roll_deg": float(np.max(np.abs(F["roll_deg"][w]))),
            "max_abs_pitch_deg": float(np.max(np.abs(F["pitch_deg"][w]))),
            "saturated_fraction": float(F["saturated"][w].mean()),
        }
        s["pass"] = (s["moved_in_correct_direction"] and settle is not None and settle <= E_CRITERIA["max_settling_s"]
                     and overshoot <= max(0.05, 0.25 * amp)
                     and abs(s["final10s_mean_error_m"]) <= E_CRITERIA["max_final_abs_mean_error_m"]
                     and max(s["max_abs_roll_deg"], s["max_abs_pitch_deg"]) <= E_CRITERIA["max_abs_roll_pitch_deg"]
                     and s["saturated_fraction"] <= E_CRITERIA["max_saturated_fraction"])
        steps.append(s)
    res["E"] = {"criteria": E_CRITERIA, "steps": steps, "pass": bool(steps) and all(s["pass"] for s in steps)}
    return res


def eval_f(run, F, marks, summary):
    ev = {e: t for t, e, _ in marks if e in ("test_F_start", "test_F_end")}
    w = window(F, ev["test_F_start"], ev["test_F_end"])
    dt = np.diff(F["t_sim"][w])
    wall = F["wall_s"][w]
    gaps = F["rx_gap_ms"][w][1:]
    # drift between simulated and wall time over the soak (should be bounded, SITL paces at speedup 1)
    sim_minus_wall = (F["t_sim"][w] - F["t_sim"][w][0]) - (wall - wall[0])
    blocks = []
    tt = F["t_sim"][w]
    for b0 in np.arange(tt[0], tt[-1] - 60, 60):
        bw = (tt >= b0) & (tt < b0 + 60)
        blocks.append(float((tt[bw][-1] - tt[bw][0]) / (wall[bw][-1] - wall[bw][0])))
    return {
        "duration_sim_s": float(tt[-1] - tt[0]), "duration_wall_s": float(wall[-1] - wall[0]),
        "sim_over_wall": float((tt[-1] - tt[0]) / (wall[-1] - wall[0])),
        "sim_over_wall_per_minute": {"min": min(blocks), "max": max(blocks), "mean": float(np.mean(blocks))},
        "sim_minus_wall_s": {"final": float(sim_minus_wall[-1]), "min": float(sim_minus_wall.min()), "max": float(sim_minus_wall.max())},
        "sim_dt": {"min": float(dt.min()), "max": float(dt.max()), "unique": sorted(set(np.round(dt, 9).tolist()))[:5]},
        "frame_count_steps": sorted(set(np.diff(F["frame_count"][w]).astype(int).tolist())),
        "rx_gap_ms": {"mean": float(gaps.mean()), "std": float(gaps.std()), "p99": float(np.percentile(gaps, 99)),
                      "p99_9": float(np.percentile(gaps, 99.9)), "max": float(gaps.max()), "n_over_50ms": int((gaps > 50).sum()),
                      "n_over_200ms": int((gaps > 200).sum())},
        "bridge_summary": {k: summary.get(k) for k in ["duplicates_resent", "lost_frames", "sitl_resets", "bad_packets", "rx_timeouts", "saturated_steps"]},
    }


def plot(run, F, marks, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    t = F["t_sim"]
    ax[0].plot(t, F["d"], label="simulated depth (CoM)")
    tg = [(tm, float(v)) for tm, e, v in marks if e == "depth_target"]
    if tg:
        xs, ys = [], []
        for k, (tm, d) in enumerate(tg):
            xs += [tm, tg[k + 1][0] if k + 1 < len(tg) else t[-1]]
            ys += [d, d]
        ax[0].plot(xs, ys, "k--", label="requested depth")
    ax[0].invert_yaxis()
    ax[0].set_ylabel("depth [m]")
    ax[0].legend(loc="best")
    for i in range(1, 9):
        ax[1].plot(t, F[f"pwm{i}"], lw=0.8, label=f"M{i}")
    ax[1].set_ylabel("PWM [us]")
    ax[1].legend(ncol=8, fontsize=7)
    for k in ("roll_deg", "pitch_deg"):
        ax[2].plot(t, F[k], label=k)
    ax[2].set_ylabel("attitude [deg]")
    ax[2].legend()
    ax[3].plot(t[1:], F["rx_gap_ms"][1:], lw=0.5)
    ax[3].set_ylabel("SITL frame gap [ms]")
    ax[3].set_xlabel("simulated time [s]")
    for tm, e, v in marks:
        if e in ("mode", "armed", "disarmed", "test_D_start", "test_E_start", "test_F_start"):
            for a in ax:
                a.axvline(tm, color="0.8", lw=0.6)
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(run / "overview.png", dpi=110)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--test", choices=["C", "DE", "F"], required=True)
    a = ap.parse_args()
    run = Path(a.run)
    F = load_frames(run)
    marks = load_marks(run)
    mav = load_mav(run, ["SERVO_OUTPUT_RAW", "STATUSTEXT"])
    summary = json.loads((run / "bridge" / "summary.json").read_text())
    rep = {"bridge_summary": summary, "clock_check": clock_check(F, mav),
           "statustext": sorted({m["text"] for m in mav["STATUSTEXT"]})}
    if a.test == "C":
        rep["C"] = eval_c(run, F, marks)
    elif a.test == "DE":
        rep.update(eval_de(run, F, marks))
    else:
        rep["F"] = eval_f(run, F, marks, summary)
    (run / "report.json").write_text(json.dumps(rep, indent=2, default=float))
    plot(run, F, marks, f"{run.name}: test {a.test}")
    print(json.dumps({k: v for k, v in rep.items() if k != "bridge_summary"}, indent=2, default=float))


if __name__ == "__main__":
    main()
