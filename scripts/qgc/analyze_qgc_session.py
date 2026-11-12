#!/usr/bin/env python
"""Evaluate a QGC GUI session from the router tlog (what QGC sent, what ArduSub did).

    python scripts/qgc/analyze_qgc_session.py results/p3_qgc
"""
import csv
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
import bluerov2_sim2real.interface  # noqa: E402,F401
from bluerov2_sim2real.interface.contract import MODE_NAMES  # noqa: E402
from bluerov2_sim2real.interface.tlog import read_tlog  # noqa: E402

EXPECT = {   # stick label -> (MANUAL_CONTROL field, sign), (check on the autopilot side)
    "forward": ("x", +1), "backward": ("x", -1), "lateral_right": ("y", +1), "lateral_left": ("y", -1),
    "ascend": ("z", +1), "descend": ("z", -1), "yaw_right": ("r", +1), "yaw_left": ("r", -1),
}
MIX = {"forward": [-1, -1, +1, +1], "backward": [+1, +1, -1, -1], "lateral_right": [+1, -1, +1, -1],
       "lateral_left": [-1, +1, -1, +1], "yaw_right": [+1, -1, -1, +1], "yaw_left": [-1, +1, +1, -1]}


def main():
    run = Path(sys.argv[1])
    msgs = list(read_tlog(run / "router" / "traffic.tlog"))
    marks = list(csv.DictReader(open(run / "qgc_gui" / "marks.csv")))
    mc = [(t, m) for t, m in msgs if m.get_type() == "MANUAL_CONTROL" and m.get_srcSystem() == 255]
    srv = [(t, m) for t, m in msgs if m.get_type() == "SERVO_OUTPUT_RAW" and m.get_srcSystem() == 17]
    gpi = [(t, m) for t, m in msgs if m.get_type() == "GLOBAL_POSITION_INT" and m.get_srcSystem() == 17]
    att = [(t, m) for t, m in msgs if m.get_type() == "ATTITUDE" and m.get_srcSystem() == 17]
    hb = [(t, m) for t, m in msgs if m.get_type() == "HEARTBEAT" and m.get_srcSystem() == 17 and m.get_srcComponent() == 1]
    acks = [(t, m.command, m.result) for t, m in msgs if m.get_type() == "COMMAND_ACK"]
    checks = []

    def win(lst, t0, t1):
        return [m for t, m in lst if t0 <= t <= t1]

    starts = {}
    for row in marks:
        t, ev, val = float(row["wall_time"]), row["event"], row["value"]
        if ev == "stick_hold_start":
            starts[val] = t
        elif ev == "stick_hold_end":
            t0 = starts[val]
            fld, sign = EXPECT[val]
            cmds = win(mc, t0, t)
            vals = [getattr(m, fld) for m in cmds]
            neutral = 500 if fld == "z" else 0
            ok_cmd = bool(vals) and all((v - neutral) * sign > 0 for v in vals[len(vals) // 2:])
            rate = len(cmds) / max(t - t0, 1e-6)
            entry = {"check": f"qgc_stick_{val}", "manual_control_field": fld,
                     "values_last": vals[-1] if vals else None, "rate_hz": round(rate, 1)}
            s = win(srv, t0 + 0.5, t)
            if val in MIX and s:
                m = s[-1]
                out = [getattr(m, f"servo{i}_raw") for i in range(1, 5)]
                entry["servo1_4"] = out
                ok_mix = all((o - 1500) * e > 0 for o, e in zip(out, MIX[val]))
            else:
                g = win(gpi, t0, t + 2)
                d = [-x.relative_alt / 1000 for x in g]
                entry["depth_change_m"] = round(d[-1] - d[0], 3) if d else None
                ok_mix = bool(d) and (d[-1] - d[0]) * (-sign) > 0.03
            if val.startswith("yaw"):
                a = win(att, t0, t)
                yr = [x.yawspeed for x in a]
                entry["yaw_rate_extreme"] = round(max(yr, key=abs), 3) if yr else None
            entry["pass"] = ok_cmd and ok_mix
            checks.append(entry)
    armed = [bool(m.base_mode & 128) for _, m in hb]
    modes = []
    for _, m in hb:
        name = MODE_NAMES.get(m.custom_mode, m.custom_mode)
        if not modes or modes[-1] != name:
            modes.append(name)
    checks.append({"check": "qgc_arm_then_disarm", "pass": True in armed and armed[-1] is False,
                   "arm_disarm_acks": [(c, r) for _, c, r in acks if c == 400]})
    checks.append({"check": "qgc_mode_sequence", "pass": modes[-3:] == ["STABILIZE", "ALT_HOLD", "MANUAL"]
                   or ["STABILIZE", "ALT_HOLD", "MANUAL"] == [x for x in modes if x != "MANUAL" or True][-3:],
                   "modes_seen": modes})
    z_neutral = [m.z for t, m in mc[:50]]
    checks.append({"check": "qgc_throttle_neutral_is_500", "pass": z_neutral and all(z == 500 for z in z_neutral),
                   "first_values": z_neutral[:5]})
    src = sorted({(m.get_srcSystem(), m.get_srcComponent(), m.target) for _, m in mc})
    checks.append({"check": "qgc_manual_control_source_target", "pass": src == [(255, 190, 17)], "src_comp_target": src})
    out = {"checks": checks, "passed": sum(c["pass"] for c in checks), "total": len(checks),
           "manual_control_messages": len(mc)}
    (run / "qgc_session_report.json").write_text(json.dumps(out, indent=2, default=str))
    for c in checks:
        print(("PASS" if c["pass"] else "FAIL"), {k: v for k, v in c.items() if k != "pass"})
    print(f"{out['passed']}/{out['total']} passed")


if __name__ == "__main__":
    main()
