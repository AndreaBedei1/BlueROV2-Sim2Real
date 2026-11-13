#!/usr/bin/env python
"""Drive the isolated QGroundControl 4.2.8 GUI like an operator (SITL only).

Uses xdotool on the PRIVATE display started by run_stack.sh --qgc private,
with QGC's on-screen virtual joystick (QGC_VIRTUAL_JOYSTICK=true). Every
action is marked with its wall-clock time in ``marks.csv`` and documented by
a screenshot; the effect is evaluated afterwards from the router's tlog
(analyze_qgc_session.py), i.e. from what QGC really sent and what ArduSub
really did.

Coordinates refer to QGC 4.2.8 at 1024x768 (the script resizes the window).
"""
import argparse
import csv
import os
import subprocess
import time
from pathlib import Path

XDOTOOL = os.environ.get("XDOTOOL", "xdotool")
LEFT_STICK = (98, 474)     # throttle (up/down), yaw (left/right)
RIGHT_STICK = (924, 474)   # forward (up/down), lateral (left/right)
ARM_MENU = (528, 27)
MODE_MENU = (383, 27)
MODE_ITEMS = {"Manual": 61, "Stabilize": 100, "Depth Hold": 178}
SLIDER = (419, 731)


class Gui:
    def __init__(self, out: Path, run_dir: Path):
        # Safety: only ever drive the isolated QGC started by run_stack on its private display.
        xauth = os.environ.get("XAUTHORITY", "")
        if not xauth.startswith(str(run_dir.resolve()) + "/display/"):
            raise SystemExit("refusing: not the private display of this run (could be a desktop QGC)")
        self.qgc_pid = int((run_dir / "qgc" / "qgc.pid").read_text())
        self.out = out
        out.mkdir(parents=True, exist_ok=True)
        self.win = self._find_window()
        self.x("windowsize", self.win, "1024", "768")
        time.sleep(1.0)
        self.marks = open(out / "marks.csv", "w", newline="", buffering=1)
        self.w = csv.writer(self.marks)
        self.w.writerow(["wall_time", "event", "value"])
        self.n = 0

    def x(self, *args):
        return subprocess.run([XDOTOOL, *map(str, args)], check=True, capture_output=True, text=True).stdout

    def _find_window(self):
        """Largest top-level window named QGroundControl (xdotool --name misses the QML main window)."""
        import re
        for _ in range(90):
            tree = subprocess.run(["xwininfo", "-root", "-tree"], capture_output=True, text=True).stdout
            best = None
            for m in re.finditer(r"(0x[0-9a-f]+) \"QGroundControl\": \([^)]*\)\s+(\d+)x(\d+)", tree):
                area = int(m.group(2)) * int(m.group(3))
                if area > 400 * 300 and (best is None or area > best[1]):
                    best = (m.group(1), area)
            if best:
                pid = subprocess.run([XDOTOOL, "getwindowpid", str(int(best[0], 16))],
                                     capture_output=True, text=True).stdout.strip()
                if pid != str(self.qgc_pid):
                    raise SystemExit(f"refusing: window belongs to pid {pid}, not this run's QGC {self.qgc_pid}")
                return best[0]
            time.sleep(1)
        raise RuntimeError("QGroundControl main window not found")

    def mark(self, event, value=""):
        self.w.writerow([f"{time.time():.3f}", event, value])
        print(f"[qgc-gui] {event} {value}", flush=True)

    def shot(self, name):
        self.n += 1
        subprocess.run(["import", "-window", self.win, str(self.out / f"{self.n:02d}_{name}.png")], check=True)

    def click(self, xy):
        self.x("mousemove", "--window", self.win, xy[0], xy[1], "click", "1")
        time.sleep(1.2)

    def slide_confirm(self):
        self.x("mousemove", "--window", self.win, *SLIDER)
        time.sleep(0.3)
        self.x("mousedown", "1")
        time.sleep(0.3)
        for _ in range(40):
            self.x("mousemove_relative", "4", "0")
            time.sleep(0.03)
        time.sleep(0.3)
        self.x("mouseup", "1")
        time.sleep(2.0)

    def toolbar_menus(self):
        """Centres of the two outlined toolbar drop-downs (mode, arm), found in a screenshot.

        Their position depends on the width of the current mode label, so fixed
        coordinates are wrong after a mode change.
        """
        import numpy as np
        from PIL import Image
        tmp = self.out / "_toolbar.png"
        subprocess.run(["import", "-window", self.win, str(tmp)], check=True)
        img = np.asarray(Image.open(tmp).convert("L")).astype(int)
        top, bottom = 14, 40               # the outlined boxes span roughly y = 14..40
        bright = img[top:bottom, :700] > 200
        cols = [x for x in range(bright.shape[1]) if bright[:, x].mean() > 0.8]   # vertical white borders
        edges = []
        for x in cols:
            if not edges or x - edges[-1][-1] > 1:
                edges.append([x])
            else:
                edges[-1].append(x)
        xs = [int(sum(e) / len(e)) for e in edges]
        if len(xs) < 4:
            raise RuntimeError(f"toolbar menus not found (edges {xs})")
        (m0, m1), (a0, a1) = (xs[-4], xs[-3]), (xs[-2], xs[-1])     # the last two outlined boxes
        return ((m0 + m1) // 2, 27), ((a0 + a1) // 2, 27)

    def arm(self, arm=True):
        _, arm_menu = self.toolbar_menus()
        self.click(arm_menu)
        self.click((arm_menu[0] - 19, 61 if arm else 100))
        self.shot("confirm_" + ("arm" if arm else "disarm"))
        self.mark("slide_confirm", "arm" if arm else "disarm")
        self.slide_confirm()
        self.shot("armed" if arm else "disarmed")

    def mode(self, name):
        mode_menu, _ = self.toolbar_menus()
        self.click(mode_menu)
        self.mark("mode_select", name)
        self.click((mode_menu[0], MODE_ITEMS[name]))
        time.sleep(1.0)
        self.shot("mode_" + name.replace(" ", "_"))

    def stick(self, which, dx, dy, hold_s=3.0, label=""):
        cx, cy = LEFT_STICK if which == "left" else RIGHT_STICK
        self.x("mousemove", "--window", self.win, cx, cy)
        time.sleep(0.3)
        self.x("mousedown", "1")
        steps = 15
        for _ in range(steps):
            self.x("mousemove_relative", "--", int(dx / steps), int(dy / steps))
            time.sleep(0.02)
        self.mark("stick_hold_start", label)
        time.sleep(hold_s)
        self.shot("stick_" + label)
        self.mark("stick_hold_end", label)
        self.x("mouseup", "1")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=None)
    ap.add_argument("--settle", type=float, default=45.0, help="seconds to wait for QGC + vehicle release")
    a = ap.parse_args()
    run_dir = Path(os.environ["RUN_DIR"])
    out = Path(a.out or run_dir / "qgc_gui")
    time.sleep(a.settle)
    g = Gui(out, run_dir)
    g.shot("connected")
    g.mark("start")
    g.arm(True)
    g.mark("armed_done")
    time.sleep(3)
    for which, dx, dy, label in [("right", 0, -45, "forward"), ("right", 0, 45, "backward"),
                                 ("right", 45, 0, "lateral_right"), ("right", -45, 0, "lateral_left"),
                                 ("left", 0, -45, "ascend"), ("left", 0, 45, "descend"),
                                 ("left", 45, 0, "yaw_right"), ("left", -45, 0, "yaw_left")]:
        g.stick(which, dx, dy, 3.0, label)
        time.sleep(4.0)
    g.mode("Stabilize")
    time.sleep(8)
    g.mode("Depth Hold")
    g.mark("depth_hold_entered")
    time.sleep(25)
    g.mode("Manual")
    time.sleep(4)
    g.arm(False)
    g.mark("done")


if __name__ == "__main__":
    main()
