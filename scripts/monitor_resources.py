#!/usr/bin/env python
"""Sample CPU/RAM of the given processes (and their children) and global GPU use.

Only reads information; never signals any process.
"""
import argparse
import csv
import subprocess
import time

import psutil


def gpu():
    try:
        out = subprocess.run(
            ["nvidia-smi", "--query-gpu=index,utilization.gpu,memory.used", "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=10).stdout.strip().splitlines()
        return ";".join(line.replace(" ", "") for line in out)
    except Exception as e:  # noqa: BLE001
        return f"error:{e}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pids", type=int, nargs="+", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--period", type=float, default=5.0)
    a = ap.parse_args()
    procs = {}
    with open(a.out, "w", newline="", buffering=1) as f:
        w = csv.writer(f)
        w.writerow(["wall", "n_procs", "cpu_percent", "rss_mb", "gpu_index_util_memMiB"])
        while True:
            alive = []
            for pid in a.pids:
                try:
                    p = psutil.Process(pid)
                    alive.append(p)
                    alive.extend(p.children(recursive=True))
                except psutil.NoSuchProcess:
                    pass
            if not alive:
                break
            cpu, rss = 0.0, 0.0
            for p in alive:
                try:
                    if p.pid not in procs:
                        procs[p.pid] = p
                        p.cpu_percent(None)
                    cpu += procs[p.pid].cpu_percent(None)
                    rss += p.memory_info().rss / 2**20
                except psutil.NoSuchProcess:
                    pass
            w.writerow([f"{time.time():.1f}", len(alive), f"{cpu:.1f}", f"{rss:.1f}", gpu()])
            time.sleep(a.period)


if __name__ == "__main__":
    main()
