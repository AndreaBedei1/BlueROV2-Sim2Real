#!/usr/bin/env python
"""Run the loopback MAVLink router (simulation profile). See interface/router.py."""
import argparse
import json
import signal
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
from bluerov2_sim2real.interface.router import MavlinkRouter, RouterConfig  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--instance", type=int, default=7)
    ap.add_argument("--qgc-tcp-port", type=int, default=5880)
    ap.add_argument("--udp-out", type=int, nargs="*", default=[14570])
    ap.add_argument("--udp-in", type=int, nargs="*", default=[14580])
    ap.add_argument("--tlog", required=True)
    a = ap.parse_args()
    stop = {"flag": False}

    def _stop(*_):
        stop["flag"] = True
    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)
    cfg = RouterConfig(master=("127.0.0.1", 5760 + 10 * a.instance), tcp_server_port=a.qgc_tcp_port,
                       udp_out_ports=a.udp_out, udp_in_ports=a.udp_in, tlog=Path(a.tlog))
    r = MavlinkRouter(cfg)
    Path(a.tlog).with_suffix(".ready").write_text("listening\n")
    try:
        r.run(stop_flag=lambda: stop["flag"])
    finally:
        r.close()
        Path(a.tlog).with_suffix(".counts.json").write_text(json.dumps(r.counts))


if __name__ == "__main__":
    main()
