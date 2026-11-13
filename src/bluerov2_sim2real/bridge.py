"""Lockstep ArduSub SITL (JSON backend) <-> physics bridge.

One physics step per *new* SITL frame:

    SITL --servo packet (frame n)--> bridge --wrench--> backend.step(dt)
    SITL <--state at t = k*dt------- bridge

* A repeated ``frame_count`` (SITL resends after ~1 s without a reply) does
  **not** advance physics. If the resend reached the socket *before* our
  reply to the original was sent (a physics step slower than 1 s), it is
  dropped: SITL already got that reply, and answering again would leave one
  stale reply queued forever (a permanent extra frame of lag). Only a resend
  received after our reply (reply lost) is answered again. Receive times are
  kernel timestamps (SO_TIMESTAMPNS).
* A jump in ``frame_count`` is counted as lost SITL->bridge packets; physics
  still advances one step. SITL advances its own clock by the difference of
  our timestamps, so the two clocks stay equal (dt = 1/200 s is an exact number
  of microseconds; SITL patch rounds the increment).
* A decrease of ``frame_count`` means SITL restarted: the run is stopped
  (reason ``sitl_reset``), because the physics state no longer matches.
* The bridge locks onto the first sender (address, port) and ignores any
  other datagram; only loopback senders are accepted.
* The reply timestamp is k*dt with k the number of physics steps: simulated
  time is exact and independent of wall time. Wall time is only measured.

Outputs in ``out_dir``: ``frames.csv`` (one row per physics step; ``t_sim``
is the time *after* the step that used the row's PWM), ``events.jsonl`` and
``summary.json`` (also written on errors).
"""
from __future__ import annotations

import csv
import json
import socket
import struct
import time
import traceback
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np

from .frames import euler_zyx_from_matrix
from .sitl_json import FdmState, ProtocolError, encode_fdm, parse_servo_packet
from .thrusters import LogicalVehicle

ALLOWED_HOSTS = {"127.0.0.1"}
SO_TIMESTAMPNS = getattr(socket, "SO_TIMESTAMPNS", 35)   # Linux value


def open_bridge_socket(host: str, port: int) -> socket.socket:
    """Bind the UDP socket SITL sends to. Fails immediately if the port is taken."""
    if host not in ALLOWED_HOSTS:
        raise ValueError("the bridge only binds to the local host (no real-vehicle networks)")
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((host, port))
    return s


@dataclass
class BridgeConfig:
    bind_host: str = "127.0.0.1"
    bind_port: int = 9072                # SITL instance 7: 9002 + 10*7
    out_dir: Path = Path("results/run")
    max_sim_s: Optional[float] = None
    max_wall_s: Optional[float] = None
    boot_hold_s: float = 0.0             # sim seconds of kinematic hold at start
    recv_timeout_s: float = 5.0
    first_frame_timeout_s: float = 180.0  # stop if SITL never sends
    idle_exit_s: float = 30.0            # stop if SITL silent this long (after first frame)
    status_every_s: float = 10.0


@dataclass
class Stats:
    frames: int = 0
    steps: int = 0
    duplicates: int = 0
    lost: int = 0
    resets: int = 0
    bad_packets: int = 0
    rejected: int = 0
    stale_duplicates_dropped: int = 0
    timeouts: int = 0
    saturated_steps: int = 0
    pwm_out_of_range_frames: int = 0
    frame_rate_values: set = field(default_factory=set)


class Bridge:
    def __init__(self, backend, cfg: BridgeConfig, vehicle: Optional[LogicalVehicle] = None,
                 sock: Optional[socket.socket] = None):
        self.backend = backend
        self.cfg = cfg
        self.vehicle = vehicle or LogicalVehicle()
        self.stats = Stats()
        self.out = Path(cfg.out_dir)
        self.out.mkdir(parents=True, exist_ok=True)
        self.sock = sock if sock is not None else open_bridge_socket(cfg.bind_host, cfg.bind_port)
        if self.sock.getsockname()[0] not in ALLOWED_HOSTS:
            raise ValueError("bridge socket is not bound to the local host")
        self.sock.settimeout(cfg.recv_timeout_s)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMPNS, 1)
            self._kernel_ts = True
        except OSError:
            self._kernel_ts = False
        self._t_reply_sent = 0.0
        self._events = open(self.out / "events.jsonl", "w", buffering=1)
        self._csvf = open(self.out / "frames.csv", "w", newline="")
        self._csv = csv.writer(self._csvf)
        self._csv.writerow(
            ["wall_s", "t_sim", "frame_count", "frame_rate", "held", "step_ms", "rx_gap_ms"]
            + [f"pwm{i}" for i in range(1, 9)] + [f"thrust_m{i}" for i in range(1, 9)]
            + [f"holo_f{i}" for i in range(8)] + ["saturated", "wrench_err_n", "wrench_err_nm"]
            + ["n", "e", "d", "vn", "ve", "vd", "roll_deg", "pitch_deg", "yaw_deg",
               "gx", "gy", "gz", "ax", "ay", "az", "drag_x", "drag_y", "drag_z"]
        )

    def event(self, kind: str, **kw):
        kw.update(kind=kind, wall=time.time(), t_sim=self.backend.ticks * self.backend.dt)
        self._events.write(json.dumps(kw, default=float) + "\n")

    def close(self):
        self._csvf.close()
        self._events.close()
        self.sock.close()

    def _recv(self):
        """(data, addr, receive time in CLOCK_REALTIME seconds)."""
        if self._kernel_ts:
            data, anc, _flags, addr = self.sock.recvmsg(2048, socket.CMSG_SPACE(16))
            for level, kind, payload in anc:
                if level == socket.SOL_SOCKET and kind == SO_TIMESTAMPNS and len(payload) >= 16:
                    sec, nsec = struct.unpack("qq", payload[:16])
                    return data, addr, sec + nsec * 1e-9
            if not getattr(self, "_warned_no_kernel_ts", False):
                self._warned_no_kernel_ts = True
                self.event("warning", message="datagram without kernel timestamp: stale-resend "
                                              "detection degraded to read time")
            return data, addr, time.time()
        data, addr = self.sock.recvfrom(2048)
        return data, addr, time.time()

    def _send(self, data, addr):
        self.sock.sendto(data, addr)
        self._t_reply_sent = time.time()

    @staticmethod
    def _state(obs) -> FdmState:
        return FdmState(obs.t_sim, obs.gyro_frd, obs.accel_frd, obs.position_ned,
                        obs.velocity_ned, obs.quaternion_wxyz)

    def run(self) -> dict:
        cfg, st = self.cfg, self.stats
        last_count: Optional[int] = None
        last_reply: Optional[bytes] = None
        peer = None
        t0 = time.perf_counter()
        t_last_rx = t_first_rx = None
        t_last_status = t_last_flush = t0
        rx_gaps, step_ms = [], []
        released = not getattr(self.backend, "hold", False)
        reason = "unknown"
        self.event("start", bind=f"{cfg.bind_host}:{cfg.bind_port}", dt=self.backend.dt,
                   boot_hold_s=cfg.boot_hold_s)
        try:
            while True:
                try:
                    data, addr, t_rx = self._recv()
                except socket.timeout:
                    st.timeouts += 1
                    self.event("rx_timeout", seconds=cfg.recv_timeout_s)
                    now = time.perf_counter()
                    if t_last_rx is None and now - t0 > cfg.first_frame_timeout_s:
                        reason = "no_first_frame"
                        break
                    if t_last_rx is not None and now - t_last_rx > cfg.idle_exit_s:
                        reason = "sitl_silent"
                        break
                    continue
                now = time.perf_counter()
                if addr[0] not in ALLOWED_HOSTS or (peer is not None and addr != peer):
                    st.rejected += 1
                    self.event("rejected_sender", host=addr[0], port=addr[1])
                    continue
                try:
                    pkt = parse_servo_packet(data)
                except ProtocolError as e:
                    st.bad_packets += 1
                    self.event("bad_packet", error=str(e), size=len(data))
                    continue
                if peer is None:
                    peer = addr
                    self.event("peer", host=addr[0], port=addr[1])
                gap = (now - t_last_rx) * 1e3 if t_last_rx is not None else float("nan")
                t_last_rx = now
                if t_first_rx is None:
                    t_first_rx = now
                st.frames += 1
                st.frame_rate_values.add(pkt.frame_rate)
                if last_count is not None and pkt.frame_count == last_count:
                    if t_rx < self._t_reply_sent:
                        st.stale_duplicates_dropped += 1
                        self.event("stale_duplicate_dropped", frame_count=pkt.frame_count,
                                   received_before_reply_s=self._t_reply_sent - t_rx)
                        continue
                    st.duplicates += 1
                    self.event("duplicate_frame", frame_count=pkt.frame_count)
                    self._send(last_reply, addr)
                    continue
                if last_count is not None and pkt.frame_count < last_count:
                    st.resets += 1
                    self.event("sitl_reset", frame_count=pkt.frame_count, previous=last_count)
                    reason = "sitl_reset"
                    break
                if last_count is not None and pkt.frame_count > last_count + 1:
                    st.lost += pkt.frame_count - last_count - 1
                    self.event("lost_frames", n=pkt.frame_count - last_count - 1, at=pkt.frame_count)
                last_count = pkt.frame_count

                if not released and self.backend.ticks * self.backend.dt >= cfg.boot_hold_s:
                    self.backend.release()
                    released = True
                    self.event("released")

                if any(0 < p < 1100 or p > 1900 for p in pkt.pwm[:8]):
                    st.pwm_out_of_range_frames += 1
                if hasattr(self.backend, "on_servo"):
                    self.backend.on_servo(pkt.pwm)      # accessories (Phase 3); no effect on forces
                thrusts = self.vehicle.pwm_to_thrusts(pkt.pwm)
                wrench = self.vehicle.wrench(thrusts)
                ts = time.perf_counter()
                obs = self.backend.step(wrench, nominal=self.vehicle.nominal_holo_forces(thrusts))
                dstep = (time.perf_counter() - ts) * 1e3
                last_reply = encode_fdm(self._state(obs))
                self._send(last_reply, addr)
                st.steps += 1
                if obs.extras.get("saturated") and not obs.extras.get("held"):
                    st.saturated_steps += 1
                if st.steps > 1:
                    rx_gaps.append(gap)
                step_ms.append(dstep)
                self._log(now - t0, pkt, obs, thrusts, dstep, gap)

                if now - t_last_flush > 1.0:
                    t_last_flush = now
                    self._csvf.flush()
                if now - t_last_status > cfg.status_every_s:
                    t_last_status = now
                    print(f"[bridge] wall {now - t0:7.1f}s sim {obs.t_sim:7.1f}s depth {obs.depth_m:6.3f}m "
                          f"frames {st.frames} dup {st.duplicates} lost {st.lost}", flush=True)
                if cfg.max_sim_s is not None and obs.t_sim >= cfg.max_sim_s:
                    reason = "max_sim_s"
                    break
                if cfg.max_wall_s is not None and now - t0 >= cfg.max_wall_s:
                    reason = "max_wall_s"
                    break
        except KeyboardInterrupt:
            reason = "interrupted"
        except Exception as e:  # noqa: BLE001 - record any failure before re-raising
            reason = f"error:{type(e).__name__}"
            self.event("exception", error=repr(e), traceback=traceback.format_exc())
            raise
        finally:
            wall = time.perf_counter() - t0
            active = (t_last_rx - t_first_rx) if t_first_rx is not None else 0.0
            summary = self._summary(wall, rx_gaps, step_ms, reason, active)
            (self.out / "summary.json").write_text(json.dumps(summary, indent=2, default=float))
            self.event("stop", reason=reason)
            self.close()
        return summary

    def _log(self, wall, pkt, obs, thrusts, dstep, gap):
        rpy = np.degrees(euler_zyx_from_matrix(obs.R_ned_frd))
        hf = obs.extras.get("holo_forces", np.full(8, np.nan))
        we = obs.extras.get("wrench_error", np.full(6, np.nan))
        dc = obs.extras.get("drag_comp_world_nwu", np.zeros(3))
        self._csv.writerow(
            [f"{wall:.6f}", f"{obs.t_sim:.6f}", pkt.frame_count, pkt.frame_rate,
             int(bool(obs.extras.get("held"))), f"{dstep:.3f}", f"{gap:.3f}"]
            + list(pkt.pwm[:8]) + [f"{x:.3f}" for x in thrusts] + [f"{x:.3f}" for x in hf]
            + [int(bool(obs.extras.get("saturated"))), f"{np.linalg.norm(we[0:3]):.4f}",
               f"{np.linalg.norm(we[3:6]):.4f}"]
            + [f"{x:.5f}" for x in obs.position_ned] + [f"{x:.5f}" for x in obs.velocity_ned]
            + [f"{x:.4f}" for x in rpy] + [f"{x:.5f}" for x in obs.gyro_frd]
            + [f"{x:.4f}" for x in obs.accel_frd] + [f"{x:.4f}" for x in dc]
        )

    def _summary(self, wall, rx_gaps, step_ms, reason, active_wall=0.0):
        st = self.stats
        sim = self.backend.ticks * self.backend.dt
        g = np.asarray([x for x in rx_gaps if np.isfinite(x)])
        s = np.asarray(step_ms)

        def pct(a, q):
            return float(np.percentile(a, q)) if a.size else None

        return {
            "stop_reason": reason, "wall_s": wall, "sim_s": sim,
            "active_wall_s": active_wall,
            "sim_over_active_wall": sim / active_wall if active_wall > 0 else None,
            "dt": self.backend.dt, "frames": st.frames, "physics_steps": st.steps,
            "duplicates_resent": st.duplicates, "lost_frames": st.lost, "sitl_resets": st.resets,
            "stale_duplicates_dropped": st.stale_duplicates_dropped,
            "bad_packets": st.bad_packets, "rejected_datagrams": st.rejected, "rx_timeouts": st.timeouts,
            "saturated_steps": st.saturated_steps, "pwm_out_of_range_frames": st.pwm_out_of_range_frames,
            "sitl_frame_rate_values": sorted(st.frame_rate_values),
            "rx_gap_ms": {"mean": float(g.mean()) if g.size else None, "std": float(g.std()) if g.size else None,
                          "p50": pct(g, 50), "p99": pct(g, 99), "p99_9": pct(g, 99.9),
                          "max": float(g.max()) if g.size else None,
                          "n_over_50ms": int((g > 50).sum()), "n_over_1000ms": int((g > 1000).sum())},
            "step_ms": {"p50": pct(s, 50), "p99": pct(s, 99), "max": float(s.max()) if s.size else None},
        }
