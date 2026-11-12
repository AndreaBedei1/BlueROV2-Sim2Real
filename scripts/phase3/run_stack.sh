#!/usr/bin/env bash
# Phase 3 simulation stack: HoloOcean bridge + ArduSub 4.1.2 SITL (with the
# vehicle's accessory/button parameters) + loopback MAVLink router
# [+ isolated QGroundControl] [+ a command to run against it].
#
#   scripts/phase3/run_stack.sh <run_dir> [options] [-- command args...]
#
# Options:
#   --backend holoocean|mock   physics backend (default holoocean)
#   --qgc none|private|desktop QGroundControl on a private headless display,
#                              on the current $DISPLAY, or not at all (default none)
#   --wait SECONDS             keep the stack up this long when no command is given
#                              (default: until Ctrl-C)
# Environment: ARDUPILOT_DIR, PYTHON, QGC_APPDIR (for --qgc), INSTANCE (7),
# RATE (200), BOOT_HOLD (25), BRIDGE_ARGS.
#
# Endpoints (all 127.0.0.1): SITL MAVLink 5760+10I (router only), bridge UDP
# 9002+10I, QGC TCP 5880, autonomy UDP 14570 (router pushes, like BlueOS'
# GCS link), sensor adapters UDP 14580.
set -uo pipefail
RUN_DIR=${1:?run_dir}; shift
BACKEND=holoocean; QGC=none; WAIT=""
while [ $# -gt 0 ]; do
  case "$1" in
    --backend) BACKEND=$2; shift 2;;
    --qgc) QGC=$2; shift 2;;
    --wait) WAIT=$2; shift 2;;
    --) shift; break;;
    *) break;;
  esac
done
REPO=$(cd "$(dirname "$0")/../.." && pwd)
PY=${PYTHON:-python}
INSTANCE=${INSTANCE:-7}; RATE=${RATE:-200}; BOOT_HOLD=${BOOT_HOLD:-25}
: "${ARDUPILOT_DIR:?set ARDUPILOT_DIR}"
mkdir -p "$RUN_DIR"; RUN_DIR=$(cd "$RUN_DIR" && pwd)
die() { echo "run_stack: $*" >&2; exit 2; }

LOCK=${XDG_RUNTIME_DIR:-/tmp}/bluerov2-sim2real-$(id -u).lock
exec 9>"$LOCK"
flock -n 9 || die "another bluerov2-sim2real experiment of this user is running ($LOCK)"
if [ "$BACKEND" = holoocean ]; then
  N_HOLO=$(pgrep -fc '/Holodeck/Binaries/Linux/Holodeck ' || true)
  [ "${N_HOLO:-0}" -lt 2 ] || die "$N_HOLO HoloOcean engines already running; not starting another"
fi
for p in $((5760 + 10*INSTANCE)) $((5762 + 10*INSTANCE)) $((5763 + 10*INSTANCE)) 5880; do
  ss -Htln | awk '{print $4}' | grep -Eq "[:.]$p\$" && die "TCP port $p in use"
done
for p in $((9002 + 10*INSTANCE)) $((5501 + 10*INSTANCE)) 14570 14580 5610; do
  ss -Huln | awk '{print $4}' | grep -Eq "[:.]$p\$" && die "UDP port $p in use"
done
{
  echo "date_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "backend=$BACKEND qgc=$QGC instance=$INSTANCE rate=$RATE boot_hold=$BOOT_HOLD"
  echo "bridge_args=${BRIDGE_ARGS:-}"
  echo "command=$*"
  echo "repo_commit=$(git -C "$REPO" rev-parse HEAD)"
  echo "repo_dirty=$(git -C "$REPO" status --porcelain | wc -l)"
} > "$RUN_DIR/run_manifest.txt"

own_alive() { [ -n "$1" ] && [ -r "/proc/$1/cmdline" ] && tr '\0' ' ' < "/proc/$1/cmdline" | grep -Eq -- "$2"; }
stop_own() {
  own_alive "$1" "$2" || return 0
  kill -s "$3" "$1" 2>/dev/null || true
  for _ in $(seq 1 $(( $4 * 5 ))); do own_alive "$1" "$2" || return 0; sleep 0.2; done
  kill -s KILL "$1" 2>/dev/null || true
}
cleanup() {
  stop_own "${CMD_PID:-}" "." TERM 5
  stop_own "$(cat "$RUN_DIR/qgc/qgc.pid" 2>/dev/null)" "AppRun|QGroundControl" TERM 10
  stop_own "$(cat "$RUN_DIR/display/xorg.pid" 2>/dev/null)" "Xorg :" TERM 5
  stop_own "${ROUTER_PID:-}" run_router.py TERM 5
  stop_own "${SITL_PID:-}" "run_sitl.sh|ardusub" TERM 5
  stop_own "${BRIDGE_PID:-}" run_bridge.py TERM 75
  ENGINE_PID=$(cat "$RUN_DIR/bridge/engine.pid" 2>/dev/null || true)
  own_alive "$ENGINE_PID" Holodeck && stop_own "$ENGINE_PID" Holodeck TERM 10
  stop_own "${MON_PID:-}" monitor_resources.py TERM 5
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP

# bridge ------------------------------------------------------------------------
"$PY" "$REPO/scripts/run_bridge.py" --backend "$BACKEND" --out "$RUN_DIR/bridge" \
  --rate "$RATE" --instance "$INSTANCE" --boot-hold "$BOOT_HOLD" ${BRIDGE_ARGS:-} > "$RUN_DIR/bridge.log" 2>&1 &
BRIDGE_PID=$!
"$PY" "$REPO/scripts/monitor_resources.py" --pids $BRIDGE_PID --out "$RUN_DIR/resources.csv" &
MON_PID=$!
for _ in $(seq 1 180); do
  [ -f "$RUN_DIR/bridge/ready" ] && break
  own_alive $BRIDGE_PID run_bridge.py || { cat "$RUN_DIR/bridge.log" >&2; die "bridge exited"; }
  sleep 1
done
[ -f "$RUN_DIR/bridge/ready" ] || die "bridge not ready"

# SITL with the accessory parameters ----------------------------------------------
SITL_EXTRA_DEFAULTS="$REPO/configs/sitl/phase3_accessories.parm" \
  "$REPO/scripts/run_sitl.sh" "$RUN_DIR" "$INSTANCE" > "$RUN_DIR/sitl.log" 2>&1 &
SITL_PID=$!
for _ in $(seq 1 60); do
  ss -Htlnp 2>/dev/null | grep "127.0.0.1:$((5760 + 10*INSTANCE)) " | grep -q "pid=$SITL_PID," && break
  own_alive $SITL_PID "run_sitl.sh|ardusub" || { cat "$RUN_DIR/sitl.log" >&2; die "SITL exited"; }
  sleep 0.5
done
ss -Htlnp 2>/dev/null | grep "127.0.0.1:$((5760 + 10*INSTANCE)) " | grep -q "pid=$SITL_PID," || die "SITL port not ours"

# router ------------------------------------------------------------------------------
mkdir -p "$RUN_DIR/router"
"$PY" "$REPO/scripts/phase3/run_router.py" --instance "$INSTANCE" --tlog "$RUN_DIR/router/traffic.tlog" \
  > "$RUN_DIR/router.log" 2>&1 &
ROUTER_PID=$!
for _ in $(seq 1 60); do [ -f "$RUN_DIR/router/traffic.ready" ] && break; sleep 0.5; done
[ -f "$RUN_DIR/router/traffic.ready" ] || die "router not ready"

# QGroundControl ----------------------------------------------------------------------
if [ "$QGC" = private ]; then
  eval "$("$REPO/scripts/qgc/private_display.sh" "$RUN_DIR/display" "${QGC_DISPLAY_NUM:-77}" | sed 's/^/export /')"
fi
if [ "$QGC" != none ]; then
  "$REPO/scripts/qgc/run_qgc_sim.sh" "$RUN_DIR/qgc" 5880
  echo "qgc_display=$DISPLAY" >> "$RUN_DIR/run_manifest.txt"
fi
echo "bridge_pid=$BRIDGE_PID sitl_pid=$SITL_PID router_pid=$ROUTER_PID engine_pid=$(cat "$RUN_DIR/bridge/engine.pid" 2>/dev/null)" >> "$RUN_DIR/run_manifest.txt"
echo "run_stack: ready (QGC=$QGC). MAVLink for clients: udp 127.0.0.1:14570 (listen), QGC: tcp 127.0.0.1:5880"

# the simulated vehicle is usable once the bridge's boot hold has released it --------
if [ "${BOOT_HOLD}" != "0" ]; then
  for _ in $(seq 1 240); do grep -q '"kind": "released"' "$RUN_DIR/bridge/events.jsonl" 2>/dev/null && break; sleep 0.5; done
  grep -q '"kind": "released"' "$RUN_DIR/bridge/events.jsonl" || die "vehicle not released by the bridge"
  sleep 5      # settle after release (vehicle at rest, fully submerged)
  echo "run_stack: simulated vehicle released"
fi

# command or wait ---------------------------------------------------------------------
RC=0
if [ $# -gt 0 ]; then
  RUN_DIR="$RUN_DIR" "$@" &
  CMD_PID=$!
  wait $CMD_PID || RC=$?
  echo "command_exit_code=$RC" >> "$RUN_DIR/run_manifest.txt"
elif [ -n "$WAIT" ]; then
  sleep "$WAIT" & wait $!          # background + wait: signals interrupt the wait at once
else
  while own_alive $SITL_PID "ardusub"; do sleep 2 & wait $!; done
fi
exit $RC
