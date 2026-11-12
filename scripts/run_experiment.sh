#!/usr/bin/env bash
# One complete run: bridge (HoloOcean or mock) + ArduSub SITL + MAVLink scenario.
#
#   scripts/run_experiment.sh <run_dir> <scenario> [--backend mock] [mission args...]
#
# Environment: ARDUPILOT_DIR (required), PYTHON (default: python), INSTANCE (7),
# RATE (200), BOOT_HOLD (25), BRIDGE_ARGS (extra bridge options).
#
# Shared-machine rules enforced here:
# * one experiment of this project at a time per user (flock), hence at most
#   one HoloOcean instance started by this script;
# * refuses to start if two or more HoloOcean engines already run on the
#   machine (any user), so the machine never exceeds two because of us;
# * every port of the chosen SITL instance is checked before anything starts;
# * only processes started by this script are ever signalled, after checking
#   their command line; other users' processes are never touched.
set -uo pipefail
RUN_DIR=${1:?run_dir}; SCENARIO=${2:?scenario}; shift 2
BACKEND=holoocean
if [ "${1:-}" = "--backend" ]; then BACKEND=$2; shift 2; fi
REPO=$(cd "$(dirname "$0")/.." && pwd)
PY=${PYTHON:-python}
INSTANCE=${INSTANCE:-7}
RATE=${RATE:-200}
BOOT_HOLD=${BOOT_HOLD:-25}
: "${ARDUPILOT_DIR:?set ARDUPILOT_DIR}"
mkdir -p "$RUN_DIR"
RUN_DIR=$(cd "$RUN_DIR" && pwd)

die() { echo "run_experiment: $*" >&2; exit 2; }

# --- 1. exclusivity -----------------------------------------------------------
LOCK=${XDG_RUNTIME_DIR:-/tmp}/bluerov2-sim2real-$(id -u).lock
exec 9>"$LOCK"
flock -n 9 || die "another bluerov2-sim2real experiment of this user is running ($LOCK)"
if [ "$BACKEND" = holoocean ]; then
  N_HOLO=$(pgrep -fc '/Holodeck/Binaries/Linux/Holodeck ' || true)
  [ "${N_HOLO:-0}" -lt 2 ] || die "$N_HOLO HoloOcean engines already running on this machine; not starting another"
fi

# --- 2. ports -------------------------------------------------------------------
UDP_BRIDGE=$((9002 + 10*INSTANCE))
for p in $((5760 + 10*INSTANCE)) $((5762 + 10*INSTANCE)) $((5763 + 10*INSTANCE)); do
  ss -Htln | awk '{print $4}' | grep -Eq "[:.]$p\$" && die "TCP port $p in use: choose another INSTANCE"
done
for p in $UDP_BRIDGE $((5501 + 10*INSTANCE)); do
  ss -Huln | awk '{print $4}' | grep -Eq "[:.]$p\$" && die "UDP port $p in use: choose another INSTANCE"
done

{
  echo "date_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "scenario=$SCENARIO backend=$BACKEND instance=$INSTANCE rate=$RATE boot_hold=$BOOT_HOLD"
  echo "bridge_args=${BRIDGE_ARGS:-}"
  echo "mission_args=$*"
  echo "repo_commit=$(git -C "$REPO" rev-parse HEAD)"
  echo "repo_dirty=$(git -C "$REPO" status --porcelain --untracked-files=no | wc -l)"
  echo "holodeck_engines_before=${N_HOLO:-n/a}"
} > "$RUN_DIR/run_manifest.txt"

# --- 3. process management --------------------------------------------------------
own_alive() {   # own_alive <pid> <cmdline substring>
  [ -n "$1" ] && [ -r "/proc/$1/cmdline" ] && tr '\0' ' ' < "/proc/$1/cmdline" | grep -q -- "$2"
}
stop_own() {    # stop_own <pid> <cmdline substring> <signal> <timeout_s>
  own_alive "$1" "$2" || return 0
  kill -s "$3" "$1" 2>/dev/null || true
  for _ in $(seq 1 $(( $4 * 5 ))); do own_alive "$1" "$2" || return 0; sleep 0.2; done
  kill -s KILL "$1" 2>/dev/null || true
}
cleanup() {
  stop_own "${MISSION_PID:-}" sitl_mission.py TERM 5
  stop_own "${SITL_PID:-}" ardusub TERM 5
  stop_own "${BRIDGE_PID:-}" run_bridge.py TERM 30          # orderly: closes the engine, writes summary
  ENGINE_PID=$(cat "$RUN_DIR/bridge/engine.pid" 2>/dev/null || true)
  if own_alive "$ENGINE_PID" "Holodeck"; then                 # only our own engine, if orphaned
    echo "run_experiment: stopping orphaned engine $ENGINE_PID" >&2
    stop_own "$ENGINE_PID" Holodeck TERM 10
  fi
  stop_own "${MON_PID:-}" monitor_resources.py TERM 5
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP

# --- 4. bridge ------------------------------------------------------------------------
"$PY" "$REPO/scripts/run_bridge.py" --backend "$BACKEND" --out "$RUN_DIR/bridge" \
  --rate "$RATE" --instance "$INSTANCE" --boot-hold "$BOOT_HOLD" ${BRIDGE_ARGS:-} \
  > "$RUN_DIR/bridge.log" 2>&1 &
BRIDGE_PID=$!
"$PY" "$REPO/scripts/monitor_resources.py" --pids $BRIDGE_PID --out "$RUN_DIR/resources.csv" &
MON_PID=$!
for _ in $(seq 1 180); do
  [ -f "$RUN_DIR/bridge/ready" ] && break
  own_alive $BRIDGE_PID run_bridge.py || { cat "$RUN_DIR/bridge.log" >&2; die "bridge exited"; }
  sleep 1
done
[ -f "$RUN_DIR/bridge/ready" ] || die "bridge not ready after 180 s"

# --- 5. SITL ----------------------------------------------------------------------------
"$REPO/scripts/run_sitl.sh" "$RUN_DIR" "$INSTANCE" > "$RUN_DIR/sitl.log" 2>&1 &
SITL_PID=$!
for _ in $(seq 1 60); do
  ss -Htlnp 2>/dev/null | grep "127.0.0.1:$((5760 + 10*INSTANCE)) " | grep -q "pid=$SITL_PID," && break
  own_alive $SITL_PID ardusub || { cat "$RUN_DIR/sitl.log" >&2; die "SITL exited"; }
  sleep 0.5
done
ss -Htlnp 2>/dev/null | grep "127.0.0.1:$((5760 + 10*INSTANCE)) " | grep -q "pid=$SITL_PID," \
  || die "SITL MAVLink port not owned by our SITL process"
echo "bridge_pid=$BRIDGE_PID sitl_pid=$SITL_PID engine_pid=$(cat "$RUN_DIR/bridge/engine.pid" 2>/dev/null)" >> "$RUN_DIR/run_manifest.txt"

# --- 6. mission -------------------------------------------------------------------------
"$PY" "$REPO/scripts/sitl_mission.py" "$SCENARIO" --out "$RUN_DIR/mission" --instance "$INSTANCE" "$@" \
  > "$RUN_DIR/mission.log" 2>&1 &
MISSION_PID=$!
MISSION_RC=0
wait $MISSION_PID || MISSION_RC=$?
echo "mission_exit_code=$MISSION_RC" >> "$RUN_DIR/run_manifest.txt"
echo "mission exit code $MISSION_RC"
exit $MISSION_RC
