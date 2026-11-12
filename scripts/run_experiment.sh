#!/usr/bin/env bash
# One complete run: bridge (HoloOcean or mock) + ArduSub SITL + MAVLink scenario.
#
#   scripts/run_experiment.sh <run_dir> <scenario> [--backend mock] [extra mission args...]
#
# Starts at most one HoloOcean instance, tears everything down at the end and
# only stops the processes it started itself (by PID).
set -euo pipefail
RUN_DIR=${1:?run_dir}; SCENARIO=${2:?scenario}; shift 2
BACKEND=holoocean
if [ "${1:-}" = "--backend" ]; then BACKEND=$2; shift 2; fi
REPO=$(cd "$(dirname "$0")/.." && pwd)
PY=${PYTHON:-python}
INSTANCE=${INSTANCE:-7}
RATE=${RATE:-200}
BOOT_HOLD=${BOOT_HOLD:-20}
mkdir -p "$RUN_DIR"
RUN_DIR=$(cd "$RUN_DIR" && pwd)

cleanup() {
  for pid in ${MISSION_PID:-} ${SITL_PID:-} ${BRIDGE_PID:-}; do
    kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT

"$PY" "$REPO/scripts/run_bridge.py" --backend "$BACKEND" --out "$RUN_DIR/bridge" \
  --rate "$RATE" --instance "$INSTANCE" --boot-hold "$BOOT_HOLD" ${BRIDGE_ARGS:-} \
  > "$RUN_DIR/bridge.log" 2>&1 &
BRIDGE_PID=$!
# wait for the bridge socket (HoloOcean start-up takes ~10 s)
for _ in $(seq 1 120); do
  ss -Huln | awk '{print $5}' | grep -q "127.0.0.1:$((9002 + 10*INSTANCE))$" && break
  kill -0 $BRIDGE_PID 2>/dev/null || { echo "bridge died"; cat "$RUN_DIR/bridge.log"; exit 1; }
  sleep 1
done
"$REPO/scripts/run_sitl.sh" "$RUN_DIR" "$INSTANCE" > "$RUN_DIR/sitl.log" 2>&1 &
SITL_PID=$!
sleep 2
"$PY" "$REPO/scripts/sitl_mission.py" "$SCENARIO" --out "$RUN_DIR/mission" --instance "$INSTANCE" "$@" \
  > "$RUN_DIR/mission.log" 2>&1 &
MISSION_PID=$!
{ echo "bridge_pid=$BRIDGE_PID sitl_pid=$SITL_PID mission_pid=$MISSION_PID"; } > "$RUN_DIR/pids.txt"
# resource monitor (own processes only), every 5 s
"$PY" "$REPO/scripts/monitor_resources.py" --pids $BRIDGE_PID $SITL_PID --out "$RUN_DIR/resources.csv" &
MON_PID=$!
wait $MISSION_PID; MISSION_RC=$?
sleep 3
kill $SITL_PID 2>/dev/null || true
wait $BRIDGE_PID 2>/dev/null || true
kill $MON_PID 2>/dev/null || true
echo "mission exit code $MISSION_RC"
exit $MISSION_RC
