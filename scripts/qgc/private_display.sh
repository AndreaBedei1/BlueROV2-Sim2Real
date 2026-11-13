#!/usr/bin/env bash
# Start a private, headless X display for QGroundControl tests.
#
#   scripts/qgc/private_display.sh <state_dir> [display_number]
#
# Uses the xorgxrdp dummy driver already installed for remote desktops, runs as
# the current user, listens only on its unix socket (-nolisten tcp) and
# requires an MIT cookie stored in <state_dir>/Xauthority, so other users of
# the shared machine cannot connect to it. Prints "DISPLAY=:N XAUTHORITY=..."
# and writes the server PID to <state_dir>/xorg.pid.
set -euo pipefail
STATE=${1:?state_dir}; N=${2:-77}
mkdir -p "$STATE"; STATE=$(cd "$STATE" && pwd)
[ -e "/tmp/.X11-unix/X$N" ] && { echo "display :$N already in use" >&2; exit 2; }
export XAUTHORITY="$STATE/Xauthority"
: > "$XAUTHORITY"; chmod 600 "$XAUTHORITY"
xauth -q add ":$N" . "$(mcookie)"
cd "$STATE"
/usr/lib/xorg/Xorg ":$N" -auth "$XAUTHORITY" -config xrdp/xorg.conf -noreset -nolisten tcp \
  -logfile "$STATE/xorg.log" > "$STATE/xorg.out" 2>&1 &
echo $! > "$STATE/xorg.pid"
for _ in $(seq 1 50); do [ -e "/tmp/.X11-unix/X$N" ] && break; sleep 0.2; done
DISPLAY=":$N" xdpyinfo > /dev/null 2>&1 || { echo "X server did not start" >&2; exit 1; }
echo "DISPLAY=:$N XAUTHORITY=$XAUTHORITY"
