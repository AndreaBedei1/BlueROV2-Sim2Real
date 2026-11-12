#!/usr/bin/env bash
# Start ArduSub 4.1.2 SITL with the JSON backend for the HoloOcean bridge.
#
#   scripts/run_sitl.sh <run_dir> [instance]
#
# Environment:
#   ARDUPILOT_DIR  ArduPilot checkout at tag ArduSub-4.1.2 with
#                  patches/ardupilot-ArduSub-4.1.2-sitl-loopback.patch applied
#                  and built with `./waf configure --board sitl && ./waf sub`.
#
# Ports for instance I (default 7): MAVLink TCP 5760+10I (serial0, waits for a
# client), 5762+10I, 5763+10I; JSON physics: SITL sends to UDP 9002+10I.
# With the loopback patch every listening socket is bound to 127.0.0.1.
set -euo pipefail
RUN_DIR=${1:?usage: run_sitl.sh <run_dir> [instance]}
INSTANCE=${2:-7}
REPO=$(cd "$(dirname "$0")/.." && pwd)
AP=${ARDUPILOT_DIR:?set ARDUPILOT_DIR to the ArduSub-4.1.2 checkout}
BIN="$AP/build/sitl/bin/ardusub"
[ -x "$BIN" ] || { echo "missing $BIN" >&2; exit 1; }

for p in $((5760 + 10*INSTANCE)) $((5762 + 10*INSTANCE)) $((5763 + 10*INSTANCE)) $((5501 + 10*INSTANCE)); do  # 9002+10I belongs to the bridge
  if ss -Htuln | awk '{print $5}' | grep -Eq "[:.]$p\$"; then
    echo "port $p already in use: choose another instance" >&2; exit 2
  fi
done

mkdir -p "$RUN_DIR/sitl"
cd "$RUN_DIR/sitl"
{
  echo "ardusub_bin=$BIN"
  echo "ardusub_sha256=$(sha256sum "$BIN" | cut -d' ' -f1)"
  echo "ardupilot_commit=$(git -C "$AP" rev-parse HEAD)"
  echo "ardupilot_diff_sha256=$(git -C "$AP" diff | sha256sum | cut -d' ' -f1)"
  echo "instance=$INSTANCE"
  echo "sub_parm_sha256=$(sha256sum "$AP/Tools/autotest/default_params/sub.parm" | cut -d' ' -f1)"
  echo "overlay_sha256=$(sha256sum "$REPO/configs/sitl/phase2_ardusub412.parm" | cut -d' ' -f1)"
  echo "loopback_patch_sha256=$(sha256sum "$REPO/patches/ardupilot-ArduSub-4.1.2-sitl-loopback.patch" | cut -d' ' -f1)"
  echo "patch_matches_checkout=$( [ "$(git -C "$AP" diff | sha256sum | cut -d' ' -f1)" = "$(sha256sum < "$REPO/patches/ardupilot-ArduSub-4.1.2-sitl-loopback.patch" | cut -d' ' -f1)" ] && echo yes || echo NO)"
} > sitl_manifest.txt
# Fixed public test location (ArduPilot CMAC default), altitude 0 = water surface.
CMD=("$BIN" --model JSON:127.0.0.1 --speedup 1 -I "$INSTANCE" -w
     --home -35.363261,149.165230,0,0
     --defaults "$AP/Tools/autotest/default_params/sub.parm,$REPO/configs/sitl/phase2_ardusub412.parm")
echo "command=${CMD[*]}" >> sitl_manifest.txt
exec "${CMD[@]}"
