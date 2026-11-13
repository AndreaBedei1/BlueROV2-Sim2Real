#!/usr/bin/env bash
# Start an ISOLATED QGroundControl connected only to the simulation router.
#
#   scripts/qgc/run_qgc_sim.sh <state_dir> [router_tcp_port]
#
# Environment: QGC_APPDIR (extracted QGroundControl 4.2.8 AppImage, i.e. the
# squashfs-root folder), DISPLAY/XAUTHORITY (your desktop or the private display
# from private_display.sh), VIDEO_PORT (UDP H.264 port, default 5610).
#
# Isolation: QGC gets its own HOME (settings, logs, map cache) inside
# <state_dir>; every auto-connect source is disabled (in particular UDP 14550,
# on which a topside computer receives the real vehicle) and the only link is a
# TCP client to 127.0.0.1:<router_tcp_port>. Your normal QGC settings are not
# read or changed, and this instance cannot connect to the physical vehicle.
set -euo pipefail
STATE=${1:?state_dir}; PORT=${2:-5880}
: "${QGC_APPDIR:?set QGC_APPDIR to the extracted QGroundControl 4.2.8 AppImage}"
: "${DISPLAY:?no DISPLAY}"
VIDEO_PORT=${VIDEO_PORT:-5610}
mkdir -p "$STATE/home/.config/QGroundControl.org" "$STATE/home/Documents"
STATE=$(cd "$STATE" && pwd)
cat > "$STATE/home/.config/QGroundControl.org/QGroundControl.ini" <<INI
[General]
SettingsVersion=9
firstRunPromptIdsShown="1,2"
savePath=$STATE/home/Documents/QGroundControl
telemetrySave=true
telemetrySaveNotArmed=true
virtualJoystick=${QGC_VIRTUAL_JOYSTICK:-false}

[AutoConnectSettings]
autoConnectUDP=false
autoConnectPixhawk=false
autoConnectSiKRadio=false
autoConnectPX4Flow=false
autoConnectLibrePilot=false
autoConnectRTKGPS=false
autoConnectZeroConf=false
autoConnectNmeaPort=Disabled
udpListenPort=0

[LinkConfigurations]
count=1
Link0\\name=BlueROV2-SIM-loopback
Link0\\type=2
Link0\\auto=true
Link0\\high_latency=false
Link0\\host=127.0.0.1
Link0\\port=$PORT

[Video]
videoSource=UDP h.264 Video Stream
udpUrl=127.0.0.1:$VIDEO_PORT
INI
echo "qgc_appimage_sha256=$(sha256sum "$QGC_APPDIR/../QGroundControl.AppImage" 2>/dev/null | cut -d' ' -f1)" > "$STATE/qgc_manifest.txt"
cd "$STATE"
# the QGroundControl binary directly (AppRun would leave it as an untracked child)
env -u XDG_CONFIG_HOME -u XDG_DATA_HOME -u XDG_CACHE_HOME HOME="$STATE/home" \
  "$QGC_APPDIR/QGroundControl" > "$STATE/qgc.log" 2>&1 &
echo $! > "$STATE/qgc.pid"
echo "QGroundControl started (pid $(cat "$STATE/qgc.pid")), link TCP 127.0.0.1:$PORT"
