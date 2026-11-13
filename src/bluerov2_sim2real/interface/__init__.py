"""Phase 3 common vehicle interface.

ArduSub 4.1.2 speaks MAVLink 2; extension fields such as SERVO_OUTPUT_RAW
servo9..16 (accessory outputs) exist only in the MAVLink 2 dialect, so the
pymavlink dialect is forced to 2.0 before pymavlink is imported.
"""
import os
import sys

if "pymavlink.mavutil" in sys.modules and os.environ.get("MAVLINK20") != "1":
    raise ImportError("pymavlink was imported before MAVLINK20=1 was set; import "
                      "bluerov2_sim2real.interface first or export MAVLINK20=1")
os.environ["MAVLINK20"] = "1"
