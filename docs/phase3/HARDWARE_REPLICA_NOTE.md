# Future activity: Raspberry Pi / Navigator hardware replica (not part of Phase 3)

Phase 3 runs ArduSub 4.1.2 as SITL on the development PC. A later, separate
activity may run parts of the stack on a spare Raspberry Pi 4 (and optionally a
Navigator board) to study computing load, timing and BlueOS services. Notes
for that activity:

* **What it would add**: BlueOS 1.3.1 services (MAVLink router, video
  manager, ping service) as on the vehicle; autonomy executed on the companion
  computer; measured CPU/memory/latency on the real computing platform.
* **What it would not prove**: a Raspberry Pi running SITL is still
  software-in-the-loop. Only a Navigator with its sensors and ESC outputs
  would exercise the flight-controller hardware path (HIL), which needs a
  dedicated design (sensor injection) and is not provided by the JSON SITL
  backend.
* **Interface reuse**: the Phase 3 contract and `VehicleClient` are
  transport-agnostic; a replica would add a profile pointing to the replica's
  BlueOS endpoints. The read-only rule for the physical vehicle stays.
* **Prerequisites**: the full parameter backup (ROVerLago archive), the
  vehicle's BlueOS configuration, and an agreed plan for any write access to
  hardware.
