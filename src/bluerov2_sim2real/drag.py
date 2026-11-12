"""Bridge-side compensation of the HoloOcean 2.3.0 drag unit defect.

In the stock engine (``HolodeckBuoyantAgent::ApplyBuoyancyDragForce``) the
quadratic drag ``F = -0.5 rho Cd A |v_rel| v_rel`` is computed in newtons but
passed to ``AddForceAtLocation``, which expects kg*cm/s^2 (UE units): the
applied drag is 0.01 of the intended value. The separately built and verified
fix multiplies the force by ``UEUnitsPerMeter`` (=100). That build exists only
as a Windows package, so on Linux the bridge adds the missing 0.99 fraction as
an extra force at the centre of mass, through the thruster allocation.

Equivalence with the patched engine holds while the compensation is not
clipped by thruster saturation and the vehicle is fully submerged; it uses
the velocity sensed at the end of the previous tick (the engine uses the
velocity at the start of the current tick, i.e. the same value). Ocean
currents are assumed zero. The engine's additional Unreal linear damping
(SetLinearDamping(1.0), SetAngularDamping(0.75)) is present in both builds and
is not modified.

The applied/expected ratio of the stock binary was measured as 0.0099-0.0097
(``scripts/characterize_holoocean.py``); with compensation the bridge test
suite checks that the total matches the intended drag.
"""
from __future__ import annotations

import numpy as np

RHO = 997.0           # HolodeckBuoyantAgent.h
CD = 0.8              # BlueROV2.cpp
AREA = 0.45           # m^2, BlueROV2.cpp
STOCK_FACTOR = 0.01   # fraction actually applied by the stock binary


def intended_drag(v_world: np.ndarray) -> np.ndarray:
    v = np.asarray(v_world, dtype=float)
    return -0.5 * RHO * CD * AREA * np.linalg.norm(v) * v


def missing_drag_world(v_world: np.ndarray, submerged_fraction: float = 1.0,
                       stock_factor: float = STOCK_FACTOR) -> np.ndarray:
    """Force (N, world frame of v) to add so that total = intended drag."""
    return (1.0 - stock_factor) * submerged_fraction * intended_drag(v_world)


def submerged_fraction_estimate(com_depth_m: float, ramp_m: float = 0.3) -> float:
    """Crude submersion estimate used only near the surface (engine samples points)."""
    return float(np.clip(com_depth_m / ramp_m, 0.0, 1.0))
