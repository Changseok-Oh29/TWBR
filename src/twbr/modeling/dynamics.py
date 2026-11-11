"""Numerical rigid-body dynamics, without contact or actuator constraints."""

from dataclasses import dataclass

import numpy as np
import pinocchio as pin


@dataclass(frozen=True)
class DynamicsResult:
    mass_matrix: np.ndarray
    nonlinear_effects: np.ndarray
    gravity: np.ndarray
    center_of_mass: np.ndarray
    total_mass: float
    moving_mass: float


def compute_dynamics(
    model: pin.Model,
    data: pin.Data,
    q: np.ndarray,
    v: np.ndarray,
) -> DynamicsResult:
    """Compute M(q), h(q,v), g(q), and the whole robot's world-frame CoM.

    q has length model.nq; v has length model.nv. Use pin.neutral and
    pin.integrate for configurations containing quaternions or continuous joints.
    data must be created from model; it is mutated, but returned arrays are copies.
    The fixed base's inertia belongs to universe (index 0). Pinocchio's standard
    CoM excludes it, so we include it explicitly in the whole-robot mass and CoM.
    """
    q = np.asarray(q, dtype=float)
    v = np.asarray(v, dtype=float)
    if q.shape != (model.nq,) or v.shape != (model.nv,):
        raise ValueError(f"Expected q shape ({model.nq},), v shape ({model.nv},)")
    if not np.isfinite(q).all() or not np.isfinite(v).all():
        raise ValueError("q and v must contain finite values")
    if not pin.isNormalized(model, q):
        raise ValueError("q is not normalized; use pin.neutral or pin.integrate")

    upper = np.triu(pin.crba(model, data, q)).copy()
    mass_matrix = upper + np.triu(upper, 1).T
    nonlinear_effects = pin.nonLinearEffects(model, data, q, v).copy()
    gravity = pin.computeGeneralizedGravity(model, data, q).copy()
    moving_com = pin.centerOfMass(model, data, q).copy()
    moving_mass = float(sum(inertia.mass for inertia in model.inertias[1:]))
    fixed_inertia = model.inertias[0]
    total_mass = moving_mass + float(fixed_inertia.mass)
    if total_mass <= 0:
        raise ValueError("The robot must have positive total mass")
    # In models loaded by load_model, universe coordinates are world coordinates.
    moving_moment = moving_mass * moving_com if moving_mass > 0 else np.zeros(3)
    center_of_mass = (
        moving_moment + fixed_inertia.mass * fixed_inertia.lever
    ) / total_mass

    return DynamicsResult(
        mass_matrix=mass_matrix,
        nonlinear_effects=nonlinear_effects,
        gravity=gravity,
        center_of_mass=center_of_mass.copy(),
        total_mass=total_mass,
        moving_mass=moving_mass,
    )
