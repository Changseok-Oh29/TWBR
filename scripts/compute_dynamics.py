"""Inspect the current URDF at its neutral configuration."""

import argparse
from pathlib import Path

import numpy as np
import pinocchio as pin

from twbr.modeling import compute_dynamics, load_model


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--urdf", type=Path,
        default=root / "2WL_V3_Assem_v2/urdf/2WL_V3_Assem_v2.urdf",
    )
    parser.add_argument("--base", choices=("floating", "fixed"), default="floating")
    parser.add_argument("--output", type=Path, help="Optional .npz result path")
    args = parser.parse_args()
    if args.output is not None and args.output.suffix != ".npz":
        parser.error("--output must end in .npz")

    model = load_model(args.urdf, base=args.base)
    data = model.createData()
    q = pin.neutral(model)
    v = np.zeros(model.nv)
    result = compute_dynamics(model, data, q, v)
    np.set_printoptions(precision=6, suppress=True)

    print(f"Pinocchio: {pin.__version__}")
    print(f"URDF: {args.urdf.resolve()}")
    print(f"Base: {args.base}; neutral configuration, v = 0; no contacts")
    print(f"nq = {model.nq}, nv = {model.nv}")
    print(f"Whole robot mass: {result.total_mass:.9f} kg")
    print(f"Moving-subtree mass: {result.moving_mass:.9f} kg")
    print(f"Whole robot CoM in world [m]: {result.center_of_mass}")
    print(f"M shape: {result.mass_matrix.shape}")
    print(f"M minimum eigenvalue: {np.linalg.eigvalsh(result.mass_matrix)[0]:.6e}")
    print(f"g(q): {result.gravity}")
    print(f"h(q, 0): {result.nonlinear_effects}")
    print("Joint mapping (slices are zero-based, stop-exclusive):")
    for name, joint in zip(list(model.names)[1:], list(model.joints)[1:]):
        print(
            f"  {name:24s} q[{joint.idx_q}:{joint.idx_q + joint.nq}]"
            f" v[{joint.idx_v}:{joint.idx_v + joint.nv}]"
        )
    if args.base == "floating":
        print("Floating-base g[:6] is a wrench, not six motor torques.")
    print("Neutral configuration is a calculation reference, not a balanced pose.")

    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        np.savez(
            args.output,
            q=q, v=v, M=result.mass_matrix, h=result.nonlinear_effects,
            g=result.gravity, com=result.center_of_mass,
            total_mass=result.total_mass, moving_mass=result.moving_mass,
            base=args.base, urdf=str(args.urdf.resolve()),
            pinocchio_version=pin.__version__,
            joint_names=np.asarray(list(model.names)[1:]),
            joint_idx_q=np.asarray([j.idx_q for j in list(model.joints)[1:]]),
            joint_nq=np.asarray([j.nq for j in list(model.joints)[1:]]),
            joint_idx_v=np.asarray([j.idx_v for j in list(model.joints)[1:]]),
            joint_nv=np.asarray([j.nv for j in list(model.joints)[1:]]),
        )
        print(f"Saved: {args.output.resolve()}")


if __name__ == "__main__":
    main()
