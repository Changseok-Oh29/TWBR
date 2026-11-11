"""Load the inertial model without requiring ROS or mesh visualization."""

from pathlib import Path

import pinocchio as pin


def load_model(urdf_path: str | Path, *, base: str = "floating") -> pin.Model:
    """Load a fixed or floating base model; gravity is world -Z in m/s²."""
    path = Path(urdf_path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"URDF not found: {path}")
    if base == "floating":
        model = pin.buildModelFromUrdf(str(path), pin.JointModelFreeFlyer())
    elif base == "fixed":
        model = pin.buildModelFromUrdf(str(path))
    else:
        raise ValueError("base must be 'floating' or 'fixed'")
    model.gravity.linear[:] = [0.0, 0.0, -9.81]
    return model
