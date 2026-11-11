"""Stage 1: inspect URDF geometry and joint directions, without contact dynamics."""

import argparse
from pathlib import Path
import shlex

import meshcat.geometry as mg
import meshcat.transformations as tf
import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

from twbr.modeling import load_model


def set_joint_angle(model, q, name, degrees):
    """Set a scalar revolute/continuous joint, preserving Pinocchio coordinates."""
    if not model.existJointName(name):
        raise ValueError(f"Unknown joint: {name}")
    joint = model.joints[model.getJointId(name)]
    if joint.nv != 1 or joint.nq not in (1, 2):
        raise ValueError("Only scalar revolute/continuous joints can be changed")
    angle = np.deg2rad(float(degrees))
    if not np.isfinite(angle):
        raise ValueError("Angle must be finite")
    result = q.copy()
    if joint.nq == 2:
        result[joint.idx_q:joint.idx_q + 2] = [np.cos(angle), np.sin(angle)]
    else:
        lower = model.lowerPositionLimit[joint.idx_q]
        upper = model.upperPositionLimit[joint.idx_q]
        if not lower <= angle <= upper:
            raise ValueError(
                f"{name}: allowed range is [{np.rad2deg(lower):.2f}, "
                f"{np.rad2deg(upper):.2f}] degrees"
            )
        result[joint.idx_q] = angle
    return result


def display_pose(viz, model, data, q):
    viz.display(q)
    com = pin.centerOfMass(model, data, q).copy()
    projection = np.array([com[0], com[1], 0.0])
    viz.viewer["markers/com"].set_transform(tf.translation_matrix(com))
    viz.viewer["markers/com_projection"].set_transform(tf.translation_matrix(projection))
    viz.viewer["markers/com_vertical"].set_object(mg.Line(
        mg.PointsGeometry(np.column_stack((com, projection))),
        mg.LineBasicMaterial(color=0xDC2626),
    ))
    print(f"Whole robot CoM (world, m): {np.round(com, 6)}", flush=True)


def export_html(viewer, path):
    path = Path(path)
    if path.suffix.lower() != ".html":
        raise ValueError("Export path must end in .html")
    path.parent.mkdir(parents=True, exist_ok=True)
    html = viewer.static_html()
    legend = """
<aside style="position:fixed;left:18px;top:18px;z-index:10;pointer-events:none;
font:14px/1.6 sans-serif;background:rgba(255,255,255,.94);color:#172033;
padding:14px 18px;border-radius:8px;box-shadow:0 2px 12px #0002">
<strong>TWBR · 1단계: URDF 형상 확인</strong><br>
공중에 띄운 검토 자세 · 접촉/균형 시뮬레이션 아님<br>
<span style="color:#dc2626">●</span> 전체 무게중심 (몸체 안에서도 표시)<br>
<span style="color:#d97706">●</span> 무게중심의 지면 투영<br>
마우스 드래그: 회전 · 휠: 확대/축소
</aside>
"""
    html = html.replace("</body>", legend + "</body>")
    path.write_text(html, encoding="utf-8")
    print(f"Standalone 3D HTML: {path.resolve()}", flush=True)


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path,
                        default=root / "2WL_V3_Assem_v2/urdf/2WL_V3_Assem_v2.urdf")
    parser.add_argument("--package-dir", type=Path,
                        help="Parent directory containing the ROS description package")
    parser.add_argument("--height", type=float, default=0.35,
                        help="Preview base height in metres, not a contact solution")
    parser.add_argument("--joint", nargs=2, action="append", default=[],
                        metavar=("NAME", "DEGREES"))
    parser.add_argument("--export", type=Path, help="Save a standalone interactive .html")
    parser.add_argument("--no-interactive", action="store_true",
                        help="Export and exit; live server closes when the script exits")
    args = parser.parse_args()
    if not np.isfinite(args.height) or args.height <= 0:
        parser.error("--height must be a positive finite number")
    if args.no_interactive and args.export is None:
        parser.error("--no-interactive requires --export")
    if args.export is not None and args.export.suffix.lower() != ".html":
        parser.error("--export must end in .html")

    urdf = args.urdf.expanduser().resolve()
    model = load_model(urdf, base="floating")
    data = model.createData()
    neutral = pin.neutral(model)
    neutral[2] = args.height
    q = neutral.copy()
    try:
        for name, degrees in args.joint:
            q = set_joint_angle(model, q, name, degrees)
    except ValueError as error:
        parser.error(str(error))

    package_dir = args.package_dir or urdf.parents[2]
    visual = pin.buildGeomFromUrdf(
        model, str(urdf), pin.GeometryType.VISUAL,
        package_dirs=[str(package_dir.resolve())],
    )
    print(f"Loaded {visual.ngeoms} visual geometries", flush=True)
    viz = MeshcatVisualizer(model, None, visual)
    viz.initViewer(open=False)
    viz.loadViewerModel(rootNodeName="twbr")
    viewer = viz.viewer
    viz.setBackgroundColor(col_top=[0.94, 0.96, 0.98], col_bot=[0.87, 0.90, 0.94])
    # MeshCat 0.3.2 uses a Y-up camera beneath its Z-up scene transform.
    # Pinocchio's newer setCameraTarget/Position helpers require newer MeshCat APIs.
    viewer["/Cameras/default"].set_transform(
        tf.translation_matrix([0.0, 0.0, args.height - 0.03])
    )
    viewer["/Cameras/default/rotated/<object>"].set_property(
        "position", [0.65, 0.4, -0.8]
    )
    viewer["/Grid"].set_property("visible", False)
    viewer["reference/ground"].set_object(
        mg.Box([1.6, 1.6, 0.008]), mg.MeshLambertMaterial(color=0xD4DBE4),
    )
    viewer["reference/ground"].set_transform(tf.translation_matrix([0, 0, -0.004]))
    viewer["reference/world_axes"].set_object(mg.triad(scale=0.16))
    viewer["markers/com"].set_object(
        mg.Sphere(0.009),
        mg.MeshBasicMaterial(color=0xDC2626, depthTest=False, transparent=True),
    )
    viewer["markers/com_projection"].set_object(
        mg.Sphere(0.008), mg.MeshBasicMaterial(color=0xF59E0B),
    )
    display_pose(viz, model, data, q)
    print("Preview only: no ground contacts, no gravity integration, no controller.")
    print("Red: whole robot CoM; amber: ground projection; axes: X red/Y green/Z blue.")
    print("Joints:", ", ".join(list(model.names)[2:]))
    print("Commands: joint NAME DEGREES | reset | save PATH.html | info | quit", flush=True)
    if args.export is not None:
        export_html(viewer, args.export)
    if args.no_interactive:
        return
    try:
        while True:
            try:
                command = shlex.split(input("twbr-viewer> "))
                if not command:
                    continue
                if command == ["quit"]:
                    break
                if command == ["reset"]:
                    q = neutral.copy()
                    display_pose(viz, model, data, q)
                elif command == ["info"]:
                    display_pose(viz, model, data, q)
                elif len(command) == 3 and command[0] == "joint":
                    q = set_joint_angle(model, q, command[1], command[2])
                    display_pose(viz, model, data, q)
                elif len(command) == 2 and command[0] == "save":
                    export_html(viewer, command[1])
                else:
                    print("Use: joint NAME DEGREES | reset | save PATH.html | info | quit")
            except (ValueError, OSError) as error:
                print(f"Error: {error}")
    except (KeyboardInterrupt, EOFError):
        print("\nViewer stopped.")


if __name__ == "__main__":
    main()
