"""One-off: extract Stewart-platform geometry from the URDF (T4.1).

Walks the URDF kinematic tree, composing fixed/joint origin transforms, to
recover for each of the 6 legs:

  * base point   -- origin of the ``stewart_N`` horn joint, in the base
                    link (``body_down_3dprint``) frame
  * horn axis    -- the horn joint's rotation axis, in the base frame
  * platform pt  -- where the leg's rod attaches to the moving platform,
                    expressed in the platform frame
  * horn length  -- horn-joint origin -> first passive joint (``passive_N_x``)
  * rod length   -- rod link -> closing frame (``closing_N_1``)

Prints a Python dict literal to paste into kinematics/geometry.py (T4.2).
Run: ``uv run python scripts/extract_stewart_geometry.py``
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

URDF = (
    Path(__file__).resolve().parents[1]
    / "robot"
    / "reachy_mini_description"
    / "urdf"
    / "robot.urdf"
)
# The URDF is a serial tree rooted at body_foot_3dprint; the moving
# platform (xl_330) hangs off leg 6's chain, so at the URDF rest pose the
# parallel loop is already assembled. We compute everything in the base
# link frame and express platform markers in the platform's rest frame.
BASE_LINK = "body_down_3dprint"
PLATFORM_LINK = "xl_330"


def _rpy_to_matrix(rpy: np.ndarray) -> np.ndarray:
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _origin_transform(joint: ET.Element) -> np.ndarray:
    origin = joint.find("origin")
    xyz = np.zeros(3)
    rpy = np.zeros(3)
    if origin is not None:
        if origin.get("xyz"):
            xyz = np.array([float(v) for v in origin.get("xyz").split()])
        if origin.get("rpy"):
            rpy = np.array([float(v) for v in origin.get("rpy").split()])
    t = np.eye(4)
    t[:3, :3] = _rpy_to_matrix(rpy)
    t[:3, 3] = xyz
    return t


def load_joints(root: ET.Element) -> dict:
    joints = {}
    for j in root.findall("joint"):
        name = j.get("name")
        parent = j.find("parent").get("link")
        child = j.find("child").get("link")
        axis_el = j.find("axis")
        axis = (
            np.array([float(v) for v in axis_el.get("xyz").split()])
            if axis_el is not None and axis_el.get("xyz")
            else np.array([0.0, 0.0, 1.0])
        )
        joints[name] = {
            "parent": parent,
            "child": child,
            "transform": _origin_transform(j),
            "axis": axis,
        }
    return joints


def transform_to_link(joints: dict, target_link: str, root_link: str) -> np.ndarray:
    """Compose transforms from ``root_link`` down to ``target_link``."""
    # Build child-link -> joint lookup.
    by_child = {j["child"]: (name, j) for name, j in joints.items()}
    chain = []
    link = target_link
    while link != root_link:
        if link not in by_child:
            raise RuntimeError(f"no path from {root_link} to {target_link} (stuck at {link})")
        name, j = by_child[link]
        chain.append(j)
        link = j["parent"]
    t = np.eye(4)
    for j in reversed(chain):
        t = t @ j["transform"]
    return t


def main() -> None:
    root = ET.parse(URDF).getroot()
    joints = load_joints(root)

    base_points = []
    horn_axes = []
    anchor_points_base = []  # rod-end/platform anchor in base frame at rest
    horn_lengths = []
    rod_lengths = []

    # Platform rest origin: xl_330 position in the base frame. We define the
    # platform frame as axis-aligned with the base at rest and centred here,
    # so the identity pose equals the URDF rest pose.
    platform_rest_origin = transform_to_link(joints, PLATFORM_LINK, BASE_LINK)[:3, 3]

    for k in range(1, 7):
        # Base point: transform from base link to the stewart_k joint frame.
        t_base = transform_to_link(joints, joints[f"stewart_{k}"]["parent"], BASE_LINK)
        t_base = t_base @ joints[f"stewart_{k}"]["transform"]
        base_points.append(t_base[:3, 3])
        horn_axes.append(t_base[:3, :3] @ joints[f"stewart_{k}"]["axis"])

        # Horn length: translation of the first passive joint off the horn.
        horn_lengths.append(float(np.linalg.norm(joints[f"passive_{k}_x"]["transform"][:3, 3])))

        # Horn tip and platform anchor in base frame at rest; their distance
        # is the rod length. Leg 6 hosts the platform body (xl_330) on its
        # rod chain, so its anchor is the xl_330 origin; legs 1-5 use the
        # rod-side loop-closure marker closing_N_1.
        tip = transform_to_link(joints, joints[f"passive_{k}_x"]["child"], BASE_LINK)[:3, 3]
        if k == 6:
            anchor = transform_to_link(joints, PLATFORM_LINK, BASE_LINK)[:3, 3]
        else:
            anchor = transform_to_link(joints, f"closing_{k}_1", BASE_LINK)[:3, 3]
        rod_lengths.append(float(np.linalg.norm(tip - anchor)))
        anchor_points_base.append(anchor)

    base_points = np.array(base_points)
    horn_axes = np.array(horn_axes)
    anchor_points_base = np.array(anchor_points_base)
    # Platform points: anchors expressed in the platform rest frame (base-
    # aligned, origin at platform_rest_origin).
    platform_points = anchor_points_base - platform_rest_origin

    np.set_printoptions(precision=6, suppress=True)
    print("GEOMETRY = {")
    print(f'    "base_points": {base_points.tolist()!r},')
    print(f'    "horn_axes": {horn_axes.tolist()!r},')
    print(f'    "platform_points": {platform_points.tolist()!r},')
    print(f'    "platform_rest_origin": {platform_rest_origin.tolist()!r},')
    print(f'    "horn_length": {np.mean(horn_lengths):.6f},  # per-leg: {[round(h,6) for h in horn_lengths]}')
    print(f'    "rod_length": {np.mean(rod_lengths):.6f},  # per-leg: {[round(r,6) for r in rod_lengths]}')
    print("}")


if __name__ == "__main__":
    main()
