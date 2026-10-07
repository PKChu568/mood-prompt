"""Stewart-platform geometry constants (data, not logic).

Extracted from robot/reachy_mini_description/urdf/robot.urdf by
scripts/extract_stewart_geometry.py. All lengths in metres, in the base
link (body_down_3dprint) and platform link (xl_330) frames respectively.

NOT verified against a physical robot -- only against the URDF numbers.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# Raw values as extracted (see module docstring).
_BASE_POINTS = [
    [-0.0328112, 0.0304906, 0.0418333],
    [-0.0428112, 0.01317, 0.0418333],
    [-0.01, -0.0436606, 0.0418333],
    [0.01, -0.0436606, 0.0418333],
    [0.0428112, 0.01317, 0.0418333],
    [0.0328112, 0.0304906, 0.0418333],
]

_HORN_AXES = [
    [0.8660266281777007, -0.49999787926917255, -3.673205103346574e-06],
    [0.8660229549648076, -0.5000042414425404, -3.673205103346574e-06],
    [2.653589793335038e-06, 0.999999999989733, -3.673205103346574e-06],
    [-3.471782471067467e-14, 0.9999999999932537, -3.673205103346574e-06],
    [-0.8660266281777007, -0.49999787926917255, -3.673205103346574e-06],
    [-0.8660229549648076, -0.5000042414425404, -3.673205103346574e-06],
]

# Platform anchors expressed in the platform rest frame (base-aligned,
# origin at PLATFORM_REST_ORIGIN). At the identity pose these map back to
# the URDF rest anchors, each exactly rod_length from its horn tip.
_PLATFORM_POINTS = [
    [-0.043526929302617534, 2.4877750886415084e-07, 1.1393882748400586e-07],
    [-0.050526964758975196, -0.012124673068495507, 1.4538803808084033e-07],
    [-0.028763445664933676, -0.049819712712483394, 5.412337245047638e-16],
    [-0.014763614271292406, -0.04981970475149036, 0.0],
    [0.006999875337563511, -0.012124246349019329, -2.0816681711721685e-16],
    [0.0, 0.0, 0.0],
]

# Platform frame origin relative to the base frame, at the rest pose.
_PLATFORM_REST_ORIGIN = [0.02176354919186463, 0.02064797713119207, 0.1147709956232715]


@dataclass(frozen=True)
class StewartGeometry:
    """Fixed geometry of the 6-leg Stewart platform.

    base_points:      6x3, horn-joint origins in the base frame.
    horn_axes:        6x3, horn-joint rotation axes (unit) in the base frame.
    platform_points:  6x3, rod-to-platform attachment points in the
                      platform frame.
    platform_rest_origin: platform-frame origin in the base frame at rest.
    horn_length:      crank/horn length (m).
    rod_length:       connecting-rod length (m).
    """

    base_points: np.ndarray = field(
        default_factory=lambda: np.array(_BASE_POINTS, dtype=float)
    )
    horn_axes: np.ndarray = field(
        default_factory=lambda: np.array(_HORN_AXES, dtype=float)
    )
    platform_points: np.ndarray = field(
        default_factory=lambda: np.array(_PLATFORM_POINTS, dtype=float)
    )
    platform_rest_origin: np.ndarray = field(
        default_factory=lambda: np.array(_PLATFORM_REST_ORIGIN, dtype=float)
    )
    horn_length: float = 0.040608
    rod_length: float = 0.085000


DEFAULT_GEOMETRY = StewartGeometry()
