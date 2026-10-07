"""Closed-form inverse / numerical forward kinematics for Reachy Mini's
rotary Stewart platform, using Pollen's vendored geometry.

Each leg is a servo horn (length ``h``) rotating about its motor frame's
local Z axis, joined by a rigid rod (length ``d``) to an attachment point
on the moving platform. IK is solved per leg in the motor's local frame,
where the horn tip is ``(h cos a, h sin a, 0)``; the assembly-mode branch
is chosen per leg by the vendored ``solution`` sign (no guessing).

Pose convention (matching Pollen's AnalyticalKinematics.ik): the head pose
is a 4x4 transform with IDENTITY as the neutral pose. Before solving, the
head Z offset is added to the pose translation. No axis remapping.
"""

from __future__ import annotations

import numpy as np

from .geometry import DEFAULT_GEOMETRY, StewartGeometry

Pose = np.ndarray  # 4x4 homogeneous transform


class StewartIK:
    def __init__(self, geometry: StewartGeometry = DEFAULT_GEOMETRY):
        self.g = geometry

    def _apply_offset(self, pose: Pose) -> Pose:
        p = np.array(pose, dtype=float)
        p[2, 3] += self.g.head_z_offset
        return p

    def inverse_kinematics(
        self,
        pose: Pose,
        reference: list[float] | np.ndarray | None = None,
        clamp: bool = False,
    ) -> list[float]:
        """Return the 6 horn angles (radians) for a head ``pose``.

        ``reference`` is accepted for API parity with trajectory playback
        but the branch here is fixed by the vendored ``solution`` sign, so a
        reference is not needed for continuity. ``clamp`` bounds each angle
        to the leg's horn limits. Raises ``ValueError`` if a leg cannot
        reach the pose and ``clamp`` is not set.
        """
        pose = self._apply_offset(pose)
        h = self.g.horn_length
        d = self.g.rod_length
        angles = []
        for k in range(6):
            # Branch (rod attachment) point into the motor's local frame.
            # Pollen's data stores T_motor_world; their Rust solve computes
            #   branch_motor = inv(t_world_motor) * pose * branch,
            # and t_world_motor = inv(T_motor_world), so the net transform is
            # T_motor_world * pose * branch. The horn then sweeps the motor
            # frame's XY plane about its local Z.
            q_platform = pose @ np.array([*self.g.branch_position[k], 1.0])
            qh = self.g.motor_world[k] @ q_platform
            x, y, z = qh[:3]

            # Horn tip t = (h cos a, h sin a, 0); |t - q|^2 = d^2 expands to
            #   2h(x cos a + y sin a) = x^2+y^2+z^2 + h^2 - d^2 = g.
            e = 2.0 * h * x
            f = 2.0 * h * y
            g = x * x + y * y + z * z + h * h - d * d

            denom = np.hypot(e, f)
            ratio = g / denom if denom != 0 else 2.0
            if not -1.0 <= ratio <= 1.0:
                if clamp:
                    ratio = float(np.clip(ratio, -1.0, 1.0))
                else:
                    raise ValueError(f"leg {k} unreachable for this pose")
            # e cos a + f sin a = g  <=>  hypot(e,f) cos(a - atan2(f,e)) = g.
            # The two roots are atan2(f,e) +/- acos(ratio); the vendored
            # solution sign selects the correct assembly mode.
            phase = np.arctan2(f, e)
            delta = np.arccos(ratio)
            angle = phase + self.g.solution[k] * delta
            angle = _wrap(angle)
            if clamp:
                lo, hi = self.g.horn_limits[k]
                angle = min(max(angle, lo), hi)
            angles.append(angle)
        return angles

    def _tip_world(self, k: int, angle: float) -> np.ndarray:
        # Horn tip in the motor frame -> platform/world frame. IK transforms
        # a world point by motor_world, so the inverse maps motor -> world.
        h = self.g.horn_length
        tip_motor = np.array([h * np.cos(angle), h * np.sin(angle), 0.0, 1.0])
        return (self.g.motor_world_inv[k] @ tip_motor)[:3]

    def forward_kinematics(
        self, horn_angles: list[float], max_iter: int = 100, tol: float = 1e-12
    ) -> Pose:
        """Return the head pose produced by the 6 ``horn_angles``.

        Numerical (Gauss-Newton on the 6 rod-length residuals over the 6-DOF
        pose). Returns the pose in the same convention as ``inverse_kinematics``
        input (head Z offset removed), so IK/FK round-trip.
        """
        angles = np.asarray(horn_angles, dtype=float)
        tips = np.array([self._tip_world(k, angles[k]) for k in range(6)])
        d = self.g.rod_length

        # Seed at the offset-applied neutral so Gauss-Newton converges to the
        # physical assembly mode (the Stewart FK has multiple solutions; a
        # zero seed can land on a spurious branch). tz starts at the head Z
        # offset, which is where the platform sits at the neutral pose.
        x = np.array([0.0, 0.0, self.g.head_z_offset, 0.0, 0.0, 0.0])

        def residuals(x: np.ndarray) -> np.ndarray:
            pose = _params_to_pose(x)
            R, T = pose[:3, :3], pose[:3, 3]
            res = np.empty(6)
            for k in range(6):
                q = R @ self.g.branch_position[k] + T
                res[k] = np.sum((q - tips[k]) ** 2) - d * d
            return res

        for _ in range(max_iter):
            r = residuals(x)
            if np.max(np.abs(r)) < tol:
                break
            J = np.empty((6, 6))
            eps = 1e-7
            for j in range(6):
                dx = x.copy()
                dx[j] += eps
                J[:, j] = (residuals(dx) - r) / eps
            step, *_ = np.linalg.lstsq(J, -r, rcond=None)
            x = x + step

        pose = _params_to_pose(x)
        pose[2, 3] -= self.g.head_z_offset  # back to input convention
        return pose


def _wrap(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def _params_to_pose(x: np.ndarray) -> Pose:
    pose = np.eye(4)
    pose[:3, 3] = x[:3]
    pose[:3, :3] = _rotvec_to_matrix(x[3:])
    return pose


def _rotvec_to_matrix(rv: np.ndarray) -> np.ndarray:
    theta = np.linalg.norm(rv)
    if theta < 1e-12:
        return np.eye(3)
    k = rv / theta
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(theta) * kx + (1 - np.cos(theta)) * (kx @ kx)


_DEFAULT = None


def _default() -> StewartIK:
    global _DEFAULT
    if _DEFAULT is None:
        _DEFAULT = StewartIK()
    return _DEFAULT


def inverse_kinematics(pose: Pose) -> list[float]:
    return _default().inverse_kinematics(pose)


def forward_kinematics(horn_angles: list[float]) -> Pose:
    return _default().forward_kinematics(horn_angles)
