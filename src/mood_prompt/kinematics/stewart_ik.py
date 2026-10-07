"""Closed-form inverse/forward kinematics for the rotary Stewart platform.

This is a 6-RSS platform: each leg is a servo horn (length ``h``) rotating
about a fixed axis, joined by a rigid rod (length ``d``) to a platform
anchor. IK solves per leg for the horn angle; FK is an iterative solve of
the same constraints (there is no simple closed form for forward).

Pose convention: a 4x4 homogeneous transform of the platform frame
relative to the base frame, with the IDENTITY transform being the URDF
rest pose (all horn angles = their rest value, taken as 0 here).

Geometry is unverified against real hardware; see geometry.py.
"""

from __future__ import annotations

import numpy as np

from .geometry import DEFAULT_GEOMETRY, StewartGeometry

Pose = np.ndarray  # 4x4 homogeneous transform


def _wrap(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def _horn_frame(axis: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return two unit vectors spanning the plane the horn rotates in.

    ``u`` is the in-plane reference (horn angle 0 points along it) and ``v``
    is perpendicular to both ``u`` and the axis, forming a right-handed set.
    The absolute choice of ``u`` is arbitrary as long as IK and FK share it,
    so we pick a stable vector not parallel to the axis.
    """
    axis = axis / np.linalg.norm(axis)
    seed = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(seed, axis)) > 0.9:
        seed = np.array([0.0, 1.0, 0.0])
    u = seed - np.dot(seed, axis) * axis
    u /= np.linalg.norm(u)
    v = np.cross(axis, u)
    return u, v


class StewartIK:
    def __init__(self, geometry: StewartGeometry = DEFAULT_GEOMETRY):
        self.g = geometry
        self._frames = [_horn_frame(a) for a in geometry.horn_axes]
        # Horn tip at angle alpha: tip = base + h*(cos(alpha)*u + sin(alpha)*v).
        # Calibrate a per-leg rest angle so that alpha from inverse_kinematics
        # at the identity pose lands near 0 (keeps the solver well-behaved).
        # Per-leg rest horn angle (raw), used to disambiguate the two IK
        # branches by choosing the one closest to rest.
        self._rest = np.zeros(6)
        self._rest = self.inverse_kinematics(np.eye(4), _raw=True)

    def inverse_kinematics(self, pose: Pose, _raw: bool = False) -> list[float]:
        """Return the 6 horn angles (radians) achieving ``pose``.

        Raises ``ValueError`` if a leg cannot reach the pose (the asin
        argument leaves [-1, 1], i.e. the rod can't bridge horn tip to
        platform anchor).
        """
        R = pose[:3, :3]
        T = pose[:3, 3]
        h = self.g.horn_length
        d = self.g.rod_length
        angles = []
        for k in range(6):
            b = self.g.base_points[k]
            p = self.g.platform_points[k]
            u, v = self._frames[k]

            # Platform anchor in base frame, and leg vector from horn origin.
            # Pose is relative to the rest pose, so the platform frame sits
            # at platform_rest_origin when pose == identity.
            q = self.g.platform_rest_origin + T + R @ p
            lk = q - b
            l2 = float(lk @ lk)

            # Horn tip = b + h*(cos a * u + sin a * v); |tip - q| = d gives
            #   e*cos a + f*sin a = g,  solved as the asin/atan2 form below.
            e = 2.0 * h * (lk @ u)
            f = 2.0 * h * (lk @ v)
            g = l2 + h * h - d * d

            denom = np.hypot(e, f)
            ratio = g / denom
            if not -1.0 <= ratio <= 1.0:
                if _raw:
                    ratio = np.clip(ratio, -1.0, 1.0)
                else:
                    raise ValueError(f"leg {k} unreachable for this pose (|ratio|>1)")
            # e*cos a + f*sin a = g  <=>  R*cos(a - atan2(f,e)) = g, with
            # R = hypot(e, f). Two solutions a = atan2(f,e) +/- acos(g/R);
            # pick the one whose horn tip sits rod_length from the anchor and
            # lies closest to rest (resolves the assembly-mode ambiguity).
            phase = np.arctan2(f, e)
            delta = np.arccos(ratio)
            cand = [phase + delta, phase - delta]
            rest = self._rest[k]
            best = min(
                cand,
                key=lambda a: (
                    abs(np.linalg.norm(self._tip(k, a) - q) - d),
                    abs(_wrap(a - rest)),
                ),
            )
            angles.append(best)

        result = np.array(angles)
        if not _raw:
            result = result - self._rest
        return result.tolist()

    def _tip(self, k: int, alpha_raw: float) -> np.ndarray:
        b = self.g.base_points[k]
        u, v = self._frames[k]
        return b + self.g.horn_length * (np.cos(alpha_raw) * u + np.sin(alpha_raw) * v)

    def forward_kinematics(
        self, horn_angles: list[float], max_iter: int = 100, tol: float = 1e-9
    ) -> Pose:
        """Return the platform pose produced by the 6 ``horn_angles``.

        Solved iteratively (Gauss-Newton on the 6 rod-length residuals over
        the 6-DOF pose), seeded at the rest pose. There is no closed-form
        forward solution for a Stewart platform.
        """
        raw = np.asarray(horn_angles) + self._rest
        tips = np.array([self._tip(k, raw[k]) for k in range(6)])
        d = self.g.rod_length

        # Pose parameters: translation (3) + rotation vector (3).
        x = np.zeros(6)

        def residuals(x: np.ndarray) -> np.ndarray:
            pose = _params_to_pose(x)
            R, T = pose[:3, :3], pose[:3, 3]
            res = np.empty(6)
            for k in range(6):
                q = self.g.platform_rest_origin + T + R @ self.g.platform_points[k]
                res[k] = np.sum((q - tips[k]) ** 2) - d * d
            return res

        for _ in range(max_iter):
            r = residuals(x)
            if np.max(np.abs(r)) < tol:
                break
            # Numerical Jacobian (6x6).
            J = np.empty((6, 6))
            eps = 1e-7
            for j in range(6):
                dx = x.copy()
                dx[j] += eps
                J[:, j] = (residuals(dx) - r) / eps
            step, *_ = np.linalg.lstsq(J, -r, rcond=None)
            x = x + step

        return _params_to_pose(x)


def _params_to_pose(x: np.ndarray) -> Pose:
    """Map [tx, ty, tz, rx, ry, rz] (rotation vector) to a 4x4 transform."""
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


# Module-level convenience wrappers using the default geometry.
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
