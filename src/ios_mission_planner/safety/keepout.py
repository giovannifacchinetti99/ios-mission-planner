from dataclasses import dataclass

import numpy as np


@dataclass
class KeepOutEllipsoid:
    """
    Ellipsoidal keep-out zone centered on the chief, in its LVLH frame.

    A relative position (x, y, z) violates the zone when
    (x/a)^2 + (y/b)^2 + (z/c)^2 < 1, i.e. when it falls inside the ellipsoid.
    A sphere of radius r is the special case a = b = c = r.

    Attributes
    ----------
    a, b, c : float
        Semi-axes along radial, along-track and cross-track [m].

    Author: Giovanni Facchinetti, 2026
    """

    a: float
    b: float
    c: float

    def normalized_radius(self, positions):
        """
        sqrt((x/a)^2 + (y/b)^2 + (z/c)^2) for relative positions [..., 3].

        Values below 1 are inside the zone (a violation); the margin to the
        boundary is 1 - normalized_radius.
        """

        positions = np.asarray(positions, dtype=float)
        scaled = positions / np.array([self.a, self.b, self.c])
        return np.linalg.norm(scaled, axis=-1)

    def is_violated(self, positions):
        """True wherever a relative position falls inside the zone."""
        return self.normalized_radius(positions) < 1.0


def check_trajectory(positions, zone):
    """
    Check a relative trajectory against a keep-out zone.

    Parameters
    ----------
    positions : array_like, shape (n, 3)
        Relative positions [x, y, z] in the chief's LVLH frame, in the same
        units as the zone's semi-axes.
    zone : KeepOutEllipsoid

    Returns
    -------
    violated : bool
        True if any sample along the trajectory falls inside the zone.
    min_margin : float
        1 - normalized_radius.min(). Positive means the trajectory entered
        the zone, by that fraction of the relevant semi-axis; negative means
        it stayed clear, with that much margin to the boundary.
    violation_mask : numpy.ndarray of bool, shape (n,)
        Per-sample violation flag, to locate when it happens.

    Author: Giovanni Facchinetti, 2026
    """

    r = zone.normalized_radius(positions)
    mask = r < 1.0

    return bool(mask.any()), float(1.0 - r.min()), mask
