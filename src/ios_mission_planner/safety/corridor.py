from dataclasses import dataclass

import numpy as np


@dataclass
class ApproachCorridor:
    """
    Conical approach corridor around a reference direction, in the chief's
    LVLH frame.

    A relative position is compliant when the angle between it and the
    reference direction is at most half_angle. The chief itself (zero
    range) is always treated as compliant, since the corridor constrains
    the direction of approach, not the final contact point.

    Attributes
    ----------
    direction : array_like
        Reference direction, e.g. [0, -1, 0] to approach the chief from
        behind along the V-bar, or [-1, 0, 0] from above along the R-bar.
        Normalized to a unit vector on construction.
    half_angle : float
        Half-angle of the cone [rad].

    Author: Giovanni Facchinetti, 2026
    """

    direction: np.ndarray
    half_angle: float

    def __post_init__(self):
        direction = np.asarray(self.direction, dtype=float)
        self.direction = direction / np.linalg.norm(direction)

    def angle(self, positions):
        """Angle [rad] between each relative position and the reference direction."""

        positions = np.asarray(positions, dtype=float)
        ranges = np.linalg.norm(positions, axis=-1)
        cos_angle = np.divide(
            positions @ self.direction, ranges,
            out=np.ones_like(ranges), where=ranges > 1e-9,
        )
        return np.arccos(np.clip(cos_angle, -1.0, 1.0))

    def is_violated(self, positions):
        """True wherever a relative position strays outside the corridor."""
        return self.angle(positions) > self.half_angle


def check_corridor(positions, corridor):
    """
    Check a relative trajectory against an approach corridor.

    Parameters
    ----------
    positions : array_like, shape (n, 3)
        Relative positions [x, y, z] in the chief's LVLH frame.
    corridor : ApproachCorridor

    Returns
    -------
    violated : bool
        True if any sample strays outside the corridor.
    max_margin : float
        angle.max() - half_angle. Positive means the trajectory left the
        corridor, by that many radians at its worst; negative means it
        stayed inside, with that much angular room to spare. Same sign
        convention as keepout.check_trajectory.
    violation_mask : numpy.ndarray of bool, shape (n,)
        Per-sample violation flag.

    Author: Giovanni Facchinetti, 2026
    """

    angle = corridor.angle(positions)
    mask = angle > corridor.half_angle

    return bool(mask.any()), float(angle.max() - corridor.half_angle), mask
