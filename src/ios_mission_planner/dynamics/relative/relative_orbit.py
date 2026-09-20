from dataclasses import dataclass

import numpy as np


@dataclass
class RelativeOrbit:
    """
    Geometric description of a Clohessy-Wiltshire relative orbit.

    In-plane, the motion is a 2:1 ellipse (along-track semi-axis = 2 x radial
    semi-axis) whose center sits at radial offset x_off and drifts along-track
    at -3/2 * n * x_off. Out-of-plane it is a plain harmonic oscillation.

        x(t) = x_off + rho * cos(n t + alpha)
        y(t) = y_c + drift * t - 2 * rho * sin(n t + alpha)
        z(t) = rho_z * cos(n t + beta)

    Attributes
    ----------
    x_off : float
        Radial offset of the ellipse center [m]. Non-zero means secular drift.
    y_c : float
        Along-track position of the ellipse center at t = 0 [m].
    rho : float
        Radial semi-axis of the in-plane ellipse [m] (along-track is 2*rho).
    alpha : float
        In-plane phase at t = 0 [rad].
    rho_z : float
        Out-of-plane amplitude [m].
    beta : float
        Out-of-plane phase at t = 0 [rad].

    Author: Giovanni Facchinetti, 2026
    """

    x_off: float
    y_c: float
    rho: float
    alpha: float
    rho_z: float = 0.0
    beta: float = 0.0

    def drift_rate(self, n):
        """Along-track drift velocity of the ellipse center [m/s]."""
        return -1.5 * n * self.x_off


def relative_orbit_from_state(state, n):
    """
    Decompose a relative state [x, y, z, vx, vy, vz] into a RelativeOrbit.

    Derived by grouping the terms of the CW closed-form solution into a
    constant, a secular part and an oscillation at frequency n.

    Author: Giovanni Facchinetti, 2026
    Reference: Analytical Mechanics of Space Systems, Fourth Edition
    """

    x0, y0, z0, vx0, vy0, vz0 = np.asarray(state, dtype=float)

    a = 3.0 * x0 + 2.0 * vy0 / n
    b = vx0 / n

    return RelativeOrbit(
        x_off=4.0 * x0 + 2.0 * vy0 / n,
        y_c=y0 - 2.0 * b,
        rho=float(np.hypot(a, b)),
        alpha=float(np.arctan2(-b, -a)),
        rho_z=float(np.hypot(z0, vz0 / n)),
        beta=float(np.arctan2(-vz0 / n, z0)),
    )


def drift_free_orbit_through(x, y, y_c=0.0):
    """
    The drift-free 2:1 ellipse centered at along-track position y_c that
    passes through the in-plane point (x, y).

    Useful to insert onto a safety ellipse from wherever the deputy actually
    ended up, instead of from the point it was supposed to reach.

    Author: Giovanni Facchinetti, 2026
    """

    half_dy = 0.5 * (y - y_c)

    return RelativeOrbit(
        x_off=0.0,
        y_c=y_c,
        rho=float(np.hypot(x, half_dy)),
        alpha=float(np.arctan2(-half_dy, x)),
    )


def state_from_relative_orbit(orbit, n):
    """
    Relative state [x, y, z, vx, vy, vz] at t = 0 for a given RelativeOrbit.

    With x_off = 0 this is a drift-free (periodic) relative orbit: the
    2:1 ellipse that satisfies vy = -2 n x at every instant.

    Author: Giovanni Facchinetti, 2026
    """

    ca, sa = np.cos(orbit.alpha), np.sin(orbit.alpha)
    cb, sb = np.cos(orbit.beta), np.sin(orbit.beta)

    return np.array([
        orbit.x_off + orbit.rho * ca,
        orbit.y_c - 2.0 * orbit.rho * sa,
        orbit.rho_z * cb,
        -n * orbit.rho * sa,
        orbit.drift_rate(n) - 2.0 * n * orbit.rho * ca,
        -n * orbit.rho_z * sb,
    ])
