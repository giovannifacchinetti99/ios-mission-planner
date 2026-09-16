import numpy as np

from ios_mission_planner.dynamics.orbital.two_body import two_body_dynamics


def j2_acceleration(state, mu, j2, r_eq):
    """
    J2 (oblateness) perturbing acceleration in the inertial frame.

    Parameters
    ----------
    state : ndarray
        Inertial state vector [x, y, z, vx, vy, vz]. Only position is used.
    mu : float
        Gravitational parameter of the central body [km^3/s^2].
    j2 : float
        J2 zonal harmonic coefficient of the central body.
    r_eq : float
        Equatorial radius of the central body [km].

    Returns
    -------
    ndarray
        Perturbing acceleration [ax, ay, az] [km/s^2].

    Author: Giovanni Facchinetti, 2026
    Reference: Vallado, Fundamentals of Astrodynamics and Applications
    """

    x, y, z = state[:3]
    r = np.linalg.norm(state[:3])

    factor = 1.5 * j2 * mu * r_eq**2 / r**5
    z_ratio_sq = (z / r) ** 2

    ax = factor * x * (5.0 * z_ratio_sq - 1.0)
    ay = factor * y * (5.0 * z_ratio_sq - 1.0)
    az = factor * z * (5.0 * z_ratio_sq - 3.0)

    return np.array([ax, ay, az])


def two_body_j2_dynamics(t, state, mu, j2, r_eq):
    """
    Two-body equations of motion with J2 perturbation, in an inertial frame.

    Parameters
    ----------
    t : float
        Time [s] (unused, kept for solve_ivp/propagate compatibility).
    state : ndarray
        Inertial state vector [x, y, z, vx, vy, vz].
    mu : float
        Gravitational parameter of the central body [km^3/s^2].
    j2 : float
        J2 zonal harmonic coefficient of the central body.
    r_eq : float
        Equatorial radius of the central body [km].

    Returns
    -------
    ndarray
        Time derivative of the state.

    Author: Giovanni Facchinetti, 2026
    """

    derivative = two_body_dynamics(t, state, mu)
    derivative[3:] += j2_acceleration(state, mu, j2, r_eq)

    return derivative
