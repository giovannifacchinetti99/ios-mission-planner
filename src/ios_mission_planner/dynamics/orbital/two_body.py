import numpy as np


def two_body_dynamics(t, state, mu):
    """
    Two-body equations of motion in an inertial frame.

    Parameters
    ----------
    t : float
        Time [s] (unused, kept for solve_ivp/propagate compatibility).
    state : ndarray
        Inertial state vector [x, y, z, vx, vy, vz].
    mu : float
        Gravitational parameter of the central body [km^3/s^2].

    Returns
    -------
    ndarray
        Time derivative of the state.

    Author: Giovanni Facchinetti, 2026
    """

    r = state[:3]
    v = state[3:]

    r_norm = np.linalg.norm(r)
    a = -mu * r / r_norm**3

    return np.concatenate([v, a])
