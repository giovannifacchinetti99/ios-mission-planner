import numpy as np

def hill_equations(t, state, n):
    """
    Hill-Clohessy-Wiltshire equations for circular reference orbit.

    Parameters:
    t : float
        Time [s].
    state : ndarray
        State vector [x, y, z, vx, vy, vz].
    n : float
        Mean motion of the chief [rad/s].

    Returns:
    ndarray
        Time derivative of the state.

    Author: Giovanni Facchinetti, 2026
    Reference: Analytical Mechanics of Space Systems, Fourth Edition
    """

    x, y, z, vx, vy, vz = state

    ax = 2.0 * n * vy + 3.0 * n**2 * x
    ay = -2.0 * n * vx
    az = -n**2 * z

    return np.array([
        vx,
        vy,
        vz,
        ax,
        ay,
        az
    ])