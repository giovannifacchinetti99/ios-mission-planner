import numpy as np


def cw_state_transition_matrix(t, n):
    """
    Compute the Clohessy-Wiltshire state transition matrix.

    Parameters
    ----------
    t : float
        Propagation time [s].
    n : float
        Mean motion of the chief [rad/s].

    Returns
    -------
    numpy.ndarray
        6x6 Clohessy-Wiltshire state transition matrix.

    Author: Giovanni Facchinetti, 2026
    Reference: https://en.wikipedia.org/wiki/Clohessy%E2%80%93Wiltshire_equations
    """

    nt = n * t

    c = np.cos(nt)
    s = np.sin(nt)

    phi = np.array([
        [
            4.0 - 3.0 * c,
            0.0,
            0.0,
            s / n,
            2.0 * (1.0 - c) / n,
            0.0,
        ],
        [
            6.0 * (s - nt),
            1.0,
            0.0,
            -2.0 * (1.0 - c) / n,
            (4.0 * s - 3.0 * nt) / n,
            0.0,
        ],
        [
            0.0,
            0.0,
            c,
            0.0,
            0.0,
            s / n,
        ],
        [
            3.0 * n * s,
            0.0,
            0.0,
            c,
            2.0 * s,
            0.0,
        ],
        [
            -6.0 * n * (1.0 - c),
            0.0,
            0.0,
            -2.0 * s,
            4.0 * c - 3.0,
            0.0,
        ],
        [
            0.0,
            0.0,
            -n * s,
            0.0,
            0.0,
            c,
        ],
    ])

    return phi