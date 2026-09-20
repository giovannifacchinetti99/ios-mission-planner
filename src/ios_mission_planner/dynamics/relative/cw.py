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


def cw_targeting(r0, v0, rf, vf, t, n):
    """
    Two-impulse Clohessy-Wiltshire targeting.

    Finds the two velocity changes that take a deputy from (r0, v0) to
    (rf, vf) in a fixed time of flight. The first impulse is applied at the
    start and sets the velocity that lands exactly on rf after t; the second
    is applied on arrival and matches the velocity to vf.

    Writing the STM in 3x3 blocks, r(t) = Phi_rr r0 + Phi_rv v0+, so
    v0+ = Phi_rv^-1 (rf - Phi_rr r0). The arrival velocity follows as
    v(t)- = Phi_vr r0 + Phi_vv v0+.

    Phi_rv loses rank for some times of flight (in-plane block at
    n*t = 2*pi*k and a few other values, out-of-plane block at n*t = k*pi).
    There the required delta-v either diverges or is non-unique, in which
    case the solver returns one arbitrary solution.

    Parameters
    ----------
    r0, v0 : array_like
        Initial relative position and velocity (before the first impulse).
    rf, vf : array_like
        Desired relative position and velocity at arrival (after the
        second impulse).
    t : float
        Time of flight [s].
    n : float
        Mean motion of the chief [rad/s].

    Returns
    -------
    dv1, dv2 : numpy.ndarray
        Delta-v vectors of the departure and arrival impulses.

    Author: Giovanni Facchinetti, 2026
    Reference: Vallado, Fundamentals of Astrodynamics and Applications
    """

    r0 = np.asarray(r0, dtype=float)
    v0 = np.asarray(v0, dtype=float)
    rf = np.asarray(rf, dtype=float)
    vf = np.asarray(vf, dtype=float)

    phi = cw_state_transition_matrix(t, n)
    phi_rr, phi_rv = phi[:3, :3], phi[:3, 3:]
    phi_vr, phi_vv = phi[3:, :3], phi[3:, 3:]

    v0_plus = np.linalg.solve(phi_rv, rf - phi_rr @ r0)
    vf_minus = phi_vr @ r0 + phi_vv @ v0_plus

    return v0_plus - v0, vf - vf_minus