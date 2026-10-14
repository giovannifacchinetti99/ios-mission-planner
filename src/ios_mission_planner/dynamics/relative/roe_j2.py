from dataclasses import dataclass

import numpy as np

from ..orbital.elements import ClassicalElements


def _wrap_difference(angle):
    """Wrap an angle difference into (-pi, pi], so e.g. a RAAN of 2pi-epsilon
    minus a RAAN of 0 reads as -epsilon, not as nearly a full turn."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


@dataclass
class QuasiNonsingularROE:
    """
    Quasi-nonsingular relative orbital elements (ROE) of a deputy
    relative to a chief.

    Attributes
    ----------
    da : float
        (a_d - a_c) / a_c.
    dlambda : float
        (M_d + argp_d) - (M_c + argp_c) + (raan_d - raan_c) cos(i_c) [rad].
    dex, dey : float
        e_d cos(argp_d) - e_c cos(argp_c), e_d sin(argp_d) - e_c sin(argp_c).
    dix : float
        i_d - i_c [rad].
    diy : float
        (raan_d - raan_c) sin(i_c) [rad].

    Author: Giovanni Facchinetti, 2026
    Reference: Koenig, Guffanti, D'Amico, "New State Transition Matrices
    for Spacecraft Relative Motion in Perturbed Orbits," Journal of
    Guidance, Control, and Dynamics, 40(7), 2017, Eq. (2).
    https://doi.org/10.2514/1.G002409
    """

    da: float
    dlambda: float
    dex: float
    dey: float
    dix: float
    diy: float

    def as_array(self):
        return np.array([self.da, self.dlambda, self.dex, self.dey, self.dix, self.diy])


def quasi_nonsingular_roe(chief, deputy):
    """
    Quasi-nonsingular ROE of a deputy relative to a chief.

    Parameters
    ----------
    chief, deputy : ios_mission_planner.dynamics.orbital.elements.ClassicalElements

    Returns
    -------
    QuasiNonsingularROE

    Author: Giovanni Facchinetti, 2026
    Reference: Koenig, Guffanti, D'Amico (2017), Eq. (2).
    """

    d_raan = _wrap_difference(deputy.raan - chief.raan)
    d_mean_arg_lat = _wrap_difference((deputy.mean_anomaly + deputy.argp) - (chief.mean_anomaly + chief.argp))

    da = (deputy.a - chief.a) / chief.a
    dlambda = d_mean_arg_lat + d_raan * np.cos(chief.i)
    dex = deputy.e * np.cos(deputy.argp) - chief.e * np.cos(chief.argp)
    dey = deputy.e * np.sin(deputy.argp) - chief.e * np.sin(chief.argp)
    dix = deputy.i - chief.i
    diy = d_raan * np.sin(chief.i)

    return QuasiNonsingularROE(da, dlambda, dex, dey, dix, diy)


def deputy_elements_from_roe(chief, roe):
    """
    Deputy classical elements that produce a given quasi-nonsingular ROE
    relative to a chief. Inverse of quasi_nonsingular_roe.

    Parameters
    ----------
    chief : ios_mission_planner.dynamics.orbital.elements.ClassicalElements
    roe : QuasiNonsingularROE

    Returns
    -------
    ios_mission_planner.dynamics.orbital.elements.ClassicalElements

    Author: Giovanni Facchinetti, 2026
    Reference: Koenig, Guffanti, D'Amico (2017), Eq. (2), inverted.
    """

    a_d = chief.a * (1.0 + roe.da)
    i_d = chief.i + roe.dix
    raan_d = chief.raan + roe.diy / np.sin(chief.i)

    ex_d = roe.dex + chief.e * np.cos(chief.argp)
    ey_d = roe.dey + chief.e * np.sin(chief.argp)
    e_d = np.hypot(ex_d, ey_d)
    argp_d = np.arctan2(ey_d, ex_d)

    mean_anomaly_d = (roe.dlambda - (raan_d - chief.raan) * np.cos(chief.i)
                       + (chief.mean_anomaly + chief.argp) - argp_d)

    return ClassicalElements(a=a_d, e=e_d, i=i_d, raan=raan_d % (2 * np.pi),
                              argp=argp_d % (2 * np.pi), mean_anomaly=mean_anomaly_d % (2 * np.pi))


def j2_quasi_nonsingular_stm(chief, tau, mu, j2, r_eq):
    """
    State transition matrix for the quasi-nonsingular ROE under the
    secular effect of J2, for a chief orbit of arbitrary eccentricity.

    Parameters
    ----------
    chief : ios_mission_planner.dynamics.orbital.elements.ClassicalElements
        Chief's osculating elements at the start of the propagation
        interval.
    tau : float
        Propagation time [s].
    mu, j2, r_eq : float
        Gravitational parameter, J2 coefficient, and equatorial radius
        of the central body, in units consistent with chief.a.

    Returns
    -------
    numpy.ndarray, shape (6, 6)
        STM such that roe(t_i + tau) = Phi @ roe(t_i), for ROE ordered
        [da, dlambda, dex, dey, dix, diy].

    Author: Giovanni Facchinetti, 2026
    Reference: Koenig, Guffanti, D'Amico (2017), Appendix A.3, Eq. (A6),
    combined with the state transformation of Eq. (20)-(21), (25).
    https://doi.org/10.2514/1.G002409
    """

    a, e, i, argp_i = chief.a, chief.e, chief.i, chief.argp
    n = np.sqrt(mu / a**3)
    eta = np.sqrt(1 - e**2)
    kappa = 0.75 * j2 * r_eq**2 * np.sqrt(mu) / (a**3.5 * eta**4)

    E = 1 + eta
    F = 4 + 3 * eta
    G = 1 / eta**2
    P = 3 * np.cos(i) ** 2 - 1
    Q = 5 * np.cos(i) ** 2 - 1
    R = np.cos(i)
    S = np.sin(2 * i)
    T = np.sin(i) ** 2

    argp_dot = kappa * Q
    argp_f = argp_i + argp_dot * tau

    e_xi, e_yi = e * np.cos(argp_i), e * np.sin(argp_i)
    e_xf, e_yf = e * np.cos(argp_f), e * np.sin(argp_f)

    n_term = 1.5 * n + 3.5 * kappa * E * P
    c, s = np.cos(argp_dot * tau), np.sin(argp_dot * tau)

    phi_prime = np.array([
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [-n_term * tau, 1.0, kappa * e_xi * F * G * P * tau, kappa * e_yi * F * G * P * tau,
         -kappa * F * S * tau, 0.0],
        [3.5 * kappa * e_yf * Q * tau, 0.0,
         c - 4 * kappa * e_xi * e_yf * G * Q * tau,
         -s - 4 * kappa * e_yi * e_yf * G * Q * tau,
         5 * kappa * e_yf * S * tau, 0.0],
        [-3.5 * kappa * e_xf * Q * tau, 0.0,
         s + 4 * kappa * e_xi * e_xf * G * Q * tau,
         c + 4 * kappa * e_yi * e_xf * G * Q * tau,
         -5 * kappa * e_xf * S * tau, 0.0],
        [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        [3.5 * kappa * S * tau, 0.0, -4 * kappa * e_xi * G * S * tau, -4 * kappa * e_yi * G * S * tau,
         2 * kappa * T * tau, 1.0],
    ])

    def eccentricity_rotation(argp):
        cw, sw = np.cos(argp), np.sin(argp)
        rot = np.eye(6)
        rot[2, 2], rot[2, 3] = cw, sw
        rot[3, 2], rot[3, 3] = -sw, cw
        return rot

    j_initial = eccentricity_rotation(argp_i)
    j_final_inv = eccentricity_rotation(argp_f).T  # inverse of a rotation is its transpose

    return j_final_inv @ phi_prime @ j_initial
