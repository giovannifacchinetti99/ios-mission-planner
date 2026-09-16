from dataclasses import dataclass

import numpy as np


@dataclass
class OrbitalElements:
    """
    Classical (Keplerian) orbital elements.

    Attributes
    ----------
    a : float
        Semi-major axis [km].
    e : float
        Eccentricity.
    i : float
        Inclination [rad].
    raan : float
        Right ascension of the ascending node [rad].
    argp : float
        Argument of periapsis [rad].
    nu : float
        True anomaly [rad].

    Author: Giovanni Facchinetti, 2026
    """

    a: float
    e: float
    i: float
    raan: float
    argp: float
    nu: float


def solve_kepler_equation(M, e, tol=1e-12, max_iter=50):
    """
    Solve Kepler's equation M = E - e*sin(E) for the eccentric anomaly.

    Parameters
    ----------
    M : float
        Mean anomaly [rad].
    e : float
        Eccentricity (elliptical orbits, 0 <= e < 1).
    tol : float, optional
        Convergence tolerance on the residual (default: 1e-12).
    max_iter : int, optional
        Maximum number of Newton-Raphson iterations (default: 50).

    Returns
    -------
    float
        Eccentric anomaly E [rad].

    Author: Giovanni Facchinetti, 2026
    """

    M = np.mod(M, 2.0 * np.pi)
    E = M + e * np.sin(M) if e < 0.8 else np.pi

    for _ in range(max_iter):
        residual = E - e * np.sin(E) - M
        if abs(residual) < tol:
            break
        E -= residual / (1.0 - e * np.cos(E))

    return E


def true_to_eccentric_anomaly(nu, e):
    """
    Convert true anomaly to eccentric anomaly.

    Author: Giovanni Facchinetti, 2026
    """

    return 2.0 * np.arctan2(
        np.sqrt(1.0 - e) * np.sin(nu / 2.0),
        np.sqrt(1.0 + e) * np.cos(nu / 2.0),
    )


def eccentric_to_true_anomaly(E, e):
    """
    Convert eccentric anomaly to true anomaly.

    Author: Giovanni Facchinetti, 2026
    """

    return 2.0 * np.arctan2(
        np.sqrt(1.0 + e) * np.sin(E / 2.0),
        np.sqrt(1.0 - e) * np.cos(E / 2.0),
    )


def eccentric_to_mean_anomaly(E, e):
    """
    Convert eccentric anomaly to mean anomaly (Kepler's equation).

    Author: Giovanni Facchinetti, 2026
    """

    return E - e * np.sin(E)


def state_vector_to_elements(state, mu):
    """
    Convert an inertial state vector to classical orbital elements.

    Note: uses the node vector and eccentricity vector directly, so it is
    not numerically robust for exactly circular and/or equatorial orbits
    (undefined argument of periapsis / RAAN). Sufficient for the elliptical,
    inclined orbits this planner targets.

    Parameters
    ----------
    state : ndarray
        Inertial state vector [x, y, z, vx, vy, vz].
    mu : float
        Gravitational parameter of the central body [km^3/s^2].

    Returns
    -------
    OrbitalElements

    Author: Giovanni Facchinetti, 2026
    Reference: Vallado, Fundamentals of Astrodynamics and Applications
    """

    r = state[:3]
    v = state[3:]

    r_norm = np.linalg.norm(r)
    v_norm = np.linalg.norm(v)

    h = np.cross(r, v)
    h_norm = np.linalg.norm(h)

    k = np.array([0.0, 0.0, 1.0])
    n = np.cross(k, h)
    n_norm = np.linalg.norm(n)

    e_vec = np.cross(v, h) / mu - r / r_norm
    e = np.linalg.norm(e_vec)

    energy = 0.5 * v_norm**2 - mu / r_norm
    a = -mu / (2.0 * energy)

    i = np.arccos(np.clip(h[2] / h_norm, -1.0, 1.0))

    raan = np.arccos(np.clip(n[0] / n_norm, -1.0, 1.0))
    if n[1] < 0.0:
        raan = 2.0 * np.pi - raan

    argp = np.arccos(np.clip(np.dot(n, e_vec) / (n_norm * e), -1.0, 1.0))
    if e_vec[2] < 0.0:
        argp = 2.0 * np.pi - argp

    nu = np.arccos(np.clip(np.dot(e_vec, r) / (e * r_norm), -1.0, 1.0))
    if np.dot(r, v) < 0.0:
        nu = 2.0 * np.pi - nu

    return OrbitalElements(a=a, e=e, i=i, raan=raan, argp=argp, nu=nu)


def elements_to_state_vector(elements, mu):
    """
    Convert classical orbital elements to an inertial state vector.

    Parameters
    ----------
    elements : OrbitalElements
    mu : float
        Gravitational parameter of the central body [km^3/s^2].

    Returns
    -------
    ndarray
        Inertial state vector [x, y, z, vx, vy, vz].

    Author: Giovanni Facchinetti, 2026
    Reference: Vallado, Fundamentals of Astrodynamics and Applications
    """

    a, e, i, raan, argp, nu = (
        elements.a,
        elements.e,
        elements.i,
        elements.raan,
        elements.argp,
        elements.nu,
    )

    p = a * (1.0 - e**2)
    r_mag = p / (1.0 + e * np.cos(nu))

    r_pf = r_mag * np.array([np.cos(nu), np.sin(nu), 0.0])
    v_pf = np.sqrt(mu / p) * np.array([-np.sin(nu), e + np.cos(nu), 0.0])

    cr, sr = np.cos(raan), np.sin(raan)
    ci, si = np.cos(i), np.sin(i)
    cw, sw = np.cos(argp), np.sin(argp)

    rotation = np.array([
        [cr * cw - sr * sw * ci, -cr * sw - sr * cw * ci, sr * si],
        [sr * cw + cr * sw * ci, -sr * sw + cr * cw * ci, -cr * si],
        [sw * si, cw * si, ci],
    ])

    return np.concatenate([rotation @ r_pf, rotation @ v_pf])
