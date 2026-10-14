from dataclasses import dataclass

import numpy as np


@dataclass
class ClassicalElements:
    """
    Classical Keplerian orbital elements.

    Attributes
    ----------
    a : float
        Semi-major axis, same length unit as the input state.
    e : float
        Eccentricity.
    i : float
        Inclination [rad].
    raan : float
        Right ascension of the ascending node [rad].
    argp : float
        Argument of perigee [rad]. For a near-circular orbit (e below the
        tolerance in classical_elements_from_state), this is set to 0 by
        convention and mean_anomaly is the argument of latitude instead.
    mean_anomaly : float
        Mean anomaly [rad].
    """

    a: float
    e: float
    i: float
    raan: float
    argp: float
    mean_anomaly: float


def classical_elements_from_state(state, mu, e_tol=1e-8):
    """
    Classical orbital elements from a Cartesian inertial state.

    Parameters
    ----------
    state : array_like
        Inertial state [x, y, z, vx, vy, vz].
    mu : float
        Gravitational parameter of the central body.
    e_tol : float
        Below this eccentricity, the orbit is treated as circular: argp is
        set to 0 and mean_anomaly is the argument of latitude (the angle
        from the ascending node to the position vector) instead, since the
        perigee direction is not meaningfully defined.

    Returns
    -------
    ClassicalElements

    Author: Giovanni Facchinetti, 2026
    Reference: Vallado, Fundamentals of Astrodynamics and Applications
    """

    state = np.asarray(state, dtype=float)
    r, v = state[:3], state[3:]
    r_mag, v_mag = np.linalg.norm(r), np.linalg.norm(v)

    h = np.cross(r, v)
    h_mag = np.linalg.norm(h)
    node = np.cross([0.0, 0.0, 1.0], h)
    node_mag = np.linalg.norm(node)

    e_vec = ((v_mag**2 - mu / r_mag) * r - np.dot(r, v) * v) / mu
    e = np.linalg.norm(e_vec)

    energy = 0.5 * v_mag**2 - mu / r_mag
    a = -mu / (2.0 * energy)

    i = np.arccos(np.clip(h[2] / h_mag, -1.0, 1.0))
    raan = np.arctan2(node[1], node[0]) if node_mag > 0 else 0.0

    if e > e_tol:
        cos_argp = np.clip(np.dot(node, e_vec) / (node_mag * e), -1.0, 1.0)
        argp = np.arccos(cos_argp)
        if e_vec[2] < 0:
            argp = 2 * np.pi - argp

        cos_nu = np.clip(np.dot(e_vec, r) / (e * r_mag), -1.0, 1.0)
        nu = np.arccos(cos_nu)
        if np.dot(r, v) < 0:
            nu = 2 * np.pi - nu

        ecc_anomaly = 2 * np.arctan2(np.sqrt(1 - e) * np.sin(nu / 2), np.sqrt(1 + e) * np.cos(nu / 2))
        mean_anomaly = ecc_anomaly - e * np.sin(ecc_anomaly)
    else:
        argp = 0.0
        cos_u = np.clip(np.dot(node, r) / (node_mag * r_mag), -1.0, 1.0) if node_mag > 0 else np.clip(r[0] / r_mag, -1.0, 1.0)
        mean_anomaly = np.arccos(cos_u)
        if r[2] < 0:
            mean_anomaly = 2 * np.pi - mean_anomaly

    return ClassicalElements(a=a, e=e, i=i, raan=raan % (2 * np.pi), argp=argp % (2 * np.pi),
                              mean_anomaly=mean_anomaly % (2 * np.pi))


def _eccentric_from_mean_anomaly(mean_anomaly, e, tol=1e-12, max_iter=50):
    """Eccentric anomaly from mean anomaly, by Newton's method on Kepler's equation."""

    ecc_anomaly = mean_anomaly if e < 0.8 else np.pi
    for _ in range(max_iter):
        delta = (ecc_anomaly - e * np.sin(ecc_anomaly) - mean_anomaly) / (1.0 - e * np.cos(ecc_anomaly))
        ecc_anomaly -= delta
        if abs(delta) < tol:
            break
    return ecc_anomaly


def cartesian_state_from_elements(elements, mu):
    """
    Cartesian inertial state from classical orbital elements. Inverse of
    classical_elements_from_state.

    Parameters
    ----------
    elements : ClassicalElements
    mu : float
        Gravitational parameter of the central body.

    Returns
    -------
    numpy.ndarray
        Inertial state [x, y, z, vx, vy, vz].

    Author: Giovanni Facchinetti, 2026
    Reference: Vallado, Fundamentals of Astrodynamics and Applications
    """

    a, e, i = elements.a, elements.e, elements.i
    raan, argp, mean_anomaly = elements.raan, elements.argp, elements.mean_anomaly

    ecc_anomaly = _eccentric_from_mean_anomaly(mean_anomaly, e)
    nu = 2 * np.arctan2(np.sqrt(1 + e) * np.sin(ecc_anomaly / 2), np.sqrt(1 - e) * np.cos(ecc_anomaly / 2))

    p = a * (1.0 - e**2)
    r_pf = p / (1.0 + e * np.cos(nu)) * np.array([np.cos(nu), np.sin(nu), 0.0])
    v_pf = np.sqrt(mu / p) * np.array([-np.sin(nu), e + np.cos(nu), 0.0])

    cO, sO = np.cos(raan), np.sin(raan)
    ci, si = np.cos(i), np.sin(i)
    cw, sw = np.cos(argp), np.sin(argp)
    rotation = np.array([
        [cO * cw - sO * sw * ci, -cO * sw - sO * cw * ci, sO * si],
        [sO * cw + cO * sw * ci, -sO * sw + cO * cw * ci, -cO * si],
        [sw * si, cw * si, ci],
    ])

    return np.concatenate([rotation @ r_pf, rotation @ v_pf])
