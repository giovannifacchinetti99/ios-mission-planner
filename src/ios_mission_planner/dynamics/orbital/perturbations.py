import heyoka as hy
import numpy as np

from ios_mission_planner.dynamics.orbital.two_body import two_body_acceleration_symbolic


def j2_acceleration_symbolic(x, y, z, mu, j2, r_eq):
    """
    J2 (oblateness) perturbing acceleration, as symbolic heyoka expressions.

    Parameters
    ----------
    x, y, z : heyoka.expression
        Inertial position components.
    mu : heyoka.expression
        Gravitational parameter of the central body [km^3/s^2].
    j2 : heyoka.expression
        J2 zonal harmonic coefficient of the central body.
    r_eq : heyoka.expression
        Equatorial radius of the central body [km].

    Returns
    -------
    tuple[heyoka.expression, heyoka.expression, heyoka.expression]
        (ax, ay, az) [km/s^2].

    Author: Giovanni Facchinetti, 2026
    Reference: Vallado, Fundamentals of Astrodynamics and Applications
    """

    r = hy.sqrt(x**2 + y**2 + z**2)
    factor = 1.5 * j2 * mu * r_eq**2 / r**5
    z_ratio_sq = (z / r) ** 2

    ax = factor * x * (5.0 * z_ratio_sq - 1.0)
    ay = factor * y * (5.0 * z_ratio_sq - 1.0)
    az = factor * z * (5.0 * z_ratio_sq - 3.0)

    return ax, ay, az


def two_body_j2_dynamics_symbolic():
    """
    Two-body equations of motion with J2 perturbation, in an inertial
    frame, as a symbolic heyoka ODE system.

    State vector: [x, y, z, vx, vy, vz].
    Runtime parameters: par[0] = mu, par[1] = j2, par[2] = r_eq.

    Returns
    -------
    list[tuple[heyoka.expression, heyoka.expression]]
        Symbolic system, for use with
        ios_mission_planner.propagation.heyoka_propagator.propagate.

    Author: Giovanni Facchinetti, 2026
    """

    x, y, z, vx, vy, vz = hy.make_vars("x", "y", "z", "vx", "vy", "vz")
    mu, j2, r_eq = hy.par[0], hy.par[1], hy.par[2]

    ax_2b, ay_2b, az_2b = two_body_acceleration_symbolic(x, y, z, mu)
    ax_j2, ay_j2, az_j2 = j2_acceleration_symbolic(x, y, z, mu, j2, r_eq)

    return [
        (x, vx),
        (y, vy),
        (z, vz),
        (vx, ax_2b + ax_j2),
        (vy, ay_2b + ay_j2),
        (vz, az_2b + az_j2),
    ]


def j2_secular_rates(a, e, i, mu, j2, r_eq):
    """
    Secular (orbit-averaged) drift rates of the classical elements under
    J2, to first order in J2.

    Averaging the J2 perturbation over one orbit leaves a, e and i
    constant and makes the three angles drift linearly in time: the node
    regresses (for a prograde orbit), the perigee rotates within the orbit
    plane, and the mean anomaly advances at a rate slightly different from
    the Keplerian mean motion.

    Parameters
    ----------
    a : float
        Semi-major axis [km].
    e : float
        Eccentricity.
    i : float
        Inclination [rad].
    mu : float
        Gravitational parameter of the central body [km^3/s^2].
    j2 : float
        J2 zonal harmonic coefficient.
    r_eq : float
        Equatorial radius of the central body [km].

    Returns
    -------
    tuple[float, float, float]
        (raan_dot, argp_dot, mean_anomaly_dot_j2) [rad/s]. The third
        entry is the J2 correction only, to be added to the Keplerian
        mean motion sqrt(mu / a^3).

    Author: Giovanni Facchinetti, 2026
    Reference: Vallado, Fundamentals of Astrodynamics and Applications
    """

    n = np.sqrt(mu / a**3)
    eta = np.sqrt(1.0 - e**2)
    p = a * (1.0 - e**2)
    k = 0.75 * j2 * (r_eq / p) ** 2 * n

    raan_dot = -2.0 * k * np.cos(i)
    argp_dot = k * (5.0 * np.cos(i) ** 2 - 1.0)
    mean_anomaly_dot_j2 = k * eta * (3.0 * np.cos(i) ** 2 - 1.0)

    return raan_dot, argp_dot, mean_anomaly_dot_j2


def sun_synchronous_inclination(a, e, mu, j2, r_eq, year_seconds=365.2421897 * 86400.0):
    """
    Inclination at which the J2 nodal drift matches the apparent mean
    motion of the Sun around the Earth, so that the orbit plane keeps a
    fixed orientation with respect to the Sun over the year.

    Parameters
    ----------
    a : float
        Semi-major axis [km].
    e : float
        Eccentricity.
    mu, j2, r_eq : float
        Gravitational parameter [km^3/s^2], J2 coefficient and equatorial
        radius [km] of the central body.
    year_seconds : float, optional
        Length of the tropical year [s].

    Returns
    -------
    float
        Inclination [rad], always retrograde (above 90 degrees).

    Raises
    ------
    ValueError
        If the orbit is too high for J2 to produce the required drift at
        any inclination.

    Author: Giovanni Facchinetti, 2026
    Reference: Vallado, Fundamentals of Astrodynamics and Applications
    """

    n = np.sqrt(mu / a**3)
    p = a * (1.0 - e**2)
    target = 2.0 * np.pi / year_seconds
    cos_i = -target / (1.5 * j2 * (r_eq / p) ** 2 * n)

    if abs(cos_i) > 1.0:
        raise ValueError("no sun-synchronous inclination exists at this altitude")

    return float(np.arccos(cos_i))
