import heyoka as hy

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
