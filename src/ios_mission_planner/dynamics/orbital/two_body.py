import heyoka as hy


def two_body_acceleration_symbolic(x, y, z, mu):
    """
    Two-body gravitational acceleration, as symbolic heyoka expressions.

    Parameters
    ----------
    x, y, z : heyoka.expression
        Inertial position components.
    mu : heyoka.expression
        Gravitational parameter of the central body [km^3/s^2].

    Returns
    -------
    tuple[heyoka.expression, heyoka.expression, heyoka.expression]
        (ax, ay, az) [km/s^2].

    Author: Giovanni Facchinetti, 2026
    """

    r = hy.sqrt(x**2 + y**2 + z**2)
    factor = -mu / r**3

    return factor * x, factor * y, factor * z


def two_body_dynamics_symbolic():
    """
    Two-body equations of motion in an inertial frame, as a symbolic
    heyoka ODE system.

    State vector: [x, y, z, vx, vy, vz].
    Runtime parameter: par[0] = mu, gravitational parameter of the
    central body [km^3/s^2].

    Returns
    -------
    list[tuple[heyoka.expression, heyoka.expression]]
        Symbolic system, for use with
        ios_mission_planner.propagation.heyoka_propagator.propagate.

    Author: Giovanni Facchinetti, 2026
    """

    x, y, z, vx, vy, vz = hy.make_vars("x", "y", "z", "vx", "vy", "vz")
    mu = hy.par[0]

    ax, ay, az = two_body_acceleration_symbolic(x, y, z, mu)

    return [
        (x, vx),
        (y, vy),
        (z, vz),
        (vx, ax),
        (vy, ay),
        (vz, az),
    ]
