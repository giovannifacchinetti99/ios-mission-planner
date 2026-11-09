import heyoka as hy

from .perturbations import j2_acceleration_symbolic
from .two_body import two_body_acceleration_symbolic


def exponential_atmosphere_density(r, rho0, r0, scale_height):
    """
    Exponential atmosphere density model, as a symbolic heyoka expression.

    rho(r) = rho0 * exp(-(r - r0) / scale_height), the standard simple
    model for atmospheric density decaying with altitude: a single
    reference density rho0 at reference radius r0, falling off
    exponentially with a scale height characteristic of the local
    altitude band. Real atmospheric density varies with solar activity,
    local time, and latitude well beyond what one exponential band
    captures; this is the first-order model good enough to size a drag
    perturbation's order of magnitude, not a precise density forecast.

    Parameters
    ----------
    r : heyoka.expression
        Radius from the central body's center [km].
    rho0 : heyoka.expression
        Reference density at r0 [kg/m^3].
    r0 : heyoka.expression
        Reference radius [km].
    scale_height : heyoka.expression
        Scale height of the local band [km].

    Returns
    -------
    heyoka.expression
        Density [kg/m^3].

    Author: Giovanni Facchinetti, 2026
    Reference: Vallado, Fundamentals of Astrodynamics and Applications
    """

    return rho0 * hy.exp(-(r - r0) / scale_height)


def drag_acceleration_symbolic(x, y, z, vx, vy, vz, bc, rho0, r0, scale_height, omega_body):
    """
    Atmospheric drag perturbing acceleration, as symbolic heyoka
    expressions.

    a_drag = -0.5 * rho * bc * |v_rel| * v_rel, with v_rel the velocity
    relative to the atmosphere, which corotates with the central body at
    omega_body about its z-axis: v_rel = v - omega_body x r. bc = Cd*A/m is
    the ballistic coefficient (drag coefficient times cross-sectional area
    over mass): a larger bc means more drag for the same density and
    velocity, which is exactly the lever differential drag pulls on, since
    two objects sharing an orbit but differing in bc decelerate at
    different rates.

    Parameters
    ----------
    x, y, z, vx, vy, vz : heyoka.expression
        Inertial position [km] and velocity [km/s] components.
    bc : heyoka.expression
        Ballistic coefficient, Cd*A/m [m^2/kg].
    rho0, r0, scale_height : heyoka.expression
        Exponential_atmosphere_density parameters.
    omega_body : heyoka.expression
        Rotation rate of the central body (and its atmosphere) about its
        z-axis [rad/s].

    Returns
    -------
    tuple[heyoka.expression, heyoka.expression, heyoka.expression]
        (ax, ay, az) [km/s^2].

    Author: Giovanni Facchinetti, 2026
    Reference: Vallado, Fundamentals of Astrodynamics and Applications
    """

    r = hy.sqrt(x**2 + y**2 + z**2)
    rho = exponential_atmosphere_density(r, rho0, r0, scale_height)

    vx_rel = vx + omega_body * y
    vy_rel = vy - omega_body * x
    vz_rel = vz

    v_rel = hy.sqrt(vx_rel**2 + vy_rel**2 + vz_rel**2)

    # rho [kg/m^3], bc [m^2/kg], v_rel [km/s]: converting v_rel to m/s and
    # the result back to km/s^2 leaves a net factor of 1e3 (1000^2 / 1000)
    factor = -0.5 * rho * bc * v_rel * 1e3

    return factor * vx_rel, factor * vy_rel, factor * vz_rel


def two_body_drag_dynamics_symbolic():
    """
    Two-body equations of motion with atmospheric drag, in an inertial
    frame, as a symbolic heyoka ODE system.

    State vector: [x, y, z, vx, vy, vz].
    Runtime parameters: par[0] = mu, par[1] = bc (Cd*A/m [m^2/kg]),
    par[2] = rho0, par[3] = r0, par[4] = scale_height, par[5] = omega_body.

    Returns
    -------
    list[tuple[heyoka.expression, heyoka.expression]]
        Symbolic system, for use with
        ios_mission_planner.propagation.heyoka_propagator.propagate.

    Author: Giovanni Facchinetti, 2026
    """

    x, y, z, vx, vy, vz = hy.make_vars("x", "y", "z", "vx", "vy", "vz")
    mu, bc, rho0, r0, scale_height, omega_body = (
        hy.par[0], hy.par[1], hy.par[2], hy.par[3], hy.par[4], hy.par[5],
    )

    ax_2b, ay_2b, az_2b = two_body_acceleration_symbolic(x, y, z, mu)
    ax_d, ay_d, az_d = drag_acceleration_symbolic(
        x, y, z, vx, vy, vz, bc, rho0, r0, scale_height, omega_body,
    )

    return [
        (x, vx),
        (y, vy),
        (z, vz),
        (vx, ax_2b + ax_d),
        (vy, ay_2b + ay_d),
        (vz, az_2b + az_d),
    ]


def two_body_j2_drag_dynamics_symbolic():
    """
    Two-body equations of motion with J2 and atmospheric drag, in an
    inertial frame, as a symbolic heyoka ODE system.

    State vector: [x, y, z, vx, vy, vz].
    Runtime parameters: par[0] = mu, par[1] = j2, par[2] = r_eq, par[3] =
    bc (Cd*A/m [m^2/kg]), par[4] = rho0, par[5] = r0, par[6] =
    scale_height, par[7] = omega_body.

    Returns
    -------
    list[tuple[heyoka.expression, heyoka.expression]]
        Symbolic system, for use with
        ios_mission_planner.propagation.heyoka_propagator.propagate.

    Author: Giovanni Facchinetti, 2026
    """

    x, y, z, vx, vy, vz = hy.make_vars("x", "y", "z", "vx", "vy", "vz")
    mu, j2, r_eq = hy.par[0], hy.par[1], hy.par[2]
    bc, rho0, r0, scale_height, omega_body = (
        hy.par[3], hy.par[4], hy.par[5], hy.par[6], hy.par[7],
    )

    ax_2b, ay_2b, az_2b = two_body_acceleration_symbolic(x, y, z, mu)
    ax_j2, ay_j2, az_j2 = j2_acceleration_symbolic(x, y, z, mu, j2, r_eq)
    ax_d, ay_d, az_d = drag_acceleration_symbolic(
        x, y, z, vx, vy, vz, bc, rho0, r0, scale_height, omega_body,
    )

    return [
        (x, vx),
        (y, vy),
        (z, vz),
        (vx, ax_2b + ax_j2 + ax_d),
        (vy, ay_2b + ay_j2 + ay_d),
        (vz, az_2b + az_j2 + az_d),
    ]
