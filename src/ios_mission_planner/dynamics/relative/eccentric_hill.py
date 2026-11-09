import heyoka as hy

from ..orbital.two_body import two_body_acceleration_symbolic


def eccentric_hill_equations_symbolic():
    """
    Linearized relative motion about a chief on an unperturbed Keplerian
    orbit of arbitrary eccentricity, as a symbolic heyoka ODE system.

    hill_equations_symbolic assumes a circular chief, with a constant
    angular rate n; here the chief's own absolute state is propagated
    alongside the deputy's relative state, so its radius r_c, angular rate
    theta_dot = h_c / r_c^2 and angular acceleration theta_ddot feed the
    relative equations as they actually evolve, rather than as constants.
    The relative motion itself is still linearized to first order in the
    relative position, exactly as in the circular case; only the
    assumption that the chief is circular is dropped. Reduces exactly to
    hill_equations_symbolic when the chief happens to be circular (r_c
    constant, theta_dot = n, theta_ddot = 0).

    State vector: [xc, yc, zc, vxc, vyc, vzc, x, y, z, vx, vy, vz]: the
    chief's own inertial state, then the deputy's relative state [x, y, z,
    vx, vy, vz] in the chief's LVLH frame (x radial, y along-track, z
    cross-track, matching ios_mission_planner.dynamics.relative.lvlh).
    Runtime parameter: par[0] = mu, gravitational parameter of the central
    body [km^3/s^2].

    Returns
    -------
    list[tuple[heyoka.expression, heyoka.expression]]
        Symbolic system, for use with
        ios_mission_planner.propagation.heyoka_propagator.propagate.

    Author: Giovanni Facchinetti, 2026
    Reference: Analytical Mechanics of Space Systems, Fourth Edition
    """

    xc, yc, zc, vxc, vyc, vzc = hy.make_vars("xc", "yc", "zc", "vxc", "vyc", "vzc")
    x, y, z, vx, vy, vz = hy.make_vars("x", "y", "z", "vx", "vy", "vz")
    mu = hy.par[0]

    axc, ayc, azc = two_body_acceleration_symbolic(xc, yc, zc, mu)

    r_c = hy.sqrt(xc**2 + yc**2 + zc**2)
    h_c = hy.sqrt(
        (yc * vzc - zc * vyc) ** 2 + (zc * vxc - xc * vzc) ** 2 + (xc * vyc - yc * vxc) ** 2
    )
    r_dot = (xc * vxc + yc * vyc + zc * vzc) / r_c

    theta_dot = h_c / r_c**2
    theta_ddot = -2.0 * theta_dot * r_dot / r_c
    mu_over_rc3 = mu / r_c**3

    ax = 2.0 * theta_dot * vy + theta_ddot * y + (theta_dot**2 + 2.0 * mu_over_rc3) * x
    ay = -2.0 * theta_dot * vx - theta_ddot * x + (theta_dot**2 - mu_over_rc3) * y
    az = -mu_over_rc3 * z

    return [
        (xc, vxc),
        (yc, vyc),
        (zc, vzc),
        (vxc, axc),
        (vyc, ayc),
        (vzc, azc),
        (x, vx),
        (y, vy),
        (z, vz),
        (vx, ax),
        (vy, ay),
        (vz, az),
    ]
