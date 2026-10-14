import heyoka as hy

from ios_mission_planner.budget.propellant import G0

from .two_body import two_body_acceleration_symbolic


def low_thrust_dynamics_symbolic():
    """
    Two-body motion with constant-direction, constant-magnitude low-thrust
    propulsion, as a symbolic heyoka ODE system.

    Extends the inertial two-body equations with mass as a state that
    depletes through the differential form of the rocket equation, and a
    thrust acceleration along a fixed inertial direction. The thrust is
    "on" for as long as the integration runs; the caller starts and stops
    the burn by choosing the propagation span, the same way an impulsive
    burn's duration is chosen elsewhere in this package.

    State vector: [x, y, z, vx, vy, vz, m], position in km, velocity in
    km/s, mass in kg.
    Runtime parameters: par[0] = mu [km^3/s^2], par[1] = F, thrust force
    [N], par[2] = isp, specific impulse [s], par[3:6] = (ux, uy, uz), the
    unit vector of the thrust direction in the inertial frame.

    Returns
    -------
    list[tuple[heyoka.expression, heyoka.expression]]
        Symbolic system, for use with
        ios_mission_planner.propagation.heyoka_propagator.propagate.

    Author: Giovanni Facchinetti, 2026
    """

    x, y, z, vx, vy, vz, m = hy.make_vars("x", "y", "z", "vx", "vy", "vz", "m")
    mu = hy.par[0]
    thrust, isp = hy.par[1], hy.par[2]
    ux, uy, uz = hy.par[3], hy.par[4], hy.par[5]

    ax_grav, ay_grav, az_grav = two_body_acceleration_symbolic(x, y, z, mu)
    thrust_accel = thrust / m * 1e-3  # N / kg = m/s^2 -> km/s^2

    return [
        (x, vx),
        (y, vy),
        (z, vz),
        (vx, ax_grav + thrust_accel * ux),
        (vy, ay_grav + thrust_accel * uy),
        (vz, az_grav + thrust_accel * uz),
        (m, -thrust / (isp * G0)),
    ]
