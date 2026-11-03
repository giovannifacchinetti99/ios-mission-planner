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


def low_thrust_tangential_dynamics_symbolic():
    """
    Two-body motion with continuous low-thrust propulsion steered along
    the instantaneous velocity direction, as a symbolic heyoka ODE system.

    Unlike low_thrust_dynamics_symbolic, the thrust direction here is not
    a fixed inertial vector: it is +-v/|v|, recomputed at every instant
    from the velocity state itself, so it keeps tracking prograde (or
    retrograde) as gravity turns the velocity vector. This is the
    standard continuous-tangential-thrust model for orbit raising or
    lowering, and it is exact for burns of any duration, unlike the
    fixed-direction model, which is only a good approximation for burns
    short compared to the orbital period.

    State vector: [x, y, z, vx, vy, vz, m], position in km, velocity in
    km/s, mass in kg.
    Runtime parameters: par[0] = mu [km^3/s^2], par[1] = F, thrust force
    [N], par[2] = isp, specific impulse [s], par[3] = sign, +1 for
    prograde thrust (raises the orbit) or -1 for retrograde (lowers it).

    Returns
    -------
    list[tuple[heyoka.expression, heyoka.expression]]
        Symbolic system, for use with
        ios_mission_planner.propagation.heyoka_propagator.propagate.

    Author: Giovanni Facchinetti, 2026
    """

    x, y, z, vx, vy, vz, m = hy.make_vars("x", "y", "z", "vx", "vy", "vz", "m")
    mu = hy.par[0]
    thrust, isp, sign = hy.par[1], hy.par[2], hy.par[3]

    ax_grav, ay_grav, az_grav = two_body_acceleration_symbolic(x, y, z, mu)
    thrust_accel = thrust / m * 1e-3  # N / kg = m/s^2 -> km/s^2

    speed = hy.sqrt(vx**2 + vy**2 + vz**2)
    tangential = sign * thrust_accel / speed

    return [
        (x, vx),
        (y, vy),
        (z, vz),
        (vx, ax_grav + tangential * vx),
        (vy, ay_grav + tangential * vy),
        (vz, az_grav + tangential * vz),
        (m, -thrust / (isp * G0)),
    ]
