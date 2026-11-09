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


def low_thrust_arc_tangential_dynamics_symbolic():
    """
    Two-body motion with velocity-tracking low-thrust propulsion, throttled
    on only within an angular arc around a fixed inertial direction.

    This generalizes low_thrust_tangential_dynamics_symbolic with a single
    extra idea: instead of thrusting continuously, the engine fires only
    while the spacecraft's position is within half_angle of a chosen
    direction, and coasts the rest of each orbit. Thrusting continuously
    matches an impulsive burn's delta-v (hence its energy, hence its final
    semi-major axis) but not its final orbit shape, since it spreads the
    same energy change evenly around the whole orbit instead of
    concentrating it where an impulsive burn would apply it; narrowing the
    arc interpolates between that continuous, nearly-circularizing limit
    (half_angle = pi, on for the whole orbit) and the impulsive limit
    (half_angle -> 0, a brief burn concentrated at one point), building
    eccentricity instead of just spreading energy, which is what is needed
    to actually drive one side of the orbit down to a target radius rather
    than just matching the impulsive burn's delta-v.

    The on/off switch is a smooth (tanh) gate rather than a hard one, so the
    system stays a plain symbolic expression heyoka can integrate directly,
    with no discontinuity event to detect. The gate also throttles the mass
    flow, not just the thrust acceleration, so propellant is only spent
    while the engine is actually (smoothly) on.

    State vector: [x, y, z, vx, vy, vz, m], position in km, velocity in
    km/s, mass in kg.
    Runtime parameters: par[0] = mu [km^3/s^2], par[1] = F, thrust force
    [N], par[2] = isp, specific impulse [s], par[3] = sign, +1 prograde or
    -1 retrograde, par[4:7] = (ex, ey, ez), the (not necessarily normalized)
    inertial direction marking the center of the thrust arc, par[7] =
    half_angle, half-width of the arc [rad], par[8] = steepness of the
    smooth gate (larger is closer to a hard on/off switch; of order
    50-200 works well for half_angle of a few tenths of a radian or more).

    Author: Giovanni Facchinetti, 2026
    """

    x, y, z, vx, vy, vz, m = hy.make_vars("x", "y", "z", "vx", "vy", "vz", "m")
    mu = hy.par[0]
    thrust, isp, sign = hy.par[1], hy.par[2], hy.par[3]
    ex, ey, ez = hy.par[4], hy.par[5], hy.par[6]
    half_angle, steepness = hy.par[7], hy.par[8]

    ax_grav, ay_grav, az_grav = two_body_acceleration_symbolic(x, y, z, mu)

    r = hy.sqrt(x**2 + y**2 + z**2)
    speed = hy.sqrt(vx**2 + vy**2 + vz**2)
    e_norm = hy.sqrt(ex**2 + ey**2 + ez**2)

    cos_angle = (x * ex + y * ey + z * ez) / (r * e_norm)
    gate = 0.5 * (1.0 + hy.tanh(steepness * (cos_angle - hy.cos(half_angle))))

    thrust_accel = gate * thrust / m * 1e-3  # N / kg = m/s^2 -> km/s^2
    tangential = sign * thrust_accel / speed

    return [
        (x, vx),
        (y, vy),
        (z, vz),
        (vx, ax_grav + tangential * vx),
        (vy, ay_grav + tangential * vy),
        (vz, az_grav + tangential * vz),
        (m, -gate * thrust / (isp * G0)),
    ]
