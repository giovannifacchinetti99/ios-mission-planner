import numpy as np

from ios_mission_planner.budget.propellant import G0, initial_mass
from ios_mission_planner.dynamics.orbital.elements import classical_elements_from_state
from ios_mission_planner.dynamics.orbital.low_thrust import low_thrust_arc_tangential_dynamics_symbolic
from ios_mission_planner.propagation.heyoka_propagator import propagate


def arc_duty_cycle(half_angle):
    """Fraction of each orbit the thrust arc is (approximately) on, half_angle / pi."""
    return half_angle / np.pi


def deorbit_with_arc_limited_thrust(chief_state0, half_angle, delta_v_reference, m_dry, isp,
                                     thrust, mu, direction=(1.0, 0.0, 0.0), steepness=100.0,
                                     n_pts=3000):
    """
    Propagate a retrograde, arc-limited low-thrust burn and report the
    final orbit it reaches.

    The propellant mass is sized from delta_v_reference through the rocket
    equation exactly as an equivalent impulsive burn of that magnitude
    would be, independent of half_angle, so that varying half_angle
    isolates the effect of the thrust arc's width alone on a fixed
    propellant budget. The wall-clock burn duration is extended from the
    thrust-on time by 1/arc_duty_cycle(half_angle), since narrower arcs
    take proportionally longer, in wall-clock time, to spend the same
    propellant.

    Parameters
    ----------
    chief_state0 : array_like, shape (6,)
        Inertial state [x, y, z, vx, vy, vz] to start the burn from.
    half_angle : float
        Half-width of the thrust arc [rad].
    delta_v_reference : float
        The equivalent impulsive burn's delta-v [m/s], used only to size
        the propellant mass.
    m_dry : float
        Dry mass [kg].
    isp : float
        Specific impulse [s].
    thrust : float
        Thrust force while the arc gate is fully on [N].
    mu : float
        Gravitational parameter of the central body [km^3/s^2].
    direction : array_like, shape (3,)
        Inertial direction marking the center of the thrust arc.
    steepness : float
        Steepness of the arc's smooth on/off gate.
    n_pts : int
        Number of output samples.

    Returns
    -------
    elements : ios_mission_planner.dynamics.orbital.elements.ClassicalElements
        Final osculating elements.
    result : ios_mission_planner.propagation.heyoka_propagator.PropagationResult

    Author: Giovanni Facchinetti, 2026
    """

    m0 = initial_mass(delta_v_reference, isp, m_dry)
    m_prop = m0 - m_dry
    burn_time_on = m_prop / (thrust / (isp * G0))
    wall_time = burn_time_on / arc_duty_cycle(half_angle)

    state0 = np.concatenate([np.asarray(chief_state0, dtype=float), [m0]])
    ex, ey, ez = direction
    pars = [mu, thrust, isp, -1.0, ex, ey, ez, half_angle, steepness]
    t_eval = np.linspace(0.0, wall_time, n_pts)

    result = propagate(low_thrust_arc_tangential_dynamics_symbolic(), state0, t_eval, pars=pars)
    elements = classical_elements_from_state(result.y[:6, -1], mu)

    return elements, result


def solve_half_angle_for_target_perigee(chief_state0, target_perigee_radius, delta_v_reference,
                                         m_dry, isp, thrust, mu, direction=(1.0, 0.0, 0.0),
                                         steepness=100.0, n_pts=1500, tol=1.0, max_iter=25):
    """
    Bisect the thrust-arc half-angle that drives the final perigee radius
    down to (approximately) target_perigee_radius, for a fixed propellant
    budget sized from delta_v_reference.

    A narrower arc concentrates the same propellant's delta-v into less of
    each orbit, building more eccentricity and reaching a lower final
    perigee; a full-orbit arc (half_angle = pi) spreads it evenly and
    barely changes the perigee at all. Over (0, pi] this is monotonic
    enough for a plain bisection, with no need for a derivative-based
    solver. Only reachable targets, between the half_angle = pi and
    half_angle -> 0 extremes for the given propellant budget, converge to
    a solution satisfying the tolerance; an unreachable target instead
    converges to whichever bracket end is actually closest to it.

    Parameters
    ----------
    See deorbit_with_arc_limited_thrust for the shared parameters.
    target_perigee_radius : float
        Desired final perigee radius [km].
    tol : float
        Convergence tolerance on the perigee radius [km].
    max_iter : int

    Returns
    -------
    half_angle : float
        The converged (or best-effort) half-width [rad].
    perigee_radius : float
        The final perigee radius actually reached at that half-width [km].

    Author: Giovanni Facchinetti, 2026
    """

    def final_perigee(half_angle):
        elements, _ = deorbit_with_arc_limited_thrust(
            chief_state0, half_angle, delta_v_reference, m_dry, isp, thrust, mu,
            direction=direction, steepness=steepness, n_pts=n_pts,
        )
        return elements.a * (1.0 - elements.e)

    lo, hi = 1e-3, np.pi
    mid, f_mid = hi, final_perigee(hi)

    for _ in range(max_iter):
        mid = 0.5 * (lo + hi)
        f_mid = final_perigee(mid)
        if abs(f_mid - target_perigee_radius) < tol:
            break
        if f_mid > target_perigee_radius:
            hi = mid
        else:
            lo = mid

    return mid, f_mid
