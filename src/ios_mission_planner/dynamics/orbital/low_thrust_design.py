import numpy as np
import pygmo as pg

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


class ArcHalfAnglePygmoProblem:
    """
    A pygmo user-defined problem (UDP) for the arc-limited deorbit burn:
    pick the thrust-arc half-angle trading off how close the final perigee
    lands to a target radius against how long, in wall-clock time, the burn
    takes to fly.

    `solve_half_angle_for_target_perigee` already finds a single half-angle
    that hits the target, by bisection, but a mission designer rarely wants
    only the closest possible match regardless of cost: a much narrower arc
    gets measurably closer to the target perigee, but can take many times
    longer in wall-clock time to spend the same propellant (see the
    arc_duty_cycle discussion), and that trade is exactly what this UDP
    exposes as two objectives to pygmo instead of collapsing it into one
    bisection target.

    Parameters
    ----------
    See deorbit_with_arc_limited_thrust for the shared parameters.
    target_perigee_radius : float
        Desired final perigee radius [km].
    half_angle_bounds : tuple[float, float]
        Search bounds for the half-angle [rad].

    Author: Giovanni Facchinetti, 2026
    """

    def __init__(self, chief_state0, target_perigee_radius, delta_v_reference, m_dry, isp,
                 thrust, mu, direction=(1.0, 0.0, 0.0), steepness=100.0,
                 half_angle_bounds=(1e-3, np.pi)):
        self.chief_state0 = chief_state0
        self.target_perigee_radius = target_perigee_radius
        self.delta_v_reference = delta_v_reference
        self.m_dry = m_dry
        self.isp = isp
        self.thrust = thrust
        self.mu = mu
        self.direction = direction
        self.steepness = steepness
        self.half_angle_bounds = half_angle_bounds

    def fitness(self, x):
        half_angle = float(x[0])
        # only the final state is needed, so a 2-point grid keeps each
        # evaluation cheap enough for an evolutionary population to afford
        elements, result = deorbit_with_arc_limited_thrust(
            self.chief_state0, half_angle, self.delta_v_reference, self.m_dry, self.isp,
            self.thrust, self.mu, direction=self.direction, steepness=self.steepness, n_pts=2,
        )
        perigee_error = abs(elements.a * (1.0 - elements.e) - self.target_perigee_radius)
        wall_time = result.t[-1]

        return [perigee_error, wall_time]

    def get_nobj(self):
        return 2

    def get_bounds(self):
        return ([self.half_angle_bounds[0]], [self.half_angle_bounds[1]])


def pareto_front_arc_half_angle(chief_state0, target_perigee_radius, delta_v_reference, m_dry,
                                 isp, thrust, mu, direction=(1.0, 0.0, 0.0), steepness=100.0,
                                 generations=30, pop_size=24, seed=None):
    """
    Evolve a Pareto front of thrust-arc half-angles trading perigee
    accuracy against burn duration, with pygmo's NSGA-II multi-objective
    algorithm.

    Parameters
    ----------
    See ArcHalfAnglePygmoProblem for the shared parameters.
    generations : int
        Number of NSGA-II generations to evolve.
    pop_size : int
        Population size.
    seed : int, optional
        Random seed, for reproducibility.

    Returns
    -------
    half_angles : numpy.ndarray, shape (pop_size,)
        Half-angles [rad] of the final, non-dominated-sorted population.
    perigee_errors : numpy.ndarray, shape (pop_size,)
        |final perigee - target| [km] for each half-angle.
    wall_times : numpy.ndarray, shape (pop_size,)
        Wall-clock burn duration [s] for each half-angle.

    Author: Giovanni Facchinetti, 2026
    """

    udp = ArcHalfAnglePygmoProblem(
        chief_state0, target_perigee_radius, delta_v_reference, m_dry, isp, thrust, mu,
        direction=direction, steepness=steepness,
    )
    problem = pg.problem(udp)
    algorithm = pg.algorithm(pg.nsga2(gen=generations))
    population = pg.population(problem, size=pop_size, seed=seed)
    population = algorithm.evolve(population)

    half_angles = population.get_x()[:, 0]
    perigee_errors = population.get_f()[:, 0]
    wall_times = population.get_f()[:, 1]

    return half_angles, perigee_errors, wall_times
