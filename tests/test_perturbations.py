import numpy as np

from ios_mission_planner.constants import MU_EARTH, R_EARTH, J2_EARTH
from ios_mission_planner.dynamics.orbital.elements import (
    ClassicalElements,
    cartesian_state_from_elements,
    classical_elements_from_state,
)
from ios_mission_planner.dynamics.orbital.perturbations import (
    j2_acceleration_symbolic,
    j2_secular_rates,
    sun_synchronous_inclination,
    two_body_j2_dynamics_symbolic,
)
from ios_mission_planner.propagation.heyoka_propagator import propagate
import heyoka as hy


def j2_acceleration_numeric(pos, mu=MU_EARTH, j2=J2_EARTH, r_eq=R_EARTH):
    x, y, z = hy.make_vars("x", "y", "z")
    ax, ay, az = j2_acceleration_symbolic(x, y, z, mu, j2, r_eq)
    cf = hy.cfunc([ax, ay, az], vars=[x, y, z])
    return np.asarray(cf(np.asarray(pos, dtype=float)))


def j2_potential(pos, mu=MU_EARTH, j2=J2_EARTH, r_eq=R_EARTH):
    r = np.linalg.norm(pos)
    z = pos[2]
    return mu / r * j2 * (r_eq / r) ** 2 * 0.5 * (3 * (z / r) ** 2 - 1)


def test_j2_acceleration_matches_finite_difference_of_potential():
    pos = np.array([R_EARTH + 500.0, 1200.0, 3000.0])
    accel = j2_acceleration_numeric(pos)

    eps = 1e-3
    grad = np.zeros(3)
    for i in range(3):
        bump = np.zeros(3)
        bump[i] = eps
        grad[i] = (j2_potential(pos + bump) - j2_potential(pos - bump)) / (2 * eps)

    # a = -grad(U) for a potential defined so that U adds onto -mu/r
    assert np.allclose(accel, -grad, rtol=1e-4, atol=1e-12)


def test_j2_perturbation_breaks_pure_keplerian_periodicity():
    a = R_EARTH + 600.0
    incl = np.deg2rad(51.6)
    v = np.sqrt(MU_EARTH / a)
    state0 = np.array([
        a, 0.0, 0.0,
        0.0, v * np.cos(incl), v * np.sin(incl),
    ])
    period = 2 * np.pi * np.sqrt(a**3 / MU_EARTH)

    states = propagate(
        two_body_j2_dynamics_symbolic(), state0, [0.0, period],
        pars=[MU_EARTH, J2_EARTH, R_EARTH],
    ).y

    # after exactly one unperturbed period, J2 has nudged the state measurably
    assert np.linalg.norm(states[:, -1] - state0) > 1.0
    # but energy (dominated by the -mu/r term) is still close to conserved
    # over a single orbit, since J2 is a small perturbation
    r0, v0 = np.linalg.norm(state0[:3]), np.linalg.norm(state0[3:])
    r1, v1 = np.linalg.norm(states[:3, -1]), np.linalg.norm(states[3:, -1])
    e0 = 0.5 * v0**2 - MU_EARTH / r0
    e1 = 0.5 * v1**2 - MU_EARTH / r1
    assert abs(e1 - e0) / abs(e0) < 1e-3


def test_j2_secular_rates_match_numerical_drift_of_node_and_perigee():
    a, e, incl = R_EARTH + 700.0, 0.02, np.deg2rad(51.6)
    chief = ClassicalElements(a=a, e=e, i=incl, raan=0.3, argp=0.5, mean_anomaly=0.0)
    state0 = cartesian_state_from_elements(chief, MU_EARTH)
    period = 2 * np.pi * np.sqrt(a**3 / MU_EARTH)

    # sample once per orbit, at the same point of the orbit, so that the
    # short-period J2 oscillations cancel out of the difference
    n_orbits = 30
    t_eval = np.arange(n_orbits + 1) * period
    sol = propagate(
        two_body_j2_dynamics_symbolic(), state0, t_eval,
        pars=[MU_EARTH, J2_EARTH, R_EARTH],
    ).y

    first = classical_elements_from_state(sol[:, 0], MU_EARTH)
    last = classical_elements_from_state(sol[:, -1], MU_EARTH)
    elapsed = t_eval[-1]

    raan_dot, argp_dot, _ = j2_secular_rates(a, e, incl, MU_EARTH, J2_EARTH, R_EARTH)

    measured_raan_dot = (last.raan - first.raan) / elapsed
    measured_argp_dot = (last.argp - first.argp) / elapsed

    assert raan_dot < 0.0  # prograde orbit: the node regresses
    assert np.isclose(measured_raan_dot, raan_dot, rtol=0.02)
    assert np.isclose(measured_argp_dot, argp_dot, rtol=0.10)


def test_j2_perigee_rate_vanishes_at_the_critical_inclination():
    critical = np.arccos(np.sqrt(1.0 / 5.0))
    _, argp_dot, _ = j2_secular_rates(R_EARTH + 700.0, 0.1, critical, MU_EARTH, J2_EARTH, R_EARTH)

    assert abs(argp_dot) < 1e-18


def test_sun_synchronous_inclination_matches_the_known_leo_value():
    a = R_EARTH + 800.0
    incl = sun_synchronous_inclination(a, 0.0, MU_EARTH, J2_EARTH, R_EARTH)
    raan_dot, _, _ = j2_secular_rates(a, 0.0, incl, MU_EARTH, J2_EARTH, R_EARTH)

    assert 98.0 < np.rad2deg(incl) < 99.0  # the textbook value at 800 km is about 98.6 deg
    assert np.isclose(raan_dot * 365.2421897 * 86400.0, 2 * np.pi, rtol=1e-9)
