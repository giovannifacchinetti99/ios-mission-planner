import numpy as np

from ios_mission_planner.constants import MU_EARTH, R_EARTH, J2_EARTH
from ios_mission_planner.dynamics.orbital.perturbations import (
    j2_acceleration_symbolic,
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
