import numpy as np

from ios_mission_planner.dynamics.relative.cw import cw_state_transition_matrix, cw_targeting
from ios_mission_planner.dynamics.relative.hill import hill_equations_symbolic
from ios_mission_planner.propagation.heyoka_propagator import propagate

N = 0.0011  # rad/s, a representative LEO mean motion


def test_stm_identity_at_t0():
    phi = cw_state_transition_matrix(0.0, N)
    assert np.allclose(phi, np.eye(6), atol=1e-12)


def test_stm_matches_numerical_hill_propagation():
    state0 = np.array([50.0, -200.0, 30.0, 0.01, -0.02, 0.005])
    t = 1800.0

    phi = cw_state_transition_matrix(t, N)
    analytic = phi @ state0

    numeric = propagate(hill_equations_symbolic(), state0, [0.0, t], pars=[N]).y[:, -1]

    assert np.allclose(analytic, numeric, rtol=1e-6, atol=1e-9)


def test_targeting_reaches_exact_arrival_state():
    r0, v0 = np.array([0.0, -1000.0, 0.0]), np.array([0.0, 0.0, 0.0])
    rf, vf = np.array([0.0, -200.0, 0.0]), np.array([0.0, 0.0, 0.0])
    t = 0.75 * 2 * np.pi / N

    dv1, dv2 = cw_targeting(r0, v0, rf, vf, t, N)

    phi = cw_state_transition_matrix(t, N)
    state0 = np.concatenate([r0, v0 + dv1])
    final = phi @ state0

    assert np.allclose(final[:3], rf, atol=1e-9)
    assert np.allclose(final[3:] + dv2, vf, atol=1e-9)
