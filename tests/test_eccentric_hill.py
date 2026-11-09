import numpy as np

from ios_mission_planner.constants import MU_EARTH, R_EARTH
from ios_mission_planner.dynamics.orbital.elements import ClassicalElements, cartesian_state_from_elements
from ios_mission_planner.dynamics.orbital.two_body import two_body_dynamics_symbolic
from ios_mission_planner.dynamics.relative.eccentric_hill import eccentric_hill_equations_symbolic
from ios_mission_planner.dynamics.relative.hill import hill_equations_symbolic
from ios_mission_planner.dynamics.relative.lvlh import inertial_to_lvlh, lvlh_to_inertial
from ios_mission_planner.dynamics.relative.relative_orbit import RelativeOrbit, state_from_relative_orbit
from ios_mission_planner.propagation.heyoka_propagator import propagate


def test_reduces_to_circular_hill_equations_for_a_circular_chief():
    a = R_EARTH + 500.0
    v = np.sqrt(MU_EARTH / a)
    n = np.sqrt(MU_EARTH / a**3)
    chief0 = np.array([a, 0.0, 0.0, 0.0, v, 0.0])
    rel0 = np.array([50.0, -200.0, 30.0, 0.01, -0.02, 0.005])
    t_eval = np.linspace(0.0, 1.5 * 2 * np.pi / n, 400)

    state0 = np.concatenate([chief0, rel0])
    rel_ecc = propagate(eccentric_hill_equations_symbolic(), state0, t_eval, pars=[MU_EARTH]).y[6:, :]
    rel_circ = propagate(hill_equations_symbolic(), rel0, t_eval, pars=[n]).y

    assert np.allclose(rel_ecc, rel_circ, atol=1e-8)


def test_matches_nonlinear_truth_for_a_small_bounded_relative_orbit():
    chief_el = ClassicalElements(
        a=R_EARTH + 500.0, e=0.1, i=np.deg2rad(30.0), raan=0.2, argp=0.5, mean_anomaly=0.0,
    )
    chief0 = cartesian_state_from_elements(chief_el, MU_EARTH)
    n0 = np.sqrt(MU_EARTH / chief_el.a**3)
    period = 2 * np.pi / n0

    orbit = RelativeOrbit(x_off=0.0, y_c=0.0, rho=0.1, alpha=0.0, rho_z=0.05, beta=0.3)
    rel0 = state_from_relative_orbit(orbit, n0)
    deputy0 = lvlh_to_inertial(chief0, rel0)

    t_eval = np.linspace(0.0, 2 * period, 500)

    sol_c = propagate(two_body_dynamics_symbolic(), chief0, t_eval, pars=[MU_EARTH]).y
    sol_d = propagate(two_body_dynamics_symbolic(), deputy0, t_eval, pars=[MU_EARTH]).y
    rel_truth = np.array([inertial_to_lvlh(sol_c[:, i], sol_d[:, i]) for i in range(len(t_eval))]).T

    state0 = np.concatenate([chief0, rel0])
    rel_lin = propagate(eccentric_hill_equations_symbolic(), state0, t_eval, pars=[MU_EARTH]).y[6:, :]

    pos_error = np.linalg.norm(rel_lin[:3, :] - rel_truth[:3, :], axis=0)
    separation = np.linalg.norm(rel0[:3])

    assert pos_error.max() / separation < 0.01  # well under 1% over two orbits


def test_chief_state_matches_plain_two_body_propagation():
    a = R_EARTH + 700.0
    v = np.sqrt(MU_EARTH / a) * 1.02  # slightly eccentric
    chief0 = np.array([a, 0.0, 0.0, 0.0, v, 0.3])
    rel0 = np.zeros(6)
    t_eval = np.linspace(0.0, 3600.0, 50)

    state0 = np.concatenate([chief0, rel0])
    chief_out = propagate(eccentric_hill_equations_symbolic(), state0, t_eval, pars=[MU_EARTH]).y[:6, :]
    chief_truth = propagate(two_body_dynamics_symbolic(), chief0, t_eval, pars=[MU_EARTH]).y

    assert np.allclose(chief_out, chief_truth, rtol=1e-8)
