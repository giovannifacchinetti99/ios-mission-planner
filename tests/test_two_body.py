import numpy as np

from ios_mission_planner.constants import MU_EARTH, R_EARTH
from ios_mission_planner.dynamics.orbital.two_body import two_body_dynamics_symbolic
from ios_mission_planner.propagation.heyoka_propagator import propagate


def circular_state(a, mu=MU_EARTH):
    v = np.sqrt(mu / a)
    return np.array([a, 0.0, 0.0, 0.0, v, 0.0])


def specific_energy(state, mu=MU_EARTH):
    r = np.linalg.norm(state[:3])
    v = np.linalg.norm(state[3:])
    return 0.5 * v**2 - mu / r


def test_energy_and_angular_momentum_conserved():
    a = R_EARTH + 500.0
    state0 = circular_state(a)
    t_eval = np.linspace(0.0, 3 * 2 * np.pi * np.sqrt(a**3 / MU_EARTH), 500)

    states = propagate(two_body_dynamics_symbolic(), state0, t_eval, pars=[MU_EARTH]).y

    energies = np.array([specific_energy(states[:, i]) for i in range(states.shape[1])])
    h0 = np.cross(state0[:3], state0[3:])
    h = np.array([np.cross(states[:3, i], states[3:, i]) for i in range(states.shape[1])])

    assert np.allclose(energies, energies[0], rtol=1e-10)
    assert np.allclose(h, h0, rtol=1e-10)


def test_circular_orbit_period():
    a = R_EARTH + 700.0
    state0 = circular_state(a)
    period = 2 * np.pi * np.sqrt(a**3 / MU_EARTH)

    states = propagate(two_body_dynamics_symbolic(), state0, [0.0, period], pars=[MU_EARTH]).y

    assert np.allclose(states[:, -1], state0, atol=1e-6)
