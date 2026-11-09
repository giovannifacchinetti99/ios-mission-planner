import numpy as np

from ios_mission_planner.dynamics.relative.relative_orbit import (
    drift_free_orbit_through,
    relative_orbit_from_state,
    state_from_relative_orbit,
)

N = 0.0011


def test_round_trip_state_orbit():
    state0 = np.array([50.0, -200.0, 30.0, 0.01, -0.02, 0.005])

    orbit = relative_orbit_from_state(state0, N)
    recovered = state_from_relative_orbit(orbit, N)

    assert np.allclose(recovered, state0, atol=1e-9)


def test_drift_free_orbit_has_zero_offset_and_drift():
    orbit = state_from_relative_orbit(
        relative_orbit_from_state(np.array([80.0, 10.0, 0.0, 0.0, -2 * N * 80.0, 0.0]), N), N
    )
    decomposed = relative_orbit_from_state(orbit, N)

    assert np.isclose(decomposed.x_off, 0.0, atol=1e-9)
    assert np.isclose(decomposed.drift_rate(N), 0.0, atol=1e-12)


def test_drift_free_orbit_through_point_passes_through_it():
    x, y = 60.0, -15.0
    orbit = drift_free_orbit_through(x, y, y_c=0.0)
    state = state_from_relative_orbit(orbit, N)

    assert np.isclose(state[0], x, atol=1e-9)
    assert np.isclose(state[1], y, atol=1e-9)
    assert np.isclose(orbit.x_off, 0.0)
