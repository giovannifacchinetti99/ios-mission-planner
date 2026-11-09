import numpy as np

from ios_mission_planner.constants import MU_EARTH, R_EARTH
from ios_mission_planner.dynamics.orbital.elements import (
    ClassicalElements,
    cartesian_state_from_elements,
    classical_elements_from_state,
)


def assert_elements_close(a, b):
    assert np.isclose(a.a, b.a, rtol=1e-9)
    assert np.isclose(a.e, b.e, atol=1e-9)
    assert np.isclose(a.i, b.i, atol=1e-9)
    assert np.isclose(a.raan % (2 * np.pi), b.raan % (2 * np.pi), atol=1e-9)
    if a.e > 1e-6:
        assert np.isclose(a.argp % (2 * np.pi), b.argp % (2 * np.pi), atol=1e-9)
    assert np.isclose(a.mean_anomaly % (2 * np.pi), b.mean_anomaly % (2 * np.pi), atol=1e-9)


def test_round_trip_eccentric_inclined_orbit():
    elements = ClassicalElements(
        a=R_EARTH + 1200.0, e=0.15, i=np.deg2rad(53.0),
        raan=np.deg2rad(40.0), argp=np.deg2rad(75.0), mean_anomaly=np.deg2rad(200.0),
    )

    state = cartesian_state_from_elements(elements, MU_EARTH)
    recovered = classical_elements_from_state(state, MU_EARTH)

    assert_elements_close(elements, recovered)


def test_round_trip_near_circular_orbit():
    elements = ClassicalElements(
        a=R_EARTH + 500.0, e=0.0, i=np.deg2rad(97.0),
        raan=np.deg2rad(10.0), argp=0.0, mean_anomaly=np.deg2rad(123.0),
    )

    state = cartesian_state_from_elements(elements, MU_EARTH)
    recovered = classical_elements_from_state(state, MU_EARTH)

    assert np.isclose(recovered.e, 0.0, atol=1e-8)
    assert recovered.argp == 0.0
    # for e=0, mean_anomaly is reinterpreted as the argument of latitude
    assert np.isclose(recovered.mean_anomaly, elements.mean_anomaly, atol=1e-6)


def test_energy_and_angular_momentum_match_elements():
    elements = ClassicalElements(
        a=R_EARTH + 800.0, e=0.3, i=np.deg2rad(28.5),
        raan=np.deg2rad(0.0), argp=np.deg2rad(300.0), mean_anomaly=np.deg2rad(50.0),
    )
    state = cartesian_state_from_elements(elements, MU_EARTH)

    r, v = state[:3], state[3:]
    energy = 0.5 * np.dot(v, v) - MU_EARTH / np.linalg.norm(r)
    a_from_energy = -MU_EARTH / (2.0 * energy)
    h = np.linalg.norm(np.cross(r, v))
    p_from_h = h**2 / MU_EARTH

    assert np.isclose(a_from_energy, elements.a, rtol=1e-9)
    assert np.isclose(p_from_h, elements.a * (1 - elements.e**2), rtol=1e-9)
