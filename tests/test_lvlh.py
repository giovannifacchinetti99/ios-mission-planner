import numpy as np

from ios_mission_planner.constants import MU_EARTH, R_EARTH
from ios_mission_planner.dynamics.relative.lvlh import (
    inertial_to_lvlh,
    lvlh_rotation,
    lvlh_to_inertial,
)


def circular_state(a, incl, theta, mu=MU_EARTH):
    p = np.array([1.0, 0.0, 0.0])
    q = np.array([0.0, np.cos(incl), np.sin(incl)])
    speed = np.sqrt(mu / a)
    r = a * (np.cos(theta) * p + np.sin(theta) * q)
    v = speed * (-np.sin(theta) * p + np.cos(theta) * q)
    return np.concatenate([r, v])


def test_rotation_is_orthonormal_and_right_handed():
    chief = circular_state(R_EARTH + 500.0, np.deg2rad(45.0), 1.3)
    rot = lvlh_rotation(chief)

    assert np.allclose(rot @ rot.T, np.eye(3), atol=1e-10)
    assert np.isclose(np.linalg.det(rot), 1.0, atol=1e-10)


def test_round_trip_inertial_lvlh():
    chief = circular_state(R_EARTH + 500.0, np.deg2rad(53.0), 0.4)
    deputy = circular_state(R_EARTH + 500.1, np.deg2rad(53.0), 0.4005)

    relative = inertial_to_lvlh(chief, deputy)
    recovered = lvlh_to_inertial(chief, relative)

    assert np.allclose(recovered, deputy, atol=1e-9)


def test_chief_is_origin_of_its_own_frame():
    chief = circular_state(R_EARTH + 700.0, np.deg2rad(30.0), 2.1)
    relative = inertial_to_lvlh(chief, chief)

    assert np.allclose(relative, np.zeros(6), atol=1e-9)
