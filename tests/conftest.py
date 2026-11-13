import numpy as np
import pytest

from ios_mission_planner.sim.config import SimulationConfig


def make_near_hold_config(seed=0):
    """
    A short scenario for the tests of the closed loop: the deputy starts
    at the last hold point, 20 m from the port, with small errors, so a
    run lasts a few hundred steps instead of several thousand.
    """

    config = SimulationConfig(seed=seed)
    config.initial.start_hold = 2
    config.initial.position_offset = np.array([0.05, -0.08, -0.04])
    config.initial.velocity_offset = np.array([0.001, -0.001, 0.001])
    config.initial.attitude_error = np.array([0.002, -0.002, 0.003])
    config.initial.navigation_position_error = np.array([0.03, -0.03, 0.02])
    config.initial.navigation_velocity_error = np.array([0.001, 0.001, -0.001])
    config.navigation.initial_position_std = 0.05
    config.navigation.initial_velocity_std = 0.005
    config.guidance.hold_durations = (300.0, 300.0, 60.0)

    return config


@pytest.fixture
def near_hold_config():
    return make_near_hold_config()
