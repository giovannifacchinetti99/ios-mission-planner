import numpy as np

from ios_mission_planner.safety.collision_probability import collision_probability
from ios_mission_planner.safety.keepout import KeepOutEllipsoid

N = 0.0011


def test_tight_distribution_already_inside_the_zone_collides_almost_surely():
    zone = KeepOutEllipsoid(a=20.0, b=20.0, c=20.0)
    mean_state = np.array([5.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    covariance = np.eye(6) * 1e-8

    pc = collision_probability(
        mean_state, covariance, N, t_eval=[0.0], zone=zone,
        n_samples=2000, rng=np.random.default_rng(0),
    )

    assert pc > 0.99


def test_tight_distribution_far_outside_never_collides():
    zone = KeepOutEllipsoid(a=20.0, b=20.0, c=20.0)
    mean_state = np.array([500.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    covariance = np.eye(6) * 1e-8

    pc = collision_probability(
        mean_state, covariance, N, t_eval=[0.0, 60.0, 120.0], zone=zone,
        n_samples=2000, rng=np.random.default_rng(0),
    )

    assert pc < 0.01


def test_more_evaluation_times_can_only_raise_the_probability():
    zone = KeepOutEllipsoid(a=20.0, b=20.0, c=20.0)
    mean_state = np.array([60.0, 0.0, 0.0, 0.0, 0.02, 0.0])
    covariance = np.diag([25.0, 25.0, 25.0, 1e-6, 1e-6, 1e-6])
    rng = np.random.default_rng(1)

    pc_one_time = collision_probability(mean_state, covariance, N, [0.0], zone, n_samples=5000, rng=rng)
    rng = np.random.default_rng(1)
    pc_many_times = collision_probability(
        mean_state, covariance, N, np.linspace(0.0, 3000.0, 20), zone, n_samples=5000, rng=rng,
    )

    assert pc_many_times >= pc_one_time
