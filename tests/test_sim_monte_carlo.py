import numpy as np
from conftest import make_near_hold_config

from ios_mission_planner.sim.monte_carlo import (
    CONTACT_QUANTITIES,
    Dispersions,
    run_monte_carlo,
    sample_config,
    wilson_interval,
)

NEAR_DISPERSIONS = Dispersions(
    position_offset=0.05, velocity_offset=0.002, attitude_error=np.deg2rad(0.2), body_rate=0.0,
    navigation_position_error=0.03, navigation_velocity_error=0.001,
)


def test_wilson_interval_known_values():
    # 50 successes out of 100 at 95 percent: the textbook value
    lower, upper = wilson_interval(50, 100)
    assert np.isclose(lower, 0.4038, atol=5e-4) and np.isclose(upper, 0.5962, atol=5e-4)

    # all successes: the upper end is one, the lower end is n / (n + z^2)
    lower, upper = wilson_interval(200, 200)
    assert upper == 1.0
    assert np.isclose(lower, 200.0 / (200.0 + 1.959964**2), atol=1e-6)

    # no trials, no information
    assert wilson_interval(0, 0) == (0.0, 1.0)


def test_wilson_interval_narrows_with_more_trials_and_widens_with_confidence():
    assert np.diff(wilson_interval(90, 100))[0] > np.diff(wilson_interval(900, 1000))[0]
    assert np.diff(wilson_interval(90, 100, 0.99))[0] > np.diff(wilson_interval(90, 100, 0.90))[0]


def test_sample_config_is_reproducible_and_leaves_the_base_untouched():
    base = make_near_hold_config()
    untouched = base.initial.position_offset.copy()

    first = sample_config(base, NEAR_DISPERSIONS, seed=11)
    again = sample_config(base, NEAR_DISPERSIONS, seed=11)
    other = sample_config(base, NEAR_DISPERSIONS, seed=12)

    assert np.array_equal(first.initial.position_offset, again.initial.position_offset)
    assert np.array_equal(first.deputy.thrusters.misalignment, again.deputy.thrusters.misalignment)
    assert not np.array_equal(first.initial.position_offset, other.initial.position_offset)
    assert np.array_equal(base.initial.position_offset, untouched)
    assert first.seed == 11 and first.initial.start_hold == 2


def test_sampled_dispersions_have_the_requested_spread():
    base = make_near_hold_config()
    dispersions = Dispersions()

    scale = np.array([sample_config(base, dispersions, seed).deputy.thrusters.scale_error for seed in range(400)])
    density = np.array([sample_config(base, dispersions, seed).environment.atmosphere_density
                        for seed in range(400)])

    assert np.allclose(scale.std(axis=0), dispersions.thruster_scale_error, rtol=0.12)
    assert abs(scale.mean()) < 0.005
    assert np.isclose(np.log10(density / base.environment.atmosphere_density).std(), 0.3, rtol=0.12)


def test_small_campaign_collects_one_entry_per_run():
    campaign = run_monte_carlo(make_near_hold_config(), NEAR_DISPERSIONS, n_runs=3, first_seed=20)

    assert campaign.n_runs == 3
    assert list(campaign.seeds) == [20, 21, 22]
    assert campaign.contact.shape == (3, len(CONTACT_QUANTITIES))
    assert campaign.success.all() and campaign.success_rate == 1.0
    assert campaign.outcome_counts() == {"docked": 3}

    lower, upper = campaign.success_interval()
    assert 0.0 < lower < 1.0 and upper == 1.0

    statistics = campaign.contact_statistics()
    assert statistics["lateral misalignment [m]"]["max"] < campaign.limits["lateral misalignment [m]"]
    assert len(campaign.trajectories) == 3 and campaign.trajectories[0].shape[1] == 3
    # the runs differ from each other
    assert np.unique(np.round(campaign.delta_v, 6)).size == 3


def test_campaign_reproduces_a_single_run():
    from ios_mission_planner.sim.simulator import simulate

    base = make_near_hold_config()
    campaign = run_monte_carlo(base, NEAR_DISPERSIONS, n_runs=1, first_seed=31)
    single = simulate(sample_config(base, NEAR_DISPERSIONS, 31))

    assert np.isclose(campaign.delta_v[0], single.total_delta_v)
    assert np.isclose(campaign.contact[0, 0], single.contact.values["lateral misalignment [m]"])
