from dataclasses import fields, replace

import numpy as np
import pytest
from conftest import make_near_hold_config

from ios_mission_planner.gnc.mission_manager import Phase
from ios_mission_planner.sim.monte_carlo import Dispersions, run_monte_carlo
from ios_mission_planner.sim.simulator import ABORTED, COLLISION, CONTACT_OUT_OF_LIMITS, DOCKED, simulate
from ios_mission_planner.sim.supervisor import SupervisorDecision, SupervisorSample

AXIS = np.array([0.0, -1.0, 0.0])        # outward port axis of the reference chief


def _same_run(first, second):
    arrays = [f.name for f in fields(first) if isinstance(getattr(first, f.name), np.ndarray)]
    return (all(np.array_equal(getattr(first, name), getattr(second, name)) for name in arrays)
            and first.outcome == second.outcome and first.events == second.events)


def _abort_at(time):
    """A supervisor that asks for an abort from the given time on."""

    def supervisor(sample):
        if sample.t >= time:
            return SupervisorDecision(request_abort=True, note="test")
        return None

    return supervisor


def _abort_factory(seed):
    return _abort_at(100.0)


def test_a_silent_supervisor_changes_nothing_and_sees_only_the_truth(near_hold_config):
    samples = []

    def watcher(sample):
        samples.append(sample)
        return None

    plain = simulate(near_hold_config)
    watched = simulate(near_hold_config, supervisor=watcher)

    assert _same_run(plain, watched)
    # one sample per step before the end, holding the truth and the command only
    assert len(samples) == len(plain.t) - 1
    assert {f.name for f in fields(SupervisorSample)} == {"t", "relative_state", "body_to_lvlh", "force_command"}
    k = 40
    assert samples[k].t == plain.t[k]
    assert np.array_equal(samples[k].relative_state, plain.relative_state[k])
    assert np.array_equal(samples[k].body_to_lvlh, plain.body_to_lvlh[k])
    with pytest.raises(ValueError):
        samples[k].relative_state[0] = 0.0


def test_a_supervisor_that_zeroes_the_force_stops_the_thrusters(near_hold_config):
    def zero_after_a_minute(sample):
        if sample.t >= 60.0:
            return SupervisorDecision(force_command=np.zeros(3), note="inhibit")
        return None

    config = replace(near_hold_config, max_duration=400.0)
    result = simulate(config, supervisor=zero_after_a_minute)

    # the accumulated impulse of the last commanded pulse may still fire once
    assert np.all(result.thrust[result.t >= 62.0] == 0.0)
    assert result.outcome != DOCKED
    notes = [note for _, _, note in result.events]
    assert "supervisor: force override (inhibit)" in notes


def test_a_supervisor_abort_request_makes_the_main_chain_abort(near_hold_config):
    config = replace(near_hold_config, max_duration=1500.0)
    result = simulate(config, supervisor=_abort_at(100.0))

    assert result.outcome == ABORTED
    assert result.abort_reason == "supervisor request"
    abort_events = [(t, phase, note) for t, phase, note in result.events if phase == "abort"]
    assert abort_events[0][0] == 100.0 and abort_events[0][2] == "supervisor request"
    assert (100.0, "abort", "supervisor: abort request (test)") in result.events
    # the maneuver of the main chain moves the deputy away from the port
    assert result.port_axial[-1] > result.port_axial[result.t == 100.0][0] + 50.0


def test_a_velocity_bias_hides_a_closing_speed_the_main_abort_should_catch(near_hold_config):
    config = replace(near_hold_config, max_duration=800.0)
    config.failures.navigation_velocity_bias_start = 90.0
    config.failures.navigation_velocity_bias_duration = 1e6
    config.failures.navigation_velocity_bias = 0.6 * AXIS     # believes it closes 0.6 m/s slower
    result = simulate(config)

    guidance = config.guidance
    closing = -(result.relative_state[:, 3:] @ AXIS)
    contact = result.hold_positions[2] - guidance.hold_distances[2] * AXIS
    distance = np.maximum((result.relative_state[:, :3] - contact) @ AXIS, 0.0)
    limit = guidance.abort_closing_speed_factor * (guidance.closing_speed_floor
                                                   + guidance.closing_speed_slope * distance)
    monitored = np.isin(result.phase, [Phase.APPROACH, Phase.HOLD_NEAR, Phase.FINAL_APPROACH])

    # the true closing speed exceeds the abort limit, yet no abort is called
    assert np.any((closing > limit + 0.05) & monitored)
    assert result.abort_reason is None
    assert result.outcome in (CONTACT_OUT_OF_LIMITS, COLLISION)
    # the logged estimate is the biased one the software believed
    later = result.t > 120.0
    error = result.relative_estimate[later, 3:] - result.relative_state[later, 3:]
    assert np.allclose(np.median(error, axis=0), 0.6 * AXIS, atol=0.01)


def _from_s2(seed=3):
    config = make_near_hold_config(seed=seed)
    config.initial.start_hold = 1
    config.max_duration = 4000.0
    return config


def _overshoot_past_s3(result):
    before_final = result.phase < Phase.FINAL_APPROACH
    past = -((result.relative_state[:, :3] - result.hold_positions[2]) @ AXIS)
    return past[before_final].max()


def test_late_braking_carries_the_deputy_past_the_hold_point():
    nominal = simulate(_from_s2())
    config = _from_s2()
    config.failures.late_braking_start = 0.0
    config.failures.late_braking_duration = 60.0
    late = simulate(config)

    assert _overshoot_past_s3(nominal) < 0.5
    assert _overshoot_past_s3(late) > 1.5

    config.failures.late_braking_hold = 0
    with pytest.raises(ValueError):
        simulate(config)


def test_an_attitude_bias_that_grows_slowly_goes_unnoticed(near_hold_config):
    config = replace(near_hold_config, max_duration=800.0)
    config.failures.attitude_bias_start = 0.0
    config.failures.attitude_bias_duration = 1e6
    config.failures.attitude_bias = np.array([0.0, 0.0, 0.3])
    config.failures.attitude_bias_ramp = 120.0
    result = simulate(config)

    later = result.t > 150.0
    # the controller steers the true attitude away from the command by the
    # bias, while the attitude estimate stays right and nobody raises an alarm
    assert abs(np.degrees(np.median(result.attitude_error[later])) - 17.2) < 1.5
    assert np.degrees(result.attitude_knowledge_error.max()) < 0.1
    assert result.abort_reason is None
    assert result.outcome != DOCKED

    # the same bias appearing at once is a jump in the estimated pointing error
    config.failures.attitude_bias_ramp = 0.0
    assert simulate(config).abort_reason == "attitude error too large"


def test_monte_carlo_builds_one_supervisor_per_run(near_hold_config):
    base = replace(near_hold_config, max_duration=1200.0)
    # dispersions sized for a start at the last hold point, not at 5 km
    small = Dispersions(position_offset=0.05, velocity_offset=0.001, navigation_position_error=0.03,
                        navigation_velocity_error=0.001)
    campaign = run_monte_carlo(base, small, n_runs=2, supervisor_factory=_abort_factory)

    assert campaign.outcomes == [ABORTED, ABORTED]
    assert campaign.abort_reasons == ["supervisor request"] * 2
    assert all(events and events[0][2].startswith("supervisor") for events in campaign.supervisor_events)

    plain = run_monte_carlo(base, small, n_runs=1)
    assert plain.supervisor_events == [[]]
