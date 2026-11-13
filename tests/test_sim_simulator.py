import numpy as np
from conftest import make_near_hold_config

from ios_mission_planner.gnc.mission_manager import Phase
from ios_mission_planner.sim.config import SimulationConfig
from ios_mission_planner.sim.simulator import (
    ABORTED,
    CONTACT_OUT_OF_LIMITS,
    DOCKED,
    TIMEOUT,
    build_truth,
    simulate,
)


def test_final_approach_docks_inside_the_contact_limits(near_hold_config):
    result = simulate(near_hold_config)

    assert result.outcome == DOCKED and result.success
    assert result.contact.success
    assert result.contact.values["lateral misalignment [m]"] < 0.02
    assert 0.065 < result.contact.values["axial speed [m/s]"] < 0.085

    # the phases were flown in order, and the last sample is the contact
    assert list(dict.fromkeys(result.phase)) == [Phase.HOLD_NEAR, Phase.FINAL_APPROACH, Phase.DOCKED]
    assert abs(result.port_axial[-1]) < 0.1
    assert result.total_delta_v > 0.0 and result.propellant_used > 0.0


def test_histories_are_consistent(near_hold_config):
    result = simulate(near_hold_config)
    n = len(result.t)

    assert result.relative_state.shape == (n, 6)
    assert result.body_to_lvlh.shape == (n, 3, 3)
    assert np.allclose(np.linalg.norm(result.quaternion, axis=1), 1.0)
    assert np.all(np.diff(result.delta_v) >= 0.0)
    assert np.all(np.diff(result.mass) <= 0.0)
    assert np.allclose(np.linalg.norm(result.sun_lvlh, axis=1), 1.0)

    spent = result.delta_v_by_phase()
    assert np.isclose(sum(spent.values()), result.total_delta_v)
    assert set(result.phase_intervals()) == {"hold at S3", "final approach", "docked"}


def test_navigation_error_stays_inside_its_own_uncertainty(near_hold_config):
    result = simulate(near_hold_config)

    error = np.abs(result.relative_state - result.relative_estimate)
    inside = error <= 3.0 * result.relative_std

    assert inside.mean() > 0.97
    assert np.linalg.norm(error[-1, :3]) < 0.01
    assert np.degrees(result.attitude_knowledge_error[30:]).max() < 0.02


def test_same_seed_gives_the_same_run_and_another_seed_a_different_one():
    first = simulate(make_near_hold_config(seed=5))
    second = simulate(make_near_hold_config(seed=5))
    other = simulate(make_near_hold_config(seed=6))

    assert np.array_equal(first.relative_state, second.relative_state)
    assert not np.array_equal(first.relative_estimate[:50], other.relative_estimate[:50])


def test_a_truth_model_can_be_reused_across_runs(near_hold_config):
    truth = build_truth(near_hold_config)

    first = simulate(near_hold_config, truth=truth)
    second = simulate(near_hold_config, truth=truth)

    assert np.array_equal(first.relative_state, second.relative_state)


def test_loss_of_the_relative_sensor_triggers_the_avoidance_maneuver(near_hold_config):
    near_hold_config.failures.sensor_outage_start = 120.0
    near_hold_config.failures.sensor_outage_duration = 1e5
    near_hold_config.guidance.abort_coast_duration = 600.0

    result = simulate(near_hold_config)

    assert result.outcome == ABORTED
    assert result.abort_reason == "relative measurements lost"
    assert result.contact is None

    # the deputy never gets closer than it was, and ends far away, behind and below
    abort_index = np.argmax(result.phase == Phase.ABORT)
    distance = np.linalg.norm(result.relative_state[:, :3], axis=1)
    assert distance[abort_index:].min() > distance[abort_index] - 0.5
    assert distance[-1] > 100.0
    assert result.relative_state[-1, 0] < 0.0 and result.relative_state[-1, 1] < -20.0


def test_without_the_abort_the_same_failure_is_not_caught(near_hold_config):
    near_hold_config.failures.sensor_outage_start = 120.0
    near_hold_config.failures.sensor_outage_duration = 1e5
    near_hold_config.guidance.abort_enabled = False

    result = simulate(near_hold_config)

    # flying blind for the last minutes: the deputy still reaches the port,
    # on the prediction of its filter alone
    assert result.outcome in (DOCKED, CONTACT_OUT_OF_LIMITS)
    assert not result.measurement_valid[200:].any()


def test_a_stuck_thruster_is_absorbed_by_the_controller(near_hold_config):
    near_hold_config.failures.stuck_thruster_start = 150.0
    near_hold_config.failures.stuck_thruster_duration = 20.0
    near_hold_config.failures.stuck_thruster_force = np.array([1.0, 0.0, 0.0])

    result = simulate(near_hold_config)
    nominal = simulate(make_near_hold_config())

    assert result.outcome == DOCKED
    assert result.total_delta_v > nominal.total_delta_v + 0.1


def test_wheels_saturate_without_momentum_management_and_not_with_it(near_hold_config):
    # thrust lines 3 cm off the center of mass, far more than a real
    # vehicle, to make the wheels fill up in a short test
    offset = np.array([0.03, 0.03, 0.03])
    limit = near_hold_config.deputy.wheels.max_momentum

    near_hold_config.deputy.thrusters.center_of_mass_offset = offset
    managed = simulate(near_hold_config)

    unmanaged_config = make_near_hold_config()
    unmanaged_config.deputy.thrusters.center_of_mass_offset = offset
    unmanaged_config.deputy.wheels.desaturation_enabled = False
    unmanaged_config.deputy.wheels.max_momentum = 0.05
    unmanaged = simulate(unmanaged_config)

    assert np.abs(managed.wheel_momentum).max() < 0.75 * limit
    assert managed.outcome == DOCKED
    assert np.isclose(np.abs(unmanaged.wheel_momentum).max(), 0.05)


def test_timeout_when_the_duration_is_too_short(near_hold_config):
    near_hold_config.max_duration = 100.0

    result = simulate(near_hold_config)

    assert result.outcome == TIMEOUT
    assert result.duration == 100.0


def test_lighting_gate_delays_the_final_approach_until_the_port_is_lit(near_hold_config):
    ungated = simulate(near_hold_config)

    near_hold_config.guidance.lighting_gate = True
    near_hold_config.max_duration = 8000.0
    gated = simulate(near_hold_config)

    start = lambda result: next(t for t, label, _ in result.events if label == "final approach")
    final = gated.phase == Phase.FINAL_APPROACH

    assert gated.outcome == DOCKED
    assert start(gated) >= start(ungated)
    # the whole final approach is flown out of eclipse with the Sun on the port
    assert not gated.eclipse[final].any()
    assert np.all(gated.sun_lvlh[final] @ near_hold_config.chief.port_axis > np.cos(np.deg2rad(71.0)))


def test_full_mission_from_the_far_hold_point():
    result = simulate(SimulationConfig(seed=1))

    assert result.outcome == DOCKED
    labels = [label for _, label, _ in result.events]
    for expected in ("transfer S1 to S2", "hold at S2", "forced approach S2 to S3", "hold at S3",
                     "final approach", "docked"):
        assert expected in labels

    spent = result.delta_v_by_phase()
    # a straight-line approach along the V-bar over a distance d costs
    # 2 n d against the Coriolis acceleration, plus speeding up and stopping
    n = np.sqrt(398600.4418 / (6378.137 + 500.0) ** 3)
    coriolis = 2.0 * n * 480.0
    assert coriolis < spent["forced approach S2 to S3"] < coriolis + 2 * 0.4 + 0.4
    assert 4.0 < result.total_delta_v < 8.0

    # the corridor and the closing speed limit were respected all the way in
    approach = result.phase >= Phase.APPROACH
    lateral = np.linalg.norm(result.port_lateral[approach], axis=1)
    allowed = 0.3 + np.tan(np.deg2rad(10.0)) * np.maximum(result.port_axial[approach], 0.0)
    assert np.all(lateral <= allowed)
    assert np.all(result.port_closing_speed[approach] <= 0.12 + 0.004 * result.port_axial[approach] + 0.02)
