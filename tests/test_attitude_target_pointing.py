import numpy as np

from ios_mission_planner.dynamics.attitude.target_pointing import (
    boresight_pointing_error_deg,
    fly_with_periodic_retargeting,
    target_quaternion_toward_chief,
)
from ios_mission_planner.dynamics.relative.cw import cw_targeting
from ios_mission_planner.dynamics.relative.relative_orbit import RelativeOrbit, state_from_relative_orbit

N = 0.0011


def test_target_quaternion_achieves_exactly_zero_pointing_error():
    rho = np.array([50.0, -200.0, 30.0])
    q_target = target_quaternion_toward_chief(rho)

    assert boresight_pointing_error_deg(q_target, rho) < 1e-9


def test_boresight_error_is_ninety_degrees_when_perpendicular():
    identity_q = np.array([1.0, 0.0, 0.0, 0.0])
    # body +x axis stays along the frame's x-axis for the identity attitude;
    # pointing straight "ahead" along +y means the chief is 90 degrees off
    rho = np.array([0.0, -100.0, 0.0])

    assert np.isclose(boresight_pointing_error_deg(identity_q, rho, body_axis=(1.0, 0.0, 0.0)), 90.0)


def _hop_scenario():
    period = 2 * np.pi / N
    r0, v0 = np.array([0.0, -5000.0, 0.0]), np.zeros(3)
    orbit_far = RelativeOrbit(x_off=0.0, y_c=0.0, rho=200.0, alpha=np.pi / 2)
    state_far = state_from_relative_orbit(orbit_far, N)
    hop_duration = 0.75 * period
    dv1, _ = cw_targeting(r0, v0, state_far[:3], state_far[3:], hop_duration, N)

    relative_state0 = np.concatenate([r0, v0 + dv1])
    q0 = target_quaternion_toward_chief(relative_state0[:3])
    attitude_state0 = np.concatenate([q0, [0.0, 0.0, 0.0]])

    return relative_state0, attitude_state0, hop_duration


def test_fly_with_periodic_retargeting_returns_consistent_shapes():
    relative_state0, attitude_state0, hop_duration = _hop_scenario()

    t, rel, att, err = fly_with_periodic_retargeting(
        relative_state0, attitude_state0, N, inertia=(10.0, 15.0, 20.0), kp=2.0, kd=8.0,
        total_duration=hop_duration, retarget_interval=60.0, pts_per_interval=20,
    )

    assert len(t) == len(err) == rel.shape[0] == att.shape[0]
    assert rel.shape[1] == 6
    assert att.shape[1] == 7
    assert np.isclose(t[-1], hop_duration, atol=1.0)


def test_pointing_error_shrinks_with_a_shorter_retarget_interval():
    relative_state0, attitude_state0, hop_duration = _hop_scenario()

    def max_error(interval):
        _, _, _, err = fly_with_periodic_retargeting(
            relative_state0, attitude_state0, N, inertia=(10.0, 15.0, 20.0), kp=2.0, kd=8.0,
            total_duration=hop_duration, retarget_interval=interval, pts_per_interval=20,
        )
        return err.max()

    assert max_error(15.0) < max_error(60.0) < max_error(120.0)
