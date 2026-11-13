import numpy as np

from ios_mission_planner.dynamics.attitude.determination import quaternion_to_rotation_matrix
from ios_mission_planner.dynamics.relative.cw import cw_discrete_matrices, cw_state_transition_matrix
from ios_mission_planner.gnc.guidance import GlideslopeReference, command_rate, plan_transfer, pointing_quaternion

N = 0.0011
PERIOD = 2.0 * np.pi / N


def _fly_finite_burn(state, dv, burn_duration, total_time, acceleration):
    """Linear truth with the first burn delivered at a finite acceleration."""

    phi, gamma = cw_discrete_matrices(1.0, N)
    direction = dv / np.linalg.norm(dv)
    state = np.array(state, dtype=float)
    remaining = np.linalg.norm(dv)
    for _ in range(int(round(total_time))):
        thrust = min(acceleration, remaining)
        remaining -= thrust
        state = phi @ state + gamma @ (thrust * direction)
    return state


def test_plan_transfer_is_exact_for_an_impulsive_burn():
    state = np.array([5.0, -5000.0, -3.0, 0.01, -0.02, 0.005])
    target = np.array([0.0, -500.0, 0.0])
    tof = 0.45 * PERIOD

    dv, duration = plan_transfer(state, target, tof, N)
    after = state.copy()
    after[3:] += dv
    arrival = cw_state_transition_matrix(tof, N) @ after

    assert duration == 0.0
    assert np.allclose(arrival[:3], target, atol=1e-6)


def test_plan_transfer_compensates_for_the_duration_of_the_burn():
    state = np.array([0.0, -5000.0, 0.0, 0.0, 0.0, 0.0])
    target = np.array([0.0, -500.0, 0.0])
    tof = float(round(0.45 * PERIOD))
    acceleration = 0.0125

    dv_impulsive, _ = plan_transfer(state, target, tof, N)
    dv_centered, duration = plan_transfer(state, target, tof, N, acceleration)

    miss_impulsive = np.linalg.norm(_fly_finite_burn(state, dv_impulsive, 0.0, tof, acceleration)[:3] - target)
    miss_centered = np.linalg.norm(_fly_finite_burn(state, dv_centered, duration, tof, acceleration)[:3] - target)

    assert duration > 60.0                      # the burn is far from instantaneous
    assert miss_impulsive > 30.0                # ignoring that misses by tens of metres
    assert miss_centered < 0.1 * miss_impulsive


def test_glideslope_speed_follows_the_law_and_respects_the_limits():
    carrot = GlideslopeReference([0.0, -500.0, 0.0], [0.0, -20.0, 0.0], v_max=0.4, v_min=0.05,
                                 time_constant=400.0, max_acceleration=0.002)

    previous_speed, steps = 0.0, 0
    speeds, distances = [], []
    while not carrot.arrived and steps < 10000:
        carrot.advance(1.0)
        assert carrot.speed <= 0.4 + 1e-12
        if not carrot.arrived:
            assert abs(carrot.speed - previous_speed) <= 0.002 + 1e-12
        previous_speed = carrot.speed
        speeds.append(carrot.speed)
        distances.append(carrot.distance_to_go)
        steps += 1

    speeds, distances = np.array(speeds), np.array(distances)
    assert carrot.arrived and np.allclose(carrot.position, [0.0, -20.0, 0.0])
    assert np.all(np.diff(distances) <= 0.0)

    # far away the cruise speed, in between the proportional law
    assert np.isclose(speeds[np.argmin(np.abs(distances - 300.0))], 0.4)
    k = np.argmin(np.abs(distances - 80.0))
    assert np.isclose(speeds[k], distances[k] / 400.0, rtol=0.05)
    # and it has almost come to rest when it reaches the end
    assert speeds[-2] < 0.02 and speeds[-1] == 0.0


def test_glideslope_without_braking_keeps_its_speed_to_the_end():
    carrot = GlideslopeReference([0.0, -21.6, 0.0], [0.0, -0.6, 0.0], v_max=0.075, v_min=0.075,
                                 time_constant=1.0, max_acceleration=0.002, brake=False)

    for _ in range(200):
        carrot.advance(1.0)

    assert np.isclose(carrot.speed, 0.075)
    assert np.allclose(carrot.velocity, [0.0, 0.075, 0.0])


def test_glideslope_horizon_is_consistent_with_advancing():
    carrot = GlideslopeReference([0.0, -500.0, 0.0], [0.0, -20.0, 0.0], 0.4, 0.05, 400.0, 0.002)
    for _ in range(50):
        carrot.advance(1.0)

    reference, feedforward = carrot.horizon(4.0, 10, N)
    assert reference.shape == (11, 6) and feedforward.shape == (10, 3)
    assert np.allclose(reference[0, :3], carrot.position)

    # the horizon does not move the carrot, and predicts where it will be
    before = carrot.distance_to_go
    assert carrot.distance_to_go == before
    for _ in range(10):
        carrot.advance(4.0)
    assert np.allclose(reference[-1, :3], carrot.position, atol=1e-9)

    # along the V-bar the feedforward is the radial Coriolis compensation
    assert np.allclose(feedforward[5, 0], -2.0 * N * reference[5, 4])


def test_pointing_quaternion_points_the_two_axes():
    rng = np.random.default_rng(0)
    lvlh, _ = np.linalg.qr(rng.standard_normal((3, 3)))
    if np.linalg.det(lvlh) < 0:
        lvlh[:, 0] *= -1.0
    lvlh_to_inertial = lvlh.T

    direction = np.array([0.1, 1.0, -0.05])
    direction /= np.linalg.norm(direction)
    quaternion = pointing_quaternion(lvlh_to_inertial, direction, [0.0, 0.0, 1.0])
    body_to_lvlh = lvlh @ quaternion_to_rotation_matrix(quaternion).T

    # the boresight is exactly along the direction; the second axis is as
    # close to the orbit normal as that allows
    assert np.allclose(body_to_lvlh @ [1.0, 0.0, 0.0], direction, atol=1e-12)
    roll_axis = body_to_lvlh @ [0.0, 0.0, 1.0]
    assert roll_axis[2] > 0.99
    assert abs(roll_axis @ direction) < 1e-12


def test_pointing_quaternion_sign_follows_the_previous_command():
    lvlh_to_inertial = np.eye(3)
    q = pointing_quaternion(lvlh_to_inertial, [0.0, 1.0, 0.0], [0.0, 0.0, 1.0])

    same = pointing_quaternion(lvlh_to_inertial, [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], previous=-q)
    assert np.allclose(same, -q)


def test_command_rate_of_a_port_aligned_attitude_is_the_orbital_rate():
    # LVLH frame of a circular equatorial orbit at two instants dt apart
    dt = 1.0

    def lvlh_to_inertial(t):
        c, s = np.cos(N * t), np.sin(N * t)
        return np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]]).T

    first = pointing_quaternion(lvlh_to_inertial(0.0), [0.0, 1.0, 0.0], [0.0, 0.0, 1.0])
    second = pointing_quaternion(lvlh_to_inertial(dt), [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], previous=first)

    # the body z axis is the orbit normal: the command turns about it at n
    assert np.allclose(command_rate(first, second, dt), [0.0, 0.0, N], atol=1e-12)
