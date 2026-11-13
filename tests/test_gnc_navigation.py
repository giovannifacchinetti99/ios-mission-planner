import numpy as np
from scipy.linalg import expm

from ios_mission_planner.dynamics.relative.cw import (
    cw_discrete_matrices,
    cw_state_transition_matrix,
    cw_system_matrices,
)
from ios_mission_planner.gnc.navigation import RelativeNavigationFilter, relative_position_from_measurement
from ios_mission_planner.gnc.sensors import RelativeMeasurement, RelativeSensor

N = 0.0011


def test_discrete_matrices_match_the_matrix_exponential():
    a, b = cw_system_matrices(N)
    augmented = np.zeros((9, 9))
    augmented[:6, :6] = a
    augmented[:6, 6:] = b

    for dt in (1.0, 5.0, 300.0):
        exact = expm(augmented * dt)
        phi, gamma = cw_discrete_matrices(dt, N)

        assert np.allclose(phi, exact[:6, :6], atol=1e-12)
        assert np.allclose(gamma, exact[:6, 6:], rtol=1e-8, atol=1e-14)


def test_system_matrix_is_the_derivative_of_the_transition_matrix():
    a, _ = cw_system_matrices(N)
    step = 1e-3
    derivative = (cw_state_transition_matrix(step, N) - cw_state_transition_matrix(-step, N)) / (2 * step)

    assert np.allclose(derivative, a, atol=1e-9)


def test_converted_measurement_recovers_the_relative_position():
    # deputy 30 m behind the chief port target, rotated by 90 degrees about z
    body_to_lvlh = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    sensor_position = np.array([0.3, 0.0, 0.25])
    target = np.array([0.0, -1.0, 0.25])
    relative_position = np.array([0.4, -31.0, -0.2])

    vector = body_to_lvlh.T @ (target - relative_position) - sensor_position
    distance = np.linalg.norm(vector)
    measurement = RelativeMeasurement(range=distance, line_of_sight=vector / distance,
                                      range_std=0.15, angle_std=5e-4)

    position, covariance = relative_position_from_measurement(
        measurement, sensor_position, body_to_lvlh, target,
    )

    assert np.allclose(position, relative_position, atol=1e-12)

    # the uncertainty is the range error along the line of sight and the
    # angular error times the distance across it
    line_of_sight_lvlh = body_to_lvlh @ measurement.line_of_sight
    across = np.cross(line_of_sight_lvlh, [0.0, 0.0, 1.0])
    across /= np.linalg.norm(across)
    assert np.isclose(line_of_sight_lvlh @ covariance @ line_of_sight_lvlh, 0.15**2)
    assert np.isclose(across @ covariance @ across, (distance * 5e-4) ** 2)


def _run_filter(steps, acceleration_true, acceleration_known, seed=0, gate=None, distance=200.0):
    """Linear truth, sensor measurements, filter; returns errors and standard deviations."""

    rng = np.random.default_rng(seed)
    dt = 1.0
    phi, gamma = cw_discrete_matrices(dt, N)
    sensor = RelativeSensor(np.zeros(3), np.array([0.0, 1.0, 0.0]), np.deg2rad(40.0), 1e4, 0.01, 0.005, 5e-4)
    target = np.zeros(3)
    body_to_lvlh = np.eye(3)

    truth = np.array([2.0, -distance, 1.0, 0.0, 0.0, 0.0])
    error0 = np.array([5.0, -4.0, 3.0, 0.02, 0.01, -0.02])
    nav = RelativeNavigationFilter(N, dt, truth + error0, np.diag([10.0**2] * 3 + [0.05**2] * 3),
                                   accel_noise_std=2e-5)

    errors, stds, used = [], [], []
    for k in range(steps):
        measurement = sensor.measure(body_to_lvlh.T @ (target - truth[:3]), rng)
        position, covariance = relative_position_from_measurement(measurement, np.zeros(3), body_to_lvlh, target)
        used.append(nav.update(position, covariance, gate=gate))
        errors.append(nav.state - truth)
        stds.append(np.sqrt(np.diag(nav.covariance)))

        truth = phi @ truth + gamma @ acceleration_true(k)
        nav.predict(acceleration_known(k))

    return np.array(errors), np.array(stds), np.array(used)


def test_filter_converges_and_its_covariance_is_honest():
    zero = lambda k: np.zeros(3)
    errors, stds, _ = _run_filter(600, zero, zero)

    # converged well below the initial error and the single-measurement noise
    assert np.linalg.norm(errors[-1, :3]) < 0.5
    assert np.linalg.norm(errors[-1, 3:]) < 5e-3

    # the errors stay inside three standard deviations of the filter
    # (99.7 percent for a consistent filter), and are not far inside either
    settled = slice(100, None)
    normalized = errors[settled] / stds[settled]
    assert np.mean(np.abs(normalized) < 3.0) > 0.98
    assert 0.3 < normalized.std() < 1.5


def test_filter_accounts_for_a_known_acceleration():
    # speed up toward the target for 30 s, then brake for 30 s
    def burn(k):
        if 100 <= k < 130:
            return np.array([0.0, 0.01, 0.0])
        if 130 <= k < 160:
            return np.array([0.0, -0.01, 0.0])
        return np.zeros(3)

    errors, _, _ = _run_filter(400, burn, burn)

    assert np.linalg.norm(errors[-1, :3]) < 0.5
    assert np.abs(errors[110:200, 3:]).max() < 0.02


def test_filter_recovers_from_an_acceleration_it_does_not_know():
    def unknown(k):
        if 200 <= k < 215:
            return np.array([0.0, 0.01, 0.0])
        if 215 <= k < 230:
            return np.array([0.0, -0.01, 0.0])
        return np.zeros(3)

    zero = lambda k: np.zeros(3)
    errors, _, used = _run_filter(500, unknown, zero, gate=25.0, distance=20.0)

    # the measurements disagree with the estimate and are first rejected...
    assert not used[200:240].all()
    # ...but never more than two in a row, and the estimate comes back
    rejected_runs = "".join("r" if not u else "." for u in used)
    assert "rrr" not in rejected_runs
    assert np.linalg.norm(errors[-1, :3]) < 0.05
    assert np.linalg.norm(errors[-1, 3:]) < 2e-3


def test_process_noise_grows_with_distance_when_the_orbit_radius_is_given():
    covariance = np.diag([1.0] * 3 + [1e-4] * 3)
    near = RelativeNavigationFilter(N, 1.0, [0.0, -100.0, 0.0, 0, 0, 0], covariance, 2e-5, orbit_radius=6.878e6)
    far = RelativeNavigationFilter(N, 1.0, [0.0, -5000.0, 0.0, 0, 0, 0], covariance, 2e-5, orbit_radius=6.878e6)
    plain = RelativeNavigationFilter(N, 1.0, [0.0, -5000.0, 0.0, 0, 0, 0], covariance, 2e-5)

    for nav in (near, far, plain):
        nav.predict(np.zeros(3))

    assert far.covariance[3, 3] > plain.covariance[3, 3]
    assert np.isclose(near.covariance[3, 3], plain.covariance[3, 3], rtol=1e-3)
