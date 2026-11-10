import numpy as np

from ios_mission_planner.dynamics.attitude.determination import (
    quaternion_to_rotation_matrix,
    rotation_matrix_to_quaternion,
    triad_attitude,
    wahba_svd_attitude,
)


def quaternion_from_axis_angle(axis, angle_rad):
    axis = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    return np.concatenate([[np.cos(angle_rad / 2)], axis * np.sin(angle_rad / 2)])


def assert_same_rotation(q_a, q_b, atol=1e-8):
    # q and -q represent the same rotation
    assert min(np.linalg.norm(q_a - q_b), np.linalg.norm(q_a + q_b)) < atol


def test_quaternion_rotation_matrix_round_trip_each_branch():
    cases = [
        ([1.0, 2.0, -1.0], 73.0),      # trace > 0
        ([1.0, 0.05, 0.02], 170.0),    # q1-dominant
        ([0.05, 1.0, 0.02], 170.0),    # q2-dominant
        ([0.05, 0.02, 1.0], 170.0),    # q3-dominant
    ]
    for axis, angle_deg in cases:
        q = quaternion_from_axis_angle(axis, np.deg2rad(angle_deg))
        r = quaternion_to_rotation_matrix(q)
        q_back = rotation_matrix_to_quaternion(r)
        assert_same_rotation(q, q_back)


def test_rotation_matrix_is_orthonormal():
    q = quaternion_from_axis_angle([1.0, 1.0, 1.0], np.deg2rad(40.0))
    r = quaternion_to_rotation_matrix(q)

    assert np.allclose(r @ r.T, np.eye(3), atol=1e-10)
    assert np.isclose(np.linalg.det(r), 1.0, atol=1e-10)


def test_triad_recovers_known_rotation_exactly():
    q = quaternion_from_axis_angle([1.0, 2.0, -1.0], np.deg2rad(73.0))
    r_true = quaternion_to_rotation_matrix(q)

    r1, r2 = np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0])
    b1, b2 = r_true @ r1, r_true @ r2

    r_est = triad_attitude(b1, b2, r1, r2)

    assert np.allclose(r_est, r_true, atol=1e-10)


def test_wahba_svd_recovers_known_rotation_exactly():
    q = quaternion_from_axis_angle([0.3, -0.5, 1.0], np.deg2rad(110.0))
    r_true = quaternion_to_rotation_matrix(q)

    rng = np.random.default_rng(0)
    r_vectors = rng.normal(size=(5, 3))
    r_vectors /= np.linalg.norm(r_vectors, axis=1, keepdims=True)
    b_vectors = (r_true @ r_vectors.T).T

    r_est = wahba_svd_attitude(b_vectors, r_vectors)

    assert np.allclose(r_est, r_true, atol=1e-10)


def test_wahba_svd_error_shrinks_with_more_measurements():
    q = quaternion_from_axis_angle([1.0, 2.0, -1.0], np.deg2rad(73.0))
    r_true = quaternion_to_rotation_matrix(q)
    rng = np.random.default_rng(1)

    def mean_angular_error(n_vec, noise_deg, n_trials=150):
        errors = []
        for _ in range(n_trials):
            r_vectors = rng.normal(size=(n_vec, 3))
            r_vectors /= np.linalg.norm(r_vectors, axis=1, keepdims=True)
            b_true = (r_true @ r_vectors.T).T
            noisy = b_true + rng.normal(scale=np.deg2rad(noise_deg), size=(n_vec, 3))
            noisy /= np.linalg.norm(noisy, axis=1, keepdims=True)

            r_est = wahba_svd_attitude(noisy, r_vectors)
            q_est = rotation_matrix_to_quaternion(r_est)
            cos_half_err = np.clip(abs(q_est[0] * q[0] + q_est[1:] @ q[1:]), 0.0, 1.0)
            errors.append(2 * np.degrees(np.arccos(cos_half_err)))
        return np.mean(errors)

    err_2 = mean_angular_error(2, noise_deg=1.0)
    err_16 = mean_angular_error(16, noise_deg=1.0)

    assert err_16 < err_2


def test_weighted_measurement_dominates_the_estimate():
    q = quaternion_from_axis_angle([0.0, 0.0, 1.0], np.deg2rad(30.0))
    r_true = quaternion_to_rotation_matrix(q)

    r1, r2 = np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0])
    b1_true, b2_true = r_true @ r1, r_true @ r2
    b2_corrupted = b2_true + np.array([0.3, 0.0, 0.0])  # badly wrong measurement
    b2_corrupted /= np.linalg.norm(b2_corrupted)

    r_heavy_1 = wahba_svd_attitude([b1_true, b2_corrupted], [r1, r2], weights=[1000.0, 1.0])
    r_heavy_2 = wahba_svd_attitude([b1_true, b2_corrupted], [r1, r2], weights=[1.0, 1000.0])

    err_heavy_1 = np.linalg.norm(r_heavy_1 - r_true)
    err_heavy_2 = np.linalg.norm(r_heavy_2 - r_true)

    assert err_heavy_1 < err_heavy_2
