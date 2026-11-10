import numpy as np

from ios_mission_planner.dynamics.attitude.determination import (
    quaternion_to_rotation_matrix,
    rotation_matrix_to_quaternion,
    wahba_svd_attitude,
)
from ios_mission_planner.dynamics.attitude.filter import MEKFState, mekf_predict, mekf_update
from ios_mission_planner.dynamics.attitude.rigid_body import rigid_body_attitude_dynamics_symbolic
from ios_mission_planner.propagation.heyoka_propagator import propagate


def test_predict_matches_exact_dynamics_for_a_principal_axis_spin():
    # a spin about one principal axis is an equilibrium of the torque-free
    # Euler equations (omega stays exactly constant), so predict's
    # constant-rate closed-form quaternion update should match the full
    # dynamics exactly, with zero injected noise/bias
    ix, iy, iz = 10.0, 10.0, 10.0
    omega_true = np.array([0.0, 0.0, 0.3])
    state0 = np.array([1.0, 0.0, 0.0, 0.0, *omega_true])
    t_final = 7.3

    q_true = propagate(
        rigid_body_attitude_dynamics_symbolic(), state0, np.array([0.0, t_final]),
        pars=[ix, iy, iz, 0.0, 0.0, 0.0],
    ).y[:4, -1]

    dt = 0.01
    mekf_state = MEKFState(q=np.array([1.0, 0.0, 0.0, 0.0]), bias=np.zeros(3), covariance=np.eye(6) * 1e-6)
    for _ in range(int(t_final / dt)):
        mekf_state = mekf_predict(mekf_state, omega_true, dt, gyro_noise_std=0.0, bias_noise_std=0.0)

    assert min(np.linalg.norm(mekf_state.q - q_true), np.linalg.norm(mekf_state.q + q_true)) < 1e-9


def test_update_reduces_covariance_and_stays_valid():
    rng = np.random.default_rng(0)
    state = MEKFState(q=np.array([1.0, 0.0, 0.0, 0.0]), bias=np.zeros(3), covariance=np.eye(6) * 0.01)

    q_true = np.array([0.9689, 0.0, 0.0, 0.2474])
    q_true = q_true / np.linalg.norm(q_true)
    r_vectors = rng.normal(size=(6, 3))
    r_vectors /= np.linalg.norm(r_vectors, axis=1, keepdims=True)
    b_vectors = (quaternion_to_rotation_matrix(q_true) @ r_vectors.T).T
    b_vectors += rng.normal(scale=np.deg2rad(1.0), size=(6, 3))
    b_vectors /= np.linalg.norm(b_vectors, axis=1, keepdims=True)
    q_measured = rotation_matrix_to_quaternion(wahba_svd_attitude(b_vectors, r_vectors))

    trace_before = np.trace(state.covariance)
    state_after = mekf_update(state, q_measured, measurement_covariance=np.eye(3) * np.deg2rad(1.0) ** 2)

    assert np.trace(state_after.covariance) < trace_before
    assert np.allclose(state_after.covariance, state_after.covariance.T)
    assert np.all(np.linalg.eigvalsh(state_after.covariance) >= -1e-10)


def _run_filter_scenario(n_steps, dt, true_bias, gyro_noise_std, update_every, meas_noise_deg, seed):
    rng = np.random.default_rng(seed)
    ix, iy, iz = 10.0, 15.0, 20.0
    state0 = np.array([1.0, 0.0, 0.0, 0.0, 0.05, 0.03, 0.3])
    t_eval = np.linspace(0.0, n_steps * dt, n_steps + 1)
    true_sol = propagate(
        rigid_body_attitude_dynamics_symbolic(), state0, t_eval, pars=[ix, iy, iz, 0.0, 0.0, 0.0],
    ).y

    r_vectors_ref = rng.normal(size=(6, 3))
    r_vectors_ref /= np.linalg.norm(r_vectors_ref, axis=1, keepdims=True)
    measurement_cov = np.eye(3) * np.deg2rad(meas_noise_deg) ** 2

    filt = MEKFState(
        q=np.array([1.0, 0.0, 0.0, 0.0]), bias=np.zeros(3),
        covariance=np.diag([np.deg2rad(5.0) ** 2] * 3 + [0.01**2] * 3),
    )
    dead = MEKFState(q=np.array([1.0, 0.0, 0.0, 0.0]), bias=np.zeros(3), covariance=np.eye(6))

    for k in range(n_steps):
        gyro_meas = true_sol[4:7, k] + true_bias + rng.normal(scale=gyro_noise_std, size=3)
        filt = mekf_predict(filt, gyro_meas, dt, gyro_noise_std, bias_noise_std=1e-6)
        dead = mekf_predict(dead, gyro_meas, dt, gyro_noise_std, bias_noise_std=0.0)

        if (k + 1) % update_every == 0:
            r_true_k = quaternion_to_rotation_matrix(true_sol[:4, k + 1])
            b_meas = (r_true_k @ r_vectors_ref.T).T
            b_meas += rng.normal(scale=np.deg2rad(meas_noise_deg), size=(6, 3))
            b_meas /= np.linalg.norm(b_meas, axis=1, keepdims=True)
            q_est = rotation_matrix_to_quaternion(wahba_svd_attitude(b_meas, r_vectors_ref))
            filt = mekf_update(filt, q_est, measurement_cov)

    def angular_error_deg(q_a, q_b):
        return 2 * np.degrees(np.arccos(np.clip(abs(q_a @ q_b), 0.0, 1.0)))

    q_true_final = true_sol[:4, -1]
    return filt, dead, angular_error_deg(filt.q, q_true_final), angular_error_deg(dead.q, q_true_final)


def test_filter_tracks_attitude_better_than_dead_reckoning():
    true_bias = np.array([0.002, -0.001, 0.0015])
    _, _, err_filt, err_dead = _run_filter_scenario(
        n_steps=3000, dt=0.1, true_bias=true_bias, gyro_noise_std=0.0005,
        update_every=20, meas_noise_deg=0.5, seed=42,
    )

    assert err_filt < 1.0  # well under a degree
    assert err_dead > 10.0  # uncorrected bias has visibly drifted
    assert err_filt < err_dead / 10.0


def test_bias_estimate_converges_toward_truth():
    true_bias = np.array([0.002, -0.001, 0.0015])
    filt, _, _, _ = _run_filter_scenario(
        n_steps=3000, dt=0.1, true_bias=true_bias, gyro_noise_std=0.0005,
        update_every=20, meas_noise_deg=0.5, seed=42,
    )

    assert np.linalg.norm(filt.bias - true_bias) < 0.0005  # well under the bias's own magnitude
