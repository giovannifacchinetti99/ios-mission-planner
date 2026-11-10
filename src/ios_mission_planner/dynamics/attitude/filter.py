from dataclasses import dataclass

import numpy as np


def _quaternion_multiply(q, p):
    """Hamilton product q (x) p, both (q0,q1,q2,q3), q0 scalar first."""

    q0, q1, q2, q3 = q
    p0, p1, p2, p3 = p

    return np.array([
        q0 * p0 - q1 * p1 - q2 * p2 - q3 * p3,
        q0 * p1 + q1 * p0 + q2 * p3 - q3 * p2,
        q0 * p2 - q1 * p3 + q2 * p0 + q3 * p1,
        q0 * p3 + q1 * p2 - q2 * p1 + q3 * p0,
    ])


def _quaternion_conjugate(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def _skew(v):
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ])


@dataclass
class MEKFState:
    """
    State of the multiplicative extended Kalman filter (MEKF): the current
    best attitude and gyro bias estimates, and the 6x6 error-state
    covariance (3 attitude-error components, 3 bias components).

    The attitude error itself is never carried as part of this state: a
    quaternion covariance would be singular, because of the unit-norm
    constraint, which is exactly the problem the "multiplicative" part of
    MEKF avoids, by keeping the small attitude error in a minimal,
    unconstrained 3-parameter form internally and folding it into q on
    every update instead.

    Attributes
    ----------
    q : numpy.ndarray, shape (4,)
        Attitude estimate, Hamilton convention, same inertial-to-body
        convention as ios_mission_planner.dynamics.attitude.rigid_body.
    bias : numpy.ndarray, shape (3,)
        Gyro bias estimate [rad/s].
    covariance : numpy.ndarray, shape (6, 6)
        Error-state covariance: [:3, :3] attitude error [rad^2], [3:, 3:]
        bias error [(rad/s)^2], cross terms in the off-diagonal blocks.

    Author: Giovanni Facchinetti, 2026
    Reference: Lefferts, Markley, Shuster, "Kalman Filtering for
    Spacecraft Attitude Estimation," Journal of Guidance, Control, and
    Dynamics, 5(5), 1982; Markley, Crassidis, Fundamentals of Spacecraft
    Attitude Determination and Control.
    """

    q: np.ndarray
    bias: np.ndarray
    covariance: np.ndarray


def mekf_predict(state, gyro_measurement, dt, gyro_noise_std, bias_noise_std):
    """
    Time-propagate an MEKFState by one step, from a (noisy, biased) gyro
    measurement.

    The attitude estimate is propagated with the bias-corrected rate,
    using the exact closed-form quaternion update for a constant rate
    over the step (rather than a first-order Euler step, which would
    drift the quaternion off unit norm); the gyro bias is modeled as a
    random walk, with no deterministic dynamics of its own. The
    error-state covariance is propagated with a first-order-in-dt
    discretization of the continuous error dynamics, valid as long as dt
    is small compared to how fast the attitude error itself evolves,
    which holds for any gyro sampled at a realistic rate relative to the
    body rates in this project's scenarios.

    Parameters
    ----------
    state : MEKFState
    gyro_measurement : array_like, shape (3,)
        Measured angular velocity [rad/s], bias and noise included.
    dt : float
        Time step [s].
    gyro_noise_std : float
        Standard deviation of the gyro's angle random walk [rad/s],
        driving the attitude error's process noise.
    bias_noise_std : float
        Standard deviation of the gyro bias's random walk [rad/s per
        sqrt(s)], driving the bias error's process noise.

    Returns
    -------
    MEKFState

    Author: Giovanni Facchinetti, 2026
    """

    omega_hat = np.asarray(gyro_measurement, dtype=float) - state.bias
    omega_norm = np.linalg.norm(omega_hat)

    if omega_norm > 1e-12:
        axis = omega_hat / omega_norm
        angle = omega_norm * dt
        delta_q = np.concatenate([[np.cos(angle / 2)], axis * np.sin(angle / 2)])
    else:
        delta_q = np.array([1.0, 0.0, 0.0, 0.0])

    q_new = _quaternion_multiply(state.q, delta_q)
    q_new = q_new / np.linalg.norm(q_new)

    f = np.zeros((6, 6))
    f[:3, :3] = -_skew(omega_hat)
    f[:3, 3:] = -np.eye(3)
    phi = np.eye(6) + f * dt

    g = np.zeros((6, 6))
    g[:3, :3] = -np.eye(3)
    g[3:, 3:] = np.eye(3)
    q_continuous = np.diag([gyro_noise_std**2] * 3 + [bias_noise_std**2] * 3)
    q_discrete = g @ q_continuous @ g.T * dt

    covariance_new = phi @ state.covariance @ phi.T + q_discrete

    return MEKFState(q=q_new, bias=state.bias.copy(), covariance=covariance_new)


def mekf_update(state, q_measured, measurement_covariance):
    """
    Correct an MEKFState with an attitude measurement (e.g. the output of
    triad_attitude or wahba_svd_attitude, converted to a quaternion with
    rotation_matrix_to_quaternion), by one step of the Kalman measurement
    update.

    The measurement model observes the attitude error directly (H = [I,
    0]: a single attitude solution has no sensitivity to the gyro bias on
    its own), with the residual taken as (twice) the vector part of the
    error quaternion between the measurement and the current estimate,
    the standard small-angle approximation of the rotation vector between
    two nearby attitudes.

    Parameters
    ----------
    state : MEKFState
    q_measured : array_like, shape (4,)
        An independent attitude measurement, same quaternion convention.
    measurement_covariance : array_like, shape (3, 3)
        Covariance of the attitude measurement's rotation-vector error
        [rad^2].

    Returns
    -------
    MEKFState

    Author: Giovanni Facchinetti, 2026
    """

    q_measured = np.asarray(q_measured, dtype=float)
    q_measured = q_measured / np.linalg.norm(q_measured)

    # body-frame error convention (q_measured = q_hat (x) delta_q), matching
    # the process model's F matrix, so left-multiply by conj(q_hat) to solve
    # for delta_q, not q_measured (x) conj(q_hat) as a reference-frame error
    # convention would
    error_q = _quaternion_multiply(_quaternion_conjugate(state.q), q_measured)
    if error_q[0] < 0:
        error_q = -error_q  # shortest-rotation convention, avoids an unwinding residual
    residual = 2.0 * error_q[1:]

    h = np.zeros((3, 6))
    h[:, :3] = np.eye(3)

    innovation_cov = h @ state.covariance @ h.T + np.asarray(measurement_covariance, dtype=float)
    kalman_gain = state.covariance @ h.T @ np.linalg.inv(innovation_cov)

    correction = kalman_gain @ residual
    delta_theta, delta_bias = correction[:3], correction[3:]

    delta_q = np.concatenate([[1.0], 0.5 * delta_theta])
    delta_q = delta_q / np.linalg.norm(delta_q)
    q_new = _quaternion_multiply(state.q, delta_q)  # right-multiply: body-frame correction
    q_new = q_new / np.linalg.norm(q_new)

    bias_new = state.bias + delta_bias

    i_kh = np.eye(6) - kalman_gain @ h
    covariance_new = (
        i_kh @ state.covariance @ i_kh.T
        + kalman_gain @ np.asarray(measurement_covariance, dtype=float) @ kalman_gain.T
    )

    return MEKFState(q=q_new, bias=bias_new, covariance=covariance_new)
