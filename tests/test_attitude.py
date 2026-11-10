import numpy as np

from ios_mission_planner.dynamics.attitude.rigid_body import (
    pd_pointing_attitude_dynamics_symbolic,
    rigid_body_attitude_dynamics_symbolic,
)
from ios_mission_planner.propagation.heyoka_propagator import propagate

IX, IY, IZ = 10.0, 15.0, 20.0


def test_identity_attitude_at_rest_with_no_torque_stays_put():
    state0 = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    pars = [IX, IY, IZ, 0.0, 0.0, 0.0]

    final = propagate(rigid_body_attitude_dynamics_symbolic(), state0, [0.0, 100.0], pars=pars).y[:, -1]

    assert np.allclose(final, state0, atol=1e-10)


def test_quaternion_norm_preserved_in_torque_free_tumbling():
    state0 = np.array([1.0, 0.0, 0.0, 0.0, 0.1, 0.2, -0.05])
    pars = [IX, IY, IZ, 0.0, 0.0, 0.0]
    t_eval = np.linspace(0.0, 200.0, 500)

    sol = propagate(rigid_body_attitude_dynamics_symbolic(), state0, t_eval, pars=pars).y
    qnorm = np.linalg.norm(sol[:4, :], axis=0)

    assert np.allclose(qnorm, 1.0, atol=1e-8)


def test_energy_and_angular_momentum_conserved_torque_free():
    state0 = np.array([1.0, 0.0, 0.0, 0.0, 0.1, 0.2, -0.05])
    pars = [IX, IY, IZ, 0.0, 0.0, 0.0]
    t_eval = np.linspace(0.0, 200.0, 500)

    sol = propagate(rigid_body_attitude_dynamics_symbolic(), state0, t_eval, pars=pars).y
    w = sol[4:7, :]
    inertia = np.diag([IX, IY, IZ])

    ke = np.array([0.5 * w[:, i] @ inertia @ w[:, i] for i in range(w.shape[1])])
    h_mag = np.array([np.linalg.norm(inertia @ w[:, i]) for i in range(w.shape[1])])

    assert np.allclose(ke, ke[0], rtol=1e-8)
    assert np.allclose(h_mag, h_mag[0], rtol=1e-8)


def test_pd_controller_converges_to_the_target_attitude():
    axis = np.array([1.0, 1.0, 1.0]) / np.sqrt(3.0)
    angle0 = np.deg2rad(60.0)
    q_init = np.concatenate([[np.cos(angle0 / 2)], np.sin(angle0 / 2) * axis])
    state0 = np.concatenate([q_init, [0.0, 0.0, 0.0]])

    q_target = np.array([1.0, 0.0, 0.0, 0.0])
    pars = [IX, IY, IZ, *q_target, 2.0, 8.0]
    t_eval = np.linspace(0.0, 60.0, 400)

    sol = propagate(pd_pointing_attitude_dynamics_symbolic(), state0, t_eval, pars=pars).y

    pointing_error_deg = 2 * np.degrees(np.arccos(np.clip(np.abs(sol[0, :]), 0.0, 1.0)))

    assert pointing_error_deg[0] > 50.0
    assert pointing_error_deg[-1] < 1.0
    assert np.linalg.norm(sol[4:7, -1]) < 1e-3  # angular velocity settles near zero


def test_pd_controller_error_decreases_monotonically_enough():
    # a well-damped PD response should not oscillate wildly; check the
    # pointing error's broad trend is downward, sampled coarsely
    axis = np.array([0.0, 0.0, 1.0])
    angle0 = np.deg2rad(45.0)
    q_init = np.concatenate([[np.cos(angle0 / 2)], np.sin(angle0 / 2) * axis])
    state0 = np.concatenate([q_init, [0.0, 0.0, 0.0]])

    pars = [IX, IY, IZ, 1.0, 0.0, 0.0, 0.0, 3.0, 10.0]
    t_eval = np.linspace(0.0, 40.0, 5)

    sol = propagate(pd_pointing_attitude_dynamics_symbolic(), state0, t_eval, pars=pars).y
    pointing_error_deg = 2 * np.degrees(np.arccos(np.clip(np.abs(sol[0, :]), 0.0, 1.0)))

    assert np.all(np.diff(pointing_error_deg) <= 1e-6)
