import numpy as np

from ios_mission_planner.dynamics.attitude.quaternion import (
    quaternion_error,
    quaternion_multiply,
    quaternion_to_rotation_vector,
    rotation_vector_to_quaternion,
)
from ios_mission_planner.dynamics.relative.cw import cw_discrete_matrices
from ios_mission_planner.gnc.control import (
    ApproachMPC,
    HoldController,
    attitude_gains,
    cw_feedforward_acceleration,
    dlqr,
    quaternion_feedback_torque,
    tracking_weights,
)

N = 0.0011
DT = 4.0
U_MAX = 0.0125


def test_dlqr_solves_the_riccati_equation_and_stabilizes():
    phi, gamma = cw_discrete_matrices(DT, N)
    q, r = tracking_weights(1.0, 0.03, 4e-3)
    gain, cost = dlqr(phi, gamma, q, r)

    closed_loop = phi - gamma @ gain
    assert np.max(np.abs(np.linalg.eigvals(closed_loop))) < 1.0

    riccati = closed_loop.T @ cost @ closed_loop + q + gain.T @ r @ gain
    assert np.allclose(riccati, cost, rtol=1e-8)


def test_feedforward_makes_the_reference_motion_a_solution():
    phi, gamma = cw_discrete_matrices(1.0, N)

    # a point at rest off the V-bar
    position = np.array([30.0, -200.0, 10.0])
    state = np.concatenate([position, np.zeros(3)])
    feedforward = cw_feedforward_acceleration(N, position, np.zeros(3))
    for _ in range(600):
        state = phi @ state + gamma @ feedforward
    assert np.allclose(state[:3], position, atol=1e-9)

    # a point on the V-bar needs nothing
    assert np.allclose(cw_feedforward_acceleration(N, [0.0, -500.0, 0.0], np.zeros(3)), 0.0)

    # a constant-speed motion along the V-bar needs a radial thrust of 2 n v
    assert np.allclose(cw_feedforward_acceleration(N, [0.0, -100.0, 0.0], [0.0, 0.3, 0.0]),
                       [-2.0 * N * 0.3, 0.0, 0.0])


def test_hold_controller_brings_the_deputy_to_the_hold_point():
    q, r = tracking_weights(5.0, 0.05, 5e-4)
    controller = HoldController(N, DT, q, r)
    phi, gamma = cw_discrete_matrices(DT, N)
    reference = np.array([0.0, -500.0, 0.0])

    state = np.array([8.0, -520.0, -5.0, 0.02, -0.01, 0.01])
    for _ in range(600):
        state = phi @ state + gamma @ controller.acceleration(state, reference)

    assert np.linalg.norm(state[:3] - reference) < 1e-3
    assert np.linalg.norm(state[3:]) < 1e-5


def _mpc(constraint_matrix=None, horizon=30):
    q, r = tracking_weights(0.5, 0.03, 4e-3)
    return ApproachMPC(N, DT, horizon, q, r, U_MAX, constraint_matrix)


def test_mpc_reproduces_the_lqr_when_no_constraint_is_active():
    mpc = _mpc()
    reference = np.tile([0.0, -20.0, 0.0, 0.0, 0.0, 0.0], (31, 1))
    state = np.array([0.05, -20.1, -0.03, 0.001, 0.0, -0.001])

    command = mpc.solve(state, reference)
    lqr = -mpc.lqr_gain @ (state - reference[0])

    assert mpc.last_status == "solved"
    assert np.max(np.abs(lqr)) < U_MAX
    assert np.allclose(command, lqr, atol=2e-5)


def test_mpc_respects_the_thrust_limit():
    mpc = _mpc()
    reference = np.tile([0.0, -20.0, 0.0, 0.0, 0.0, 0.0], (31, 1))
    state = np.array([15.0, -60.0, -10.0, 0.0, 0.0, 0.0])

    command = mpc.solve(state, reference)
    lqr = -mpc.lqr_gain @ (state - reference[0])

    assert np.max(np.abs(lqr)) > U_MAX            # the regulator alone would ask for more
    assert np.max(np.abs(command)) <= U_MAX * (1.0 + 1e-3)


def test_mpc_slows_down_for_a_closing_speed_constraint_the_lqr_ignores():
    # constraint: velocity along +y not above 0.1 m/s
    constraint = np.array([[0.0, 0.0, 0.0, 0.0, 1.0, 0.0]])
    constrained = _mpc(constraint)
    free = _mpc()
    phi, gamma = cw_discrete_matrices(DT, N)
    reference = np.tile([0.0, -20.0, 0.0, 0.0, 0.0, 0.0], (31, 1))

    def fly(controller, bound):
        state = np.array([0.0, -80.0, 0.0, 0.0, 0.0, 0.0])
        fastest = 0.0
        for _ in range(150):
            command = controller.solve(state, reference, constraint_bound=bound)
            state = phi @ state + gamma @ command
            fastest = max(fastest, state[4])
        return fastest, state

    fastest_free, _ = fly(free, None)
    fastest_constrained, final = fly(constrained, np.array([0.1]))

    assert fastest_free > 0.3
    assert fastest_constrained < 0.1 + 5e-3
    assert np.linalg.norm(final[:3] - reference[0, :3]) < 30.0   # still getting there, only slower


def test_mpc_stays_feasible_when_the_state_starts_outside_a_soft_constraint():
    constraint = np.array([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0]])    # x <= 1
    mpc = _mpc(constraint)
    reference = np.tile([0.0, -20.0, 0.0, 0.0, 0.0, 0.0], (31, 1))

    command = mpc.solve(np.array([5.0, -20.0, 0.0, 0.0, 0.0, 0.0]), reference, constraint_bound=np.array([1.0]))

    assert command is not None
    assert mpc.last_slack > 1.0       # the violation is acknowledged, not hidden
    assert command[0] < 0.0           # and the command pushes back inside


def _simulate_attitude(q0, rate0, target, target_rate, inertia, kp, kd, steps, dt=0.5, max_slew_rate=None,
                       max_torque=None):
    """Rigid body under the feedback law, integrated with small explicit steps."""

    q, rate = np.array(q0, dtype=float), np.array(rate0, dtype=float)
    fastest = 0.0
    for _ in range(steps):
        torque = quaternion_feedback_torque(q, rate, target, target_rate, inertia, np.zeros(3), kp, kd,
                                            max_slew_rate)
        if max_torque is not None:
            torque = np.clip(torque, -max_torque, max_torque)
        for _ in range(10):
            rate = rate + (torque - np.cross(rate, inertia * rate)) / inertia * dt / 10
            q = quaternion_multiply(q, rotation_vector_to_quaternion(rate * dt / 10))
        fastest = max(fastest, np.linalg.norm(rate))
    return q, rate, fastest


def test_attitude_gains_give_the_requested_closed_loop():
    inertia = np.array([22.0, 25.0, 28.0])
    kp, kd = attitude_gains(inertia, 0.08, 0.9)

    assert np.allclose(kp / (2.0 * inertia), 0.08**2)
    assert np.allclose(kd / (2.0 * np.sqrt(kp / 2.0 * inertia)), 0.9)


def test_quaternion_feedback_converges_to_the_target():
    inertia = np.array([22.0, 25.0, 28.0])
    kp, kd = attitude_gains(inertia, 0.08, 0.9)
    target = rotation_vector_to_quaternion([0.2, -0.1, 0.3])
    q0 = quaternion_multiply(target, rotation_vector_to_quaternion([0.05, 0.04, -0.06]))

    q, rate, _ = _simulate_attitude(q0, [0.001, 0.0, -0.001], target, np.zeros(3), inertia, kp, kd, 600)

    assert np.linalg.norm(quaternion_to_rotation_vector(quaternion_error(q, target))) < 1e-6
    assert np.linalg.norm(rate) < 1e-7


def test_quaternion_feedback_takes_the_short_way_and_caps_the_slew_rate():
    inertia = np.array([22.0, 25.0, 28.0])
    kp, kd = attitude_gains(inertia, 0.08, 0.9)
    target = np.array([1.0, 0.0, 0.0, 0.0])
    slew = np.deg2rad(0.5)

    # 170 degrees away, written with the sign that would unwind the long way
    q0 = -rotation_vector_to_quaternion([0.0, 0.0, np.deg2rad(170.0)])
    torque = quaternion_feedback_torque(q0, np.zeros(3), target, np.zeros(3), inertia, np.zeros(3), kp, kd, slew)
    assert torque[2] < 0.0      # back through 170 degrees, not forward through 190

    q, rate, fastest = _simulate_attitude(q0, np.zeros(3), target, np.zeros(3), inertia, kp, kd, 1200,
                                          max_slew_rate=slew, max_torque=0.02)
    assert fastest < 1.15 * slew
    assert np.linalg.norm(quaternion_to_rotation_vector(quaternion_error(q, target))) < 1e-4


def test_quaternion_feedback_tracks_a_rotating_target_without_lag():
    inertia = np.array([22.0, 25.0, 28.0])
    kp, kd = attitude_gains(inertia, 0.08, 0.9)
    target_rate = np.array([0.0, 0.0, N])     # once per orbit about the body z axis

    dt, steps = 0.5, 800
    q, rate = np.array([1.0, 0.0, 0.0, 0.0]), target_rate.copy()
    target = np.array([1.0, 0.0, 0.0, 0.0])
    for _ in range(steps):
        torque = quaternion_feedback_torque(q, rate, target, target_rate, inertia, np.zeros(3), kp, kd)
        rate = rate + (torque - np.cross(rate, inertia * rate)) / inertia * dt
        q = quaternion_multiply(q, rotation_vector_to_quaternion(rate * dt))
        target = quaternion_multiply(target, rotation_vector_to_quaternion(target_rate * dt))

    assert np.linalg.norm(quaternion_to_rotation_vector(quaternion_error(q, target))) < 1e-6
