import numpy as np
import osqp
import scipy.sparse as sp
from scipy.linalg import solve_discrete_are

from ios_mission_planner.dynamics.attitude.determination import quaternion_to_rotation_matrix
from ios_mission_planner.dynamics.attitude.quaternion import quaternion_error
from ios_mission_planner.dynamics.relative.cw import cw_discrete_matrices
from ios_mission_planner.linalg import cross3


def cw_feedforward_acceleration(n, position, velocity, acceleration=(0.0, 0.0, 0.0)):
    """
    Acceleration that the thrusters must supply for the deputy to follow
    a prescribed relative motion exactly, according to the
    Clohessy-Wiltshire equations: the acceleration of the reference minus
    the natural acceleration (gravity gradient and Coriolis terms) at the
    reference state.

    A point at rest on the V-bar needs none. A point at rest off the
    V-bar, or one moving along a straight line, needs a continuous
    thrust, and this is how much.

    Parameters
    ----------
    n : float
        Mean motion of the chief [rad/s].
    position, velocity : array_like, shape (3,)
        Reference relative position [m] and velocity [m/s] in the LVLH
        frame.
    acceleration : array_like, shape (3,)
        Acceleration of the reference [m/s^2].

    Returns
    -------
    numpy.ndarray, shape (3,)

    Author: Giovanni Facchinetti, 2026
    """

    x, _, z = position
    vx, vy, _ = velocity
    natural = np.array([3.0 * n**2 * x + 2.0 * n * vy, -2.0 * n * vx, -(n**2) * z])

    return np.asarray(acceleration, dtype=float) - natural


def dlqr(a, b, q, r):
    """
    Discrete-time linear quadratic regulator: the gain K of the state
    feedback u = -K x that minimizes the sum over time of x'Qx + u'Ru for
    the system x[k+1] = A x[k] + B u[k], and the matrix P of the optimal
    cost x'Px, solution of the discrete algebraic Riccati equation.

    Parameters
    ----------
    a, b : numpy.ndarray
        System matrices.
    q, r : numpy.ndarray
        State and input weights, symmetric, q positive semidefinite and
        r positive definite.

    Returns
    -------
    k : numpy.ndarray
    p : numpy.ndarray

    Author: Giovanni Facchinetti, 2026
    Reference: Anderson, Moore, Optimal Control: Linear Quadratic Methods
    """

    p = solve_discrete_are(a, b, q, r)
    k = np.linalg.solve(r + b.T @ p @ b, b.T @ p @ a)

    return k, p


def tracking_weights(position_tolerance, velocity_tolerance, acceleration_scale):
    """
    Weights of a quadratic cost by Bryson's rule: each quantity is
    weighted by the inverse square of the deviation considered
    acceptable, so that the cost compares numbers of order one.

    Returns
    -------
    q : numpy.ndarray, shape (6, 6)
    r : numpy.ndarray, shape (3, 3)

    Author: Giovanni Facchinetti, 2026
    """

    q = np.diag([1.0 / position_tolerance**2] * 3 + [1.0 / velocity_tolerance**2] * 3)
    r = np.eye(3) / acceleration_scale**2

    return q, r


class HoldController:
    """
    Linear quadratic regulator that keeps the deputy at a reference
    relative state: the feedforward acceleration that makes the reference
    an equilibrium, plus a state feedback on the deviation from it.

    Parameters
    ----------
    n : float
        Mean motion of the chief [rad/s].
    dt : float
        Control period [s].
    q, r : numpy.ndarray
        Weights of the cost, see tracking_weights.

    Author: Giovanni Facchinetti, 2026
    """

    def __init__(self, n, dt, q, r):
        self.n = n
        self.dt = dt
        phi, gamma = cw_discrete_matrices(dt, n)
        self.gain, self.cost = dlqr(phi, gamma, q, r)

    def acceleration(self, state, reference_position, reference_velocity=(0.0, 0.0, 0.0)):
        """Commanded acceleration in the LVLH frame [m/s^2], not limited."""

        reference = np.concatenate([reference_position, reference_velocity])
        feedforward = cw_feedforward_acceleration(self.n, reference_position, reference_velocity)

        return feedforward - self.gain @ (np.asarray(state, dtype=float) - reference)


class ApproachMPC:
    """
    Model predictive controller for the final approach.

    At every control period it solves, over a horizon of N steps, the
    quadratic program

        minimize   sum_k (x_k - r_k)' Q (x_k - r_k) + (u_k - f_k)' R (u_k - f_k)
                   + (x_N - r_N)' P (x_N - r_N) + penalty on the slacks
        subject to x_{k+1} = Phi x_k + Gamma u_k,   x_0 = current state,
                   |u_k| <= u_max on each axis,
                   C x_k <= d + s_k,   s_k >= 0,

    and applies only the first acceleration u_0; at the next period the
    problem is solved again from the new state. r is the reference
    trajectory, f the feedforward acceleration that would follow it
    exactly, and P the cost of the linear quadratic regulator with the
    same weights, which stands for the cost beyond the horizon.

    The thrust limit is a hard constraint, because it is a physical fact.
    The constraints on the state (approach corridor, closing speed) are
    softened by the slack variables s, at a price high enough that they
    are violated only when it is impossible not to: a hard state
    constraint would make the problem infeasible, and leave the
    controller without an answer, as soon as a disturbance pushed the
    state outside.

    The problem is solved with OSQP. Its matrices do not change between
    periods, only the vectors do, so the factorization is reused and each
    solution starts from the previous one.

    Parameters
    ----------
    n : float
        Mean motion of the chief [rad/s].
    dt : float
        Control period [s].
    horizon : int
        Number of steps N of the prediction.
    q, r : numpy.ndarray
        Weights of the cost, see tracking_weights.
    max_acceleration : float
        Limit of the acceleration on each axis [m/s^2].
    constraint_matrix : numpy.ndarray, shape (n_c, 6), optional
        Matrix C of the state constraints C x <= d.
    slack_linear, slack_quadratic : float
        Weights of the linear and quadratic penalties on the slacks. The
        linear weight must exceed what the tracking cost would gain by
        violating a constraint, or the constraint is traded away; the
        default is large enough for tracking errors of hundreds of
        metres with the weights of tracking_weights.

    Author: Giovanni Facchinetti, 2026
    Reference: Rawlings, Mayne, Diehl, Model Predictive Control: Theory,
    Computation, and Design; Stellato et al., "OSQP: an operator splitting
    solver for quadratic programs," Mathematical Programming Computation,
    2020
    """

    def __init__(self, n, dt, horizon, q, r, max_acceleration, constraint_matrix=None,
                 slack_linear=1e6, slack_quadratic=1e6):
        self.n = n
        self.dt = dt
        self.horizon = horizon
        self.max_acceleration = max_acceleration

        phi, gamma = cw_discrete_matrices(dt, n)
        self.phi, self.gamma = phi, gamma
        self.q = np.asarray(q, dtype=float)
        self.r = np.asarray(r, dtype=float)
        self.lqr_gain, self.terminal_cost = dlqr(phi, gamma, self.q, self.r)

        c = np.zeros((0, 6)) if constraint_matrix is None else np.asarray(constraint_matrix, dtype=float)
        self.constraint_matrix = c
        n_c = c.shape[0]
        self.n_constraints = n_c

        big_n = horizon
        nx, nu = 6 * (big_n + 1), 3 * big_n
        ns = n_c * big_n
        self._nx, self._nu, self._ns = nx, nu, ns

        # inputs are normalized by the acceleration limit, so that states
        # and inputs have comparable magnitudes in the solver
        gamma_s = gamma * max_acceleration
        self._r_scaled = self.r * max_acceleration**2

        cost = sp.block_diag([
            sp.kron(sp.eye(big_n), self.q), self.terminal_cost,
            sp.kron(sp.eye(big_n), self._r_scaled),
            slack_quadratic * sp.eye(ns),
        ], format="csc")
        self._slack_linear = slack_linear

        dynamics_x = sp.kron(sp.eye(big_n + 1), -sp.eye(6)) + sp.kron(sp.eye(big_n + 1, k=-1), phi)
        dynamics_u = sp.kron(sp.vstack([sp.csc_matrix((1, big_n)), sp.eye(big_n)]), gamma_s)
        rows = [sp.hstack([dynamics_x, dynamics_u, sp.csc_matrix((nx, ns))])]
        rows.append(sp.hstack([sp.csc_matrix((nu, nx)), sp.eye(nu), sp.csc_matrix((nu, ns))]))
        if ns > 0:
            select = sp.hstack([sp.csc_matrix((big_n, 1)), sp.eye(big_n)])
            rows.append(sp.hstack([sp.kron(select, c), sp.csc_matrix((ns, nu)), -sp.eye(ns)]))
            rows.append(sp.hstack([sp.csc_matrix((ns, nx + nu)), sp.eye(ns)]))
        constraints = sp.vstack(rows, format="csc")

        self._lower = np.concatenate([np.zeros(nx), -np.ones(nu), -np.inf * np.ones(ns), np.zeros(ns)])
        self._upper = np.concatenate([np.zeros(nx), np.ones(nu), np.zeros(ns), np.inf * np.ones(ns)])
        self._linear = np.zeros(nx + nu + ns)
        self._linear[nx + nu:] = slack_linear

        self._solver = osqp.OSQP()
        self._solver.setup(
            sp.triu(cost, format="csc"), self._linear, constraints, self._lower, self._upper,
            verbose=False, warm_starting=True, eps_abs=1e-5, eps_rel=1e-5, max_iter=4000,
            polishing=False,
        )

        self.last_status = None
        self.last_prediction = None
        self.last_slack = 0.0

    def solve(self, state, reference, feedforward=None, constraint_bound=None):
        """
        Solve the problem from the current state.

        Parameters
        ----------
        state : array_like, shape (6,)
            Current relative state [m, m/s].
        reference : array_like, shape (N + 1, 6)
            Reference states over the horizon.
        feedforward : array_like, shape (N, 3), optional
            Feedforward accelerations over the horizon [m/s^2].
        constraint_bound : array_like, shape (n_c,) or (N, n_c), optional
            Right-hand side d of the state constraints, the same at every
            step or one per step.

        Returns
        -------
        numpy.ndarray, shape (3,) or None
            Acceleration to apply over the next period [m/s^2], or None
            if the solver did not return a solution.
        """

        big_n = self.horizon
        nx, nu, ns = self._nx, self._nu, self._ns
        state = np.asarray(state, dtype=float)
        reference = np.asarray(reference, dtype=float)

        linear = self._linear
        linear[:nx - 6] = -(reference[:-1] @ self.q).ravel()
        linear[nx - 6:nx] = -self.terminal_cost @ reference[-1]
        if feedforward is None:
            linear[nx:nx + nu] = 0.0
        else:
            scaled = np.asarray(feedforward, dtype=float) / self.max_acceleration
            linear[nx:nx + nu] = -(scaled @ self._r_scaled).ravel()

        self._lower[:6] = -state
        self._upper[:6] = -state
        if ns > 0:
            bound = np.asarray(constraint_bound, dtype=float)
            if bound.ndim == 1:
                bound = np.tile(bound, big_n)
            self._upper[nx + nu:nx + nu + ns] = bound.ravel()

        self._solver.update(q=linear, l=self._lower, u=self._upper)
        result = self._solver.solve(raise_error=False)
        self.last_status = result.info.status

        if result.info.status not in ("solved", "solved inaccurate") or result.x is None:
            return None

        self.last_prediction = result.x[:nx].reshape(big_n + 1, 6)
        self.last_slack = float(result.x[nx + nu:].max()) if ns > 0 else 0.0

        return result.x[nx:nx + 3] * self.max_acceleration


def attitude_gains(inertia, natural_frequency, damping_ratio):
    """
    Gains of the quaternion feedback law for a prescribed closed-loop
    natural frequency and damping ratio, axis by axis. For small errors
    the vector part of the error quaternion is half the error angle,
    which is where the factor of two in the proportional gain comes from.

    Returns
    -------
    kp, kd : numpy.ndarray, shape (3,)

    Author: Giovanni Facchinetti, 2026
    Reference: Wie, Space Vehicle Dynamics and Control
    """

    inertia = np.asarray(inertia, dtype=float)
    kp = 2.0 * inertia * natural_frequency**2
    kd = 2.0 * damping_ratio * natural_frequency * inertia

    return kp, kd


def quaternion_feedback_torque(quaternion, body_rate, target_quaternion, target_rate, inertia,
                               wheel_momentum, kp, kd, max_slew_rate=None):
    """
    Quaternion feedback control torque for tracking a moving target
    attitude with reaction wheels.

        tau = -kp * sat(q_e) - kd * (w - w_target) + w x (I w + h_w)

    q_e is the vector part of the error quaternion conj(q_target) (x) q,
    taken with the sign that gives the shorter rotation, so that the body
    never unwinds the long way around. The rate term damps the motion
    with respect to the target rate, not to zero, so a target that
    rotates (a deputy that keeps facing the chief turns once per orbit)
    is tracked without a steady error. The last term cancels the
    gyroscopic torque of the body and of the wheels.

    The proportional term is limited in magnitude, so that for a large
    error it balances the rate term at a chosen slew rate: the body turns
    at that constant rate until the error is small, which keeps the
    wheels away from their momentum limit during large maneuvers.

    Parameters
    ----------
    quaternion, target_quaternion : array_like, shape (4,)
        Actual and commanded attitude, inertial to body, scalar first.
    body_rate : array_like, shape (3,)
        Angular velocity in the body frame [rad/s].
    target_rate : array_like, shape (3,)
        Angular velocity of the commanded attitude, in the commanded
        body frame [rad/s].
    inertia : array_like, shape (3,)
        Principal moments of inertia [kg m^2].
    wheel_momentum : array_like, shape (3,)
        Angular momentum stored in the wheels [N m s].
    kp, kd : array_like, shape (3,)
        Gains, see attitude_gains.
    max_slew_rate : float, optional
        Slew rate for large errors [rad/s].

    Returns
    -------
    numpy.ndarray, shape (3,)
        Torque requested from the wheels [N m].

    Author: Giovanni Facchinetti, 2026
    Reference: Wie, Space Vehicle Dynamics and Control
    """

    inertia = np.asarray(inertia, dtype=float)
    body_rate = np.asarray(body_rate, dtype=float)
    kp = np.asarray(kp, dtype=float)
    kd = np.asarray(kd, dtype=float)

    error = quaternion_error(quaternion, target_quaternion)
    target_rate_body = quaternion_to_rotation_matrix(error) @ np.asarray(target_rate, dtype=float)

    proportional = kp * error[1:]
    if max_slew_rate is not None:
        ratio = np.max(np.abs(proportional) / (kd * max_slew_rate))
        if ratio > 1.0:
            proportional = proportional / ratio

    gyroscopic = cross3(body_rate, inertia * body_rate + np.asarray(wheel_momentum, dtype=float))

    return -proportional - kd * (body_rate - target_rate_body) + gyroscopic
