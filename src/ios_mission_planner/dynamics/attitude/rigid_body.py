import heyoka as hy


def quaternion_kinematics_symbolic(q0, q1, q2, q3, wx, wy, wz):
    """
    Quaternion kinematics, as symbolic heyoka expressions.

    q = (q0, q1, q2, q3), Hamilton convention with q0 the scalar part,
    represents the rotation from the inertial frame to the body frame.
    wx, wy, wz are the body's angular velocity, expressed in the body
    frame. The kinematic differential equation is q_dot = 0.5 * q (x) w,
    where (x) is the Hamilton quaternion product and w is treated as the
    pure quaternion (0, wx, wy, wz).

    Parameters
    ----------
    q0, q1, q2, q3, wx, wy, wz : heyoka.expression

    Returns
    -------
    tuple[heyoka.expression, heyoka.expression, heyoka.expression, heyoka.expression]
        (q0_dot, q1_dot, q2_dot, q3_dot).

    Author: Giovanni Facchinetti, 2026
    Reference: Markley, Crassidis, Fundamentals of Spacecraft Attitude
    Determination and Control
    """

    q0_dot = 0.5 * (-q1 * wx - q2 * wy - q3 * wz)
    q1_dot = 0.5 * (q0 * wx + q2 * wz - q3 * wy)
    q2_dot = 0.5 * (q0 * wy - q1 * wz + q3 * wx)
    q3_dot = 0.5 * (q0 * wz + q1 * wy - q2 * wx)

    return q0_dot, q1_dot, q2_dot, q3_dot


def euler_rotational_equations_symbolic(wx, wy, wz, ix, iy, iz, taux, tauy, tauz):
    """
    Euler's rotational equations for a rigid body on its principal axes,
    as symbolic heyoka expressions.

    ix, iy, iz are the principal moments of inertia (the inertia tensor is
    assumed diagonal in the body frame used for wx, wy, wz); taux, tauy,
    tauz is the external torque, in the same body frame.

    Parameters
    ----------
    wx, wy, wz, ix, iy, iz, taux, tauy, tauz : heyoka.expression

    Returns
    -------
    tuple[heyoka.expression, heyoka.expression, heyoka.expression]
        (wx_dot, wy_dot, wz_dot) [rad/s^2].

    Author: Giovanni Facchinetti, 2026
    Reference: Markley, Crassidis, Fundamentals of Spacecraft Attitude
    Determination and Control
    """

    wx_dot = ((iy - iz) * wy * wz + taux) / ix
    wy_dot = ((iz - ix) * wz * wx + tauy) / iy
    wz_dot = ((ix - iy) * wx * wy + tauz) / iz

    return wx_dot, wy_dot, wz_dot


def rigid_body_attitude_dynamics_symbolic():
    """
    Torque-driven rigid body attitude dynamics, as a symbolic heyoka ODE
    system: quaternion kinematics coupled to Euler's rotational equations,
    with an externally supplied, constant body-frame torque.

    State vector: [q0, q1, q2, q3, wx, wy, wz]. q is the quaternion from
    the inertial to the body frame (Hamilton convention, q0 scalar); wx,
    wy, wz is the angular velocity in the body frame [rad/s].
    Runtime parameters: par[0:3] = (ix, iy, iz), principal moments of
    inertia [kg m^2], par[3:6] = (taux, tauy, tauz), body-frame torque
    [N m]. Setting the torque to zero gives torque-free rigid body motion.

    Returns
    -------
    list[tuple[heyoka.expression, heyoka.expression]]
        Symbolic system, for use with
        ios_mission_planner.propagation.heyoka_propagator.propagate.

    Author: Giovanni Facchinetti, 2026
    """

    q0, q1, q2, q3, wx, wy, wz = hy.make_vars("q0", "q1", "q2", "q3", "wx", "wy", "wz")
    ix, iy, iz = hy.par[0], hy.par[1], hy.par[2]
    taux, tauy, tauz = hy.par[3], hy.par[4], hy.par[5]

    q0_dot, q1_dot, q2_dot, q3_dot = quaternion_kinematics_symbolic(q0, q1, q2, q3, wx, wy, wz)
    wx_dot, wy_dot, wz_dot = euler_rotational_equations_symbolic(wx, wy, wz, ix, iy, iz, taux, tauy, tauz)

    return [
        (q0, q0_dot),
        (q1, q1_dot),
        (q2, q2_dot),
        (q3, q3_dot),
        (wx, wx_dot),
        (wy, wy_dot),
        (wz, wz_dot),
    ]


def pd_pointing_attitude_dynamics_symbolic():
    """
    Rigid body attitude dynamics under quaternion-feedback PD pointing
    control, as a symbolic heyoka ODE system.

    Drives the body attitude toward a fixed target quaternion q_t with a
    proportional-derivative control torque built from the attitude error
    quaternion q_e = conj(q_t) (x) q and the body rate:

        torque = -kp * (qe1, qe2, qe3) - kd * (wx, wy, wz)

    using only the vector part of the error quaternion, which goes to zero
    exactly when the body attitude reaches the target (qe0 -> +-1). This
    control law does not correct for unwinding (settling through the long
    way around when qe0 starts near -1); a real controller would flip the
    sign of the proportional term based on the sign of qe0 to always take
    the short rotation, which this simplified version omits.

    State vector: [q0, q1, q2, q3, wx, wy, wz], same convention as
    rigid_body_attitude_dynamics_symbolic.
    Runtime parameters: par[0:3] = (ix, iy, iz), principal moments of
    inertia [kg m^2], par[3:7] = (qt0, qt1, qt2, qt3), the target
    quaternion (should be unit norm), par[7] = kp, proportional gain,
    par[8] = kd, derivative gain.

    Returns
    -------
    list[tuple[heyoka.expression, heyoka.expression]]
        Symbolic system, for use with
        ios_mission_planner.propagation.heyoka_propagator.propagate.

    Author: Giovanni Facchinetti, 2026
    Reference: Markley, Crassidis, Fundamentals of Spacecraft Attitude
    Determination and Control
    """

    q0, q1, q2, q3, wx, wy, wz = hy.make_vars("q0", "q1", "q2", "q3", "wx", "wy", "wz")
    ix, iy, iz = hy.par[0], hy.par[1], hy.par[2]
    qt0, qt1, qt2, qt3 = hy.par[3], hy.par[4], hy.par[5], hy.par[6]
    kp, kd = hy.par[7], hy.par[8]

    # error quaternion q_e = conj(q_t) (x) q, conj(q_t) = (qt0, -qt1, -qt2, -qt3)
    qe1 = qt0 * q1 - qt1 * q0 - qt2 * q3 + qt3 * q2
    qe2 = qt0 * q2 + qt1 * q3 - qt2 * q0 - qt3 * q1
    qe3 = qt0 * q3 - qt1 * q2 + qt2 * q1 - qt3 * q0

    taux = -kp * qe1 - kd * wx
    tauy = -kp * qe2 - kd * wy
    tauz = -kp * qe3 - kd * wz

    q0_dot, q1_dot, q2_dot, q3_dot = quaternion_kinematics_symbolic(q0, q1, q2, q3, wx, wy, wz)
    wx_dot, wy_dot, wz_dot = euler_rotational_equations_symbolic(wx, wy, wz, ix, iy, iz, taux, tauy, tauz)

    return [
        (q0, q0_dot),
        (q1, q1_dot),
        (q2, q2_dot),
        (q3, q3_dot),
        (wx, wx_dot),
        (wy, wy_dot),
        (wz, wz_dot),
    ]
