import copy

import heyoka as hy
import numpy as np

from ios_mission_planner.dynamics.attitude.rigid_body import quaternion_kinematics_symbolic
from ios_mission_planner.dynamics.orbital.drag import drag_acceleration_symbolic
from ios_mission_planner.dynamics.orbital.perturbations import j2_acceleration_symbolic
from ios_mission_planner.dynamics.orbital.two_body import two_body_acceleration_symbolic

N_STATES = 23
N_PARS = 23

# slices of the state vector
CHIEF_R = slice(0, 3)
CHIEF_V = slice(3, 6)
DEPUTY_R = slice(6, 9)
DEPUTY_V = slice(9, 12)
DEPUTY_MASS = 12
DEPUTY_Q = slice(13, 17)
DEPUTY_W = slice(17, 20)
WHEEL_H = slice(20, 23)

# slices of the runtime parameter vector
PAR_MU, PAR_J2, PAR_R_EQ = 0, 1, 2
PAR_BC_CHIEF, PAR_BC_DEPUTY = 3, 4
PAR_RHO0, PAR_R0, PAR_SCALE_HEIGHT, PAR_OMEGA_BODY = 5, 6, 7, 8
PAR_THRUST = slice(9, 12)
PAR_MDOT = 12
PAR_INERTIA = slice(13, 16)
PAR_WHEEL_TORQUE = slice(16, 19)
PAR_EXTERNAL_TORQUE = slice(19, 22)
PAR_GRAVITY_GRADIENT = 22


def docking_truth_dynamics_symbolic():
    """
    Six-degree-of-freedom truth model of a docking mission, as a symbolic
    heyoka ODE system: the absolute orbits of the chief and of the deputy,
    and the mass, attitude, body rate and reaction-wheel momentum of the
    deputy, all integrated together.

    Both spacecraft feel two-body gravity, J2 and atmospheric drag, each
    with its own ballistic coefficient, so the relative motion contains
    the nonlinearity, the eccentricity induced by J2 and the differential
    drag that the linear model used on board ignores. The deputy also
    feels its thrust, commanded in the body frame and rotated to the
    inertial frame by its actual attitude, so that a pointing error
    becomes a thrust direction error. Its attitude obeys Euler's equations
    with reaction wheels,

        I w_dot = tau_w + tau_ext + tau_gg - w x (I w + h_w),
        h_w_dot = -tau_w,

    where tau_w is the torque the wheels apply to the body, h_w the
    angular momentum stored in the wheels (body frame, wheels along the
    principal axes), tau_ext an external disturbance torque and tau_gg the
    gravity-gradient torque 3 mu / r^5 (r_b x I r_b), switched by a
    parameter.

    State vector (23): chief position and velocity [km, km/s] (0:6),
    deputy position and velocity [km, km/s] (6:12), deputy mass [kg]
    (12), deputy quaternion from inertial to body, scalar first (13:17),
    deputy body rate [rad/s] (17:20), wheel momentum [N m s] (20:23).

    Runtime parameters (23): par[0] = mu, par[1] = j2, par[2] = r_eq,
    par[3] = ballistic coefficient of the chief, par[4] = of the deputy
    [m^2/kg], par[5:8] = rho0, r0, scale_height of the exponential
    atmosphere, par[8] = rotation rate of the central body, par[9:12] =
    thrust in the body frame [N], par[12] = propellant mass flow [kg/s],
    par[13:16] = principal moments of inertia [kg m^2], par[16:19] =
    wheel torque on the body [N m], par[19:22] = external torque on the
    body [N m], par[22] = gravity-gradient switch (0 or 1). Setting j2 or
    rho0 to zero removes the corresponding perturbation.

    Thrust, mass flow and torques are meant to be held constant over one
    control step and changed between steps.

    Returns
    -------
    list[tuple[heyoka.expression, heyoka.expression]]

    Author: Giovanni Facchinetti, 2026
    Reference: Markley, Crassidis, Fundamentals of Spacecraft Attitude
    Determination and Control; Vallado, Fundamentals of Astrodynamics and
    Applications
    """

    xc, yc, zc, vxc, vyc, vzc = hy.make_vars("xc", "yc", "zc", "vxc", "vyc", "vzc")
    xd, yd, zd, vxd, vyd, vzd = hy.make_vars("xd", "yd", "zd", "vxd", "vyd", "vzd")
    m = hy.make_vars("m")
    q0, q1, q2, q3 = hy.make_vars("q0", "q1", "q2", "q3")
    wx, wy, wz = hy.make_vars("wx", "wy", "wz")
    hx, hy_, hz = hy.make_vars("hx", "hy", "hz")

    mu, j2, r_eq = hy.par[PAR_MU], hy.par[PAR_J2], hy.par[PAR_R_EQ]
    bc_c, bc_d = hy.par[PAR_BC_CHIEF], hy.par[PAR_BC_DEPUTY]
    rho0, r0, scale_height = hy.par[PAR_RHO0], hy.par[PAR_R0], hy.par[PAR_SCALE_HEIGHT]
    omega_body = hy.par[PAR_OMEGA_BODY]
    fx, fy, fz = hy.par[9], hy.par[10], hy.par[11]
    mdot = hy.par[PAR_MDOT]
    ix, iy, iz = hy.par[13], hy.par[14], hy.par[15]
    twx, twy, twz = hy.par[16], hy.par[17], hy.par[18]
    tex, tey, tez = hy.par[19], hy.par[20], hy.par[21]
    gg = hy.par[PAR_GRAVITY_GRADIENT]

    def natural_acceleration(x, y, z, vx, vy, vz, bc):
        a2b = two_body_acceleration_symbolic(x, y, z, mu)
        aj2 = j2_acceleration_symbolic(x, y, z, mu, j2, r_eq)
        adr = drag_acceleration_symbolic(x, y, z, vx, vy, vz, bc, rho0, r0, scale_height, omega_body)
        return tuple(a + b + c for a, b, c in zip(a2b, aj2, adr))

    axc, ayc, azc = natural_acceleration(xc, yc, zc, vxc, vyc, vzc, bc_c)
    axd, ayd, azd = natural_acceleration(xd, yd, zd, vxd, vyd, vzd, bc_d)

    # inertial-to-body rotation matrix of the deputy, same convention as
    # dynamics.attitude.determination.quaternion_to_rotation_matrix
    r11 = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
    r12 = 2.0 * (q1 * q2 + q0 * q3)
    r13 = 2.0 * (q1 * q3 - q0 * q2)
    r21 = 2.0 * (q1 * q2 - q0 * q3)
    r22 = 1.0 - 2.0 * (q1 * q1 + q3 * q3)
    r23 = 2.0 * (q2 * q3 + q0 * q1)
    r31 = 2.0 * (q1 * q3 + q0 * q2)
    r32 = 2.0 * (q2 * q3 - q0 * q1)
    r33 = 1.0 - 2.0 * (q1 * q1 + q2 * q2)

    # thrust: body to inertial is the transpose; N / kg = m/s^2, 1e-3 to km/s^2
    atx = 1e-3 * (r11 * fx + r21 * fy + r31 * fz) / m
    aty = 1e-3 * (r12 * fx + r22 * fy + r32 * fz) / m
    atz = 1e-3 * (r13 * fx + r23 * fy + r33 * fz) / m

    # gravity-gradient torque, from the deputy position in body axes
    rbx = r11 * xd + r12 * yd + r13 * zd
    rby = r21 * xd + r22 * yd + r23 * zd
    rbz = r31 * xd + r32 * yd + r33 * zd
    rd5 = (xd * xd + yd * yd + zd * zd) ** 2.5
    gg_factor = gg * 3.0 * mu / rd5
    tggx = gg_factor * (iz - iy) * rby * rbz
    tggy = gg_factor * (ix - iz) * rbz * rbx
    tggz = gg_factor * (iy - ix) * rbx * rby

    # total angular momentum in the body frame
    lx = ix * wx + hx
    ly = iy * wy + hy_
    lz = iz * wz + hz

    wx_dot = (twx + tex + tggx - (wy * lz - wz * ly)) / ix
    wy_dot = (twy + tey + tggy - (wz * lx - wx * lz)) / iy
    wz_dot = (twz + tez + tggz - (wx * ly - wy * lx)) / iz

    q0_dot, q1_dot, q2_dot, q3_dot = quaternion_kinematics_symbolic(q0, q1, q2, q3, wx, wy, wz)

    return [
        (xc, vxc), (yc, vyc), (zc, vzc),
        (vxc, axc), (vyc, ayc), (vzc, azc),
        (xd, vxd), (yd, vyd), (zd, vzd),
        (vxd, axd + atx), (vyd, ayd + aty), (vzd, azd + atz),
        (m, -mdot),
        (q0, q0_dot), (q1, q1_dot), (q2, q2_dot), (q3, q3_dot),
        (wx, wx_dot), (wy, wy_dot), (wz, wz_dot),
        (hx, -twx), (hy_, -twy), (hz, -twz),
    ]


_INTEGRATOR_TEMPLATE = None


def _integrator_template():
    """Compile the truth integrator once per process and hand out copies."""

    global _INTEGRATOR_TEMPLATE

    if _INTEGRATOR_TEMPLATE is None:
        state = np.zeros(N_STATES)
        state[0] = state[6] = 7000.0
        state[4] = state[10] = 7.5
        state[DEPUTY_MASS] = 1.0
        state[13] = 1.0
        pars = np.zeros(N_PARS)
        pars[PAR_MU] = 398600.4418
        pars[PAR_R_EQ] = 6378.137
        pars[PAR_R0] = 7000.0
        pars[PAR_SCALE_HEIGHT] = 60.0
        pars[PAR_INERTIA] = 1.0
        _INTEGRATOR_TEMPLATE = hy.taylor_adaptive(
            docking_truth_dynamics_symbolic(), state=state.tolist(), pars=pars.tolist(),
            compact_mode=True,
        )

    return _INTEGRATOR_TEMPLATE


class TruthModel:
    """
    The truth model of the simulator: a persistent heyoka integrator of
    docking_truth_dynamics_symbolic, advanced one control step at a time
    with the actuator outputs held constant over the step.

    The integrator is compiled once per process; every TruthModel works
    on its own copy, so that many simulations (a Monte Carlo campaign)
    pay the compilation once.

    Parameters
    ----------
    mu, j2, r_eq, omega_body : float
        Gravitational parameter [km^3/s^2], J2 coefficient, equatorial
        radius [km] and rotation rate [rad/s] of the central body.
    rho0, r0, scale_height : float
        Exponential atmosphere: reference density [kg/m^3] at the radius
        r0 [km], and scale height [km].
    bc_chief, bc_deputy : float
        Ballistic coefficients Cd*A/m [m^2/kg].
    inertia : array_like, shape (3,)
        Principal moments of inertia of the deputy [kg m^2].
    gravity_gradient : bool
        Whether the gravity-gradient torque acts on the deputy.

    Author: Giovanni Facchinetti, 2026
    """

    def __init__(self, mu, j2, r_eq, omega_body, rho0, r0, scale_height,
                 bc_chief, bc_deputy, inertia, gravity_gradient=True):
        self._ta = copy.deepcopy(_integrator_template())

        pars = np.zeros(N_PARS)
        pars[PAR_MU], pars[PAR_J2], pars[PAR_R_EQ] = mu, j2, r_eq
        pars[PAR_BC_CHIEF], pars[PAR_BC_DEPUTY] = bc_chief, bc_deputy
        pars[PAR_RHO0], pars[PAR_R0], pars[PAR_SCALE_HEIGHT] = rho0, r0, scale_height
        pars[PAR_OMEGA_BODY] = omega_body
        pars[PAR_INERTIA] = inertia
        pars[PAR_GRAVITY_GRADIENT] = 1.0 if gravity_gradient else 0.0
        self._ta.pars[:] = pars

        self.inertia = np.asarray(inertia, dtype=float)

    def reset(self, chief_state, deputy_state, mass, quaternion, body_rate, wheel_momentum=(0.0, 0.0, 0.0),
              time=0.0):
        """
        Set the full state and the epoch.

        Parameters
        ----------
        chief_state, deputy_state : array_like, shape (6,)
            Inertial position [km] and velocity [km/s].
        mass : float
            Deputy mass [kg].
        quaternion : array_like, shape (4,)
            Deputy attitude, inertial to body, scalar first.
        body_rate : array_like, shape (3,)
            Deputy angular velocity in the body frame [rad/s].
        wheel_momentum : array_like, shape (3,)
            Angular momentum stored in the wheels [N m s].
        time : float
            Epoch [s].
        """

        state = self._ta.state
        state[0:6] = chief_state
        state[6:12] = deputy_state
        state[DEPUTY_MASS] = mass
        q = np.asarray(quaternion, dtype=float)
        state[DEPUTY_Q] = q / np.linalg.norm(q)
        state[DEPUTY_W] = body_rate
        state[WHEEL_H] = wheel_momentum
        self._ta.time = float(time)
        self.set_controls()

    def set_controls(self, thrust_body=(0.0, 0.0, 0.0), mass_flow=0.0,
                     wheel_torque=(0.0, 0.0, 0.0), external_torque=(0.0, 0.0, 0.0)):
        """
        Set the actuator outputs held over the next step: thrust in the
        body frame [N], propellant mass flow [kg/s], torque applied by
        the wheels to the body [N m], external disturbance torque on the
        body [N m].
        """

        pars = self._ta.pars
        pars[PAR_THRUST] = thrust_body
        pars[PAR_MDOT] = mass_flow
        pars[PAR_WHEEL_TORQUE] = wheel_torque
        pars[PAR_EXTERNAL_TORQUE] = external_torque

    def step(self, dt):
        """Advance the truth by dt seconds with the current controls."""

        self._ta.propagate_for(float(dt))

        # the quaternion norm is an invariant of the equations; removing
        # the integration error on it keeps the rotation matrices exact
        q = self._ta.state[DEPUTY_Q]
        q /= np.sqrt(q @ q)

    @property
    def time(self):
        return self._ta.time

    @property
    def state(self):
        """A copy of the full 23-element state vector."""
        return self._ta.state.copy()

    @property
    def chief_state(self):
        return self._ta.state[0:6].copy()

    @property
    def deputy_state(self):
        return self._ta.state[6:12].copy()

    @property
    def mass(self):
        return float(self._ta.state[DEPUTY_MASS])

    @property
    def quaternion(self):
        return self._ta.state[DEPUTY_Q].copy()

    @property
    def body_rate(self):
        return self._ta.state[DEPUTY_W].copy()

    @property
    def wheel_momentum(self):
        return self._ta.state[WHEEL_H].copy()
