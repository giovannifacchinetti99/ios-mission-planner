# Attitude dynamics and pointing control

Every other chapter in this book treats a spacecraft as a point mass: its
position and velocity are everything that matters. A real rendezvous and
proximity operation also cares which way the spacecraft is facing, keeping
a sensor, antenna, or docking port pointed where it needs to be while the
translational dynamics built everywhere else in this project fly the
trajectory. This chapter adds the rotational counterpart to that point-mass
dynamics: how a rigid body's orientation evolves, and a simple feedback law
that points it at a target.

## Representing orientation: quaternions

Code: {py:func}`quaternion_kinematics_symbolic
<ios_mission_planner.dynamics.attitude.rigid_body.quaternion_kinematics_symbolic>`
in `src/ios_mission_planner/dynamics/attitude/rigid_body.py`.
Reference: Markley, Crassidis, *Fundamentals of Spacecraft Attitude
Determination and Control*.

A rotation from the inertial frame to a body frame can be represented in
several ways: a $3\times3$ rotation matrix, a set of Euler angles, or a
unit quaternion $\vect q = (q_0, q_1, q_2, q_3)$, with $q_0$ the scalar
part (the Hamilton convention used throughout this chapter). Quaternions
avoid the singularities Euler angles have at certain orientations (gimbal
lock) and are cheaper to propagate and renormalize than a full rotation
matrix, which is why they are the standard representation for numerically
integrating attitude. Given the body's angular velocity $\vect\omega$,
expressed in the body frame, the quaternion evolves by

$$
\dot{\vect q} = \frac12\,\vect q \otimes \vect\omega,
$$

treating $\vect\omega$ as the pure quaternion $(0,\omega_x,\omega_y,
\omega_z)$ and $\otimes$ as the Hamilton product. Expanded into components,
this is exactly what `quaternion_kinematics_symbolic` returns:

$$
\dot q_0 = -\tfrac12(q_1\omega_x+q_2\omega_y+q_3\omega_z), \qquad
\dot q_1 = \tfrac12(q_0\omega_x+q_2\omega_z-q_3\omega_y),
$$

$$
\dot q_2 = \tfrac12(q_0\omega_y-q_1\omega_z+q_3\omega_x), \qquad
\dot q_3 = \tfrac12(q_0\omega_z+q_1\omega_y-q_2\omega_x).
$$

A quaternion that starts with unit norm stays at unit norm under this
equation exactly, in continuous time; what the validation below actually
checks is that the numerical integration preserves that property too.

## Euler's rotational equations

Code: {py:func}`euler_rotational_equations_symbolic
<ios_mission_planner.dynamics.attitude.rigid_body.euler_rotational_equations_symbolic>`,
{py:func}`rigid_body_attitude_dynamics_symbolic
<ios_mission_planner.dynamics.attitude.rigid_body.rigid_body_attitude_dynamics_symbolic>`
in `src/ios_mission_planner/dynamics/attitude/rigid_body.py`.

Quaternion kinematics says how orientation changes given the angular
velocity; Euler's rotational equations say how the angular velocity itself
changes under an applied torque. For a rigid body on its principal axes,
with diagonal inertia tensor $\mathrm{diag}(I_x,I_y,I_z)$ in the body
frame,

$$
I_x\dot\omega_x = (I_y-I_z)\,\omega_y\omega_z + \tau_x, \qquad
I_y\dot\omega_y = (I_z-I_x)\,\omega_z\omega_x + \tau_y, \qquad
I_z\dot\omega_z = (I_x-I_y)\,\omega_x\omega_y + \tau_z.
$$

The cross terms are the gyroscopic coupling between axes: unless the body
is symmetric about two of its principal axes ($I_x=I_y$, say), rotation
about one axis feeds into the others, which is exactly why a tumbling body
with three distinct principal moments does not simply spin about a fixed
axis. `rigid_body_attitude_dynamics_symbolic` couples this to the
quaternion kinematics above into one seven-state system, with the torque
supplied as a runtime parameter; setting it to zero gives torque-free
motion, the case with the cleanest validation available, since both the
rotational kinetic energy $\tfrac12\vect\omega\cdot\mathbf I\vect\omega$
and the magnitude of the angular momentum $\mathbf I\vect\omega$ are
exactly conserved with no torque applied, the rotational analogue of the
two-body energy and angular momentum checks from the absolute dynamics
chapter.

![Torque-free tumbling: body rates vary, but energy and angular momentum stay flat to machine precision](images/attitude_torque_free_validation.png)

## Pointing control

Code: {py:func}`pd_pointing_attitude_dynamics_symbolic
<ios_mission_planner.dynamics.attitude.rigid_body.pd_pointing_attitude_dynamics_symbolic>`
in `src/ios_mission_planner/dynamics/attitude/rigid_body.py`.

Keeping a docking port or sensor pointed at a target needs feedback: a
torque that pushes the attitude error toward zero and damps out the
angular velocity as it gets there, the same proportional-derivative
structure used informally everywhere else a `kp`/`kd` pair shows up. The
attitude error between the current orientation $\vect q$ and a fixed
target $\vect q_t$ is itself a quaternion, $\vect q_e =
\mathrm{conj}(\vect q_t)\otimes\vect q$, whose vector part
$(q_{e1},q_{e2},q_{e3})$ is zero exactly when the body is at the target
attitude (and whose scalar part $q_{e0}\to\pm1$ there). The control torque

$$
\vect\tau = -k_p\,(q_{e1},q_{e2},q_{e3}) - k_d\,\vect\omega
$$

drives that vector part, and the body rate alongside it, toward zero.
`pd_pointing_attitude_dynamics_symbolic` builds this directly into the
symbolic system, the same way the low-thrust chapter's arc gate builds a
smooth on/off switch into its own dynamics, rather than computing a torque
externally and feeding it back in at every step.

![PD pointing control: a 60-degree attitude error settles to a fraction of a degree, with the body rate decaying alongside it](images/attitude_pd_pointing.png)

This control law is deliberately the simple version. A real implementation
would flip the sign of the proportional term based on the sign of $q_{e0}$
to always take the shorter of the two rotations between the current and
target attitude; without that correction, a starting attitude on the "far
side" ($q_{e0}$ near $-1$) can settle the long way around instead, a known
issue called unwinding that this chapter's controller does not correct
for.

## What this chapter does not build

This stays a self-contained rotational model. The attitude state here is
not coupled to the orbital or relative-motion state built everywhere else
in this project: there is no model of how a pointing requirement
constrains an approach trajectory, no plume impingement or sensor
field-of-view check tied to the actual relative geometry from the relative
dynamics chapter, and no estimator behind any of it, the controller above
assumes the attitude and rate are known exactly rather than measured
through a real sensor and filter. Coupling pointing requirements to the
trajectories this project already designs, and closing the loop with an
actual navigation filter, are substantial pieces of future work this
chapter does not attempt.

## See it in action

The worked example for this chapter is
{doc}`../examples/11_attitude_pointing_control/attitude_pointing_control`,
which runs the torque-free validation and the pointing control scenario
above.
