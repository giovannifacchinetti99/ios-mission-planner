# Attitude dynamics and control

Up to this point a spacecraft has been a point: a position and a velocity.
A servicer is also a body with an orientation, and the orientation matters
at every stage of the approach. The relative sensor must look at the
target, the thrusters must push in the direction the guidance asks for, and
at contact the docking port must be aligned with the port of the chief.
This chapter describes how the orientation of a rigid body is represented,
how it evolves under torques, and how a feedback law drives it to a
commanded attitude.

## Representing orientation: quaternions

The attitude of a spacecraft is the rotation that takes the axes of an
inertial frame into the axes of a frame fixed to the body. A rotation has
three degrees of freedom and can be written in several ways. A rotation
matrix uses nine numbers with six constraints. Three Euler angles use the
minimum number of parameters but become singular at certain orientations,
where two of the three rotations act about the same axis and one degree of
freedom is lost. A unit quaternion uses four numbers with one constraint,
has no singularity, and is the standard choice for numerical work.

By Euler's theorem any rotation is a single rotation by an angle $\theta$
about an axis $\hat{\vect e}$. The corresponding quaternion is

$$
\vect q = \begin{bmatrix} q_0 \\ \vect q_v \end{bmatrix} =
\begin{bmatrix} \cos(\theta/2) \\ \hat{\vect e}\,\sin(\theta/2)
\end{bmatrix},
$$

with scalar part $q_0$ and vector part $\vect q_v = (q_1,q_2,q_3)$, and
with unit norm. Two properties follow from the half angle. The quaternions
$\vect q$ and $-\vect q$ represent the same attitude, since a rotation by
$\theta$ about $\hat{\vect e}$ and one by $2\pi - \theta$ about
$-\hat{\vect e}$ end in the same orientation. And for a small rotation the
vector part is half the rotation vector, $\vect q_v \approx
\tfrac12\theta\,\hat{\vect e}$, which is what makes the vector part a
convenient measure of a small attitude error.

Rotations are composed with the quaternion product,

$$
\vect p \otimes \vect q = \begin{bmatrix}
p_0 q_0 - \vect p_v\cdot\vect q_v \\
p_0\,\vect q_v + q_0\,\vect p_v + \vect p_v\times\vect q_v
\end{bmatrix},
$$

which is not commutative, as rotations are not. The inverse of a unit
quaternion is its conjugate, obtained by changing the sign of the vector
part. Throughout the package, $\vect q$ describes the rotation from the
inertial frame to the body frame, with the scalar part first.

## Kinematics

If the body turns with angular velocity $\vect\omega$, expressed in body
axes, the quaternion evolves as

$$
\dot{\vect q} = \frac12\,\vect q \otimes
\begin{bmatrix}0\\ \vect\omega\end{bmatrix},
$$

or, in components,

$$
\dot q_0 = -\tfrac12\,\vect q_v\cdot\vect\omega, \qquad
\dot{\vect q}_v = \tfrac12\left(q_0\,\vect\omega + \vect q_v\times
\vect\omega\right).
$$

The equation is linear in the quaternion and preserves its norm exactly. A
numerical integration preserves it only to its own accuracy, so the norm is
a first check of the quality of the integration.

## Dynamics: Euler's equations

The kinematics gives the attitude from the angular velocity. The dynamics
gives the angular velocity from the torques. The angular momentum of a
rigid body is $\vect H = \mathbf I\,\vect\omega$, with $\mathbf I$ the
inertia tensor, and Newton's law for rotation states that its inertial
rate of change equals the applied torque. Written in body axes, which turn
with the body, the law acquires a transport term:

$$
\mathbf I\,\dot{\vect\omega} + \vect\omega\times\mathbf I\,\vect\omega =
\vect\tau .
$$

In principal axes the inertia tensor is diagonal and the equation becomes
the three Euler equations,

$$
I_x\dot\omega_x = (I_y-I_z)\,\omega_y\omega_z + \tau_x, \qquad
I_y\dot\omega_y = (I_z-I_x)\,\omega_z\omega_x + \tau_y, \qquad
I_z\dot\omega_z = (I_x-I_y)\,\omega_x\omega_y + \tau_z .
$$

The products of angular rates are the gyroscopic coupling between the axes.
Unless two principal moments are equal, a rotation about one axis feeds the
other two, which is why a body with three different moments of inertia
tumbles instead of spinning about a fixed axis.

With no torque applied, two quantities are conserved: the rotational
kinetic energy $\tfrac12\,\vect\omega\cdot\mathbf I\,\vect\omega$ and the
magnitude of the angular momentum $|\mathbf I\,\vect\omega|$. They are the
rotational counterparts of the energy and angular momentum of the two-body
problem and serve the same purpose, the validation of the numerical model.

## Pointing control

Pointing the body at a commanded attitude $\vect q_t$ requires a torque
that depends on the error. The error is itself a rotation, the one that
takes the commanded attitude into the current one,

$$
\vect q_e = \vect q_t^{-1} \otimes \vect q ,
$$

whose vector part vanishes when the two coincide. The classical control law
is proportional to the vector part of the error and to the angular
velocity,

$$
\vect\tau = -k_p\,\vect q_{e,v} - k_d\,\vect\omega .
$$

The first term is a spring that pulls the body toward the target, the
second a damper that removes the angular velocity as it gets there. For a
rigid body this law is globally stabilizing, which can be shown with a
Lyapunov function that adds the kinetic energy to a measure of the attitude
error.

The gains follow from the small-angle behaviour. Near the target, $\vect
q_{e,v}\approx\tfrac12\vect\theta$ and each axis behaves as

$$
I\,\ddot\theta + k_d\,\dot\theta + \tfrac12 k_p\,\theta = 0 ,
$$

a damped oscillator with natural frequency $\omega_n$ and damping ratio
$\zeta$ given by

$$
k_p = 2\,I\,\omega_n^2, \qquad k_d = 2\,\zeta\,I\,\omega_n .
$$

Choosing the bandwidth $\omega_n$ and a damping ratio close to one fixes
both gains. Scaling the gains with the inertia of each axis gives the same
response on all three.

Because $\vect q_e$ and $-\vect q_e$ are the same attitude, the law above
has two equilibria in quaternion space, and from an error larger than half
a turn it rotates the long way round. This behaviour is called unwinding.
The remedy is to multiply the proportional term by the sign of the scalar
part of the error, so that the body always takes the shorter rotation.

This structure, a proportional action on the quaternion error and a
derivative action on the rate, is what flies on real spacecraft. There is
usually no integral term: in orbit the disturbance torques are tiny and
slowly varying, the steady-state error of a proportional-derivative law is
negligible at reasonable gains, and an integrator would only add the risk
of wind-up when the actuators saturate. What the simple law leaves out is
the actuator, which has a limited torque and a limited momentum, and the
fact that the attitude and the rate are not known but estimated. The first
is addressed in the chapters on guidance, navigation and control, the
second in the next chapter.

## In the code

The model is in `dynamics/attitude/rigid_body.py`.
`quaternion_kinematics_symbolic` returns the four kinematic equations and
`euler_rotational_equations_symbolic` the three dynamic ones, for a body
described in its principal axes. `rigid_body_attitude_dynamics_symbolic`
couples them in a system of seven states, with the three moments of inertia
and the three components of the torque as runtime parameters; a zero torque
gives free tumbling.

`pd_pointing_attitude_dynamics_symbolic` builds the control law into the
same system: the commanded quaternion and the two gains are runtime
parameters, and the torque is computed inside the equations from the
current state. This version is the plain law, without the sign correction
against unwinding and without actuator limits, and is meant for studying
the response of the ideal loop.

## In the notebook

{doc}`../examples/10_attitude_pointing_control/attitude_pointing_control`
first lets a body with three different moments of inertia tumble freely and
verifies that the kinetic energy, the angular momentum and the norm of the
quaternion are conserved to machine precision. It then closes the loop and
drives an initial error of 60 degrees to a small fraction of a degree,
showing the angular rate decaying with it.

## References

1. F. L. Markley, J. L. Crassidis, *Fundamentals of Spacecraft Attitude
   Determination and Control*, Springer, 2014.
2. B. Wie, *Space Vehicle Dynamics and Control*, 2nd edition, AIAA, 2008.
3. B. Wie, P. M. Barba, "Quaternion feedback for spacecraft large angle
   maneuvers," *Journal of Guidance, Control, and Dynamics*, 8(3), 1985.
