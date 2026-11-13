# Guidance and control of the approach

With an estimate of the relative state available from {doc}`navigation`,
two functions remain between the sensors and the actuators. The guidance
decides what the deputy should do: which point to reach, along which path,
at which speed, pointing where. The control computes the force and the
torque that make it happen. This chapter follows the approach from the
first hold point to the contact, and for each leg describes the guidance
law that defines it and the controller that flies it.

## The profile of the approach

An approach to a target is flown in steps. The deputy moves from one hold
point to the next, and at each one it stops and can wait: for its
navigation to converge, for a go from the ground, for the lighting to be
right. The hold points are on the V-bar, behind the chief, because a point
at rest on the V-bar is an equilibrium of the relative motion: the
equations of {doc}`../relative/hcw` give zero acceleration there, and the
deputy stays without thrust. A point at rest anywhere else needs a
continuous acceleration, $3n^2x$ in the radial direction for a point at a
radial offset $x$, which for 100 m amounts to 2 m/s per orbit.

```{figure} ../figures/approach_profile.png
:name: fig-approach-profile
:width: 100%

The profile of the reference approach, not to scale. The transfer between
the first two hold points is a free arc between two burns; from the second
hold point on, the deputy is driven along the V-bar inside the approach
corridor.
```

The reference mission uses three hold points, at 5 km, 500 m and 20 m
between the ports. The legs between them are flown in different ways, for a
reason that has to do with distance. Far from the chief, time and
propellant matter and safety is ensured by the geometry: the deputy can
coast on a free trajectory. Close to the chief nothing can be left to a
free trajectory that would be wrong if a burn were slightly off, and the
deputy is steered all the way.

## The far transfer: two impulses and their corrections

The leg from 5 km to 500 m is the two-impulse transfer of
{doc}`../maneuvers/impulsive`: a first burn puts the deputy on a free arc
that reaches the target point after a chosen time, and a second burn stops
it there. The first burn is computed with the targeting in the linear
model, from the estimated state.

Two things separate that computation from what happens. The thrusters take
time: at 12.5 mm/s², a burn of 1.5 m/s lasts two minutes, during which the
deputy keeps moving. Planning the burn as if the impulse were applied at
its midpoint, by propagating the state over half the burn before solving
the targeting, removes most of this error. And the transfer is flown in the
real dynamics with real thrusters, not in the model with ideal ones. What
remains is corrected on the way: at a few instants during the coast the
targeting is solved again, from the current estimate, for the same arrival
point and the remaining time, and the small difference is burned. A
correction made early is cheap, because a small change of velocity has a
long time to act; the same error left to the arrival costs many times
more.

The time of flight is 0.45 orbits. The targeting has a singularity at half
an orbit for the out-of-plane motion, where no impulse can change the
cross-track position at arrival, and staying below it keeps every
correction well defined as the remaining time shrinks.

## Holding a point: the linear quadratic regulator

At a hold point the deputy must stay near a reference state $\vect x_r$.
The control is a state feedback around the feedforward that makes the
reference an equilibrium,

$$
\vect u = \vect u_{ff} - K\left(\hat{\vect x} - \vect x_r\right),
$$

and the gain is the one of the linear quadratic regulator: for the discrete
model $\vect x_{k+1} = \Phi\vect x_k + \Gamma\vect u_k$, the feedback that
minimizes

$$
J = \sum_k \tilde{\vect x}_k^{\mathsf T} Q\,\tilde{\vect x}_k + \vect
u_k^{\mathsf T} R\,\vect u_k
$$

is $K = (R + \Gamma^{\mathsf T}P\Gamma)^{-1}\Gamma^{\mathsf T}P\Phi$, with
$P$ the solution of the discrete algebraic Riccati equation

$$
P = Q + \Phi^{\mathsf T}P\Phi - \Phi^{\mathsf T}P\Gamma\left(R +
\Gamma^{\mathsf T}P\Gamma\right)^{-1}\Gamma^{\mathsf T}P\Phi .
$$

The weights are chosen by Bryson's rule: each state and each input is
divided by the deviation considered acceptable, so that $Q$ and $R$ state
in physical units how tightly the point must be held and how hard the
thrusters may work for it. The tolerance grows with the distance from the
chief. A hold at 5 km is kept loosely, within tens of metres, because the
navigation is noisy there and a tight regulator would spend propellant
following the noise; near the port the tolerance is centimetres.

## The forced approach: glideslope

From the second hold point on, the deputy follows a straight line along the
V-bar. The reference is a point that moves along the line with a speed
that decreases with the distance $d$ still to cover,

$$
v(d) = \min\left(v_{max},\;\max\left(v_{min},\;\frac{d}{\tau}\right)\right).
$$

This is the glideslope law. Far away the speed is constant; closer in it is
proportional to the distance, so that the time needed to stop is always the
same, $\tau$, and the distance decreases exponentially; at the end a
minimum speed makes the arrival happen in finite time. The speed of the
reference changes with a limited acceleration, so that it never asks for a
step the thrusters could not follow.

A straight line along the V-bar is not a free motion, and the equations
say what it costs. A deputy moving at speed $v$ along the direction of
flight feels the Coriolis acceleration $2nv$ in the radial direction. The
thrusters must cancel it for as long as the motion lasts, and since the
time is the distance divided by the speed,

$$
\Delta v_{\mathrm{Coriolis}} = \int 2nv\,\mathrm dt = 2n\,d ,
$$

whatever the speed. Covering 480 m on the V-bar costs 1.06 m/s against the
Coriolis acceleration alone. Speeding up and slowing down add twice the
cruise speed. Going slower saves only this second part, and takes
proportionally longer.

## Constraints: model predictive control

A regulator has one rule, proportional to the error, and no notion of a
limit. Near the chief there are limits that must hold. The thrust is
bounded. The deputy must stay inside an approach corridor, a cone around
the port axis with its vertex at the port, so that the sensors of the chief
and the people watching the approach know where it will be. And the closing
speed must decrease with the distance, so that a failure always leaves time
to stop.

Model predictive control handles limits as part of the problem. At each
control period it predicts the motion over the next $N$ steps and solves

$$
\begin{aligned}
\min_{\vect u_0,\dots,\vect u_{N-1}}\quad & \sum_{k=0}^{N-1}\left[\tilde{\vect
x}_k^{\mathsf T} Q\,\tilde{\vect x}_k + \tilde{\vect u}_k^{\mathsf T}
R\,\tilde{\vect u}_k\right] + \tilde{\vect x}_N^{\mathsf T} P\,\tilde{\vect
x}_N + \rho\sum_k \vect 1^{\mathsf T}\vect s_k \\
\text{subject to}\quad & \vect x_{k+1} = \Phi\vect x_k + \Gamma\vect u_k,
\qquad \vect x_0 = \hat{\vect x}, \\
& |\vect u_k| \le u_{max}, \\
& C\vect x_k \le \vect d + \vect s_k, \qquad \vect s_k \ge 0 ,
\end{aligned}
$$

where the tilde denotes the deviation from the reference trajectory and
from its feedforward. Only the first input $\vect u_0$ is applied. At the
next period the horizon moves forward by one step and the problem is
solved again from the new estimate, which is what turns an open-loop
optimization into a feedback law.

Three choices in this formulation deserve a word. The terminal cost uses
the matrix $P$ of the linear quadratic regulator with the same weights. It
stands for the cost of the motion beyond the horizon, and it makes the
predictive controller coincide with the regulator whenever no constraint is
active: the two are the same law until a limit is reached.

The thrust limit is a hard constraint, because it is a physical fact. The
constraints on the state are softened by the slack variables $\vect s_k$,
which allow a violation at a price $\rho$. A hard constraint on the state
would make the problem infeasible, and leave the controller without an
answer, as soon as a disturbance or a navigation error placed the state
outside it. The price must be high enough that violating a constraint is
never a bargain. If it is lower than what the tracking cost gains from the
violation, the controller trades the constraint away; this is a mistake
that is easy to make and that a test with a large initial error exposes.

The constraints on the state are all linear, so the problem is a quadratic
program. The corridor is a cone, approximated by four planes: for a lateral
direction $\hat{\vect e}$, a port axis $\hat{\vect a}$ and a distance $d$
along the axis, $\pm\hat{\vect e}\cdot\vect r \le r_0 + d\tan\gamma$. The
closing speed is bounded by a value that grows with the distance,
$v_c \le v_0 + k\,d$. A last plane keeps the deputy on its side of the port
until the final approach begins.

The matrices of the quadratic program do not change from one period to the
next, only the vectors do. The solver, OSQP, factorizes the problem once
and starts each solution from the previous one, so that a solve takes a few
milliseconds.

## The final approach

From the last hold point the reference moves toward the port at a constant
speed, chosen in the middle of the band that the docking mechanism accepts,
and the predictive controller tracks it with weights tightened to
centimetres. The reference continues past the port: the contact is meant to
happen at the commanded speed, not after a braking.

## Pointing: the attitude command

While the translation is steered, the attitude has its own guidance. Far
from the chief the sensor must look at it: the boresight axis of the deputy
is pointed along the estimated line of sight, and a second axis is kept
along the orbit normal. Near the chief the requirement changes, and the
port axis of the deputy is aligned with the port axis of the chief.

In both cases the command is built with the TRIAD construction of
{doc}`../attitude/pointing`, with one difference from that chapter: the
two directions are first rotated to the inertial frame. The command is
then a true inertial attitude, and it contains the rotation of once per
orbit that a spacecraft facing the chief from the V-bar must perform. The
command is recomputed at every step, and its rate, obtained from two
successive commands, is given to the controller together with it.

The control law is the quaternion feedback of
{doc}`../attitude/dynamics`, completed for use with real wheels:

$$
\vect\tau = -k_p\,\mathrm{sat}\left(\vect q_e\right) - k_d\left(\vect\omega -
\vect\omega_{c}\right) + \vect\omega\times\left(I\vect\omega + \vect
h_w\right).
$$

The error quaternion is taken with the sign that gives the shorter
rotation, which removes the unwinding that the simpler law allowed. The
rate term damps the motion with respect to the rate of the command, so a
rotating command is tracked without a steady lag. The last term cancels
the gyroscopic torque of the body and of the wheels. And the proportional
term is limited in magnitude, so that for a large error it balances the
rate term at a chosen rate: the body turns at that constant rate until the
error is small. Without this limit a wide slew accelerates the body until
the wheels are full. The gains follow from the natural frequency
$\omega_n$ and the damping ratio $\zeta$ wanted for the closed loop,
$k_p = 2I\omega_n^2$ and $k_d = 2\zeta\omega_n I$, the factor of two coming
from the fact that the vector part of the error quaternion is half the
error angle.

## Sequencing and the decision to abort

A mission manager ties the legs together. It keeps the current phase,
applies the guidance law and the controller that belong to it, and decides
when to move on: a hold ends when its minimum duration has passed and the
estimated state is within tolerance of the hold point, and the final
approach begins only if the go/no-go checks are passed.

During the approach it also watches for the conditions under which
continuing is no longer acceptable: the estimated position outside a
corridor wider than the one the controller respects, a closing speed well
above its limit, no relative measurement for too long, a navigation
uncertainty too large for the distance, an attitude error too large. All of
them are evaluated on board, from estimates. If one is violated, the
approach is abandoned with a collision avoidance maneuver: a single burn
away from the chief along the port axis. For an approach from behind this
is a retrograde burn. It lowers the orbit of the deputy, which first falls
back and down and then, being lower and faster, passes under the chief at a
distance of hundreds of metres and drifts ahead. The trajectory after the
burn needs no further thrust and no navigation to stay clear of the chief,
which is the passive safety of {doc}`../safety/safety`, here obtained on
purpose.

## In the code

`gnc/guidance.py` holds the guidance laws. `plan_transfer` computes the
first burn of a two-impulse transfer with the compensation for its
duration, `GlideslopeReference` is the moving reference of the forced
approach, with `horizon` returning the reference states and the feedforward
over the prediction horizon, and `pointing_quaternion` and `command_rate`
build the attitude command and its rate.

`gnc/control.py` holds the controllers. `cw_feedforward_acceleration` gives
the acceleration that follows a reference exactly, `dlqr` solves the
Riccati equation, `tracking_weights` applies Bryson's rule, `HoldController`
is the regulator of a hold point, `ApproachMPC` is the predictive
controller, and `attitude_gains` and `quaternion_feedback_torque` are the
attitude law.

`MissionManager` in `gnc/mission_manager.py` is the sequencer. It owns the
controllers, calls the guidance laws, checks the abort conditions and
returns the commanded acceleration at each step.

## In the notebooks

{doc}`../examples/14_final_approach_control/final_approach_control` builds
the controllers on the linear model, with the state known exactly. It shows
that a V-bar point is an equilibrium and an R-bar point is not, designs the
regulator and compares a loose and a tight hold, flies the glideslope and
measures its cost against $2nd$, compares the regulator and the predictive
controller on a case that starts outside the corridor, turns the commands
into thruster pulses to show the limit cycle that the minimum impulse bit
causes, and flies a wide slew with and without the limit on the rate.

{doc}`../examples/15_closed_loop_docking/closed_loop_docking` flies the
same controllers in the complete loop, and shows the sequence of phases,
the corrections of the transfer and the avoidance maneuver.

## References

1. W. Fehse, *Automated Rendezvous and Docking of Spacecraft*, Cambridge
   University Press, 2003.
2. B. D. O. Anderson, J. B. Moore, *Optimal Control: Linear Quadratic
   Methods*, Prentice Hall, 1990.
3. J. B. Rawlings, D. Q. Mayne, M. M. Diehl, *Model Predictive Control:
   Theory, Computation, and Design*, 2nd edition, Nob Hill Publishing,
   2017.
4. B. Stellato, G. Banjac, P. Goulart, A. Bemporad, S. Boyd, "OSQP: an
   operator splitting solver for quadratic programs," *Mathematical
   Programming Computation*, 12, 2020.
5. B. Wie, *Space Vehicle Dynamics and Control*, 2nd edition, AIAA, 2008.
