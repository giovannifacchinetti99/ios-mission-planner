# The closed-loop simulator

Every model of the previous parts was validated on its own. A relative
trajectory was flown with the true state known exactly, an attitude
controller applied an ideal torque, a filter was fed measurements generated
for the purpose. Each of those tests answers a question about one model.
None of them answers the question of the mission, which is whether the
estimates, the commands and the hardware, with all their imperfections
acting at the same time, bring the docking port of the deputy to the port of
the chief inside the limits that the mechanism accepts.

This part of the book puts the models in a single loop. This chapter
describes the loop and the two ends of it that touch physics: the truth
model, which plays the role of reality, and the models of the sensors and of
the actuators. The two chapters that follow describe what sits between
them, the navigation and the guidance and control.

## The loop

A spacecraft does not know its state. It has sensors, which observe the
state with errors; a navigation function, which turns the observations into
estimates; a guidance function, which decides from the estimates where to go
and where to point; a control function, which turns that decision into a
force and a torque; and actuators, which deliver the force and the torque
as well as their limits allow. The force and the torque change the true
state, the sensors observe the new state, and the cycle repeats.

```{figure} ../figures/gnc_loop.png
:name: fig-gnc-loop
:width: 100%

The loop of the simulation. Only the truth model knows the true state; the
on-board functions, in the lower row, see it through the sensors and act on
it through the actuators.
```

The simulation reproduces this cycle with a fixed step of one second. At
each step the sensor models read the true state and add their errors, the
on-board functions run on those readings, the actuator models turn the
commands into the force and the torque actually produced, and the truth
model advances by one step with that force and torque held constant. The
distinction between the truth and what the on-board software believes is
kept everywhere. The guidance never reads the true position, the navigation
is told the force that was commanded and not the one that was produced, and
the geometry of the chief known on board is the nominal one even when the
true one differs.

## The truth model

The truth model integrates, together, the absolute orbits of the two
spacecraft and the mass, the attitude, the body rate and the wheel momentum
of the deputy: 23 states in all.

Each spacecraft moves under two-body gravity, the $J_2$ term and
atmospheric drag, the models of {doc}`../foundations/two_body` and
{doc}`../foundations/perturbations`, each with its own ballistic
coefficient. The relative motion is not modelled: it is the difference of
two absolute motions. It therefore contains everything that the linear
model of {doc}`../relative/hcw` leaves out, the curvature of the orbit, the
small eccentricity that $J_2$ induces, and the differential drag of
{doc}`../relative/perturbed`. This matters because the on-board functions
use the linear model, and a simulation that used the same model for the
truth would test them against their own assumptions.

The deputy also feels its thrust. The thrusters are fixed to the body, so
the force is known in body axes, $\vect F_b$, and reaches the orbit through
the attitude:

$$
\ddot{\vect r}_d = \vect a_{\mathrm{grav}} + \vect a_{\mathrm{drag}}
+ \frac{1}{m}\,R^{\mathsf T}\vect F_b , \qquad
\dot m = -\frac{\sum_i |F_{b,i}|}{g_0 I_{sp}} ,
$$

with $R$ the rotation matrix from the inertial to the body frame. This is
the first coupling between rotation and translation: a pointing error turns
into a thrust direction error.

The attitude obeys the kinematics and Euler's equations of
{doc}`../attitude/dynamics`, extended with reaction wheels. A wheel is a
rotor that the motor accelerates in one direction, and the reaction turns
the body in the other. If $\vect h_w$ is the angular momentum stored in the
wheels, expressed in body axes, and $\vect\tau_w$ the torque that the wheels
apply to the body, the total angular momentum $I\vect\omega + \vect h_w$
changes only because of external torques:

$$
I\dot{\vect\omega} = \vect\tau_w + \vect\tau_{\mathrm{ext}} +
\vect\tau_{gg} - \vect\omega\times\left(I\vect\omega + \vect h_w\right),
\qquad \dot{\vect h}_w = -\vect\tau_w .
$$

The wheels move momentum between themselves and the body and cannot create
it. This is the property that the tests check, and it has a practical
consequence that will appear below: a persistent external torque fills the
wheels, and only another external torque can empty them.

The gravity-gradient torque is the external torque that is always present.
Gravity is slightly stronger on the side of the body nearer to the Earth,
and the difference produces a torque that depends on how the inertia is
distributed with respect to the local vertical,

$$
\vect\tau_{gg} = \frac{3\mu}{r^5}\;\vect r_b\times\left(I\,\vect r_b\right),
$$

where $\vect r_b$ is the position of the deputy in body axes. It vanishes
when a principal axis is along the vertical. For the deputy of the
reference mission it is of the order of $10^{-5}$ N m, small against the
torque of the wheels but acting all the time.

The 23 equations are integrated with the same Taylor integrator as the rest
of the book. The force, the mass flow and the torques are parameters of the
integrator, changed between steps and constant within a step, which is
exactly how a digital controller acts on a continuous system.

## Sensors

Three sensors feed the navigation.

The gyro measures the body rate with a white noise and a bias, the model of
{doc}`../attitude/estimation`. The bias performs a random walk, so it can
never be calibrated once and for all. The star tracker measures the
inertial attitude with a small random rotation error.

The relative sensor stands for a camera with a laser range finder that
looks at a target pattern next to the docking port of the chief. It returns
the distance $\rho$ of the target and its direction $\hat{\vect u}$ in body
axes. Its errors have a specific shape. The range error is a fraction of
the distance, with a floor at short range,

$$
\sigma_\rho = \max\left(\sigma_{\rho,0},\; k_\rho\,\rho\right),
$$

and the direction error is a fixed angle $\sigma_\theta$. In metres, both
shrink as the deputy closes in: the sensor is coarse at 5 km and accurate
at the port, which is the opposite of what a fixed-accuracy position
measurement would give and is what makes a docking possible. The sensor has
a conical field of view and returns nothing when the target is outside it.
This is the second coupling between rotation and translation: if the
attitude control lets the chief leave the field of view, the relative
navigation goes blind.

A mounting error of the sensor rotates every measured direction by the same
small angle, unknown to the on-board software. Its effect in metres is the
angle times the distance, so it too vanishes at the port.

## Actuators

The thrusters are on-off devices of fixed thrust, one set for each
direction of each body axis. A thruster cannot be throttled. It can only be
opened for a time, and not for less than a minimum time, which defines the
smallest impulse it can deliver, the minimum impulse bit. The modulator
that drives them accumulates, axis by axis, the impulse requested by the
controller. When the accumulated request reaches the minimum impulse bit
the thruster fires for the time needed to deliver it, up to the whole step.
A request too small to fire is not discarded, it stays in the accumulator
until later requests add up to a pulse. Over time the impulse delivered
equals the impulse requested, to within one bit.

The force actually produced differs from the intended one in three ways: a
scale error on each axis, a misalignment of the whole set, both fixed for a
given vehicle and unknown on board, and a random variation from pulse to
pulse. If the thrust lines do not pass exactly through the center of mass,
every firing also produces a torque, $\vect\tau = -\vect c\times\vect F$
for an offset $\vect c$ of the center of mass.

The reaction wheels are limited in two ways. The motor limits the torque,
and the maximum speed of the rotor limits the momentum that can be stored.
A wheel at its momentum limit cannot push further in the same direction,
and the axis it serves is no longer controlled.

The two actuators meet in the momentum management. The torque produced by
a center of mass offset of a few millimetres is small, but it acts in the
same direction for the whole duration of a burn, and the wheels that
oppose it accumulate momentum. Left alone they saturate, and the attitude
is lost. The remedy is the one the conservation of angular momentum
requires: an external torque. When the momentum of a wheel exceeds a
fraction of its capacity, a pair of thrusters firing in opposite directions
applies a pure torque to the body, and the wheel is commanded to apply the
opposite torque at the same instant. The body feels no net torque, and the
wheel is slowed down. The couple stays on until the momentum has fallen
below a lower threshold, so that it does not switch on and off around a
single value.

The cancellation is a feedforward, and it is not a detail. The on-board
software commands the couple, so it knows its torque without having to
measure anything. If the wheel were left to oppose the couple through the
attitude feedback alone, the attitude would have to drift until the
feedback produced that torque: with the gains of the reference deputy,
several degrees for a couple of 16 mN m, every time an unloading starts,
including during the final approach.

The torque of the couple has to be sized. It must be larger than the
largest disturbance torque that the thrusters can produce, their thrust
times the largest credible offset of the center of mass, or during a long
burn the wheel fills faster than it is emptied. And it must be smaller than
the torque of the wheel, which has to oppose it. For the reference deputy,
2 N of thrust and an offset of up to 8 mm give 16 mN m, against 20 mN m of
the wheels.

## The two docking ports

What must meet at the end are the two ports, not the two centers of mass.
The port of the chief is at a known position on its body, with an outward
axis; the port of the deputy is at a position $\vect p_b$ in its body axes.
The position and the velocity of the deputy port with respect to the chief
port are therefore

$$
\vect\Delta = \vect\rho + R_{lb}\,\vect p_b - \vect p_c , \qquad
\dot{\vect\Delta} = \dot{\vect\rho} + R_{lb}\left(\vect\omega_{rel}\times
\vect p_b\right),
$$

where $R_{lb}$ rotates from the body of the deputy to the LVLH frame and
$\vect\omega_{rel}$ is the angular velocity of the deputy with respect to
that frame. A deputy whose center of mass is exactly on the axis but which
is rotated by a small angle presents its port off the axis, and a deputy
that is turning moves its port sideways.

The simulation ends when the deputy port crosses the plane of the chief
port. The state of the two ports at that instant is resolved into the
quantities that a docking mechanism specifies: the lateral offset and the
lateral speed across the axis, the closing speed along it, the angle
between the two axes, the roll about the axis and the relative angular
rate. Each is compared with its limit. The closing speed has a lower limit
as well as an upper one, because the mechanism needs a minimum speed to
capture.

The simulation can end in three other ways. If a collision avoidance
maneuver is commanded, described in {doc}`guidance_control`, the run ends
after the deputy has moved away. If the port plane is crossed in a phase
other than the final approach, the run is a collision. And if neither
happens within the maximum duration, it is a timeout.

## Injected failures

A loop that only flies nominal missions says little about safety, so the
simulation can break parts of it on purpose, each from a start time and for
a duration. Two failures act on hardware. The relative sensor can stop
returning measurements, and a thruster can stay open, adding a constant
force along a body axis that the navigation does not know about.

Three more act on the software, and they are the dangerous kind, because
the main chain keeps working with a wrong picture of the world without
knowing it. The first is a constant error on the velocity estimate of the
relative navigation, the kind of failure behind the collision of the DART
spacecraft with its target in 2005. Guidance, control and the abort logic all see the biased
estimate, while the filter underneath keeps tracking the truth, so the
position measurements cannot remove the error. A bias that makes the deputy
believe it closes more slowly than it does is the worst case: the guidance
accelerates to restore the planned speed, and the check on the closing
speed, which compares the same biased estimate with its limit, stays quiet
while the true speed exceeds it.

The second is a late brake at a hold point. Once the estimated distance to
the chosen hold falls below a threshold, the part of the command that slows
the deputy along the approach axis is suppressed for a given delay, and the
deputy coasts past the point where it should have stopped. At the last hold
the closing speed is a few centimetres per second, so a delay of a minute
carries it a couple of metres closer to the chief than planned.

The third is an error on the attitude estimate used by the attitude control
and by the conversion of the force command into the body axes. The
controller steers the true attitude away from the commanded one by that
error, and the thrusters fire in directions rotated by the same amount. If
the error appears in a single step, the controller sees a sudden pointing
error and the abort logic reacts. If it grows over a ramp of a minute or
two, as an undetected drift would, the controller follows it and nothing on
board notices; only its consequences, a lateral drift or a target leaving
the field of view of the relative sensor, eventually do.

## A second layer: the supervisor

Crewed and high-value vehicles do not rely on the main chain alone. The
Automated Transfer Vehicle carried a monitoring and safety unit, separate
from its main computer, with the authority to stop an approach. The
investigation of DART drew the same lesson: collision avoidance should rely
on information independent of the navigation that guides the approach.

The simulation lets such a layer be plugged into the loop as a supervisor,
a function called once per step after the main chain has computed its force
command and before the thrusters execute it. It receives the time, the true
relative state of the deputy, its true attitude with respect to the LVLH
frame, and the force about to be commanded, and nothing else: no estimate,
no covariance, no state of the guidance or the mission manager. A
supervisor meant to be independent of the main navigation has to build its
own knowledge from those truths, typically by simulating its own sensor
with its own errors, and the interface makes it impossible to cheat by
peeking at the main estimate.

A supervisor can do two things. It can replace the force command, which
then goes to the same thrusters with the same limits and errors: setting
some components to zero inhibits thrusters that would fire in a dangerous
direction, and a command of its own flies its own avoidance maneuver. And it
can ask the main chain to abort, which makes the mission manager start its
collision avoidance maneuver from the next step, whatever the phase. Its
decisions are recorded among the events of the run whenever they change.
Without a supervisor and without the new failures, the simulation produces
exactly the same results as before they were introduced. The chapter
{doc}`../supervision/failures` measures what each failure does to the
approach and builds a minimal supervisor on this interface.

## In the code

The truth model is in `sim/truth.py`. `docking_truth_dynamics_symbolic`
builds the 23 equations as a symbolic system, reusing the acceleration and
kinematics functions of the earlier chapters, and `TruthModel` wraps a
persistent integrator of that system: `reset` sets the state,
`set_controls` the force, the mass flow and the torques of the next step,
and `step` advances it. The integrator is compiled once per process and
copied for every new `TruthModel`, so that a campaign of many runs pays the
compilation once.

The sensor models are in `gnc/sensors.py`: `Gyro`, `StarTracker` and
`RelativeSensor`, each with a `measure` method that takes the true quantity
and a random generator. The actuator models are in `gnc/actuators.py`:
`ThrusterSet` implements the modulator and the execution errors,
`ReactionWheels` the torque and momentum limits, and `MomentumManager` the
unloading with thruster couples.

`sim/docking.py` holds the geometry of the ports. `port_state` computes the
relative state of the two ports from the relative state of the centers of
mass and the attitude, and `assess_contact` compares it with the limits of
`DockingLimits`.

`sim/config.py` collects every number that defines a scenario in
dataclasses with defaults that describe the reference mission, grouped
under `SimulationConfig`: the environment, the chief, the deputy with its
sensors, thrusters and wheels, the navigation, the control, the guidance,
the contact limits, the initial conditions and the failures to inject.
`simulate` in `sim/simulator.py` runs the loop for a `SimulationConfig` and
returns a `SimulationResult` with the time histories of the true and
estimated quantities, the list of events, the outcome and the assessment of
the contact.

The failures are the fields of `FailureConfig`. The supervisor is the
optional argument `supervisor` of `simulate`; `sim/supervisor.py` defines
what it receives, a `SupervisorSample` of read-only arrays, and what it may
return, a `SupervisorDecision`. `run_monte_carlo` accepts a
`supervisor_factory`, a function of the seed that builds a fresh supervisor
for each run, so that a campaign can be flown with and without the second
layer.

## In the notebook

{doc}`../examples/15_closed_loop_docking/closed_loop_docking` flies the
reference mission from a hold point 5 km behind the chief to the contact
of the ports and examines each part of the loop on the same run: the
trajectory, the navigation error against its own uncertainty, the closing
speed and the propellant by phase, the attitude and the wheels, and the
conditions at contact. It then removes the mid-course corrections to show
what they are for, injects the loss of the relative sensor to show the
avoidance maneuver, and offsets the center of mass by 3 mm to show the
saturation of the wheels and its cure.

## References

1. W. Fehse, *Automated Rendezvous and Docking of Spacecraft*, Cambridge
   University Press, 2003.
2. F. L. Markley, J. L. Crassidis, *Fundamentals of Spacecraft Attitude
   Determination and Control*, Springer, 2014.
3. B. Wie, *Space Vehicle Dynamics and Control*, 2nd edition, AIAA, 2008.
4. NASA, *Overview of the DART Mishap Investigation Results*, 2006,
   https://www.nasa.gov/pdf/148072main_DART_mishap_overview.pdf.
4. International Docking System Standard (IDSS), *Interface Definition
   Document*.
