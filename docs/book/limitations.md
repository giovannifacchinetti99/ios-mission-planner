# Assumptions and limitations

Every model in this book is a simplification, and a result is only as good
as the assumptions behind it. This page collects them in one place, in the
order of the book, so that they can be checked against the mission at hand
before a number is trusted.

## Orbital dynamics

The gravity model is a point mass plus the $J_2$ term. Higher harmonics of
the gravity field, the attraction of the Sun and the Moon, and solar
radiation pressure are not modelled. For the relative motion of two
spacecraft a few kilometres apart over a few days in low Earth orbit these
are small compared with $J_2$ and with differential drag, but they are not
negligible for long-term predictions of an absolute orbit.

The atmosphere is a single exponential band that rotates rigidly with the
Earth. The reference density and the scale height are inputs, and the real
density varies by a factor of several with solar activity and by tens of
percent along one orbit. Drag results are meant to size an effect. The
ballistic coefficient is constant: its dependence on attitude is not
modelled.

## Relative motion

The Hill-Clohessy-Wiltshire model assumes a circular, unperturbed chief and
a separation small compared with the orbit radius. Its error grows with the
square of the separation, and at a few kilometres it already produces
errors of metres over a transfer, unless the curvature of the orbit is
corrected for.

The model for an eccentric chief is integrated numerically; no closed-form
transition matrix is provided for it, and the targeting and safety tools
use the circular model.

The transition matrix for the relative orbital elements includes only the
secular, first-order effect of $J_2$. Short-period oscillations are not
represented, so its predictions are to be compared with mean, not
osculating, relative elements. Differential drag is modelled as a
perturbation; no control law that uses it is implemented.

## Maneuvers and propulsion

Orbital maneuvers and two-impulse targeting assume impulsive burns.
Targeting is solved in the linear model, so a transfer designed with it and
flown in the nonlinear dynamics has a residual error that a later burn must
correct.

The thrust-arc model centers the arc on a direction fixed in inertial
space, which is valid while the line of apsides does not move during the
maneuver. The duration of an arc-limited maneuver is estimated from a
constant duty cycle, an estimate that degrades for arcs much narrower than
those used in the examples. Thrust is constant in magnitude and perfectly
aligned.

## Safety

The keep-out zone is an ellipsoid fixed in the LVLH frame; the actual shape
and attitude of the chief are not represented. Passive safety is evaluated
by propagating the failed state with the linear model for a given time, and
says nothing beyond that horizon.

The collision probability of the safety chapter propagates a Gaussian
initial uncertainty with the linear transition matrix and applies no
corrections along the way. It is a screening value, generally pessimistic
for a mission that navigates and corrects continuously. The station-keeping
cost is measured for one ellipse, one orbit and one correction scheme, and
is not optimized.

## Attitude

The spacecraft is a rigid body described in its principal axes. Flexible
appendages, propellant slosh and the cross products of inertia are not
modelled. The symbolic pointing model applies an ideal torque, without
actuator limits and without protection against unwinding.

The attitude determination and filter examples use synthetic direction
measurements, generated from the true attitude with added Gaussian noise;
no specific sensor is modelled, and outages or asynchronous measurements
are not considered. The chapter on pointing at the target computes the
commanded attitude in the LVLH frame treated as inertial, and holds the
command between updates.

## Guidance, navigation and control

The closed-loop simulation assumes a cooperative chief. It holds its
attitude fixed in its own LVLH frame, apart from a constant error, it does
not maneuver, and its orbit is known on board without error, so the LVLH
frame used by the on-board functions is the true one.

The relative sensor measures the range and the direction of a target on the
chief. It does not measure the relative attitude: the deputy aligns its port
with the nominal direction of the port of the chief, and any attitude error
of the chief appears in full as an angular misalignment at contact. Its
error model is a range error proportional to the distance and a fixed
angular error, with a mounting bias; no specific instrument is modelled,
and the lighting has no effect on its measurements unless the go/no-go on
lighting is enabled.

The on-board model is the linear model for a circular chief. The navigation
filter accounts for what that model neglects through its process noise,
tuned for the reference scenario, and the guidance corrects it with
mid-course burns; neither uses a more accurate model.

The thrusters are a set of independent on-off thrusters along the body
axes, with a minimum impulse bit, scale and alignment errors and a center
of mass offset. The plumes do not push on the chief or contaminate it, and
the thrust does not depend on how long a thruster has been firing. The
reaction wheels are ideal within their torque and momentum limits, and the
couples that unload them are pure torques. Both spacecraft are rigid:
propellant slosh and flexible appendages are not modelled.

The simulation ends at first contact. What happens afterward, the capture
and the dynamics of the mated spacecraft, is not simulated, and the limits
on the conditions at contact are representative values, not those of a
specific mechanism. Contact anywhere but between the two ports, a solar
array touching the chief for example, is not detected.

The collision avoidance maneuver is a single burn of fixed size along the
port axis. Its safety is verified on the cases shown, not proven for every
state from which it might be commanded.

## Mission analysis

The Monte Carlo campaigns draw the dispersed quantities from independent
normal distributions with assumed standard deviations, and inherit every
assumption of the simulation they repeat. A probability of success
estimated from them refers to those distributions and to those models; the
failures that they do not contain, a component that breaks for example,
are not in the estimate. The confidence interval quantifies the sampling
error of the campaign and nothing else.

The illumination analysis uses a low-precision position of the Sun, a
cylindrical shadow without penumbra, and a circular orbit whose node drifts
at the secular rate. The docking port is a flat surface with one normal:
the shadows that the chief casts on itself, the light reflected by the
Earth and the response of a specific camera are not modelled, and the
limits on the incidence and on the angle between the Sun and the camera
axis are scenario parameters.

The margins of the budget are defaults in the spirit of European practice,
not requirements of a specific project. The fast screening of the report
uses closed-form maneuvers and the linear relative model, does not budget
the correction burn that the nonlinear flight of the approach requires,
and applies to one mission architecture. Its verdict depends on thresholds
supplied in the scenario. The mission analysis report is as good as the
simulation, the campaign and the lighting analysis that it summarizes.

## Visualization

The viewer plays back the true state of a simulation and adds nothing to
it. The two spacecraft are schematic shapes with the right geometry of the
ports and of the sensor, not models of real vehicles, and the Earth is a
sphere without real geography. The page loads its 3D library from a public
content delivery network and needs a connection the first time it is
opened.

## The real case study

The reconstruction of the proximity event rests on public two-line element
sets, whose accuracy is of the order of a kilometre. The geometry and the
timing of the event are robust; distances of a few hundred metres and the
size of individual maneuvers are at the limit of what the data can
resolve.
