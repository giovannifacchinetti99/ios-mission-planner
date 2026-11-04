# Propellant budget and maneuver design

The previous chapters design and check a trajectory. This chapter turns a
trajectory into a number a mission can actually be built around: how much
propellant it costs, with enough margin to survive the parts of the design
that are still uncertain, and what the standard closed-form maneuvers that
feed into that number look like.

## The rocket equation

Code: {py:func}`mass_fraction
<ios_mission_planner.budget.propellant.mass_fraction>`,
{py:func}`propellant_mass
<ios_mission_planner.budget.propellant.propellant_mass>`,
{py:func}`initial_mass <ios_mission_planner.budget.propellant.initial_mass>`
in `src/ios_mission_planner/budget/propellant.py`.

A rocket expels propellant at an exhaust velocity $v_e = I_{sp} g_0$, where
$I_{sp}$ is the specific impulse and $g_0$ is standard gravity by definition
rather than the local gravitational acceleration. Conservation of momentum
between the vehicle and the expelled mass, integrated over a burn, gives the
Tsiolkovsky rocket equation relating the total delta-v to the ratio of
initial (wet) mass $m_0$ to final (dry) mass $m_f$:

$$
\Delta v = I_{sp}\,g_0 \ln\frac{m_0}{m_f}.
$$

Solving this for the propellant mass fraction gives `mass_fraction`,

$$
\frac{m_0 - m_f}{m_0} = 1 - e^{-\Delta v / (I_{sp} g_0)},
$$

and `propellant_mass` simply multiplies this fraction by a given $m_0$.
`initial_mass` inverts the same relation the other way, solving for the wet
mass needed to deliver a given delta-v on top of a fixed dry mass $m_f$,
$m_0 = m_f\, e^{\Delta v/(I_{sp}g_0)}$, which is the more useful direction
when the payload and structure mass are already fixed and the open question
is how much propellant (and therefore how much wet mass) a mission's total
delta-v budget actually costs.

## Margined budget lines

Code: {py:class}`BudgetLine <ios_mission_planner.budget.margins.BudgetLine>`,
{py:func}`total_delta_v <ios_mission_planner.budget.margins.total_delta_v>`,
`MATURITY_MARGIN` in `src/ios_mission_planner/budget/margins.py`.

A delta-v budget is never just the sum of nominal maneuver costs: each line
item carries its own uncertainty, and the less validated a maneuver's
design, the more margin it should carry on top of its nominal value.
`BudgetLine` pairs a nominal delta-v with a fractional margin, so that
`margined_delta_v` is `delta_v * (1 + margin)`, and `MATURITY_MARGIN`
provides reasonable defaults in the spirit of ECSS and ESA
concurrent-design-facility budgeting practice: 2% for a flight-proven
maneuver, 10% for one that has been analyzed and validated (for example,
checked against nonlinear propagation elsewhere in this project), and 20%
for one that is still preliminary, resting on assumptions the project has
not yet derived for itself. `total_delta_v` sums the already-margined line
items and then applies one more fractional reserve on top of that subtotal,
representing the mission-level contingency that covers whatever the
per-line margins individually do not.

## Classic impulsive maneuvers

Code: {py:func}`hohmann_transfer
<ios_mission_planner.dynamics.orbital.maneuvers.hohmann_transfer>`,
{py:func}`plane_change_dv
<ios_mission_planner.dynamics.orbital.maneuvers.plane_change_dv>`,
{py:func}`combined_plane_change_dv
<ios_mission_planner.dynamics.orbital.maneuvers.combined_plane_change_dv>` in
`src/ios_mission_planner/dynamics/orbital/maneuvers.py`.
Reference: Vallado, *Fundamentals of Astrodynamics and Applications*.

A Hohmann transfer moves between two coplanar circular orbits of radii $r_1$
and $r_2$ along a single connecting ellipse of semi-major axis $a_t =
(r_1+r_2)/2$, with two tangential burns, one at each end. Each burn's size
follows from the vis-viva equation, the Cartesian form of the specific
energy conservation already used in the absolute dynamics chapter,
$v^2 = \mu(2/r - 1/a)$, evaluated once on the circular orbit and once on the
transfer ellipse at the same radius:

$$
\Delta v_1 = \sqrt{\mu\left(\frac2{r_1}-\frac1{a_t}\right)} -
\sqrt{\frac\mu{r_1}}, \qquad
\Delta v_2 = \sqrt{\frac\mu{r_2}} -
\sqrt{\mu\left(\frac2{r_2}-\frac1{a_t}\right)},
$$

and the transfer time is half the transfer ellipse's own orbital period,
$\pi\sqrt{a_t^3/\mu}$. This is exactly `hohmann_transfer`; the same function
also gives a deorbit burn directly, since dropping a circular orbit's
perigee to a chosen lower radius is simply this transfer's first burn,
without a second burn to circularize, because atmospheric drag is left to
finish the job instead.

A pure plane change rotates the velocity vector by an angle $\delta i$
without changing its magnitude $v$. The two velocity vectors, before and
after, form an isosceles triangle with two sides of length $v$ and an
included angle $\delta i$, so the delta-v is the triangle's base,
$\Delta v = 2v\sin(\delta i/2)$, which is exactly `plane_change_dv`; the
$\sin$ of a half-angle rather than the angle itself is why plane changes get
disproportionately expensive as $\delta i$ grows; doubling a small angle
roughly doubles the cost, but a large angle costs far more than
proportionally. `combined_plane_change_dv` generalizes this to a single
burn that changes both speed, from $v_1$ to $v_2$, and direction at the
same time, by the law of cosines on the same velocity triangle,
$\Delta v = \sqrt{v_1^2+v_2^2-2v_1v_2\cos\delta i}$. Combining a speed
change and a plane change into one burn is never more expensive than doing
them as two separate burns, since the lone-speed-change and lone-plane-change
cases above are both degenerate special cases of this same triangle.

## Continuous low-thrust propulsion

Code: {py:func}`low_thrust_dynamics_symbolic
<ios_mission_planner.dynamics.orbital.low_thrust.low_thrust_dynamics_symbolic>`,
{py:func}`low_thrust_tangential_dynamics_symbolic
<ios_mission_planner.dynamics.orbital.low_thrust.low_thrust_tangential_dynamics_symbolic>`
in `src/ios_mission_planner/dynamics/orbital/low_thrust.py`.

The maneuvers above assume an impulsive burn: all of a maneuver's delta-v
applied instantaneously, a good approximation whenever the burn is short
compared to the orbital period. Electric propulsion typically is not short:
thrust levels in the tens to hundreds of millinewtons mean a burn can span
many orbits, during which gravity continuously turns the velocity vector
underneath a fixed thrust direction. `low_thrust_dynamics_symbolic` models a
thrust held along a fixed inertial direction, which is only a good
approximation for the short-burn case; `low_thrust_tangential_dynamics_symbolic`
instead steers the thrust continuously along $\pm\vect v/|\vect v|$,
recomputed at every instant from the velocity state itself, and stays exact
for a burn of any duration. Both still deplete mass through the same
differential form of the rocket equation seen above, $\dot m = -F/(I_{sp}
g_0)$, for a thrust force $F$.

Comparing a long continuous burn against an equivalent impulsive maneuver
surfaces a result that is easy to get wrong by comparing the wrong
quantity. Spending the same propellant mass through the same engine changes
the orbit's specific energy, and hence its semi-major axis, by the same
amount regardless of how the thrust was steered. This is a direct
consequence of the rocket equation above: it fixes a delta-v budget, and
delta-v is what sets the energy change, whether that delta-v is delivered
in one instant or spread continuously over days. What is not the same is
the shape the orbit ends up with. An impulsive burn placed at one point
produces an eccentric transfer orbit whose opposite side drops immediately
to the intended target radius; a continuous tangential burn instead spirals
the whole orbit down nearly circularly, reaching the same semi-major axis
but never bringing any single point as low as the impulsive transfer orbit's
perigee does. For a deorbit specifically, this means matching a maneuver's
energy is not the same as matching its purpose: a continuous burn sized to
match an impulsive deorbit's delta-v can still fail to lower perigee into
the atmosphere, and getting the same outcome out of continuous thrust needs
a different strategy (concentrating thrust arcs near one point in the orbit,
or stopping once enough eccentricity has built up) rather than just thrusting
continuously for the equivalent delta-v.

## See it in action

The worked example for this chapter is {doc}`../examples/mission_budget`,
which assembles a full rendezvous-to-disposal mission as a margined
delta-v budget, including a plane-change sensitivity case, and runs the
impulsive-versus-continuous-thrust comparison above on the mission's
deorbit burn.
