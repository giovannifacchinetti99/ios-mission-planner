# Safety

The chapters so far cover how a trajectory is designed and how it drifts
under perturbations. This chapter is about a different question: whether a
planned trajectory, and its failure modes, are acceptable. Unlike the
previous chapters, the tools here are not derived from a textbook or a
paper; they encode standard mission-design practice (keep-out geometry,
graceful failure, directional approach constraints, and probabilistic
collision risk) as plain, checkable functions rather than as a closed-form
theory with its own numbered equations. All four live under
`src/ios_mission_planner/safety/`.

## Keep-out zones

Code: {py:class}`KeepOutEllipsoid
<ios_mission_planner.safety.keepout.KeepOutEllipsoid>`,
{py:func}`check_trajectory
<ios_mission_planner.safety.keepout.check_trajectory>` in
`src/ios_mission_planner/safety/keepout.py`.

A keep-out zone is a region around the chief that no planned trajectory
should ever enter. `KeepOutEllipsoid(a, b, c)` represents it as an ellipsoid
in the chief's LVLH frame, with semi-axes along the radial, along-track, and
cross-track directions, so that a sphere of radius $r$ is just the special
case $a=b=c=r$. A relative position $(x,y,z)$ violates the zone when

$$
\left(\frac{x}{a}\right)^2 + \left(\frac{y}{b}\right)^2 +
\left(\frac{z}{c}\right)^2 < 1,
$$

which `KeepOutEllipsoid.normalized_radius` evaluates directly, and
`check_trajectory` applies along a whole sampled trajectory, returning
whether the zone was ever violated, a signed margin, and a per-sample
violation mask that locates exactly when. The margin is defined as $1 -
r_{\min}$, where $r_{\min}$ is the smallest normalized radius reached along
the trajectory, so a positive margin means the trajectory did enter the
zone (by that fraction of the relevant semi-axis), while a negative margin
means it stayed clear, by that much room to spare; the sign is easy to read
backwards on a first look, so it is worth keeping this convention in mind
when interpreting a result.

## Passive safety

Code: {py:func}`check_passive_safety
<ios_mission_planner.safety.passive_safety.check_passive_safety>` in
`src/ios_mission_planner/safety/passive_safety.py`.

A planned burn can simply fail to execute, and a passively safe design is
one where that failure stays harmless long enough to be noticed and reacted
to. `check_passive_safety` takes the relative state right before a planned
burn, coasts it forward as a free ballistic trajectory for a given duration
using the true Hill equations from the relative dynamics chapter, exactly as
if the burn had never fired, and runs the result through the same
`check_trajectory` keep-out check. The burn counts as passively safe when
that ballistic coast stays clear of the zone for the whole duration. This
is a property of the trajectory design itself, evaluated before anything
flies, not a runtime safeguard: it says whether the design leaves room for
things to go wrong, not that it will always be the version of the design
that gets flown.

## Approach corridors

Code: {py:class}`ApproachCorridor
<ios_mission_planner.safety.corridor.ApproachCorridor>`,
{py:func}`check_corridor <ios_mission_planner.safety.corridor.check_corridor>`
in `src/ios_mission_planner/safety/corridor.py`.

Distance from the chief is not the only constraint a real approach has to
respect; the direction of approach matters too, both for sensor and plume
geometry and for collision avoidance design. `ApproachCorridor(direction,
half_angle)` defines a cone of allowed directions around a reference vector,
such as $[0,-1,0]$ for an approach along the V-bar from behind, or
$[-1,0,0]$ along the R-bar from above; `check_corridor` reports whether a
trajectory's angle away from that reference direction ever exceeds
`half_angle`, and the signed margin follows the same convention as the
keep-out check: `angle.max() - half_angle` is positive when the trajectory
left the corridor, negative when it stayed within it. Passive safety and
corridor compliance are genuinely independent properties of a trajectory:
a hop can coast safely if a burn fails while still swinging briefly outside
its approach cone mid-transfer, or the reverse, so a real design has to be
checked against both rather than assuming one implies the other.

## Monte Carlo collision probability

Code: {py:func}`collision_probability
<ios_mission_planner.safety.collision_probability.collision_probability>` in
`src/ios_mission_planner/safety/collision_probability.py`.

The checks above are deterministic: given one trajectory, did it violate
the zone or not. In practice, navigation and burn-execution errors mean the
actual relative state at any given time is really a probability
distribution, not a single point, and what matters is the probability that
some realization of that distribution ends up inside the keep-out zone.
`collision_probability` estimates this by Monte Carlo: it draws a large
number of samples from a Gaussian centered on a nominal relative state, with
a covariance combining navigation and burn-execution uncertainty, and
propagates every sample forward using the closed-form Clohessy-Wiltshire
state transition matrix from the relative dynamics chapter rather than a
full nonlinear integration, which is what keeps tens of thousands of samples
computationally cheap. A sample counts as a collision if it ever falls
inside the zone at any of the requested evaluation times, and the reported
probability is simply the fraction of samples that do. Because it reuses the
same `KeepOutEllipsoid` as the deterministic check, the hard body and the
probabilistic risk assessment are always defined consistently with each
other.

## See it in action

The worked example for this chapter is {doc}`../examples/safety_fundamentals`,
which walks through keep-out zones, passive safety, and approach corridors
on the same pair of two-impulse hops, then adds a fourth part on how J2
erodes a nominally drift-free safety ellipse over many orbits and what a
once-per-orbit station-keeping correction costs to counter it. The Monte
Carlo collision probability check is exercised separately, as part of a
full mission's safety assessment, in `examples/rendezvous_safety_assessment.ipynb`.
