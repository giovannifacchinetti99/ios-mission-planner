# Safety of the approach

The previous parts answer the question of how to bring the deputy to the
chief. This chapter asks a different one: whether a planned approach is
acceptable, including when something goes wrong. Two spacecraft within a
few hundred metres of each other, moving at several kilometres per second
around the Earth, can collide because of a burn that does not fire or a
velocity known a centimetre per second less well than assumed. The tools of
this chapter are not derived from a single theory. They state the practice
of rendezvous design as checks that can be computed on any trajectory
produced with the models of the previous chapters.

## Keep-out zones

The first requirement is that no planned trajectory enters a region around
the chief. The region is modelled as an ellipsoid centered on the chief,
with semi-axes $a$, $b$, $c$ along the radial, along-track and cross-track
directions; a sphere is the special case of three equal semi-axes. A
relative position $(x,y,z)$ is inside the zone when

$$
\left(\frac{x}{a}\right)^2 + \left(\frac{y}{b}\right)^2 +
\left(\frac{z}{c}\right)^2 < 1 .
$$

The left-hand side is the square of a normalized distance, equal to one on
the surface. Evaluated along a trajectory, its minimum gives a margin,
defined here as one minus the smallest normalized distance: negative when
the trajectory stays outside, by that fraction of the zone, and positive
when it enters. Real missions use nested zones, a large *approach
ellipsoid* that the deputy enters only with permission and a small
*keep-out sphere* that it enters only along the final corridor.

## Passive safety

A trajectory that avoids the keep-out zone when everything works is the
minimum. The stronger requirement is that it avoids the zone when a burn
fails. A burn is *passively safe* if, should it not be executed, the free
motion that follows stays outside the keep-out zone for long enough to
detect the failure and react, typically one or more orbits.

The check follows from the definition. The relative state just before the
burn is propagated forward without the burn, with the equations of relative
motion, and the resulting trajectory is tested against the keep-out zone.
Passive safety is a property of the design, verified before flight. It is
the reason for much of the geometry of an approach: the closed ellipse that
surrounds the chief without crossing the V-bar, the hold points on the
V-bar, and the radial hops of {doc}`../maneuvers/impulsive`, which bring
the deputy back to where it started if the second burn is missed.

## Approach corridors

Distance is not the only constraint. The direction of the approach matters
as well, because the sensors of the deputy and the docking port of the
chief look in specific directions, and because the plume of the thrusters
must not hit the target. The constraint is a cone with its vertex on the
chief, defined by an axis and a half-angle: the V-bar from behind and the
R-bar from below are the usual choices of axis. A position $\vect\rho$
satisfies the constraint when the angle between $\vect\rho$ and the axis
$\hat{\vect d}$,

$$
\theta = \arccos\frac{\vect\rho\cdot\hat{\vect d}}{|\vect\rho|},
$$

is smaller than the half-angle, and the margin is the largest angle along
the trajectory minus the half-angle, negative when the trajectory stays
inside.

Passive safety and the corridor are independent requirements. A two-impulse
transfer between two points of the V-bar bows away from it, because of the
Coriolis coupling, and can leave a narrow corridor while being passively
safe; a faster and straighter transfer can stay in the corridor while
pointing at the chief if its braking burn fails. A design has to be checked
against both.

## Holding a relative orbit against perturbations

A closed ellipse is closed only in the unperturbed linear model.
{doc}`../relative/perturbed` showed that under $J_2$ the relative orbit
deforms through channels that the Cartesian drift $x_{\mathrm{off}}$ does
not see. Whether the deformation ever threatens the keep-out zone depends
on the size and orientation of the ellipse and on how long it is left
alone, so it has to be checked for the geometry at hand.

Holding the design has a cost. A simple and robust scheme measures the
relative state once per orbit and returns it to the designed state with a
two-impulse transfer of nearly one orbit, the cheap end of the cost curve
of {doc}`../maneuvers/impulsive`. The $\Delta v$ spent per orbit is the
station-keeping rate, and multiplied by the planned duration of the hold it
becomes a line of the mission budget.

## Collision probability

The checks above take a trajectory as given. In reality the relative state
is known only through navigation, with an error, and burns are executed
with an error. The state at any time is a probability distribution, and the
quantity of interest is the probability that the deputy enters the keep-out
zone.

The uncertainty at an initial time is modelled as a Gaussian distribution
around the nominal state $\bar{\vect X}_0$, with a covariance $P_0$ that
combines the navigation error and the execution error of the last burn. In
the linear model the distribution stays Gaussian, with mean
$\Phi(t)\bar{\vect X}_0$ and covariance

$$
P(t) = \Phi(t)\,P_0\,\Phi(t)^{\mathsf T} .
$$

The structure of $\Phi$ explains why this matters more than intuition
suggests. The element that maps an initial along-track velocity into a
later along-track position is $(4\sin nt - 3nt)/n$, which grows without
bound: after one orbit it equals $-6\pi/n$. In low Earth orbit an error of
one centimetre per second in the along-track velocity becomes about 170 m
of along-track position one orbit later. Position errors of a few metres
are harmless; velocity errors of a few millimetres per second are what
makes a close relative orbit risky.

The probability of entering the zone at any time within a horizon has no
closed form, because the event involves the whole trajectory. It is
estimated by Monte Carlo: initial states are sampled from the Gaussian,
each is propagated with the state transition matrix to a set of times, and
the probability is the fraction of samples that fall inside the zone at any
of those times. The use of the closed-form matrix makes tens of thousands
of samples cheap. With $N$ samples and an estimated probability $\hat p$,
the standard error of the estimate is $\sqrt{\hat p(1-\hat p)/N}$, which
sets how small a probability a given sample size can resolve.

This estimate propagates an initial uncertainty through the free motion. It
does not include the corrections that a closed-loop guidance would apply
along the way, and it is therefore a conservative screening of a design.
The Monte Carlo analysis of the complete closed-loop mission, later in the
book, removes that limitation.

## In the code

The checks are in the `safety` package. `KeepOutEllipsoid` in
`safety/keepout.py` holds the three semi-axes and evaluates the normalized
distance, and `check_trajectory` applies it to a sampled trajectory,
returning whether the zone was violated, the margin, and a mask of the
violating samples.

`check_passive_safety` in `safety/passive_safety.py` takes the relative
state before a burn, propagates it without the burn for a chosen duration
by integrating the HCW equations, and returns the result of the keep-out
check on that trajectory together with the trajectory itself.

`ApproachCorridor` in `safety/corridor.py` holds the axis and the
half-angle of the cone, and `check_corridor` returns whether a trajectory
leaves it, the margin, and a mask of the samples that are outside.

`collision_probability` in `safety/collision_probability.py` takes the
nominal state, the covariance, the keep-out zone and the evaluation times,
and returns the Monte Carlo estimate, using the state transition matrix of
`cw_state_transition_matrix` for the propagation. All the margins follow
the same sign convention: positive means violation.

## In the notebooks

{doc}`../examples/08_safety_fundamentals/safety_fundamentals` builds the
checks one at a time on simple examples. It tests two transfers between
hold points against a keep-out zone, a failed braking burn and a corridor,
and shows that each transfer passes one check and fails another. Its last
part measures how $J_2$ deforms a closed ellipse over twenty orbits and
what it costs to hold it with a correction per orbit.

{doc}`../examples/09_rendezvous_safety_assessment/rendezvous_safety_assessment`
applies all the checks to the mission flown in
{doc}`../examples/06_rendezvous_to_safety_ellipse/rendezvous_to_safety_ellipse`.
The mission passes every deterministic check with margin, and the collision
probability at the insertion onto the ellipse is nevertheless a few
percent with the assumed navigation and execution errors, a risk that only
the probabilistic check reveals.

## References

1. W. Fehse, *Automated Rendezvous and Docking of Spacecraft*, Cambridge
   University Press, 2003.
2. S. D'Amico, O. Montenbruck, "Proximity operations of formation-flying
   spacecraft using an eccentricity/inclination vector separation,"
   *Journal of Guidance, Control, and Dynamics*, 29(3), 2006.
3. F. K. Chan, *Spacecraft Collision Probability*, The Aerospace Press,
   2008.
