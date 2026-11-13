# The two-body problem and orbital elements

A rendezvous is the relative motion of two spacecraft, but each of them is
first of all a body in orbit around the Earth. This chapter describes that
motion in its simplest form, with the Earth as a point mass and nothing
else acting on the spacecraft. Everything in the following chapters is
either a correction to this model or a difference between two solutions of
it.

## The equation of motion

Treating the Earth and the spacecraft as point masses, Newton's law of
gravitation gives the force on the spacecraft as

$$
\vect F = -\frac{G M m}{r^2}\,\hat{\vect r},
$$

where $\vect r$ is the position of the spacecraft relative to the center of
the Earth, $r = |\vect r|$, and $\hat{\vect r} = \vect r / r$. Dividing by
the spacecraft mass $m$ and writing $\mu = GM$ for the gravitational
parameter of the Earth gives the two-body equation of motion,

$$
\ddot{\vect r} = -\frac{\mu}{r^3}\,\vect r .
$$

The mass of the spacecraft has dropped out: every body follows the same
orbit from the same initial conditions, which is why a chief and a deputy
released together stay together.

## What the motion conserves

Two quantities are constant along any solution. Taking the cross product of
the equation of motion with $\vect r$ shows that the specific angular
momentum

$$
\vect h = \vect r \times \dot{\vect r}
$$

does not change, because gravity is directed along $\vect r$ and exerts no
torque. A constant $\vect h$ confines the motion to a fixed plane, the orbit
plane, perpendicular to $\vect h$. Taking the dot product with
$\dot{\vect r}$ instead shows that the specific mechanical energy

$$
\mathcal{E} = \frac{v^2}{2} - \frac{\mu}{r}
$$

is constant as well. Together with the inverse-square law, these two
invariants force the trajectory to be a conic section with the Earth at one
focus. For a bound orbit, $\mathcal E < 0$, the conic is an ellipse of
semi-major axis

$$
a = -\frac{\mu}{2\mathcal E},
$$

so the energy fixes the size of the orbit and nothing else. Solving the
energy equation for the speed gives the *vis-viva* equation,

$$
v^2 = \mu\left(\frac{2}{r} - \frac{1}{a}\right),
$$

which relates the speed at any point to the size of the orbit and is the
basis of every impulsive maneuver in {doc}`../maneuvers/impulsive`. The
orbital period depends on the size alone,

$$
T = 2\pi\sqrt{\frac{a^3}{\mu}}, \qquad n = \frac{2\pi}{T} =
\sqrt{\frac{\mu}{a^3}},
$$

with $n$ the *mean motion*, the average angular rate of the spacecraft
along its orbit. A lower orbit is a faster one. This simple fact governs
the whole first phase of a rendezvous, in which the deputy waits on a lower
orbit precisely to catch up with the chief.

## Classical orbital elements

A Cartesian state $[\vect r, \vect v]$ is six numbers that all change at
every instant, even on an orbit that never changes. The classical orbital
elements express the same information as a size, a shape, an orientation,
and a position along the orbit, and on a two-body orbit five of the six are
constant.

The semi-major axis $a$ is half the long axis of the ellipse and sets the
size. The eccentricity $e$ describes the shape: zero for a circle, and
growing toward one as the ellipse becomes more elongated. Three angles fix
the orientation in space. The inclination $i$ is the angle between the
orbit plane and the equatorial plane of the Earth. The right ascension of
the ascending node $\Omega$ locates the point where the orbit crosses the
equator going northward, measured in the equatorial plane from a fixed
inertial direction. The argument of perigee $\omega$ is the angle, measured
within the orbit plane from the ascending node, to perigee, the point of
closest approach. The sixth element, the mean anomaly $M$, says where the
spacecraft currently is along the orbit, as an angle that advances at the
constant rate $n$.

Size and shape come directly from the invariants. The semi-major axis
follows from $\mathcal E$, and the shape from the eccentricity vector

$$
\vect e = \frac{1}{\mu}\left[\left(v^2 - \frac{\mu}{r}\right)\vect r -
(\vect r \cdot \vect v)\,\vect v\right],
$$

a constant vector that points from the focus toward perigee and has
magnitude $e$. The three orientation angles follow from the geometry of
$\vect h$, of the node vector $\hat{\vect z} \times \vect h$, and of
$\vect e$ relative to one another, with the quadrant of each angle resolved
by the sign of a third component.

Two degenerate cases need a convention. For a circular orbit the
eccentricity vector vanishes and perigee is undefined, so $\omega$ is set
to zero and the position along the orbit is measured from the ascending
node instead (the *argument of latitude*). For an equatorial orbit the node
is undefined, and the perigee direction is measured from the inertial $x$
axis.

## Position along the orbit: Kepler's equation

The mean anomaly advances uniformly, $M = M_0 + n(t - t_0)$, but the
spacecraft does not: it moves faster near perigee and slower near apogee,
because the angular momentum $h = r^2\dot\nu$ is constant and $r$ is not.
The actual angle from perigee to the spacecraft is the *true anomaly*
$\nu$. It is related to $M$ through an intermediate angle, the *eccentric
anomaly* $E$, by Kepler's equation,

$$
M = E - e \sin E .
$$

This equation has no closed-form inverse, since $E$ appears both linearly
and inside a sine. Given $M$ it is solved by Newton's method: starting from
a guess $E_0$, the iteration

$$
E_{k+1} = E_k - \frac{E_k - e\sin E_k - M}{1 - e\cos E_k}
$$

converges quadratically, doubling the number of correct digits at each
step, so a handful of iterations is always enough. With $E$ known, the true
anomaly follows from

$$
\tan\frac{\nu}{2} = \sqrt{\frac{1+e}{1-e}}\,\tan\frac{E}{2},
$$

and position and velocity in the orbit plane (the perifocal frame, with $x$
toward perigee and $y$ a quarter of a turn ahead) follow from the orbit
equation $r = p/(1+e\cos\nu)$, with semi-latus rectum $p = a(1-e^2)$:

$$
\vect r_{pf} = \frac{p}{1+e\cos\nu}
\begin{bmatrix}\cos\nu\\ \sin\nu\\ 0\end{bmatrix}, \qquad
\vect v_{pf} = \sqrt{\frac{\mu}{p}}
\begin{bmatrix}-\sin\nu\\ e+\cos\nu\\ 0\end{bmatrix}.
$$

A final sequence of three rotations, by $\omega$ about the orbit normal,
by $i$ about the node line, and by $\Omega$ about the polar axis, carries
these vectors from the perifocal frame into the inertial frame.

## Numerical propagation

The two-body problem has the closed-form solution above, but nothing that
follows does: perturbations, thrust, and the coupled motion of two
spacecraft all need numerical integration. This project integrates every
dynamical model with a high-order Taylor method, and since every result in
the book rests on it, it is worth knowing what that means.

Given $\dot{\vect x}=\vect f(\vect x,t)$, a Taylor method advances the
solution by one step $h$ by expanding the solution itself in a Taylor
series,

$$
\vect x(t+h) = \vect x(t) + h\,\dot{\vect x}(t) +
\frac{h^2}{2!}\,\ddot{\vect x}(t) + \cdots +
\frac{h^p}{p!}\,\vect x^{(p)}(t) + O(h^{p+1}),
$$

truncated at some order $p$. The usual obstacle is that the higher
derivatives of the solution require differentiating $\vect f$ over and
over, which is unmanageable by hand. The obstacle disappears if $\vect f$
is available as a symbolic expression: the derivatives can then be
generated automatically, to any order, and compiled into machine code
specific to that system. The order and the step are chosen at run time to
meet a target error at every step. For a smooth problem such as orbital
motion the order is typically around twenty, far beyond what a fixed
low-order scheme such as Runge-Kutta uses, which is why these integrations
conserve energy and angular momentum to machine precision over many orbits
without a fine step.

## In the code

The equation of motion lives in `dynamics/orbital/two_body.py`.
`two_body_acceleration_symbolic` returns the three components of the
gravitational acceleration as symbolic expressions, so that other models
can add their own terms to it, and `two_body_dynamics_symbolic` assembles
the six-state system $[\vect r, \vect v]$.

Every dynamical model in the package follows this pattern and carries the
suffix `_symbolic`: it returns a list of pairs (state variable, right-hand
side) built with the expression system of
[heyoka](https://bluescarni.github.io/heyoka.py/), not a Python function
that evaluates a derivative. Numbers that may change from one run to the
next, such as $\mu$ or a thrust level, enter as runtime parameters, so the
same compiled system can be reused with different values. The function
`propagate` in `propagation/heyoka_propagator.py` takes such a system, an
initial state, the parameter values and a grid of times, builds the Taylor
integrator, and returns the states on the grid.

The conversion between Cartesian states and orbital elements is in
`dynamics/orbital/elements.py`. `ClassicalElements` is a small data class
holding the six elements; `classical_elements_from_state` computes them from
a state, handling the circular and equatorial cases with the conventions
described above, and `cartesian_state_from_elements` is the inverse, which
solves Kepler's equation by Newton's method on the way. The physical
constants used everywhere are collected in `constants.py`.

## In the notebook

{doc}`../examples/01_orbital_foundations/orbital_foundations` propagates an
eccentric, inclined orbit for ten revolutions and verifies that energy and
angular momentum stay constant to machine precision. It then converts every
propagated state to orbital elements, checks that five of them do not move
while the mean anomaly grows linearly, and closes the loop with a round
trip from state to elements and back. The second half of the notebook
belongs to the next chapter.

## References

1. D. A. Vallado, *Fundamentals of Astrodynamics and Applications*, 4th
   edition, Microcosm Press, 2013.
2. H. D. Curtis, *Orbital Mechanics for Engineering Students*, 4th edition,
   Butterworth-Heinemann, 2020.
3. F. Biscani, D. Izzo, "Revisiting high-order Taylor methods for
   astrodynamics and celestial mechanics," *Monthly Notices of the Royal
   Astronomical Society*, 504(2), 2021.
