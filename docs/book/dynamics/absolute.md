# Absolute dynamics

"Absolute" here means motion described in an inertial frame centered on the
Earth, as opposed to the *relative* motion of one spacecraft with respect to
another, which is the subject of the next chapter. Everything in this chapter
is the foundation the relative-motion equations are later linearized from, so
it is worth having it in one place before moving on.

## The two-body problem

Code: {py:func}`two_body_acceleration_symbolic
<ios_mission_planner.dynamics.orbital.two_body.two_body_acceleration_symbolic>`,
{py:func}`two_body_dynamics_symbolic
<ios_mission_planner.dynamics.orbital.two_body.two_body_dynamics_symbolic>` in
`src/ios_mission_planner/dynamics/orbital/two_body.py`.

Treating the Earth and a spacecraft as point masses, Newton's law of
gravitation gives the force on the spacecraft as

$$
\vect{F} = -\frac{G M m}{r^2}\,\hat{\vect{r}},
$$

where $\vect{r}$ is the position of the spacecraft relative to the Earth's
center, $r = |\vect{r}|$, and $\hat{\vect r} = \vect r / r$. Dividing by the
spacecraft mass $m$ and writing $\mu = GM$ for the Earth's gravitational
parameter gives the two-body equation of motion,

$$
\ddot{\vect r} = -\frac{\mu}{r^3}\,\vect r .
$$

This is exactly what `two_body_acceleration_symbolic` returns componentwise,
and `two_body_dynamics_symbolic` turns into the six-state ODE system
$[\vect r, \dot{\vect r}]$ that heyoka integrates.

Two quantities are conserved along any solution of this equation: the
specific angular momentum $\vect h = \vect r \times \dot{\vect r}$ and the
specific mechanical energy

$$
\mathcal{E} = \frac{v^2}{2} - \frac{\mu}{r}.
$$

Conservation of $\vect h$ confines the motion to a fixed plane (the orbit
plane, normal to $\vect h$); conservation of $\mathcal E$ fixes the orbit's
size through the semi-major axis, $a = -\mu / (2\mathcal E)$. Together with
the inverse-square force law, these conserved quantities are what force the
trajectory to be a conic section: an ellipse for bound orbits ($\mathcal E <
0$), the case this project is concerned with throughout.

## Classical orbital elements

Code: {py:func}`classical_elements_from_state
<ios_mission_planner.dynamics.orbital.elements.classical_elements_from_state>`,
{py:func}`cartesian_state_from_elements
<ios_mission_planner.dynamics.orbital.elements.cartesian_state_from_elements>`
in `src/ios_mission_planner/dynamics/orbital/elements.py`.
Reference: Vallado, *Fundamentals of Astrodynamics and Applications*.

A Cartesian state $[\vect r, \vect v]$ is six numbers, but they mix position
and velocity in a way that is hard to picture. The six classical orbital
elements re-express the same information as a size, a shape, an orientation,
and a position along the orbit, each with a direct geometric meaning:

- $a$ — semi-major axis: half the long axis of the ellipse, i.e. the orbit's
  size.
- $e$ — eccentricity: how elongated the ellipse is ($e=0$ circular, $0<e<1$
  elliptical).
- $i$ — inclination: the angle between the orbit plane and the Earth's
  equatorial plane.
- $\Omega$ — right ascension of the ascending node (RAAN): where the orbit
  crosses the equator going northward, measured in the equatorial plane from
  a reference direction.
- $\omega$ — argument of perigee: the angle, measured in the orbit plane from
  the ascending node, to perigee (closest approach).
- $M$ — mean anomaly: where the spacecraft is along the orbit right now,
  expressed as an angle that advances at the orbit's *average* rate.

The orbit's size and shape come directly from the conserved energy and
angular momentum: $a$ from $\mathcal E$ as above, and the eccentricity vector

$$
\vect e = \frac{1}{\mu}\left[\left(v^2 - \frac{\mu}{r}\right)\vect r -
(\vect r \cdot \vect v)\,\vect v\right],
$$

a vector that points from the focus toward perigee with magnitude $e$ — this
is exactly the `e_vec` computed in `classical_elements_from_state`. The
orientation angles $i$, $\Omega$, $\omega$ follow from the geometry of
$\vect h$, the node vector $\vect n = \hat{\vect z} \times \vect h$, and
$\vect e$ relative to each other; the code computes each as the angle between
the relevant pair of vectors, resolving the quadrant ambiguity ($\arccos$ only
returns $[0,\pi]$) by checking the sign of a third vector, exactly as Vallado
does.

For a near-circular orbit ($e$ below a small tolerance), perigee is not
meaningfully defined, since the eccentricity vector has near-zero magnitude
and an undefined direction. `classical_elements_from_state` handles this by
setting $\omega = 0$ by convention and reporting the *argument of latitude*
(the angle from the ascending node to the current position) in place of the
mean anomaly.

### Position along the orbit: anomalies and Kepler's equation

The mean anomaly $M$ advances uniformly, $M = M_0 + n(t - t_0)$, where
$n = \sqrt{\mu/a^3}$ is the mean motion. But a real, eccentric orbit does not
move at a uniform angular rate; it moves faster near perigee and slower near
apogee. The *true anomaly* $\nu$, the actual angle from perigee to the
spacecraft, is related to $M$ through an intermediate angle, the *eccentric
anomaly* $E$, defined geometrically on the auxiliary circle that circumscribes
the ellipse. The three are linked by Kepler's equation,

$$
M = E - e \sin E,
$$

which has no closed-form inverse: `cartesian_state_from_elements` solves it
for $E$ given $M$ by Newton's method (see
`_eccentric_from_mean_anomaly`), then gets $\nu$ from $E$ via

$$
\tan\frac{\nu}{2} = \sqrt{\frac{1+e}{1-e}}\,\tan\frac{E}{2}.
$$

With $\nu$ known, the position and velocity in the orbit plane (the
perifocal frame, $x$ toward perigee, $y$ 90° ahead in the direction of
motion) follow from the orbit equation $r = p/(1+e\cos\nu)$, with semi-latus
rectum $p = a(1-e^2)$:

$$
\vect r_{pf} = \frac{p}{1+e\cos\nu}
\begin{bmatrix}\cos\nu\\ \sin\nu\\ 0\end{bmatrix}, \qquad
\vect v_{pf} = \sqrt{\frac{\mu}{p}}
\begin{bmatrix}-\sin\nu\\ e+\cos\nu\\ 0\end{bmatrix}.
$$

A final 3-1-3 rotation by $\Omega$, $i$, $\omega$ carries this from the
perifocal frame into the inertial frame, which is exactly the `rotation`
matrix built in `cartesian_state_from_elements`. `classical_elements_from_state`
is the inverse of this whole chain.

## The J2 perturbation

Code: {py:func}`j2_acceleration_symbolic
<ios_mission_planner.dynamics.orbital.perturbations.j2_acceleration_symbolic>`,
{py:func}`two_body_j2_dynamics_symbolic
<ios_mission_planner.dynamics.orbital.perturbations.two_body_j2_dynamics_symbolic>`
in `src/ios_mission_planner/dynamics/orbital/perturbations.py`.
Reference: Vallado, *Fundamentals of Astrodynamics and Applications*.

The two-body equation treats the Earth as a point mass, but the Earth is
actually an oblate spheroid, slightly flattened at the poles and bulging at
the equator. This mass distribution adds a small correction to the
gravitational potential beyond the simple $-\mu/r$ term; the dominant
correction, by a wide margin, is the $J_2$ zonal harmonic. Its contribution to
the potential is

$$
U_{J_2} = \frac{\mu}{r}\,J_2\left(\frac{R_\oplus}{r}\right)^2
\frac{1}{2}\left(3\sin^2\phi - 1\right),
$$

where $\phi$ is geocentric latitude and $R_\oplus$ is the Earth's equatorial
radius. Taking the gradient of $U_{J_2}$ (with a sign convention matching the
two-body term) and expressing $\sin\phi = z/r$ in Cartesian coordinates gives
the perturbing acceleration implemented in `j2_acceleration_symbolic`:

$$
\vect a_{J_2} = \frac{3}{2}\,\frac{J_2\,\mu\,R_\oplus^2}{r^5}
\begin{bmatrix}
x\left(5\dfrac{z^2}{r^2}-1\right)\\[4pt]
y\left(5\dfrac{z^2}{r^2}-1\right)\\[4pt]
z\left(5\dfrac{z^2}{r^2}-3\right)
\end{bmatrix}.
$$

`two_body_j2_dynamics_symbolic` simply adds this term to the two-body
acceleration from the previous section; the resulting six-state ODE system is
no longer exactly a conic section (the orbital elements $a$, $e$, $i$ stay
nearly constant but $\Omega$, $\omega$, and $M$'s rate all pick up secular
drift terms), but it is a much closer match to how a real low-Earth orbit
actually precesses. In this project, the $J_2$-driven secular rates of
$\Omega$ and $\omega$ reappear explicitly in the relative-orbit-elements state
transition matrix (see the relative orbit elements chapter), since a J2-aware
relative orbit design needs to account for the chief and deputy precessing at
slightly different rates whenever their orbits differ even slightly in $a$,
$e$, or $i$.

## How these systems are propagated

Both `two_body_dynamics_symbolic` and `two_body_j2_dynamics_symbolic` return a
*symbolic* system: a list of (state variable, right-hand-side expression)
pairs built with `heyoka`'s expression graph, rather than a Python function
that evaluates a derivative numerically. `heyoka` compiles this expression
graph once into a high-order Taylor series integrator, then reuses the
compiled integrator across however many numerical propagations are needed.
This is why every dynamics function in this project is named
`*_dynamics_symbolic`, and why runtime-varying numbers (like $\mu$, $J_2$, or
a thrust level) are passed in as `heyoka.par[...]` placeholders rather than
baked into the expression at construction time: the same compiled system can
then be re-run many times with different parameter values without
re-compiling.

## See it in action

Unlike the other chapters in this book, there is currently no standalone
example notebook dedicated to absolute two-body and J2 dynamics on their own;
an earlier one was removed from the repository during the migration to
`heyoka`, and has not been rebuilt since. In practice, this chapter's
functions are exercised indirectly, as the chief's absolute trajectory
underneath a relative-motion or mission-budget notebook, for example in
`examples/rendezvous_to_safety_ellipse.ipynb`, `examples/mission_budget.ipynb`,
and `examples/rendezvous_safety_assessment.ipynb`.
