# Relative motion about a circular chief

The first part described how one spacecraft moves around the Earth. A
rendezvous is about something else: how the deputy moves *as seen from the
chief*. The guidance of a servicer does not ask where it is in an
Earth-centered frame, it asks how far the target is and in which direction.
This chapter builds the frame in which that question is asked, derives the
linear equations that govern the motion in it, and reads their solution
geometrically. The picture that results, a small ellipse that may or may
not drift along the orbit, is the vocabulary of every later chapter.

## The LVLH frame

The frame attached to the chief is the local-vertical, local-horizontal
(LVLH) frame. Its radial axis $\hat{\vect x}$ points along the position
vector of the chief, away from the Earth. Its cross-track axis
$\hat{\vect z}$ points along the orbit normal, $\vect h_c = \vect r_c
\times \vect v_c$. Its along-track axis $\hat{\vect y} = \hat{\vect z}
\times \hat{\vect x}$ completes the right-handed triad and, for a circular
orbit, points along the velocity. In rendezvous practice the along-track
axis is called the V-bar and the radial axis the R-bar.

```{figure} ../figures/lvlh_frame.png
:width: 520px
:name: fig-lvlh

The LVLH frame of the chief and the relative position of the deputy.
```

The frame turns with the chief, once per orbit, so it is not inertial. Its
angular velocity is $\vect\omega = \vect h_c / r_c^2$, directed along
$\hat{\vect z}$. Expressing the state of the deputy in this frame is
therefore more than a rotation of two vectors. If $R$ is the rotation
matrix from inertial to LVLH axes, whose rows are the three unit vectors
above, the relative position and velocity are

$$
\vect\rho = R\,(\vect r_d - \vect r_c), \qquad
\dot{\vect\rho} = R\,(\vect v_d - \vect v_c) - \vect\omega \times
\vect\rho,
$$

where the last term is the correction for the rotation of the frame given
by the transport theorem. This transformation is exact: it assumes neither
a circular chief nor a small separation. It is the tool used throughout the
project to move between the inertial frame, in which the truth is
propagated, and the LVLH frame, in which the approach is designed.

## The Hill-Clohessy-Wiltshire equations

The exact equations of relative motion follow from differentiating
$\vect\rho$ twice in the rotating frame and inserting the two-body
acceleration of both spacecraft. They are nonlinear and depend on the
motion of the chief. Two assumptions reduce them to a form that can be
solved by hand. The first is that the chief is on a circular orbit, so the
frame turns at the constant rate $n = \sqrt{\mu/a_c^3}$. The second is that
the deputy stays close to the chief, $|\vect\rho| \ll r_c$, so the
difference between the gravitational accelerations of the two can be
replaced by its first-order expansion in $\vect\rho$. The result is the
set of Hill-Clohessy-Wiltshire (HCW) equations,

$$
\ddot x - 2n\dot y - 3n^2 x = 0, \qquad
\ddot y + 2n\dot x = 0, \qquad
\ddot z + n^2 z = 0 .
$$

Their structure already tells most of the story. The terms $2n\dot y$ and
$2n\dot x$ are Coriolis accelerations, present because the frame rotates,
and they couple the radial and the along-track motion: a velocity along
the track pushes the deputy radially, and a radial velocity pushes it along
the track. The term $3n^2x$ is the combined effect of the centrifugal
acceleration and of the gradient of gravity: a deputy displaced outward is
pulled further outward. The cross-track motion is separate from the other
two and is a plain harmonic oscillation at the orbital frequency, which is
the linear image of a small difference in orbit plane.

## The closed-form solution

Being linear with constant coefficients, the HCW equations have an explicit
solution. The state at time $t$ is a linear function of the initial state,

$$
\begin{bmatrix}\vect\rho(t)\\ \dot{\vect\rho}(t)\end{bmatrix}
= \Phi(t)\begin{bmatrix}\vect\rho_0\\ \dot{\vect\rho}_0\end{bmatrix},
$$

through the $6\times6$ state transition matrix, written with $c=\cos nt$
and $s=\sin nt$ as

$$
\Phi(t) = \begin{bmatrix}
4-3c & 0 & 0 & s/n & 2(1-c)/n & 0\\
6(s-nt) & 1 & 0 & -2(1-c)/n & (4s-3nt)/n & 0\\
0 & 0 & c & 0 & 0 & s/n\\
3ns & 0 & 0 & c & 2s & 0\\
-6n(1-c) & 0 & 0 & -2s & 4c-3 & 0\\
0 & 0 & -ns & 0 & 0 & c
\end{bmatrix}.
$$

Propagating the relative motion is then a single matrix product, with no
numerical integration. This is why the HCW model is the working tool of
rendezvous design, even though it is an approximation: within its range of
validity it answers instantly, and its structure can be read off the
matrix. The only terms that grow with time are the two containing $nt$,
both in the along-track row. Everything else is periodic.

## Reading the solution: the relative orbit

Grouping the terms of the solution by kind, periodic or growing, gives a
geometric description:

$$
x(t) = x_{\mathrm{off}} + \rho\cos(nt+\alpha), \qquad
y(t) = y_c - \tfrac32\,n\,x_{\mathrm{off}}\,t - 2\rho\sin(nt+\alpha),
\qquad
z(t) = \rho_z \cos(nt+\beta).
$$

In the orbit plane the deputy describes an ellipse whose along-track
semi-axis is always exactly twice its radial semi-axis, a consequence of
the factor two in the Coriolis terms. The ellipse has radial size $\rho$
and phase $\alpha$ and is centered at a radial offset $x_{\mathrm{off}}$
and an along-track position $y_c$. Out of the plane the deputy oscillates
with amplitude $\rho_z$ and phase $\beta$. The six numbers
$(x_{\mathrm{off}}, y_c, \rho, \alpha, \rho_z, \beta)$ are an invertible
change of variables from the Cartesian relative state, and they are far
easier to reason with.

```{figure} ../figures/relative_orbit.png
:width: 620px
:name: fig-relative-orbit

The in-plane relative orbit: a 2:1 ellipse, closed when the radial offset
is zero and drifting along the track otherwise.
```

The one term that grows is the drift of the center,

$$
\dot y_c = -\tfrac32\,n\,x_{\mathrm{off}} .
$$

Its meaning is the fact noted in the first chapter, that a lower orbit is a
faster one. A radial offset of the center of the ellipse is a difference in
semi-major axis, $x_{\mathrm{off}} = \Delta a$, hence a difference in
period: a deputy whose mean position is below the chief gains
$3\pi\,|\Delta a|$ along the track at every orbit, and one above loses the
same amount. All the rest of the motion is the ellipse, which is the image
of a small difference in eccentricity, and the cross-track oscillation,
which is the image of a small difference in orbit plane.

## Trajectories worth knowing

A few special cases of this solution recur in every mission and are worth
naming.

A deputy at rest on the V-bar, with $x=0$ and zero relative velocity, stays
there indefinitely. It is simply on the same orbit as the chief, a little
ahead or behind. This is a *hold point*, and an approach is usually built
as a sequence of them.

A deputy with $x_{\mathrm{off}}=0$ and $\rho>0$ circles the chief on a
closed ellipse once per orbit, without ever spending propellant. In terms
of the initial state the condition is $\dot y_0 = -2n\,x_0$. This is the
*drift-free* condition, and the closed ellipse it produces is the passively
safe parking orbit that the safety analysis relies on: a deputy on it that
stops responding keeps circling instead of drifting into the chief. Adding
a cross-track oscillation with the right phase tilts the ellipse out of the
orbit plane, so that the deputy never crosses the V-bar, where a small
along-track error could bring it to the chief; this is the natural motion
circumnavigation used for inspection.

A deputy with $x_{\mathrm{off}}\neq0$ drifts. A slow, deliberate drift is
how the deputy closes large along-track distances at almost no cost, and an
unintended one is the main hazard of proximity operations, because a small
error in velocity turns into a drift that grows without bound.

## Range of validity

The two assumptions set the limits of the model. The separation must be
small compared with the orbit radius: the neglected terms are of second
order, so the error grows with the square of the distance. One consequence
is visible even for a deputy on exactly the same orbit as the chief: at an
along-track distance $y$ it appears in the LVLH frame at a small radial
offset $x \approx -y^2/(2a_c)$, about 1.8 m at 5 km, only because the
along-track axis is straight while the orbit is curved. The linear model
reads that offset as a real one and predicts a drift that does not exist.
The chief must also be on a circular orbit and free of perturbations. The
next two chapters remove these last two restrictions.

## In the code

The frame is implemented in `dynamics/relative/lvlh.py`. `lvlh_rotation`
builds the rotation matrix from a chief state and `lvlh_angular_rate` the
angular velocity of the frame; `inertial_to_lvlh` and `lvlh_to_inertial`
apply the exact transformation above in the two directions, including the
transport term.

The equations of motion are in `dynamics/relative/hill.py`, where
`hill_equations_symbolic` returns the HCW system with the mean motion as a
runtime parameter, for numerical integration. The closed form is in
`dynamics/relative/cw.py`: `cw_state_transition_matrix` returns $\Phi(t)$.

The geometric description is in `dynamics/relative/relative_orbit.py`.
`RelativeOrbit` holds the six geometric quantities,
`relative_orbit_from_state` computes them from a Cartesian relative state,
and `state_from_relative_orbit` is the inverse, which is how a relative
orbit is designed: one chooses the ellipse and obtains the state that
realizes it. `drift_free_orbit_through` returns the closed ellipse that
passes through a given position.

## In the notebook

{doc}`../examples/02_cw_fundamentals/cw_fundamentals` integrates the HCW
equations numerically and compares them with the state transition matrix,
then explores the free motion: a closed ellipse next to a drifting one, and
an out-of-plane circumnavigation. It introduces the geometric elements,
propagates a relative orbit defined directly through them, and ends by
inserting the deputy onto a closed ellipse. The sections of the notebook on two-impulse targeting belong to
{doc}`../maneuvers/impulsive`.

## References

1. W. H. Clohessy, R. S. Wiltshire, "Terminal guidance system for satellite
   rendezvous," *Journal of the Aerospace Sciences*, 27(9), 1960.
2. H. Schaub, J. L. Junkins, *Analytical Mechanics of Space Systems*, 4th
   edition, AIAA, 2018.
3. W. Fehse, *Automated Rendezvous and Docking of Spacecraft*, Cambridge
   University Press, 2003.
