# Impulsive maneuvers

The previous two parts described how the two spacecraft move when nobody
acts on them. This chapter is about changing that motion on purpose. It
treats every burn as an instantaneous change of velocity, a $\Delta v$,
which is an excellent approximation whenever the burn is short compared
with the orbital period, and it follows the mission in order: first the
large maneuvers that bring the deputy from its parking orbit to the orbit
of the chief, then the small ones that move it between relative states near
the chief.

## Changing the size of an orbit: the Hohmann transfer

The vis-viva equation of {doc}`../foundations/two_body`, $v^2 =
\mu(2/r - 1/a)$, says that at a given radius the speed determines the
semi-major axis. A burn along the velocity therefore changes the size of
the orbit, and it is the most efficient way to do so, since all of the
$\Delta v$ goes into changing the speed.

The Hohmann transfer uses two such burns to move between two coplanar
circular orbits of radii $r_1$ and $r_2$. The first burn puts the
spacecraft on an ellipse of semi-major axis $a_t = (r_1+r_2)/2$ that is
tangent to both circles; half a revolution later, at the other end of the
ellipse, the second burn circularizes. Each burn is the difference between
the speed on the circle and the speed on the ellipse at the same radius:

$$
\Delta v_1 = \sqrt{\mu\left(\frac2{r_1}-\frac1{a_t}\right)} -
\sqrt{\frac\mu{r_1}}, \qquad
\Delta v_2 = \sqrt{\frac\mu{r_2}} -
\sqrt{\mu\left(\frac2{r_2}-\frac1{a_t}\right)},
$$

and the transfer lasts half the period of the ellipse, $t_H =
\pi\sqrt{a_t^3/\mu}$. For the reference mission, raising the deputy from
400 km to 500 km costs about 56 m/s in total. The first burn alone, applied
backward, is also how a spacecraft is removed from orbit: it lowers the
opposite side of the orbit into the atmosphere, and drag does the rest.

## Catching up: phasing

Being on the right orbit is not enough, the deputy must also be at the
right place on it. This is the phasing problem, and its solution uses the
fact that a lower orbit is faster. On a circular parking orbit of radius
$a_p < a_c$ the deputy has mean motion $n_p > n_c$ and gains on the chief
at the constant angular rate $n_p - n_c$.

The Hohmann transfer fixes the geometry of the departure. During the
transfer the deputy sweeps half a revolution while the chief sweeps the
angle $n_c t_H$. For the deputy to arrive a distance $s_f$ behind the
chief, the chief must lead it at departure by the angle

$$
\varphi_{\mathrm{dep}} = \pi - n_c\,t_H + \frac{s_f}{a_c},
$$

and if the chief leads by $\varphi_0$ at the start, the deputy waits on its
parking orbit for

$$
t_{\mathrm{wait}} = \frac{\varphi_0 - \varphi_{\mathrm{dep}}}{n_p - n_c}.
$$

Phasing costs time, not propellant. It is the same mechanism as the drift
of the relative orbit in {doc}`../relative/hcw`: the difference in altitude
is a radial offset $x_{\mathrm{off}}$, and the wait on the parking orbit is
where the resulting drift is spent.

## Changing the orbit plane

A burn perpendicular to the orbit plane rotates the velocity vector
without changing its magnitude. The velocities before and after form an
isosceles triangle with two sides $v$ and the angle $\delta i$ between
them, so the cost is the base of the triangle,

$$
\Delta v = 2v\sin\frac{\delta i}{2}.
$$

In low Earth orbit $v$ is about 7.6 km/s, so each degree of plane change
costs about 130 m/s, more than the whole Hohmann transfer above. This is
why a rendezvous mission is launched into the plane of its target and
avoids plane changes whenever it can. When a change of plane and a change
of speed are both needed, doing them with a single burn is cheaper than
doing them separately, by the law of cosines on the same triangle:

$$
\Delta v = \sqrt{v_1^2+v_2^2-2v_1v_2\cos\delta i}.
$$

## Impulses in the relative frame

Near the chief the natural description is the linear model of
{doc}`../relative/hcw`, and the effect of a small impulse can be read
directly from the state transition matrix.

An impulse $\Delta v_y$ along the track changes the semi-major axis by
$\Delta a = 2\Delta v_y/n$. The deputy starts moving in the direction of
the burn, rises, and then falls behind: after one orbit it has moved along
the track by

$$
\Delta y = -\frac{6\pi}{n}\,\Delta v_y ,
$$

in the direction opposite to the impulse. Accelerating to go forward takes
the deputy backward, the most famous counterintuitive result of orbital
rendezvous. A second, opposite impulse one orbit later stops the drift, and
the pair of them moves the deputy along the V-bar.

An impulse $\Delta v_x$ in the radial direction leaves the semi-major axis
unchanged. The deputy describes a closed 2:1 ellipse that reaches its
largest along-track displacement, $4\Delta v_x/n$, half an orbit later, and
returns to the starting point after a full orbit. Stopping it at the half
orbit with a second radial impulse also moves the deputy along the V-bar,
and if the second burn fails the deputy simply comes back, which is why
radial hops are preferred close to the chief.

An impulse $\Delta v_z$ across the track starts an oscillation of amplitude
$\Delta v_z/n$ and has no effect in the orbit plane.

## Two-impulse targeting

The general problem is to move from a relative position $\vect\rho_0$ to a
relative position $\vect\rho_f$ in a given time $t$. Writing the state
transition matrix in $3\times3$ blocks,

$$
\Phi(t) = \begin{bmatrix}\Phi_{rr} & \Phi_{rv}\\ \Phi_{vr} &
\Phi_{vv}\end{bmatrix}, \qquad
\vect\rho_f = \Phi_{rr}\,\vect\rho_0 + \Phi_{rv}\,\dot{\vect\rho}_0^{+},
$$

the velocity needed at departure is

$$
\dot{\vect\rho}_0^{+} = \Phi_{rv}^{-1}\left(\vect\rho_f -
\Phi_{rr}\,\vect\rho_0\right).
$$

The first impulse is the difference between this velocity and the one the
deputy has, $\Delta\vect v_1 = \dot{\vect\rho}_0^{+} -
\dot{\vect\rho}_0^{-}$. The velocity on arrival follows from the lower
blocks, $\dot{\vect\rho}_f^{-} = \Phi_{vr}\,\vect\rho_0 +
\Phi_{vv}\,\dot{\vect\rho}_0^{+}$, and the second impulse changes it into
the velocity wanted there, $\Delta\vect v_2 = \dot{\vect\rho}_f^{+} -
\dot{\vect\rho}_f^{-}$.

The choice of the arrival velocity is what gives the maneuver its purpose.
Zero velocity on the V-bar ends in a hold point. The velocity that
satisfies the drift-free condition for the arrival position ends on a
closed ellipse, on which the deputy can stay without further burns. A small
closing velocity at the position of the docking port ends in contact.

The time of flight is the free parameter, and the cost depends on it
strongly. Short transfers are expensive because they fight the natural
dynamics; transfers close to the natural half-orbit and full-orbit
solutions are cheap. The method fails when $\Phi_{rv}$ cannot be inverted:
the in-plane block is singular at whole orbits (and at a few other isolated
times), where the natural motion returns to the same position whatever the
initial velocity, and the out-of-plane block is singular at every half
orbit, for the same reason.

## From the linear design to the real orbit

A burn computed with the linear model is flown in the nonlinear world, and
two practical points follow.

The first is that the burn should be computed from the state the deputy
actually has when it fires, as an on-board guidance would do, not from the
state the plan assumed. A second, correcting burn, or an insertion onto the
closed ellipse that passes through the point actually reached, then absorbs
the error of the first.

The second is the curvature of the orbit. As noted in
{doc}`../relative/hcw`, a deputy on the same orbit as the chief appears in
the LVLH frame at a radial offset $x \approx -y^2/(2a_c)$. Feeding that
measured state to the linear model introduces a drift that does not exist,
and at 5 km a transfer designed that way misses by tens of metres. Removing
the offset before targeting, $x \to x + y^2/(2a_c)$, brings the miss down
to a few metres. The effect is geometric and it is the first correction any
far-range use of the linear model needs.

## In the code

The orbital maneuvers are in `dynamics/orbital/maneuvers.py`.
`hohmann_transfer` returns the two burns and the transfer time for two
radii, `plane_change_dv` the cost of a pure rotation of the velocity, and
`combined_plane_change_dv` the cost of a single burn that changes speed and
plane together.

Targeting is in `dynamics/relative/cw.py`. `cw_targeting` takes the initial
relative position and velocity, the desired final position and velocity,
the time of flight and the mean motion, and returns the two impulses. The
arrival state on a closed ellipse is obtained from the tools of the
previous part: `state_from_relative_orbit` for an ellipse chosen in
advance, or `drift_free_orbit_through` for the ellipse that passes through
the point the deputy has actually reached.

## In the notebook

The targeting sections of {doc}`../examples/02_cw_fundamentals/cw_fundamentals`
compute a transfer between two hold points, show how its cost varies with
the time of flight, and verify the result against the numerical integration
of the HCW equations.

{doc}`../examples/06_rendezvous_to_safety_ellipse/rendezvous_to_safety_ellipse`
then flies the whole first half of the reference mission: the wait on the
parking orbit, the Hohmann transfer to a hold point 5 km behind the chief,
a two-impulse transfer toward the chief, and the insertion onto a closed
ellipse. The maneuvers are designed with the formulas of this chapter and
flown with the nonlinear two-body dynamics of both spacecraft, so the
errors of the linear design are measured, including the effect of the
curvature correction, and the last section repeats the final orbits with
$J_2$.

## References

1. D. A. Vallado, *Fundamentals of Astrodynamics and Applications*, 4th
   edition, Microcosm Press, 2013.
2. W. Fehse, *Automated Rendezvous and Docking of Spacecraft*, Cambridge
   University Press, 2003.
3. W. H. Clohessy, R. S. Wiltshire, "Terminal guidance system for satellite
   rendezvous," *Journal of the Aerospace Sciences*, 27(9), 1960.
