# Perturbations: oblateness and drag

The two-body orbit of the previous chapter never changes. A real low Earth
orbit does, slowly, because the Earth is not a point mass and because a
thin atmosphere is still present at the altitudes where servicing missions
fly. This chapter adds the two corrections that dominate there: the
oblateness of the Earth, which rotates the orbit without changing its
energy, and atmospheric drag, which removes energy and makes the orbit
decay. Both matter for a rendezvous in a specific way: they act almost, but
not exactly, equally on two nearby spacecraft, and the small difference is
what the chapters on relative motion will have to account for.

## The oblateness of the Earth

The Earth is slightly flattened at the poles and bulges at the equator.
This mass distribution adds to the point-mass potential $-\mu/r$ a series
of corrections, of which the first zonal harmonic, with coefficient $J_2$,
is larger than all the others by about three orders of magnitude. Its
contribution to the potential energy per unit mass is

$$
U_{J_2} = \frac{\mu}{r}\,J_2\left(\frac{R_\oplus}{r}\right)^2
\frac{1}{2}\left(3\sin^2\phi - 1\right),
$$

where $\phi$ is the geocentric latitude and $R_\oplus$ the equatorial
radius of the Earth. Taking the gradient and writing $\sin\phi = z/r$ gives
the perturbing acceleration in Cartesian inertial components,

$$
\vect a_{J_2} = \frac{3}{2}\,\frac{J_2\,\mu\,R_\oplus^2}{r^5}
\begin{bmatrix}
x\left(5\dfrac{z^2}{r^2}-1\right)\\[4pt]
y\left(5\dfrac{z^2}{r^2}-1\right)\\[4pt]
z\left(5\dfrac{z^2}{r^2}-3\right)
\end{bmatrix},
$$

which is added to the two-body acceleration. At 500 km this term is about
a thousandth of the main attraction, small enough to be treated as a
perturbation and large enough to move the orbit by degrees per day.

## Secular drift of the elements

The acceleration above depends on latitude, so it oscillates along the
orbit and makes all the elements oscillate with it. Averaged over one
revolution, the oscillations of $a$, $e$ and $i$ cancel: the oblateness
does no net work, and the size, shape and inclination of the orbit are
preserved on average. The three angles do not average out. To first order
in $J_2$ they drift at constant rates,

$$
\dot\Omega = -\frac{3}{2}\,J_2\left(\frac{R_\oplus}{p}\right)^2 n\cos i,
\qquad
\dot\omega = \frac{3}{4}\,J_2\left(\frac{R_\oplus}{p}\right)^2 n\,
\left(5\cos^2 i - 1\right),
$$

$$
\dot M = n + \frac{3}{4}\,J_2\left(\frac{R_\oplus}{p}\right)^2 n\,
\sqrt{1-e^2}\,\left(3\cos^2 i - 1\right),
$$

with $p = a(1-e^2)$. Each of the three has a direct physical reading.

The first is the *regression of the nodes*. The equatorial bulge pulls the
spacecraft toward the equator, and the orbit responds as a gyroscope does,
by precessing: the orbit plane turns about the polar axis, westward for a
prograde orbit and eastward for a retrograde one. For the reference orbit
of this book, at 500 km and 51.6 degrees, the node moves by almost five
degrees per day.

The second is the *rotation of the line of apsides*: perigee moves within
the orbit plane, forward when $5\cos^2 i > 1$ and backward otherwise. The
rate vanishes at the critical inclination, $\cos^2 i = 1/5$, which is about
63.4 degrees, the reason why orbits that need a fixed perigee use it.

The third is a small change in the mean motion: the spacecraft completes
its revolutions slightly faster or slower than the two-body period of the
same semi-major axis would suggest.

All three rates depend on $a$, $e$ and $i$. Two spacecraft whose orbits
differ slightly in any of these therefore drift at slightly different
rates, and their relative geometry changes even though neither of them
maneuvers. This differential drift is the subject of
{doc}`../relative/perturbed`.

## Sun-synchronous orbits

The nodal drift can be put to use. The direction of the Sun, seen from the
Earth, completes one revolution per year, about 0.986 degrees per day
eastward. Choosing the inclination so that $\dot\Omega$ equals exactly that
rate keeps the orbit plane at a fixed angle to the Sun all year, which
fixes the lighting conditions along the orbit. Since the drift must be
eastward, $\cos i$ must be negative and the orbit slightly retrograde:
about 97.4 degrees at 500 km and 98.6 degrees at 800 km. For any other
inclination, the orbit plane turns with respect to the Sun and the lighting
changes from week to week, which is what the illumination analysis of a
mission has to track.

## Atmospheric drag

Drag is the one perturbation considered here that removes energy. The
residual atmosphere exerts on the spacecraft a force opposite to its
velocity relative to the air,

$$
\vect a_{\mathrm{drag}} = -\frac12\,\rho\,\frac{C_d A}{m}\,
|\vect v_{\mathrm{rel}}|\,\vect v_{\mathrm{rel}},
\qquad
\vect v_{\mathrm{rel}} = \vect v - \vect\omega_\oplus \times \vect r,
$$

where $\rho$ is the local density and $\vect\omega_\oplus$ the rotation
vector of the Earth, with which the atmosphere is assumed to rotate. The
properties of the spacecraft enter only through the combination $C_dA/m$,
drag coefficient times cross-section over mass, called here the *ballistic
coefficient*: a larger value means more deceleration for the same density
and speed. (Some texts use the name for the inverse, $m/(C_dA)$; the
package uses the form above, in square metres per kilogram.)

The density falls off roughly exponentially with altitude. Over a limited
altitude band it is modelled as

$$
\rho(r) = \rho_0 \, \exp\!\left(-\frac{r - r_0}{H}\right),
$$

with a reference density $\rho_0$ at a reference radius $r_0$ and a scale
height $H$ of a few tens of kilometres. The real density also depends
strongly on solar activity, local time and latitude, so this model sizes
the effect but does not forecast it.

## Orbital decay

For a circular orbit, the decay rate follows from an energy balance. The
drag force does work at the rate $\vect a_{\mathrm{drag}}\cdot\vect v =
-\tfrac12\rho\,(C_dA/m)\,v^3$, neglecting the rotation of the atmosphere,
and the orbital energy is $\mathcal E = -\mu/(2a)$, so that $\dot{\mathcal
E} = \mu\,\dot a/(2a^2)$. Equating the two with $v=\sqrt{\mu/a}$ gives

$$
\dot a = -\rho\,\frac{C_dA}{m}\,\sqrt{\mu a}.
$$

The orbit shrinks at a rate proportional to the density and to the
ballistic coefficient. As it shrinks, the spacecraft speeds up: drag makes
a satellite go faster, because for every unit of energy removed the
potential energy drops by two units and the kinetic energy rises by one. This
"drag paradox" is the same mechanism by which a lower orbit catches up with
a higher one, and it is what makes differential drag usable as a control.
The rotation of the atmosphere reduces the relative speed of a prograde
orbit and slows the decay by roughly ten percent in low Earth orbit.

## In the code

The oblateness model is in `dynamics/orbital/perturbations.py`.
`j2_acceleration_symbolic` returns the perturbing acceleration, and
`two_body_j2_dynamics_symbolic` adds it to the two-body system, with $\mu$,
$J_2$ and $R_\oplus$ as runtime parameters. The averaged rates are
available in closed form from `j2_secular_rates`, which returns the drift
of the node, of the perigee, and the correction to the mean motion, and
`sun_synchronous_inclination` inverts the nodal rate for the inclination
that follows the Sun.

The drag model is in `dynamics/orbital/drag.py`.
`exponential_atmosphere_density` and `drag_acceleration_symbolic` implement
the two formulas above, with the reference density, radius, scale height
and the rotation rate of the atmosphere supplied by the caller, since no
single table of values is right for every epoch.
`two_body_drag_dynamics_symbolic` and `two_body_j2_drag_dynamics_symbolic`
assemble the complete systems, without and with oblateness.
`circular_orbit_decay_rate` evaluates the closed-form decay rate, which is
used to check the numerical model and to size a decay quickly.

## In the notebook

The second half of {doc}`../examples/01_orbital_foundations/orbital_foundations`
propagates the reference orbit with the oblateness term for three days,
samples the node and the perigee once per revolution, and compares their
drift with the secular rates: they agree to a fraction of a percent. It
then lets a 400 km circular orbit decay for ten days and compares the loss
of altitude with the closed-form decay rate, which matches when the
atmosphere is held at rest, and measures by how much the rotation of the
atmosphere slows the decay.

## References

1. D. A. Vallado, *Fundamentals of Astrodynamics and Applications*, 4th
   edition, Microcosm Press, 2013.
2. D. Brouwer, "Solution of the problem of artificial satellite theory
   without drag," *The Astronomical Journal*, 64, 1959.
3. O. Montenbruck, E. Gill, *Satellite Orbits: Models, Methods and
   Applications*, Springer, 2000.
