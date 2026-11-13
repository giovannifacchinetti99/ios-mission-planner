# Illumination

The simulation of {doc}`../gnc/simulator` gave the relative sensor a
target that it could always see. An optical sensor needs light. A camera
that looks at a pattern on the docking port of the chief works only while
the Sun illuminates the port, from a direction that does not leave it in
deep shadow, and while the Sun itself is out of its field of view. For a
third of every orbit there is no Sun at all. The final approach lasts a few
minutes, and those minutes must fall where the light is right.

This chapter derives when that happens. It needs the direction of the Sun,
the geometry of the shadow of the Earth, and the orientation of the port,
and it ends with a set of windows that the planning of the approach has to
respect.

## The direction of the Sun

Seen from the Earth, the Sun moves along the ecliptic once a year. Its
position is given with ample accuracy by a short series. With $T$ the time
in Julian centuries from the epoch J2000, the mean longitude and the mean
anomaly of the Sun grow linearly,

$$
\lambda_M = 280.460^\circ + 36000.771^\circ\,T, \qquad
M = 357.529^\circ + 35999.050^\circ\,T,
$$

the eccentricity of the orbit of the Earth adds the equation of the center,

$$
\lambda = \lambda_M + 1.9147^\circ\sin M + 0.0200^\circ\sin 2M,
$$

and the direction follows by rotating through the obliquity of the
ecliptic, $\varepsilon = 23.439^\circ - 0.013^\circ\,T$:

$$
\hat{\vect s} = \begin{bmatrix}\cos\lambda \\ \cos\varepsilon\sin\lambda \\
\sin\varepsilon\sin\lambda\end{bmatrix}.
$$

The result is accurate to about a hundredth of a degree. It can be checked
against events whose instants are published: at an equinox the declination
of the Sun is zero, at a solstice it equals the obliquity.

## Beta angle and eclipses

For an orbit, what matters is where the Sun is with respect to the orbit
plane. The beta angle is the angle between the direction of the Sun and the
plane, positive on the side of the orbit normal $\hat{\vect h}$:

$$
\beta = \arcsin\left(\hat{\vect s}\cdot\hat{\vect h}\right).
$$

The shadow of the Earth is, to a good approximation in low orbit, a
cylinder of the radius $R$ of the Earth on the side opposite to the Sun. A
spacecraft at $\vect r$ is in eclipse when it is on the night side,
$\vect r\cdot\hat{\vect s} < 0$, and its distance from the axis of the
cylinder is less than $R$.

```{figure} ../figures/beta_eclipse.png
:name: fig-beta-eclipse
:width: 100%

Left: the orbit seen from a direction perpendicular to the Sun line. The
beta angle tilts the orbit plane with respect to the Sun, and the orbit
crosses the shadow along a shorter chord. Right: the direction of the Sun
seen from the chief during one orbit, with the arc on which a port looking
backward is lit.
```

For a circular orbit of radius $a$ the eclipse can be computed in closed
form. Measuring the position along the orbit by the angle $u$ from the
point opposite to the projection of the Sun on the orbit plane, the
distance from the shadow axis is $a\sqrt{1 - \cos^2\beta\cos^2 u}$, so the
spacecraft is in shadow while

$$
\cos u > \frac{\sqrt{1 - (R/a)^2}}{\cos\beta},
$$

and the fraction of the orbit spent in eclipse is

$$
f = \frac{1}{\pi}\arccos\left(\frac{\sqrt{1 - (R/a)^2}}{\cos\beta}\right),
\qquad |\beta| < \beta^* = \arcsin\frac{R}{a}.
$$

At $\beta = 0$ the Sun is in the orbit plane and the eclipse is longest,
$f = \arcsin(R/a)/\pi$; at 500 km this is 38 percent of the orbit, 36
minutes. Above the critical angle $\beta^*$, 68 degrees at 500 km, the
orbit misses the shadow and is in sunlight all the time.

The beta angle is not constant. The Sun moves along the ecliptic, and the
orbit plane itself turns, because the node regresses under $J_2$ at the
rate found in {doc}`../foundations/perturbations`. For the orbit of the
reference mission the second motion is the faster, about five degrees per
day, and the beta angle oscillates with a period of about two months,
within limits set by the sum of the inclination and of the obliquity.

## The lighting of a docking port

The port of the chief is fixed in the LVLH frame. In that frame the Sun is
not fixed: the frame turns once per orbit, so the Sun appears to turn once
per orbit about the orbit normal, at the constant angle $\beta$ from the
orbit plane. With $\theta$ the angle of that rotation, measured from
orbital noon, its direction in LVLH axes is

$$
\hat{\vect s}_{lvlh} = \begin{bmatrix}\cos\beta\cos\theta \\
-\cos\beta\sin\theta \\ \sin\beta\end{bmatrix}.
$$

Three conditions decide whether a camera on the deputy can work with a
target on the port. The chief must be out of eclipse. The incidence, the
angle between the Sun and the outward normal $\hat{\vect n}$ of the port,
must be below a limit,

$$
\arccos\left(\hat{\vect s}_{lvlh}\cdot\hat{\vect n}\right) \le i_{max},
$$

because under a grazing light the target is crossed by long shadows and has
little contrast. And the Sun must stay outside a cone around the optical
axis of the camera, which looks along $-\hat{\vect n}$.

For a port that looks backward along the V-bar, $\hat{\vect n} =
-\hat{\vect y}$, the cosine of the incidence is $\cos\beta\sin\theta$. The
port is lit during the half of the orbit in which the chief flies away from
the Sun, from orbital noon to orbital midnight, and the second part of that
half is in eclipse. The favourable window therefore opens when the
incidence falls below its limit, some time after noon, and closes at the
entry into eclipse. With a limit of 70 degrees it lasts about a quarter of
the orbit. In the other sunlit quarter, after the exit from eclipse, the
Sun is ahead of the chief: the port is in its own shadow and the camera of
the deputy looks toward the Sun. A port on the front face would see the
mirror image.

The same expression shows what happens at high beta angles. The incidence
can never be smaller than $\beta$, since $\cos\beta\sin\theta \le
\cos\beta$. When the beta angle exceeds the incidence limit there is no
window at all, although the orbit may be in continuous sunlight: the Sun is
nearly perpendicular to the orbit plane, the normal of the port lies in
that plane, and the light grazes the port all orbit long.

## Windows and the timing of the approach

The outcome of the analysis is a list of windows, the intervals in which
the three conditions hold. The final approach, from the last hold point to
the contact, must fit inside one of them with a margin.

The mission manager applies this as a go/no-go at the last hold point: it
starts the final approach only if the lighting is favourable and will
remain so for the whole duration of the approach. The check is necessary
and should not be the only use of the windows. Holding a point 20 m from
the port costs propellant, and a deputy that arrives there at the wrong
time may wait most of an orbit. The windows are known days in advance, so
the departure from the previous hold point is the thing to time.

## In the code

The package `illumination` has three modules. In `illumination/sun.py`,
`julian_date` converts a calendar date, `sun_position` and `sun_direction`
implement the series, `orbit_normal` gives the normal of an orbit plane and
`beta_angle` the beta angle. In `illumination/eclipse.py`, `in_eclipse`
applies the cylindrical shadow to one position or to an array,
`eclipse_fraction_circular` is the closed form and `critical_beta` the
angle above which there is no eclipse.

In `illumination/lighting.py`, `port_lighting_timeline` propagates a
circular orbit, with the node drifting at the $J_2$ rate, and returns a
`LightingTimeline` with the direction of the Sun in LVLH axes, the eclipse,
the incidence on the port, the angle between the Sun and the camera axis,
the beta angle and the favourable intervals; its method `favourable_for`
answers the go/no-go question for a given start and duration.
`beta_angle_history` follows the beta angle over a long span.

The simulation uses the same functions: it records the direction of the Sun
and the eclipse at every step, and, with the option `lighting_gate` of the
guidance configuration, it holds the deputy at the last hold point until
`favourable_for` gives the go.

## In the notebook

{doc}`../examples/18_illumination_analysis/illumination_analysis` checks
the direction of the Sun against the equinoxes and the solstices of 2026,
compares the closed form of the eclipse with a direct count along the
orbit, follows the beta angle and the eclipse duration of the reference
orbit over a year, builds the lighting timeline of the port and its
windows, shows how the windows change over four months and disappear around
a peak of the beta angle, and flies the final approach with and without the
go/no-go on lighting.

## References

1. D. A. Vallado, *Fundamentals of Astrodynamics and Applications*, 4th
   edition, Microcosm Press, 2013.
2. *The Astronomical Almanac*, United States Naval Observatory and His
   Majesty's Nautical Almanac Office.
3. J. R. Wertz, D. F. Everett, J. J. Puschell (eds.), *Space Mission
   Engineering: The New SMAD*, Microcosm Press, 2011.
