# Relative motion about an eccentric chief

The Hill-Clohessy-Wiltshire model of the previous chapter rests on two
assumptions: a circular chief and a small separation. This chapter removes
the first and keeps the second. Most targets are not exactly circular, and
it is worth knowing what that does to a relative orbit designed as if they
were.

## What changes when the chief is eccentric

In the circular case the LVLH frame turns at the constant rate $n$ and the
chief stays at a constant radius. On an eccentric orbit neither is true:
the chief moves closer to the Earth and faster near perigee, further and
slower near apogee. Its angular rate $\dot\theta$ and its radius $r_c$ vary
along the orbit, but they are still known exactly at every instant, because
they belong to the two-body motion of the chief alone, which nothing in the
relative problem disturbs. The orbit plane does not move, so the angular
velocity of the frame is still directed along the orbit normal,

$$
\vect\omega = \dot\theta\,\hat{\vect z}, \qquad \dot\theta =
\frac{h_c}{r_c^2},
$$

with $h_c$ the constant angular momentum of the chief.

## The linearized equations

Applying the transport theorem twice to the acceleration of the deputy,
subtracting the acceleration of the chief, and expanding the difference of
the gravitational terms to first order in $\vect\rho$, exactly as in the
circular derivation, gives

$$
\ddot x = 2\dot\theta\,\dot y + \ddot\theta\,y +
\left(\dot\theta^2 + \frac{2\mu}{r_c^3}\right) x, \qquad
\ddot y = -2\dot\theta\,\dot x - \ddot\theta\,x +
\left(\dot\theta^2 - \frac{\mu}{r_c^3}\right) y, \qquad
\ddot z = -\frac{\mu}{r_c^3}\,z .
$$

Compared with the HCW equations there is one new kind of term, proportional
to the angular acceleration of the frame,

$$
\ddot\theta = -2\,\dot\theta\,\frac{\dot r_c}{r_c}, \qquad
\dot r_c = \frac{\vect r_c\cdot\vect v_c}{r_c},
$$

and the coefficients, which were constants, now vary periodically with the
position of the chief along its orbit. For a circular chief, $\dot\theta =
n$, $\ddot\theta = 0$ and $\mu/r_c^3 = n^2$, and the equations reduce
exactly to the HCW equations. That reduction is also the most direct check
of an implementation.

The equations are still linear in the relative state, so the superposition
of solutions still holds, but they no longer have constant coefficients,
and the simple transition matrix of the previous chapter does not apply.
Closed-form solutions exist with the true anomaly as the independent
variable (the Tschauner-Hempel equations and the Yamanaka-Ankersen
transition matrix); this project integrates the equations numerically
instead, together with the motion of the chief.

## What eccentricity does to a relative orbit

Two things follow, and they are easy to confuse.

The first is that the linearization itself does not get worse. The only
approximation left is the smallness of the separation, the same as in the
circular model, and the linear equations track the nonlinear truth to a
small fraction of the separation for a close relative orbit, whatever the
eccentricity of the chief.

The second is that a design made with the circular model stops working even
when the linearization is excellent. The drift-free condition $\dot y_0 =
-2n\,x_0$ expresses the equality of the two orbital periods for a circular
chief. Applied to an eccentric chief, the same initial state corresponds to
slightly different periods, and the relative orbit no longer closes: each
revolution is displaced from the previous one. The correct condition is
still the equality of the two semi-major axes, but written in the variables
of the eccentric problem it is no longer the simple formula above.

The lesson is general and returns in the next chapter with a different
cause. A relative orbit designed to be drift-free in one dynamical model is
drift-free only to the extent that the assumptions of that model hold.

## In the code

`eccentric_hill_equations_symbolic`, in
`dynamics/relative/eccentric_hill.py`, returns a single system of twelve
states: the six inertial states of the chief, advanced with the two-body
acceleration, and the six relative states of the deputy, advanced with the
equations above. The quantities $r_c$, $\dot\theta$ and $\ddot\theta$ are
built as symbolic expressions of the chief state, so the model is exact in
the motion of the chief, with no expansion in the eccentricity, and
linearized only in the separation.

## In the notebook

{doc}`../examples/03_eccentric_chief_relative_motion/eccentric_chief_relative_motion`
runs three checks. It verifies that the model reduces to the HCW equations
for a circular chief. It validates the model against the nonlinear truth,
obtained by propagating the two spacecraft separately and transforming to
the LVLH frame. Finally it flies the same drift-free design against a
circular and an eccentric chief and shows that the second does not close.

## References

1. H. Schaub, J. L. Junkins, *Analytical Mechanics of Space Systems*, 4th
   edition, AIAA, 2018.
2. J. Tschauner, P. Hempel, "Rendezvous zu einem in elliptischer Bahn
   umlaufenden Ziel," *Astronautica Acta*, 11(2), 1965.
3. K. Yamanaka, F. Ankersen, "New state transition matrix for relative
   motion on an arbitrary elliptical orbit," *Journal of Guidance, Control,
   and Dynamics*, 25(1), 2002.
