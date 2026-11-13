# Relative motion under perturbations

The two previous chapters treated the relative motion of two spacecraft
that feel only the point-mass attraction of the Earth. The chapter on
{doc}`../foundations/perturbations` showed that a real low Earth orbit
precesses under the oblateness of the Earth and decays under drag. Neither
effect cancels in the difference between two nearby orbits. This chapter
describes what survives the difference: a slow deformation of the relative
orbit under $J_2$, which needs a new set of relative variables to be
described, and a steady along-track drift under differential drag.

## Why the Cartesian description is not enough

In the linear model of {doc}`hcw`, the only source of drift is the radial
offset $x_{\mathrm{off}}$, the image of a difference in semi-major axis.
Under $J_2$, a deputy placed on a closed ellipse with $x_{\mathrm{off}}=0$
drifts anyway, while $x_{\mathrm{off}}$ stays close to zero. The drift
comes from the secular rates of the node, of the perigee and of the mean
anomaly, which depend on $a$, $e$ and $i$: two orbits that differ slightly
in eccentricity or inclination precess at slightly different rates, and the
difference accumulates. The Cartesian relative state has no variable that
measures a difference in eccentricity or inclination directly, so it cannot
express this mechanism in a simple way. A description built from the
orbital elements of the two spacecraft can.

## Relative orbital elements

Given the classical elements of the chief and of the deputy, the
quasi-nonsingular relative orbital elements (ROE) are six combinations of
their differences:

$$
\delta\vect\alpha =
\begin{bmatrix}\delta a\\ \delta\lambda\\ \delta e_x\\ \delta e_y\\
\delta i_x\\ \delta i_y\end{bmatrix} =
\begin{bmatrix}
(a_d-a_c)/a_c\\
(M_d+\omega_d)-(M_c+\omega_c)+(\Omega_d-\Omega_c)\cos i_c\\
e_d\cos\omega_d - e_c\cos\omega_c\\
e_d\sin\omega_d - e_c\sin\omega_c\\
i_d-i_c\\
(\Omega_d-\Omega_c)\sin i_c
\end{bmatrix}.
$$

Each has a geometric meaning, and each corresponds to one of the features
of the relative orbit found in {doc}`hcw`. The relative semi-major axis
$\delta a$ is the quantity that sets the along-track drift, the same role
played by $x_{\mathrm{off}}$. The relative mean longitude $\delta\lambda$
is the mean along-track separation, expressed as an angle. The relative
eccentricity vector $(\delta e_x, \delta e_y)$ is the difference between
the eccentricity vectors of the two orbits; its magnitude is the size of
the in-plane ellipse and its direction the phase. The relative inclination
vector $(\delta i_x, \delta i_y)$ plays the same role for the cross-track
oscillation: its magnitude is the amplitude and its direction the phase.
Multiplied by $a_c$, each element becomes a length that can be compared
directly with the geometric quantities of the linear model.

For unperturbed motion all six are constant except $\delta\lambda$, which
drifts at a rate proportional to $\delta a$. This is the property that
makes them useful: the perturbations appear as slow changes of quantities
that would otherwise be constant.

## The effect of $J_2$ on the relative elements

The secular rates of the previous part, written with $\eta=\sqrt{1-e^2}$,
are

$$
\begin{pmatrix}\dot M\\ \dot\omega\\ \dot\Omega\end{pmatrix} =
\frac{3}{4}\frac{J_2 R_\oplus^2\sqrt\mu}{a^{7/2}\eta^4}
\begin{pmatrix}\eta(3\cos^2 i - 1)\\ 5\cos^2 i - 1\\ -2\cos i\end{pmatrix},
$$

where the first is the correction to the mean motion. Differencing them
between deputy and chief, to first order in the separation, gives the
evolution of the relative elements. Three effects stand out.

The relative eccentricity vector rotates. Both perigees precess at nearly
the same rate $\dot\omega$, so the vector $(\delta e_x,\delta e_y)$ keeps
its length and turns at that rate: the in-plane ellipse keeps its size
while its phase slowly changes.

The relative inclination vector drifts. A difference in inclination,
$\delta i_x$, means a difference in nodal rate, so the two nodes separate
steadily and $\delta i_y$ grows linearly with time.

The relative mean longitude drifts. The same difference in inclination, and
a difference in eccentricity, change the mean motion of the deputy relative
to the chief, so $\delta\lambda$ drifts even when $\delta a = 0$. This is
the channel through which a closed ellipse opens under $J_2$, and it is
invisible to $x_{\mathrm{off}}$.

These effects combine into a closed-form state transition matrix,

$$
\delta\vect\alpha(t+\tau) = \Phi_{J_2}(\tau)\,\delta\vect\alpha(t),
$$

valid for a chief of any eccentricity, which predicts the secular evolution
of the relative orbit over any time span with a single matrix product. The
one subtlety in its construction is that the relative eccentricity vector
is referred to the perigee of the chief, which itself moves: the matrix is
evaluated in axes aligned with the perigee at the start of the interval and
then rotated to the axes aligned with the perigee at the end.

For the design of a formation, the matrix says what to avoid. A relative
orbit with $\delta i_x = 0$, that is, with the out-of-plane separation
obtained entirely from a difference in node, has no differential nodal
drift, and one with parallel relative eccentricity and inclination vectors
keeps a minimum separation in the plane normal to the track however the
along-track position evolves. These are the conditions used to make
close formations passively safe under $J_2$.

## Differential drag

Drag acts on the relative motion in a simpler way. The decay rate of a
circular orbit is proportional to the ballistic coefficient,

$$
\dot a = -\rho\,\frac{C_dA}{m}\,\sqrt{\mu a},
$$

so two spacecraft on the same orbit with different ballistic coefficients
decay at different rates. The difference in semi-major axis grows linearly
with time,

$$
\Delta\dot a = -\rho\,\Delta\!\left(\frac{C_dA}{m}\right)\sqrt{\mu a},
$$

and a difference in semi-major axis is a drift along the track at the rate
$-\tfrac32 n\,\Delta a$. Since $\Delta a$ itself grows linearly, the
along-track separation grows with the square of the time. The spacecraft
with more drag sinks, speeds up, and moves ahead. The radial separation
stays small in comparison, and the cross-track separation is not affected:
differential drag acts along the track.

The effect starts small and builds up: for two spacecraft at 450 km whose
ballistic coefficients differ by a factor of three, the separation is a few
kilometres after ten orbits and four times as much after twenty. It is also
free. A spacecraft can change its
ballistic coefficient by changing its attitude, presenting a larger or
smaller cross-section to the flow, and can therefore control its
along-track position relative to another without propellant. Several
operational constellations keep their spacing this way.

## In the code

The relative elements are in `dynamics/relative/roe_j2.py`.
`quasi_nonsingular_roe` computes them from the classical elements of chief
and deputy, wrapping the angular differences so that two nearly equal
angles on opposite sides of a full turn give a small difference, and
`deputy_elements_from_roe` is the inverse, used to design a formation by
choosing the relative elements. `j2_quasi_nonsingular_stm` returns the
closed-form transition matrix for a given chief orbit and time span.

Differential drag needs no model of its own. It is obtained by propagating
the two spacecraft with `two_body_drag_dynamics_symbolic`, or with
`two_body_j2_drag_dynamics_symbolic` when the oblateness is included, each
with its own ballistic coefficient, and transforming the result to the LVLH
frame of the chief.

## In the notebook

{doc}`../examples/04_j2_roe_state_transition/j2_roe_state_transition`
designs a deputy orbit from a chosen set of relative elements, propagates
both spacecraft with the nonlinear dynamics including $J_2$, and compares
the evolution of the relative elements with the prediction of the
transition matrix.

{doc}`../examples/05_differential_drag/differential_drag` starts two
spacecraft on the same orbit with different ballistic coefficients,
measures the along-track separation that develops, and shows how it scales
with the difference between the two coefficients.

## References

1. A. W. Koenig, T. Guffanti, S. D'Amico, "New state transition matrices
   for spacecraft relative motion in perturbed orbits," *Journal of
   Guidance, Control, and Dynamics*, 40(7), 2017.
2. S. D'Amico, *Autonomous Formation Flying in Low Earth Orbit*, PhD
   thesis, Delft University of Technology, 2010.
3. C. L. Leonard, W. M. Hollister, E. V. Bergmann, "Orbital formationkeeping
   with differential drag," *Journal of Guidance, Control, and Dynamics*,
   12(1), 1989.
