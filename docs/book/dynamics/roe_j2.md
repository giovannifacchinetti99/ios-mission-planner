# Relative orbit elements under J2

The previous chapter built relative motion on the Hill-Clohessy-Wiltshire
model: a circular chief orbit, unperturbed by anything beyond the Earth's
point-mass gravity. Real low-Earth orbits precess under J2, as the absolute
dynamics chapter showed, and that precession does not cancel out just
because we are looking at a difference between two nearby orbits. If the
chief and deputy differ even slightly in semi-major axis, eccentricity, or
inclination, they precess at slightly different rates, and that differential
drift shows up as relative motion that the CW model has no way to represent:
its drift term $x_{\mathrm{off}}$ only captures a difference in semi-major
axis, and stays at zero even while J2 visibly pulls the deputy away from the
chief through a channel built entirely out of differential nodal and
apsidal precession. Capturing that channel needs a different set of relative
elements, built from the orbital elements themselves rather than from
Cartesian position and velocity, and a state transition matrix derived with
J2 included from the start.

## Quasi-nonsingular relative orbital elements

Code: {py:func}`quasi_nonsingular_roe
<ios_mission_planner.dynamics.relative.roe_j2.quasi_nonsingular_roe>`,
{py:func}`deputy_elements_from_roe
<ios_mission_planner.dynamics.relative.roe_j2.deputy_elements_from_roe>` in
`src/ios_mission_planner/dynamics/relative/roe_j2.py`.
Reference: Koenig, Guffanti, D'Amico, "New State Transition Matrices for
Spacecraft Relative Motion in Perturbed Orbits," *Journal of Guidance,
Control, and Dynamics*, 40(7), 2017, Eq. (2).

Given the classical elements $(a, e, i, \Omega, \omega, M)$ of chief and
deputy, the quasi-nonsingular ROE collect six differences between them:

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

Each of the six keeps a direct geometric meaning rather than being an
abstract combination of numbers. $\delta a$ is the fractional semi-major
axis difference, the single quantity that sets the along-track drift rate,
exactly as $x_{\mathrm{off}}$ did for the CW elements, but built from an
orbital element rather than from a Cartesian position. $\delta\lambda$ is a
relative mean longitude: essentially the along-track separation, expressed
as an angle, with a correction for the nodes being rotated apart whenever
the orbit is inclined. $(\delta e_x, \delta e_y)$ is the difference between
the chief's and the deputy's eccentricity vectors (the same eccentricity
vector from the absolute dynamics chapter, projected into $\cos\omega$,
$\sin\omega$ components), so it is zero exactly when the two orbits share
the same perigee direction and the same shape. $(\delta i_x, \delta i_y)$
plays the equivalent role for the orbit's orientation: $\delta i_x$ is simply
the inclination difference, and $\delta i_y$ is the node separation weighted
by $\sin i_c$, so that together they behave like a two-dimensional "relative
inclination vector." `quasi_nonsingular_roe` computes exactly these six
quantities from a pair of `ClassicalElements`, wrapping the two angle
differences into $(-\pi,\pi]$ so a RAAN difference near a full turn reads as
a small negative angle rather than a near-$2\pi$ one.
`deputy_elements_from_roe` is the inverse, useful for designing a formation
by specifying the ROE directly and solving for the deputy's actual orbital
elements.

## The secular drift that makes J2 matter here

Reference: Koenig, Guffanti, D'Amico (2017), Eq. (13).

J2 drives three secular, ever-growing drifts in the classical elements, the
Brouwer rates already used in the absolute dynamics chapter's discussion of
nodal regression and apsidal precession:

$$
\begin{pmatrix}\dot M\\ \dot\omega\\ \dot\Omega\end{pmatrix} =
\frac{3}{4}\frac{J_2 R_\oplus^2\sqrt\mu}{a^{7/2}\eta^4}
\begin{pmatrix}\eta(3\cos^2 i - 1)\\ 5\cos^2 i - 1\\ -2\cos i\end{pmatrix},
\qquad \eta=\sqrt{1-e^2}.
$$

Since each rate depends on $a$, $e$, and $i$, a chief and deputy with
slightly different orbital elements drift at slightly different rates, and
that differential drift is precisely what the ROE above are built to encode.
Writing $\kappa$ for the common prefactor in these rates, in particular the
argument-of-perigee rate $\dot\omega=\kappa(5\cos^2 i-1)$ turns out to govern
most of the structure of the state transition matrix below, since it sets
how fast the reference direction that $(\delta e_x,\delta e_y)$ is measured
against itself rotates.

## The J2-aware state transition matrix

Code: {py:func}`j2_quasi_nonsingular_stm
<ios_mission_planner.dynamics.relative.roe_j2.j2_quasi_nonsingular_stm>` in
`src/ios_mission_planner/dynamics/relative/roe_j2.py`.
Reference: Koenig, Guffanti, D'Amico (2017), Appendix A.3, Eq. (A6), combined
with the state transformation of Eq. (20)-(21), (25).

The paper derives a closed-form $6\times6$ state transition matrix for the
quasi-nonsingular ROE under the secular effect of J2, valid for a chief of
arbitrary eccentricity, from a first-order Taylor expansion of the relative
dynamics about zero separation. One subtlety in building it is that
$(\delta e_x,\delta e_y)$ is defined relative to the argument of perigee,
and the argument of perigee itself precesses at $\dot\omega$ over the
propagation interval. `j2_quasi_nonsingular_stm` handles this by evaluating
the paper's matrix, `phi_prime`, in a frame aligned with the perigee
direction at the start of the interval, then rotating into a frame aligned
with the (precessed) perigee direction at the end: `eccentricity_rotation`
builds that $2\times2$ rotation embedded in the $(\delta e_x, \delta e_y)$
block of an otherwise-identity $6\times6$ matrix, and the returned STM is
`j_final_inv @ phi_prime @ j_initial`. Everywhere else, the matrix couples
$\delta\lambda$, $\delta e_x$, $\delta e_y$, and $\delta i_y$ to $\delta a$,
$\delta e_x$, $\delta e_y$, and $\delta i_x$ at the start of the interval,
each coupling term built from the same $\kappa$ prefactor and the orbit
geometry factors $E$, $F$, $G$, $P$, $Q$, $R$, $S$, $T$ defined in the
function, which are exactly the shorthand the paper itself introduces for
combinations of $\eta$ and $\cos i$. The net result is a single matrix
multiplication, `roe(t+tau) = Phi @ roe(t)`, that predicts the J2-driven
evolution of the relative orbit elements across any time span, without
needing to numerically integrate chief and deputy separately and subtract.

## See it in action

The worked example for this chapter is
{doc}`../examples/j2_roe_state_transition`, which designs a deputy orbit
from a target set of ROE, propagates it against the closed-form STM, and
compares that analytical prediction against brute-force nonlinear
propagation of both chief and deputy under full two-body-plus-J2 dynamics.
