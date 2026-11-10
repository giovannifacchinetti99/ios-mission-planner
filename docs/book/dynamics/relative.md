# Relative dynamics

Where the previous chapter described how a single spacecraft moves around the
Earth, this chapter describes how one spacecraft (the deputy) moves *as seen
from* another (the chief). This is the frame a servicer's guidance actually
cares about: not "where am I in an Earth-centered frame," but "how far am I
from the target, and in which direction."

## The LVLH frame

Code: {py:func}`lvlh_rotation
<ios_mission_planner.dynamics.relative.lvlh.lvlh_rotation>`,
{py:func}`inertial_to_lvlh
<ios_mission_planner.dynamics.relative.lvlh.inertial_to_lvlh>`,
{py:func}`lvlh_to_inertial
<ios_mission_planner.dynamics.relative.lvlh.lvlh_to_inertial>` in
`src/ios_mission_planner/dynamics/relative/lvlh.py`.
Reference: *Analytical Mechanics of Space Systems*, Fourth Edition.

"As seen from the chief" needs a frame attached to the chief, and the
standard choice is the **local-vertical, local-horizontal (LVLH)** frame. Its
radial axis $\hat x$ runs along the chief's position vector, pointing away
from the Earth; its cross-track axis $\hat z$ runs along the chief's orbit
normal, $\vect h_c = \vect r_c \times \vect v_c$; and its along-track axis
$\hat y$ completes the right-handed triad, $\hat y = \hat z \times \hat x$,
which for a circular reference orbit points along the velocity.

This triad rotates with the chief as it orbits, so it is a non-inertial
frame. `lvlh_rotation` builds the $3\times3$ rotation matrix from inertial to
LVLH by stacking these three unit vectors as rows.

Converting a deputy's inertial state into the chief's LVLH frame is not just
a rotation of the position and velocity vectors: because the frame itself is
rotating at the instantaneous angular rate $\vect\omega = \vect h_c /
r_c^2$, an observer riding in the LVLH frame sees a velocity that differs
from the inertially-rotated velocity difference by the frame's own motion.
The **transport theorem** gives the correction,

$$
\vect\rho = R\,(\vect r_d - \vect r_c), \qquad
\dot{\vect\rho} = R\,(\vect v_d - \vect v_c) - \vect\omega \times \vect\rho,
$$

with $R$ the rotation matrix from `lvlh_rotation`. This is exactly what
`inertial_to_lvlh` computes, and `lvlh_to_inertial` inverts it the same way.
Note that this transformation is *exact*: it makes no assumption about the
chief's orbit being circular or about the deputy being close to the chief. It
is the general tool used to get in and out of the LVLH frame; the
simplification comes next.

## Linearizing the relative motion: the Hill-Clohessy-Wiltshire equations

Code: {py:func}`hill_equations_symbolic
<ios_mission_planner.dynamics.relative.hill.hill_equations_symbolic>` in
`src/ios_mission_planner/dynamics/relative/hill.py`.
Reference: *Analytical Mechanics of Space Systems*, Fourth Edition.

Writing out the full nonlinear equation of relative motion in the rotating
LVLH frame (differentiating $\vect\rho$ twice while accounting for the
frame's rotation, and substituting the two-body gravitational acceleration of
both chief and deputy from the previous chapter) gives an exact but nonlinear
and chief-orbit-dependent result. **Clohessy and Wiltshire's** key
simplification is to assume:

1. the chief is on a **circular** reference orbit, so its angular rate is
   constant, $n = \sqrt{\mu/a_c^3}$;
2. the deputy stays **close** to the chief ($|\vect\rho| \ll r_c$), so the
   difference in gravitational acceleration between the two can be
   linearized in $\vect\rho$ (a first-order Taylor expansion of $-\mu
   \vect r/r^3$ about the chief's position).

Under these two assumptions, the relative equations of motion decouple into
the linear, constant-coefficient **Hill-Clohessy-Wiltshire (HCW) equations**:

$$
\ddot x - 2n\dot y - 3n^2 x = 0, \qquad
\ddot y + 2n\dot x = 0, \qquad
\ddot z + n^2 z = 0.
$$

This is exactly `hill_equations_symbolic`, just written as first-order
accelerations ($a_x = 2n\dot y + 3n^2 x$, etc.) for heyoka's state-space form.
A few structural features are worth noting directly from the equations: the
in-plane motion ($x$, $y$) is coupled through the Coriolis-like $2n\dot
{(\cdot)}$ terms, since the frame is rotating, while the out-of-plane motion
$z$ is a completely decoupled, undamped harmonic oscillator at the chief's
orbital frequency $n$, so cross-track relative motion never interacts with
the in-plane motion at this order of approximation.

## The closed-form solution and its state transition matrix

Code: {py:func}`cw_state_transition_matrix
<ios_mission_planner.dynamics.relative.cw.cw_state_transition_matrix>`,
{py:func}`cw_targeting
<ios_mission_planner.dynamics.relative.cw.cw_targeting>` in
`src/ios_mission_planner/dynamics/relative/cw.py`.
Reference: [Wikipedia, Clohessy-Wiltshire equations](https://en.wikipedia.org/wiki/Clohessy%E2%80%93Wiltshire_equations);
Vallado, *Fundamentals of Astrodynamics and Applications*.

Being linear and constant-coefficient, the HCW equations admit a closed-form
solution: given an initial relative state $[\vect\rho_0, \dot{\vect\rho}_0]$,
the state at any later time $t$ is

$$
\begin{bmatrix}\vect\rho(t)\\ \dot{\vect\rho}(t)\end{bmatrix}
= \Phi(t)\begin{bmatrix}\vect\rho_0\\ \dot{\vect\rho}_0\end{bmatrix},
$$

with $\Phi(t)$ the $6\times6$ **state transition matrix**, built from
$c=\cos(nt)$, $s=\sin(nt)$:

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

This is exactly `cw_state_transition_matrix`. Being able to propagate
relative motion by a single matrix multiply, with no numerical integration,
is what makes the HCW model so useful for mission design even though it is
only an approximation: the chief's eccentricity and the deputy's distance
both have to stay small for it to hold, but within that regime it gives exact
analytic answers instantly.

### Two-impulse targeting

`cw_targeting` uses $\Phi(t)$ to solve the opposite problem: given a desired
arrival state $(\vect\rho_f, \dot{\vect\rho}_f)$ at a fixed time of flight
$t$, what two impulsive $\Delta v$'s (one at departure, one at arrival)
achieve it? Splitting $\Phi$ into its $3\times3$ position/velocity blocks,

$$
\vect\rho(t) = \Phi_{\rho\rho}\,\vect\rho_0 + \Phi_{\rho v}\,\dot{\vect\rho}_0^{+},
$$

so the departure velocity needed is $\dot{\vect\rho}_0^+ =
\Phi_{\rho v}^{-1}(\vect\rho_f - \Phi_{\rho\rho}\vect\rho_0)$, and the first
impulse is this minus the velocity the deputy actually had,
$\Delta v_1 = \dot{\vect\rho}_0^+ - \dot{\vect\rho}_0^-$. The arrival velocity
*before* the second impulse follows from the velocity row of $\Phi$, and
$\Delta v_2$ matches it to whatever is wanted on arrival. This only works
when $\Phi_{\rho v}$ is invertible; it loses rank at specific times of flight
(multiples of a full orbit for the in-plane block, half-orbits for the
out-of-plane one), where the required $\Delta v$ either blows up or is not
unique. This is a geometric consequence of the relative orbit returning to,
or passing through, its own starting point at those exact times regardless
of how it got there.

## Relative orbit elements: a geometric reading of the same solution

Code: {py:func}`relative_orbit_from_state
<ios_mission_planner.dynamics.relative.relative_orbit.relative_orbit_from_state>`,
{py:func}`state_from_relative_orbit
<ios_mission_planner.dynamics.relative.relative_orbit.state_from_relative_orbit>`
in `src/ios_mission_planner/dynamics/relative/relative_orbit.py`.
Reference: *Analytical Mechanics of Space Systems*, Fourth Edition.

The $\Phi(t)$ solution above is exact but not very intuitive to read at a
glance: it mixes the secular along-track drift with the periodic in-plane and
out-of-plane oscillations. Regrouping the same closed-form solution by the
*kind* of term it contains, rather than by $x$/$y$/$z$ component, gives a far
more geometric picture:

$$
x(t) = x_{\mathrm{off}} + \rho\cos(nt+\alpha), \qquad
y(t) = y_c + \dot y_{\mathrm{drift}}\,t - 2\rho\sin(nt+\alpha), \qquad
z(t) = \rho_z \cos(nt+\beta).
$$

In other words, in-plane the deputy traces a **2:1 ellipse** (its along-track
semi-axis is always exactly twice its radial semi-axis, a direct consequence
of the $2n\dot{(\cdot)}$ coupling in the HCW equations) of radial size $\rho$
and phase $\alpha$, centered at radial offset $x_{\mathrm{off}}$; if that
offset is non-zero, the whole ellipse drifts along-track at the constant rate
$\dot y_{\mathrm{drift}} = -\tfrac32 n\, x_{\mathrm{off}}$. Out-of-plane, the
motion is a simple, independent oscillation of amplitude $\rho_z$ and phase
$\beta$. The six numbers $(x_{\mathrm{off}}, y_c, \rho, \alpha, \rho_z,
\beta)$ are a direct, reversible reparametrization of the same Cartesian
relative state. This is the `RelativeOrbit` dataclass:
`relative_orbit_from_state` groups an instantaneous $[\vect\rho,
\dot{\vect\rho}]$ into these six numbers (by algebraically matching
coefficients between the Cartesian solution and the grouped form above), and
`state_from_relative_orbit` is the inverse.

The practical payoff is design, not just description. Setting
$x_{\mathrm{off}}=0$ by construction (via `state_from_relative_orbit`) gives
a **drift-free**, perfectly periodic relative orbit, the natural starting
point for a passively safe hold or inspection ellipse, since it requires no
station-keeping to stay bounded. This connects directly to the
safety-ellipse machinery used elsewhere in the project.

## See it in action

The worked example for this chapter is
{doc}`../examples/01_cw_fundamentals/cw_fundamentals`, which builds the LVLH frame, propagates
the HCW equations, decomposes the result into relative orbit elements, and
designs a drift-free relative orbit from scratch.
