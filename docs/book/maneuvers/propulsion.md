# Propulsion: from delta-v to propellant

The previous chapter measured every maneuver in metres per second. A
spacecraft does not carry metres per second, it carries propellant, and
its thrusters do not deliver impulses, they deliver a finite force for a
finite time. This chapter makes both connections. It derives the relation
between $\Delta v$ and propellant mass, and then studies what changes when
the thrust is so low that a burn lasts for many orbits, which is the normal
situation with electric propulsion.

## The rocket equation

A thruster expels propellant at an exhaust velocity $v_e$, usually
expressed through the specific impulse, $v_e = I_{sp}\,g_0$, where $g_0$ is
the standard acceleration of gravity, a conversion constant by definition
and not the local gravity. A thrust force $F$ consumes propellant at the
rate

$$
\dot m = -\frac{F}{I_{sp}\,g_0}.
$$

Conservation of momentum between the spacecraft and the expelled mass,
integrated over a burn in which the mass goes from $m_0$ to $m_f$, gives
the Tsiolkovsky rocket equation,

$$
\Delta v = I_{sp}\,g_0 \ln\frac{m_0}{m_f}.
$$

It can be read in two directions. For a given initial mass, the fraction
that must be propellant is

$$
\frac{m_0 - m_f}{m_0} = 1 - \exp\!\left(-\frac{\Delta v}{I_{sp}\,g_0}
\right),
$$

and for a given dry mass $m_f$, the initial mass needed is $m_0 = m_f\,
\exp(\Delta v/(I_{sp}\,g_0))$. The second form is the useful one in mission
analysis, where the dry mass is fixed and the question is how much
propellant the total $\Delta v$ requires.

The specific impulse is the figure of merit. A chemical thruster with
$I_{sp}$ around 220 s and an electric thruster with $I_{sp}$ around 1800 s
need, for the same small $\Delta v$, propellant masses in the inverse ratio
of those two numbers. The price of the electric thruster is its thrust,
thousands of times lower.

## Finite thrust

An impulsive burn is the limit of a thrust arc whose duration is negligible
compared with the orbital period. A chemical thruster of a few newtons on a
150 kg spacecraft delivers one metre per second in under a minute, and the
approximation holds. An electric thruster of a tenth of a newton takes days
to deliver the tens of metres per second of a deorbit burn. During that
time the spacecraft completes many orbits, and the direction of the
velocity turns through a full circle at each of them.

Two steering laws matter here. A thrust held along a fixed inertial
direction is a faithful model only of a short burn. A thrust held along the
velocity, forward or backward, follows the orbit for as long as needed and
is the natural law for changing the orbital energy: the power delivered to
the orbit is $\vect F\cdot\vect v$, largest when the two are aligned.

## Energy and shape

Comparing a long tangential burn with the impulsive maneuver of the same
$\Delta v$ exposes a distinction that is easy to miss.

The two change the orbital energy by nearly the same amount. The rocket
equation fixes the $\Delta v$ that a given propellant mass provides, and
for thrust along the velocity that $\Delta v$ converts into energy at the
same rate wherever on the orbit it is applied, as long as the orbit stays
nearly circular. The final semi-major axis is therefore almost the same.

The shape of the final orbit is completely different. An impulse applied at
one point changes the orbit only at the opposite point: a retrograde
impulse on a circular orbit lowers the perigee and leaves the apogee where
it was, producing an eccentric orbit. The same $\Delta v$ applied
continuously around the orbit lowers every point by the same amount, and
the orbit spirals inward remaining almost circular.

For a deorbit this is decisive. The purpose of the burn is to bring the
perigee into the atmosphere. An impulsive burn sized to lower the perigee
from 500 km to 200 km does exactly that. The same propellant spent in a
continuous spiral lowers the whole orbit to about 350 km, still a stable
orbit. Matching the energy of a maneuver is not matching its purpose.

## Thrust arcs

The remedy needs no new physics. The thrust is applied only while the
spacecraft is within a chosen angular distance of one point of the orbit,
and switched off for the rest of the revolution. With a half-width of the
arc equal to $\pi$ the thrust is always on, and this is the continuous
spiral. As the arc narrows, the same total $\Delta v$ is concentrated
around one point, and the maneuver approaches the impulsive one: the
opposite side of the orbit comes down and the eccentricity grows.

The price is time. With an arc of half-width $\theta_h$ the thruster works
for a fraction $\theta_h/\pi$ of each orbit, so delivering the same
$\Delta v$ takes $\pi/\theta_h$ times as long. Accuracy in reaching the
target perigee and duration of the maneuver pull in opposite directions,
and the width of the arc is the design variable that trades one for the
other.

Given a propellant mass and a target perigee, the width of the arc that
reaches the target can be found by bisection, since the final perigee
decreases steadily as the arc narrows. When the duration of the maneuver
also matters, there is no single best width. The problem has two
objectives, the error on the perigee and the duration, and its solution is
a Pareto front: the set of arc widths for which neither objective can be
improved without worsening the other. The front is computed here with a
multi-objective evolutionary algorithm, NSGA-II, which evolves a population
of candidate widths toward it.

## In the code

The rocket equation is in `budget/propellant.py`: `mass_fraction` returns
the propellant fraction for a given $\Delta v$ and specific impulse,
`propellant_mass` multiplies it by an initial mass, and `initial_mass`
gives the mass needed at the start to deliver a $\Delta v$ on top of a
given dry mass.

The finite-thrust models are in `dynamics/orbital/low_thrust.py`, and all
of them carry the mass as a seventh state, depleted at the rate given
above. `low_thrust_dynamics_symbolic` holds the thrust along a fixed
inertial direction, `low_thrust_tangential_dynamics_symbolic` steers it
along the velocity, and `low_thrust_arc_tangential_dynamics_symbolic` adds
the arc: thrust and mass flow are multiplied by a smooth gate that is one
within the chosen half-width of a reference direction and zero elsewhere.
The gate is built from a hyperbolic tangent, so the system remains a smooth
expression that the integrator handles without detecting switching events.

The design functions are in `dynamics/orbital/low_thrust_design.py`.
`deorbit_with_arc_limited_thrust` flies a deorbit with a given arc width
and a propellant mass fixed by the equivalent impulsive burn, and
`solve_half_angle_for_target_perigee` bisects on the width until the
perigee reaches the target. `ArcHalfAnglePygmoProblem` states the
two-objective problem in the form expected by
[pygmo](https://esa.github.io/pygmo2/), the optimization library of the
European Space Agency, and `pareto_front_arc_half_angle` evolves it with
NSGA-II and returns the front.

## In the notebook

{doc}`../examples/07_low_thrust_arc_optimization/low_thrust_arc_optimization`
takes the disposal burn of the reference mission and flies it with electric
propulsion. It sweeps the width of the thrust arc and shows the semi-major
axis staying nearly constant while the perigee moves from the value of the
continuous spiral toward the target, solves for the width that reaches a
200 km perigee, and computes the Pareto front between accuracy and
duration. It also shows where the estimate of the duration stops being
reliable, for arcs much narrower than the one the problem needs.

## References

1. G. P. Sutton, O. Biblarz, *Rocket Propulsion Elements*, 9th edition,
   Wiley, 2017.
2. D. A. Vallado, *Fundamentals of Astrodynamics and Applications*, 4th
   edition, Microcosm Press, 2013.
3. K. Deb, A. Pratap, S. Agarwal, T. Meyarivan, "A fast and elitist
   multiobjective genetic algorithm: NSGA-II," *IEEE Transactions on
   Evolutionary Computation*, 6(2), 2002.
4. F. Biscani, D. Izzo, "A parallel global multiobjective framework for
   optimization: pagmo," *Journal of Open Source Software*, 5(53), 2020.
