# The mission budget

The previous parts produced, one at a time, the pieces of the reference
mission: the transfer from the parking orbit, the approach, the hold on a
closed ellipse, the final approach, and the tools to convert a $\Delta v$
into propellant. This part looks at the mission as a whole, and it starts
with the question on which every other depends: how much propellant the
mission needs, with enough margin to remain valid while the design is still
changing.

## From maneuvers to a budget

A $\Delta v$ budget is a table with one line per maneuver or group of
maneuvers. For the reference mission the lines are the phasing and transfer
from the parking orbit, the approach from the first hold point to the
closed ellipse, the station keeping on the ellipse for the planned
duration, the final approach to contact, and the disposal at the end of
the mission. Each nominal value comes from one of the previous chapters:
the Hohmann and plane-change formulas and the two-impulse targeting of
{doc}`../maneuvers/impulsive`, and the station-keeping rate of
{doc}`../safety/safety`. The final approach is budgeted here as a
two-impulse transfer, a first estimate that a closed-loop simulation of the
approach refines.

A sum of nominal values is not a budget. Each value carries an uncertainty
that depends on how the number was obtained, and the budget must hold when
the numbers move.

## Margins

Space projects handle this with margins applied in two layers.

The first layer is applied line by line and reflects the maturity of each
number. A maneuver computed with a validated model and verified against a
nonlinear simulation deserves a small margin; one that rests on an
assumption not yet analyzed deserves a large one. With a fractional margin
$m_k$ on the line $k$,

$$
\Delta v_k^{\mathrm{marg}} = \Delta v_k\,(1 + m_k).
$$

The values used here follow the practice of European concurrent design
studies: 2 percent for a maneuver that is flight proven, 10 percent for one
that has been analyzed and validated, 20 percent for a preliminary
estimate.

The second layer is a system margin applied to the total, which covers what
no single line accounts for: maneuvers not yet identified, contingencies,
and the growth that every design experiences,

$$
\Delta v_{\mathrm{tot}} = (1 + m_{\mathrm{sys}})\sum_k
\Delta v_k^{\mathrm{marg}} .
$$

The two layers answer different questions and are not interchangeable. The
line margins say how much each number is trusted; the system margin says
how much the list itself is trusted.

## From the budget to the mass

The rocket equation of {doc}`../maneuvers/propulsion` converts the total
into mass. With the dry mass $m_f$ fixed, the mass at the start of the
mission and the propellant are

$$
m_0 = m_f\,\exp\!\left(\frac{\Delta v_{\mathrm{tot}}}{I_{sp}\,g_0}\right),
\qquad m_p = m_0 - m_f .
$$

For a small total, the propellant mass is nearly proportional to
$\Delta v_{\mathrm{tot}}/I_{sp}$, so the choice between chemical and
electric propulsion changes it by almost an order of magnitude. The choice
is not free, however: the low thrust of an electric system changes how the
maneuvers themselves must be flown, as the chapter on propulsion showed for
the disposal burn, and the short, precise impulses of the final approach
need a chemical or cold-gas system in any case.

## What drives the budget

Three features of the reference mission are worth noting, because they
generalize.

The plane change dominates whenever it is present. A coplanar transfer of
100 km in altitude costs about 56 m/s. A plane change of five degrees costs
several hundred. No other line comes close, and no margin policy
compensates for a launch into the wrong plane.

The small lines are the uncertain ones. Station keeping and final approach
contribute little to the total, but they are the lines that depend on
perturbations, on navigation errors and on the behaviour of the control
loop, and they are the ones that a simple formula cannot provide. Their
nominal values have to come from simulation.

A funded line is not a successful maneuver. The disposal burn can be fully
covered by the budget and still fail to deorbit the spacecraft if it is
flown as a continuous spiral instead of with thrust arcs. The budget counts
$\Delta v$; whether that $\Delta v$ achieves its purpose is a property of
the maneuver strategy.

## In the code

The budget tools are in the `budget` package. `BudgetLine` in
`budget/margins.py` holds a label, a nominal $\Delta v$ and a fractional
margin, and provides the margined value. `MATURITY_MARGIN` contains the
three default margins by level of maturity, and `total_delta_v` adds the
margined lines and applies the system margin. The conversion to mass uses
`initial_mass` and `propellant_mass` from `budget/propellant.py`.

## In the notebook

{doc}`../examples/16_mission_budget/mission_budget` assembles the budget of
the reference mission from the parking orbit to disposal. It rebuilds the
approach, adds the station keeping for a hold of two days, a final approach
and a deorbit, applies the two layers of margin, and converts the total
into propellant for a chemical and for an electric system. It quantifies
the cost of a plane change of five degrees, and compares the impulsive
deorbit with a continuous low-thrust one, showing the difference between
matching the energy of a burn and matching its purpose.

## References

1. European Cooperation for Space Standardization, *ECSS-E-ST-10C: System
   engineering general requirements*, 2017.
2. European Space Agency, *Margin philosophy for science assessment
   studies*, SRE-PA/2011.097, 2012.
3. J. R. Wertz, D. F. Everett, J. J. Puschell (eds.), *Space Mission
   Engineering: The New SMAD*, Microcosm Press, 2011.
