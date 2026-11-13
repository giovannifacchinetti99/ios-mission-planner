# The mission analysis report

The chapters of this book build and validate the analysis of a mission one
piece at a time, with a notebook for each piece. A project needs the
opposite as well: a single document that takes the description of a
mission and returns its assessment. This chapter describes how the
analyses of the previous chapters are assembled automatically into such a
document, starting from a description of the mission in a text file.

## A mission described as data

The input is a scenario: the orbit of the chief, the parking orbit and the
propulsion of the deputy, the size of the closed ellipse on which it holds,
the duration of the hold, the final approach, the disposal, the parameters
of the safety checks, and the thresholds against which the result is
judged. The scenario is a text file in YAML format. A new mission is a new
file, and no code has to be written.

The scenario describes one *shape* of mission, the one followed throughout
this book: a coplanar rendezvous, or one with a single plane change, from a
parking orbit to a closed ellipse, a hold, a final approach and a disposal.
A mission with a different architecture needs a different pipeline, not
only different numbers.

## A fast screening

The first level of analysis uses only closed-form results, so that it runs
in seconds and can be repeated over many scenarios. The $\Delta v$ of the
transfer and of the disposal come from the Hohmann formulas, those of the
approach and of the final approach from the two-impulse targeting, the
station keeping from an assumed rate, and the propellant from the rocket
equation with the margins of {doc}`budget`. The trajectory that results is
then subjected to the checks of {doc}`../safety/safety`: keep-out zone,
passive safety, approach corridor, and collision probability.

This speed has a stated price. The approach is computed in the linear
model, in which a single two-impulse transfer reaches the ellipse exactly.
Flown with the nonlinear dynamics, the same approach needs a second, small
correcting burn. The screening does not model that correction, and it says
so in its output whenever the approach is a significant part of the
budget.

## A verdict made of explicit rules

The screening ends with one of three verdicts. The mission is *not
feasible* if any deterministic safety check fails, if the collision
probability exceeds the threshold of the scenario, or if the required mass
exceeds the allowed one. It is *feasible with caveats* if it passes all of
these but the analysis has raised a warning, such as the approximation
described above. It is *feasible* otherwise.

There is deliberately no score and no weighting behind the verdict. Every
line of it refers to one number and to the threshold next to it, so that
the reader can verify it and, more importantly, can see what to change.
The baseline scenario of the reference mission illustrates the point. It
passes every deterministic check with margin and its propellant is a small
fraction of the allowed mass, and it is judged not feasible, because its
collision probability at the insertion onto the ellipse is above the
threshold of one percent. A second scenario, identical except for a better
relative navigation, passes. The report identifies navigation accuracy as
the quantity that decides the feasibility of this mission, which is the
kind of answer an analysis is for.

## The document

The result is rendered as a self-contained HTML document from a template:
the scenario, the budget table, the safety checks with their margins, the
figures, and the verdict with its reasons. The figures are embedded in the
file, so the report is a single file that can be sent as it is. The same
document can be converted to PDF.

## From the screening to the full analysis

The screening is fast because it avoids everything that takes time: it does
not fly the approach, it does not simulate sensors or controllers, and it
judges safety with a linear propagation. A mission that passes it deserves
the complete analysis, and the report of that analysis is built in the same
spirit, from three inputs produced by the chapters before this one: a
closed-loop simulation with nominal hardware, from {doc}`../gnc/simulator`;
a Monte Carlo campaign, from {doc}`monte_carlo`; and the lighting of the
docking port, from {doc}`illumination`.

The requirements are again explicit. The campaign must demonstrate a
required probability of docking inside the contact limits, which means that
the lower end of the confidence interval of the success rate, and not the
rate itself, must reach the requirement. The velocity change at a high
percentile of the campaign must be within what is available. And the final
approach must be flown in favourable lighting.

The verdict follows the same three levels. The mission is not feasible if
the nominal run does not dock, if the observed probability of success is
below the requirement, or if the propellant does not suffice. It is
feasible with caveats when it passes those but something must be known: a
campaign too small to demonstrate the required probability, some runs that
ended in an abort, a contact condition that uses a large part of its limit,
a final approach flown in the dark. The distinction between the first and
the second case of the probability matters. A campaign of 60 runs without
a failure does not show that the probability is below the requirement, and
it does not show that it is above 0.95 either: the honest statement is that
more runs are needed, and the report says so.

The document contains the verdict with its reasons, the scenario, the
timeline of the nominal run with the velocity change of each phase, the
conditions at contact, the statistics of the campaign, and the lighting
windows with the position of the final approach among them. The report of
the reference mission, as produced by the notebook of this chapter, is
<a href="../examples/19_mission_analysis_report/docking_mission_report.html">here</a>.

## In the code

The pipeline is in the `report` package. `MissionScenario` in
`report/scenario.py` is the data structure of the scenario, and
`load_scenario` reads it from a YAML file. `run_feasibility_analysis` in
`report/pipeline.py` runs the screening and returns a `FeasibilityResult`
with the budget lines, the propellant and mass, the outcome of each safety
check, the collision probability, the verdict and the list of reasons and
warnings. `render_html` in `report/render.py` fills the template, and
`render_pdf` converts the document to PDF with
[WeasyPrint](https://weasyprint.org/).

The full analysis is in `report/mission_report.py`. `MissionRequirements`
holds the requirements, `analyze_mission` takes a `SimulationResult`, a
`MonteCarloResult` and a `LightingTimeline` and returns a `MissionAnalysis`
with the verdict, the issues and the caveats, and `render_mission_report`
produces the document, which `render_pdf` converts as well.

## In the notebook

{doc}`../examples/19_mission_analysis_report/mission_analysis_report` runs
the baseline scenario of the reference mission through the screening, shows
why it is judged not feasible, and then runs the scenario with improved
navigation, which passes. It generates the HTML and PDF documents for both.
It then runs the full analysis of the approach and docking, with a
closed-loop simulation, a Monte Carlo campaign and the lighting of the
port, and generates the mission analysis report.

## References

1. European Cooperation for Space Standardization, *ECSS-E-ST-10C: System
   engineering general requirements*, 2017.
2. J. R. Wertz, D. F. Everett, J. J. Puschell (eds.), *Space Mission
   Engineering: The New SMAD*, Microcosm Press, 2011.
