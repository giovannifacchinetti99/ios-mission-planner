# Failures the main chain cannot see

The earlier parts built a chain that guides the deputy to the port and
protects itself: its mission manager watches the corridor, the closing
speed and the health of the sensors, and aborts when they go wrong. The
Monte Carlo campaigns measured how well that chain docks when its hardware
is imperfect. This chapter asks a different question: what happens when the
failure is not in the hardware but in what the software believes, and why
the answer calls for a second, independent layer of protection.

## Common-mode failures

The abort logic of the mission manager compares the navigation estimate
with its limits. It is a good protection against failures that the
estimate reveals: a thruster stuck open pushes the deputy off the corridor,
the navigation sees it, the abort fires. It is no protection against
failures of the estimate itself, because then the guidance and the check
look at the same wrong picture and agree with each other. Engineers call
such a failure common-mode: it defeats a protection by striking the
function it relies on.

The DART demonstrator of NASA collided in 2005 with the satellite it was
approaching after a navigation error of this kind, and the investigation
concluded that collision avoidance should rely on information independent
of the navigation that guides the approach. The European cargo vehicle ATV
had followed that principle from the start, with a monitoring and safety
unit separate from its main computer.

## Three failures of what the software believes

The simulator can inject three failures of this kind, each from a start
time and for a duration, through `FailureConfig`.

A bias on the velocity estimate adds a constant vector to the velocity that
guidance, control and the abort logic receive, while the filter underneath
keeps tracking the truth. In the direction that makes the deputy believe it
closes more slowly than it does, the guidance accelerates to restore the
planned speed and the true closing speed exceeds the abort limit unseen.

A late brake removes, for a delay, the part of the command that slows the
deputy along the approach axis as it arrives at a hold point. The
navigation is right and the closing speed is small, so the abort logic,
which watches speeds, sees nothing; what is broken is the volume the deputy
may occupy before the go, as it coasts past the hold toward the chief.

A bias on the attitude estimate rotates the attitude that the controller
and the conversion of the force command use. If it appears in one step the
estimated pointing error jumps and the abort logic reacts; if it grows over
a minute or two, as a real drift would, the controller follows it, the
docking port points off the axis by the same angle, and the error is
noticed only if it grows large enough to push the target out of the field
of view of the relative sensor.

## Measuring them

The notebook of this chapter injects each failure into the final part of
the reference approach and sweeps its size, because the useful answer is
not whether a failure is dangerous but from which size on, and whether the
main chain notices it at that size. For the velocity bias the sweep goes
from 0.1 to 0.8 m/s, for the late brake from 30 to 120 s, for the attitude
error from 5 to 25 degrees. Every run is compared with the same nominal
run, with the same seed, so that the difference is the failure alone.

## A second layer

The remedy is a layer of protection that shares nothing with the main
navigation. The simulator provides a place for one: the `supervisor`
argument of `simulate`, a function called at every step after the main
chain has computed its force command and before the thrusters execute it.
It receives the time, the true relative state, the true attitude and the
force command, and nothing from the main chain's software, so that it
cannot lean on the very estimate it is meant to check. It uses the truth
only to simulate a sensor of its own, with its own errors and its own random
numbers, and it may replace the force command or ask the main chain to
abort.

The notebook builds the smallest useful example, a watchdog of the closing
speed. Its sensor measures the distance to the chief only, with an error of
2 cm plus 1 percent. It estimates the closing speed as the slope of a line
fitted to the last ten distances, compares it with the same speed profile
as the main abort, and asks for an abort when the profile is exceeded for
three consecutive seconds. Three design choices carry it: independence of
the measurement, a limit consistent with the main chain's, so that the two
layers disagree only when one of them is wrong, and persistence, so that the
noise of a single measurement never triggers an abort.

The watchdog is an illustration of the mechanism, not a safety unit. It
watches one quantity and would not catch the late brake or the attitude
error. A complete unit would estimate the full relative state from its own
sensor, check volumes, speeds, passive safety and plumes, decide with
persistence, act with graded authority and record what it did.

## In the code

The failures are the fields `navigation_velocity_bias_*`, `late_braking_*`
and `attitude_bias_*` of `sim.config.FailureConfig`, described in
{doc}`../gnc/simulator`. The supervisor interface is in `sim/supervisor.py`:
`SupervisorSample` is what the supervisor receives and `SupervisorDecision`
what it may return. `MissionManager.request_abort` is how an abort request
reaches the main chain, and `run_monte_carlo` accepts a
`supervisor_factory` to fly campaigns with a supervisor.

## In the notebook

{doc}`../examples/22_failures_and_supervision/failures_and_supervision`
measures the three failures against the nominal final approach, with a
sweep of each one's size, and flies the minimal watchdog on the nominal
approach, where it must stay silent, and on the approach with the velocity
bias, where it must turn a contact into an abort.

## References

1. NASA, [Overview of the DART Mishap Investigation
   Results](https://www.nasa.gov/pdf/148072main_DART_mishap_overview.pdf),
   2006.
2. W. Fehse, *Automated Rendezvous and Docking of Spacecraft*, Cambridge
   University Press, 2003.
