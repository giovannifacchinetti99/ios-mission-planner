# Monte Carlo analysis

A closed-loop simulation with nominal hardware shows that the mission can
work. It does not show how often it will. The thrusters of the real vehicle
will not deliver exactly their nominal thrust and will not be mounted
exactly along the body axes, the center of mass will not lie exactly on the
thrust lines, the relative sensor will be mounted with a small error, the
chief will not hold exactly its nominal attitude, and the deputy will not
start exactly at its hold point. None of these deviations is known in
advance, and the docking has to succeed for all the combinations of them
that are likely to occur.

A Monte Carlo analysis answers by repetition. The mission is flown many
times with the simulation of {doc}`../gnc/simulator`, each time with its
own draw of the uncertain quantities and its own sequence of sensor noise,
and the results are treated as a statistical sample. This chapter describes
what is drawn, what can be concluded from a finite number of runs, and how
this analysis relates to the collision probability of
{doc}`../safety/safety`.

## What is dispersed

The uncertain quantities are of two kinds, and the difference decides how
they are drawn.

Some are fixed for a given vehicle and a given day but unknown: the scale
error and the misalignment of the thrusters, the offset of the center of
mass, the mounting error of the sensor, the attitude error of the chief,
the density of the atmosphere, the state in which the approach begins and
the error of the first navigation estimate. Each is drawn once per run,
from a distribution that describes what is known about it, and stays
constant during the run. These are the dispersions.

The others change all the time: the noise of the gyro, of the star tracker
and of the relative sensor, the variation of the thrust from pulse to
pulse. Each run draws its own sequence of them.

Both come from a random generator started from the seed of the run. The
same seed always reproduces the same run, to the last digit, which is what
makes a campaign useful beyond its statistics: any run, and in particular
any failure, can be flown again alone and examined in detail.

The distributions are normal, with zero mean and a standard deviation for
each quantity, independent from one quantity to another; the atmospheric
density, which varies by factors and not by differences, is multiplied by a
log-normal factor. These are assumptions, and the results depend on them.
A standard deviation is a statement about the hardware and about how well
it has been measured, and belongs to the scenario as much as a mass or a
thrust level.

## Estimating a probability

Each run ends in success, a docking inside the contact limits, or in
something else. If $k$ runs out of $n$ succeed, the natural estimate of the
probability of success is the observed frequency,

$$
\hat p = \frac{k}{n}.
$$

It is an estimate from a sample, and another campaign of the same size
would give a different value. The runs are independent and each succeeds
with the same unknown probability $p$, so $k$ follows a binomial
distribution, with standard deviation $\sqrt{np(1-p)}$. The uncertainty of
the estimate decreases as $1/\sqrt n$: halving it takes four times as many
runs.

The estimate must come with an interval. The textbook interval, $\hat p \pm
z\sqrt{\hat p(1-\hat p)/n}$, replaces the unknown $p$ in the standard
deviation by its estimate. It fails precisely in the case of interest. When
every run succeeds, $\hat p = 1$, the interval has zero width, and the
analysis would claim that the mission cannot fail, on the strength of a few
dozen runs.

Wilson's interval avoids the substitution. It asks for which values of $p$
the observed frequency would be within $z$ standard deviations of $p$,
using the standard deviation that belongs to that $p$:

$$
\left|\hat p - p\right| \le z\sqrt{\frac{p(1-p)}{n}}.
$$

This is a quadratic inequality in $p$, and its solution is the interval

$$
p \in \frac{1}{1 + z^2/n}\left[\hat p + \frac{z^2}{2n} \;\pm\; z\sqrt{\frac{\hat
p(1-\hat p)}{n} + \frac{z^2}{4n^2}}\right],
$$

with $z = 1.96$ for a confidence of 95 percent. It is never of zero width
and never leaves the interval from 0 to 1.

## What a campaign without failures can claim

When all $n$ runs succeed, the upper end of Wilson's interval is 1 and the
lower end reduces to

$$
p_{low} = \frac{n}{n + z^2}.
$$

This is the strongest statement that the campaign supports: the probability
of success is at least $p_{low}$, with the chosen confidence. It also gives
the size of the campaign needed for a requirement. To demonstrate a
probability of success of at least $p_{req}$ with runs that all succeed,

$$
n \ge z^2\,\frac{p_{req}}{1 - p_{req}}.
$$

At 95 percent confidence, 73 runs demonstrate 0.95, 381 demonstrate 0.99,
and 3838 demonstrate 0.999. Each additional nine costs ten times as many
runs. A campaign of a few hundred runs is therefore a statement about the
ordinary behaviour of the system, and cannot say anything about failures
that happen once in ten thousand flights. Those need other methods: the
analysis of specific failure cases, of which the injected failures of
{doc}`../gnc/simulator` are examples.

The same reasoning applies in reverse to a campaign with a few failures. A
single failure in 200 runs gives an estimate of 0.995 and an interval that
extends from about 0.97 to 0.999: the campaign does not show that the
probability is 0.995, it shows that it is very probably above 0.97.

## Distributions and margins

The probability of success is one number. The campaign also gives the
distribution of every quantity of the mission, and two of them are used
directly.

The conditions at contact, the lateral offset, the speeds, the angles and
the rates, each have a limit. The distance between the worst value of the
campaign and the limit is the margin of the design, and it says more than
the count of successes: a campaign in which every run succeeds with the
lateral offset at nine tenths of its limit is not as reassuring as one in
which it stays below a tenth.

The velocity change of the approach enters the budget of {doc}`budget`.
The nominal run gives the expected consumption, and the budget needs what
must be on board: a high percentile of the distribution, the 99th for
example. The difference between that percentile and the nominal value is a
margin obtained from the physics of the mission, and it replaces, for the
approach, the generic percentage that the budget applies to a maneuver that
has only been estimated.

## Where the design breaks

A campaign in which everything works shows that there is margin and not
where it ends. The complementary analysis degrades one element of the
scenario at a time, a noisier sensor, coarser thrusters, a narrower field
of view, a missing function, and repeats a smaller campaign for each. The
elements whose degradation makes the success rate fall are the ones on
which the mission depends, and the level at which it falls is the
requirement to place on them.

This is a sensitivity analysis done by simulation. It is cruder than a
formal one and it has an advantage: it finds the interactions that nobody
thought of listing, because the simulation contains them. The saturation of
the wheels by a center of mass offset, found in the closed-loop notebook,
is of this kind.

## Two ways of propagating uncertainty

The safety chapter computed a collision probability by drawing an initial
uncertainty and propagating each sample with the transition matrix of the
linear model, without corrections. This chapter flies the complete loop for
each sample. The two are both called Monte Carlo and answer different
questions.

The linear propagation is fast, thousands of samples in a fraction of a
second, and it describes a deputy that executes a burn and then does
nothing: no measurement, no correction. It is the right model for the
question of passive safety, what happens if control is lost, and a
pessimistic one for a mission that navigates, because it lets every initial
error grow freely for the whole propagation.

The closed-loop campaign takes seconds per sample and describes the mission
as it is flown. Errors do not grow freely: the navigation observes them and
the controller removes them, so the dispersion at the port is set by the
accuracy of the sensor at short range and by the resolution of the
thrusters, and not by the uncertainty at the beginning. This is why a
mission whose screening gives a collision probability of a few percent at
one burn can have a docking that succeeds in almost every run of a campaign. The
two numbers do not contradict each other. One measures what an uncontrolled
deputy would do, the other what the controlled one does, and a safe mission
needs both to be acceptable.

## In the code

The functions are in `sim/monte_carlo.py`. `Dispersions` holds the standard
deviation of each dispersed quantity, and `sample_config` returns a copy of
a base scenario with the dispersed quantities drawn for a given seed.
`run_monte_carlo` flies a campaign, on one process or distributed over
several, and returns a `MonteCarloResult` with, for every run, the outcome,
the reason of an abort, the conditions at contact, the velocity change in
total and by phase, the propellant, the duration and a coarse sampling of
the trajectory. Its methods give the success rate, its interval, the count
of each outcome and the statistics of the contact conditions.
`wilson_interval` computes the interval for any number of successes and
trials.

## In the notebook

{doc}`../examples/17_monte_carlo_docking/monte_carlo_docking` flies the
reference mission 200 times with dispersed hardware and initial conditions.
It reports the outcomes and the interval of the probability of success,
with the curve of what a campaign without failures can claim as a function
of its size, a failure found by an earlier version of the campaign, flown again alone
to find its cause, and the side effect of its repair, the distributions of
the conditions at contact against their limits, the distribution of the velocity change in total and by phase, and
the trajectories. It then degrades the scenario one element at a time to
find where the design stops working.

## References

1. E. B. Wilson, "Probable Inference, the Law of Succession, and
   Statistical Inference," *Journal of the American Statistical
   Association*, 22(158), 1927.
2. L. D. Brown, T. T. Cai, A. DasGupta, "Interval Estimation for a
   Binomial Proportion," *Statistical Science*, 16(2), 2001.
3. W. Fehse, *Automated Rendezvous and Docking of Spacecraft*, Cambridge
   University Press, 2003.
