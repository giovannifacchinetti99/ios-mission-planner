# IOS Mission Planner

`ios-mission-planner` is a Python toolkit for the analysis of in-orbit
servicing missions, in which a servicer spacecraft approaches a target
spacecraft, flies around it, and finally docks with it. This book is the
theoretical companion to the code. The docstrings remain the place to look up
what a specific function expects and returns; the book explains where the
formulas come from, why they take the form they do, and how one piece leads
to the next.

## The mission this book follows

Every chapter serves one reference mission. A target, called the **chief**
throughout, flies on a low Earth orbit at 500 km altitude and 51.6 degrees of
inclination. A servicer, called the **deputy**, is released on a lower
parking orbit, thousands of kilometres behind. The deputy has to catch up
with the chief, transfer to its orbit, close the remaining distance through a
sequence of hold points, and translate along a narrow corridor until its
docking port touches the port of the chief with a small, controlled contact
velocity. Along the way it must never be on a trajectory that would hit the
chief if a burn failed, it must keep its sensors pointed at the target, and
it must do all of this with a propellant budget that closes.

Answering "can this mission be flown, and how well" requires a chain of
models, and the book is ordered along that chain, from the simplest model to
the complete simulation.

## Road map

Part I, *Orbital foundations*, describes the motion of a single spacecraft
around the Earth: the two-body problem, the orbital elements, and the two
perturbations that matter most in low Earth orbit, the oblateness of the
Earth and atmospheric drag.

Part II, *Relative motion*, puts a second spacecraft on a nearby orbit and
describes how it moves as seen from the first. It starts from the linear
model for a circular chief, which gives the geometric vocabulary of every
rendezvous, and then removes its assumptions one at a time: an eccentric
chief, then perturbations.

Part III, *Maneuvers and propulsion*, turns a desired trajectory into burns.
It covers the classical impulsive maneuvers, the two-impulse targeting that
moves the deputy between two relative states, and what changes when the
thrust is low and continuous.

Part IV, *Safety*, asks whether a planned approach is acceptable, including
when something goes wrong: keep-out zones, passive safety, approach
corridors, and the probability of collision under navigation and execution
errors.

Part V, *Attitude*, adds the rotational motion that the previous parts
ignored: how a rigid body turns, how its orientation is controlled, how it is
estimated from sensors, and how it is tied to the relative geometry of the
approach.

Each chapter depends only on the chapters before it, and each begins by
saying what it takes from them.

## How a chapter is organized

Every chapter has the same three movements. It first develops the theory,
with the notation used in the code. It then describes, in a section called
*In the code*, which functions implement that theory and how they are meant
to be used. It closes with a section called *In the notebook*, which says
what the companion notebook does with those functions: every notebook
validates the model of its chapter against an independent reference, and the
plots and numbers live there, not in the chapter. The notebooks are the same
files found under `examples/` in the repository, numbered in the reading
order of this book, and they are embedded here with their computed outputs.

The assumptions and simplifications of the whole project are collected in a
single page, {doc}`limitations`, instead of being scattered across the
chapters.

## Conventions

Inertial positions and velocities are expressed in kilometres and kilometres
per second, in an Earth-centered inertial frame. Relative positions and
velocities are expressed in metres and metres per second, in the
local-vertical local-horizontal (LVLH) frame of the chief, whose $x$ axis is
radial (pointing away from the Earth), whose $z$ axis is along the orbit
normal, and whose $y$ axis completes the triad and points along the velocity
for a circular orbit. The $y$ axis is also called the V-bar and the $x$ axis
the R-bar. Angles are in radians inside the code and in degrees in the
plots. Attitude is represented by unit quaternions with the scalar part
first, in the Hamilton convention, describing the rotation from the inertial
frame to the body frame.

Vectors are written in bold, $\vect r$, their magnitude in plain italics,
$r$, and time derivatives with dots. The mean motion of the chief is always
$n$, and $\mu$ is the gravitational parameter of the Earth.
