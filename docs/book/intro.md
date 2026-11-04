# IOS Mission Planner

This book is the theoretical companion to the `ios-mission-planner` code. It is
not a replacement for the function docstrings, which stay the place to look up
what a specific call does, what it expects, and in which units. This book
exists for the layer underneath that: where the formulas come from, why they
take the form they do, and how they connect to each other across the project.
Each chapter ends with a worked example notebook, which is the place to see
the theory applied to an actual case rather than just stated.

## What the project does

`ios-mission-planner` analyzes in-orbit servicing (IOS) missions, in which a
servicer spacecraft (the "deputy") approaches, inspects, or berths with a
target spacecraft (the "chief"). The analysis is organized in four layers,
each building on the one before it.

The first layer is dynamics: how the chief and deputy move, both in an
absolute sense as they orbit the Earth, and relative to each other, meaning
the deputy's motion as seen from the chief. This is the subject of this
book's first two chapters.

The second layer builds relative orbit elements and safety analysis on top of
that dynamics: a compact, geometric way to describe and design relative
orbits such as passively safe ellipses, hold points, and approach corridors.

The third layer is maneuver design and propagation, turning a desired
relative trajectory into actual burns, whether impulsive two-impulse
targeting or continuous low-thrust steering, with the resulting propellant
cost budgeted through the rocket equation.

The fourth and final layer is mission-level analysis, stringing maneuvers
together into a full mission timeline and budget, validated against real
tracked objects.

## How to read this book

Each chapter develops the theory with the same notation and conventions used
in the code, cites the textbook or paper each result comes from, and links
directly to the source file and function that implements it. The worked
example notebook at the end of a chapter is the same notebook you can open and
re-run yourself under `examples/` in the repository; it is embedded here
read-only, with its already-computed outputs, so that this book does not need
the project's full numerical environment (heyoka, in particular, is a
conda-only dependency) to be built.
