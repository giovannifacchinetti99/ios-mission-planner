# A real case: Cosmos 2612 and ICEYE-X36

Every trajectory in the previous chapters was designed within this
project. This chapter applies the same tools to one that was not: a
proximity event between two real satellites, reconstructed from public
tracking data. In mid-October 2026 the Russian satellite Cosmos 2612
maneuvered to within a few kilometres, and at times a few hundred metres,
of ICEYE-X36, a commercial radar imaging satellite. The event was tracked
by LeoLabs and reported in the press; the links are in the notebook. The
question here is whether the relative-motion and safety tools of this book,
pointed at someone else's maneuver, can reconstruct it.

## From tracking data to trajectories

The public description of an orbit is the two-line element set (TLE): a
snapshot of the mean orbital elements of an object at one epoch, fitted to
tracking observations and published, in this case, by Space-Track. A TLE is
meant to be propagated with one specific analytical model, SGP4, which
includes the secular and periodic effects of the oblateness of the Earth
and a simple drag model. The elements of a TLE are mean elements in the
sense of that model, and using them with any other propagator gives errors
of kilometres. This is the one place in the project where the propagation
is not done with the dynamics of {doc}`../foundations/perturbations`.

A single TLE is accurate only near its epoch, to about a kilometre at the
epoch and degrading by kilometres per day. A reconstruction over several
days therefore uses the whole history of element sets published for each
object, and at every instant propagates the set with the nearest epoch.
The result is a continuous trajectory with small discontinuities where the
source switches from one set to the next. These seams are an artifact of
the method. When one of them is much larger than the others, however, it
indicates that the orbit changed between the two fits, which is how a
maneuver shows up.

## The relative motion

With both objects reduced to inertial position and velocity at each
instant, the analysis is the one of {doc}`../relative/hcw`. The exact
transformation to the LVLH frame gives the state of Cosmos 2612 relative to
ICEYE-X36, taken as the chief because it is the object that does not
maneuver. A coarse scan of the whole period locates the closest approaches
and a fine scan around them gives the shape of the relative trajectory.

The shape can be read with the vocabulary of the linear model. Before the
largest seam the relative trajectory is a small, closed loop around the
chief, the 2:1 ellipse of a deliberately designed hold. After it, the loop
opens into a slow drift along the track, the signature of a small
difference in semi-major axis.

The accuracy of the data limits what can be concluded. With errors of the
order of a kilometre in each trajectory, distances of a few hundred metres
are at the edge of what the reconstruction can resolve: the geometry and
the timing of the event are reliable, the exact minimum distance is not.

## The safety check and the maneuver signature

The keep-out check of {doc}`../safety/safety` applies to this trajectory as
to any other. Against an illustrative zone of one kilometre around
ICEYE-X36, the reconstructed trajectory is inside the zone for several
hours.

The history of the elements contains a second, independent piece of
evidence. Each element set is the result of a fit, so a maneuver appears as
a step in the fitted elements between two consecutive sets. The
eccentricity and the mean motion of Cosmos 2612 show two such episodes in
the period of the event, each resolved to within a day and each followed by
a return toward the previous values, which is consistent with corrections
of a close relative orbit and not with a single transfer.

## What the reconstruction shows

The closest-approach scan, the shape of the relative orbit, the violation
of the keep-out zone and the steps in the elements are four readings of the
same public data, and they agree on the same days. None of them needs more
than a free Space-Track account and tools built for a different purpose.
From the point of view of the dynamics, designing a rendezvous and
recognizing one from the outside are the same problem.

## In the code

This chapter adds no function to the package. The propagation of the
element sets uses the `sgp4` library; the loading of the element history
and the choice of the nearest set are small helper functions defined in the
notebook. Everything after that uses `inertial_to_lvlh`, `KeepOutEllipsoid`
and `check_trajectory` unchanged.

## In the notebook

{doc}`../examples/20_cosmos2612_iceye_x36_rpo/cosmos2612_iceye_x36_rpo`
loads the two element histories, saved next to the notebook, reconstructs
the relative geometry over the period of the event, plots the relative
orbit of the closest approach, applies the keep-out check, and extracts the
maneuver signature from the element history. It also gives the query used
to download the data and the links to the public reporting.

## References

1. D. A. Vallado, P. Crawford, R. Hujsak, T. S. Kelso, "Revisiting
   Spacetrack Report #3," AIAA 2006-6753, 2006.
2. F. R. Hoots, R. L. Roehrich, *Spacetrack Report No. 3: Models for
   propagation of NORAD element sets*, 1980.
