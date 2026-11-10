# A real case study: Cosmos 2612 and ICEYE-X36

Every previous chapter works with trajectories this project itself
designed. This chapter turns the same tools on a trajectory nobody in this
project designed: a real, publicly reported proximity event between the
Russian satellite Cosmos 2612 and ICEYE-X36, a commercial radar satellite
operated by the Finnish company ICEYE. In mid-October 2026, Cosmos 2612
maneuvered to within roughly 500 metres to a few kilometres of ICEYE-X36
over several days, tracked and reported publicly by LeoLabs and covered by
outlets including Ars Technica, SpaceNews/Via Satellite, and The War Zone
(full links are in the notebook itself). The question this chapter asks is
whether the dynamics, relative-motion, and safety tools built in the rest
of this book, pointed at someone else's maneuver instead of a designed
mission, can reconstruct that event from nothing but freely available
tracking data.

## From public tracking data to a continuous trajectory

A two-line element set (TLE) is a compact, standardized snapshot of an
object's orbit at one epoch, fit from tracking observations by whoever
publishes it, here Space-Track.org. Propagating one forward is done with
SGP4, the analytical propagator every TLE is implicitly paired with; this
project uses the `sgp4` Python package for that step rather than anything
derived in `ios_mission_planner` itself, since SGP4's own internal
perturbation model is a separate piece of machinery from the two-body-plus-J2
dynamics built in the absolute dynamics chapter.
Reference: Vallado, Crawford, Hujsak, Kelso, "Revisiting Spacetrack Report
#3," AIAA 2006-6753.

A single TLE is only accurate for a limited span around its own epoch, so
reconstructing a multi-day trajectory needs more than one. Space-Track's
`gp_history` endpoint returns every element set fit for an object in a date
range, not just one per day, several a day exactly where an encounter like
this one matters. The notebook's `load_tle_history` and `nearest_sat`
helpers turn that into a continuous estimate the simplest way possible: at
each requested time, propagate from whichever element set in the history has
the closest epoch. This works well inside each element set's own span of
validity, but it means the reconstructed trajectory has a visible seam
wherever the nearest element set switches from one fit to the next; that
seam is an artifact of the reconstruction method, not necessarily a
maneuver, though in this event it happens to line up with one.

## Reusing this project's own relative-motion and safety tools

With both objects reduced to inertial position and velocity at any instant,
everything from here on reuses functions already covered in earlier
chapters, unmodified. `inertial_to_lvlh`, from the relative dynamics
chapter, turns the pair of inertial states into a relative one in ICEYE-X36's
LVLH frame, with ICEYE-X36 treated as the chief and Cosmos 2612 as the
deputy, following the same convention used everywhere else in this
project: the non-maneuvering object is the reference. A coarse scan across
the whole reported window locates the closest approach, and a finer scan
around it, every 30 seconds for 12 hours, reconstructs the actual shape of
the pass in the radial/along-track plane, the same view `cw_fundamentals.ipynb`
uses for every relative orbit this project designs on purpose. The
reconstructed pass shows two distinct regimes either side of the seam
mentioned above: a small, tight, bounded loop beforehand, the same kind of
shape a deliberately designed drift-free hold produces, and a longer,
shallower, drifting arc afterward.

The same keep-out check from the safety chapter, `check_trajectory`, then
applies directly to this reconstructed pass against a generous,
illustrative 1 km `KeepOutEllipsoid`, exactly as it would to one of this
project's own designed approaches. It reports a real violation, not a
hypothetical one: the reconstructed trajectory spends several hours inside
that zone.

## Spotting a maneuver from the element history alone

Each TLE is already the result of an orbit-fitting process, so any
maneuver that happened before a given element set's own fit epoch is
silently absorbed into it. That makes a sequence of element sets a crude
but genuinely useful maneuver detector: a real burn shows up as a step
change in orbital elements between consecutive sets, bracketed in time by
nothing more precise than the gap between them. Reading Cosmos 2612's
eccentricity and mean motion across every element set in the reported
window surfaces exactly two such episodes, each resolved to well under a
day thanks to Space-Track's several-sets-per-day coverage during the event,
each reverted rather than kept, consistent with correction burns on an
already-close approach rather than one continuous transfer.

## What the four independent reads agree on

The closest-approach scan, the reconstructed relative orbit's shape, the
keep-out violation, and the element-history anomalies are four genuinely
independent ways of reading the same public data, and all four agree on the
same few days in mid-October. None of them required anything beyond a free
Space-Track account and tools this project built for a different purpose
entirely, designing its own missions rather than reading someone else's.
That is itself worth stating plainly: the line between planning a
rendezvous and recognizing one from the outside, from this toolkit's point
of view, is thinner than it looks.

## See it in action

The worked example for this chapter is
{doc}`../examples/10_cosmos2612_iceye_x36_rpo/cosmos2612_iceye_x36_rpo`, which carries out every step
above in full, including the source links for the public reporting and the
exact Space-Track query used to fetch the element histories saved alongside
the notebook.
