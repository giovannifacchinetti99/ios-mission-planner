import numpy as np

from ..dynamics.relative.hill import hill_equations_symbolic
from ..propagation.heyoka_propagator import propagate
from .keepout import check_trajectory


def check_passive_safety(state_before_burn, n, zone, duration, pts=400):
    """
    Check whether a burn's failure to execute would be passively safe.

    Coasts the relative state right before a planned burn forward, as if
    the burn never happened, for `duration`, integrating the true Hill
    equations, and checks the resulting ballistic trajectory against a
    keep-out zone. A burn is passively safe when this ballistic coast stays
    clear of the zone for the whole duration, giving that much time to
    detect the failure and react before any danger of collision.

    Parameters
    ----------
    state_before_burn : array_like
        Relative state [x, y, z, vx, vy, vz] right before the burn, i.e.
        before its delta-v is applied.
    n : float
        Mean motion of the chief [rad/s].
    zone : KeepOutEllipsoid
    duration : float
        How long to coast after the failed burn before considering it
        safe [s].
    pts : int

    Returns
    -------
    violated : bool
    min_margin : float
        Same sign convention as keepout.check_trajectory: positive means
        the ballistic coast enters the zone, negative means it stays clear.
    t, states : the coast's time samples and relative states [pts, 6].

    Author: Giovanni Facchinetti, 2026
    """

    t = np.linspace(0.0, duration, pts)
    states = propagate(hill_equations_symbolic(), state_before_burn, t, pars=[n]).y.T

    violated, min_margin, _ = check_trajectory(states[:, :3], zone)

    return violated, min_margin, t, states
