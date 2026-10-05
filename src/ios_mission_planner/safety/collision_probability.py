import numpy as np

from ..dynamics.relative.cw import cw_state_transition_matrix


def collision_probability(mean_state, covariance, n, t_eval, zone, n_samples=20000, rng=None):
    """
    Monte Carlo collision probability of a relative state against a
    keep-out zone, under navigation and burn-execution uncertainty.

    Samples are drawn from a Gaussian around `mean_state` and propagated
    through the linear closed-form Clohessy-Wiltshire state transition
    matrix (not the nonlinear equations, so this stays cheap for a large
    number of samples). A sample counts as a collision if it falls inside
    the zone at any of the given times, so `zone` doubles here as the hard
    body used for the probability and as the deterministic keep-out region
    checked elsewhere in this package.

    Parameters
    ----------
    mean_state : array_like, shape (6,)
        Nominal relative state [x, y, z, vx, vy, vz] at t=0.
    covariance : array_like, shape (6, 6)
        Covariance of the relative state at t=0, combining navigation and
        burn-execution uncertainty.
    n : float
        Mean motion of the chief [rad/s].
    t_eval : array_like, shape (m,)
        Times at which to check for a collision [s].
    zone : KeepOutEllipsoid
        The hard body: a sample collides if it ever falls inside it.
    n_samples : int
    rng : numpy.random.Generator, optional

    Returns
    -------
    float
        Fraction of samples that enter the zone at any of the given times.

    Author: Giovanni Facchinetti, 2026
    """

    rng = rng if rng is not None else np.random.default_rng()
    samples = rng.multivariate_normal(mean_state, covariance, size=n_samples)

    collided = np.zeros(n_samples, dtype=bool)
    for t in t_eval:
        phi = cw_state_transition_matrix(t, n)
        propagated = samples @ phi.T
        collided |= zone.is_violated(propagated[:, :3])

    return float(collided.mean())
