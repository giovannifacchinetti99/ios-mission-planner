import numpy as np


def in_eclipse(position, sun_unit, body_radius):
    """
    Whether a spacecraft is in the shadow of the central body, with the
    cylindrical shadow model: the Sun is taken infinitely far, so the
    shadow is a cylinder of the radius of the body, on the side opposite
    to the Sun. The penumbra, a transition of a few seconds in low Earth
    orbit, is neglected.

    Parameters
    ----------
    position : array_like, shape (3,) or (n, 3)
        Position of the spacecraft with respect to the center of the
        body [km].
    sun_unit : array_like, shape (3,) or (n, 3)
        Unit vector toward the Sun.
    body_radius : float
        Radius of the body [km].

    Returns
    -------
    bool or numpy.ndarray of bool

    Author: Giovanni Facchinetti, 2026
    Reference: Vallado, Fundamentals of Astrodynamics and Applications
    """

    position = np.asarray(position, dtype=float)
    sun_unit = np.asarray(sun_unit, dtype=float)

    along = np.sum(position * sun_unit, axis=-1)
    across_sq = np.sum(position * position, axis=-1) - along**2

    return (along < 0.0) & (across_sq < body_radius**2)


def critical_beta(radius, body_radius):
    """
    Beta angle above which a circular orbit of the given radius has no
    eclipse [rad].
    """

    return np.arcsin(body_radius / radius)


def eclipse_fraction_circular(radius, beta, body_radius):
    """
    Fraction of a circular orbit spent in the cylindrical shadow of the
    central body, for a given beta angle.

    Measuring the position along the orbit by the angle u from the point
    opposite to the projection of the Sun on the orbit plane, the
    spacecraft is in shadow while cos(u) > sqrt(1 - (R/a)^2) / cos(beta),
    which is an arc of half-width arccos of that ratio.

    Parameters
    ----------
    radius : float
        Radius of the orbit [km].
    beta : float or numpy.ndarray
        Beta angle [rad].
    body_radius : float
        Radius of the body [km].

    Returns
    -------
    float or numpy.ndarray
        Between 0 and 0.5.

    Author: Giovanni Facchinetti, 2026
    """

    beta = np.asarray(beta, dtype=float)
    ratio = np.sqrt(1.0 - (body_radius / radius) ** 2) / np.maximum(np.cos(beta), 1e-12)

    return np.where(ratio < 1.0, np.arccos(np.clip(ratio, -1.0, 1.0)) / np.pi, 0.0)
