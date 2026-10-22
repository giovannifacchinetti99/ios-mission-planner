import numpy as np


def hohmann_transfer(r1, r2, mu):
    """
    Delta-v and time of flight for a Hohmann transfer between two
    coplanar circular orbits.

    Parameters
    ----------
    r1 : float
        Departure orbit radius.
    r2 : float
        Arrival orbit radius.
    mu : float
        Gravitational parameter of the central body, consistent units
        with r1, r2 (e.g. km, km^3/s^2 gives km/s velocities and seconds).

    Returns
    -------
    dv1, dv2 : float
        Delta-v of the departure (periapsis) and arrival (apoapsis) burns.
    time_of_flight : float
        Transfer time, half the period of the transfer ellipse.

    Author: Giovanni Facchinetti, 2026
    Reference: Vallado, Fundamentals of Astrodynamics and Applications
    """

    a_t = 0.5 * (r1 + r2)
    v1 = np.sqrt(mu / r1)
    v2 = np.sqrt(mu / r2)
    v_t1 = np.sqrt(mu * (2.0 / r1 - 1.0 / a_t))
    v_t2 = np.sqrt(mu * (2.0 / r2 - 1.0 / a_t))

    dv1 = v_t1 - v1
    dv2 = v2 - v_t2
    time_of_flight = np.pi * np.sqrt(a_t**3 / mu)

    return dv1, dv2, time_of_flight


def plane_change_dv(v, delta_i):
    """
    Delta-v of a pure plane change: rotating the velocity direction by
    delta_i at constant speed v.

    Parameters
    ----------
    v : float
        Speed at the maneuver point.
    delta_i : float
        Plane change angle [rad].

    Returns
    -------
    float

    Author: Giovanni Facchinetti, 2026
    Reference: Vallado, Fundamentals of Astrodynamics and Applications
    """

    return 2.0 * v * np.sin(delta_i / 2.0)


def combined_plane_change_dv(v1, v2, delta_i):
    """
    Delta-v of a single burn that changes speed from v1 to v2 and rotates
    the velocity direction (e.g. the orbital plane) by delta_i at the
    same time, by the law of cosines on the velocity triangle. Always at
    most dv_speed_only + plane_change_dv(v, delta_i), i.e. combining the
    two changes into one burn is never more expensive, and usually
    cheaper, than doing them as two separate burns.

    Parameters
    ----------
    v1, v2 : float
        Speed before and after the burn.
    delta_i : float
        Plane change angle [rad].

    Returns
    -------
    float

    Author: Giovanni Facchinetti, 2026
    Reference: Vallado, Fundamentals of Astrodynamics and Applications
    """

    return np.sqrt(v1**2 + v2**2 - 2.0 * v1 * v2 * np.cos(delta_i))
