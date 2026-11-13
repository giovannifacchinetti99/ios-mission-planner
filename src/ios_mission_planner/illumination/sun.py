import numpy as np

AU_KM = 149597870.7  # astronomical unit [km]


def julian_date(year, month, day, hour=0, minute=0, second=0.0):
    """
    Julian date of a calendar date and time (Gregorian calendar, valid
    from 1900 to 2100).

    The time scale is not distinguished here: the difference between UTC,
    UT1 and the dynamical time is below the accuracy of the low-precision
    Sun position this date is meant for.

    Author: Giovanni Facchinetti, 2026
    Reference: Vallado, Fundamentals of Astrodynamics and Applications
    """

    jd = (
        367.0 * year
        - np.floor(7.0 * (year + np.floor((month + 9.0) / 12.0)) / 4.0)
        + np.floor(275.0 * month / 9.0)
        + day
        + 1721013.5
    )

    return jd + ((second / 60.0 + minute) / 60.0 + hour) / 24.0


def julian_date_from_datetime(moment):
    """Julian date of a datetime.datetime, taken as UTC."""

    return julian_date(
        moment.year, moment.month, moment.day, moment.hour, moment.minute,
        moment.second + moment.microsecond * 1e-6,
    )


def sun_position(jd):
    """
    Position of the Sun with respect to the Earth, in the Earth-centered
    inertial frame, from the low-precision series of the Astronomical
    Almanac: mean longitude and mean anomaly of the Sun, the equation of
    the center to two terms, and the obliquity of the ecliptic. The
    direction is accurate to about 0.01 degrees between 1950 and 2050,
    far better than any lighting analysis needs.

    Parameters
    ----------
    jd : float or numpy.ndarray
        Julian date.

    Returns
    -------
    numpy.ndarray, shape (3,) or (n, 3)
        Position of the Sun [km].

    Author: Giovanni Facchinetti, 2026
    Reference: Vallado, Fundamentals of Astrodynamics and Applications
    (algorithm for the position of the Sun); The Astronomical Almanac
    """

    jd = np.asarray(jd, dtype=float)
    t = (jd - 2451545.0) / 36525.0

    mean_longitude = np.deg2rad(280.460 + 36000.771 * t)
    mean_anomaly = np.deg2rad(357.5291092 + 35999.05034 * t)

    ecliptic_longitude = mean_longitude + np.deg2rad(
        1.914666471 * np.sin(mean_anomaly) + 0.019994643 * np.sin(2.0 * mean_anomaly)
    )
    distance = AU_KM * (
        1.000140612 - 0.016708617 * np.cos(mean_anomaly) - 0.000139589 * np.cos(2.0 * mean_anomaly)
    )
    obliquity = np.deg2rad(23.439291 - 0.0130042 * t)

    return np.stack([
        distance * np.cos(ecliptic_longitude),
        distance * np.cos(obliquity) * np.sin(ecliptic_longitude),
        distance * np.sin(obliquity) * np.sin(ecliptic_longitude),
    ], axis=-1)


def sun_direction(jd):
    """Unit vector from the Earth to the Sun, see sun_position."""

    position = sun_position(jd)

    return position / np.linalg.norm(position, axis=-1, keepdims=True)


def orbit_normal(inclination, raan):
    """
    Unit vector normal to an orbit plane (direction of the orbital
    angular momentum), from the inclination and the right ascension of
    the ascending node [rad].
    """

    raan = np.asarray(raan, dtype=float)

    return np.stack([
        np.sin(inclination) * np.sin(raan),
        -np.sin(inclination) * np.cos(raan),
        np.cos(inclination) * np.ones_like(raan),
    ], axis=-1)


def beta_angle(sun_unit, normal_unit):
    """
    Beta angle: the angle between the direction of the Sun and the orbit
    plane, positive when the Sun is on the side of the orbit normal
    [rad]. At beta = 0 the Sun lies in the orbit plane and the eclipses
    are the longest; above a critical value the orbit is in sunlight all
    the time.

    Author: Giovanni Facchinetti, 2026
    """

    return np.arcsin(np.clip(np.sum(np.asarray(sun_unit) * np.asarray(normal_unit), axis=-1), -1.0, 1.0))
