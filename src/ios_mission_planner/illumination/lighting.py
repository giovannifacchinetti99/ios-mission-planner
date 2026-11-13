from dataclasses import dataclass

import numpy as np

from ios_mission_planner.dynamics.orbital.perturbations import j2_secular_rates
from ios_mission_planner.illumination.eclipse import in_eclipse
from ios_mission_planner.illumination.sun import beta_angle, orbit_normal, sun_direction


def lvlh_axes_circular(inclination, raan, argument_of_latitude):
    """
    Axes of the LVLH frame of a spacecraft on a circular orbit, as the
    rows of the rotation matrix from the inertial frame: radial,
    along-track, orbit normal. Accepts arrays of raan and argument of
    latitude and then returns one matrix per entry, shape (n, 3, 3).

    Author: Giovanni Facchinetti, 2026
    """

    raan = np.asarray(raan, dtype=float)
    u = np.asarray(argument_of_latitude, dtype=float)
    ci, si = np.cos(inclination), np.sin(inclination)
    co, so = np.cos(raan), np.sin(raan)
    cu, su = np.cos(u), np.sin(u)

    radial = np.stack([co * cu - so * su * ci, so * cu + co * su * ci, su * si], axis=-1)
    along = np.stack([-co * su - so * cu * ci, -so * su + co * cu * ci, cu * si], axis=-1)
    normal = np.stack([so * si, -co * si, ci * np.ones_like(u)], axis=-1)

    return np.stack([radial, along, normal], axis=-2)


def intervals(t, mask):
    """Intervals of t over which a boolean mask is true, as (start, end) pairs."""

    mask = np.asarray(mask, dtype=bool)
    if not mask.any():
        return []

    edges = np.diff(mask.astype(int))
    starts = list(np.flatnonzero(edges == 1) + 1)
    ends = list(np.flatnonzero(edges == -1))
    if mask[0]:
        starts.insert(0, 0)
    if mask[-1]:
        ends.append(len(mask) - 1)

    return [(float(t[i]), float(t[j])) for i, j in zip(starts, ends)]


@dataclass
class LightingTimeline:
    """
    Lighting conditions of a docking port along the orbit of the chief.

    Attributes
    ----------
    t : numpy.ndarray
        Time from the epoch [s].
    sun_lvlh : numpy.ndarray, shape (n, 3)
        Unit vector toward the Sun in the LVLH frame of the chief.
    eclipse : numpy.ndarray of bool
        Whether the chief is in the shadow of the Earth.
    incidence : numpy.ndarray
        Angle between the Sun and the outward normal of the port [rad]:
        below 90 degrees the Sun shines on the port.
    sun_to_boresight : numpy.ndarray
        Angle between the Sun and the optical axis of the camera of the
        deputy looking at the port [rad].
    beta : numpy.ndarray
        Beta angle [rad].
    favourable : numpy.ndarray of bool
        Whether the port is lit at an acceptable incidence and the Sun is
        outside the exclusion cone of the camera.

    Author: Giovanni Facchinetti, 2026
    """

    t: np.ndarray
    sun_lvlh: np.ndarray
    eclipse: np.ndarray
    incidence: np.ndarray
    sun_to_boresight: np.ndarray
    beta: np.ndarray
    favourable: np.ndarray

    def windows(self):
        """Favourable intervals, as a list of (start, end) times [s]."""

        return intervals(self.t, self.favourable)

    def favourable_for(self, t, duration):
        """
        Whether the conditions are favourable from t for at least
        duration seconds. This is the question a go/no-go check asks
        before starting a final approach.
        """

        inside = (self.t >= t) & (self.t <= t + duration)
        if not np.any(inside):
            return False

        return bool(np.all(self.favourable[inside]))


def port_lighting_timeline(radius, inclination, raan, argument_of_latitude, jd_epoch, duration, step,
                           mu, j2, r_eq, port_normal_lvlh=(0.0, -1.0, 0.0),
                           max_incidence=np.deg2rad(70.0), sun_exclusion=np.deg2rad(30.0)):
    """
    Lighting of a docking port fixed in the LVLH frame of a chief on a
    circular orbit, over a span of time.

    Three conditions decide whether a camera on the deputy can see a
    target on the port. The chief must be out of the shadow of the Earth.
    The Sun must shine on the port from a direction not too far from its
    normal, since a grazing light leaves long shadows and little
    contrast. And the Sun must not be close to the optical axis of the
    camera, which looks along the inward normal of the port, or the
    detector is blinded; this last condition can only fail when the port
    is in backlight, but it is reported separately because it also
    applies to sensors that do not need the port to be lit.

    The orbit is propagated as a circle whose node drifts at the secular
    J2 rate, which is all that matters for the direction of the Sun.

    Parameters
    ----------
    radius : float
        Radius of the orbit [km].
    inclination, raan, argument_of_latitude : float
        Orbit plane and position along the orbit at the epoch [rad].
    jd_epoch : float
        Julian date of the epoch.
    duration, step : float
        Span and sampling of the timeline [s].
    mu, j2, r_eq : float
        Gravitational parameter, J2 coefficient and equatorial radius of
        the central body.
    port_normal_lvlh : array_like, shape (3,)
        Outward normal of the port in the LVLH frame.
    max_incidence : float
        Largest acceptable angle between the Sun and the port normal
        [rad].
    sun_exclusion : float
        Half-angle of the cone around the camera axis that the Sun must
        stay out of [rad].

    Returns
    -------
    LightingTimeline

    Author: Giovanni Facchinetti, 2026
    """

    t = np.arange(0.0, duration + 0.5 * step, step)
    n = np.sqrt(mu / radius**3)
    raan_dot, argp_dot, mean_dot = j2_secular_rates(radius, 0.0, inclination, mu, j2, r_eq)

    raan_t = raan + raan_dot * t
    u_t = argument_of_latitude + (n + mean_dot + argp_dot) * t

    axes = lvlh_axes_circular(inclination, raan_t, u_t)
    sun = sun_direction(jd_epoch + t / 86400.0)
    sun_lvlh = np.einsum("nij,nj->ni", axes, sun)

    position = radius * axes[:, 0, :]
    eclipse = in_eclipse(position, sun, r_eq)

    normal = np.asarray(port_normal_lvlh, dtype=float)
    normal = normal / np.linalg.norm(normal)
    incidence = np.arccos(np.clip(sun_lvlh @ normal, -1.0, 1.0))
    sun_to_boresight = np.arccos(np.clip(sun_lvlh @ (-normal), -1.0, 1.0))

    beta = beta_angle(sun, orbit_normal(inclination, raan_t))
    favourable = (~eclipse) & (incidence <= max_incidence) & (sun_to_boresight >= sun_exclusion)

    return LightingTimeline(
        t=t, sun_lvlh=sun_lvlh, eclipse=eclipse, incidence=incidence,
        sun_to_boresight=sun_to_boresight, beta=beta, favourable=favourable,
    )


def beta_angle_history(radius, inclination, raan, jd_epoch, days, mu, j2, r_eq, step_days=0.25):
    """
    Beta angle over a long span, with the node of the orbit drifting at
    the secular J2 rate and the Sun moving along the ecliptic. The two
    motions together make the beta angle oscillate with a period of
    weeks to months, and with it the duration of the eclipses and the
    lighting of an approach.

    Parameters
    ----------
    radius : float
        Radius of the (circular) orbit [km].
    inclination, raan : float
        Inclination and right ascension of the node at the epoch [rad].
    jd_epoch : float
        Julian date of the epoch.
    days : float
        Span [days].
    mu, j2, r_eq : float
        Gravitational parameter, J2 coefficient and equatorial radius of
        the central body.
    step_days : float
        Sampling [days].

    Returns
    -------
    t_days : numpy.ndarray
    beta : numpy.ndarray
        Beta angle [rad].

    Author: Giovanni Facchinetti, 2026
    """

    t_days = np.arange(0.0, days + 0.5 * step_days, step_days)
    raan_dot, _, _ = j2_secular_rates(radius, 0.0, inclination, mu, j2, r_eq)
    raan_t = raan + raan_dot * t_days * 86400.0

    sun = sun_direction(jd_epoch + t_days)

    return t_days, beta_angle(sun, orbit_normal(inclination, raan_t))
