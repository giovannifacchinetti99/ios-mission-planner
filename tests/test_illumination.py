from datetime import datetime

import numpy as np

from ios_mission_planner.constants import J2_EARTH, MU_EARTH, R_EARTH
from ios_mission_planner.dynamics.orbital.elements import ClassicalElements, cartesian_state_from_elements
from ios_mission_planner.dynamics.relative.lvlh import lvlh_rotation
from ios_mission_planner.illumination.eclipse import critical_beta, eclipse_fraction_circular, in_eclipse
from ios_mission_planner.illumination.lighting import (
    beta_angle_history,
    intervals,
    lvlh_axes_circular,
    port_lighting_timeline,
)
from ios_mission_planner.illumination.sun import (
    AU_KM,
    beta_angle,
    julian_date,
    julian_date_from_datetime,
    orbit_normal,
    sun_direction,
    sun_position,
)

RADIUS = R_EARTH + 500.0


def test_julian_date_reference_epochs():
    assert julian_date(2000, 1, 1, 12) == 2451545.0
    # worked example in Vallado
    assert np.isclose(julian_date(1996, 10, 26, 14, 20, 0.0), 2450383.09722222, atol=1e-8, rtol=0.0)
    assert julian_date_from_datetime(datetime(2026, 3, 20, 14, 46)) == julian_date(2026, 3, 20, 14, 46)


def test_sun_is_on_the_equator_at_the_equinoxes_and_highest_at_the_solstices():
    # instants of the 2026 equinoxes and solstices (UTC)
    events = {
        (2026, 3, 20, 14, 46): (0.0, 0.0),
        (2026, 6, 21, 8, 25): (23.436, 90.0),
        (2026, 9, 23, 0, 5): (0.0, 180.0),
        (2026, 12, 21, 20, 50): (-23.436, 270.0),
    }

    for instant, (declination, right_ascension) in events.items():
        sun = sun_direction(julian_date(*instant))
        assert np.isclose(np.degrees(np.arcsin(sun[2])), declination, atol=0.02)
        measured = np.degrees(np.arctan2(sun[1], sun[0])) % 360.0
        assert min(abs(measured - right_ascension), 360.0 - abs(measured - right_ascension)) < 0.02


def test_sun_distance_over_the_year():
    days = np.arange(366)
    distance = np.linalg.norm(sun_position(julian_date(2026, 1, 1) + days), axis=1) / AU_KM

    # perihelion in the first days of January, aphelion in the first days of July
    assert np.argmin(distance) < 6 and np.isclose(distance.min(), 0.9833, atol=2e-4)
    assert 182 < np.argmax(distance) < 190 and np.isclose(distance.max(), 1.0167, atol=2e-4)


def test_beta_angle_from_the_orbit_normal():
    normal = orbit_normal(np.deg2rad(51.6), np.deg2rad(40.0))
    elements = ClassicalElements(a=RADIUS, e=0.0, i=np.deg2rad(51.6), raan=np.deg2rad(40.0), argp=0.0,
                                 mean_anomaly=1.0)
    state = cartesian_state_from_elements(elements, MU_EARTH)
    momentum = np.cross(state[:3], state[3:])

    assert np.allclose(normal, momentum / np.linalg.norm(momentum), atol=1e-12)
    assert np.isclose(beta_angle(normal, normal), np.pi / 2)
    assert np.isclose(beta_angle(np.cross(normal, [0.0, 0.0, 1.0]) / np.sin(np.deg2rad(51.6)), normal), 0.0)


def test_eclipse_fraction_matches_a_direct_count():
    u = np.linspace(0.0, 2.0 * np.pi, 200001)
    position = RADIUS * np.stack([np.cos(u), np.sin(u), np.zeros_like(u)], axis=-1)

    for beta_deg in (0.0, 30.0, 60.0, 67.0, 75.0):
        beta = np.deg2rad(beta_deg)
        sun = np.array([np.cos(beta), 0.0, np.sin(beta)])
        counted = in_eclipse(position, sun, R_EARTH).mean()

        assert np.isclose(eclipse_fraction_circular(RADIUS, beta, R_EARTH), counted, atol=2e-5)

    # at beta = 0 the half-width of the shadow arc is asin(R / a)
    assert np.isclose(eclipse_fraction_circular(RADIUS, 0.0, R_EARTH), np.arcsin(R_EARTH / RADIUS) / np.pi)
    # above the critical beta the orbit is always lit
    assert np.isclose(np.degrees(critical_beta(RADIUS, R_EARTH)), 68.02, atol=0.01)
    assert eclipse_fraction_circular(RADIUS, critical_beta(RADIUS, R_EARTH) + 0.01, R_EARTH) == 0.0


def test_lvlh_axes_match_the_frame_built_from_the_state():
    inclination, raan, latitude = np.deg2rad(51.6), np.deg2rad(40.0), np.deg2rad(75.0)
    elements = ClassicalElements(a=RADIUS, e=0.0, i=inclination, raan=raan, argp=0.0, mean_anomaly=latitude)
    state = cartesian_state_from_elements(elements, MU_EARTH)

    assert np.allclose(lvlh_axes_circular(inclination, raan, latitude), lvlh_rotation(state), atol=1e-12)


def test_intervals_of_a_mask():
    t = np.arange(10.0)
    mask = np.array([1, 1, 0, 0, 1, 1, 1, 0, 0, 1], dtype=bool)

    assert intervals(t, mask) == [(0.0, 1.0), (4.0, 6.0), (9.0, 9.0)]
    assert intervals(t, np.zeros(10, dtype=bool)) == []


def _timeline(**overrides):
    arguments = dict(
        radius=RADIUS, inclination=np.deg2rad(51.6), raan=np.deg2rad(40.0), argument_of_latitude=0.0,
        jd_epoch=julian_date(2026, 3, 20, 12), duration=3 * 5677.0, step=10.0,
        mu=MU_EARTH, j2=J2_EARTH, r_eq=R_EARTH,
    )
    arguments.update(overrides)
    return port_lighting_timeline(**arguments)


def test_lighting_timeline_is_consistent_with_the_eclipse_fraction():
    timeline = _timeline()

    assert np.allclose(np.linalg.norm(timeline.sun_lvlh, axis=1), 1.0)
    assert np.isclose(timeline.eclipse.mean(), eclipse_fraction_circular(RADIUS, timeline.beta[0], R_EARTH),
                      atol=0.005)
    # the Sun stays at the beta angle from the orbit plane, seen from LVLH
    assert np.allclose(np.arcsin(timeline.sun_lvlh[:, 2]), timeline.beta, atol=1e-9)


def test_favourable_windows_are_lit_ports_out_of_eclipse():
    timeline = _timeline()
    windows = timeline.windows()

    assert len(windows) >= 3
    favourable = timeline.favourable
    assert not timeline.eclipse[favourable].any()
    assert np.all(timeline.incidence[favourable] <= np.deg2rad(70.0) + 1e-12)
    # the windows repeat once per orbit
    starts = np.array([start for start, _ in windows if start > 0.0])
    assert np.allclose(np.diff(starts), 5677.0, atol=30.0)


def test_favourable_for_asks_for_the_whole_duration():
    timeline = _timeline()
    start, end = next(w for w in timeline.windows() if w[0] > 0.0)

    assert timeline.favourable_for(start + 10.0, 60.0)
    assert not timeline.favourable_for(end - 30.0, 120.0)
    assert not timeline.favourable_for(1e9, 60.0)


def test_a_port_on_the_other_side_is_lit_in_the_other_half_of_the_orbit():
    behind = _timeline()
    ahead = _timeline(port_normal_lvlh=(0.0, 1.0, 0.0))

    assert not np.any(behind.favourable & ahead.favourable)
    assert np.allclose(behind.incidence + ahead.incidence, np.pi)


def test_beta_angle_history_stays_inside_its_bounds_and_oscillates():
    inclination = np.deg2rad(51.6)
    t_days, beta = beta_angle_history(RADIUS, inclination, np.deg2rad(40.0), julian_date(2026, 1, 1), 365.0,
                                      MU_EARTH, J2_EARTH, R_EARTH)

    limit = inclination + np.deg2rad(23.44)
    assert np.all(np.abs(beta) <= limit + 1e-9)
    assert beta.max() > 0.8 * limit and beta.min() < -0.8 * limit
    # the node regresses by about five degrees per day at this altitude and
    # inclination, so beta goes through several cycles in a year
    assert np.count_nonzero(np.diff(np.sign(beta))) >= 8
