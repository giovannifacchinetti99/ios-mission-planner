import numpy as np

from ios_mission_planner.constants import MU_EARTH, R_EARTH
from ios_mission_planner.dynamics.orbital.maneuvers import (
    combined_plane_change_dv,
    hohmann_transfer,
    plane_change_dv,
)


def test_hohmann_transfer_matches_vis_viva_by_construction():
    r1, r2 = R_EARTH + 300.0, R_EARTH + 35786.0  # LEO to GEO-ish altitude
    dv1, dv2, tof = hohmann_transfer(r1, r2, MU_EARTH)

    a_t = 0.5 * (r1 + r2)
    v1, v2 = np.sqrt(MU_EARTH / r1), np.sqrt(MU_EARTH / r2)
    v_t1 = np.sqrt(MU_EARTH * (2 / r1 - 1 / a_t))
    v_t2 = np.sqrt(MU_EARTH * (2 / r2 - 1 / a_t))

    assert np.isclose(dv1, v_t1 - v1)
    assert np.isclose(dv2, v2 - v_t2)
    assert np.isclose(tof, np.pi * np.sqrt(a_t**3 / MU_EARTH))
    # raising altitude: both burns should add energy (same sign as the net climb)
    assert dv1 > 0.0
    assert dv2 > 0.0


def test_deorbit_burn_is_the_first_hohmann_burn():
    r1 = R_EARTH + 500.0
    r_perigee_target = R_EARTH + 200.0

    dv1, _, _ = hohmann_transfer(r1, r_perigee_target, MU_EARTH)

    assert dv1 < 0.0  # retrograde, lowering the orbit


def test_plane_change_dv_formula():
    v, di = 7.6, np.deg2rad(5.0)
    assert np.isclose(plane_change_dv(v, di), 2 * v * np.sin(di / 2))
    assert plane_change_dv(v, 0.0) == 0.0


def test_combined_plane_change_reduces_to_special_cases():
    v1, v2, di = 7.6, 7.8, np.deg2rad(3.0)

    # no plane change: reduces to a pure speed change
    assert np.isclose(combined_plane_change_dv(v1, v2, 0.0), abs(v2 - v1))
    # no speed change: reduces to a pure plane change
    assert np.isclose(combined_plane_change_dv(v1, v1, di), plane_change_dv(v1, di))


def test_combining_is_never_more_expensive_than_two_separate_burns():
    v1, v2, di = 7.6, 7.3, np.deg2rad(10.0)
    combined = combined_plane_change_dv(v1, v2, di)
    separate = abs(v2 - v1) + plane_change_dv(v2, di)

    assert combined <= separate + 1e-12
