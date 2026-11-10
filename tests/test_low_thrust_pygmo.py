import numpy as np

from ios_mission_planner.constants import MU_EARTH, R_EARTH
from ios_mission_planner.dynamics.orbital.low_thrust_design import (
    ArcHalfAnglePygmoProblem,
    pareto_front_arc_half_angle,
)
from ios_mission_planner.dynamics.orbital.maneuvers import hohmann_transfer


def deorbit_scenario():
    a_c = R_EARTH + 500.0
    v = np.sqrt(MU_EARTH / a_c)
    chief0 = np.array([a_c, 0.0, 0.0, 0.0, v, 0.0])
    r_perigee_target = R_EARTH + 200.0
    dv_deorbit, _, _ = hohmann_transfer(a_c, r_perigee_target, MU_EARTH)

    return chief0, r_perigee_target, abs(dv_deorbit) * 1e3


def test_udp_fitness_returns_two_objectives():
    chief0, r_perigee_target, dv_ms = deorbit_scenario()
    udp = ArcHalfAnglePygmoProblem(chief0, r_perigee_target, dv_ms, m_dry=150.0, isp=1800.0, thrust=0.2, mu=MU_EARTH)

    f = udp.fitness([0.1])

    assert len(f) == 2
    assert f[0] >= 0.0  # perigee error is non-negative
    assert f[1] > 0.0  # wall-clock time is positive
    assert udp.get_nobj() == 2


def test_udp_bounds_match_half_angle_range():
    chief0, r_perigee_target, dv_ms = deorbit_scenario()
    udp = ArcHalfAnglePygmoProblem(
        chief0, r_perigee_target, dv_ms, m_dry=150.0, isp=1800.0, thrust=0.2, mu=MU_EARTH,
        half_angle_bounds=(0.01, np.pi),
    )

    lo, hi = udp.get_bounds()
    assert np.isclose(lo[0], 0.01)
    assert np.isclose(hi[0], np.pi)


def test_pareto_front_has_the_requested_population_size():
    chief0, r_perigee_target, dv_ms = deorbit_scenario()

    half_angles, perigee_errors, wall_times = pareto_front_arc_half_angle(
        chief0, r_perigee_target, dv_ms, m_dry=150.0, isp=1800.0, thrust=0.2, mu=MU_EARTH,
        generations=5, pop_size=8, seed=0,
    )

    assert len(half_angles) == 8
    assert len(perigee_errors) == 8
    assert len(wall_times) == 8
    assert np.all(half_angles > 0.0) and np.all(half_angles <= np.pi)


def test_pareto_front_shows_a_genuine_time_accuracy_trade_off():
    chief0, r_perigee_target, dv_ms = deorbit_scenario()

    half_angles, perigee_errors, wall_times = pareto_front_arc_half_angle(
        chief0, r_perigee_target, dv_ms, m_dry=150.0, isp=1800.0, thrust=0.2, mu=MU_EARTH,
        generations=15, pop_size=12, seed=1,
    )

    # the point with the smallest perigee error should not also have the
    # smallest wall time: there is a real trade-off, not a dominant point
    best_accuracy_idx = np.argmin(perigee_errors)
    best_speed_idx = np.argmin(wall_times)

    assert best_accuracy_idx != best_speed_idx
    assert wall_times[best_accuracy_idx] > wall_times[best_speed_idx]
