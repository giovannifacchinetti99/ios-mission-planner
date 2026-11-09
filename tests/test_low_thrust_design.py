import numpy as np

from ios_mission_planner.constants import MU_EARTH, R_EARTH
from ios_mission_planner.dynamics.orbital.low_thrust import (
    low_thrust_arc_tangential_dynamics_symbolic,
    low_thrust_tangential_dynamics_symbolic,
)
from ios_mission_planner.dynamics.orbital.low_thrust_design import (
    arc_duty_cycle,
    deorbit_with_arc_limited_thrust,
    solve_half_angle_for_target_perigee,
)
from ios_mission_planner.dynamics.orbital.maneuvers import hohmann_transfer
from ios_mission_planner.propagation.heyoka_propagator import propagate


def circular_chief(altitude):
    a = R_EARTH + altitude
    v = np.sqrt(MU_EARTH / a)
    return a, np.array([a, 0.0, 0.0, 0.0, v, 0.0])


def test_duty_cycle():
    assert np.isclose(arc_duty_cycle(np.pi), 1.0)
    assert np.isclose(arc_duty_cycle(np.pi / 2), 0.5)


def test_full_arc_matches_continuous_tangential_thrust():
    a_c, chief0 = circular_chief(500.0)
    state0 = np.concatenate([chief0, [100.0]])
    pars_tangential = [MU_EARTH, 0.2, 1800.0, -1.0]
    pars_arc = [MU_EARTH, 0.2, 1800.0, -1.0, 1.0, 0.0, 0.0, np.pi, 200.0]
    t_eval = [0.0, 3600.0]

    tangential = propagate(low_thrust_tangential_dynamics_symbolic(), state0, t_eval, pars=pars_tangential).y[:, -1]
    arc_full = propagate(low_thrust_arc_tangential_dynamics_symbolic(), state0, t_eval, pars=pars_arc).y[:, -1]

    assert np.allclose(tangential, arc_full, rtol=1e-4, atol=1e-6)


def test_narrower_arc_reaches_a_lower_final_perigee():
    a_c, chief0 = circular_chief(500.0)
    r_perigee_target = R_EARTH + 200.0
    dv_deorbit, _, _ = hohmann_transfer(a_c, r_perigee_target, MU_EARTH)
    dv_ms = abs(dv_deorbit) * 1e3

    perigees = []
    for half_angle in (np.pi, np.pi / 4, np.pi / 16):
        elements, _ = deorbit_with_arc_limited_thrust(
            chief0, half_angle, dv_ms, m_dry=150.0, isp=1800.0, thrust=0.2, mu=MU_EARTH, n_pts=1500,
        )
        perigees.append(elements.a * (1.0 - elements.e))

    assert perigees == sorted(perigees, reverse=True)  # narrower arc -> lower perigee


def test_solver_converges_close_to_the_target_perigee():
    a_c, chief0 = circular_chief(500.0)
    r_perigee_target = R_EARTH + 200.0
    dv_deorbit, _, _ = hohmann_transfer(a_c, r_perigee_target, MU_EARTH)
    dv_ms = abs(dv_deorbit) * 1e3

    half_angle, r_p = solve_half_angle_for_target_perigee(
        chief0, r_perigee_target, dv_ms, m_dry=150.0, isp=1800.0, thrust=0.2, mu=MU_EARTH,
        n_pts=1200, tol=1.0,
    )

    assert 0.0 < half_angle < np.pi
    assert abs(r_p - r_perigee_target) < 2.0
