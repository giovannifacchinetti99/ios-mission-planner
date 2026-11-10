from dataclasses import dataclass, field

import numpy as np

from ios_mission_planner.budget.margins import MATURITY_MARGIN, BudgetLine, total_delta_v
from ios_mission_planner.budget.propellant import initial_mass
from ios_mission_planner.constants import MU_EARTH, R_EARTH
from ios_mission_planner.dynamics.orbital.maneuvers import combined_plane_change_dv, hohmann_transfer
from ios_mission_planner.dynamics.relative.cw import cw_targeting
from ios_mission_planner.dynamics.relative.hill import hill_equations_symbolic
from ios_mission_planner.dynamics.relative.relative_orbit import RelativeOrbit, state_from_relative_orbit
from ios_mission_planner.propagation.heyoka_propagator import propagate
from ios_mission_planner.report.scenario import MissionScenario
from ios_mission_planner.safety.collision_probability import collision_probability
from ios_mission_planner.safety.corridor import ApproachCorridor, check_corridor
from ios_mission_planner.safety.keepout import KeepOutEllipsoid, check_trajectory
from ios_mission_planner.safety.passive_safety import check_passive_safety

FEASIBLE = "FEASIBLE"
FEASIBLE_WITH_CAVEATS = "FEASIBLE WITH CAVEATS"
NOT_FEASIBLE = "NOT FEASIBLE"


@dataclass
class SafetyCheckResult:
    label: str
    passed: bool
    margin: float
    margin_unit: str


@dataclass
class FeasibilityResult:
    """
    The output of run_feasibility_analysis: every number and verdict a
    feasibility report needs, in one place.

    Author: Giovanni Facchinetti, 2026
    """

    scenario: MissionScenario
    budget_lines: list
    delta_v_total: float
    propellant_mass_kg: float
    wet_mass_kg: float
    safety_checks: list
    collision_probability: float
    hop_trajectory_m: np.ndarray
    verdict: str
    issues: list = field(default_factory=list)


def run_feasibility_analysis(scenario):
    """
    Run the Layer 4 feasibility pipeline for a MissionScenario.

    Every delta-v is computed from the same closed-form building blocks
    validated elsewhere in this project (hohmann_transfer, cw_targeting,
    the rocket equation), not from a fresh nonlinear re-flight of the
    mission: this keeps the pipeline fast and fully determined by the
    scenario file, at the cost of folding what examples/08_mission_budget
    flies as two separate burns (a far-range hop, then an ellipse
    insertion correction) into a single closed-form targeting burn here,
    since the discrepancy between those two only exists because of
    nonlinear effects this pipeline does not model. That simplification is
    reported back explicitly, in the issues list, whenever it is more than
    a token amount of the total budget.

    Parameters
    ----------
    scenario : ios_mission_planner.report.scenario.MissionScenario

    Returns
    -------
    FeasibilityResult

    Author: Giovanni Facchinetti, 2026
    """

    mu = MU_EARTH
    a_c = R_EARTH + scenario.chief.altitude_km
    a_p = R_EARTH + scenario.deputy.parking_altitude_km
    n_c = np.sqrt(mu / a_c**3)
    period = 2 * np.pi / n_c

    # --- phasing and rendezvous delta-v, all closed-form ---
    dv_h1, _, _ = hohmann_transfer(a_p, a_c, mu)
    a_transfer_up = 0.5 * (a_p + a_c)
    v_t2 = np.sqrt(mu * (2.0 / a_c - 1.0 / a_transfer_up))
    v_chief = np.sqrt(mu / a_c)
    dv_h2_combined = combined_plane_change_dv(v_t2, v_chief, np.deg2rad(scenario.plane_change_deg))

    rho_m = scenario.safety_ellipse.radius_m
    hop_duration_s = scenario.safety_ellipse.hop_duration_orbits * period
    orbit_far = RelativeOrbit(x_off=0.0, y_c=0.0, rho=rho_m, alpha=np.pi / 2)
    state_far = state_from_relative_orbit(orbit_far, n_c)

    r0_hop = np.array([0.0, -scenario.phasing.hold_range_km * 1e3, 0.0])
    v0_hop = np.zeros(3)
    dv1_hop, dv2_hop = cw_targeting(r0_hop, v0_hop, state_far[:3], state_far[3:], hop_duration_s, n_c)
    dv_approach = np.linalg.norm(dv1_hop) + np.linalg.norm(dv2_hop)

    dwell_orbits = scenario.station_keeping.dwell_days * 86400.0 / period
    dv_station_keeping = scenario.station_keeping.rate_m_s_per_orbit * dwell_orbits

    orbit_near = RelativeOrbit(x_off=0.0, y_c=0.0, rho=rho_m, alpha=0.0)
    state_near = state_from_relative_orbit(orbit_near, n_c)
    dock_duration_s = scenario.docking.duration_orbits * period
    vf_dock = np.array([-scenario.docking.closing_speed_m_s, 0.0, 0.0])
    dv1_dock, dv2_dock = cw_targeting(
        state_near[:3], state_near[3:], np.zeros(3), vf_dock, dock_duration_s, n_c,
    )
    dv_docking = np.linalg.norm(dv1_dock) + np.linalg.norm(dv2_dock)

    r_perigee_target = R_EARTH + scenario.disposal.target_perigee_altitude_km
    dv_deorbit, _, _ = hohmann_transfer(a_c, r_perigee_target, mu)
    dv_deorbit_ms = abs(dv_deorbit) * 1e3

    plane_change_note = f" (+{scenario.plane_change_deg:.1f} deg plane change)" if scenario.plane_change_deg else ""
    budget_lines = [
        BudgetLine("Hohmann burn 1 (raise to transfer)", dv_h1 * 1e3, MATURITY_MARGIN["analyzed"]),
        BudgetLine("Hohmann burn 2 (circularize)" + plane_change_note, dv_h2_combined * 1e3, MATURITY_MARGIN["analyzed"]),
        BudgetLine("Far-range approach & ellipse insertion", dv_approach, MATURITY_MARGIN["analyzed"]),
        BudgetLine("Station-keeping dwell", dv_station_keeping, MATURITY_MARGIN["preliminary"]),
        BudgetLine("Docking", dv_docking, MATURITY_MARGIN["analyzed"]),
        BudgetLine("Disposal (deorbit)", dv_deorbit_ms, MATURITY_MARGIN["analyzed"]),
    ]
    delta_v_total = total_delta_v(budget_lines, scenario.margins.system_reserve)
    propellant_mass_kg = initial_mass(
        delta_v_total, scenario.deputy.propulsion.isp_s, scenario.deputy.dry_mass_kg,
    ) - scenario.deputy.dry_mass_kg
    wet_mass_kg = scenario.deputy.dry_mass_kg + propellant_mass_kg

    # --- safety checks, on the far-range hop and the resulting ellipse ---
    zone = KeepOutEllipsoid(*scenario.safety_checks.keep_out_zone_m)
    t_eval = np.linspace(0.0, hop_duration_s, 300)
    hop_state0 = np.concatenate([r0_hop, v0_hop + dv1_hop])
    hop_trajectory_m = propagate(hill_equations_symbolic(), hop_state0, t_eval, pars=[n_c]).y[:3, :].T

    keepout_violated, keepout_margin, _ = check_trajectory(hop_trajectory_m, zone)

    ps_violated, ps_margin, _, _ = check_passive_safety(
        hop_state0, n_c, zone, scenario.safety_checks.passive_safety_duration_s,
    )

    corridor = ApproachCorridor(
        direction=scenario.safety_checks.approach_corridor_direction,
        half_angle=np.deg2rad(scenario.safety_checks.approach_corridor_half_angle_deg),
    )
    corridor_violated, corridor_margin, _ = check_corridor(hop_trajectory_m, corridor)

    mc = scenario.safety_checks.monte_carlo
    covariance = np.diag([mc.nav_sigma_m**2] * 3 + [mc.burn_sigma_m_s**2] * 3)
    pc = collision_probability(
        state_far, covariance, n_c, np.linspace(0.0, period, 50), zone,
        n_samples=mc.n_samples, rng=np.random.default_rng(0),
    )

    safety_checks = [
        SafetyCheckResult("Keep-out zone (far-range hop)", not keepout_violated, -keepout_margin, "margin"),
        SafetyCheckResult("Passive safety (hop burn failure)", not ps_violated, -ps_margin, "margin"),
        SafetyCheckResult("Approach corridor (far-range hop)", not corridor_violated, -corridor_margin, "rad margin"),
    ]

    # --- verdict ---
    issues = []
    hard_fail = False

    if keepout_violated:
        issues.append("The far-range hop trajectory enters the keep-out zone.")
        hard_fail = True
    if ps_violated:
        issues.append("A failed hop burn would coast into the keep-out zone before the passive-safety window ends.")
        hard_fail = True
    if corridor_violated:
        issues.append("The far-range hop strays outside the approach corridor.")
        hard_fail = True
    if pc > scenario.verdict_thresholds.max_collision_probability:
        issues.append(
            f"Monte Carlo collision probability ({pc:.2%}) exceeds the "
            f"threshold ({scenario.verdict_thresholds.max_collision_probability:.2%})."
        )
        hard_fail = True
    if wet_mass_kg > scenario.verdict_thresholds.max_wet_mass_kg:
        issues.append(
            f"Required wet mass ({wet_mass_kg:.1f} kg) exceeds the stated "
            f"budget ({scenario.verdict_thresholds.max_wet_mass_kg:.1f} kg)."
        )
        hard_fail = True

    if dv_approach / sum(line.delta_v for line in budget_lines) > 0.02:
        issues.append(
            "The far-range approach and ellipse insertion are combined into one closed-form "
            "burn here; the fully nonlinear flight in examples/08_mission_budget needs a second, "
            "small correction burn this pipeline does not separately budget for."
        )

    if hard_fail:
        verdict = NOT_FEASIBLE
    elif issues:
        verdict = FEASIBLE_WITH_CAVEATS
    else:
        verdict = FEASIBLE

    return FeasibilityResult(
        scenario=scenario,
        budget_lines=budget_lines,
        delta_v_total=delta_v_total,
        propellant_mass_kg=propellant_mass_kg,
        wet_mass_kg=wet_mass_kg,
        safety_checks=safety_checks,
        collision_probability=pc,
        hop_trajectory_m=hop_trajectory_m,
        verdict=verdict,
        issues=issues,
    )
