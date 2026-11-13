import numpy as np
import pytest
from conftest import make_near_hold_config

from ios_mission_planner.constants import J2_EARTH, MU_EARTH, R_EARTH
from ios_mission_planner.illumination.lighting import port_lighting_timeline
from ios_mission_planner.illumination.sun import julian_date_from_datetime
from ios_mission_planner.report.mission_report import MissionRequirements, analyze_mission, render_mission_report
from ios_mission_planner.report.pipeline import FEASIBLE, FEASIBLE_WITH_CAVEATS, NOT_FEASIBLE
from ios_mission_planner.sim.monte_carlo import Dispersions, run_monte_carlo
from ios_mission_planner.sim.simulator import simulate

NEAR_DISPERSIONS = Dispersions(
    position_offset=0.05, velocity_offset=0.002, attitude_error=np.deg2rad(0.2), body_rate=0.0,
    navigation_position_error=0.03, navigation_velocity_error=0.001,
)


@pytest.fixture(scope="module")
def nominal():
    return simulate(make_near_hold_config())


@pytest.fixture(scope="module")
def campaign():
    return run_monte_carlo(make_near_hold_config(), NEAR_DISPERSIONS, n_runs=4, first_seed=50)


def _lighting(config, argument_of_latitude_deg=None):
    chief = config.chief
    latitude = chief.argument_of_latitude_deg if argument_of_latitude_deg is None else argument_of_latitude_deg
    return port_lighting_timeline(
        R_EARTH + chief.altitude_km, np.deg2rad(chief.inclination_deg), np.deg2rad(chief.raan_deg),
        np.deg2rad(latitude), julian_date_from_datetime(config.epoch), 12000.0, 10.0,
        MU_EARTH, J2_EARTH, R_EARTH, port_normal_lvlh=chief.port_axis,
    )


def test_without_a_campaign_the_verdict_carries_a_caveat(nominal):
    analysis = analyze_mission("test", nominal)

    assert analysis.verdict == FEASIBLE_WITH_CAVEATS
    assert not analysis.issues
    assert any("No Monte Carlo campaign" in caveat for caveat in analysis.caveats)
    assert analysis.final_approach[0] < analysis.final_approach[1]


def test_a_small_campaign_cannot_demonstrate_a_high_probability(nominal, campaign):
    # four successes out of four: the lower end of the interval is 0.51
    demanding = analyze_mission("test", nominal, campaign, requirements=MissionRequirements(0.95))
    modest = analyze_mission("test", nominal, campaign, requirements=MissionRequirements(0.40))

    assert demanding.verdict == FEASIBLE_WITH_CAVEATS
    assert any("too small to demonstrate" in caveat for caveat in demanding.caveats)
    assert modest.verdict == FEASIBLE and not modest.caveats


def test_too_little_delta_v_makes_the_mission_not_feasible(nominal, campaign):
    requirements = MissionRequirements(min_success_probability=0.40, max_delta_v=0.05)
    analysis = analyze_mission("test", nominal, campaign, requirements=requirements)

    assert analysis.verdict == NOT_FEASIBLE
    assert any("velocity change" in issue for issue in analysis.issues)


def test_a_nominal_run_that_does_not_dock_is_not_feasible():
    config = make_near_hold_config()
    config.failures.sensor_outage_start = 100.0
    config.failures.sensor_outage_duration = 1e5
    config.guidance.abort_coast_duration = 120.0
    analysis = analyze_mission("test", simulate(config))

    assert analysis.verdict == NOT_FEASIBLE
    assert "aborted" in analysis.issues[0] and "relative measurements lost" in analysis.issues[0]


def test_lighting_of_the_final_approach_is_checked(nominal, campaign):
    requirements = MissionRequirements(min_success_probability=0.40)
    timeline = _lighting(nominal.config)
    start, end = analyze_mission("test", nominal).final_approach
    lit = timeline.favourable_for(start, end - start)

    analysis = analyze_mission("test", nominal, campaign, timeline, requirements)
    assert analysis.final_approach_lit == lit
    assert (analysis.verdict == FEASIBLE) == lit
    if not lit:
        assert any("not flown in favourable lighting" in caveat for caveat in analysis.caveats)

    # the same approach half an orbit later sees the opposite lighting or an eclipse
    other = _lighting(nominal.config, argument_of_latitude_deg=nominal.config.chief.argument_of_latitude_deg + 180.0)
    assert not (lit and other.favourable_for(start, end - start))


def test_report_contains_every_section(nominal, campaign):
    analysis = analyze_mission("Reference docking", nominal, campaign, _lighting(nominal.config),
                               MissionRequirements(min_success_probability=0.40))
    html = render_mission_report(analysis)

    for text in ("Mission analysis report: Reference docking", analysis.verdict, "Nominal run",
                 "Conditions at contact", "Monte Carlo campaign", "Lighting of the docking port",
                 "final approach", "lateral misalignment [m]"):
        assert text in html
    assert html.count("data:image/png;base64,") == 4
    assert "{{" not in html and "{%" not in html


def test_report_without_campaign_and_lighting_omits_their_sections(nominal):
    html = render_mission_report(analyze_mission("test", nominal))

    assert "Monte Carlo campaign</h2>" not in html
    assert "Lighting of the docking port</h2>" not in html
    assert html.count("data:image/png;base64,") == 2
