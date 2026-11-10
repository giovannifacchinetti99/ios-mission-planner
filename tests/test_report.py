from pathlib import Path

from ios_mission_planner.report.pipeline import (
    FEASIBLE,
    FEASIBLE_WITH_CAVEATS,
    NOT_FEASIBLE,
    run_feasibility_analysis,
)
from ios_mission_planner.report.render import render_html, render_pdf
from ios_mission_planner.report.scenario import load_scenario

SCENARIOS_DIR = Path(__file__).resolve().parent.parent / "examples" / "12_mission_feasibility_report"


def test_load_baseline_scenario():
    scenario = load_scenario(SCENARIOS_DIR / "baseline_scenario.yaml")

    assert scenario.name
    assert scenario.chief.altitude_km == 500.0
    assert scenario.deputy.dry_mass_kg == 150.0
    assert scenario.safety_checks.keep_out_zone_m == (20.0, 20.0, 20.0)


def test_baseline_scenario_flags_the_known_collision_probability_risk():
    scenario = load_scenario(SCENARIOS_DIR / "baseline_scenario.yaml")
    result = run_feasibility_analysis(scenario)

    # cross-checked against the independently-derived ~2.4% Pc finding in
    # examples/05_rendezvous_safety_assessment for the same kind of burn
    assert result.collision_probability > 0.01
    assert result.verdict == NOT_FEASIBLE
    assert any("collision probability" in issue.lower() for issue in result.issues)


def test_improved_navigation_scenario_is_feasible():
    scenario = load_scenario(SCENARIOS_DIR / "improved_nav_scenario.yaml")
    result = run_feasibility_analysis(scenario)

    assert result.collision_probability < 0.01
    assert result.verdict in (FEASIBLE, FEASIBLE_WITH_CAVEATS)


def test_budget_lines_are_positive_and_sum_consistently():
    scenario = load_scenario(SCENARIOS_DIR / "baseline_scenario.yaml")
    result = run_feasibility_analysis(scenario)

    assert all(line.delta_v > 0.0 for line in result.budget_lines)
    assert all(line.margined_delta_v >= line.delta_v for line in result.budget_lines)
    subtotal = sum(line.margined_delta_v for line in result.budget_lines)
    expected_total = subtotal * (1.0 + scenario.margins.system_reserve)
    assert abs(result.delta_v_total - expected_total) < 1e-6


def test_wet_mass_is_dry_mass_plus_propellant():
    scenario = load_scenario(SCENARIOS_DIR / "baseline_scenario.yaml")
    result = run_feasibility_analysis(scenario)

    assert abs(result.wet_mass_kg - (scenario.deputy.dry_mass_kg + result.propellant_mass_kg)) < 1e-9


def test_html_report_renders_and_contains_the_verdict():
    scenario = load_scenario(SCENARIOS_DIR / "baseline_scenario.yaml")
    result = run_feasibility_analysis(scenario)

    html = render_html(result)

    assert result.verdict in html
    assert scenario.name in html
    assert "data:image/png;base64," in html


def test_pdf_report_renders(tmp_path):
    scenario = load_scenario(SCENARIOS_DIR / "baseline_scenario.yaml")
    result = run_feasibility_analysis(scenario)
    html = render_html(result)

    pdf_path = tmp_path / "report.pdf"
    render_pdf(html, pdf_path)

    assert pdf_path.exists()
    assert pdf_path.read_bytes().startswith(b"%PDF")
