import numpy as np

from ios_mission_planner.budget.margins import MATURITY_MARGIN, BudgetLine, total_delta_v


def test_margined_delta_v():
    line = BudgetLine("test burn", 100.0, 0.10)
    assert np.isclose(line.margined_delta_v, 110.0)


def test_total_delta_v_applies_system_reserve_on_top_of_lines():
    lines = [
        BudgetLine("a", 50.0, MATURITY_MARGIN["analyzed"]),
        BudgetLine("b", 20.0, MATURITY_MARGIN["preliminary"]),
    ]

    subtotal = 50.0 * 1.10 + 20.0 * 1.20
    expected = subtotal * 1.10

    assert abs(total_delta_v(lines, system_margin=0.10) - expected) < 1e-9


def test_maturity_margins_ordered_by_confidence():
    assert MATURITY_MARGIN["flight_proven"] < MATURITY_MARGIN["analyzed"] < MATURITY_MARGIN["preliminary"]


def test_no_lines_gives_zero_budget():
    assert total_delta_v([], system_margin=0.10) == 0.0
