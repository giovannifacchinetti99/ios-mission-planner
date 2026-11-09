import numpy as np

from ios_mission_planner.budget.propellant import initial_mass, mass_fraction, propellant_mass


def test_mass_fraction_zero_delta_v():
    assert mass_fraction(0.0, 300.0) == 0.0


def test_initial_mass_inverts_propellant_mass():
    isp, delta_v, m_dry = 300.0, 120.0, 150.0

    m0 = initial_mass(delta_v, isp, m_dry)
    m_prop = propellant_mass(delta_v, isp, m0)

    assert np.isclose(m0 - m_prop, m_dry, rtol=1e-9)


def test_higher_isp_needs_less_propellant_for_the_same_delta_v():
    delta_v, m0 = 100.0, 500.0

    assert propellant_mass(delta_v, 220.0, m0) > propellant_mass(delta_v, 1800.0, m0)


def test_rocket_equation_consistency():
    isp, delta_v, m0 = 230.0, 55.0, 400.0
    g0 = 9.80665

    mf = m0 * np.exp(-delta_v / (isp * g0))
    assert np.isclose(propellant_mass(delta_v, isp, m0), m0 - mf, rtol=1e-9)
