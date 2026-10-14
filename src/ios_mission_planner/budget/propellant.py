import numpy as np

G0 = 9.80665  # standard gravity [m/s^2], by definition of specific impulse


def mass_fraction(delta_v, isp, g0=G0):
    """
    Propellant mass fraction for a total delta-v, via the Tsiolkovsky
    rocket equation: 1 - exp(-delta_v / (isp * g0)).

    Parameters
    ----------
    delta_v : float
        Total delta-v [m/s].
    isp : float
        Specific impulse [s].
    g0 : float
        Standard gravity [m/s^2].

    Returns
    -------
    float
        Fraction of the initial (wet) mass that is propellant, in [0, 1).

    Author: Giovanni Facchinetti, 2026
    """

    return 1.0 - np.exp(-delta_v / (isp * g0))


def propellant_mass(delta_v, isp, m0, g0=G0):
    """
    Propellant mass required for a total delta-v, given the initial
    (wet) mass.

    Parameters
    ----------
    delta_v : float
        Total delta-v [m/s].
    isp : float
        Specific impulse [s].
    m0 : float
        Initial (wet) mass, before the maneuver(s) [kg].
    g0 : float
        Standard gravity [m/s^2].

    Returns
    -------
    float
        Propellant mass [kg].

    Author: Giovanni Facchinetti, 2026
    """

    return m0 * mass_fraction(delta_v, isp, g0)


def initial_mass(delta_v, isp, m_dry, g0=G0):
    """
    Initial (wet) mass required to deliver a total delta-v on top of a
    given dry mass.

    Inverse of `propellant_mass`: useful when the dry mass (payload plus
    structure) is fixed and the question is how much propellant, hence
    initial mass, it takes to fly a given delta-v budget.

    Parameters
    ----------
    delta_v : float
        Total delta-v [m/s].
    isp : float
        Specific impulse [s].
    m_dry : float
        Dry mass, after all propellant is spent [kg].
    g0 : float
        Standard gravity [m/s^2].

    Returns
    -------
    float
        Initial (wet) mass [kg].

    Author: Giovanni Facchinetti, 2026
    """

    return m_dry * np.exp(delta_v / (isp * g0))
