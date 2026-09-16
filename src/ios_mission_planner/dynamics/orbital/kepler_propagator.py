import numpy as np

from ios_mission_planner.dynamics.orbital.elements import (
    OrbitalElements,
    eccentric_to_mean_anomaly,
    eccentric_to_true_anomaly,
    elements_to_state_vector,
    solve_kepler_equation,
    state_vector_to_elements,
    true_to_eccentric_anomaly,
)


def propagate_kepler_elements(elements0, t, mu):
    """
    Analytically propagate classical orbital elements by a time interval.

    Only the mean anomaly advances (all other elements are constant for
    unperturbed two-body motion); it is propagated linearly at the mean
    motion and mapped back to true anomaly through Kepler's equation.

    Parameters
    ----------
    elements0 : OrbitalElements
        Orbital elements at the initial epoch.
    t : float
        Propagation time [s].
    mu : float
        Gravitational parameter of the central body [km^3/s^2].

    Returns
    -------
    OrbitalElements
        Orbital elements at t0 + t.

    Author: Giovanni Facchinetti, 2026
    """

    n = np.sqrt(mu / elements0.a**3)

    E0 = true_to_eccentric_anomaly(elements0.nu, elements0.e)
    M0 = eccentric_to_mean_anomaly(E0, elements0.e)

    M = M0 + n * t

    E = solve_kepler_equation(M, elements0.e)
    nu = eccentric_to_true_anomaly(E, elements0.e)

    return OrbitalElements(
        a=elements0.a,
        e=elements0.e,
        i=elements0.i,
        raan=elements0.raan,
        argp=elements0.argp,
        nu=nu,
    )


def propagate_kepler(state0, t, mu):
    """
    Analytically propagate an inertial two-body state vector by a time interval.

    Convenience wrapper around propagate_kepler_elements that converts to
    and from classical orbital elements.

    Parameters
    ----------
    state0 : ndarray
        Inertial state vector [x, y, z, vx, vy, vz] at the initial epoch.
    t : float
        Propagation time [s].
    mu : float
        Gravitational parameter of the central body [km^3/s^2].

    Returns
    -------
    ndarray
        Inertial state vector at t0 + t.

    Author: Giovanni Facchinetti, 2026
    """

    elements0 = state_vector_to_elements(state0, mu)
    elements = propagate_kepler_elements(elements0, t, mu)

    return elements_to_state_vector(elements, mu)
