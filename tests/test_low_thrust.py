import numpy as np

from ios_mission_planner.budget.propellant import G0
from ios_mission_planner.constants import MU_EARTH, R_EARTH
from ios_mission_planner.dynamics.orbital.low_thrust import (
    low_thrust_dynamics_symbolic,
    low_thrust_tangential_dynamics_symbolic,
)
from ios_mission_planner.propagation.heyoka_propagator import propagate


def test_mass_depletes_by_the_rocket_equation():
    a = R_EARTH + 500.0
    v = np.sqrt(MU_EARTH / a)
    state0 = np.array([a, 0.0, 0.0, 0.0, v, 0.0, 100.0])
    F, isp = 0.5, 1800.0
    burn_time = 600.0

    final = propagate(
        low_thrust_tangential_dynamics_symbolic(), state0, [0.0, burn_time],
        pars=[MU_EARTH, F, isp, 1.0],
    ).y[:, -1]

    expected_mass = 100.0 - F / (isp * G0) * burn_time
    assert np.isclose(final[6], expected_mass, rtol=1e-6)


def test_short_burn_converges_to_the_impulsive_limit():
    a = R_EARTH + 500.0
    v = np.sqrt(MU_EARTH / a)
    state0_orbit = np.array([a, 0.0, 0.0, 0.0, v, 0.0])
    m0, isp, target_dv = 100.0, 1800.0, 0.001  # km/s

    errors = []
    for thrust in (0.05, 0.5, 5.0, 50.0):
        mdot = thrust / (isp * G0)
        m_prop = m0 * (1 - np.exp(-target_dv * 1e3 / (isp * G0)))
        burn_time = m_prop / mdot

        state0 = np.concatenate([state0_orbit, [m0]])
        final = propagate(
            low_thrust_tangential_dynamics_symbolic(), state0, [0.0, burn_time],
            pars=[MU_EARTH, thrust, isp, 1.0],
        ).y[:, -1]
        coast = propagate(
            low_thrust_tangential_dynamics_symbolic(), state0, [0.0, burn_time],
            pars=[MU_EARTH, 0.0, isp, 1.0],
        ).y[:, -1]

        dv_achieved = np.linalg.norm(final[3:6] - coast[3:6])
        errors.append(abs(dv_achieved - target_dv) / target_dv)

    # higher thrust -> shorter burn -> closer to the impulsive idealization
    assert errors == sorted(errors, reverse=True)
    assert errors[-1] < 1e-3


def test_fixed_direction_and_tangential_agree_for_a_short_burn():
    a = R_EARTH + 500.0
    v = np.sqrt(MU_EARTH / a)
    state0 = np.array([a, 0.0, 0.0, 0.0, v, 0.0, 100.0])
    F, isp, burn_time = 20.0, 1800.0, 2.0

    fixed = propagate(
        low_thrust_dynamics_symbolic(), state0, [0.0, burn_time],
        pars=[MU_EARTH, F, isp, 0.0, 1.0, 0.0],  # thrust along +y, matching v direction
    ).y[:, -1]
    tangential = propagate(
        low_thrust_tangential_dynamics_symbolic(), state0, [0.0, burn_time],
        pars=[MU_EARTH, F, isp, 1.0],
    ).y[:, -1]

    # not identical: tangential steering keeps re-aligning with velocity as
    # gravity turns it, even over a short burn, so a loose tolerance only
    assert np.allclose(fixed, tangential, rtol=1e-4, atol=1e-6)
