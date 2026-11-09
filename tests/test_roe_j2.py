import numpy as np

from ios_mission_planner.constants import MU_EARTH, R_EARTH, J2_EARTH
from ios_mission_planner.dynamics.orbital.elements import ClassicalElements
from ios_mission_planner.dynamics.relative.roe_j2 import (
    QuasiNonsingularROE,
    deputy_elements_from_roe,
    j2_quasi_nonsingular_stm,
    quasi_nonsingular_roe,
)


def test_round_trip_roe_elements():
    chief = ClassicalElements(
        a=R_EARTH + 500.0, e=0.001, i=np.deg2rad(51.6),
        raan=np.deg2rad(10.0), argp=np.deg2rad(20.0), mean_anomaly=np.deg2rad(30.0),
    )
    roe = QuasiNonsingularROE(da=1e-5, dlambda=2e-5, dex=3e-5, dey=-1e-5, dix=4e-5, diy=-2e-5)

    deputy = deputy_elements_from_roe(chief, roe)
    recovered = quasi_nonsingular_roe(chief, deputy)

    np.testing.assert_allclose(recovered.as_array(), roe.as_array(), atol=1e-12)


def test_stm_is_identity_at_tau_zero():
    chief = ClassicalElements(
        a=R_EARTH + 500.0, e=0.01, i=np.deg2rad(97.0),
        raan=0.0, argp=np.deg2rad(45.0), mean_anomaly=0.0,
    )

    phi = j2_quasi_nonsingular_stm(chief, 0.0, MU_EARTH, J2_EARTH, R_EARTH)

    assert np.allclose(phi, np.eye(6), atol=1e-10)


def test_da_is_invariant_under_the_stm():
    # J2's secular effect does not change the along-track drift driver da,
    # since differential energy is conserved by a pair of unperturbed-mass
    # Keplerian elements drifting under the same central-body J2.
    chief = ClassicalElements(
        a=R_EARTH + 500.0, e=0.02, i=np.deg2rad(63.4),
        raan=np.deg2rad(5.0), argp=np.deg2rad(10.0), mean_anomaly=0.0,
    )
    roe0 = QuasiNonsingularROE(da=2e-5, dlambda=0.0, dex=1e-5, dey=1e-5, dix=0.0, diy=0.0).as_array()

    period = 2 * np.pi * np.sqrt(chief.a**3 / MU_EARTH)
    phi = j2_quasi_nonsingular_stm(chief, 5 * period, MU_EARTH, J2_EARTH, R_EARTH)
    roe_t = phi @ roe0

    assert np.isclose(roe_t[0], roe0[0], rtol=1e-9)
