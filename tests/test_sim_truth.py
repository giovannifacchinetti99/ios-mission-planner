import numpy as np

from ios_mission_planner.budget.propellant import G0
from ios_mission_planner.constants import J2_EARTH, MU_EARTH, OMEGA_EARTH, R_EARTH
from ios_mission_planner.dynamics.attitude.determination import (
    quaternion_to_rotation_matrix,
    rotation_matrix_to_quaternion,
)
from ios_mission_planner.dynamics.orbital.drag import two_body_j2_drag_dynamics_symbolic
from ios_mission_planner.dynamics.relative.lvlh import lvlh_rotation
from ios_mission_planner.propagation.heyoka_propagator import propagate
from ios_mission_planner.sim.truth import TruthModel

RADIUS = R_EARTH + 500.0
SPEED = np.sqrt(MU_EARTH / RADIUS)
INCLINATION = np.deg2rad(51.6)
CHIEF = np.array([RADIUS, 0.0, 0.0, 0.0, SPEED * np.cos(INCLINATION), SPEED * np.sin(INCLINATION)])
INERTIA = np.array([22.0, 25.0, 28.0])
ATMOSPHERE = (1e-12, RADIUS, 60.0)


def _truth(j2=J2_EARTH, density=ATMOSPHERE[0], gravity_gradient=False, bc=(0.01, 0.02)):
    return TruthModel(MU_EARTH, j2, R_EARTH, OMEGA_EARTH, density, ATMOSPHERE[1], ATMOSPHERE[2],
                      bc[0], bc[1], INERTIA, gravity_gradient=gravity_gradient)


def test_orbits_match_the_single_spacecraft_propagator():
    deputy = CHIEF.copy()
    deputy[1] -= 5.0

    truth = _truth()
    truth.reset(CHIEF, deputy, 160.0, [1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    for _ in range(600):
        truth.step(5.0)

    for state, bc, simulated in ((CHIEF, 0.01, truth.chief_state), (deputy, 0.02, truth.deputy_state)):
        reference = propagate(
            two_body_j2_drag_dynamics_symbolic(), state, np.array([0.0, 3000.0]),
            pars=[MU_EARTH, J2_EARTH, R_EARTH, bc, *ATMOSPHERE, OMEGA_EARTH],
        ).y[:, -1]
        assert np.linalg.norm(simulated[:3] - reference[:3]) < 1e-8  # km


def test_total_angular_momentum_is_conserved_while_wheels_exchange_it():
    q = np.array([0.9, 0.1, -0.3, 0.2])
    q /= np.linalg.norm(q)
    rate = np.array([0.01, -0.02, 0.015])
    wheel = np.array([0.1, -0.05, 0.02])

    truth = _truth()
    truth.reset(CHIEF, CHIEF, 160.0, q, rate, wheel_momentum=wheel)
    initial = quaternion_to_rotation_matrix(q).T @ (INERTIA * rate + wheel)

    truth.set_controls(wheel_torque=(0.01, -0.02, 0.005))
    for _ in range(200):
        truth.step(1.0)

    final = quaternion_to_rotation_matrix(truth.quaternion).T @ (INERTIA * truth.body_rate + truth.wheel_momentum)

    # the wheels moved momentum between themselves and the body...
    assert np.allclose(truth.wheel_momentum, wheel - 200.0 * np.array([0.01, -0.02, 0.005]), atol=1e-9)
    # ...but with no external torque the total, seen from inertial space, did not change
    assert np.linalg.norm(final - initial) < 1e-12 * np.linalg.norm(initial) + 1e-12
    assert abs(np.linalg.norm(truth.quaternion) - 1.0) < 1e-14


def test_thrust_delivers_the_rocket_equation_delta_v():
    isp, force, duration = 220.0, 2.0, 10.0

    truth = _truth(j2=0.0, density=0.0)
    truth.reset(CHIEF, CHIEF, 160.0, [1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    truth.set_controls(thrust_body=(force, 0.0, 0.0), mass_flow=force / (G0 * isp))
    truth.step(duration)

    delta_v = np.linalg.norm(truth.deputy_state[3:] - truth.chief_state[3:]) * 1e3
    expected = G0 * isp * np.log(160.0 / truth.mass)

    assert abs(truth.mass - (160.0 - force / (G0 * isp) * duration)) < 1e-12
    # the two spacecraft separate by a few metres during the burn, so
    # the difference in gravity adds a term of relative order 1e-5
    assert abs(delta_v - expected) < 1e-4 * expected


def test_thrust_direction_follows_the_attitude():
    # body x along the inertial y axis: a body-x thrust accelerates along +y
    rotation = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])

    truth = _truth(j2=0.0, density=0.0)
    truth.reset(CHIEF, CHIEF, 160.0, rotation_matrix_to_quaternion(rotation), [0.0, 0.0, 0.0])
    truth.set_controls(thrust_body=(2.0, 0.0, 0.0))
    truth.step(1.0)

    difference = (truth.deputy_state[3:] - truth.chief_state[3:]) * 1e3
    assert np.allclose(difference, [0.0, 2.0 / 160.0, 0.0], atol=1e-7)


def test_gravity_gradient_torque_matches_the_closed_form():
    # body rotated by 45 degrees about the orbit normal with respect to LVLH
    c = s = np.sqrt(0.5)
    body_from_lvlh = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])
    quaternion = rotation_matrix_to_quaternion(body_from_lvlh @ lvlh_rotation(CHIEF))

    truth = _truth(j2=0.0, density=0.0, gravity_gradient=True)
    truth.reset(CHIEF, CHIEF, 160.0, quaternion, [0.0, 0.0, 0.0])
    truth.step(1e-3)
    torque = truth.body_rate / 1e-3 * INERTIA

    radial_body = body_from_lvlh @ np.array([1.0, 0.0, 0.0])
    expected = 3.0 * MU_EARTH / RADIUS**3 * np.cross(radial_body, INERTIA * radial_body)

    assert np.allclose(torque, expected, rtol=1e-6, atol=1e-12)
    assert abs(expected[2]) > 1e-6  # the case is not trivially zero


def test_differential_drag_separates_two_spacecraft_on_the_same_orbit():
    truth = _truth(j2=0.0, bc=(0.01, 0.05))
    truth.reset(CHIEF, CHIEF, 160.0, [1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    for _ in range(600):
        truth.step(10.0)

    separation = np.linalg.norm(truth.deputy_state[:3] - truth.chief_state[:3]) * 1e3
    assert separation > 1.0

    same = _truth(j2=0.0, bc=(0.02, 0.02))
    same.reset(CHIEF, CHIEF, 160.0, [1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    for _ in range(600):
        same.step(10.0)

    assert np.linalg.norm(same.deputy_state[:3] - same.chief_state[:3]) * 1e3 < 1e-6
