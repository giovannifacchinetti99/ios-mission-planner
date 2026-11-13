import numpy as np

from ios_mission_planner.sim.config import DockingLimits
from ios_mission_planner.sim.docking import PortState, approach_basis, assess_contact, port_state

CHIEF_PORT = np.array([0.0, -1.0, 0.0])
CHIEF_AXIS = np.array([0.0, -1.0, 0.0])
DEPUTY_PORT = np.array([0.6, 0.0, 0.0])
PORT_AXIS_BODY = np.array([1.0, 0.0, 0.0])
ROLL_AXIS_BODY = np.array([0.0, 0.0, 1.0])

# body x along +y (toward the chief), body z along the orbit normal
ALIGNED = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])


def _ports(relative_state, body_to_lvlh=ALIGNED, rate=(0.0, 0.0, 0.0)):
    return port_state(relative_state, body_to_lvlh, np.array(rate), CHIEF_PORT, CHIEF_AXIS,
                      DEPUTY_PORT, PORT_AXIS_BODY, ROLL_AXIS_BODY)


def test_approach_basis_for_a_port_on_the_v_bar():
    axis, lateral_1, lateral_2 = approach_basis(CHIEF_AXIS)

    assert np.allclose(axis, [0.0, -1.0, 0.0])
    assert np.allclose(lateral_1, [1.0, 0.0, 0.0])      # radial
    assert np.allclose(lateral_2, [0.0, 0.0, 1.0])      # orbit normal
    assert np.allclose(np.cross(lateral_1, lateral_2), axis)


def test_approach_basis_is_orthonormal_for_any_axis():
    for axis_in in ([1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.3, -0.8, 0.1]):
        basis = np.array(approach_basis(axis_in))
        assert np.allclose(basis @ basis.T, np.eye(3), atol=1e-12)


def test_aligned_deputy_on_the_axis():
    # center of mass 10.6 m behind the chief port: the ports are 10 m apart
    state = _ports([0.0, -11.6, 0.0, 0.0, 0.08, 0.0])

    assert np.isclose(state.axial_distance, 10.0)
    assert np.allclose(state.lateral_offset, 0.0)
    assert np.isclose(state.closing_speed, 0.08)
    assert np.isclose(state.axis_misalignment, 0.0, atol=1e-7)
    assert np.isclose(state.roll_misalignment, 0.0)


def test_lateral_offsets_and_speeds_are_resolved_on_the_port_axes():
    state = _ports([0.03, -1.6, -0.04, 0.002, 0.07, -0.001])

    assert np.isclose(state.axial_distance, 0.0)
    assert np.allclose(state.lateral_offset, [0.03, -0.04])
    assert np.isclose(state.lateral_misalignment, 0.05)
    assert np.allclose(state.lateral_velocity, [0.002, -0.001])


def test_a_rotated_deputy_presents_its_port_off_the_axis():
    # yaw of 2 degrees about the orbit normal: the port, 0.6 m ahead of
    # the center of mass, moves sideways by 0.6 sin(2 deg)
    angle = np.deg2rad(2.0)
    yaw = np.array([[np.cos(angle), -np.sin(angle), 0.0], [np.sin(angle), np.cos(angle), 0.0], [0.0, 0.0, 1.0]])
    state = _ports([0.0, -1.6, 0.0, 0.0, 0.0, 0.0], body_to_lvlh=yaw @ ALIGNED)

    assert np.isclose(state.axis_misalignment, angle)
    assert np.isclose(abs(state.lateral_offset[0]), 0.6 * np.sin(angle))
    assert np.isclose(state.roll_misalignment, 0.0, atol=1e-12)


def test_roll_about_the_port_axis_is_reported_as_roll():
    angle = np.deg2rad(3.0)
    roll = np.array([[np.cos(angle), 0.0, np.sin(angle)], [0.0, 1.0, 0.0], [-np.sin(angle), 0.0, np.cos(angle)]])
    state = _ports([0.0, -1.6, 0.0, 0.0, 0.0, 0.0], body_to_lvlh=roll @ ALIGNED)

    assert np.isclose(abs(state.roll_misalignment), angle)
    assert np.isclose(state.axis_misalignment, 0.0, atol=1e-7)


def test_body_rate_moves_the_port():
    # rate about body z: the port, on the body x axis, moves along body y
    state = _ports([0.0, -1.6, 0.0, 0.0, 0.0, 0.0], rate=(0.0, 0.0, 0.01))

    assert np.isclose(state.lateral_speed, 0.6 * 0.01)
    assert np.isclose(state.angular_rate, 0.01)


def _contact(**overrides):
    values = dict(axial_distance=0.0, lateral_offset=np.array([0.02, 0.01]), closing_speed=0.075,
                  lateral_velocity=np.array([0.005, 0.0]), axis_misalignment=np.deg2rad(0.5),
                  roll_misalignment=np.deg2rad(-0.4), angular_rate=np.deg2rad(0.02))
    values.update(overrides)
    return PortState(**values)


def test_contact_inside_every_limit_is_a_success():
    assessment = assess_contact(_contact(), DockingLimits())

    assert assessment.success
    assert assessment.violations == []
    assert np.isclose(assessment.values["roll misalignment [deg]"], 0.4)


def test_each_limit_is_checked_on_its_own():
    limits = DockingLimits()
    cases = {
        "lateral misalignment [m]": dict(lateral_offset=np.array([0.09, 0.09])),
        "axial speed [m/s]": dict(closing_speed=0.12),
        "lateral speed [m/s]": dict(lateral_velocity=np.array([0.05, 0.0])),
        "angular misalignment [deg]": dict(axis_misalignment=np.deg2rad(5.0)),
        "roll misalignment [deg]": dict(roll_misalignment=np.deg2rad(-5.0)),
        "angular rate [deg/s]": dict(angular_rate=np.deg2rad(0.3)),
    }

    for name, change in cases.items():
        assessment = assess_contact(_contact(**change), limits)
        assert not assessment.success
        assert assessment.violations == [name]


def test_an_approach_that_is_too_slow_also_fails():
    assessment = assess_contact(_contact(closing_speed=0.03), DockingLimits())

    assert assessment.violations == ["axial speed [m/s]"]
