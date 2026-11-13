from dataclasses import dataclass

import numpy as np

from ios_mission_planner.linalg import cross3, norm3


def approach_basis(port_axis):
    """
    Orthonormal basis attached to a docking port: the outward axis of
    the port and two lateral directions. The second lateral direction is
    the orbit normal whenever the port axis lies in the orbit plane.

    Parameters
    ----------
    port_axis : array_like, shape (3,)
        Outward normal of the port, in the LVLH frame.

    Returns
    -------
    axis, lateral_1, lateral_2 : numpy.ndarray, shape (3,)

    Author: Giovanni Facchinetti, 2026
    """

    axis = np.asarray(port_axis, dtype=float)
    axis = axis / np.linalg.norm(axis)

    normal = np.array([0.0, 0.0, 1.0])
    if abs(axis @ normal) > 0.99:
        normal = np.array([1.0, 0.0, 0.0])

    lateral_2 = normal - (normal @ axis) * axis
    lateral_2 = lateral_2 / np.linalg.norm(lateral_2)
    lateral_1 = np.cross(lateral_2, axis)

    return axis, lateral_1, lateral_2


@dataclass
class PortState:
    """
    Relative state of the docking port of the deputy with respect to the
    port of the chief.

    Attributes
    ----------
    axial_distance : float
        Distance between the ports along the axis of the chief port,
        positive before contact [m].
    lateral_offset : numpy.ndarray, shape (2,)
        Offset across the axis [m].
    closing_speed : float
        Rate at which the axial distance decreases [m/s].
    lateral_velocity : numpy.ndarray, shape (2,)
        Velocity across the axis [m/s].
    axis_misalignment : float
        Angle between the axis of the deputy port and the inward axis of
        the chief port [rad].
    roll_misalignment : float
        Rotation of the deputy about the port axis with respect to its
        nominal orientation [rad].
    angular_rate : float
        Magnitude of the angular velocity of the deputy with respect to
        the chief [rad/s].

    Author: Giovanni Facchinetti, 2026
    """

    axial_distance: float
    lateral_offset: np.ndarray
    closing_speed: float
    lateral_velocity: np.ndarray
    axis_misalignment: float
    roll_misalignment: float
    angular_rate: float

    @property
    def lateral_misalignment(self):
        return float(np.hypot(*self.lateral_offset))

    @property
    def lateral_speed(self):
        return float(np.hypot(*self.lateral_velocity))


def port_state(relative_state, body_to_lvlh, body_rate_relative, chief_port_position, chief_port_axis,
               deputy_port_position_body, deputy_port_axis_body, deputy_roll_axis_body, basis=None):
    """
    Relative state of the two docking ports, from the relative state of
    the centers of mass and the attitude of the deputy.

    The port of the deputy is not at its center of mass, so its position
    and velocity depend on the attitude and on the body rate: a deputy
    that arrives with its center of mass exactly on the axis but slightly
    rotated presents its port off the axis.

    Parameters
    ----------
    relative_state : array_like, shape (6,)
        Position [m] and velocity [m/s] of the center of mass of the
        deputy in the LVLH frame of the chief.
    body_to_lvlh : numpy.ndarray, shape (3, 3)
        Rotation from the body frame of the deputy to the LVLH frame.
    body_rate_relative : array_like, shape (3,)
        Angular velocity of the deputy with respect to the LVLH frame,
        in body axes [rad/s].
    chief_port_position, chief_port_axis : array_like, shape (3,)
        Position [m] and outward normal of the chief port, in LVLH.
    deputy_port_position_body, deputy_port_axis_body : array_like, shape (3,)
        Position [m] and outward normal of the deputy port, in body axes.
    deputy_roll_axis_body : array_like, shape (3,)
        Body axis that is nominally along the second lateral direction
        of approach_basis.
    basis : tuple, optional
        Result of approach_basis(chief_port_axis), to avoid recomputing
        it at every call.

    Returns
    -------
    PortState

    Author: Giovanni Facchinetti, 2026
    """

    relative_state = np.asarray(relative_state, dtype=float)
    axis, lateral_1, lateral_2 = approach_basis(chief_port_axis) if basis is None else basis

    arm = body_to_lvlh @ np.asarray(deputy_port_position_body, dtype=float)
    arm_velocity = body_to_lvlh @ cross3(body_rate_relative, deputy_port_position_body)

    offset = relative_state[:3] + arm - np.asarray(chief_port_position, dtype=float)
    velocity = relative_state[3:] + arm_velocity

    deputy_axis = body_to_lvlh @ np.asarray(deputy_port_axis_body, dtype=float)
    axis_misalignment = np.arccos(min(1.0, max(-1.0, -(deputy_axis @ axis))))

    roll_axis = body_to_lvlh @ np.asarray(deputy_roll_axis_body, dtype=float)
    roll_misalignment = np.arctan2(roll_axis @ lateral_1, roll_axis @ lateral_2)

    return PortState(
        axial_distance=float(offset @ axis),
        lateral_offset=np.array([offset @ lateral_1, offset @ lateral_2]),
        closing_speed=float(-(velocity @ axis)),
        lateral_velocity=np.array([velocity @ lateral_1, velocity @ lateral_2]),
        axis_misalignment=float(axis_misalignment),
        roll_misalignment=float(roll_misalignment),
        angular_rate=norm3(body_rate_relative),
    )


@dataclass
class ContactAssessment:
    """
    Conditions at first contact compared with the limits of the docking
    mechanism.

    Attributes
    ----------
    values : dict
        The measured condition, by name.
    limits : dict
        The corresponding limit: a maximum, or a (min, max) pair.
    passed : dict
        Whether each condition is inside its limit.
    """

    values: dict
    limits: dict
    passed: dict

    @property
    def success(self):
        return all(self.passed.values())

    @property
    def violations(self):
        return [name for name, ok in self.passed.items() if not ok]


def assess_contact(state, limits):
    """
    Compare the port state at first contact with the limits of the
    docking mechanism.

    Parameters
    ----------
    state : PortState
    limits : ios_mission_planner.sim.config.DockingLimits

    Returns
    -------
    ContactAssessment

    Author: Giovanni Facchinetti, 2026
    Reference: International Docking System Standard, Interface
    Definition Document
    """

    values = {
        "lateral misalignment [m]": state.lateral_misalignment,
        "axial speed [m/s]": state.closing_speed,
        "lateral speed [m/s]": state.lateral_speed,
        "angular misalignment [deg]": np.degrees(state.axis_misalignment),
        "roll misalignment [deg]": abs(np.degrees(state.roll_misalignment)),
        "angular rate [deg/s]": np.degrees(state.angular_rate),
    }
    bounds = {
        "lateral misalignment [m]": limits.lateral_misalignment,
        "axial speed [m/s]": (limits.axial_speed_min, limits.axial_speed_max),
        "lateral speed [m/s]": limits.lateral_speed,
        "angular misalignment [deg]": limits.angular_misalignment_deg,
        "roll misalignment [deg]": limits.roll_misalignment_deg,
        "angular rate [deg/s]": limits.angular_rate_deg_s,
    }

    passed = {}
    for name, value in values.items():
        bound = bounds[name]
        if isinstance(bound, tuple):
            passed[name] = bool(bound[0] <= value <= bound[1])
        else:
            passed[name] = bool(value <= bound)

    return ContactAssessment(values=values, limits=bounds, passed=passed)
