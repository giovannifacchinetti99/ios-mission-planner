import math

import numpy as np

from ios_mission_planner.dynamics.attitude.determination import rotation_matrix_to_quaternion, triad_attitude
from ios_mission_planner.dynamics.attitude.quaternion import quaternion_conjugate, quaternion_multiply
from ios_mission_planner.dynamics.relative.cw import cw_state_transition_matrix, cw_targeting


def plan_transfer(state, target_position, time_of_flight, n, max_acceleration=None, target_velocity=(0.0, 0.0, 0.0)):
    """
    First burn of a two-impulse transfer to a target relative position,
    with the Clohessy-Wiltshire targeting.

    The targeting assumes an instantaneous change of velocity, while the
    thrusters need a finite time to deliver it. If max_acceleration is
    given, the burn is planned as if the impulse were applied at its
    midpoint: the state is first propagated, without thrust, over half
    the burn duration, and the transfer is solved from there with the
    time of flight reduced accordingly. This removes most of the error of
    the impulsive approximation; what remains is left to the mid-course
    corrections.

    Parameters
    ----------
    state : array_like, shape (6,)
        Current relative state in the LVLH frame [m, m/s].
    target_position : array_like, shape (3,)
        Relative position to reach [m].
    time_of_flight : float
        Time from now to the arrival [s].
    n : float
        Mean motion of the chief [rad/s].
    max_acceleration : float, optional
        Acceleration available for the burn [m/s^2].
    target_velocity : array_like, shape (3,)
        Relative velocity wanted after the arrival burn [m/s].

    Returns
    -------
    dv : numpy.ndarray, shape (3,)
        Velocity change of the first burn [m/s].
    burn_duration : float
        Time needed to deliver it [s]; zero if max_acceleration is not
        given.

    Author: Giovanni Facchinetti, 2026
    """

    state = np.asarray(state, dtype=float)

    dv, _ = cw_targeting(state[:3], state[3:], target_position, target_velocity, time_of_flight, n)
    if max_acceleration is None:
        return dv, 0.0

    burn_duration = 0.0
    for _ in range(3):
        burn_duration = np.sqrt(dv @ dv) / max_acceleration
        half = min(0.5 * burn_duration, 0.5 * time_of_flight)
        midpoint_state = cw_state_transition_matrix(half, n) @ state
        dv, _ = cw_targeting(
            midpoint_state[:3], midpoint_state[3:], target_position, target_velocity,
            time_of_flight - half, n,
        )

    return dv, np.sqrt(dv @ dv) / max_acceleration


class GlideslopeReference:
    """
    Reference point (the "carrot") that moves along a straight line
    toward an end point, with a speed that decreases with the distance
    still to cover,

        v(d) = min(v_max, max(v_min, d / tau)),

    the glideslope law used for forced approaches: fast while far, slower
    and slower while closing, so that the time needed to stop is always
    available. The speed of the carrot changes with a limited
    acceleration, so the reference never asks for a step in velocity that
    the thrusters could not follow, and if a braking acceleration is
    given the carrot also slows down to rest at the end point.

    Parameters
    ----------
    start, end : array_like, shape (3,)
        End points of the line, in the LVLH frame [m].
    v_max, v_min : float
        Largest and smallest speed of the carrot [m/s].
    time_constant : float
        Ratio tau between distance to go and speed [s].
    max_acceleration : float
        Limit on the acceleration of the carrot [m/s^2].
    brake : bool
        Whether the carrot stops at the end point. If not, it keeps the
        speed v_min until it gets there.

    Author: Giovanni Facchinetti, 2026
    Reference: Fehse, Automated Rendezvous and Docking of Spacecraft;
    Hablani, Tapper, Dana-Bashian, "Guidance and Relative Navigation for
    Autonomous Rendezvous in a Circular Orbit," Journal of Guidance,
    Control, and Dynamics, 25(3), 2002
    """

    def __init__(self, start, end, v_max, v_min, time_constant, max_acceleration, brake=True):
        self.start = np.asarray(start, dtype=float)
        self.end = np.asarray(end, dtype=float)
        line = self.end - self.start
        self.length = np.sqrt(line @ line)
        self.direction = line / self.length

        self.v_max = v_max
        self.v_min = v_min
        self.time_constant = time_constant
        self.max_acceleration = max_acceleration
        self.brake = brake

        self.distance_to_go = self.length
        self.speed = 0.0

    def _desired_speed(self, distance_to_go):
        if distance_to_go <= 0.0:
            return 0.0
        speed = min(self.v_max, max(self.v_min, distance_to_go / self.time_constant))
        if self.brake:
            speed = min(speed, math.sqrt(2.0 * self.max_acceleration * distance_to_go))
        return speed

    def _step(self, distance_to_go, speed, dt):
        desired = self._desired_speed(distance_to_go)
        limit = self.max_acceleration * dt
        change = min(max(desired - speed, -limit), limit)
        new_speed = speed + change
        new_distance = distance_to_go - 0.5 * (speed + new_speed) * dt
        if new_distance <= 0.0:
            return 0.0, 0.0, 0.0
        return new_distance, new_speed, change / dt

    def advance(self, dt):
        """Move the carrot forward by dt seconds."""

        self.distance_to_go, self.speed, _ = self._step(self.distance_to_go, self.speed, dt)

    @property
    def arrived(self):
        return self.distance_to_go <= 0.0

    @property
    def position(self):
        return self.end - self.distance_to_go * self.direction

    @property
    def velocity(self):
        return self.speed * self.direction

    def horizon(self, dt, steps, n):
        """
        Reference over the next steps, without moving the carrot.

        Parameters
        ----------
        dt : float
            Step of the horizon [s].
        steps : int
            Number of steps.
        n : float
            Mean motion of the chief, for the feedforward [rad/s].

        Returns
        -------
        reference : numpy.ndarray, shape (steps + 1, 6)
            Reference states, starting from the present one.
        feedforward : numpy.ndarray, shape (steps, 3)
            Acceleration that follows the reference exactly [m/s^2].
        """

        distances = np.empty(steps + 1)
        speeds = np.empty(steps + 1)
        accelerations = np.empty(steps)

        distance, speed = self.distance_to_go, self.speed
        for k in range(steps):
            distances[k], speeds[k] = distance, speed
            distance, speed, accelerations[k] = self._step(distance, speed, dt)
        distances[steps], speeds[steps] = distance, speed

        positions = self.end - distances[:, None] * self.direction
        velocities = speeds[:, None] * self.direction
        reference = np.hstack([positions, velocities])

        # same expression as cw_feedforward_acceleration, for all steps at once
        natural = np.stack([
            3.0 * n**2 * positions[:-1, 0] + 2.0 * n * velocities[:-1, 1],
            -2.0 * n * velocities[:-1, 0],
            -(n**2) * positions[:-1, 2],
        ], axis=1)
        feedforward = accelerations[:, None] * self.direction - natural

        return reference, feedforward


def pointing_quaternion(lvlh_to_inertial, boresight_direction_lvlh, secondary_direction_lvlh,
                        boresight_body=(1.0, 0.0, 0.0), secondary_body=(0.0, 0.0, 1.0), previous=None):
    """
    Commanded attitude, as a quaternion from the inertial to the body
    frame, that points a body axis along a direction given in the LVLH
    frame of the chief, and keeps a second body axis as close as possible
    to a second direction.

    The two directions are first rotated to the inertial frame, so the
    command is a true inertial attitude: it contains the rotation of the
    LVLH frame, once per orbit, that a deputy facing the chief must
    follow.

    Parameters
    ----------
    lvlh_to_inertial : numpy.ndarray, shape (3, 3)
        Rotation from the LVLH frame of the chief to the inertial frame.
    boresight_direction_lvlh, secondary_direction_lvlh : array_like, shape (3,)
        Directions along which the two body axes must point, in the LVLH
        frame.
    boresight_body, secondary_body : array_like, shape (3,)
        The two body axes, in body coordinates.
    previous : array_like, shape (4,), optional
        Previous command. The sign of the result is chosen to be
        continuous with it, since q and -q are the same attitude.

    Returns
    -------
    numpy.ndarray, shape (4,)

    Author: Giovanni Facchinetti, 2026
    """

    primary = lvlh_to_inertial @ np.asarray(boresight_direction_lvlh, dtype=float)
    secondary = lvlh_to_inertial @ np.asarray(secondary_direction_lvlh, dtype=float)

    rotation = triad_attitude(boresight_body, secondary_body, primary, secondary)
    quaternion = rotation_matrix_to_quaternion(rotation)

    if previous is not None and quaternion @ np.asarray(previous) < 0.0:
        quaternion = -quaternion

    return quaternion


def command_rate(previous, current, dt):
    """
    Angular velocity of a commanded attitude, in its own body frame,
    from two successive commands dt seconds apart.

    Author: Giovanni Facchinetti, 2026
    """

    delta = quaternion_multiply(quaternion_conjugate(previous), current)
    if delta[0] < 0.0:
        delta = -delta

    return 2.0 * delta[1:] / dt
