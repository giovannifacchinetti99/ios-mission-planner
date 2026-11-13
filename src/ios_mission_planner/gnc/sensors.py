from dataclasses import dataclass, field

import numpy as np

from ios_mission_planner.dynamics.attitude.quaternion import quaternion_multiply, rotation_vector_to_quaternion


@dataclass
class Gyro:
    """
    Rate gyro: the body rate plus a slowly drifting bias and white noise.

    The bias is the reason the gyro alone cannot give the attitude: its
    integral grows without bound. It performs a random walk, so the
    attitude filter must keep estimating it.

    Attributes
    ----------
    noise_std : float
        Standard deviation of the white noise on each output sample
        [rad/s].
    bias_walk_std : float
        Random walk of the bias [rad/s per sqrt(s)].
    bias : numpy.ndarray, shape (3,)
        Current bias [rad/s].

    Author: Giovanni Facchinetti, 2026
    """

    noise_std: float
    bias_walk_std: float
    bias: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def measure(self, body_rate, dt, rng):
        """Measured body rate [rad/s]; advances the bias by one step dt."""

        self.bias = self.bias + self.bias_walk_std * np.sqrt(dt) * rng.standard_normal(3)
        return np.asarray(body_rate) + self.bias + self.noise_std * rng.standard_normal(3)


@dataclass
class StarTracker:
    """
    Star tracker: the inertial attitude with a small random rotation
    error, the same on the three axes.

    Attributes
    ----------
    noise_std : float
        Standard deviation of the error about each body axis [rad].

    Author: Giovanni Facchinetti, 2026
    """

    noise_std: float

    def measure(self, quaternion, rng):
        """Measured quaternion, inertial to body, scalar first."""

        error = rotation_vector_to_quaternion(self.noise_std * rng.standard_normal(3))
        return quaternion_multiply(quaternion, error)


@dataclass
class RelativeMeasurement:
    """
    One measurement of the relative sensor.

    Attributes
    ----------
    range : float
        Measured distance from the sensor to the target [m].
    line_of_sight : numpy.ndarray, shape (3,)
        Measured unit vector from the sensor to the target, in the body
        frame.
    range_std : float
        Standard deviation of the range error [m].
    angle_std : float
        Standard deviation of the direction error, per axis [rad].
    """

    range: float
    line_of_sight: np.ndarray
    range_std: float
    angle_std: float


@dataclass
class RelativeSensor:
    """
    Relative navigation sensor fixed to the body of the deputy, of the
    kind obtained from a camera and a laser range finder looking at a
    target pattern on the docking port of the chief. It measures the
    distance and the direction of the target in the body frame.

    The range error grows with the distance, and the direction error is
    constant in angle, so the position error across the line of sight
    also grows with the distance: the measurement is poor far away and
    becomes accurate exactly where accuracy is needed. The sensor returns
    nothing when the target is outside its field of view or beyond its
    maximum range.

    Attributes
    ----------
    position_body : numpy.ndarray, shape (3,)
        Position of the sensor in the body frame [m].
    boresight_body : numpy.ndarray, shape (3,)
        Unit vector of the optical axis in the body frame.
    fov_half_angle : float
        Half-angle of the conical field of view [rad].
    max_range : float
        Maximum range at which the target is detected [m].
    range_noise_floor : float
        Range error at short distance [m].
    range_noise_fraction : float
        Range error as a fraction of the distance.
    angle_noise_std : float
        Direction error, per axis [rad].

    Author: Giovanni Facchinetti, 2026
    Reference: Fehse, Automated Rendezvous and Docking of Spacecraft
    """

    position_body: np.ndarray
    boresight_body: np.ndarray
    fov_half_angle: float
    max_range: float
    range_noise_floor: float
    range_noise_fraction: float
    angle_noise_std: float

    def range_std(self, distance):
        """Standard deviation of the range error at a given distance [m]."""

        return max(self.range_noise_floor, self.range_noise_fraction * distance)

    def measure(self, target_position_body, rng, misalignment=None):
        """
        Measure a target.

        Parameters
        ----------
        target_position_body : array_like, shape (3,)
            True position of the target with respect to the center of
            mass of the deputy, in the body frame [m].
        rng : numpy.random.Generator
        misalignment : numpy.ndarray, shape (3, 3), optional
            Rotation from the nominal to the true axes of the sensor: a
            mounting error, which rotates every measured direction by
            the same small angle.

        Returns
        -------
        RelativeMeasurement or None
            None when the target is not visible.
        """

        vector = np.asarray(target_position_body, dtype=float) - self.position_body
        if misalignment is not None:
            vector = misalignment @ vector
        distance = np.sqrt(vector @ vector)
        if distance > self.max_range or distance < 1e-6:
            return None

        direction = vector / distance
        if direction @ self.boresight_body < np.cos(self.fov_half_angle):
            return None

        range_std = self.range_std(distance)
        measured_range = distance + range_std * rng.standard_normal()

        # small random rotation of the direction, in the plane normal to it
        perturbed = direction + self.angle_noise_std * rng.standard_normal(3)
        perturbed = perturbed - (perturbed @ direction - 1.0) * direction
        perturbed = perturbed / np.sqrt(perturbed @ perturbed)

        return RelativeMeasurement(
            range=measured_range, line_of_sight=perturbed,
            range_std=range_std, angle_std=self.angle_noise_std,
        )
