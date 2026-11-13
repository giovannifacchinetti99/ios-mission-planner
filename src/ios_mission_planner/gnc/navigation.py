import numpy as np

from ios_mission_planner.dynamics.relative.cw import cw_discrete_matrices


def relative_position_from_measurement(measurement, sensor_position_body, body_to_lvlh, target_position_lvlh,
                                       attitude_std=0.0):
    """
    Convert a range and line-of-sight measurement of a target fixed to
    the chief into a measurement of the relative position of the deputy
    in the LVLH frame of the chief, with its covariance.

    The sensor sees the vector from itself to the target in the body
    frame. Rotating it to the LVLH frame with the estimated attitude and
    subtracting it from the known position of the target gives the
    position of the sensor, and from it the position of the center of
    mass of the deputy. The error of this converted measurement is not
    the same in every direction: along the line of sight it is the range
    error, across it the angular error multiplied by the distance, to
    which the attitude knowledge error adds in the same way. The
    covariance returned keeps this shape, so the filter can trust each
    direction as much as it deserves.

    Parameters
    ----------
    measurement : ios_mission_planner.gnc.sensors.RelativeMeasurement
    sensor_position_body : array_like, shape (3,)
        Position of the sensor in the body frame [m].
    body_to_lvlh : numpy.ndarray, shape (3, 3)
        Rotation from the body frame of the deputy to the LVLH frame of
        the chief, from the estimated attitude.
    target_position_lvlh : array_like, shape (3,)
        Position of the target in the LVLH frame of the chief [m].
    attitude_std : float
        Standard deviation of the attitude knowledge error, per axis
        [rad].

    Returns
    -------
    position : numpy.ndarray, shape (3,)
        Measured relative position of the deputy [m].
    covariance : numpy.ndarray, shape (3, 3)
        Covariance of its error [m^2].

    Author: Giovanni Facchinetti, 2026
    Reference: Bar-Shalom, Li, Kirubarajan, Estimation with Applications
    to Tracking and Navigation
    """

    los = measurement.line_of_sight
    sensor_to_target_body = measurement.range * los

    position = np.asarray(target_position_lvlh) - body_to_lvlh @ (sensor_to_target_body + sensor_position_body)

    along = np.outer(los, los)
    cross_var = measurement.range**2 * (measurement.angle_std**2 + attitude_std**2)
    covariance_body = measurement.range_std**2 * along + cross_var * (np.eye(3) - along)
    covariance = body_to_lvlh @ covariance_body @ body_to_lvlh.T

    return position, covariance


class RelativeNavigationFilter:
    """
    Kalman filter for the relative position and velocity of the deputy in
    the LVLH frame of the chief.

    The dynamic model is the Clohessy-Wiltshire system, discretized
    exactly for the acceleration commanded to the thrusters, held
    constant over the step. The measurement is the relative position
    obtained from the relative sensor. Everything the model leaves out
    enters as process noise, in three parts. A white acceleration of
    constant spectral density stands for the differential perturbations.
    A term proportional to the commanded acceleration stands for the
    execution errors of the thrusters, larger for larger burns. And a
    term that grows with the square of the distance stands for the
    nonlinearity of the relative motion: the linear model drops the
    second-order terms of the gravity gradient, of magnitude 3/2 n^2
    rho^2 / R, which at a few kilometres are far larger than any
    perturbation. This last error is slowly varying, not white; it is
    represented by a white acceleration of the same magnitude and a
    correlation time 1/n.

    Parameters
    ----------
    n : float
        Mean motion of the chief [rad/s].
    dt : float
        Step of the filter [s].
    state : array_like, shape (6,)
        Initial estimate [x, y, z, vx, vy, vz] in metres and metres per
        second.
    covariance : array_like, shape (6, 6)
        Initial covariance.
    accel_noise_std : float
        Square root of the spectral density of the unmodelled
        acceleration [m/s^2 per sqrt(Hz)].
    thrust_noise_fraction : float
        Standard deviation of the thrust execution error, as a fraction
        of the commanded acceleration.
    orbit_radius : float, optional
        Radius of the orbit of the chief [m]. If given, the process noise
        includes the term for the nonlinearity of the relative motion.

    Author: Giovanni Facchinetti, 2026
    Reference: Bar-Shalom, Li, Kirubarajan, Estimation with Applications
    to Tracking and Navigation
    """

    def __init__(self, n, dt, state, covariance, accel_noise_std, thrust_noise_fraction=0.0,
                 orbit_radius=None):
        self.n = n
        self.dt = dt
        self.state = np.array(state, dtype=float)
        self.covariance = np.array(covariance, dtype=float)
        self.thrust_noise_fraction = thrust_noise_fraction
        self.orbit_radius = orbit_radius

        self.phi, self.gamma = cw_discrete_matrices(dt, n)

        # discrete process noise of a white acceleration over one step
        q = accel_noise_std**2
        self._q_unit = np.zeros((6, 6))
        self._q_unit[:3, :3] = np.eye(3) * dt**3 / 3.0
        self._q_unit[:3, 3:] = np.eye(3) * dt**2 / 2.0
        self._q_unit[3:, :3] = np.eye(3) * dt**2 / 2.0
        self._q_unit[3:, 3:] = np.eye(3) * dt
        self._q_base = q * self._q_unit

        self.last_innovation = None
        self.last_nis = None

        # recovery from a diverged estimate, see update
        self.rejections = 0
        self.max_rejections = 3
        self.recovery_time = 5.0

    def predict(self, acceleration):
        """
        Advance the estimate by one step.

        Parameters
        ----------
        acceleration : array_like, shape (3,)
            Acceleration commanded over the step, in the LVLH frame
            [m/s^2].
        """

        acceleration = np.asarray(acceleration, dtype=float)

        process_noise = self._q_base
        if self.orbit_radius is not None:
            position = self.state[:3]
            neglected = 1.5 * self.n**2 * (position @ position) / self.orbit_radius
            process_noise = process_noise + (2.0 * neglected**2 / self.n) * self._q_unit

        self.state = self.phi @ self.state + self.gamma @ acceleration

        if self.thrust_noise_fraction > 0.0:
            # the execution error is constant over the step, not white:
            # it enters through the input matrix
            error_std = self.thrust_noise_fraction * np.sqrt(acceleration @ acceleration)
            if error_std > 0.0:
                process_noise = process_noise + error_std**2 * (self.gamma @ self.gamma.T)

        self.covariance = self.phi @ self.covariance @ self.phi.T + process_noise

    def update(self, position, position_covariance, gate=None):
        """
        Correct the estimate with a measurement of the relative position.

        Parameters
        ----------
        position : array_like, shape (3,)
            Measured relative position [m].
        position_covariance : array_like, shape (3, 3)
            Covariance of the measurement error [m^2].
        gate : float, optional
            Threshold on the normalized innovation squared. A measurement
            whose innovation is larger than the filter considers
            plausible is rejected as an outlier. By default nothing is
            rejected.

        A single implausible measurement is an outlier. Several in a row
        mean that it is the estimate that is wrong, typically because the
        deputy felt an acceleration the model does not know about. After
        max_rejections consecutive rejections the filter therefore stops
        defending its estimate: it enlarges its covariance to the size of
        the disagreement and accepts the measurement.

        Returns
        -------
        bool
            Whether the measurement was used.
        """

        p = self.covariance
        innovation = np.asarray(position, dtype=float) - self.state[:3]
        s = p[:3, :3] + position_covariance
        s_inv = np.linalg.inv(s)

        self.last_innovation = innovation
        self.last_nis = float(innovation @ s_inv @ innovation)

        if gate is not None and self.last_nis > gate:
            self.rejections += 1
            if self.rejections < self.max_rejections:
                return False

            disagreement = innovation @ innovation
            p = p.copy()
            p[:3, :3] += disagreement * np.eye(3)
            p[3:, 3:] += disagreement / self.recovery_time**2 * np.eye(3)
            s_inv = np.linalg.inv(p[:3, :3] + position_covariance)

        self.rejections = 0
        gain = p[:, :3] @ s_inv
        self.state = self.state + gain @ innovation

        # Joseph form: stays symmetric and positive definite
        i_kh = np.eye(6)
        i_kh[:, :3] -= gain
        self.covariance = i_kh @ p @ i_kh.T + gain @ position_covariance @ gain.T

        return True

    @property
    def position_std(self):
        """Standard deviation of the position estimate, per axis [m]."""

        return np.sqrt(np.diag(self.covariance)[:3])

    @property
    def velocity_std(self):
        """Standard deviation of the velocity estimate, per axis [m/s]."""

        return np.sqrt(np.diag(self.covariance)[3:])
