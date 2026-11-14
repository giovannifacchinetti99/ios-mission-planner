from dataclasses import dataclass, field

import numpy as np

from ios_mission_planner.budget.propellant import G0
from ios_mission_planner.linalg import cross3


@dataclass
class ThrusterSet:
    """
    Reaction control thrusters for translation: on-off thrusters of fixed
    thrust along the positive and negative direction of each body axis,
    assumed balanced so that firing them produces no torque about the
    nominal center of mass.

    A thruster cannot produce an arbitrary force. It is either on or off,
    and it cannot be on for less than a minimum time, which defines the
    smallest impulse it can deliver, the minimum impulse bit. The
    modulator turns a requested force into firings: on each axis it
    accumulates the requested impulse and, once this exceeds the minimum
    impulse bit, fires for the time needed to deliver it, up to the whole
    step. Requests too small to fire are not lost, they stay in the
    accumulator until they add up to a pulse. The force returned is the
    average over the step.

    The force actually produced differs from the one the modulator
    intended by a fixed scale error per axis, a fixed misalignment of the
    whole set, and a random pulse-to-pulse variation.

    Attributes
    ----------
    max_force : float
        Thrust available along each direction of each body axis [N].
    isp : float
        Specific impulse [s].
    min_impulse_bit : float
        Smallest impulse of one firing [N s].
    scale_error : numpy.ndarray, shape (3,)
        Fractional error of the thrust on each axis.
    misalignment : numpy.ndarray, shape (3,)
        Rotation vector of the small rotation between intended and
        actual thrust directions [rad].
    pulse_noise_std : float
        Standard deviation of the fractional pulse-to-pulse variation.
    center_of_mass_offset : numpy.ndarray, shape (3,)
        Offset of the true center of mass from the point through which
        the thrust lines pass, in body axes [m]. It turns the thrust into
        a disturbance torque.

    Author: Giovanni Facchinetti, 2026
    Reference: Wie, Space Vehicle Dynamics and Control
    """

    max_force: float
    isp: float
    min_impulse_bit: float
    scale_error: np.ndarray = field(default_factory=lambda: np.zeros(3))
    misalignment: np.ndarray = field(default_factory=lambda: np.zeros(3))
    pulse_noise_std: float = 0.0
    center_of_mass_offset: np.ndarray = field(default_factory=lambda: np.zeros(3))
    _accumulator: np.ndarray = field(default_factory=lambda: np.zeros(3), repr=False)

    def reset(self):
        """Empty the impulse accumulator."""

        self._accumulator = np.zeros(3)

    def command(self, force_body, dt, rng=None):
        """
        Turn a requested body-frame force into the force of the firings
        over one step.

        Parameters
        ----------
        force_body : array_like, shape (3,)
            Requested force [N]. Each component is limited to max_force.
        dt : float
            Duration of the step [s].
        rng : numpy.random.Generator, optional
            Source of the pulse-to-pulse variation. Without it the
            variation is not applied.

        Returns
        -------
        intended : numpy.ndarray, shape (3,)
            Average force of the commanded firings [N]: what the
            on-board software knows it has commanded.
        actual : numpy.ndarray, shape (3,)
            Average force really produced [N].
        mass_flow : float
            Average propellant mass flow over the step [kg/s].
        torque : numpy.ndarray, shape (3,)
            Disturbance torque caused by the center of mass offset [N m].
        """

        requested = np.clip(np.asarray(force_body, dtype=float), -self.max_force, self.max_force)
        self._accumulator = self._accumulator + requested * dt

        impulse = np.minimum(np.abs(self._accumulator), self.max_force * dt)
        impulse = np.where(impulse >= self.min_impulse_bit, impulse, 0.0)
        impulse = np.sign(self._accumulator) * impulse
        self._accumulator = self._accumulator - impulse

        intended = impulse / dt

        actual = intended * (1.0 + self.scale_error)
        if rng is not None and self.pulse_noise_std > 0.0:
            actual = actual * (1.0 + self.pulse_noise_std * rng.standard_normal(3))
        mass_flow = np.sum(np.abs(actual)) / (G0 * self.isp)
        actual = actual + cross3(self.misalignment, actual)

        torque = cross3(-self.center_of_mass_offset, actual)

        return intended, actual, mass_flow, torque


@dataclass
class ReactionWheels:
    """
    Three reaction wheels along the body axes. A wheel exchanges angular
    momentum with the body: the torque it applies to the body is limited
    by its motor, and the momentum it can store is limited by its maximum
    speed. A wheel at its momentum limit cannot push further in the same
    direction; it can still be slowed down.

    Attributes
    ----------
    max_torque : float
        Maximum torque of each wheel [N m].
    max_momentum : float
        Maximum angular momentum of each wheel [N m s].

    Author: Giovanni Facchinetti, 2026
    Reference: Markley, Crassidis, Fundamentals of Spacecraft Attitude
    Determination and Control
    """

    max_torque: float
    max_momentum: float

    def command(self, torque_body, wheel_momentum, dt):
        """
        Torque that the wheels really apply to the body over one step.

        Parameters
        ----------
        torque_body : array_like, shape (3,)
            Requested torque on the body [N m].
        wheel_momentum : array_like, shape (3,)
            Momentum currently stored in the wheels [N m s].
        dt : float
            Duration of the step [s].

        Returns
        -------
        numpy.ndarray, shape (3,)
        """

        torque = np.clip(np.asarray(torque_body, dtype=float), -self.max_torque, self.max_torque)
        wheel_momentum = np.asarray(wheel_momentum, dtype=float)

        # the wheel momentum changes at the rate -torque: keep it inside
        # the limits at the end of the step
        lower = (wheel_momentum - self.max_momentum) / dt
        upper = (wheel_momentum + self.max_momentum) / dt

        return np.clip(torque, np.minimum(lower, 0.0), np.maximum(upper, 0.0))

    def saturated(self, wheel_momentum, tolerance=1e-3):
        """Which wheels are at their momentum limit."""

        return np.abs(np.asarray(wheel_momentum)) >= self.max_momentum * (1.0 - tolerance)


@dataclass
class MomentumManager:
    """
    Unloading of the reaction wheels with thruster couples.

    A persistent disturbance torque, such as the one produced by thrust
    lines that do not pass exactly through the center of mass, makes the
    wheels accumulate momentum until they saturate, and a saturated wheel
    no longer controls its axis. The remedy is an external torque that
    only thrusters can provide. When the momentum of a wheel exceeds a
    threshold, a pair of thrusters firing in opposite directions applies
    a pure torque to the body, and the wheel is commanded to apply the
    opposite torque at the same instant: the body feels no net torque,
    and the wheel is slowed down. The cancellation matters. If the wheel
    were left to oppose the couple through the attitude feedback alone,
    the attitude would have to drift by degrees before the feedback
    produced enough torque. The couple stays on until the momentum has
    fallen below a lower threshold, so that it does not switch on and
    off around a single value.

    Attributes
    ----------
    start, stop : float
        Momentum at which the unloading of a wheel starts and stops
        [N m s].
    torque : float
        Torque of the thruster couple [N m]. It must exceed the largest
        disturbance torque that the thrusters can produce, and stay
        below what the wheel can oppose.
    arm : float
        Distance between the two thrusters of a couple [m].
    isp : float
        Specific impulse of the thrusters [s].

    Author: Giovanni Facchinetti, 2026
    Reference: Wie, Space Vehicle Dynamics and Control
    """

    start: float
    stop: float
    torque: float
    arm: float
    isp: float
    _active: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=bool), repr=False)

    def command(self, wheel_momentum):
        """
        Torque of the thruster couples for the current wheel momentum.

        Returns
        -------
        torque : numpy.ndarray, shape (3,)
            External torque applied to the body [N m].
        mass_flow : float
            Propellant mass flow of the couples [kg/s].
        """

        magnitude = np.abs(np.asarray(wheel_momentum, dtype=float))
        self._active = np.where(self._active, magnitude > self.stop, magnitude > self.start)

        # the wheel momentum changes at the rate of the external torque it
        # opposes: a negative torque unloads a positive momentum
        torque = np.where(self._active, -np.sign(wheel_momentum) * self.torque, 0.0)
        mass_flow = 2.0 * np.count_nonzero(self._active) * (self.torque / self.arm) / (G0 * self.isp)

        return torque, mass_flow

    @property
    def active(self):
        return self._active.copy()
