from dataclasses import dataclass, field

import numpy as np

from ios_mission_planner.budget.propellant import G0
from ios_mission_planner.dynamics.attitude.determination import quaternion_to_rotation_matrix
from ios_mission_planner.dynamics.attitude.filter import MEKFState, mekf_predict, mekf_update
from ios_mission_planner.dynamics.attitude.quaternion import (
    quaternion_error,
    quaternion_multiply,
    quaternion_to_rotation_vector,
    rotation_vector_to_quaternion,
)
from ios_mission_planner.dynamics.orbital.elements import ClassicalElements, cartesian_state_from_elements
from ios_mission_planner.dynamics.relative.lvlh import lvlh_to_inertial
from ios_mission_planner.gnc.actuators import MomentumManager, ReactionWheels, ThrusterSet
from ios_mission_planner.gnc.control import attitude_gains, quaternion_feedback_torque
from ios_mission_planner.gnc.guidance import command_rate, pointing_quaternion
from ios_mission_planner.gnc.mission_manager import PHASE_LABELS, MissionManager, Phase
from ios_mission_planner.gnc.navigation import RelativeNavigationFilter, relative_position_from_measurement
from ios_mission_planner.gnc.sensors import Gyro, RelativeSensor, StarTracker
from ios_mission_planner.illumination.eclipse import in_eclipse
from ios_mission_planner.illumination.lighting import port_lighting_timeline
from ios_mission_planner.illumination.sun import julian_date_from_datetime, sun_direction
from ios_mission_planner.linalg import cross3, norm3
from ios_mission_planner.sim.config import SimulationConfig
from ios_mission_planner.sim.docking import PortState, assess_contact, port_state
from ios_mission_planner.sim.truth import TruthModel

DOCKED = "docked"
CONTACT_OUT_OF_LIMITS = "contact out of limits"
COLLISION = "collision"
ABORTED = "aborted"
TIMEOUT = "timeout"

# beyond this distance from the port axis a crossing of the port plane is
# not a contact between the ports
_CONTACT_RADIUS = 1.5


@dataclass
class SimulationResult:
    """
    Time histories and outcome of one closed-loop simulation.

    All arrays have one row per step. Quantities marked "true" come from
    the truth model, the others are what the on-board software believed
    or commanded.

    Attributes
    ----------
    t : numpy.ndarray
        Time [s].
    phase : numpy.ndarray of int
        Mission phase, see ios_mission_planner.gnc.mission_manager.Phase.
    relative_state : numpy.ndarray, shape (n, 6)
        True relative state of the deputy in the LVLH frame of the chief
        [m, m/s].
    relative_estimate, relative_std : numpy.ndarray, shape (n, 6)
        Navigation estimate of the relative state and its standard
        deviation.
    measurement_valid : numpy.ndarray of bool
        Whether the relative sensor returned a measurement.
    quaternion, quaternion_command, quaternion_estimate : numpy.ndarray, shape (n, 4)
        True, commanded and estimated attitude, inertial to body.
    body_to_lvlh : numpy.ndarray, shape (n, 3, 3)
        True rotation from the body frame of the deputy to the LVLH frame.
    body_rate : numpy.ndarray, shape (n, 3)
        True angular velocity, body frame [rad/s].
    gyro_bias, gyro_bias_estimate : numpy.ndarray, shape (n, 3)
        True and estimated gyro bias [rad/s].
    attitude_error, attitude_knowledge_error : numpy.ndarray
        Angle between true and commanded attitude, and between true and
        estimated attitude [rad].
    wheel_momentum : numpy.ndarray, shape (n, 3)
        Momentum stored in the wheels [N m s].
    wheel_torque : numpy.ndarray, shape (n, 3)
        Torque applied by the wheels to the body [N m].
    mass : numpy.ndarray
        True mass of the deputy [kg].
    acceleration_command : numpy.ndarray, shape (n, 3)
        Commanded acceleration, LVLH [m/s^2].
    thrust : numpy.ndarray, shape (n, 3)
        Force actually produced by the thrusters, body frame [N].
    delta_v : numpy.ndarray
        Cumulative velocity change delivered by the thrusters [m/s].
    port_axial, port_lateral, port_closing_speed, port_lateral_speed : numpy.ndarray
        True relative state of the two docking ports, see
        ios_mission_planner.sim.docking.PortState.
    port_axis_misalignment, port_roll_misalignment, port_angular_rate : numpy.ndarray
        Same, angular part [rad, rad/s].
    sun_lvlh : numpy.ndarray, shape (n, 3)
        Unit vector toward the Sun in the LVLH frame.
    eclipse : numpy.ndarray of bool
        Whether the chief is in the shadow of the Earth.
    events : list of (time, phase, note)
        Phase changes and burns.
    outcome : str
        One of "docked", "contact out of limits", "collision", "aborted",
        "timeout".
    contact_state : PortState or None
        Port state at first contact.
    contact : ContactAssessment or None
        Its comparison with the limits of the docking mechanism.
    abort_reason : str or None
    config : SimulationConfig

    Author: Giovanni Facchinetti, 2026
    """

    t: np.ndarray
    phase: np.ndarray
    relative_state: np.ndarray
    relative_estimate: np.ndarray
    relative_std: np.ndarray
    measurement_valid: np.ndarray
    quaternion: np.ndarray
    quaternion_command: np.ndarray
    quaternion_estimate: np.ndarray
    body_to_lvlh: np.ndarray
    body_rate: np.ndarray
    gyro_bias: np.ndarray
    gyro_bias_estimate: np.ndarray
    attitude_error: np.ndarray
    attitude_knowledge_error: np.ndarray
    wheel_momentum: np.ndarray
    wheel_torque: np.ndarray
    mass: np.ndarray
    acceleration_command: np.ndarray
    thrust: np.ndarray
    delta_v: np.ndarray
    port_axial: np.ndarray
    port_lateral: np.ndarray
    port_closing_speed: np.ndarray
    port_lateral_speed: np.ndarray
    port_axis_misalignment: np.ndarray
    port_roll_misalignment: np.ndarray
    port_angular_rate: np.ndarray
    sun_lvlh: np.ndarray
    eclipse: np.ndarray
    events: list
    outcome: str
    contact_state: PortState
    contact: object
    abort_reason: str
    config: SimulationConfig
    hold_positions: list = field(default_factory=list)
    contact_position: np.ndarray = None

    @property
    def duration(self):
        return float(self.t[-1])

    @property
    def total_delta_v(self):
        return float(self.delta_v[-1])

    @property
    def propellant_used(self):
        return float(self.mass[0] - self.mass[-1])

    @property
    def success(self):
        return self.outcome == DOCKED

    def delta_v_by_phase(self):
        """Velocity change delivered in each phase, as {label: m/s}."""

        spent = {}
        increments = np.diff(self.delta_v, prepend=0.0)
        for phase in Phase:
            mask = self.phase == int(phase)
            if mask.any():
                spent[PHASE_LABELS[phase]] = float(increments[mask].sum())
        return spent

    def phase_intervals(self):
        """Start and end time of each phase flown, as {label: (start, end)}."""

        spans = {}
        for phase in Phase:
            mask = self.phase == int(phase)
            if mask.any():
                times = self.t[mask]
                spans[PHASE_LABELS[phase]] = (float(times[0]), float(times[-1]))
        return spans


def _initial_chief_state(config):
    env, chief = config.environment, config.chief
    elements = ClassicalElements(
        a=env.r_eq + chief.altitude_km, e=0.0, i=np.deg2rad(chief.inclination_deg),
        raan=np.deg2rad(chief.raan_deg), argp=0.0,
        mean_anomaly=np.deg2rad(chief.argument_of_latitude_deg),
    )
    return cartesian_state_from_elements(elements, env.mu)


def _lvlh_frame(chief_state):
    """LVLH rotation (inertial to LVLH) and angular rate of the chief."""

    r, v = chief_state[:3], chief_state[3:]
    r_norm = norm3(r)
    h = cross3(r, v)
    h_norm = norm3(h)
    r_hat = r / r_norm
    h_hat = h / h_norm

    return np.array([r_hat, cross3(h_hat, r_hat), h_hat]), h_norm / r_norm**2


def simulate(config=None, truth=None, progress=None):
    """
    Run one closed-loop simulation of the approach and docking.

    At every step the loop does what the spacecraft does. The sensors
    observe the true state with their errors. The navigation corrects its
    estimates of attitude and relative state with those measurements. The
    mission manager and the guidance decide, from the estimates only,
    where the deputy should go and where it should point. The controllers
    turn that into a force and a torque. The actuators deliver them as
    well as their limits allow. The truth model then advances by one
    step under the force and torque really produced, and the navigation
    predicts its estimates forward with what it believes was commanded.

    The simulation ends when the docking ports touch, when a collision
    avoidance maneuver has been flown, or at the maximum duration.

    Parameters
    ----------
    config : ios_mission_planner.sim.config.SimulationConfig, optional
        Scenario. Defaults to the reference mission.
    truth : ios_mission_planner.sim.truth.TruthModel, optional
        A truth model to reuse, built by build_truth with the same
        environment and spacecraft.
    progress : callable, optional
        Called with (time, phase) once per simulated minute.

    Returns
    -------
    SimulationResult

    Author: Giovanni Facchinetti, 2026
    """

    config = SimulationConfig() if config is None else config
    env, chief, deputy = config.environment, config.chief, config.deputy
    sensors_cfg, nav_cfg, ctl_cfg, guid_cfg = deputy.sensors, config.navigation, config.control, config.guidance
    dt = config.dt
    rng = np.random.default_rng(config.seed)

    radius = env.r_eq + chief.altitude_km
    n = np.sqrt(env.mu / radius**3)
    inertia = np.asarray(deputy.inertia, dtype=float)

    # illumination
    jd0 = julian_date_from_datetime(config.epoch)
    lighting_check = None
    if guid_cfg.lighting_gate:
        timeline = port_lighting_timeline(
            radius, np.deg2rad(chief.inclination_deg), np.deg2rad(chief.raan_deg),
            np.deg2rad(chief.argument_of_latitude_deg), jd0, config.max_duration + 1200.0, 10.0,
            env.mu, env.j2, env.r_eq, port_normal_lvlh=chief.port_axis,
        )
        lighting_check = timeline.favourable_for

    manager = MissionManager(
        guid_cfg, ctl_cfg, n, dt, deputy.thrusters.max_force / deputy.mass,
        chief.port_position, chief.port_axis, deputy.port_position_body, deputy.port_axis_body,
        deputy.roll_axis_body, start_hold=config.initial.start_hold, lighting_check=lighting_check,
    )
    axis, lateral_2 = manager.axis, manager.lateral_2

    # the true geometry of the chief differs from the nominal one, known on
    # board, by its attitude error; the true sensor axes differ from the
    # nominal ones by the mounting error
    chief_rotation = quaternion_to_rotation_matrix(rotation_vector_to_quaternion(chief.attitude_error)).T
    true_port_position = chief_rotation @ np.asarray(chief.port_position, dtype=float)
    true_port_axis = chief_rotation @ np.asarray(chief.port_axis, dtype=float)
    true_target_position = chief_rotation @ np.asarray(chief.target_position, dtype=float)
    true_basis = tuple(
        chief_rotation @ direction for direction in (manager.axis, manager.lateral_1, manager.lateral_2)
    )
    sensor_rotation = quaternion_to_rotation_matrix(
        rotation_vector_to_quaternion(sensors_cfg.relative_misalignment)
    )

    # ------------------------------------------------------------------
    # initial true state
    # ------------------------------------------------------------------
    chief_state = _initial_chief_state(config)
    lvlh, lvlh_rate = _lvlh_frame(chief_state)

    initial = config.initial
    relative = np.concatenate([
        manager.hold_positions[initial.start_hold] + initial.position_offset, initial.velocity_offset,
    ])
    deputy_state = lvlh_to_inertial(chief_state, relative * 1e-3)

    def boresight_direction(position):
        if manager.pointing_mode == "port":
            return -axis
        line = np.asarray(chief.target_position) - position
        return line / np.sqrt(line @ line)

    q_command = pointing_quaternion(
        lvlh.T, boresight_direction(relative[:3]), lateral_2, deputy.port_axis_body, deputy.roll_axis_body,
    )
    quaternion = quaternion_multiply(q_command, rotation_vector_to_quaternion(initial.attitude_error))
    body_rate = quaternion_to_rotation_matrix(quaternion) @ (lvlh_rate * lvlh[2]) + initial.body_rate

    if truth is None:
        truth = build_truth(config)
    truth.reset(chief_state, deputy_state, deputy.mass, quaternion, body_rate,
                wheel_momentum=initial.wheel_momentum)

    # ------------------------------------------------------------------
    # sensors, actuators, navigation
    # ------------------------------------------------------------------
    gyro = Gyro(sensors_cfg.gyro_noise_std, sensors_cfg.gyro_bias_walk_std,
                bias=sensors_cfg.gyro_bias_std * rng.standard_normal(3))
    star_tracker = StarTracker(sensors_cfg.star_tracker_noise_std)
    relative_sensor = RelativeSensor(
        position_body=np.asarray(sensors_cfg.relative_position_body, dtype=float),
        boresight_body=np.asarray(sensors_cfg.relative_boresight_body, dtype=float),
        fov_half_angle=np.deg2rad(sensors_cfg.relative_fov_half_angle_deg),
        max_range=sensors_cfg.relative_max_range,
        range_noise_floor=sensors_cfg.relative_range_noise_floor,
        range_noise_fraction=sensors_cfg.relative_range_noise_fraction,
        angle_noise_std=sensors_cfg.relative_angle_noise_std,
    )
    thr = deputy.thrusters
    thrusters = ThrusterSet(
        thr.max_force, thr.isp, thr.min_impulse_bit, np.asarray(thr.scale_error, dtype=float),
        np.asarray(thr.misalignment, dtype=float), thr.pulse_noise_std,
        np.asarray(thr.center_of_mass_offset, dtype=float),
    )
    wheels = ReactionWheels(deputy.wheels.max_torque, deputy.wheels.max_momentum)
    momentum_manager = None
    if deputy.wheels.desaturation_enabled:
        momentum_manager = MomentumManager(
            deputy.wheels.desaturation_start * deputy.wheels.max_momentum,
            deputy.wheels.desaturation_stop * deputy.wheels.max_momentum,
            deputy.wheels.desaturation_torque, deputy.wheels.desaturation_arm, thr.isp,
        )

    navigation = RelativeNavigationFilter(
        n, dt,
        relative + np.concatenate([initial.navigation_position_error, initial.navigation_velocity_error]),
        np.diag([nav_cfg.initial_position_std**2] * 3 + [nav_cfg.initial_velocity_std**2] * 3),
        nav_cfg.accel_noise_std, nav_cfg.thrust_noise_fraction, orbit_radius=radius * 1e3,
    )
    attitude_filter = MEKFState(
        q=star_tracker.measure(quaternion, rng), bias=np.zeros(3),
        covariance=np.diag([nav_cfg.initial_attitude_std**2] * 3 + [nav_cfg.initial_bias_std**2] * 3),
    )
    star_tracker_covariance = np.eye(3) * sensors_cfg.star_tracker_noise_std**2
    kp, kd = attitude_gains(inertia, ctl_cfg.attitude_natural_frequency, ctl_cfg.attitude_damping_ratio)
    max_slew_rate = np.deg2rad(ctl_cfg.max_slew_rate_deg_s)

    # ------------------------------------------------------------------
    # logs
    # ------------------------------------------------------------------
    n_max = int(np.ceil(config.max_duration / dt)) + 2
    log = {
        "t": np.zeros(n_max), "phase": np.zeros(n_max, dtype=int),
        "relative_state": np.zeros((n_max, 6)), "relative_estimate": np.zeros((n_max, 6)),
        "relative_std": np.zeros((n_max, 6)), "measurement_valid": np.zeros(n_max, dtype=bool),
        "quaternion": np.zeros((n_max, 4)), "quaternion_command": np.zeros((n_max, 4)),
        "quaternion_estimate": np.zeros((n_max, 4)), "body_to_lvlh": np.zeros((n_max, 3, 3)),
        "body_rate": np.zeros((n_max, 3)),
        "gyro_bias": np.zeros((n_max, 3)), "gyro_bias_estimate": np.zeros((n_max, 3)),
        "attitude_error": np.zeros(n_max), "attitude_knowledge_error": np.zeros(n_max),
        "wheel_momentum": np.zeros((n_max, 3)), "wheel_torque": np.zeros((n_max, 3)),
        "mass": np.zeros(n_max), "acceleration_command": np.zeros((n_max, 3)),
        "thrust": np.zeros((n_max, 3)), "delta_v": np.zeros(n_max),
        "port_axial": np.zeros(n_max), "port_lateral": np.zeros((n_max, 2)),
        "port_closing_speed": np.zeros(n_max), "port_lateral_speed": np.zeros(n_max),
        "port_axis_misalignment": np.zeros(n_max), "port_roll_misalignment": np.zeros(n_max),
        "port_angular_rate": np.zeros(n_max),
        "sun_lvlh": np.zeros((n_max, 3)), "eclipse": np.zeros(n_max, dtype=bool),
    }

    mass_estimate = deputy.mass
    applied_acceleration = np.zeros(3)
    gyro_measurement = gyro.measure(truth.body_rate, dt, rng)
    last_measurement_time = 0.0
    relative_steps = max(1, int(round(sensors_cfg.relative_period / dt)))
    previous_command = q_command
    command_rate_body = quaternion_to_rotation_matrix(q_command) @ (lvlh_rate * lvlh[2])
    delta_v = 0.0
    sun = sun_direction(jd0)
    previous_port = None
    outcome, contact_state, contact = TIMEOUT, None, None

    failures = config.failures
    stuck_force = np.asarray(failures.stuck_thruster_force, dtype=float)

    def active(start, duration, time):
        return start is not None and start <= time < start + duration

    k = 0
    while True:
        t = k * dt

        # ------------------------------ truth
        chief_state = truth.chief_state
        deputy_state = truth.deputy_state
        quaternion = truth.quaternion
        body_rate = truth.body_rate
        wheel_momentum = truth.wheel_momentum
        mass = truth.mass

        lvlh, lvlh_rate = _lvlh_frame(chief_state)
        difference = (deputy_state - chief_state) * 1e3
        position = lvlh @ difference[:3]
        velocity = lvlh @ difference[3:]
        # velocity seen from the rotating frame: remove omega x rho, omega along z
        velocity[0] += lvlh_rate * position[1]
        velocity[1] -= lvlh_rate * position[0]
        relative = np.concatenate([position, velocity])
        inertial_to_body = quaternion_to_rotation_matrix(quaternion)
        body_to_lvlh = lvlh @ inertial_to_body.T
        rate_relative = body_rate - inertial_to_body @ (lvlh_rate * lvlh[2])

        ports = port_state(
            relative, body_to_lvlh, rate_relative, true_port_position, true_port_axis,
            deputy.port_position_body, deputy.port_axis_body, deputy.roll_axis_body, basis=true_basis,
        )

        # ------------------------------ contact
        if (previous_port is not None and ports.axial_distance <= 0.0 < previous_port.axial_distance
                and ports.lateral_misalignment < _CONTACT_RADIUS):
            fraction = previous_port.axial_distance / (previous_port.axial_distance - ports.axial_distance)
            contact_state = _interpolate_port(previous_port, ports, fraction)
            contact = assess_contact(contact_state, config.docking_limits)
            if manager.phase != Phase.FINAL_APPROACH:
                outcome = COLLISION
            else:
                outcome = DOCKED if contact.success else CONTACT_OUT_OF_LIMITS
            manager.notify_contact(t)
        previous_port = ports

        # ------------------------------ sensors and navigation update
        attitude_filter = mekf_update(attitude_filter, star_tracker.measure(quaternion, rng),
                                      star_tracker_covariance)
        estimated_body_to_lvlh = lvlh @ quaternion_to_rotation_matrix(attitude_filter.q).T

        valid = False
        outage = active(failures.sensor_outage_start, failures.sensor_outage_duration, t)
        if k % relative_steps == 0 and not outage:
            target_body = body_to_lvlh.T @ (true_target_position - relative[:3])
            measurement = relative_sensor.measure(
                target_body, rng, misalignment=sensor_rotation,
            )
            if measurement is not None:
                measured_position, covariance = relative_position_from_measurement(
                    measurement, relative_sensor.position_body, estimated_body_to_lvlh,
                    chief.target_position, attitude_std=sensors_cfg.star_tracker_noise_std,
                )
                valid = navigation.update(measured_position, covariance, gate=nav_cfg.innovation_gate)
                if valid:
                    last_measurement_time = t

        estimate = navigation.state
        estimate_std = np.sqrt(np.diag(navigation.covariance))

        # ------------------------------ guidance and control
        attitude_error_estimate = quaternion_to_rotation_vector(quaternion_error(attitude_filter.q, previous_command))
        acceleration_command = manager.update(
            t, estimate, estimate_std[:3], t - last_measurement_time,
            np.sqrt(attitude_error_estimate @ attitude_error_estimate), applied_acceleration,
        )

        q_command = pointing_quaternion(
            lvlh.T, boresight_direction(estimate[:3]), lateral_2, deputy.port_axis_body,
            deputy.roll_axis_body, previous=previous_command,
        )
        if k > 0:
            new_rate = command_rate(previous_command, q_command, dt)
            # a change of pointing mode is a step in the command, not a rate
            if np.sqrt(new_rate @ new_rate) < 5.0 * max_slew_rate:
                command_rate_body = new_rate
        previous_command = q_command

        torque_command = quaternion_feedback_torque(
            attitude_filter.q, gyro_measurement - attitude_filter.bias, q_command, command_rate_body,
            inertia, wheel_momentum, kp, kd, max_slew_rate,
        )

        force_command = mass_estimate * (estimated_body_to_lvlh.T @ acceleration_command)
        largest = np.max(np.abs(force_command))
        if largest > thrusters.max_force:
            force_command = force_command * (thrusters.max_force / largest)

        # ------------------------------ actuators
        if manager.finished:
            intended, actual, mass_flow, disturbance = np.zeros(3), np.zeros(3), 0.0, np.zeros(3)
            wheel_torque = np.zeros(3)
        else:
            intended, actual, mass_flow, disturbance = thrusters.command(force_command, dt, rng)
            if active(failures.stuck_thruster_start, failures.stuck_thruster_duration, t):
                actual = actual + stuck_force
                mass_flow += np.sum(np.abs(stuck_force)) / (G0 * thr.isp)
            unloading = np.zeros(3)
            if momentum_manager is not None:
                unloading, unloading_flow = momentum_manager.command(wheel_momentum)
                disturbance = disturbance + unloading
                mass_flow += unloading_flow
            # the on-board software commands the unloading couples, so it
            # knows their torque: the wheels are asked to cancel it at the
            # same instant, and the body does not feel it
            if deputy.wheels.desaturation_feedforward:
                torque_command = torque_command - unloading
            wheel_torque = wheels.command(torque_command, wheel_momentum, dt)

        # ------------------------------ log
        log["t"][k] = t
        log["phase"][k] = int(manager.phase)
        log["relative_state"][k] = relative
        log["relative_estimate"][k] = estimate
        log["relative_std"][k] = estimate_std
        log["measurement_valid"][k] = valid
        log["quaternion"][k] = quaternion
        log["quaternion_command"][k] = q_command
        log["quaternion_estimate"][k] = attitude_filter.q
        log["body_to_lvlh"][k] = body_to_lvlh
        log["body_rate"][k] = body_rate
        log["gyro_bias"][k] = gyro.bias
        log["gyro_bias_estimate"][k] = attitude_filter.bias
        true_error = quaternion_to_rotation_vector(quaternion_error(quaternion, q_command))
        log["attitude_error"][k] = np.sqrt(true_error @ true_error)
        knowledge_error = quaternion_to_rotation_vector(quaternion_error(attitude_filter.q, quaternion))
        log["attitude_knowledge_error"][k] = np.sqrt(knowledge_error @ knowledge_error)
        log["wheel_momentum"][k] = wheel_momentum
        log["wheel_torque"][k] = wheel_torque
        log["mass"][k] = mass
        log["acceleration_command"][k] = acceleration_command
        log["thrust"][k] = actual
        log["delta_v"][k] = delta_v
        log["port_axial"][k] = ports.axial_distance
        log["port_lateral"][k] = ports.lateral_offset
        log["port_closing_speed"][k] = ports.closing_speed
        log["port_lateral_speed"][k] = ports.lateral_speed
        log["port_axis_misalignment"][k] = ports.axis_misalignment
        log["port_roll_misalignment"][k] = ports.roll_misalignment
        log["port_angular_rate"][k] = ports.angular_rate
        if k % 60 == 0:
            sun = sun_direction(jd0 + t / 86400.0)
            if progress is not None:
                progress(t, manager.phase)
        log["sun_lvlh"][k] = lvlh @ sun
        log["eclipse"][k] = in_eclipse(chief_state[:3], sun, env.r_eq)

        if manager.finished:
            if manager.phase == Phase.ABORT:
                outcome = ABORTED
            break
        if t >= config.max_duration:
            break

        # ------------------------------ propagate truth and estimates
        truth.set_controls(actual, mass_flow, wheel_torque, disturbance)
        truth.step(dt)

        delta_v += np.sum(np.abs(actual)) / mass * dt
        mass_estimate -= np.sum(np.abs(intended)) / (G0 * thr.isp) * dt

        applied_acceleration = estimated_body_to_lvlh @ intended / mass_estimate
        navigation.predict(applied_acceleration)

        gyro_measurement = gyro.measure(0.5 * (body_rate + truth.body_rate), dt, rng)
        attitude_filter = mekf_predict(attitude_filter, gyro_measurement, dt,
                                       nav_cfg.gyro_noise_std, nav_cfg.gyro_bias_walk_std)

        k += 1

    result = {name: values[:k + 1] for name, values in log.items()}

    return SimulationResult(
        **result, events=[(time, PHASE_LABELS[phase], note) for time, phase, note in manager.events],
        outcome=outcome, contact_state=contact_state, contact=contact,
        abort_reason=manager.abort_reason, config=config,
        hold_positions=[p.copy() for p in manager.hold_positions],
        contact_position=manager.contact_position.copy(),
    )


def build_truth(config):
    """
    Truth model for a scenario. Building it once and passing it to
    several calls of simulate avoids repeating the setup.

    Author: Giovanni Facchinetti, 2026
    """

    env, chief, deputy = config.environment, config.chief, config.deputy

    return TruthModel(
        env.mu, env.j2, env.r_eq, env.omega_body,
        env.atmosphere_density, env.r_eq + env.atmosphere_altitude_km, env.atmosphere_scale_height_km,
        chief.ballistic_coefficient, deputy.ballistic_coefficient, deputy.inertia,
        gravity_gradient=env.gravity_gradient,
    )


def _interpolate_port(before, after, fraction):
    """Port state at a fraction of the way between two successive steps."""

    def mix(a, b):
        return a + fraction * (b - a)

    return PortState(
        axial_distance=0.0,
        lateral_offset=mix(before.lateral_offset, after.lateral_offset),
        closing_speed=mix(before.closing_speed, after.closing_speed),
        lateral_velocity=mix(before.lateral_velocity, after.lateral_velocity),
        axis_misalignment=mix(before.axis_misalignment, after.axis_misalignment),
        roll_misalignment=mix(before.roll_misalignment, after.roll_misalignment),
        angular_rate=mix(before.angular_rate, after.angular_rate),
    )
