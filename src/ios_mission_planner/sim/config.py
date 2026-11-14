"""
Configuration of the closed-loop docking simulation.

Every number that defines a scenario lives here, in plain dataclasses
with defaults that describe the reference mission of the book: a 160 kg
servicer docking to a cooperative target on a 500 km orbit, approaching
from behind along the V-bar. The values are representative of a small
servicer, not the data sheet of a specific spacecraft; a real study
replaces them with its own.

Units: metres, metres per second, seconds, radians, kilograms and newtons
unless the name of the field says otherwise.
"""

from dataclasses import dataclass, field
from datetime import datetime

import numpy as np

from ios_mission_planner.constants import J2_EARTH, MU_EARTH, OMEGA_EARTH, R_EARTH


def _array(*values):
    return field(default_factory=lambda: np.array(values, dtype=float))


@dataclass
class EnvironmentConfig:
    """Central body, perturbations and atmosphere of the truth model."""

    mu: float = MU_EARTH
    j2: float = J2_EARTH
    r_eq: float = R_EARTH
    omega_body: float = OMEGA_EARTH
    # exponential atmosphere, the same illustrative band used in the drag
    # chapter: density at the reference altitude and scale height
    atmosphere_density: float = 1.0e-12       # [kg/m^3]
    atmosphere_altitude_km: float = 450.0
    atmosphere_scale_height_km: float = 60.0
    gravity_gradient: bool = True


@dataclass
class ChiefConfig:
    """
    The target. It is cooperative: it holds its attitude fixed in its own
    LVLH frame and carries a docking port and a visual target next to it.
    """

    altitude_km: float = 500.0
    inclination_deg: float = 51.6
    raan_deg: float = 40.0
    argument_of_latitude_deg: float = 0.0
    ballistic_coefficient: float = 0.010      # Cd A / m [m^2/kg]
    port_position: np.ndarray = _array(0.0, -1.0, 0.0)      # LVLH [m]
    port_axis: np.ndarray = _array(0.0, -1.0, 0.0)          # outward normal, LVLH
    target_position: np.ndarray = _array(0.0, -1.0, 0.25)   # visual target, LVLH [m]
    # true attitude of the chief with respect to its nominal, LVLH-fixed
    # one, as a rotation vector [rad]; unknown to the deputy
    attitude_error: np.ndarray = _array(0.0, 0.0, 0.0)


@dataclass
class SensorConfig:
    gyro_noise_std: float = 1.0e-5            # [rad/s] per sample
    gyro_bias_std: float = 5.0e-5             # initial bias, per axis [rad/s]
    gyro_bias_walk_std: float = 1.0e-8        # [rad/s/sqrt(s)]
    star_tracker_noise_std: float = 1.0e-4    # per axis [rad]
    relative_position_body: np.ndarray = _array(0.3, 0.0, 0.25)
    relative_boresight_body: np.ndarray = _array(1.0, 0.0, 0.0)
    relative_fov_half_angle_deg: float = 20.0
    relative_max_range: float = 10000.0
    relative_range_noise_floor: float = 0.01
    relative_range_noise_fraction: float = 0.005
    relative_angle_noise_std: float = 5.0e-4  # [rad]
    relative_period: float = 1.0              # time between measurements [s]
    # mounting error of the relative sensor, as a rotation vector [rad];
    # unknown to the on-board software
    relative_misalignment: np.ndarray = _array(0.0, 0.0, 0.0)


@dataclass
class ThrusterConfig:
    max_force: float = 2.0                    # per direction of each body axis [N]
    isp: float = 220.0
    min_impulse_bit: float = 0.04             # [N s]
    scale_error: np.ndarray = _array(0.0, 0.0, 0.0)
    misalignment: np.ndarray = _array(0.0, 0.0, 0.0)        # rotation vector [rad]
    pulse_noise_std: float = 0.01
    center_of_mass_offset: np.ndarray = _array(0.0, 0.0, 0.0)


@dataclass
class WheelConfig:
    max_torque: float = 0.02                  # [N m]
    max_momentum: float = 0.5                 # [N m s]
    # momentum management: thruster couples unload a wheel when it gets
    # close to its limit. The torque of the couple must exceed the largest
    # disturbance torque that the thrusters can produce (their thrust times
    # the largest credible offset of the center of mass: 2 N and 8 mm give
    # 0.016 N m) and stay below max_torque, or the wheel cannot oppose it
    desaturation_enabled: bool = True
    desaturation_start: float = 0.6           # fraction of max_momentum
    desaturation_stop: float = 0.2
    desaturation_torque: float = 0.016        # of the thruster couple [N m]
    # the wheels are commanded to cancel the torque of the couple, which the
    # on-board software knows; without this the attitude feedback has to
    # find it out from the attitude error that the couple causes
    desaturation_feedforward: bool = True
    desaturation_arm: float = 0.5             # between the two thrusters [m]


@dataclass
class DeputyConfig:
    """The servicer."""

    mass: float = 160.0                       # at the start of the simulation [kg]
    inertia: np.ndarray = _array(22.0, 25.0, 28.0)          # principal moments [kg m^2]
    ballistic_coefficient: float = 0.020
    port_position_body: np.ndarray = _array(0.6, 0.0, 0.0)
    port_axis_body: np.ndarray = _array(1.0, 0.0, 0.0)
    roll_axis_body: np.ndarray = _array(0.0, 0.0, 1.0)      # kept along the orbit normal
    sensors: SensorConfig = field(default_factory=SensorConfig)
    thrusters: ThrusterConfig = field(default_factory=ThrusterConfig)
    wheels: WheelConfig = field(default_factory=WheelConfig)


@dataclass
class NavigationConfig:
    accel_noise_std: float = 2.0e-5           # unmodelled acceleration [m/s^2/sqrt(Hz)]
    thrust_noise_fraction: float = 0.03
    initial_position_std: float = 30.0        # of the first estimate, per axis [m]
    initial_velocity_std: float = 0.05        # [m/s]
    innovation_gate: float = 25.0             # on the normalized innovation squared
    gyro_noise_std: float = 1.0e-5            # as assumed by the attitude filter
    gyro_bias_walk_std: float = 1.0e-7
    initial_attitude_std: float = 1.0e-3      # [rad]
    initial_bias_std: float = 1.0e-4          # [rad/s]


@dataclass
class ControlConfig:
    translation_period: float = 4.0           # period of the translation control [s]
    # hold controller: tolerances grow with the distance from the chief,
    # because far away the navigation is too noisy to hold tightly
    hold_position_tolerance_fraction: float = 0.01
    hold_position_tolerance_min: float = 0.3
    hold_acceleration_scale: float = 5.0e-4
    # model predictive controller of the approach
    mpc_horizon: int = 30
    mpc_position_tolerance: float = 0.5
    mpc_velocity_tolerance: float = 0.03
    mpc_acceleration_scale: float = 4.0e-3
    # same controller with tighter weights for the last metres
    final_position_tolerance: float = 0.05
    final_velocity_tolerance: float = 0.01
    final_acceleration_scale: float = 4.0e-3
    # attitude
    attitude_natural_frequency: float = 0.08  # [rad/s]
    attitude_damping_ratio: float = 0.9
    max_slew_rate_deg_s: float = 0.5


@dataclass
class GuidanceConfig:
    """
    The approach profile: a sequence of hold points on the approach axis,
    at given distances between the two docking ports, and how the deputy
    moves from one to the next.
    """

    hold_distances: tuple = (5000.0, 500.0, 20.0)           # port to port [m]
    hold_durations: tuple = (300.0, 300.0, 120.0)           # minimum stay [s]
    hold_position_tolerance: tuple = (60.0, 6.0, 0.15)      # to leave a hold [m]
    hold_velocity_tolerance: tuple = (0.10, 0.02, 0.01)     # [m/s]
    # far-range transfer between the first two hold points
    transfer_orbit_fraction: float = 0.45
    correction_fractions: tuple = (0.3, 0.6, 0.85)
    correction_threshold: float = 0.003       # smallest correction worth a burn [m/s]
    # forced approach between the last two hold points
    approach_v_max: float = 0.4
    approach_v_min: float = 0.05
    approach_time_constant: float = 400.0
    approach_acceleration: float = 0.002
    # final translation to contact
    docking_speed: float = 0.075
    docking_overtravel: float = 1.0           # the reference continues past the port [m]
    # attitude: point at the chief when far, align the ports when near
    port_alignment_range: float = 60.0
    # approach corridor, a cone around the approach axis with its vertex
    # at the port, of the given half-angle plus a radius at the vertex
    corridor_half_angle_deg: float = 10.0
    corridor_radius: float = 0.3
    closing_speed_floor: float = 0.12         # closing speed allowed at the port [m/s]
    closing_speed_slope: float = 0.004        # plus this much per metre of distance [1/s]
    # go/no-go before the final approach
    lighting_gate: bool = False
    lighting_margin: float = 60.0             # beyond the predicted duration [s]
    # collision avoidance maneuver and the conditions that trigger it
    abort_enabled: bool = True
    abort_corridor_half_angle_deg: float = 15.0
    abort_corridor_radius: float = 0.5
    abort_closing_speed_factor: float = 1.5
    abort_measurement_outage: float = 30.0    # without relative measurements [s]
    abort_position_std_fraction: float = 0.02
    abort_position_std_min: float = 0.10
    abort_pointing_error_deg: float = 8.0
    abort_delta_v: float = 0.5                # along the outward port axis [m/s]
    abort_coast_duration: float = 900.0


@dataclass
class DockingLimits:
    """
    Conditions at first contact that the docking mechanism accepts. The
    defaults are representative of the initial contact conditions of the
    International Docking System Standard, and are scenario parameters:
    the limits of a real mechanism come from its interface definition.
    """

    lateral_misalignment: float = 0.10        # [m]
    axial_speed_min: float = 0.05             # [m/s]
    axial_speed_max: float = 0.10
    lateral_speed: float = 0.04               # [m/s]
    angular_misalignment_deg: float = 4.0     # of the port axes
    roll_misalignment_deg: float = 4.0        # about the port axis
    angular_rate_deg_s: float = 0.20


@dataclass
class InitialConditions:
    """
    Where the simulation starts. start_hold selects the hold point the
    deputy is at (0 is the farthest); the offsets are true deviations
    from it, unknown to the on-board software, which starts from its own
    estimate.
    """

    start_hold: int = 0
    position_offset: np.ndarray = _array(20.0, -30.0, -15.0)
    velocity_offset: np.ndarray = _array(0.01, -0.02, 0.01)
    attitude_error: np.ndarray = _array(0.03, -0.04, 0.05)  # rotation vector [rad]
    body_rate: np.ndarray = _array(0.0, 0.0, 0.0)           # relative to the command [rad/s]
    wheel_momentum: np.ndarray = _array(0.0, 0.0, 0.0)      # stored in the wheels [N m s]
    navigation_position_error: np.ndarray = _array(25.0, -20.0, 15.0)
    navigation_velocity_error: np.ndarray = _array(0.03, 0.02, -0.02)


@dataclass
class FailureConfig:
    """
    Failures injected into the simulation, to see how the mission
    reacts. By default none.
    """

    # the relative sensor stops returning measurements
    sensor_outage_start: float = None         # [s]
    sensor_outage_duration: float = 0.0
    # a thruster stays open: a constant force along a body axis
    stuck_thruster_start: float = None        # [s]
    stuck_thruster_duration: float = 0.0
    stuck_thruster_force: np.ndarray = _array(0.0, 0.0, 0.0)    # body frame [N]


@dataclass
class SimulationConfig:
    dt: float = 1.0
    max_duration: float = 12000.0
    seed: int = 0
    epoch: datetime = datetime(2026, 3, 20, 12, 0, 0)
    environment: EnvironmentConfig = field(default_factory=EnvironmentConfig)
    chief: ChiefConfig = field(default_factory=ChiefConfig)
    deputy: DeputyConfig = field(default_factory=DeputyConfig)
    navigation: NavigationConfig = field(default_factory=NavigationConfig)
    control: ControlConfig = field(default_factory=ControlConfig)
    guidance: GuidanceConfig = field(default_factory=GuidanceConfig)
    docking_limits: DockingLimits = field(default_factory=DockingLimits)
    initial: InitialConditions = field(default_factory=InitialConditions)
    failures: FailureConfig = field(default_factory=FailureConfig)
