import numpy as np

from ios_mission_planner.budget.propellant import G0
from ios_mission_planner.dynamics.attitude.quaternion import quaternion_error, quaternion_to_rotation_vector
from ios_mission_planner.gnc.actuators import MomentumManager, ReactionWheels, ThrusterSet
from ios_mission_planner.gnc.sensors import Gyro, RelativeSensor, StarTracker


def _relative_sensor(**overrides):
    parameters = dict(
        position_body=np.zeros(3), boresight_body=np.array([1.0, 0.0, 0.0]),
        fov_half_angle=np.deg2rad(20.0), max_range=10000.0,
        range_noise_floor=0.01, range_noise_fraction=0.005, angle_noise_std=5e-4,
    )
    parameters.update(overrides)
    return RelativeSensor(**parameters)


def test_gyro_noise_and_bias_statistics():
    rng = np.random.default_rng(0)
    gyro = Gyro(noise_std=1e-4, bias_walk_std=0.0, bias=np.array([1e-3, -2e-3, 3e-3]))
    rate = np.array([0.01, 0.02, -0.03])

    samples = np.array([gyro.measure(rate, 1.0, rng) for _ in range(4000)])

    assert np.allclose(samples.mean(axis=0), rate + gyro.bias, atol=5e-6)
    assert np.allclose(samples.std(axis=0), 1e-4, rtol=0.05)


def test_gyro_bias_random_walk_grows_with_the_square_root_of_time():
    rng = np.random.default_rng(1)
    finals = []
    for _ in range(400):
        gyro = Gyro(noise_std=0.0, bias_walk_std=1e-5)
        for _ in range(100):
            gyro.measure(np.zeros(3), 1.0, rng)
        finals.append(gyro.bias)

    assert np.allclose(np.std(finals, axis=0), 1e-5 * np.sqrt(100.0), rtol=0.1)


def test_star_tracker_error_statistics():
    rng = np.random.default_rng(2)
    tracker = StarTracker(noise_std=1e-4)
    truth = np.array([0.5, 0.5, -0.5, 0.5])

    errors = np.array([
        quaternion_to_rotation_vector(quaternion_error(tracker.measure(truth, rng), truth))
        for _ in range(3000)
    ])

    assert np.allclose(errors.mean(axis=0), 0.0, atol=1e-5)
    assert np.allclose(errors.std(axis=0), 1e-4, rtol=0.05)


def test_relative_sensor_sees_only_inside_its_field_of_view_and_range():
    rng = np.random.default_rng(3)
    sensor = _relative_sensor()

    assert sensor.measure([100.0, 0.0, 0.0], rng) is not None
    assert sensor.measure([100.0, 30.0, 0.0], rng) is not None       # 16.7 degrees off axis
    assert sensor.measure([100.0, 40.0, 0.0], rng) is None           # 21.8 degrees off axis
    assert sensor.measure([-100.0, 0.0, 0.0], rng) is None           # behind
    assert sensor.measure([20000.0, 0.0, 0.0], rng) is None          # too far


def test_relative_sensor_noise_grows_with_distance():
    rng = np.random.default_rng(4)
    sensor = _relative_sensor()

    for distance, expected_range_std in ((1.0, 0.01), (1000.0, 5.0)):
        target = np.array([distance, 0.0, 0.0])
        measurements = [sensor.measure(target, rng) for _ in range(3000)]
        ranges = np.array([m.range for m in measurements])
        across = np.array([m.line_of_sight[1:] for m in measurements])

        assert abs(ranges.mean() - distance) < 4.0 * expected_range_std / np.sqrt(3000)
        assert np.isclose(ranges.std(), expected_range_std, rtol=0.06)
        assert np.allclose(across.std(axis=0), 5e-4, rtol=0.06)
        assert measurements[0].range_std == expected_range_std


def test_relative_sensor_mounting_error_rotates_the_measured_direction():
    sensor = _relative_sensor(range_noise_floor=0.0, range_noise_fraction=0.0, angle_noise_std=0.0)
    angle = 1e-3
    rotation = np.array([[np.cos(angle), np.sin(angle), 0.0], [-np.sin(angle), np.cos(angle), 0.0],
                         [0.0, 0.0, 1.0]])

    measurement = sensor.measure([50.0, 0.0, 0.0], np.random.default_rng(0), misalignment=rotation)

    assert np.isclose(measurement.range, 50.0)
    assert np.allclose(measurement.line_of_sight, [np.cos(angle), -np.sin(angle), 0.0])


def test_thrusters_accumulate_small_requests_into_minimum_pulses():
    thrusters = ThrusterSet(max_force=2.0, isp=220.0, min_impulse_bit=0.04)

    # a request of 0.01 N s per step cannot fire: three steps deliver nothing
    for _ in range(3):
        intended, actual, mass_flow, _ = thrusters.command([0.01, 0.0, 0.0], 1.0)
        assert np.allclose(intended, 0.0) and mass_flow == 0.0

    # the fourth brings the accumulated request to the minimum impulse bit
    intended, actual, mass_flow, _ = thrusters.command([0.01, 0.0, 0.0], 1.0)
    assert np.allclose(intended, [0.04, 0.0, 0.0])
    assert np.isclose(mass_flow, 0.04 / (G0 * 220.0))


def test_thrusters_deliver_the_requested_impulse_over_time():
    thrusters = ThrusterSet(max_force=2.0, isp=220.0, min_impulse_bit=0.04)
    rng = np.random.default_rng(5)

    requested, delivered = np.zeros(3), np.zeros(3)
    for _ in range(2000):
        force = 0.05 * rng.standard_normal(3) + np.array([0.02, -0.01, 0.0])
        intended, _, _, _ = thrusters.command(force, 1.0)
        requested += force
        delivered += intended
        assert np.all((np.abs(intended) == 0.0) | (np.abs(intended) >= 0.04 - 1e-12))

    # what was not delivered is what is still waiting in the accumulator
    assert np.all(np.abs(requested - delivered) < 0.04)


def test_thrusters_saturate_at_their_maximum_force():
    thrusters = ThrusterSet(max_force=2.0, isp=220.0, min_impulse_bit=0.04)

    intended, _, mass_flow, _ = thrusters.command([10.0, -10.0, 0.5], 1.0)

    assert np.allclose(intended, [2.0, -2.0, 0.5])
    assert np.isclose(mass_flow, 4.5 / (G0 * 220.0))

    # the excess was not stored: the next step does not keep firing
    intended, _, _, _ = thrusters.command([0.0, 0.0, 0.0], 1.0)
    assert np.allclose(intended, 0.0)


def test_thruster_errors_and_disturbance_torque():
    thrusters = ThrusterSet(
        max_force=2.0, isp=220.0, min_impulse_bit=0.04, scale_error=np.array([0.05, 0.0, 0.0]),
        misalignment=np.array([0.0, 0.0, 0.01]), center_of_mass_offset=np.array([0.0, 0.002, 0.0]),
    )

    intended, actual, _, torque = thrusters.command([1.0, 0.0, 0.0], 1.0)

    assert np.allclose(intended, [1.0, 0.0, 0.0])
    assert np.allclose(actual, [1.05, 0.0105, 0.0])
    # thrust line 2 mm to the side of the center of mass
    assert np.allclose(torque, np.cross([0.0, -0.002, 0.0], actual))


def test_reaction_wheels_limit_torque_and_momentum():
    wheels = ReactionWheels(max_torque=0.02, max_momentum=0.5)

    assert np.allclose(wheels.command([0.05, -0.05, 0.01], np.zeros(3), 1.0), [0.02, -0.02, 0.01])

    # the wheel momentum changes at minus the torque: a wheel at +0.5
    # cannot take a negative torque, but accepts a positive one
    momentum = np.array([0.5, 0.5, -0.495])
    torque = wheels.command([-0.02, 0.02, 0.02], momentum, 1.0)
    assert np.allclose(torque, [0.0, 0.02, 0.005])
    assert np.all(np.abs(momentum - torque * 1.0) <= 0.5 + 1e-12)
    assert list(wheels.saturated(momentum)) == [True, True, False]


def test_momentum_manager_unloads_with_hysteresis():
    manager = MomentumManager(start=0.3, stop=0.1, torque=0.008, arm=0.5, isp=220.0)

    torque, mass_flow = manager.command([0.2, 0.0, 0.0])
    assert np.allclose(torque, 0.0) and mass_flow == 0.0

    torque, mass_flow = manager.command([0.35, -0.4, 0.0])
    assert np.allclose(torque, [-0.008, 0.008, 0.0])
    assert np.isclose(mass_flow, 2 * 2 * (0.008 / 0.5) / (G0 * 220.0))

    # between the two thresholds the unloading continues...
    torque, _ = manager.command([0.2, -0.2, 0.0])
    assert np.allclose(torque, [-0.008, 0.008, 0.0])

    # ...and it stops below the lower one
    torque, _ = manager.command([0.05, -0.05, 0.0])
    assert np.allclose(torque, 0.0)
