import numpy as np

from ios_mission_planner.safety.corridor import ApproachCorridor, check_corridor


def test_direction_is_normalized():
    corridor = ApproachCorridor(direction=[0.0, -5.0, 0.0], half_angle=np.deg2rad(30.0))
    assert np.isclose(np.linalg.norm(corridor.direction), 1.0)


def test_angle_zero_on_axis():
    corridor = ApproachCorridor(direction=[0.0, -1.0, 0.0], half_angle=np.deg2rad(30.0))
    assert np.isclose(corridor.angle([0.0, -5.0, 0.0]), 0.0, atol=1e-9)


def test_angle_ninety_degrees_perpendicular():
    corridor = ApproachCorridor(direction=[0.0, -1.0, 0.0], half_angle=np.deg2rad(30.0))
    assert np.isclose(corridor.angle([5.0, 0.0, 0.0]), np.pi / 2, atol=1e-9)


def test_chief_itself_is_compliant():
    corridor = ApproachCorridor(direction=[0.0, -1.0, 0.0], half_angle=np.deg2rad(5.0))
    assert not corridor.is_violated([0.0, 0.0, 0.0])


def test_check_corridor_margin_sign_convention():
    corridor = ApproachCorridor(direction=[0.0, -1.0, 0.0], half_angle=np.deg2rad(30.0))

    inside = np.array([[0.0, -10.0, 0.0], [1.0, -10.0, 0.0]])
    violated, margin, mask = check_corridor(inside, corridor)
    assert not violated
    assert margin < 0.0

    outside = np.array([[0.0, -10.0, 0.0], [10.0, -1.0, 0.0]])
    violated, margin, mask = check_corridor(outside, corridor)
    assert violated
    assert margin > 0.0
