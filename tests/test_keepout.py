import numpy as np

from ios_mission_planner.safety.keepout import KeepOutEllipsoid, check_trajectory


def test_sphere_special_case():
    zone = KeepOutEllipsoid(a=20.0, b=20.0, c=20.0)

    assert zone.is_violated([10.0, 0.0, 0.0])
    assert not zone.is_violated([21.0, 0.0, 0.0])


def test_anisotropic_ellipsoid():
    zone = KeepOutEllipsoid(a=20.0, b=100.0, c=20.0)

    # well inside the along-track semi-axis, but outside the radial one
    assert not zone.is_violated([25.0, 0.0, 0.0])
    assert zone.is_violated([0.0, 50.0, 0.0])


def test_check_trajectory_margin_sign_convention():
    zone = KeepOutEllipsoid(a=20.0, b=20.0, c=20.0)

    positions_safe = np.array([[100.0, 0.0, 0.0], [50.0, 0.0, 0.0]])
    violated, margin, mask = check_trajectory(positions_safe, zone)
    assert not violated
    assert margin < 0.0  # negative margin means it stayed clear
    assert not mask.any()

    positions_unsafe = np.array([[100.0, 0.0, 0.0], [10.0, 0.0, 0.0]])
    violated, margin, mask = check_trajectory(positions_unsafe, zone)
    assert violated
    assert margin > 0.0  # positive margin means it entered the zone
    assert mask[1] and not mask[0]
