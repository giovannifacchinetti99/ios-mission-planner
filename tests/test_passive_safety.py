import numpy as np

from ios_mission_planner.safety.keepout import KeepOutEllipsoid
from ios_mission_planner.safety.passive_safety import check_passive_safety

N = 0.0011


def test_outbound_coast_is_passively_safe():
    # already moving away along-track, well clear of a small zone
    state_before_burn = np.array([0.0, -500.0, 0.0, 0.0, 0.3, 0.0])
    zone = KeepOutEllipsoid(a=20.0, b=20.0, c=20.0)

    violated, margin, t, states = check_passive_safety(state_before_burn, N, zone, duration=600.0)

    assert not violated
    assert margin < 0.0
    assert states.shape[0] == len(t)


def test_inbound_coast_towards_the_zone_is_unsafe():
    # closing in along-track, straight toward a zone in its way
    state_before_burn = np.array([0.0, -50.0, 0.0, 0.0, 0.3, 0.0])
    zone = KeepOutEllipsoid(a=20.0, b=20.0, c=20.0)

    violated, margin, t, states = check_passive_safety(state_before_burn, N, zone, duration=600.0)

    assert violated
    assert margin > 0.0
