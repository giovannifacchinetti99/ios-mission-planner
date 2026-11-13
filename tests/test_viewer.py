import json
import re

import numpy as np
from conftest import make_near_hold_config

from ios_mission_planner.sim.simulator import simulate
from ios_mission_planner.viz.viewer import (
    LVLH_TO_SCENE,
    build_viewer_data,
    export_viewer,
    rotation_to_scene_quaternion,
)


def _rotate(q, v):
    """Rotate a vector by a quaternion (x, y, z, w), as a 3D graphics library does."""

    axis, w = q[:3], q[3]
    return v + 2.0 * np.cross(axis, np.cross(axis, v) + w * v)


def test_scene_quaternion_rotates_like_the_matrix():
    rng = np.random.default_rng(0)
    vector = np.array([0.3, -1.2, 0.7])

    # rotations of 170 degrees about each axis and about a random one go
    # through all four branches of the conversion
    axes = [np.eye(3)[k] for k in range(3)] + [rng.standard_normal(3) for _ in range(20)]
    for axis in axes:
        axis = axis / np.linalg.norm(axis)
        for angle in (0.2, np.deg2rad(170.0)):
            k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
            rotation = np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * k @ k
            q = rotation_to_scene_quaternion(rotation)

            assert np.isclose(np.linalg.norm(q), 1.0)
            assert np.allclose(_rotate(q, vector), rotation @ vector, atol=1e-12)


def test_scene_axes_are_a_proper_rotation_with_flight_direction_to_the_right():
    assert np.isclose(np.linalg.det(LVLH_TO_SCENE), 1.0)
    assert np.allclose(LVLH_TO_SCENE @ LVLH_TO_SCENE.T, np.eye(3))
    assert np.allclose(LVLH_TO_SCENE @ [0.0, 1.0, 0.0], [1.0, 0.0, 0.0])   # along-track to the right
    assert np.allclose(LVLH_TO_SCENE @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])   # radial up


def test_viewer_data_is_consistent_and_serializable():
    result = simulate(make_near_hold_config())
    data = build_viewer_data(result, title="test", far_step=5, near_range=10.0)

    count = len(data["t"])
    assert len(data["p"]) == 3 * count and len(data["q"]) == 4 * count and len(data["f"]) == 3 * count
    assert len(data["ph"]) == len(data["e"]) == len(data["ax"]) == count
    assert data["t"][0] == 0.0 and data["t"][-1] == result.duration
    json.dumps(data)

    # coarse far from the port, every step near it
    steps = np.diff(data["t"])
    assert steps[0] == 5.0 and steps[-2] == 1.0
    assert count < len(result.t)

    # the docked deputy has its port axis, body x, along the direction of flight
    q = np.array(data["q"][-4:])
    assert np.allclose(_rotate(q, np.array([1.0, 0.0, 0.0])), [1.0, 0.0, 0.0], atol=1e-3)
    # along-track position of the center of mass, within the last step of travel
    assert abs(data["p"][-2] - (-1.6)) < 0.1

    meta = data["meta"]
    assert meta["outcome"] == "docked" and len(meta["contact"]) == 6
    assert [phase["label"] for phase in meta["phases"]] == ["hold at S3", "final approach", "docked"]
    assert len(meta["hold_points"]) == 3


def test_export_writes_a_single_self_contained_page(tmp_path):
    result = simulate(make_near_hold_config())
    path = export_viewer(result, tmp_path / "viewer.html", title="A docking")

    html = path.read_text(encoding="utf-8")
    assert "__DATA__" not in html and "__TITLE__" not in html
    assert "<title>A docking</title>" in html

    payload = re.search(r'<script id="data" type="application/json">(.*?)</script>', html, re.S).group(1)
    data = json.loads(payload)
    assert data["meta"]["title"] == "A docking"
    assert len(data["t"]) > 100
