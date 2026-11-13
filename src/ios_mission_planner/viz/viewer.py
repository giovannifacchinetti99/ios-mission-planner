import json
from pathlib import Path

import numpy as np

from ios_mission_planner.gnc.mission_manager import PHASE_LABELS, Phase

TEMPLATE = Path(__file__).resolve().parent / "templates" / "viewer.html"

# LVLH (radial, along-track, cross-track) to the axes of the 3D scene, in
# which the direction of flight is to the right and the radial direction
# is up: (X, Y, Z) = (y, x, -z). The determinant is +1, so rotations keep
# their handedness.
LVLH_TO_SCENE = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])

PHASE_COLORS = {
    Phase.HOLD_FAR: "#8a94a0", Phase.TRANSFER: "#4c9be8", Phase.HOLD_MID: "#8a94a0",
    Phase.APPROACH: "#f0a030", Phase.HOLD_NEAR: "#8a94a0", Phase.FINAL_APPROACH: "#e8504c",
    Phase.DOCKED: "#4cc070", Phase.ABORT: "#b07ce8",
}


def rotation_to_scene_quaternion(rotation):
    """
    Quaternion (x, y, z, w), in the convention of 3D graphics libraries,
    of a rotation matrix that maps the components of a vector in the
    rotated object to its components in the scene: v_scene = R v_object.

    This is the active convention, the transpose of the inertial-to-body
    matrices used by the attitude modules of this package, which is why
    the conversion has its own function instead of reusing theirs.

    Parameters
    ----------
    rotation : array_like, shape (3, 3)

    Returns
    -------
    numpy.ndarray, shape (4,)

    Author: Giovanni Facchinetti, 2026
    Reference: Shepperd, "Quaternion from Rotation Matrix," Journal of
    Guidance and Control, 1(3), 1978
    """

    r = np.asarray(rotation, dtype=float)
    trace = r[0, 0] + r[1, 1] + r[2, 2]

    if trace > 0.0:
        s = 2.0 * np.sqrt(trace + 1.0)
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = 2.0 * np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = 2.0 * np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s

    q = np.array([x, y, z, w])
    return q / np.sqrt(q @ q)


def _sample_indices(result, far_step, near_range):
    """Every step when the ports are close, one step in far_step when they are far."""

    near = result.port_axial < near_range
    keep = near | (np.arange(len(result.t)) % far_step == 0)
    keep[0] = keep[-1] = True

    return np.flatnonzero(keep)


def build_viewer_data(result, title=None, far_step=5, near_range=100.0):
    """
    Reduce a simulation result to the data that the 3D viewer plays back:
    the true relative position and attitude of the deputy, its thrust,
    the navigation estimate, the direction of the Sun, and the numbers
    shown on screen, all sampled at the rate of the simulation near the
    chief and more coarsely far from it.

    Parameters
    ----------
    result : ios_mission_planner.sim.simulator.SimulationResult
    title : str, optional
    far_step : int
        Far from the chief, one sample is kept every far_step steps.
    near_range : float
        Distance between the ports below which every step is kept [m].

    Returns
    -------
    dict
        Plain lists and numbers, ready to be written as JSON.

    Author: Giovanni Facchinetti, 2026
    """

    config = result.config
    index = _sample_indices(result, far_step, near_range)

    quaternions = np.empty((len(index), 4))
    previous = None
    for row, k in enumerate(index):
        q = rotation_to_scene_quaternion(LVLH_TO_SCENE @ result.body_to_lvlh[k])
        if previous is not None and q @ previous < 0.0:
            q = -q
        quaternions[row] = previous = q

    phases = []
    for phase in Phase:
        mask = result.phase == int(phase)
        if mask.any():
            times = result.t[mask]
            phases.append({
                "id": int(phase), "label": PHASE_LABELS[phase], "color": PHASE_COLORS[phase],
                "start": float(times[0]), "end": float(times[-1]),
            })

    contact = []
    if result.contact is not None:
        for name, value in result.contact.values.items():
            bound = result.contact.limits[name]
            limit = f"{bound[0]} to {bound[1]}" if isinstance(bound, tuple) else f"max {bound}"
            contact.append({"name": name, "value": round(float(value), 4), "limit": limit,
                            "ok": bool(result.contact.passed[name])})

    radius = config.environment.r_eq + config.chief.altitude_km
    wheel_limit = config.deputy.wheels.max_momentum

    def flat(values, decimals):
        return np.round(np.asarray(values)[index], decimals).ravel().tolist()

    return {
        "meta": {
            "title": title or "Closed-loop docking simulation",
            "outcome": result.outcome,
            "abort_reason": result.abort_reason,
            "epoch": config.epoch.strftime("%Y-%m-%d %H:%M UTC"),
            "mean_motion": float(np.sqrt(config.environment.mu / radius**3)),
            "altitude_m": config.chief.altitude_km * 1e3,
            "earth_radius_m": config.environment.r_eq * 1e3,
            "hold_points": [np.round(p, 3).tolist() for p in result.hold_positions],
            "chief_port": np.asarray(config.chief.port_position, dtype=float).tolist(),
            "chief_port_axis": np.asarray(config.chief.port_axis, dtype=float).tolist(),
            "chief_target": np.asarray(config.chief.target_position, dtype=float).tolist(),
            "deputy_port_body": np.asarray(config.deputy.port_position_body, dtype=float).tolist(),
            "sensor_position_body": np.asarray(config.deputy.sensors.relative_position_body, dtype=float).tolist(),
            "sensor_fov_deg": config.deputy.sensors.relative_fov_half_angle_deg,
            "corridor_half_angle_deg": config.guidance.corridor_half_angle_deg,
            "corridor_radius": config.guidance.corridor_radius,
            "max_thrust": config.deputy.thrusters.max_force,
            "phases": phases,
            "events": [[float(t), label, note] for t, label, note in result.events],
            "contact": contact,
        },
        "t": flat(result.t, 1),
        "ph": result.phase[index].astype(int).tolist(),
        "p": flat(result.relative_state[:, :3], 3),
        "g": flat(result.relative_estimate[:, :3], 3),
        "q": np.round(quaternions, 5).ravel().tolist(),
        "f": flat(result.thrust, 2),
        "s": flat(result.sun_lvlh, 3),
        "e": result.eclipse[index].astype(int).tolist(),
        "ax": flat(result.port_axial, 3),
        "la": np.round(np.linalg.norm(result.port_lateral[index], axis=1), 4).tolist(),
        "cs": flat(result.port_closing_speed, 4),
        "ae": np.round(np.degrees(result.attitude_error[index]), 3).tolist(),
        "dv": flat(result.delta_v, 3),
        "wm": np.round(np.abs(result.wheel_momentum[index]).max(axis=1) / wheel_limit, 3).tolist(),
    }


def export_viewer(result, path, title=None, far_step=5, near_range=100.0):
    """
    Write a 3D viewer of a simulation as a single HTML file.

    The file contains the data of the run and the code that plays it
    back: the two spacecraft in the LVLH frame of the chief, with the
    Earth below, the Sun where it was, the trajectory, the approach
    corridor, the thruster firings, and the numbers of the approach. It
    opens in any recent browser, with no server and nothing to install.
    The 3D library, three.js, is loaded from a public content delivery
    network, so the browser needs a connection the first time.

    Parameters
    ----------
    result : ios_mission_planner.sim.simulator.SimulationResult
    path : str or pathlib.Path
        Output file.
    title : str, optional
    far_step, near_range :
        See build_viewer_data.

    Returns
    -------
    pathlib.Path

    Author: Giovanni Facchinetti, 2026
    """

    data = build_viewer_data(result, title, far_step, near_range)
    template = TEMPLATE.read_text(encoding="utf-8")

    # "</" inside the JSON would end the script element that carries it
    payload = json.dumps(data, separators=(",", ":")).replace("</", "<\\/")
    html = template.replace("__TITLE__", data["meta"]["title"]).replace("__DATA__", payload)

    path = Path(path)
    path.write_text(html, encoding="utf-8")

    return path
