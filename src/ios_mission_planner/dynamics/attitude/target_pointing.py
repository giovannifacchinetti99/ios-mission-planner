import numpy as np

from ios_mission_planner.dynamics.attitude.determination import (
    quaternion_to_rotation_matrix,
    rotation_matrix_to_quaternion,
    triad_attitude,
)
from ios_mission_planner.dynamics.attitude.rigid_body import pd_pointing_attitude_dynamics_symbolic
from ios_mission_planner.dynamics.relative.hill import hill_equations_symbolic
from ios_mission_planner.propagation.heyoka_propagator import propagate


def target_quaternion_toward_chief(relative_position, secondary_reference=(0.0, 0.0, 1.0),
                                    primary_body_axis=(1.0, 0.0, 0.0),
                                    secondary_body_axis=(0.0, 0.0, 1.0)):
    """
    Target attitude quaternion that points primary_body_axis at the
    chief, i.e. along -relative_position, with secondary_body_axis fixed
    as closely as possible to secondary_reference (by default the LVLH
    cross-track/orbit-normal direction, a natural "roll" reference for a
    target-pointing law, the same way a nadir-pointing satellite commonly
    fixes its remaining axis to the orbit normal).

    Fixing where two body axes should end up pointing is mathematically
    the same problem as recovering a rotation from two *measured*
    directions, so this reuses triad_attitude, built for attitude
    determination elsewhere in this chapter, here for attitude design
    instead.

    This treats the frame relative_position is expressed in (LVLH,
    throughout this module) as if it were the inertial frame the rest of
    this chapter's attitude dynamics use. LVLH itself rotates, at the
    chief's orbital rate, slow compared to any realistic attitude control
    bandwidth; folding that rotation into a true inertial target would add
    real bookkeeping for a small correction, which is why this module
    states the simplification plainly instead of applying it silently.

    Parameters
    ----------
    relative_position : array_like, shape (3,)
        Deputy's position relative to the chief, in the chief's LVLH
        frame (any consistent length unit; only the direction matters).
    secondary_reference : array_like, shape (3,)
        Reference direction the secondary body axis should align with.
    primary_body_axis, secondary_body_axis : array_like, shape (3,)
        The body-frame axes being pointed (unit vectors in body
        coordinates, not reference coordinates).

    Returns
    -------
    numpy.ndarray, shape (4,)
        Target quaternion (q0, q1, q2, q3), Hamilton convention.

    Author: Giovanni Facchinetti, 2026
    """

    pointing_direction = -np.asarray(relative_position, dtype=float)
    r = triad_attitude(primary_body_axis, secondary_body_axis, pointing_direction, secondary_reference)
    return rotation_matrix_to_quaternion(r)


def boresight_pointing_error_deg(q, relative_position, body_axis=(1.0, 0.0, 0.0)):
    """
    Angle, in degrees, between a body-frame direction's actual boresight
    and the true instantaneous direction to the chief.

    Unlike the approach corridor check in the safety chapter, which only
    ever looks at the trajectory's geometry, this looks at the attitude
    actually achieved: whether a sensor pointed along body_axis would, at
    this instant, really have the chief in view.

    Parameters
    ----------
    q : array_like, shape (4,)
        Achieved attitude quaternion (same frame convention as
        target_quaternion_toward_chief: LVLH-as-reference).
    relative_position : array_like, shape (3,)
        Deputy's position relative to the chief, in the same frame.
    body_axis : array_like, shape (3,)
        The sensor's boresight direction, in body coordinates.

    Returns
    -------
    float

    Author: Giovanni Facchinetti, 2026
    """

    r = quaternion_to_rotation_matrix(q)
    boresight_in_frame = r.T @ np.asarray(body_axis, dtype=float)
    target_direction = -np.asarray(relative_position, dtype=float)
    target_direction = target_direction / np.linalg.norm(target_direction)

    cos_angle = np.dot(boresight_in_frame, target_direction) / np.linalg.norm(boresight_in_frame)
    return np.degrees(np.arccos(np.clip(cos_angle, -1.0, 1.0)))


def fly_with_periodic_retargeting(relative_state0, attitude_state0, n, inertia, kp, kd,
                                   total_duration, retarget_interval, pts_per_interval=50,
                                   secondary_reference=(0.0, 0.0, 1.0),
                                   primary_body_axis=(1.0, 0.0, 0.0),
                                   secondary_body_axis=(0.0, 0.0, 1.0)):
    """
    Fly a relative trajectory (Hill's equations) and a body attitude
    together, re-pointing primary_body_axis at the chief every
    retarget_interval and holding that target fixed in between: a
    periodic guidance update, not a continuously recomputed closed-form
    target, chosen because the alternative needs the target quaternion
    computed by conditional, branching logic (Shepperd's method inside
    rotation_matrix_to_quaternion) embedded in a single symbolic ODE
    system, a real jump in complexity this module does not take.

    Parameters
    ----------
    relative_state0 : array_like, shape (6,)
        Initial relative state [x, y, z, vx, vy, vz] in the chief's LVLH
        frame.
    attitude_state0 : array_like, shape (7,)
        Initial attitude state [q0, q1, q2, q3, wx, wy, wz].
    n : float
        Mean motion of the chief [rad/s].
    inertia : array_like, shape (3,)
        Principal moments of inertia (ix, iy, iz) [kg m^2].
    kp, kd : float
        PD pointing controller gains.
    total_duration : float
        Total time to fly [s].
    retarget_interval : float
        How often to recompute the pointing target [s].
    pts_per_interval : int
        Output samples per interval.
    secondary_reference, primary_body_axis, secondary_body_axis :
        See target_quaternion_toward_chief.

    Returns
    -------
    t : numpy.ndarray
    relative_states : numpy.ndarray, shape (n_samples, 6)
    attitude_states : numpy.ndarray, shape (n_samples, 7)
    pointing_error_deg : numpy.ndarray, shape (n_samples,)

    Author: Giovanni Facchinetti, 2026
    """

    ix, iy, iz = inertia
    n_intervals = int(np.ceil(total_duration / retarget_interval))

    relative_state = np.asarray(relative_state0, dtype=float)
    attitude_state = np.asarray(attitude_state0, dtype=float)

    t_all, rel_all, att_all = [], [], []
    t_start = 0.0

    for k in range(n_intervals):
        duration = min(retarget_interval, total_duration - t_start)
        if duration <= 0:
            break
        t_local = np.linspace(0.0, duration, pts_per_interval)

        rel_hist = propagate(hill_equations_symbolic(), relative_state, t_local, pars=[n]).y

        q_target = target_quaternion_toward_chief(
            relative_state[:3], secondary_reference, primary_body_axis, secondary_body_axis,
        )
        pars_att = [ix, iy, iz, *q_target, kp, kd]
        att_hist = propagate(pd_pointing_attitude_dynamics_symbolic(), attitude_state, t_local, pars=pars_att).y

        t_all.append(t_start + t_local)
        rel_all.append(rel_hist.T)
        att_all.append(att_hist.T)

        relative_state = rel_hist[:, -1]
        attitude_state = att_hist[:, -1]
        t_start += duration

    t = np.concatenate(t_all)
    relative_states = np.concatenate(rel_all)
    attitude_states = np.concatenate(att_all)

    pointing_error_deg = np.array([
        boresight_pointing_error_deg(attitude_states[i, :4], relative_states[i, :3], primary_body_axis)
        for i in range(len(t))
    ])

    return t, relative_states, attitude_states, pointing_error_deg
