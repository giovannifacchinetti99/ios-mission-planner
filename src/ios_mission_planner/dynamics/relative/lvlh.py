import numpy as np


def lvlh_rotation(chief_state):
    """
    Rotation matrix from the inertial frame to the chief's LVLH frame.

    Rows are the LVLH axes expressed in inertial coordinates: x radial
    (away from the central body), y along-track (completes the triad, along
    the velocity for a circular orbit), z cross-track (orbit normal).

    Parameters
    ----------
    chief_state : array_like
        Inertial state of the chief [x, y, z, vx, vy, vz].

    Author: Giovanni Facchinetti, 2026
    """

    chief_state = np.asarray(chief_state, dtype=float)
    r, v = chief_state[:3], chief_state[3:]

    r_hat = r / np.linalg.norm(r)
    h = np.cross(r, v)
    h_hat = h / np.linalg.norm(h)
    theta_hat = np.cross(h_hat, r_hat)

    return np.vstack([r_hat, theta_hat, h_hat])


def lvlh_angular_rate(chief_state):
    """Angular rate of the LVLH frame, |r x v| / |r|^2 [rad/s]."""

    chief_state = np.asarray(chief_state, dtype=float)
    r, v = chief_state[:3], chief_state[3:]

    return np.linalg.norm(np.cross(r, v)) / np.linalg.norm(r) ** 2


def inertial_to_lvlh(chief_state, deputy_state):
    """
    Relative state of the deputy in the chief's LVLH frame.

    The relative velocity is the one seen by an observer rotating with the
    frame, so the frame's own rotation is subtracted: v_rel = R (v_d - v_c) - w x rho.

    Units are whatever the inputs use (km and km/s give km and km/s).

    Returns
    -------
    numpy.ndarray
        Relative state [x, y, z, vx, vy, vz] in the LVLH frame.

    Author: Giovanni Facchinetti, 2026
    Reference: Analytical Mechanics of Space Systems, Fourth Edition
    """

    chief_state = np.asarray(chief_state, dtype=float)
    deputy_state = np.asarray(deputy_state, dtype=float)

    rotation = lvlh_rotation(chief_state)
    omega = np.array([0.0, 0.0, lvlh_angular_rate(chief_state)])

    rho = rotation @ (deputy_state[:3] - chief_state[:3])
    rho_dot = rotation @ (deputy_state[3:] - chief_state[3:]) - np.cross(omega, rho)

    return np.concatenate([rho, rho_dot])


def lvlh_to_inertial(chief_state, relative_state):
    """
    Inertial state of the deputy from its relative state in the chief's LVLH frame.

    Inverse of inertial_to_lvlh.

    Author: Giovanni Facchinetti, 2026
    """

    chief_state = np.asarray(chief_state, dtype=float)
    relative_state = np.asarray(relative_state, dtype=float)

    rotation = lvlh_rotation(chief_state)
    omega = np.array([0.0, 0.0, lvlh_angular_rate(chief_state)])

    rho, rho_dot = relative_state[:3], relative_state[3:]

    position = chief_state[:3] + rotation.T @ rho
    velocity = chief_state[3:] + rotation.T @ (rho_dot + np.cross(omega, rho))

    return np.concatenate([position, velocity])
