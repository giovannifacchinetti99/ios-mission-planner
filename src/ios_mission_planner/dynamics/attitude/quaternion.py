import numpy as np


def quaternion_multiply(q, p):
    """
    Hamilton product q (x) p of two quaternions, both (q0, q1, q2, q3)
    with the scalar part first.

    With the inertial-to-body convention of this package, q (x) p is the
    attitude reached by applying first q and then the rotation p
    expressed in the body frame that q has reached.

    Author: Giovanni Facchinetti, 2026
    """

    q0, q1, q2, q3 = q
    p0, p1, p2, p3 = p

    return np.array([
        q0 * p0 - q1 * p1 - q2 * p2 - q3 * p3,
        q0 * p1 + q1 * p0 + q2 * p3 - q3 * p2,
        q0 * p2 - q1 * p3 + q2 * p0 + q3 * p1,
        q0 * p3 + q1 * p2 - q2 * p1 + q3 * p0,
    ])


def quaternion_conjugate(q):
    """Conjugate (inverse, for a unit quaternion) of q."""

    return np.array([q[0], -q[1], -q[2], -q[3]])


def rotation_vector_to_quaternion(rotation_vector):
    """
    Unit quaternion of the rotation by |rotation_vector| radians about
    the direction of rotation_vector.

    Author: Giovanni Facchinetti, 2026
    """

    rotation_vector = np.asarray(rotation_vector, dtype=float)
    angle = np.sqrt(rotation_vector @ rotation_vector)

    if angle < 1e-12:
        q = np.array([1.0, *(0.5 * rotation_vector)])
        return q / np.sqrt(q @ q)

    return np.array([np.cos(0.5 * angle), *(np.sin(0.5 * angle) / angle * rotation_vector)])


def quaternion_to_rotation_vector(q):
    """
    Rotation vector (axis times angle, with the angle in [0, pi]) of a
    unit quaternion. Inverse of rotation_vector_to_quaternion.

    Author: Giovanni Facchinetti, 2026
    """

    q = np.asarray(q, dtype=float)
    if q[0] < 0.0:
        q = -q

    vector_norm = np.sqrt(q[1:] @ q[1:])
    if vector_norm < 1e-12:
        return 2.0 * q[1:]

    angle = 2.0 * np.arctan2(vector_norm, q[0])
    return angle / vector_norm * q[1:]


def quaternion_error(q, q_target):
    """
    Error quaternion conj(q_target) (x) q: the rotation, expressed in the
    target body frame, that takes the target attitude to the actual one.
    The sign is chosen so that the scalar part is not negative, which
    selects the shorter of the two equivalent rotations.

    Author: Giovanni Facchinetti, 2026
    """

    error = quaternion_multiply(quaternion_conjugate(q_target), q)
    return -error if error[0] < 0.0 else error
