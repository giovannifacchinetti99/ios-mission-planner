import numpy as np

from ios_mission_planner.linalg import cross3, unit3


def quaternion_to_rotation_matrix(q):
    """
    Rotation matrix from a quaternion, Hamilton convention (q0 scalar
    part), consistent with the quaternion used throughout
    ios_mission_planner.dynamics.attitude.rigid_body: q represents the
    rotation from the inertial frame to the body frame, and the returned
    matrix maps inertial-frame vector components to body-frame
    components, v_body = R @ v_inertial.

    Parameters
    ----------
    q : array_like, shape (4,)
        (q0, q1, q2, q3), not required to be exactly unit norm.

    Returns
    -------
    numpy.ndarray, shape (3, 3)

    Author: Giovanni Facchinetti, 2026
    Reference: Markley, Crassidis, Fundamentals of Spacecraft Attitude
    Determination and Control
    """

    q = np.asarray(q, dtype=float)
    q0, q1, q2, q3 = q / np.sqrt(q @ q)

    return np.array([
        [1 - 2 * (q2**2 + q3**2), 2 * (q1 * q2 + q0 * q3), 2 * (q1 * q3 - q0 * q2)],
        [2 * (q1 * q2 - q0 * q3), 1 - 2 * (q1**2 + q3**2), 2 * (q2 * q3 + q0 * q1)],
        [2 * (q1 * q3 + q0 * q2), 2 * (q2 * q3 - q0 * q1), 1 - 2 * (q1**2 + q2**2)],
    ])


def rotation_matrix_to_quaternion(r):
    """
    Quaternion from a rotation matrix, by Shepperd's method: picks
    whichever of the four algebraically equivalent formulas has the
    largest denominator, so the result stays numerically well-conditioned
    regardless of the rotation angle (the naive single formula loses
    precision, or divides by near-zero, close to a 180 degree rotation).

    Parameters
    ----------
    r : array_like, shape (3, 3)
        A proper rotation matrix (orthonormal, determinant +1).

    Returns
    -------
    numpy.ndarray, shape (4,)
        Unit quaternion (q0, q1, q2, q3), Hamilton convention.

    Author: Giovanni Facchinetti, 2026
    Reference: Shepperd, "Quaternion from Rotation Matrix," Journal of
    Guidance and Control, 1(3), 1978.
    """

    r = np.asarray(r, dtype=float)
    trace = r[0, 0] + r[1, 1] + r[2, 2]

    # note: the antisymmetric (difference) terms below are deliberately
    # R[j,i] - R[i,j], not R[i,j] - R[j,i], the order Shepperd's method is
    # usually stated with; this matrix is the inertial-to-body (passive)
    # convention used throughout this package, the transpose of the
    # body-to-inertial convention that order is usually derived for
    if trace > 0.0:
        s = 2.0 * np.sqrt(trace + 1.0)
        q0 = 0.25 * s
        q1 = (r[1, 2] - r[2, 1]) / s
        q2 = (r[2, 0] - r[0, 2]) / s
        q3 = (r[0, 1] - r[1, 0]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = 2.0 * np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
        q0 = (r[1, 2] - r[2, 1]) / s
        q1 = 0.25 * s
        q2 = (r[0, 1] + r[1, 0]) / s
        q3 = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = 2.0 * np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
        q0 = (r[2, 0] - r[0, 2]) / s
        q1 = (r[0, 1] + r[1, 0]) / s
        q2 = 0.25 * s
        q3 = (r[1, 2] + r[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
        q0 = (r[0, 1] - r[1, 0]) / s
        q1 = (r[0, 2] + r[2, 0]) / s
        q2 = (r[1, 2] + r[2, 1]) / s
        q3 = 0.25 * s

    q = np.array([q0, q1, q2, q3])
    return q / np.linalg.norm(q)


def triad_attitude(b1, b2, r1, r2):
    """
    Deterministic two-vector attitude determination (the TRIAD method).

    Given two direction measurements in the body frame (b1, b2) and the
    same two directions as known in the reference (e.g. inertial) frame
    (r1, r2), builds an orthonormal triad from each pair the same way, and
    returns the rotation that maps one triad onto the other. b1/r1 is
    trusted exactly; b2/r2 only fixes the plane, so this is deterministic,
    not a least-squares fit, and does not average away noise in either
    vector the way wahba_svd_attitude does with more than two
    measurements.

    Parameters
    ----------
    b1, b2 : array_like, shape (3,)
        Two (not necessarily unit) direction vectors, in the body frame.
    r1, r2 : array_like, shape (3,)
        The same two directions, in the reference frame.

    Returns
    -------
    numpy.ndarray, shape (3, 3)
        Rotation matrix mapping reference-frame vector components to
        body-frame components, v_body = R @ v_reference.

    Author: Giovanni Facchinetti, 2026
    Reference: Markley, Crassidis, Fundamentals of Spacecraft Attitude
    Determination and Control
    """

    b1 = unit3(b1)
    b2 = unit3(b2)
    r1 = unit3(r1)
    r2 = unit3(r2)

    t1_b = b1
    t2_b = unit3(cross3(b1, b2))
    t3_b = cross3(t1_b, t2_b)

    t1_r = r1
    t2_r = unit3(cross3(r1, r2))
    t3_r = cross3(t1_r, t2_r)

    # t_b @ t_r.T, with the triads as the columns of t_b and t_r
    return np.outer(t1_b, t1_r) + np.outer(t2_b, t2_r) + np.outer(t3_b, t3_r)


def wahba_svd_attitude(b_vectors, r_vectors, weights=None):
    """
    Attitude determination from any number of vector measurements, by the
    singular value decomposition solution to Wahba's problem.

    Wahba's problem asks for the rotation R minimizing the weighted sum of
    squared errors between measured body-frame directions and the same
    directions rotated from the reference frame, sum_i w_i |b_i -
    R r_i|^2. This is the same problem the classic QUEST algorithm solves
    (and gives the identical answer to, since both derive from the same
    attitude profile matrix); this implementation solves it through the
    SVD, which is simpler and numerically robust with modern linear
    algebra, rather than QUEST's quaternion eigenvalue iteration, a
    shortcut only worth taking when a full eigendecomposition itself was
    too expensive to compute, not a consideration today.

    Parameters
    ----------
    b_vectors : array_like, shape (n, 3)
        n direction measurements, in the body frame.
    r_vectors : array_like, shape (n, 3)
        The same n directions, in the reference frame.
    weights : array_like, shape (n,), optional
        Per-measurement weight (e.g. inverse measurement variance).
        Defaults to equal weights.

    Returns
    -------
    numpy.ndarray, shape (3, 3)
        Rotation matrix mapping reference-frame vector components to
        body-frame components, v_body = R @ v_reference.

    Author: Giovanni Facchinetti, 2026
    Reference: Wahba, "A Least Squares Estimate of Satellite Attitude,"
    SIAM Review, 7(3), 1965; Markley, "Attitude Determination Using
    Vector Observations and the Singular Value Decomposition," Journal of
    the Astronautical Sciences, 36(3), 1988.
    """

    b_vectors = np.asarray(b_vectors, dtype=float)
    r_vectors = np.asarray(r_vectors, dtype=float)
    n = b_vectors.shape[0]
    weights = np.ones(n) if weights is None else np.asarray(weights, dtype=float)

    b_hat = b_vectors / np.linalg.norm(b_vectors, axis=1, keepdims=True)
    r_hat = r_vectors / np.linalg.norm(r_vectors, axis=1, keepdims=True)

    b_matrix = sum(w * np.outer(b, r) for w, b, r in zip(weights, b_hat, r_hat))

    u, _, vt = np.linalg.svd(b_matrix)
    d = np.linalg.det(u) * np.linalg.det(vt)

    return u @ np.diag([1.0, 1.0, d]) @ vt
