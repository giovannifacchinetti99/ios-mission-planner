import numpy as np

from ios_mission_planner.dynamics.attitude.determination import quaternion_to_rotation_matrix
from ios_mission_planner.dynamics.attitude.quaternion import (
    quaternion_conjugate,
    quaternion_error,
    quaternion_multiply,
    quaternion_to_rotation_vector,
    rotation_vector_to_quaternion,
)


def _random_quaternion(rng):
    q = rng.standard_normal(4)
    return q / np.linalg.norm(q)


def test_product_composes_rotation_matrices_in_body_order():
    rng = np.random.default_rng(1)
    q, p = _random_quaternion(rng), _random_quaternion(rng)

    # q (x) p applies q first and then p in the frame reached by q, so
    # the inertial-to-body matrices multiply in the opposite order
    composed = quaternion_to_rotation_matrix(quaternion_multiply(q, p))
    expected = quaternion_to_rotation_matrix(p) @ quaternion_to_rotation_matrix(q)

    assert np.allclose(composed, expected, atol=1e-12)


def test_conjugate_is_the_inverse():
    q = _random_quaternion(np.random.default_rng(2))

    assert np.allclose(quaternion_multiply(q, quaternion_conjugate(q)), [1.0, 0.0, 0.0, 0.0], atol=1e-12)


def test_rotation_vector_round_trip():
    rng = np.random.default_rng(3)
    for angle in (1e-9, 1e-3, 1.0, 3.0):
        axis = rng.standard_normal(3)
        rotation_vector = angle * axis / np.linalg.norm(axis)
        recovered = quaternion_to_rotation_vector(rotation_vector_to_quaternion(rotation_vector))

        assert np.allclose(recovered, rotation_vector, rtol=1e-9, atol=1e-15)

    # beyond half a turn the same attitude is reached the short way around
    long_way = rotation_vector_to_quaternion([0.0, 0.0, np.deg2rad(200.0)])
    assert np.allclose(quaternion_to_rotation_vector(long_way), [0.0, 0.0, -np.deg2rad(160.0)])


def test_rotation_vector_angle_is_the_rotation_angle():
    q = rotation_vector_to_quaternion([0.0, 0.0, 0.5])
    rotation = quaternion_to_rotation_matrix(q)

    # a rotation of the body by +0.5 rad about z: an inertial x vector has
    # components (cos, -sin) in the body
    assert np.allclose(rotation @ [1.0, 0.0, 0.0], [np.cos(0.5), -np.sin(0.5), 0.0], atol=1e-12)


def test_error_quaternion_takes_the_short_way():
    target = rotation_vector_to_quaternion([0.1, -0.2, 0.3])
    actual = quaternion_multiply(target, rotation_vector_to_quaternion([0.0, 0.02, 0.0]))

    for sign in (1.0, -1.0):
        error = quaternion_error(sign * actual, target)
        assert error[0] > 0.0
        assert np.allclose(quaternion_to_rotation_vector(error), [0.0, 0.02, 0.0], atol=1e-12)
