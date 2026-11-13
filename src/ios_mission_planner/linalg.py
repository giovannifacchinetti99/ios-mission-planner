"""
Small vector helpers for three-dimensional vectors.

numpy.cross and numpy.linalg.norm are general, and for single 3-vectors
most of their time goes into handling the general case. Inside a
simulation loop that calls them many times per step, these direct
versions are an order of magnitude faster and give the same result.
"""

import numpy as np


def cross3(a, b):
    """Cross product of two 3-vectors."""

    return np.array([
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ])


def norm3(a):
    """Euclidean norm of a 3-vector."""

    return float(np.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]))


def unit3(a):
    """Unit vector along a 3-vector."""

    return np.asarray(a, dtype=float) / norm3(a)
