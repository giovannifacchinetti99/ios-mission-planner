import heyoka as hy


def hill_equations_symbolic():
    """
    Hill-Clohessy-Wiltshire equations for a circular reference orbit, as a
    symbolic heyoka ODE system.

    State vector: [x, y, z, vx, vy, vz].
    Runtime parameter: par[0] = n, mean motion of the chief [rad/s].

    Returns
    -------
    list[tuple[heyoka.expression, heyoka.expression]]
        Symbolic system, for use with
        ios_mission_planner.propagation.heyoka_propagator.propagate.

    Author: Giovanni Facchinetti, 2026
    Reference: Analytical Mechanics of Space Systems, Fourth Edition
    """

    x, y, z, vx, vy, vz = hy.make_vars("x", "y", "z", "vx", "vy", "vz")
    n = hy.par[0]

    ax = 2.0 * n * vy + 3.0 * n**2 * x
    ay = -2.0 * n * vx
    az = -(n**2) * z

    return [
        (x, vx),
        (y, vy),
        (z, vz),
        (vx, ax),
        (vy, ay),
        (vz, az),
    ]
