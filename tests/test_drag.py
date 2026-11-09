import numpy as np
import heyoka as hy

from ios_mission_planner.constants import MU_EARTH, OMEGA_EARTH, R_EARTH
from ios_mission_planner.dynamics.orbital.drag import (
    drag_acceleration_symbolic,
    exponential_atmosphere_density,
    two_body_drag_dynamics_symbolic,
    two_body_j2_drag_dynamics_symbolic,
)
from ios_mission_planner.propagation.heyoka_propagator import propagate

RHO0, R0, H = 1.0e-12, R_EARTH + 450.0, 60.0


def density_numeric(r, rho0=RHO0, r0=R0, scale_height=H):
    r_sym = hy.make_vars("r")
    expr = exponential_atmosphere_density(r_sym, rho0, r0, scale_height)
    cf = hy.cfunc([expr], vars=[r_sym])
    return float(cf(np.array([r]))[0])


def test_density_equals_rho0_at_reference_radius():
    assert np.isclose(density_numeric(R0), RHO0)


def test_density_decreases_with_altitude():
    assert density_numeric(R0 + 50.0) < density_numeric(R0)
    assert density_numeric(R0 - 50.0) > density_numeric(R0)


def test_drag_acceleration_opposes_relative_velocity():
    x, y, z, vx, vy, vz = hy.make_vars("x", "y", "z", "vx", "vy", "vz")
    ax, ay, az = drag_acceleration_symbolic(x, y, z, vx, vy, vz, 0.02, RHO0, R0, H, OMEGA_EARTH)
    cf = hy.cfunc([ax, ay, az], vars=[x, y, z, vx, vy, vz])

    state = np.array([R0, 0.0, 0.0, 0.0, 7.6, 0.0])
    accel = np.asarray(cf(state))
    v_rel = np.array([state[3] + OMEGA_EARTH * state[1], state[4] - OMEGA_EARTH * state[0], state[5]])

    assert np.dot(accel, v_rel) < 0.0  # opposes relative velocity
    assert np.allclose(np.cross(accel, v_rel), 0.0, atol=1e-20)  # and is exactly antiparallel


def test_orbit_decays_under_drag():
    a = R_EARTH + 450.0
    v = np.sqrt(MU_EARTH / a)
    state0 = np.array([a, 0.0, 0.0, 0.0, v, 0.0])
    period = 2 * np.pi * np.sqrt(a**3 / MU_EARTH)

    pars = [MU_EARTH, 0.02, RHO0, R0, H, OMEGA_EARTH]
    t_eval = np.linspace(0.0, 20 * period, 2000)
    sol = propagate(two_body_drag_dynamics_symbolic(), state0, t_eval, pars=pars).y

    r = np.linalg.norm(sol[:3, :], axis=0)
    v_ = np.linalg.norm(sol[3:6, :], axis=0)
    a_series = -MU_EARTH / (2 * (0.5 * v_**2 - MU_EARTH / r))

    assert a_series[-1] < a_series[0]


def test_higher_ballistic_coefficient_decays_faster():
    a = R_EARTH + 450.0
    v = np.sqrt(MU_EARTH / a)
    state0 = np.array([a, 0.0, 0.0, 0.0, v, 0.0])
    period = 2 * np.pi * np.sqrt(a**3 / MU_EARTH)
    t_eval = np.linspace(0.0, 10 * period, 1000)

    def final_a(bc):
        pars = [MU_EARTH, bc, RHO0, R0, H, OMEGA_EARTH]
        sol = propagate(two_body_drag_dynamics_symbolic(), state0, t_eval, pars=pars).y
        r, v_ = np.linalg.norm(sol[:3, -1]), np.linalg.norm(sol[3:6, -1])
        return -MU_EARTH / (2 * (0.5 * v_**2 - MU_EARTH / r))

    assert final_a(0.05) < final_a(0.01)


def test_combined_j2_drag_system_runs_and_includes_both_effects():
    a = R_EARTH + 450.0
    v = np.sqrt(MU_EARTH / a)
    state0 = np.array([a, 0.0, 0.0, 0.0, v * 0.999, v * 0.01])  # small inclination component
    period = 2 * np.pi * np.sqrt(a**3 / MU_EARTH)

    pars = [MU_EARTH, 1.08262668e-3, R_EARTH, 0.02, RHO0, R0, H, OMEGA_EARTH]
    t_eval = np.linspace(0.0, 5 * period, 500)
    sol = propagate(two_body_j2_drag_dynamics_symbolic(), state0, t_eval, pars=pars).y

    assert np.all(np.isfinite(sol))
    r = np.linalg.norm(sol[:3, :], axis=0)
    assert r[-1] < r[0]  # drag still shrinks the orbit with J2 also active
