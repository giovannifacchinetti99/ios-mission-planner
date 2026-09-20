from dataclasses import dataclass

import numpy as np
import heyoka as hy


@dataclass
class PropagationResult:
    """
    Result of a heyoka propagation.

    Uses the same (t, y) convention as scipy.integrate.solve_ivp's
    OdeResult (y indexed [state, time]), for interface parity with the
    scipy-based propagator this replaces.

    Attributes
    ----------
    t : ndarray, shape (n_times,)
        Times at which the solution was evaluated.
    y : ndarray, shape (n_states, n_times)
        State vector at each time in t.

    Author: Giovanni Facchinetti, 2026
    """

    t: np.ndarray
    y: np.ndarray


def propagate(sys, state0, t_eval, pars=None, t0=None):
    """
    Propagate a symbolic heyoka ODE system over a time grid.

    Builds a Taylor-adaptive integrator (adaptive-order Taylor series,
    compiled just-in-time via LLVM) from a symbolic system and evaluates
    it at the requested times.

    Parameters
    ----------
    sys : list[tuple[heyoka.expression, heyoka.expression]]
        Symbolic ODE system as (state_variable, rhs_expression) pairs,
        built with heyoka.make_vars for state variables and heyoka.par[i]
        for runtime parameters (e.g.
        ios_mission_planner.dynamics.relative.hill.hill_equations_symbolic).
    state0 : ndarray
        Initial state vector, at t0.
    t_eval : ndarray
        Times at which to evaluate the solution.
    pars : ndarray, optional
        Runtime parameter values, in the order referenced by heyoka.par[i]
        in sys (default: no parameters).
    t0 : float, optional
        Initial epoch (default: t_eval[0]).

    Returns
    -------
    PropagationResult

    Author: Giovanni Facchinetti, 2026
    """

    ta = hy.taylor_adaptive(
        sys,
        state=np.asarray(state0, dtype=float).tolist(),
        time=float(t_eval[0] if t0 is None else t0),
        pars=[] if pars is None else np.asarray(pars, dtype=float).tolist(),
    )

    *_, states = ta.propagate_grid(t_eval)

    return PropagationResult(t=np.asarray(t_eval), y=states.T)
