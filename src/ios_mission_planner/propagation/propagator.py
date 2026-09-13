from scipy.integrate import solve_ivp


def propagate(dynamics, state0, t_span, t_eval, method="RK45", rtol=1e-3, atol=1e-6, **kwargs):
    """
    Generic numerical propagator.

    Parameters
    ----------
    dynamics : callable
        Function defining the equations of motion.
        Signature: dynamics(t, state, **kwargs)
    state0 : ndarray
        Initial state vector.
    t_span : tuple
        Initial and final propagation time.
    t_eval : ndarray
        Times at which to store the solution.
    method : str, optional
        Integration method passed to solve_ivp (default: "RK45").
        Use "DOP853" for high-accuracy propagation with tight tolerances.
    rtol : float, optional
        Relative tolerance passed to solve_ivp (default: 1e-3, same as
        solve_ivp's own default).
    atol : float, optional
        Absolute tolerance passed to solve_ivp (default: 1e-6, same as
        solve_ivp's own default).
    **kwargs
        Additional parameters passed to the dynamics function.

    Returns
    -------
    OdeResult
        Solution returned by scipy.integrate.solve_ivp.

    Author: Giovanni Facchinetti, 2026
    """

    solution = solve_ivp(
        fun=lambda t, state: dynamics(t, state, **kwargs),
        t_span=t_span,
        y0=state0,
        t_eval=t_eval,
        method=method,
        rtol=rtol,
        atol=atol,
    )

    return solution