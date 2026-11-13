import copy
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, field

import numpy as np
from scipy.stats import norm

from ios_mission_planner.sim.simulator import DOCKED, simulate

CONTACT_QUANTITIES = (
    "lateral misalignment [m]",
    "axial speed [m/s]",
    "lateral speed [m/s]",
    "angular misalignment [deg]",
    "roll misalignment [deg]",
    "angular rate [deg/s]",
)


@dataclass
class Dispersions:
    """
    Standard deviations of the quantities that change from one run of a
    Monte Carlo campaign to the next. Each is drawn from a zero-mean
    normal distribution, independently on each axis, and replaces the
    corresponding value of the base scenario; the atmospheric density is
    multiplied by a log-normal factor instead.

    The sensor noises are not listed here: they are part of the scenario
    and every run draws its own sequence from its own seed.

    Attributes
    ----------
    position_offset, velocity_offset : float
        True initial deviation from the hold point [m, m/s].
    attitude_error, body_rate : float
        True initial attitude error [rad] and body rate error [rad/s].
    navigation_position_error, navigation_velocity_error : float
        Error of the first navigation estimate [m, m/s].
    thruster_scale_error : float
        Fractional error of the thrust on each axis.
    thruster_misalignment : float
        Misalignment of the thruster set [rad].
    center_of_mass_offset : float
        Offset of the center of mass from the thrust lines [m].
    sensor_misalignment : float
        Mounting error of the relative sensor [rad].
    chief_attitude_error : float
        Attitude error of the chief with respect to its nominal
        orientation [rad].
    density_log10 : float
        Standard deviation of the base-10 logarithm of the factor on the
        atmospheric density.

    Author: Giovanni Facchinetti, 2026
    """

    position_offset: float = 20.0
    velocity_offset: float = 0.02
    attitude_error: float = np.deg2rad(2.0)
    body_rate: float = np.deg2rad(0.02)
    navigation_position_error: float = 25.0
    navigation_velocity_error: float = 0.03
    thruster_scale_error: float = 0.03
    thruster_misalignment: float = np.deg2rad(0.5)
    center_of_mass_offset: float = 0.002
    sensor_misalignment: float = np.deg2rad(0.05)
    chief_attitude_error: float = np.deg2rad(0.3)
    density_log10: float = 0.3


def sample_config(base, dispersions, seed):
    """
    One dispersed scenario: a copy of the base scenario with the
    dispersed quantities drawn from their distributions.

    The draw depends only on the seed, so a run can be repeated exactly
    by sampling again with the same seed.

    Parameters
    ----------
    base : ios_mission_planner.sim.config.SimulationConfig
    dispersions : Dispersions
    seed : int

    Returns
    -------
    ios_mission_planner.sim.config.SimulationConfig

    Author: Giovanni Facchinetti, 2026
    """

    rng = np.random.default_rng([seed, 7919])
    config = copy.deepcopy(base)
    config.seed = seed
    d = dispersions

    initial = config.initial
    initial.position_offset = d.position_offset * rng.standard_normal(3)
    initial.velocity_offset = d.velocity_offset * rng.standard_normal(3)
    initial.attitude_error = d.attitude_error * rng.standard_normal(3)
    initial.body_rate = d.body_rate * rng.standard_normal(3)
    initial.navigation_position_error = d.navigation_position_error * rng.standard_normal(3)
    initial.navigation_velocity_error = d.navigation_velocity_error * rng.standard_normal(3)

    thrusters = config.deputy.thrusters
    thrusters.scale_error = d.thruster_scale_error * rng.standard_normal(3)
    thrusters.misalignment = d.thruster_misalignment * rng.standard_normal(3)
    thrusters.center_of_mass_offset = d.center_of_mass_offset * rng.standard_normal(3)

    config.deputy.sensors.relative_misalignment = d.sensor_misalignment * rng.standard_normal(3)
    config.chief.attitude_error = d.chief_attitude_error * rng.standard_normal(3)
    config.environment.atmosphere_density = (
        base.environment.atmosphere_density * 10.0 ** (d.density_log10 * rng.standard_normal())
    )

    return config


def _run_one(arguments):
    """One run, reduced to the numbers the campaign keeps."""

    base, dispersions, seed, trajectory_step = arguments
    result = simulate(sample_config(base, dispersions, seed))

    contact = [np.nan] * len(CONTACT_QUANTITIES)
    violations = []
    if result.contact is not None:
        contact = [result.contact.values[name] for name in CONTACT_QUANTITIES]
        violations = result.contact.violations

    estimate_error = result.relative_state[:, :3] - result.relative_estimate[:, :3]

    return {
        "seed": seed,
        "outcome": result.outcome,
        "abort_reason": result.abort_reason,
        "violations": violations,
        "contact": contact,
        "delta_v": result.total_delta_v,
        "delta_v_by_phase": result.delta_v_by_phase(),
        "propellant": result.propellant_used,
        "duration": result.duration,
        "max_wheel_momentum": float(np.abs(result.wheel_momentum).max()),
        "final_navigation_error": float(np.linalg.norm(estimate_error[-1])),
        "trajectory": result.relative_state[::trajectory_step, :3].copy(),
        "trajectory_time": result.t[::trajectory_step].copy(),
    }


def wilson_interval(successes, trials, confidence=0.95):
    """
    Confidence interval for a probability estimated from a number of
    successes out of a number of independent trials, by Wilson's score
    method. Unlike the textbook normal approximation, it remains
    meaningful when the observed frequency is 0 or 1, which is the
    interesting case here: a campaign in which every run succeeds does
    not show that the probability of success is one, only that it is
    above the lower end of this interval.

    Parameters
    ----------
    successes, trials : int
    confidence : float
        Two-sided confidence level.

    Returns
    -------
    lower, upper : float

    Author: Giovanni Facchinetti, 2026
    Reference: Wilson, "Probable Inference, the Law of Succession, and
    Statistical Inference," Journal of the American Statistical
    Association, 22(158), 1927
    """

    if trials == 0:
        return 0.0, 1.0

    z = norm.ppf(0.5 + 0.5 * confidence)
    p = successes / trials
    denominator = 1.0 + z**2 / trials
    center = (p + z**2 / (2.0 * trials)) / denominator
    half = z * np.sqrt(p * (1.0 - p) / trials + z**2 / (4.0 * trials**2)) / denominator

    return float(max(0.0, center - half)), float(min(1.0, center + half))


@dataclass
class MonteCarloResult:
    """
    Outcome of a Monte Carlo campaign, one entry per run.

    Attributes
    ----------
    seeds : numpy.ndarray
        Seed of each run; sample_config(base, dispersions, seed)
        reproduces its scenario.
    outcomes : list of str
        Outcome of each run, see ios_mission_planner.sim.simulator.
    abort_reasons : list
        Reason of the abort, or None.
    violations : list of list
        Contact conditions outside their limits.
    contact : numpy.ndarray, shape (n, 6)
        Conditions at first contact, in the order of CONTACT_QUANTITIES;
        not-a-number for the runs without contact.
    delta_v, propellant, duration : numpy.ndarray
        Velocity change delivered [m/s], propellant used [kg] and
        duration of the run [s].
    delta_v_by_phase : list of dict
    max_wheel_momentum : numpy.ndarray
        Largest momentum stored in a wheel [N m s].
    final_navigation_error : numpy.ndarray
        Position error of the navigation at the end of the run [m].
    trajectories, trajectory_times : list of numpy.ndarray
        True relative position of each run, sampled coarsely [m, s].
    limits : dict
        Limits of the contact conditions, by name.

    Author: Giovanni Facchinetti, 2026
    """

    seeds: np.ndarray
    outcomes: list
    abort_reasons: list
    violations: list
    contact: np.ndarray
    delta_v: np.ndarray
    propellant: np.ndarray
    duration: np.ndarray
    delta_v_by_phase: list
    max_wheel_momentum: np.ndarray
    final_navigation_error: np.ndarray
    trajectories: list = field(repr=False)
    trajectory_times: list = field(repr=False)
    limits: dict = field(default_factory=dict)

    @property
    def n_runs(self):
        return len(self.outcomes)

    @property
    def success(self):
        """Boolean array: which runs docked within the limits."""

        return np.array([outcome == DOCKED for outcome in self.outcomes])

    @property
    def success_rate(self):
        return float(self.success.mean())

    def success_interval(self, confidence=0.95):
        """Wilson confidence interval of the probability of success."""

        return wilson_interval(int(self.success.sum()), self.n_runs, confidence)

    def outcome_counts(self):
        """Number of runs for each outcome."""

        counts = {}
        for outcome in self.outcomes:
            counts[outcome] = counts.get(outcome, 0) + 1
        return counts

    def contact_statistics(self):
        """
        Mean, standard deviation and extreme values of each contact
        condition over the runs that reached contact, as
        {name: {"mean", "std", "min", "max", "p99"}}.
        """

        statistics = {}
        reached = ~np.isnan(self.contact[:, 0])
        for column, name in enumerate(CONTACT_QUANTITIES):
            values = self.contact[reached, column]
            if values.size == 0:
                continue
            statistics[name] = {
                "mean": float(values.mean()), "std": float(values.std()),
                "min": float(values.min()), "max": float(values.max()),
                "p99": float(np.percentile(values, 99.0)),
            }
        return statistics


def run_monte_carlo(base, dispersions=None, n_runs=100, first_seed=1, workers=1, trajectory_step=10,
                    progress=None):
    """
    Run a Monte Carlo campaign of the closed-loop simulation: the same
    mission flown many times, each with its own draw of the dispersed
    quantities and its own sensor noise.

    Parameters
    ----------
    base : ios_mission_planner.sim.config.SimulationConfig
        Nominal scenario.
    dispersions : Dispersions, optional
    n_runs : int
    first_seed : int
        The runs use the seeds first_seed, first_seed + 1, ...
    workers : int
        Number of processes. The runs are independent, so they can be
        distributed over the cores of the machine.
    trajectory_step : int
        One sample of the trajectory is kept every trajectory_step steps.
    progress : callable, optional
        Called with the number of runs completed.

    Returns
    -------
    MonteCarloResult

    Author: Giovanni Facchinetti, 2026
    """

    dispersions = Dispersions() if dispersions is None else dispersions
    arguments = [(base, dispersions, first_seed + k, trajectory_step) for k in range(n_runs)]

    runs = []
    if workers <= 1:
        for k, argument in enumerate(arguments):
            runs.append(_run_one(argument))
            if progress is not None:
                progress(k + 1)
    else:
        with ProcessPoolExecutor(max_workers=workers) as pool:
            for k, run in enumerate(pool.map(_run_one, arguments)):
                runs.append(run)
                if progress is not None:
                    progress(k + 1)

    limits = base.docking_limits
    return MonteCarloResult(
        seeds=np.array([run["seed"] for run in runs]),
        outcomes=[run["outcome"] for run in runs],
        abort_reasons=[run["abort_reason"] for run in runs],
        violations=[run["violations"] for run in runs],
        contact=np.array([run["contact"] for run in runs], dtype=float),
        delta_v=np.array([run["delta_v"] for run in runs]),
        propellant=np.array([run["propellant"] for run in runs]),
        duration=np.array([run["duration"] for run in runs]),
        delta_v_by_phase=[run["delta_v_by_phase"] for run in runs],
        max_wheel_momentum=np.array([run["max_wheel_momentum"] for run in runs]),
        final_navigation_error=np.array([run["final_navigation_error"] for run in runs]),
        trajectories=[run["trajectory"] for run in runs],
        trajectory_times=[run["trajectory_time"] for run in runs],
        limits={
            "lateral misalignment [m]": limits.lateral_misalignment,
            "axial speed [m/s]": (limits.axial_speed_min, limits.axial_speed_max),
            "lateral speed [m/s]": limits.lateral_speed,
            "angular misalignment [deg]": limits.angular_misalignment_deg,
            "roll misalignment [deg]": limits.roll_misalignment_deg,
            "angular rate [deg/s]": limits.angular_rate_deg_s,
        },
    )
