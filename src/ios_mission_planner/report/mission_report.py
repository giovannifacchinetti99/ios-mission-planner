import base64
import io
from dataclasses import dataclass, field
from pathlib import Path

import jinja2
import matplotlib.pyplot as plt
import numpy as np

from ios_mission_planner.gnc.mission_manager import PHASE_LABELS, Phase
from ios_mission_planner.illumination.lighting import intervals
from ios_mission_planner.report.pipeline import FEASIBLE, FEASIBLE_WITH_CAVEATS, NOT_FEASIBLE
from ios_mission_planner.sim.monte_carlo import CONTACT_QUANTITIES
from ios_mission_planner.sim.simulator import DOCKED

TEMPLATE_DIR = Path(__file__).resolve().parent / "templates"

VERDICT_COLOR = {FEASIBLE: "#1e7e34", FEASIBLE_WITH_CAVEATS: "#b8860b", NOT_FEASIBLE: "#b22222"}

BLUE, RED, PURPLE, GREY = "#1f77b4", "#d62728", "#9467bd", "#7f7f7f"


@dataclass
class MissionRequirements:
    """
    What the mission analysis is judged against.

    Attributes
    ----------
    min_success_probability : float
        Probability of a docking inside the contact limits that the
        campaign must demonstrate: the lower end of the confidence
        interval of the success rate must not be below it.
    confidence : float
        Confidence level of that interval.
    max_delta_v : float
        Velocity change available for the approach [m/s].
    delta_v_percentile : float
        Percentile of the campaign compared with max_delta_v.
    contact_margin_warning : float
        A contact condition whose worst case over the campaign exceeds
        this fraction of its limit is reported.
    require_lighting : bool
        Whether the final approach must be flown in favourable lighting.

    Author: Giovanni Facchinetti, 2026
    """

    min_success_probability: float = 0.95
    confidence: float = 0.95
    max_delta_v: float = 8.0
    delta_v_percentile: float = 99.0
    contact_margin_warning: float = 0.5
    require_lighting: bool = True


@dataclass
class MissionAnalysis:
    """
    Result of analyze_mission: the inputs, the verdict and the reasons.

    Attributes
    ----------
    name : str
    nominal : ios_mission_planner.sim.simulator.SimulationResult
    campaign : ios_mission_planner.sim.monte_carlo.MonteCarloResult or None
    lighting : ios_mission_planner.illumination.lighting.LightingTimeline or None
    requirements : MissionRequirements
    verdict : str
        "FEASIBLE", "FEASIBLE WITH CAVEATS" or "NOT FEASIBLE".
    issues : list of str
        Reasons for a verdict of not feasible.
    caveats : list of str
        Findings that do not prevent the mission but must be known.
    final_approach : tuple or None
        Start and end time of the final approach of the nominal run [s].
    final_approach_lit : bool or None
        Whether the lighting was favourable throughout it.
    """

    name: str
    nominal: object
    campaign: object
    lighting: object
    requirements: MissionRequirements
    verdict: str = FEASIBLE
    issues: list = field(default_factory=list)
    caveats: list = field(default_factory=list)
    final_approach: tuple = None
    final_approach_lit: bool = None


def analyze_mission(name, nominal, campaign=None, lighting=None, requirements=None):
    """
    Judge a mission from its closed-loop simulation, its Monte Carlo
    campaign and its lighting, against explicit requirements.

    As for the feasibility screening, there is no score. The mission is
    not feasible if the nominal run does not dock, if the campaign shows
    a probability of success below the required one, or if the velocity
    change at the required percentile exceeds what is available. It is
    feasible with caveats if it passes those but something must be known:
    a campaign too small to demonstrate the required probability, a
    contact condition that uses a large part of its limit, runs that
    ended in an abort, a final approach flown in the dark. Each line of
    the result refers to a number and to the threshold beside it.

    Parameters
    ----------
    name : str
        Name of the mission, for the report.
    nominal : ios_mission_planner.sim.simulator.SimulationResult
        Run with nominal hardware.
    campaign : ios_mission_planner.sim.monte_carlo.MonteCarloResult, optional
    lighting : ios_mission_planner.illumination.lighting.LightingTimeline, optional
        Lighting of the docking port, from the epoch of the simulation.
    requirements : MissionRequirements, optional

    Returns
    -------
    MissionAnalysis

    Author: Giovanni Facchinetti, 2026
    """

    requirements = MissionRequirements() if requirements is None else requirements
    analysis = MissionAnalysis(name, nominal, campaign, lighting, requirements)
    issues, caveats = analysis.issues, analysis.caveats

    if nominal.outcome != DOCKED:
        detail = nominal.abort_reason or ", ".join(nominal.contact.violations if nominal.contact else [])
        issues.append(f"The nominal run does not end in a docking inside the limits: {nominal.outcome}"
                      + (f" ({detail})." if detail else "."))

    if campaign is None:
        caveats.append("No Monte Carlo campaign was run: the probability of success and the propellant "
                       "at a high percentile are not assessed.")
    else:
        required = requirements.min_success_probability
        lower, _ = campaign.success_interval(requirements.confidence)
        failures = campaign.n_runs - int(campaign.success.sum())

        if campaign.success_rate < required:
            issues.append(f"The probability of success observed over {campaign.n_runs} runs is "
                          f"{campaign.success_rate:.3f}, below the required {required:.3f}.")
        elif lower < required:
            caveats.append(f"The campaign of {campaign.n_runs} runs is too small to demonstrate the required "
                           f"probability of success: the lower end of the {requirements.confidence * 100:.0f} % "
                           f"interval is {lower:.3f} against {required:.3f}.")
        if failures and campaign.success_rate >= required:
            caveats.append(f"{failures} of {campaign.n_runs} runs did not dock: {_failure_summary(campaign)}.")

        percentile = float(np.percentile(campaign.delta_v, requirements.delta_v_percentile))
        if percentile > requirements.max_delta_v:
            issues.append(f"The velocity change at the {requirements.delta_v_percentile:.0f}th percentile is "
                          f"{percentile:.2f} m/s, above the {requirements.max_delta_v:.2f} m/s available.")

        for name_, statistics in campaign.contact_statistics().items():
            limit = campaign.limits[name_]
            if isinstance(limit, tuple):
                half = 0.5 * (limit[1] - limit[0])
                used = max(statistics["max"] - 0.5 * (limit[0] + limit[1]),
                           0.5 * (limit[0] + limit[1]) - statistics["min"]) / half
            else:
                used = statistics["max"] / limit
            if used > requirements.contact_margin_warning:
                caveats.append(f"The worst case of the {name_.split(' [')[0]} over the campaign uses "
                               f"{used * 100:.0f} % of its limit.")

    final = nominal.phase == int(Phase.FINAL_APPROACH)
    if final.any():
        analysis.final_approach = (float(nominal.t[final][0]), float(nominal.t[final][-1]))

    if lighting is not None:
        windows = lighting.windows()
        if not windows:
            text = "The docking port has no favourable lighting window over the span analyzed."
            (issues if requirements.require_lighting else caveats).append(text)
        elif analysis.final_approach is not None:
            start, end = analysis.final_approach
            analysis.final_approach_lit = lighting.favourable_for(start, end - start)
            if not analysis.final_approach_lit and requirements.require_lighting:
                following = [w for w in windows if w[0] >= start and w[1] - w[0] >= end - start]
                wait = f" The next window long enough opens {(following[0][0] - start) / 60:.0f} min later." \
                    if following else ""
                caveats.append("The final approach of the nominal run is not flown in favourable lighting; "
                               "it has to be held at the last hold point until a window opens." + wait)

    if issues:
        analysis.verdict = NOT_FEASIBLE
    elif caveats:
        analysis.verdict = FEASIBLE_WITH_CAVEATS
    else:
        analysis.verdict = FEASIBLE

    return analysis


def _failure_summary(campaign):
    reasons = {}
    for outcome, reason, violations in zip(campaign.outcomes, campaign.abort_reasons, campaign.violations):
        if outcome != DOCKED:
            key = f"{outcome} ({reason or ', '.join(v.split(' [')[0] for v in violations) or 'no contact'})"
            reasons[key] = reasons.get(key, 0) + 1
    return "; ".join(f"{count} {key}" for key, count in reasons.items())


def _figure_to_base64(fig):
    buffer = io.BytesIO()
    fig.savefig(buffer, format="png", dpi=140, bbox_inches="tight")
    plt.close(fig)
    return base64.b64encode(buffer.getvalue()).decode("ascii")


def _phase_bands(ax, result):
    colors = {Phase.TRANSFER: BLUE, Phase.APPROACH: "#ff7f0e", Phase.FINAL_APPROACH: RED, Phase.ABORT: PURPLE}
    for phase, color in colors.items():
        mask = result.phase == int(phase)
        if mask.any():
            ax.axvspan(result.t[mask][0] / 60, result.t[mask][-1] / 60, color=color, alpha=0.08, lw=0)


def _trajectory_figure(result):
    x = result.relative_state
    limits = result.config.docking_limits
    fig, axes = plt.subplots(1, 2, figsize=(10.5, 3.6), gridspec_kw={"width_ratios": [1.5, 1]})

    axes[0].plot(x[:, 1], x[:, 0], color=BLUE, lw=1.6)
    for label, point in zip(("S1", "S2", "S3"), result.hold_positions):
        axes[0].plot(point[1], point[0], "k^", ms=6)
        axes[0].annotate(label, (point[1], point[0]), textcoords="offset points", xytext=(0, 8), ha="center",
                         fontsize=8)
    axes[0].plot(0, 0, "*", color=PURPLE, ms=12)
    axes[0].set_xlabel("along-track [m]")
    axes[0].set_ylabel("radial [m]")
    axes[0].set_title("Approach trajectory")

    near = result.port_axial < 25.0
    axes[1].plot(-result.port_axial[near], result.port_lateral[near, 0], color=RED, lw=1.3, label="radial")
    axes[1].plot(-result.port_axial[near], result.port_lateral[near, 1], color=BLUE, lw=1.3, label="cross-track")
    axes[1].axhline(limits.lateral_misalignment, color="k", ls=":", lw=1)
    axes[1].axhline(-limits.lateral_misalignment, color="k", ls=":", lw=1, label="limit at contact")
    axes[1].set_ylim(-2.5 * limits.lateral_misalignment, 2.5 * limits.lateral_misalignment)
    axes[1].set_xlabel("distance between the ports [m]")
    axes[1].set_ylabel("offset of the deputy port [m]")
    axes[1].set_title("Last 25 m")
    axes[1].legend(fontsize=7)
    for ax in axes:
        ax.grid(alpha=0.3)
    fig.tight_layout()

    return _figure_to_base64(fig)


def _timeline_figure(result):
    minutes = result.t / 60
    fig, axes = plt.subplots(1, 3, figsize=(12.5, 3.3))

    axes[0].semilogy(minutes, np.maximum(result.port_axial, 1e-2), color=BLUE)
    axes[0].set_ylabel("distance between the ports [m]")
    axes[0].set_title("Range")

    late = result.phase >= int(Phase.APPROACH)
    limits = result.config.docking_limits
    axes[1].plot(minutes[late], result.port_closing_speed[late], color=RED)
    axes[1].axhspan(limits.axial_speed_min, limits.axial_speed_max, color="k", alpha=0.08)
    axes[1].set_ylabel("closing speed [m/s]")
    axes[1].set_title("Closing speed")

    error = np.linalg.norm(result.relative_state[:, :3] - result.relative_estimate[:, :3], axis=1)
    bound = 3.0 * np.linalg.norm(result.relative_std[:, :3], axis=1)
    axes[2].semilogy(minutes, np.maximum(error, 1e-5), color=RED, lw=0.7, label="position error")
    axes[2].semilogy(minutes, bound, color=BLUE, lw=1.3, label="3 sigma")
    axes[2].set_ylabel("navigation error [m]")
    axes[2].set_title("Navigation")
    axes[2].legend(fontsize=7)

    for ax in axes:
        _phase_bands(ax, result)
        for start, end in intervals(minutes, result.eclipse):
            ax.axvspan(start, end, color="k", alpha=0.10, lw=0)
        ax.set_xlabel("time [min]")
        ax.grid(alpha=0.3)
    fig.tight_layout()

    return _figure_to_base64(fig)


def _campaign_figure(campaign):
    reached = ~np.isnan(campaign.contact[:, 0])
    names, ratios = [], []
    for column, name in enumerate(CONTACT_QUANTITIES):
        limit = campaign.limits[name]
        values = campaign.contact[reached, column]
        if isinstance(limit, tuple):
            center, half = 0.5 * (limit[0] + limit[1]), 0.5 * (limit[1] - limit[0])
            ratios.append(np.abs(values - center) / half)
        else:
            ratios.append(values / limit)
        names.append(name.split(" [")[0])

    fig, axes = plt.subplots(1, 2, figsize=(11.5, 3.5))
    axes[0].boxplot(ratios, orientation="horizontal", tick_labels=names, widths=0.6, whis=(0, 100),
                    medianprops={"color": RED})
    axes[0].axvline(1.0, color=RED, ls="--", lw=1)
    axes[0].set_xscale("log")
    axes[0].invert_yaxis()
    axes[0].set_xlabel("value at contact, as a fraction of its limit")
    axes[0].set_title("Contact conditions over the campaign")

    axes[1].hist(campaign.delta_v, bins=30, color=BLUE, alpha=0.85)
    axes[1].axvline(np.percentile(campaign.delta_v, 99), color=RED, ls="--", lw=1, label="99th percentile")
    axes[1].set_xlabel("delta-v of the approach [m/s]")
    axes[1].set_ylabel("runs")
    axes[1].set_title("Velocity change")
    axes[1].legend(fontsize=7)
    for ax in axes:
        ax.grid(alpha=0.3)
    fig.tight_layout()

    return _figure_to_base64(fig)


def _lighting_figure(lighting, final_approach):
    minutes = lighting.t / 60
    fig, ax = plt.subplots(figsize=(11.5, 2.8))

    ax.plot(minutes, np.degrees(lighting.incidence), color=BLUE, label="Sun incidence on the port")
    ax.fill_between(minutes, 0, 180, where=lighting.eclipse, color="k", alpha=0.15, lw=0, label="eclipse")
    ax.fill_between(minutes, 0, 180, where=lighting.favourable, color="#2ca02c", alpha=0.18, lw=0,
                    label="favourable")
    if final_approach is not None:
        ax.axvspan(final_approach[0] / 60, final_approach[1] / 60, color=RED, alpha=0.35, lw=0,
                   label="final approach")
    ax.set_ylim(0, 180)
    ax.set_xlabel("time from the epoch [min]")
    ax.set_ylabel("incidence [deg]")
    ax.set_title("Lighting of the docking port")
    ax.legend(fontsize=7, ncol=4, loc="upper right")
    ax.grid(alpha=0.3)
    fig.tight_layout()

    return _figure_to_base64(fig)


def render_mission_report(analysis):
    """
    Render a MissionAnalysis into a self-contained HTML document: the
    verdict and its reasons, the scenario, the timeline and the propellant
    of the nominal run, the conditions at contact, the statistics of the
    Monte Carlo campaign and the lighting of the port, with the figures
    embedded. The document can be converted to PDF with
    ios_mission_planner.report.render.render_pdf.

    Parameters
    ----------
    analysis : MissionAnalysis

    Returns
    -------
    str

    Author: Giovanni Facchinetti, 2026
    """

    nominal, campaign, lighting = analysis.nominal, analysis.campaign, analysis.lighting
    config = nominal.config

    spent = nominal.delta_v_by_phase()
    intervals = nominal.phase_intervals()
    timeline = [
        {"label": label, "start": start / 60, "duration": (end - start) / 60, "delta_v": spent.get(label, 0.0)}
        for label, (start, end) in intervals.items() if label != PHASE_LABELS[Phase.DOCKED]
    ]

    contact = []
    if nominal.contact is not None:
        for name, value in nominal.contact.values.items():
            bound = nominal.contact.limits[name]
            contact.append({
                "name": name, "value": value,
                "limit": f"{bound[0]} to {bound[1]}" if isinstance(bound, tuple) else f"below {bound}",
                "passed": nominal.contact.passed[name],
            })

    campaign_view = None
    if campaign is not None:
        lower, upper = campaign.success_interval(analysis.requirements.confidence)
        statistics = []
        for name, values in campaign.contact_statistics().items():
            bound = campaign.limits[name]
            statistics.append({
                "name": name, **values,
                "limit": f"{bound[0]} to {bound[1]}" if isinstance(bound, tuple) else f"below {bound}",
            })
        campaign_view = {
            "n_runs": campaign.n_runs, "outcomes": campaign.outcome_counts(),
            "rate": campaign.success_rate, "lower": lower, "upper": upper,
            "statistics": statistics,
            "dv_mean": float(campaign.delta_v.mean()), "dv_std": float(campaign.delta_v.std()),
            "dv_percentile": float(np.percentile(campaign.delta_v, analysis.requirements.delta_v_percentile)),
            "dv_max": float(campaign.delta_v.max()),
            "propellant_max": float(campaign.propellant.max()),
            "figure": _campaign_figure(campaign),
        }

    lighting_view = None
    if lighting is not None:
        windows = lighting.windows()
        lighting_view = {
            "beta": float(np.degrees(lighting.beta[0])),
            "eclipse_fraction": float(lighting.eclipse.mean()),
            "favourable_fraction": float(lighting.favourable.mean()),
            "windows": [{"start": a / 60, "end": b / 60, "duration": (b - a) / 60} for a, b in windows[:8]],
            "figure": _lighting_figure(lighting, analysis.final_approach),
        }

    env = jinja2.Environment(loader=jinja2.FileSystemLoader(TEMPLATE_DIR), autoescape=True)
    template = env.get_template("mission_report.html.jinja")

    return template.render(
        analysis=analysis, config=config, nominal=nominal,
        verdict_color=VERDICT_COLOR[analysis.verdict],
        timeline=timeline, contact=contact, campaign=campaign_view, lighting=lighting_view,
        trajectory_figure=_trajectory_figure(nominal),
        timeline_figure=_timeline_figure(nominal),
        epoch=config.epoch.strftime("%Y-%m-%d %H:%M UTC"),
    )
