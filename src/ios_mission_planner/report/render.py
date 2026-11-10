import base64
import io
from pathlib import Path

import jinja2
import matplotlib.pyplot as plt

TEMPLATE_DIR = Path(__file__).resolve().parent / "templates"

VERDICT_COLOR = {
    "FEASIBLE": "#1e7e34",
    "FEASIBLE WITH CAVEATS": "#b8860b",
    "NOT FEASIBLE": "#b22222",
}


def _figure_to_base64(fig):
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=140, bbox_inches="tight")
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode("ascii")


def _budget_figure(result):
    labels = [line.label for line in result.budget_lines]
    nominal = [line.delta_v for line in result.budget_lines]
    margin = [line.margined_delta_v - line.delta_v for line in result.budget_lines]

    fig, ax = plt.subplots(figsize=(8, 0.5 * len(labels) + 1.5))
    ax.barh(labels, nominal, color="#1f77b4", label="nominal")
    ax.barh(labels, margin, left=nominal, color="#9467bd", label="margin")
    ax.set_xlabel("delta-v [m/s]")
    ax.set_title("Delta-v budget by phase")
    ax.legend(fontsize=8)
    ax.invert_yaxis()
    fig.tight_layout()

    return _figure_to_base64(fig)


def _trajectory_figure(result):
    traj = result.hop_trajectory_m
    fig, ax = plt.subplots(figsize=(6.5, 5))
    ax.plot(traj[:, 1], traj[:, 0], color="#d62728", lw=2)
    ax.scatter([traj[0, 1]], [traj[0, 0]], color="k", zorder=5, label="hold point")
    ax.scatter([traj[-1, 1]], [traj[-1, 0]], color="#1e7e34", marker="^", s=80, zorder=5, label="ellipse far point")
    ax.scatter([0], [0], color="#9467bd", marker="*", s=120, zorder=5, label="chief")
    ax.set_xlabel("along-track [m]")
    ax.set_ylabel("radial [m]")
    ax.set_title("Far-range approach trajectory")
    ax.set_aspect("equal")
    ax.legend(fontsize=8)
    fig.tight_layout()

    return _figure_to_base64(fig)


def render_html(result):
    """
    Render a FeasibilityResult into a self-contained HTML report (figures
    embedded as base64 PNGs, no external files needed to view it).

    Parameters
    ----------
    result : ios_mission_planner.report.pipeline.FeasibilityResult

    Returns
    -------
    str
        The rendered HTML document.

    Author: Giovanni Facchinetti, 2026
    """

    env = jinja2.Environment(loader=jinja2.FileSystemLoader(TEMPLATE_DIR), autoescape=True)
    template = env.get_template("report.html.jinja")

    return template.render(
        result=result,
        scenario=result.scenario,
        verdict_color=VERDICT_COLOR[result.verdict],
        budget_figure=_budget_figure(result),
        trajectory_figure=_trajectory_figure(result),
    )


def render_pdf(html, path):
    """
    Export an already-rendered HTML report to a PDF file, with weasyprint.

    Parameters
    ----------
    html : str
        HTML produced by render_html.
    path : str or pathlib.Path
        Output PDF path.

    Author: Giovanni Facchinetti, 2026
    """

    import weasyprint

    weasyprint.HTML(string=html).write_pdf(path)
