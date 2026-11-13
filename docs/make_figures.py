"""
Generate the schematic figures used in the theory chapters of the book.

These are explanatory diagrams (frames, geometry, block diagrams), not
results: the plots of results live in the example notebooks. The output
goes to docs/book/figures/ and is committed, so the book can be built
without matplotlib.

Run with the project environment: python docs/make_figures.py
"""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch, Polygon, Rectangle

FIGURES = Path(__file__).resolve().parent / "book" / "figures"

INK = "#1b2733"
BLUE = "#1f77b4"
RED = "#d62728"
GREY = "#8a94a0"
EARTH = "#c6dbef"

plt.rcParams.update({
    "font.size": 11,
    "figure.facecolor": "white",
    "savefig.facecolor": "white",
})


def arrow(ax, start, end, color=INK, lw=1.8, style="-|>", scale=14, zorder=5, ls="-"):
    ax.add_patch(FancyArrowPatch(
        start, end, arrowstyle=style, mutation_scale=scale, color=color, lw=lw,
        zorder=zorder, linestyle=ls, shrinkA=0, shrinkB=0,
    ))


def save(fig, name):
    fig.savefig(FIGURES / name, dpi=170, bbox_inches="tight", pad_inches=0.1)
    plt.close(fig)


def lvlh_frame():
    fig, ax = plt.subplots(figsize=(6.4, 3.6))

    radius = 6.0
    earth_center = np.array([0.0, -radius])
    ax.add_patch(Circle(earth_center, 4.2, color=EARTH, zorder=1))
    ax.text(earth_center[0], earth_center[1] + 3.3, "Earth", ha="center", color=INK)

    theta = np.linspace(np.deg2rad(40), np.deg2rad(140), 200)
    ax.plot(earth_center[0] + radius * np.cos(theta), earth_center[1] + radius * np.sin(theta),
            color=GREY, lw=1.2, ls="--", zorder=2)
    ax.text(4.9, -2.45, "orbit of the chief", color=GREY, ha="right", va="center", fontsize=10)

    chief = np.array([0.0, 0.0])
    arrow(ax, earth_center + np.array([0.0, 4.2]), chief, color=GREY, lw=1.0, style="-", ls=":")
    ax.text(-0.15, -1.0, r"$\mathbf{r}_c$", color=GREY, fontsize=12, ha="right")

    arrow(ax, chief, chief + np.array([0.0, 1.9]), color=BLUE)
    ax.text(-0.15, 1.95, r"radial (R-bar)  $\hat{\mathbf{x}}$", color=BLUE, va="center", ha="right")
    arrow(ax, chief, chief + np.array([1.9, 0.0]), color=BLUE)
    ax.text(2.0, 0.28, r"$\hat{\mathbf{y}}$  along-track (V-bar)", color=BLUE, ha="left", va="center")

    # x up and y to the right leave z pointing into the page
    z_mark = chief + np.array([-0.55, -0.45])
    ax.add_patch(Circle(z_mark, 0.16, fill=False, color=BLUE, lw=1.6, zorder=6))
    d = 0.11
    ax.plot([z_mark[0] - d, z_mark[0] + d], [z_mark[1] - d, z_mark[1] + d], color=BLUE, lw=1.4, zorder=7)
    ax.plot([z_mark[0] - d, z_mark[0] + d], [z_mark[1] + d, z_mark[1] - d], color=BLUE, lw=1.4, zorder=7)
    ax.text(-0.85, -0.47, r"cross-track, into the page  $\hat{\mathbf{z}}$", color=BLUE, va="center", ha="right")

    ax.plot(*chief, "o", color=INK, ms=7, zorder=8)
    ax.text(0.2, -0.38, "chief", color=INK, ha="left")

    deputy = np.array([-2.6, 1.0])
    arrow(ax, chief, deputy, color=RED)
    ax.plot(*deputy, "o", color=RED, ms=7, zorder=8)
    ax.text(deputy[0] - 0.15, deputy[1] + 0.12, "deputy", color=RED, ha="right")
    ax.text(-1.45, 0.82, r"$\mathbf{\rho}$", color=RED, fontsize=13)

    arrow(ax, np.array([3.9, 0.95]), np.array([5.1, 0.45]), color=GREY, lw=1.2, scale=11)
    ax.text(3.8, 1.05, "direction of flight", color=GREY, fontsize=10, ha="right", va="bottom")

    ax.set_xlim(-6.2, 6.2)
    ax.set_ylim(-3.0, 2.4)
    ax.set_aspect("equal")
    ax.axis("off")
    save(fig, "lvlh_frame.png")


def relative_orbit():
    fig, axes = plt.subplots(1, 2, figsize=(10.2, 4.0))
    phase = np.linspace(0.0, 2 * np.pi, 300)

    # closed ellipse: x = rho cos(phase), y = -2 rho sin(phase)
    ax = axes[0]
    rho = 1.0
    x = rho * np.cos(phase)
    y = -2 * rho * np.sin(phase)
    ax.plot(y, x, color=RED, lw=1.8)
    for ph in (0.15, np.pi + 0.15):
        arrow(ax, (-2 * rho * np.sin(ph), rho * np.cos(ph)),
              (-2 * rho * np.sin(ph + 0.12), rho * np.cos(ph + 0.12)), color=RED, lw=1.8)
    ax.plot(0, 0, "o", color=INK, ms=7)
    ax.text(0.1, -0.24, "chief", color=INK)
    ax.annotate("", xy=(0, rho), xytext=(0, 0), arrowprops=dict(arrowstyle="<->", color=GREY, lw=1.0))
    ax.text(0.1, 0.5, r"$\rho$", color=GREY, fontsize=12)
    ax.annotate("", xy=(-2 * rho, 0), xytext=(0, 0), arrowprops=dict(arrowstyle="<->", color=GREY, lw=1.0))
    ax.text(-1.1, 0.1, r"$2\rho$", color=GREY, fontsize=12)
    ax.set_title(r"$x_{\mathrm{off}} = 0$: closed ellipse", fontsize=11, color=INK)

    # drifting orbit: the center slides by -3 pi x_off per revolution
    ax = axes[1]
    rho, x_off = 0.55, 0.45
    long_phase = np.linspace(0.0, 3 * 2 * np.pi, 900)
    x = x_off + rho * np.cos(long_phase)
    y = 3.2 - 1.5 * x_off * long_phase / (2 * np.pi) * 2.0 - 2 * rho * np.sin(long_phase)
    ax.plot(y, x, color=RED, lw=1.8)
    arrow(ax, (y[-12], x[-12]), (y[-1], x[-1]), color=RED, lw=1.8)
    ax.axhline(x_off, color=GREY, lw=1.0, ls="--")
    ax.text(3.9, x_off + 0.07, r"$x_{\mathrm{off}}$", color=GREY, fontsize=12, ha="right")
    ax.plot(0, 0, "o", color=INK, ms=7)
    ax.text(0.1, -0.24, "chief", color=INK)
    arrow(ax, (1.2, -0.75), (-1.2, -0.75), color=GREY, lw=1.2, scale=11)
    ax.text(0.0, -1.0, r"drift $-3\pi\,x_{\mathrm{off}}$ per orbit", color=GREY, ha="center", fontsize=10)
    ax.set_title(r"$x_{\mathrm{off}} > 0$: the ellipse drifts backward", fontsize=11, color=INK)

    for ax in axes:
        ax.axhline(0, color=INK, lw=0.6)
        ax.axvline(0, color=INK, lw=0.6)
        ax.set_xlim(-4.2, 4.2)
        ax.set_ylim(-1.35, 1.35)
        ax.set_aspect("equal")
        ax.set_xticks([])
        ax.set_yticks([])
        for spine in ax.spines.values():
            spine.set_visible(False)
        ax.text(4.15, 0.07, r"$y$ (along-track)", ha="right", color=INK, fontsize=10)
        ax.text(0.08, 1.25, r"$x$ (radial)", va="center", color=INK, fontsize=10)

    fig.tight_layout()
    save(fig, "relative_orbit.png")


def box(ax, center, width, height, text, face="#eef3f9", edge=INK, fontsize=10.5):
    x, y = center
    ax.add_patch(FancyBboxPatch((x - width / 2, y - height / 2), width, height,
                                boxstyle="round,pad=0.02,rounding_size=0.08", fc=face, ec=edge, lw=1.3))
    ax.text(x, y, text, ha="center", va="center", color=INK, fontsize=fontsize)


def gnc_loop():
    fig, ax = plt.subplots(figsize=(10.4, 3.9))

    names = ["Sensors", "Navigation", "Guidance", "Control", "Actuators"]
    xs = np.linspace(1.0, 9.0, 5)
    y_low, y_top = 0.9, 2.9
    width, height = 1.45, 0.75

    box(ax, (5.0, y_top), 8.0 + width, height,
        "Truth model: orbits of chief and deputy, attitude, wheels, mass", face="#fdf1e3")
    for x, name in zip(xs, names):
        box(ax, (x, y_low), width, height, name)

    labels = ["measurements", "estimates", "references", "commands"]
    for x0, x1, label in zip(xs[:-1], xs[1:], labels):
        arrow(ax, (x0 + width / 2, y_low), (x1 - width / 2, y_low), color=BLUE, lw=1.6)
        ax.text(0.5 * (x0 + x1), y_low - 0.52, label, ha="center", va="top", color=BLUE, fontsize=9.5)

    arrow(ax, (xs[0], y_top - height / 2), (xs[0], y_low + height / 2), color=RED, lw=1.6)
    ax.text(xs[0] + 0.12, 1.9, "true state", color=RED, fontsize=9.5, va="center")
    arrow(ax, (xs[-1], y_low + height / 2), (xs[-1], y_top - height / 2), color=RED, lw=1.6)
    ax.text(xs[-1] - 0.12, 1.9, "force and torque\nactually produced", color=RED, fontsize=9.5, va="center",
            ha="right")
    ax.text(5.0, 1.9, "on board: only measurements and estimates", color=GREY, fontsize=9.5, ha="center",
            va="center", style="italic")

    ax.set_xlim(0.0, 10.0)
    ax.set_ylim(-0.05, 3.45)
    ax.axis("off")
    save(fig, "gnc_loop.png")


def approach_profile():
    fig, ax = plt.subplots(figsize=(10.4, 3.7))

    # positions on a compressed, not-to-scale axis
    chief, s3, s2, s1 = 9.0, 7.6, 5.4, 1.0

    ax.plot([0.3, 9.9], [0, 0], color=GREY, lw=1.0, ls="--", zorder=1)
    ax.text(3.2, 0.1, "V-bar", color=GREY, ha="center", fontsize=10)
    arrow(ax, (8.6, 1.55), (9.7, 1.55), color=GREY, lw=1.2, scale=11)
    ax.text(9.15, 1.68, "direction of flight", color=GREY, ha="center", fontsize=9)
    arrow(ax, (9.6, -0.95), (9.6, -1.65), color=GREY, lw=1.2, scale=11)
    ax.text(9.48, -1.3, "to the Earth", color=GREY, va="center", ha="right", fontsize=9)

    # corridor
    ax.add_patch(Polygon([[chief - 0.25, 0.0], [s2 - 0.2, 0.62], [s2 - 0.2, -0.62]], closed=True,
                         fc="#9467bd", ec="none", alpha=0.13, zorder=1))
    ax.text(6.35, 0.7, "approach corridor", color="#7a52a8", fontsize=9, ha="center")

    # chief
    ax.add_patch(Rectangle((chief - 0.05, -0.28), 0.56, 0.56, fc="#fdf1e3", ec=INK, lw=1.3, zorder=4))
    ax.add_patch(Rectangle((chief - 0.25, -0.1), 0.2, 0.2, fc=GREY, ec=INK, lw=1.0, zorder=4))
    ax.text(chief + 0.23, 0.42, "chief", ha="center", color=INK)
    ax.text(chief - 0.15, -0.32, "port", ha="center", va="top", color=INK, fontsize=8.5)

    # transfer arc
    t = np.linspace(0.0, np.pi, 100)
    arc_x = 0.5 * (s1 + s2) - 0.5 * (s2 - s1) * np.cos(t)
    arc_y = -1.15 * np.sin(t)
    ax.plot(arc_x, arc_y, color=BLUE, lw=1.8, zorder=3)
    arrow(ax, (arc_x[52], arc_y[52]), (arc_x[56], arc_y[56]), color=BLUE, lw=1.8)
    for k in (25, 50, 75):
        ax.plot(arc_x[k], arc_y[k], "o", color=BLUE, ms=4, zorder=5)
    ax.text(0.5 * (s1 + s2), -1.42, "transfer: two burns, a free arc, corrections on the way", color=BLUE,
            ha="center", fontsize=9.5)

    # forced approach and final approach
    arrow(ax, (s2, 0), (s3 - 0.08, 0), color="#e07b00", lw=2.2)
    ax.text(0.5 * (s2 + s3), -0.22, "forced approach", color="#e07b00", ha="center", va="top", fontsize=9.5)
    arrow(ax, (s3, 0), (chief - 0.27, 0), color=RED, lw=2.2)
    ax.text(0.5 * (s3 + chief - 0.25), -0.22, "final\napproach", color=RED, ha="center", va="top", fontsize=9)

    for x, name, distance in ((s1, "S1", "5 km"), (s2, "S2", "500 m"), (s3, "S3", "20 m")):
        ax.plot(x, 0, "^", color=INK, ms=9, zorder=6)
        ax.text(x, 0.22, name, ha="center", color=INK, fontsize=11, fontweight="bold")
        ax.text(x, 0.52, distance, ha="center", color=GREY, fontsize=9)

    ax.set_xlim(0.1, 10.0)
    ax.set_ylim(-1.9, 1.95)
    ax.axis("off")
    save(fig, "approach_profile.png")


def beta_eclipse():
    fig, axes = plt.subplots(1, 2, figsize=(10.4, 4.1), gridspec_kw={"width_ratios": [1.25, 1]})

    # left: orbit seen edge-on, perpendicular to the Sun line
    ax = axes[0]
    radius, orbit = 1.0, 1.32
    beta = np.deg2rad(32.0)
    ax.add_patch(Rectangle((0.0, -radius), 2.6, 2 * radius, fc="#22303f", ec="none", alpha=0.16))
    ax.add_patch(Circle((0, 0), radius, color=EARTH, zorder=2))
    ax.text(-0.42, -0.5, "Earth", ha="center", va="center", color=INK)
    ax.text(1.85, -0.55, "shadow", ha="center", va="center", color=INK, fontsize=10)
    for y in (-0.7, 0.0, 0.7):
        arrow(ax, (-2.6, y), (-1.75, y), color="#d9a400", lw=1.6)
    ax.text(-2.15, 0.95, "sunlight", color="#b98a00", ha="center", fontsize=10)

    direction = np.array([np.cos(beta), np.sin(beta)])
    ax.plot(*np.array([-orbit * direction, orbit * direction]).T, color=RED, lw=2.0, zorder=3)
    ax.plot(*(orbit * direction), "o", color=RED, ms=6, zorder=4)
    ax.text(1.28, 0.86, "orbit plane,\nseen edge-on", color=RED, fontsize=9.5, ha="left")
    ax.plot([0, 1.6], [0, 0], color=GREY, lw=0.9, ls="--", zorder=3)
    arc = np.linspace(0, beta, 30)
    ax.plot(0.62 * np.cos(arc) + 1.0 * 0, 0.62 * np.sin(arc), color=INK, lw=1.0, zorder=5)
    ax.text(1.12, 0.22, r"$\beta$", color=INK, fontsize=13, zorder=6)
    ax.set_xlim(-2.8, 2.7)
    ax.set_ylim(-1.5, 1.5)
    ax.set_aspect("equal")
    ax.axis("off")

    # right: direction of the Sun in the LVLH frame during one orbit
    ax = axes[1]
    theta = np.linspace(0.0, 2 * np.pi, 721)       # from orbital noon
    along, radial = -np.sin(theta), np.cos(theta)
    half_shadow = np.arcsin(6378.0 / 6878.0)
    eclipse = np.abs(theta - np.pi) < half_shadow
    lit = (np.sin(theta) > np.cos(np.deg2rad(70.0))) & ~eclipse

    ax.plot(along, radial, color="#cfd4da", lw=5, solid_capstyle="butt")
    ax.plot(np.where(eclipse, along, np.nan), np.where(eclipse, radial, np.nan), color="#22303f", lw=5)
    ax.plot(np.where(lit, along, np.nan), np.where(lit, radial, np.nan), color="#2ca02c", lw=5)
    ax.plot(0, 0, "o", color=INK, ms=6)
    arrow(ax, (0, 0), (-0.62, 0), color=RED)
    ax.text(-0.33, 0.08, "port normal", color=RED, fontsize=9, ha="center")
    arrow(ax, (0, 0), (0, -0.6), color=GREY, lw=1.2, scale=11)
    ax.text(0.06, -0.42, "to the Earth", color=GREY, fontsize=9)
    arrow(ax, (0.25, 0.25), (0.75, 0.25), color=GREY, lw=1.2, scale=11)
    ax.text(0.5, 0.33, "flight", color=GREY, fontsize=9, ha="center")
    ax.text(0.0, 1.3, "noon: Sun overhead", ha="center", color=INK, fontsize=9)
    ax.text(0.0, -1.2, "eclipse", ha="center", color=INK, fontsize=9.5)
    ax.text(-1.14, 0.3, "port lit", ha="right", color="#1e7e34", fontsize=9.5)
    ax.text(1.12, 0.3, "port in its\nown shadow", ha="left", color=GREY, fontsize=9)
    arc = np.linspace(np.deg2rad(98), np.deg2rad(128), 20)
    ax.plot(1.14 * np.cos(arc), 1.14 * np.sin(arc), color=INK, lw=1.0)
    arrow(ax, (1.14 * np.cos(arc[-2]), 1.14 * np.sin(arc[-2])), (1.14 * np.cos(arc[-1]), 1.14 * np.sin(arc[-1])),
          color=INK, lw=1.0, scale=10)
    ax.set_xlim(-1.75, 1.85)
    ax.set_ylim(-1.4, 1.5)
    ax.set_aspect("equal")
    ax.axis("off")

    fig.tight_layout()
    save(fig, "beta_eclipse.png")


def main():
    FIGURES.mkdir(parents=True, exist_ok=True)
    lvlh_frame()
    relative_orbit()
    gnc_loop()
    approach_profile()
    beta_eclipse()
    print(f"figures written to {FIGURES}")


if __name__ == "__main__":
    main()
