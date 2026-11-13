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
from matplotlib.patches import Circle, FancyArrowPatch

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


def main():
    FIGURES.mkdir(parents=True, exist_ok=True)
    lvlh_frame()
    relative_orbit()
    print(f"figures written to {FIGURES}")


if __name__ == "__main__":
    main()
