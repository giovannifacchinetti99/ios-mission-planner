"""
Copy the example notebooks referenced by the documentation book into
docs/book/examples/, so Jupyter Book can embed them without the book's
source root reaching outside docs/book/.

Run before every `jupyter-book build docs/book`. The destination folder is
gitignored; this script is the only thing that populates it.
"""

import shutil
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
EXAMPLES_SRC = REPO_ROOT / "examples"
EXAMPLES_DST = REPO_ROOT / "docs" / "book" / "examples"

NOTEBOOKS = [
    "cw_fundamentals.ipynb",
    "eccentric_chief_relative_motion.ipynb",
    "differential_drag.ipynb",
    "j2_roe_state_transition.ipynb",
    "safety_fundamentals.ipynb",
    "mission_budget.ipynb",
    "low_thrust_arc_optimization.ipynb",
    "cosmos2612_iceye_x36_rpo.ipynb",
]

EXTRA_FILES = [
    "spacetrack_68762.tle",
    "spacetrack_59103.tle",
]


def main():
    EXAMPLES_DST.mkdir(parents=True, exist_ok=True)
    for name in NOTEBOOKS + EXTRA_FILES:
        shutil.copy2(EXAMPLES_SRC / name, EXAMPLES_DST / name)
        print(f"synced {name}")


if __name__ == "__main__":
    main()
