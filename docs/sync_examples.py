"""
Copy the example notebooks referenced by the documentation book into
docs/book/examples/, so Jupyter Book can embed them without the book's
source root reaching outside docs/book/.

Run before every `jupyter-book build docs/book`. The destination folder is
gitignored; this script is the only thing that populates it. Source paths
are relative to examples/, numbered by code progression, matching the
numbered folders under examples/ itself.
"""

import shutil
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
EXAMPLES_SRC = REPO_ROOT / "examples"
EXAMPLES_DST = REPO_ROOT / "docs" / "book" / "examples"

FILES = [
    "01_cw_fundamentals/cw_fundamentals.ipynb",
    "02_eccentric_chief_relative_motion/eccentric_chief_relative_motion.ipynb",
    "06_j2_roe_state_transition/j2_roe_state_transition.ipynb",
    "04_safety_fundamentals/safety_fundamentals.ipynb",
    "07_differential_drag/differential_drag.ipynb",
    "08_mission_budget/mission_budget.ipynb",
    "09_low_thrust_arc_optimization/low_thrust_arc_optimization.ipynb",
    "10_cosmos2612_iceye_x36_rpo/cosmos2612_iceye_x36_rpo.ipynb",
    "10_cosmos2612_iceye_x36_rpo/spacetrack_68762.tle",
    "10_cosmos2612_iceye_x36_rpo/spacetrack_59103.tle",
    "11_attitude_pointing_control/attitude_pointing_control.ipynb",
    "12_mission_feasibility_report/mission_feasibility_report.ipynb",
    "12_mission_feasibility_report/baseline_scenario.yaml",
    "12_mission_feasibility_report/improved_nav_scenario.yaml",
    "13_attitude_determination/attitude_determination.ipynb",
    "14_attitude_estimation_filter/attitude_estimation_filter.ipynb",
]


def main():
    for relpath in FILES:
        dst = EXAMPLES_DST / relpath
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(EXAMPLES_SRC / relpath, dst)
        print(f"synced {relpath}")


if __name__ == "__main__":
    main()
