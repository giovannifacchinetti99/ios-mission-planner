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
]


def main():
    EXAMPLES_DST.mkdir(parents=True, exist_ok=True)
    for name in NOTEBOOKS:
        shutil.copy2(EXAMPLES_SRC / name, EXAMPLES_DST / name)
        print(f"synced {name}")


if __name__ == "__main__":
    main()
