"""
Copy the example notebooks into docs/book/examples/, so Jupyter Book can
embed them without the book's source root reaching outside docs/book/.

Run before every `jupyter-book build docs/book`. The destination folder is
gitignored and rebuilt from scratch at every run; this script is the only
thing that populates it. Every notebook under examples/ is copied, together
with the small data files some of them read (element sets, scenarios), so a
new numbered example folder needs no change here, only an entry in
docs/book/_toc.yml.
"""

import shutil
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
EXAMPLES_SRC = REPO_ROOT / "examples"
EXAMPLES_DST = REPO_ROOT / "docs" / "book" / "examples"

COPIED_SUFFIXES = {".ipynb", ".tle", ".yaml"}


def main():
    if EXAMPLES_DST.exists():
        shutil.rmtree(EXAMPLES_DST)

    for src in sorted(EXAMPLES_SRC.glob("*/*")):
        if src.suffix not in COPIED_SUFFIXES or ".ipynb_checkpoints" in src.parts:
            continue
        relpath = src.relative_to(EXAMPLES_SRC)
        dst = EXAMPLES_DST / relpath
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
        print(f"synced {relpath.as_posix()}")


if __name__ == "__main__":
    main()
