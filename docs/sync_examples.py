"""
Copy the example notebooks into docs/book/examples/, so Jupyter Book can
embed them without the book's source root reaching outside docs/book/.

Run before every `jupyter-book build docs/book`. The destination folder is
gitignored and rebuilt from scratch at every run; this script is the only
thing that populates it. Every notebook under examples/ is copied, together
with the small data files some of them read (element sets, scenarios), so a
new numbered example folder needs no change here, only an entry in
docs/book/_toc.yml.

The standalone pages that some notebooks produce (the 3D viewers) are
copied to docs/book/_extra/, which the book publishes as it is, at the same
relative path as the notebook that created them. This is what lets a
notebook, and a chapter, embed such a page with a relative link.
"""

import shutil
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
EXAMPLES_SRC = REPO_ROOT / "examples"
EXAMPLES_DST = REPO_ROOT / "docs" / "book" / "examples"
PAGES_DST = REPO_ROOT / "docs" / "book" / "_extra" / "examples"

COPIED_SUFFIXES = {".ipynb", ".tle", ".yaml"}
PAGE_SUFFIXES = {".html"}


def main():
    for destination in (EXAMPLES_DST, PAGES_DST):
        if destination.exists():
            shutil.rmtree(destination)
    PAGES_DST.mkdir(parents=True)

    for src in sorted(EXAMPLES_SRC.glob("*/*")):
        if ".ipynb_checkpoints" in src.parts:
            continue
        if src.suffix in COPIED_SUFFIXES:
            destination = EXAMPLES_DST
        elif src.suffix in PAGE_SUFFIXES:
            destination = PAGES_DST
        else:
            continue
        relpath = src.relative_to(EXAMPLES_SRC)
        dst = destination / relpath
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
        print(f"synced {relpath.as_posix()}")


if __name__ == "__main__":
    main()
