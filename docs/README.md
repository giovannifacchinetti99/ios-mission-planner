# Building the documentation book

The book lives under `docs/book/` and is built with
[Jupyter Book](https://jupyterbook.org). Its toolchain is isolated from the
project's conda environment (heyoka is conda-only and not needed to build
the book, since notebooks are embedded with their already-computed outputs,
not re-executed).

```
python -m venv docs/.venv
docs/.venv/Scripts/pip install -r docs/requirements.txt   # docs/.venv/bin/pip on macOS/Linux

docs/.venv/Scripts/python docs/sync_examples.py           # copies examples/*.ipynb into docs/book/examples/
docs/.venv/Scripts/jupyter-book build docs/book

# open docs/book/_build/html/index.html
```

`docs/sync_examples.py` must be re-run (and the book rebuilt) any time one of
the listed notebooks changes under `examples/`, since `docs/book/examples/`
is just a gitignored copy used to keep the book's source root inside
`docs/book/`.
