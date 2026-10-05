# Contributing

Issues and pull requests are welcome: bug reports, questions about the method, new
controllers or disturbances, and corrections to the vehicle estimates (with a public source).

## Development workflow

```bash
python -m venv .venv && source .venv/bin/activate
pip install -r requirements-dev.txt
python -m playwright install chromium        # only for the replay-page checks
python -m pytest tests/ -q                   # must pass before a pull request
```

* **Library changes** (`boostercatch/`): keep the shared stack in `controllers._Base` shared,
  so the controllers stay comparable; add a test for new behaviour. See
  [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md#extending-the-code).
* **Changes that affect results**: re-run the affected campaigns (they are resumable; delete or
  move the old file first, or use `scripts/drop_rows.py` for one controller), then
  `python scripts/make_figures.py`, and update the numbers in `REPORT.md`, `README.md` and
  `docs/replay/paper.html`. Never edit result files by hand.
* **Replay page**: edit `docs/replay/template.html` (shell, styles, 3D engine) or
  `docs/replay/paper.html` (report body), run `python scripts/build_replay.py`, then
  `python scripts/check_replay.py` and `python scripts/check_camera.py`. Do not edit
  `docs/replay/index.html`; it is generated.

## Style

Plain, readable Python in the style of the surrounding code: NumPy, dataclasses, docstrings
that explain *why*. Units are SI inside the library. Figures follow the conventions at the top
of `scripts/make_figures.py`.
