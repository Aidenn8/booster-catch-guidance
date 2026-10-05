# Reproducing the results

Everything here runs on a laptop; the study was produced on a fanless MacBook Air (M2).
The committed `results/` files are the exact data behind the report, so you can regenerate
the tables and figures without re-running any simulation, or re-run everything from scratch.

- [Setup](#setup)
- [Regenerate tables and figures from the committed results](#regenerate-tables-and-figures-from-the-committed-results)
- [Re-run the campaigns from scratch](#re-run-the-campaigns-from-scratch)
- [Expected numbers](#expected-numbers)
- [The replay page](#the-replay-page)
- [Running on a fanless laptop: the thermal guard](#running-on-a-fanless-laptop-the-thermal-guard)
- [Debugging tools](#debugging-tools)
- [Troubleshooting](#troubleshooting)

## Setup

Python 3.11 or newer.

```bash
git clone https://github.com/Aidenn8/booster-catch-guidance.git
cd booster-catch-guidance
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python -m pytest tests/ -q          # 14 tests, about a minute
```

Dependencies: NumPy, SciPy, CVXPY, Clarabel (the conic solver), Matplotlib and pytest. The
optional browser checks of the replay page also need Playwright
(`pip install -r requirements-dev.txt && python -m playwright install chromium`).

## Regenerate tables and figures from the committed results

These read `results/` and take seconds to a couple of minutes:

```bash
python scripts/summarize.py results/mc_random.jsonl        # outcome table, 95% intervals, miss reasons
python scripts/split_summary.py results/mc_random.jsonl    # split by engine failure
python scripts/summarize.py results/mc_tune_v4.jsonl       # tuning set (any results file works)
python scripts/validate_guidance.py                        # verification numbers -> results/validation.json
python scripts/fuel_ablation.py                            # cost of each robustness feature -> results/fuel_ablation.json
python scripts/make_figures.py                             # every figure -> docs/images/*.png
```

`make_figures.py` also flies the two nominal flights (about a minute) for the trajectory
figure. Figures use the bundled Inter font in `tools/fonts/`.

## Re-run the campaigns from scratch

Each campaign writes one JSON line per flight to `results/mc_<campaign><tag>.jsonl` and is
resumable: re-running the same command skips flights already in the file. To start fresh,
move or delete the file first. Timings are for 4 worker processes on an M2; the MPC takes
about 10–20 s per flight in Python, the baseline about 0.1 s.

| Campaign | Command | Output | Time |
|---|---|---|---|
| Tuning set (seed 99) | see below | `results/mc_tune_v4.jsonl` | ~30 min |
| Engine-failure timing | `python scripts/run_mc.py engineout --controllers "mpc,baseline,mpc[approach_cap=1],mpc[eo_reserve=1]"` | `results/mc_engineout.jsonl` | ~10 min |
| Crosswind sweep | `python scripts/run_mc.py wind --controllers "mpc,baseline"` | `results/mc_wind.jsonl` | ~5 min |
| Capture basin | `python scripts/run_mc.py basin --controllers "mpc,baseline"` | `results/mc_basin.jsonl` | ~10 min |
| Main Monte Carlo (seed 2026) | `python scripts/run_mc.py random --n 300 --controllers "mpc,baseline,mpc[approach_cap=1],mpc[eo_reserve=1]"` | `results/mc_random.jsonl` | ~1 h |
| Engine-out propellant study | `python scripts/run_mc.py eoprop --controllers "mpc[eo_reserve=1],mpc"` | `results/mc_eoprop.jsonl` | ~17 min |

Add `--workers N` to change the number of processes (default 4).

**Tuning set.** Every controller setting on the same 40 dispersed flights. With today's
defaults (no approach cap), the capped settings must say so explicitly:

```bash
python scripts/run_mc.py tune --n 40 --tag _v4 --controllers \
  "baseline[a2=6],baseline[a2=7],baseline[a2=8],baseline[a2=9],baseline[a2=10],\
mpc[margin=0.04],mpc[margin=0.08],mpc[margin=0.12],\
mpc[margin=0.04;approach_cap=1],mpc[margin=0.08;approach_cap=1],mpc[margin=0.12;approach_cap=1]"
```

The committed `mc_tune_v4.jsonl` was flown when the cap was on by default, so its keys
are named the other way round (see the note in [DATA.md](DATA.md#campaign-files-resultsmc_jsonl));
the figure script reads both.

**All four evaluation campaigns in one go**, through the thermal guard (macOS):

```bash
PY=.venv/bin/python bash scripts/run_all.sh 4
```

On other systems, or without the guard, run the four commands from the table directly.

## Expected numbers

The campaigns are deterministic given their seeds; re-running them should reproduce these
(small differences can appear across solver versions, since a plan that sits exactly on a
feasibility boundary can flip):

| File | Check |
|---|---|
| `mc_random.jsonl` | `mpc` 259/300 caught, 21 safe miss, 20 pad strike; `baseline` 219 / 73 / 8 |
| `mc_engineout.jsonl` | `mpc[eo_reserve=1]` 32 of 33 failure times caught; `baseline` 4 |
| `mc_wind.jsonl` | `mpc` 12/12 caught up to 20 m/s; `baseline` 8/12 at 20 m/s |
| `mc_basin.jsonl` | `mpc` 127/143 caught, 0 pad strikes; `baseline` 86/143, 25 pad strikes |
| `mc_eoprop.jsonl` | `mpc[eo_reserve=1]` at 50 t: 20/20 without and 19/20 with an engine failure |
| nominal flight | MPC caught 0.01 m from the catch point, 5.48 t left, at 17.1 s; baseline 0.45 m, 3.93 t, 18.9 s |

## The replay page

```bash
python scripts/export_replay.py     # fly the six showcase scenarios -> docs/replay/flights.json (~5 min)
python scripts/build_replay.py      # assemble docs/replay/index.html
```

`build_replay.py` inlines the report body (`docs/replay/paper.html`), the flight data and
the figures into one self-contained HTML file. See [REPLAY.md](REPLAY.md) for how the page
works and how it is tested.

## Running on a fanless laptop: the thermal guard

Long campaigns can drive a fanless laptop into thermal throttling. `tools/thermal/` keeps
them in check on macOS:

```bash
swiftc tools/thermal/thermalstate.swift -o tools/thermal/thermalstate   # once: reads macOS thermal pressure
caffeinate -i -s bash tools/thermal/daemon.sh &                         # checks every 20 s, writes tools/thermal/state
python tools/thermal/guarded_run.py -- python scripts/run_mc.py random --n 300
```

`guarded_run.py` starts the job in its own process group at reduced priority and registers it
with the daemon. The daemon freezes the job (SIGSTOP) when macOS reports serious thermal
pressure or the battery reaches 40 °C, resumes it below 38 °C, and terminates it at critical
pressure or 45 °C. The stop is sticky: delete `tools/thermal/state` to allow jobs again.
Thresholds can be changed with `PAUSE_C`, `RESUME_C` and `STOP_C`.

## Debugging tools

Used while developing the controllers; each prints to the terminal.

| Command | What it shows |
|---|---|
| `python scripts/debug_flight.py mpc '{"margin": 0.08}'` | A flight trace for one controller (constructor options as JSON) |
| `python scripts/replan_log.py results/mc_random.jsonl 17` | Every MPC re-plan of case 17: which margin and gate shift worked, and how far each plan moved |
| `python scripts/probe_replan.py results/mc_random.jsonl 17` | Fly case 17 until the first failed re-plan, then probe which constraint makes it infeasible |
| `python scripts/rerun_cases.py results/mc_random.jsonl mpc 5 17` | Re-fly selected cases in-process after a code change |
| `python scripts/drop_rows.py mpc results/mc_basin.jsonl` | Remove one controller family's rows so the resumable runner re-flies them |

## Troubleshooting

* **`SolverError` or `Clarabel not installed`.** Install the solver: `pip install clarabel`. CVXPY
  picks it by name; no other solver is needed.
* **Campaigns are slow.** The MPC solves a cone program every second of simulated flight. Use
  more workers (`--workers 8`) on a machine with more cores. `OMP_NUM_THREADS=1` is set by
  `run_mc.py` so worker processes do not oversubscribe the CPU.
* **A campaign was interrupted.** Run the same command again; finished flights are skipped.
* **The replay page is blank.** It needs WebGL and loads three.js and MathJax from public CDNs,
  so it must be opened in a browser with network access.
