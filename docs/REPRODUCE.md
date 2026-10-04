# Reproducing the results

Everything runs on a laptop (developed on a fanless MacBook Air M2). Python 3.11+.

```bash
python -m venv .venv && source .venv/bin/activate
pip install numpy scipy cvxpy clarabel matplotlib pytest
python -m pytest tests/ -q                     # 14 tests, ~45 s
```

## Experiments

Each campaign writes one JSON line per flight to `results/` and can be resumed (finished
flights are skipped). The MPC takes ~10-20 s per flight in Python, the baseline ~0.1 s.

```bash
# tuning set (seed 99): every controller setting on the same 40 dispersed flights
python scripts/run_mc.py tune --n 40 --tag _v2 --controllers \
  "baseline[a2=6],baseline[a2=7],baseline[a2=8],baseline[a2=9],baseline[a2=10],\
mpc[margin=0.04],mpc[margin=0.08],mpc[margin=0.12],\
mpc[margin=0.04;approach_cap=0],mpc[margin=0.08;approach_cap=0],mpc[margin=0.12;approach_cap=0]"

# evaluation (tuned settings are the class defaults): engine-failure timing, wind, capture
# basin and the 300-flight Monte Carlo, all through the thermal guard (~1.5 h on 4 cores)
PY=.venv/bin/python bash scripts/run_all.sh 4

# what engine-out tolerance costs: 45 / 50 / 55 t of landing propellant (~17 min)
python scripts/run_mc.py eoprop --controllers "mpc[eo_reserve=1],mpc"

python scripts/summarize.py results/mc_random.jsonl     # outcome table with 95% intervals
python scripts/split_summary.py results/mc_random.jsonl # with / without engine failures
python scripts/validate_guidance.py                     # results/validation.json
python scripts/fuel_ablation.py                         # results/fuel_ablation.json
python scripts/make_figures.py                          # docs/images/*.png
python scripts/export_replay.py && python scripts/build_replay.py   # docs/replay/index.html
```

On a fanless laptop, run long campaigns through the thermal guard:

```bash
caffeinate -i -s bash tools/thermal/daemon.sh &        # writes tools/thermal/state every 20 s
python tools/thermal/guarded_run.py -- python scripts/run_mc.py random --n 300
```

The guard pauses the job (SIGSTOP) when macOS reports serious thermal pressure or the
battery passes 40 °C, resumes below 38 °C, and stops it at critical pressure or 45 °C.

## Debugging tools

* `scripts/debug_flight.py mpc|baseline '{kwargs}' [results.jsonl case_id]` - print a flight
* `scripts/replan_log.py results.jsonl case_id '{kwargs}'` - every MPC re-plan and why
* `scripts/probe_replan.py results.jsonl case_id` - stop at a failed re-plan, probe which
  constraint makes it infeasible
* `scripts/rerun_cases.py results.jsonl controller [case ids]` - re-fly cases after a change
