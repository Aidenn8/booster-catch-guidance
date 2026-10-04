#!/bin/bash
# Run the evaluation campaigns back to back (each resumable), through the thermal guard.
#   bash scripts/run_all.sh [workers]
# Settings chosen on the tuning set (results/mc_tune_v4.jsonl): MPC margin 8%, no approach
# cap; baseline a2 = 8 m/s^2. The capped MPC is flown as an ablation where it matters.
set -e
cd "$(dirname "$0")/.."
PY="${PY:-.venv/bin/python}"
W="${1:-4}"
G="$PY tools/thermal/guarded_run.py --"
$G $PY scripts/run_mc.py engineout --workers "$W" --controllers "mpc,baseline,mpc[approach_cap=1],mpc[eo_reserve=1]"
$G $PY scripts/run_mc.py wind --workers "$W" --controllers "mpc,baseline"
$G $PY scripts/run_mc.py basin --workers "$W" --controllers "mpc,baseline"
$G $PY scripts/run_mc.py random --n 300 --workers "$W" --controllers "mpc,baseline,mpc[approach_cap=1],mpc[eo_reserve=1]"
echo "all campaigns done"
