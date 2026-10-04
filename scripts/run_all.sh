#!/bin/bash
# Run the evaluation campaigns back to back (each resumable), through the thermal guard.
#   bash scripts/run_all.sh "<mpc spec>,<baseline spec>[,...]" [workers]
set -e
cd "$(dirname "$0")/.."
PY="${PY:-.venv/bin/python}"
CTLS="$1"
W="${2:-4}"
G="$PY tools/thermal/guarded_run.py --"
$G $PY scripts/run_mc.py engineout --workers "$W" --controllers "$CTLS"
$G $PY scripts/run_mc.py wind --workers "$W" --controllers "$CTLS"
$G $PY scripts/run_mc.py basin --workers "$W" --controllers "$CTLS"
$G $PY scripts/run_mc.py random --n 300 --workers "$W" --controllers "$CTLS"
echo "all campaigns done"
