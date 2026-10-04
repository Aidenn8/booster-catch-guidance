"""Outcomes split by whether an engine failed (and when), per controller variant.

    python scripts/split_summary.py results/mc_tune.jsonl
"""
import json
import sys
from collections import defaultdict

import numpy as np

sys.path.insert(0, __import__("os").path.dirname(__file__))
from summarize import outcome  # noqa: E402

rows = [json.loads(line) for line in open(sys.argv[1])]
by = defaultdict(list)
for d in rows:
    by[d["controller"]].append(d)
print(f"{'controller':34s} {'no engine failure':>34s} | {'engine failure':>30s}")
for key in sorted(by):
    rs = by[key]
    ne = [d for d in rs if d["d_engine_out_time"] is None]
    eo = [d for d in rs if d["d_engine_out_time"] is not None]

    def fmt(group):
        if not group:
            return "-"
        k = sum(d["caught"] for d in group)
        s = sum(outcome(d) == "pad strike" for d in group)
        prop = [d["propellant_left"] / 1e3 for d in group if d["caught"]]
        return f"{k:2d}/{len(group):2d} caught, {s} strike, fuel {np.median(prop) if prop else float('nan'):.2f} t"
    print(f"{key:34s} {fmt(ne):>34s} | {fmt(eo):>30s}")
    miss = [d for d in ne if not d["caught"]]
    for d in miss:
        print(f"      no-EO miss case {d['case_id']}: {d['reason'][:90]}")
eo_cases = sorted({(d["case_id"], round(d["d_engine_out_time"], 1)) for d in rows if d["d_engine_out_time"] is not None})
print("engine-failure cases (case, time):", eo_cases)
for cid, te in eo_cases:
    line = [f"{key.replace('baseline', 'B').replace('mpc', 'M')}:{'C' if d['caught'] else ('s' if d['pad_safe'] else 'X')}"
            for key in sorted(by) for d in by[key] if d["case_id"] == cid]
    print(f"  case {cid:2d} t={te:5.1f}: " + " ".join(line))
