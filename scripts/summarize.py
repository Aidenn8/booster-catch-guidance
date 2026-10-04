"""Summarize a Monte Carlo results file: outcomes per controller, failure reasons, fuel.

    python scripts/summarize.py results/mc_random.jsonl [--cases]
"""
import json
import sys
from collections import Counter, defaultdict

import numpy as np


def wilson(k, n, z=1.96):
    """95% Wilson score interval for a binomial proportion."""
    if n == 0:
        return (0.0, 0.0)
    p = k / n
    den = 1 + z ** 2 / n
    c = (p + z ** 2 / (2 * n)) / den
    h = z * np.sqrt(p * (1 - p) / n + z ** 2 / (4 * n ** 2)) / den
    return (c - h, c + h)


def outcome(d):
    if d["caught"]:
        return "caught"
    return "safe miss" if d.get("pad_safe") else "pad strike"


def summarize(rows):
    by = defaultdict(list)
    for d in rows:
        by[d["controller"]].append(d)
    out = {}
    for key, rs in sorted(by.items()):
        n = len(rs)
        oc = Counter(outcome(d) for d in rs)
        prop = [d["propellant_left"] for d in rs if d["caught"]]
        lat = [d["lateral_error"] for d in rs if d["caught"]]
        out[key] = {
            "n": n,
            **{k: oc.get(k, 0) for k in ("caught", "safe miss", "pad strike")},
            "catch_ci": wilson(oc.get("caught", 0), n),
            "strike_ci": wilson(oc.get("pad strike", 0), n),
            "prop_median_t": float(np.median(prop) / 1e3) if prop else None,
            "prop_p05_t": float(np.percentile(prop, 5) / 1e3) if prop else None,
            "lat_median_m": float(np.median(lat)) if lat else None,
            "reasons": Counter(d["reason"].split(";")[0] for d in rs if not d["caught"]).most_common(),
            "wall_mean_s": float(np.mean([d["wall_s"] for d in rs])),
        }
    return out


def main():
    rows = [json.loads(line) for line in open(sys.argv[1])]
    for key, s in summarize(rows).items():
        lo, hi = s["catch_ci"]
        print(f"{key:9s} n={s['n']:4d} caught {s['caught']:4d} ({100 * s['caught'] / s['n']:.1f}%, "
              f"95% CI {100 * lo:.1f}-{100 * hi:.1f})  safe miss {s['safe miss']:3d}  pad strike {s['pad strike']:3d}  "
              f"| propellant left median {s['prop_median_t']} t, 5th pct {s['prop_p05_t']} t | "
              f"lateral median {s['lat_median_m']} m | {s['wall_mean_s']:.1f} s/flight")
        for r, c in s["reasons"]:
            print(f"      {c:4d}  {r}")
    if "--cases" in sys.argv:
        rows.sort(key=lambda d: (d["case_id"], d["controller"]))
        for d in rows:
            if not d["caught"]:
                print(d["case_id"], d["controller"], d["reason"], "eo", d.get("d_engine_out_time"),
                      "impact", d.get("impact"), "fails", d.get("fail_times"))


if __name__ == "__main__":
    main()
