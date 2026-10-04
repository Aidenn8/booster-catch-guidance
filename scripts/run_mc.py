"""Run a Monte Carlo campaign (both controllers, same dispersed cases).

    python scripts/run_mc.py random  --n 300 --workers 4      results/mc_random.jsonl
    python scripts/run_mc.py pilot   --n 16                   results/mc_pilot.jsonl
    python scripts/run_mc.py engineout                         results/mc_engineout.jsonl
    python scripts/run_mc.py basin                             results/mc_basin.jsonl
    python scripts/run_mc.py wind                              results/mc_wind.jsonl

Resumable: finished cases already in the output file are skipped.
Run it through tools/thermal/guarded_run.py on a fanless laptop.
"""
import argparse
import os
import sys
import warnings
from dataclasses import asdict
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
warnings.filterwarnings("ignore")

import numpy as np  # noqa: E402

from boostercatch import ROOT, montecarlo as MC, sim  # noqa: E402

RESULTS = ROOT / "results"


def engineout_jobs(ctls):
    """Engine failure at each time from ignition to the gate, otherwise nominal (calm air,
    no other errors): where in the burn can each controller survive losing an engine?"""
    jobs = []
    times = [None] + list(np.round(np.arange(0.0, 16.01, 0.5), 2))
    for i, t in enumerate(times):
        d = asdict(sim.Dispersion(engine_out_time=None if t is None else float(t)))
        for key in ctls:
            jobs.append((key, d, {"case_id": i, "engine_out_time": t}))
    return jobs


def basin_jobs(ctls):
    """Capture basin: shift the ignition point downrange (x) and in altitude (z), calm air.
    The ignition velocity is kept, so a higher ignition point means more time to brake."""
    jobs = []
    i = 0
    for dz in np.arange(-300, 301, 50):
        for dx in np.arange(-200, 201, 40):
            d = asdict(sim.Dispersion(dr0=(float(dx), 0.0, float(dz))))
            for key in ctls:
                jobs.append((key, d, {"case_id": i, "dx": float(dx), "dz": float(dz)}))
            i += 1
    return jobs


def wind_jobs(ctls, n_per=12, seed=7):
    """Wind sweep: mean wind 0-30 m/s with gusts and the usual small errors, no engine failures."""
    rng = np.random.default_rng(seed)
    jobs = []
    i = 0
    for speed in np.arange(0, 30.1, 5):
        for _ in range(n_per):
            d = MC.sample(rng, wind_max=0.0, p_engine_out=0.0, seed=int(seed * 1000 + i))
            ang = rng.uniform(0, 2 * np.pi)
            d.wind_mean = (float(speed * np.cos(ang)), float(speed * np.sin(ang)))
            d.gust_sigma = 0.5 + 0.15 * float(speed)
            dd = asdict(d)
            for key in ctls:
                jobs.append((key, dd, {"case_id": i, "wind_speed": float(speed)}))
            i += 1
    return jobs


def eoprop_jobs(ctls, n=40, seed=31):
    """How much landing propellant makes the engine-out reserve robust? The same dispersed
    flights (full dispersion model) at 45, 50 and 55 t of landing propellant, half of them
    with an engine failure at a random time, half without."""
    rng = np.random.default_rng(seed)
    base = []
    for i in range(n):
        d = MC.sample(rng, p_engine_out=1.0 if i % 2 == 0 else 0.0, seed=int(seed * 1000 + i))
        base.append(asdict(d))
    jobs = []
    for j, prop in enumerate((45e3, 50e3, 55e3)):
        for i, d in enumerate(base):
            for key in ctls:
                jobs.append((key, d, {"case_id": j * 1000 + i, "flight": i, "landing_propellant": prop}))
    return jobs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("campaign", choices=["pilot", "random", "engineout", "basin", "wind", "tune", "eoprop"])
    ap.add_argument("--controllers", default="mpc,baseline",
                    help="comma-separated; variants as name[k=v;k=v], e.g. mpc[margin=0.12]")
    ap.add_argument("--n", type=int, default=300)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--tag", default="", help="suffix for the output file name")
    a = ap.parse_args()
    ctls = [c.replace(";", ",") for c in a.controllers.split(",")]
    MC.CAMPAIGN_CONTROLLERS = ctls
    if a.campaign in ("pilot", "random", "tune"):
        seed = {"pilot": 11, "random": 2026, "tune": 99}[a.campaign]
        jobs = MC.random_campaign(a.n, seed=seed, controllers=ctls)
    else:
        jobs = {"engineout": engineout_jobs, "basin": basin_jobs, "wind": wind_jobs,
                "eoprop": eoprop_jobs}[a.campaign](ctls)
    MC.run_jobs(jobs, str(RESULTS / f"mc_{a.campaign}{a.tag}.jsonl"), workers=a.workers)


if __name__ == "__main__":
    main()
