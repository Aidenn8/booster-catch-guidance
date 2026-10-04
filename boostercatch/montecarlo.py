"""Monte Carlo campaigns: the same dispersed flights for both controllers.

Dispersion model (1-sigma unless noted; all public-estimate guesses, see REPORT.md):
  ignition position     20 m horizontal, 15 m vertical (boostback/coast error)
  ignition velocity     3 m/s horizontal, 5 m/s vertical
  mean wind             speed uniform in [0, wind_max], direction uniform
  gusts                 OU process, sigma = 0.5 + 0.15 x mean wind, 3 s correlation
  drag coefficient      x U(0.8, 1.2)
  thrust                bias N(0, 2%) clipped to +-5%
  Isp                   x N(1, 0.5%)
  dry mass              x N(1, 2%)
  engine failure        probability p_engine_out, at a uniform time in [0, 15] s
  navigation noise      0.3 m position, 0.1 m/s velocity, white
"""
import json
import os
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict

import numpy as np

from . import controllers as C
from . import sim
from .vehicle import NOMINAL, SUPER_HEAVY, TOWER

CONTROLLERS = {"mpc": C.ConvexMPC, "baseline": C.Baseline}


def make_controller(spec):
    """'mpc', 'baseline' or a variant such as 'mpc[margin=0.12,approach_cap=0]'."""
    name, _, rest = spec.partition("[")
    kw = {}
    for item in filter(None, rest.rstrip("]").split(",")):
        k, v = item.split("=")
        kw[k] = bool(int(v)) if k in ("approach_cap", "eo_reserve") else float(v)
    return CONTROLLERS[name](**kw)


def sample(rng, wind_max=15.0, p_engine_out=0.2, seed=0):
    speed = rng.uniform(0, wind_max)
    ang = rng.uniform(0, 2 * np.pi)
    eo = float(rng.uniform(0, 15.0)) if rng.uniform() < p_engine_out else None
    return sim.Dispersion(
        dr0=tuple(rng.normal(0, [20.0, 20.0, 15.0])),
        dv0=tuple(rng.normal(0, [3.0, 3.0, 5.0])),
        wind_mean=(speed * np.cos(ang), speed * np.sin(ang)),
        gust_sigma=0.5 + 0.15 * speed,
        cd_scale=float(rng.uniform(0.8, 1.2)),
        thrust_bias=float(np.clip(rng.normal(0, 0.02), -0.05, 0.05)),
        isp_scale=float(rng.normal(1, 0.005)),
        dry_mass_scale=float(rng.normal(1, 0.02)),
        engine_out_time=eo,
        nav_sigma_r=0.3, nav_sigma_v=0.1,
        seed=seed,
    )


def run_case(job):
    """job = (controller key, Dispersion as dict, extra tags). Returns a flat summary dict."""
    key, disp_d, tags = job
    disp = sim.Dispersion(**disp_d)
    ctl = make_controller(key)
    t0 = time.time()
    try:
        res = sim.simulate(ctl, SUPER_HEAVY, TOWER, NOMINAL, disp, record_every=10)
        out = {"caught": bool(res.caught), "reason": res.reason, "pad_safe": bool(res.pad_safe),
               "impact": res.impact, "abort_reason": res.abort_reason, "abort_time": res.abort_time,
               **res.capture,
               "t_end": float(res.t[-1]) if len(res.t) else 0.0,
               "min_tower_clearance": float(_tower_clearance(res.r)) if len(res.r) else None,
               "fail_times": getattr(ctl, "fail_times", None)}
    except Exception as e:  # a crash in the controller is a failure, recorded as such
        out = {"caught": False, "reason": f"error: {type(e).__name__}: {e}", "pad_safe": False}
    out.update({"controller": key, "wall_s": time.time() - t0,
                "n_solves": getattr(ctl, "n_solves", None), "failed_replans": getattr(ctl, "failed_replans", None),
                "recoveries": getattr(ctl, "recoveries", None),
                **{f"d_{k}": v for k, v in disp_d.items()}, **tags})
    return out


def _tower_clearance(r):
    """Closest horizontal distance from the booster's skin to the tower's front face."""
    below = r[:, 2] < 146.0
    if not below.any():
        return np.inf
    return float(np.min(9.0 - (r[below, 0] + 4.5)))


def run_jobs(jobs, out_path, workers=4):
    """Run jobs in a process pool, appending one JSON line per finished case."""
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    done = set()
    if os.path.exists(out_path):
        with open(out_path) as f:
            for line in f:
                d = json.loads(line)
                done.add((d["controller"], d["case_id"]))
    todo = [j for j in jobs if (j[0], j[2]["case_id"]) not in done]
    print(f"{len(jobs)} jobs, {len(jobs) - len(todo)} already done, running {len(todo)} on {workers} workers",
          flush=True)
    t0 = time.time()
    with ProcessPoolExecutor(max_workers=workers) as ex, open(out_path, "a") as f:
        for i, out in enumerate(ex.map(run_case, todo, chunksize=1)):
            f.write(json.dumps(out, default=float) + "\n")
            f.flush()
            if (i + 1) % 10 == 0 or i + 1 == len(todo):
                print(f"  {i + 1}/{len(todo)}  {time.time() - t0:.0f}s", flush=True)


def random_campaign(n, seed=2026, controllers=("mpc", "baseline"), **kw):
    rng = np.random.default_rng(seed)
    jobs = []
    for i in range(n):
        d = asdict(sample(rng, seed=int(seed * 1000 + i), **kw))
        for key in controllers:
            jobs.append((key, d, {"case_id": i}))
    return jobs
