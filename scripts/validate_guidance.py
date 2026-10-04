"""Numbers behind the guidance validation (the tests assert them; this prints them).

    python scripts/validate_guidance.py      -> results/validation.json
"""
import json
import sys
import time
import warnings
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tests"))
warnings.filterwarnings("ignore")

import numpy as np  # noqa: E402
from scipy.optimize import brentq  # noqa: E402

import test_guidance as T  # noqa: E402
from boostercatch import ROOT, guidance as G, sim  # noqa: E402
from boostercatch.vehicle import G0, NOMINAL, SUPER_HEAVY, TOWER, Tower  # noqa: E402

veh, out = SUPER_HEAVY, {}

# 1. nominal plan: lossless gap, solve time
t0 = time.time()
p = G.solve_fixed(veh, TOWER, np.array(NOMINAL.r0), np.array(NOMINAL.v0), veh.wet_mass, 14.0,
                  switch_time=NOMINAL.switch_time)
out["nominal_solve_s"] = time.time() - t0
out["nominal_rel_lossless_gap"] = p.lossless_gap / float(np.max(p.sigma))

# 2. vertical descent vs shooting
tower, r0, v0 = T._vertical_problem()
m0 = 305e3
kw = dict(switch_time=None, engines=(3, 3), use_offshore=False, use_glideslope=False, final_align=False, n_nodes=120)
pc = G.solve_earliest(veh, tower, r0, v0, m0, tf_lo=2.0, tf_hi=30.0, tol=0.02, drag=False, **kw)
ts = brentq(lambda s: T._shoot(veh, r0[2], v0[2], m0, s, 3)[0], 0.0, 5.0, xtol=1e-6)
_, m_end, t_end = T._shoot(veh, r0[2], v0[2], m0, ts, 3)
out["vertical"] = {"convex_fuel_kg": pc.fuel, "shooting_fuel_kg": m0 - m_end, "convex_tf": pc.tf, "shooting_tf": t_end,
                   "rel_fuel_diff": (pc.fuel - (m0 - m_end)) / (m0 - m_end)}

# 3. SLSQP on the non-convex problem, free final time, three different starting guesses
r0, v0, m0, N = np.array([-60.0, 20.0, 700.0]), np.array([12.0, -3.0, -85.0]), 310e3, 16
kw = dict(switch_time=None, engines=(3, 3), use_offshore=False, use_glideslope=False, final_align=False)
t0 = time.time()
tower = Tower(catch_point=(0.0, 0.0, 0.0), gate_height=0.0, standoff=0.0, commit_time=0.0,
              final_descent_rate=2.0, terminal_decel=1.0)
pc = G.solve_earliest(veh, tower, r0, v0, m0, tf_lo=2.0, tf_hi=40.0, tol=0.005, drag=False, n_nodes=N, **kw)
t_cvx = time.time() - t0
starts = {"gentle braking, tf 14 s": (np.tile([0.0, 0.0, 1.2 * G0], N), 14.0),
          "hover thrust, tf 18 s": (np.tile([0.0, 0.0, 1.0 * G0], N), 18.0),
          "random thrust, tf 12 s": (np.random.default_rng(1).normal([0, 0, 14], [1, 1, 3], (N, 3)).ravel(), 12.0)}
out["nlp"] = {"convex_tf": pc.tf, "convex_fuel_kg": pc.fuel, "convex_s": t_cvx, "slsqp": {}}
for name, (x0, tf0) in starts.items():
    t0 = time.time()
    res, tf_n, f_n, _ = T.nlp_free_time(r0, v0, m0, N, x0, tf0)
    out["nlp"]["slsqp"][name] = {"tf": float(tf_n), "fuel_kg": float(f_n), "iterations": int(res.nit),
                                 "success": bool(res.success), "seconds": time.time() - t0,
                                 "rel_fuel_diff_convex_minus_nlp": float((pc.fuel - f_n) / f_n)}

# 4. simulator vs plan
r0, v0 = np.array(NOMINAL.r0), np.array(NOMINAL.v0)
tf = G.solve_earliest(veh, TOWER, r0, v0, veh.wet_mass, switch_time=NOMINAL.switch_time).tf + 0.5
pp = G.solve_with_drag(veh, TOWER, r0, v0, veh.wet_mass, tf, passes=3, n_nodes=80, switch_time=NOMINAL.switch_time)
rs = sim.simulate(T._OpenLoop(pp), veh, TOWER, NOMINAL, sim.Dispersion(tau_lag=0.0), t_max=pp.tf, record_every=1)
i = int(np.argmin(np.abs(rs.t - pp.tf)))
out["sim_vs_plan"] = {"pos_err_m": float(np.linalg.norm(rs.r[i] - pp.r[-1])),
                      "vel_err_ms": float(np.linalg.norm(rs.v[i] - pp.v[-1])), "mass_err_kg": float(abs(rs.m[i] - pp.m[-1]))}
json.dump(out, open(ROOT / "results" / "validation.json", "w"), indent=1, default=float)
print(json.dumps(out, indent=1, default=float))
