"""Log every MPC re-plan of one case: which margin / gate shift worked, and the deviation
from the previous plan (to see what eats the thrust margin).

    python scripts/replan_log.py results/mc_pilot_v3.jsonl 0
"""
import json
import sys
import warnings
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
warnings.filterwarnings("ignore")

import numpy as np  # noqa: E402

from boostercatch import controllers as C, sim  # noqa: E402
from boostercatch.vehicle import NOMINAL, SUPER_HEAVY, TOWER  # noqa: E402

disp = sim.Dispersion()
if len(sys.argv) > 2:
    for line in open(sys.argv[1]):
        d = json.loads(line)
        if d["case_id"] == int(sys.argv[2]) and d["controller"].startswith("mpc"):
            disp = sim.Dispersion(**{k[2:]: v for k, v in d.items() if k.startswith("d_")})
print(f"thrust bias {disp.thrust_bias:+.3f}  dry mass x{disp.dry_mass_scale:.3f}  cd x{disp.cd_scale:.2f}  "
      f"isp x{disp.isp_scale:.4f}  wind {np.round(disp.wind_mean, 1)}  engine out {disp.engine_out_time}")
kw = json.loads(sys.argv[3]) if len(sys.argv) > 3 else {}
ctl = C.ConvexMPC(**kw)
orig_solve, orig_replan = ctl._solve, ctl._replan
attempts = []


def solve(t, r, v, tf, margin, gate_speed_max=None):
    p = orig_solve(t, r, v, tf, margin, gate_speed_max)
    attempts.append((tf, margin, gate_speed_max, p is not None))
    return p


def replan(t, r, v):
    attempts.clear()
    old = ctl.plan
    dev = ""
    if old is not None:
        tau = t - ctl.plan_t0
        rr = np.array([np.interp(tau, old.t, old.r[:, i]) for i in range(3)])
        vv = np.array([np.interp(tau, old.t, old.v[:, i]) for i in range(3)])
        dev = f"dev r={np.round(r - rr, 1)} v={np.round(v - vv, 2)}"
    orig_replan(t, r, v)
    ok = [a for a in attempts if a[3]]
    used = f"margin={ok[0][1]:.2f} tf={ok[0][0]:.2f} gate<={ok[0][2]}" if ok else ("initial" if old is None else "FAILED")
    print(f"t={t:5.2f} z={r[2]:6.1f} vz={v[2]:7.2f} m_est={ctl.m_est / 1e3:5.1f}t d_hat={np.round(ctl.d_hat, 2)} "
          f"{dev} -> {used} ({len(attempts)} solves)")


ctl._solve, ctl._replan = solve, replan
res = sim.simulate(ctl, SUPER_HEAVY, TOWER, NOMINAL, disp)
print(res.reason, {k: round(v, 2) for k, v in res.capture.items()})
