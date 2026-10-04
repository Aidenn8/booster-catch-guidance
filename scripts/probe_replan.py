"""Debug a failed MPC re-plan: fly a case, stop at the first failure, probe the solver.

    python scripts/probe_replan.py results/mc_pilot_v2.jsonl 0
"""
import json
import sys
import warnings
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
warnings.filterwarnings("ignore")

import cvxpy as cp  # noqa: E402
import numpy as np  # noqa: E402

from boostercatch import controllers as C, guidance as G, sim  # noqa: E402
from boostercatch.vehicle import NOMINAL, SUPER_HEAVY, TOWER  # noqa: E402

path, case = sys.argv[1], int(sys.argv[2])
for line in open(path):
    d = json.loads(line)
    if d["case_id"] == case and d["controller"].startswith("mpc"):
        disp = sim.Dispersion(**{k[2:]: v for k, v in d.items() if k.startswith("d_")})


class Stop(Exception):
    pass


ctl = C.ConvexMPC(**(json.loads(sys.argv[3]) if len(sys.argv) > 3 else {}))
captured = {}
orig_abort = ctl.abort


def on_abort(t, reason):
    raise Stop


ctl.abort = on_abort
orig_replan = ctl._replan


def replan(t, r, v):
    captured.update(t=t, r=r.copy(), v=v.copy(), m=ctl.m_eff, k_hat=ctl.k_hat, t_gate=ctl.t_gate, d_hat=ctl.d_hat.copy(),
                    eng=ctl.engines_for_planning(t), plan=ctl.plan, plan_t0=ctl.plan_t0)
    orig_replan(t, r, v)


ctl._replan = replan
try:
    sim.simulate(ctl, SUPER_HEAVY, TOWER, NOMINAL, disp)
    print("no failure")
    sys.exit()
except Stop:
    pass
c = captured
t, r, v, m = c["t"], c["r"], c["v"], c["m"]
remaining = c["t_gate"] - t
print(f"failure at t={t:.2f}: r={np.round(r, 1)} v={np.round(v, 2)} m_est={m / 1e3:.1f}t remaining={remaining:.2f}s "
      f"engines={c['eng']} d_hat={np.round(c['d_hat'], 2)} k_hat={c['k_hat']:.4f}")
p = c["plan"]
tau = t - c["plan_t0"]
rr = np.array([np.interp(tau, p.t, p.r[:, i]) for i in range(3)])
vv = np.array([np.interp(tau, p.t, p.v[:, i]) for i in range(3)])
print(f"old plan at this time: r={np.round(rr, 1)} v={np.round(vv, 2)}  (deviation {np.round(r - rr, 1)}, {np.round(v - vv, 2)})")

resid = ctl._residual()
for label, kw in [("as flown", dict(margin=0.0, extra_accel=resid)),
                  ("no residual", dict(margin=0.0)),
                  ("no offshore", dict(margin=0.0, extra_accel=resid, use_offshore=False)),
                  ("no glideslope", dict(margin=0.0, extra_accel=resid, use_glideslope=False)),
                  ("13 engines all the way", dict(margin=0.0, extra_accel=resid, engines=(13, 13))),
                  ("recovery gate 12", dict(margin=0.0, extra_accel=resid, gate_speed_max=12.0)),
                  ("recovery gate 30", dict(margin=0.0, extra_accel=resid, gate_speed_max=30.0)),
                  ("no final-align", dict(margin=0.0, extra_accel=resid, final_align=False)),
                  ("no thrust-floor check", dict(margin=0.0, extra_accel=resid, check_floor=False))]:
    res = []
    for tf in np.arange(max(remaining - 2, 1.0), remaining + 10.01, 1.0):
        kw2 = dict(n_nodes=32, switch_time=NOMINAL.switch_time, engines=c["eng"], elapsed=t, total_tf=t + tf)
        kw2.update(kw)
        pl = G.solve_with_drag(SUPER_HEAVY, TOWER, r, v, m, tf, passes=1, **kw2)
        res.append(f"{tf:.1f}:{'ok' if pl is not None else '--'}")
    print(f"{label:24s}", " ".join(res))

# raw solver status for the as-flown problem at the current gate time (without the
# physical-thrust rejection) to see whether the SOCP itself is infeasible
orig_solve = cp.Problem.solve
status = []


def spy(self, *a, **k):
    out = orig_solve(self, *a, **k)
    status.append(self.status)
    return out


cp.Problem.solve = spy
G.solve_fixed(SUPER_HEAVY, TOWER, r, v, m, remaining, n_nodes=32, switch_time=NOMINAL.switch_time, engines=c["eng"],
              elapsed=t, total_tf=t + remaining, extra_accel=resid)
print("raw solver status at the current gate time:", status)

# what did the old plan assume from here on?
k0 = int(np.searchsorted(p.t, tau))
lo, hi = SUPER_HEAVY.thrust_bounds(c["eng"][1])
print(f"old plan: tf={p.tf:.2f}, gate velocity {np.round(p.v[-1], 2)}, plan mass here {np.interp(tau, p.t, p.m) / 1e3:.1f}t "
      f"vs m_est {m / 1e3:.1f}t, recoveries so far {ctl.recoveries}")
print("old plan thrust / max from here:", np.round(np.linalg.norm(p.thrust[k0:], axis=1) / hi, 3))
acc = p.accel[k0:]
print("old plan tilt (deg):", np.round(np.degrees(np.arctan2(np.linalg.norm(acc[:, :2], axis=1), acc[:, 2])), 1))
