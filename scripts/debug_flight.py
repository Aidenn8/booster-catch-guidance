"""Print a flight trace (both controllers) to debug guidance problems."""
import sys
import warnings
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
warnings.filterwarnings("ignore")

import numpy as np  # noqa: E402

from boostercatch import controllers as C, sim, vehicle as V  # noqa: E402

veh, tw, sc = V.SUPER_HEAVY, V.TOWER, V.NOMINAL
which = sys.argv[1] if len(sys.argv) > 1 else "mpc"
import json
kw = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
ctl = C.ConvexMPC(**kw) if which == "mpc" else C.Baseline(**kw)
if which == "mpc":
    orig = ctl._replan

    def traced(t, r, v):
        before = ctl.plan
        orig(t, r, v)
        ok = ctl.plan is not before and ctl.plan is not None
        msg = (f"ok tf={ctl.plan.tf:.2f} gate_at={ctl.t_gate:.2f} fuel={ctl.plan.fuel / 1e3:.1f}t "
               f"d_hat={np.round(ctl.d_hat, 2)}") if ok else "FAILED"
        print(f"  replan t={t:5.2f} z={r[2]:7.1f} x={r[0]:7.1f} vz={v[2]:7.1f} vx={v[0]:6.1f} "
              f"m_est={ctl.m_est / 1e3:6.1f}t -> {msg}")
    ctl._replan = traced
disp = sim.Dispersion()
if len(sys.argv) > 3:   # replay a case from a results file: <file> <case_id>
    for line in open(sys.argv[3]):
        d = json.loads(line)
        if d["case_id"] == int(sys.argv[4]) and d["controller"] == which:
            disp = sim.Dispersion(**{k[2:]: v for k, v in d.items() if k.startswith("d_")})
    print(disp)
res = sim.simulate(ctl, veh, tw, sc, disp, record_every=25)
for t, r, v, m, T, n in zip(res.t, res.r, res.v, res.m, res.thrust, res.n_engines):
    tilt = np.degrees(np.arctan2(np.linalg.norm(T[:2]), T[2])) if T[2] > 0 else 90
    print(f"t={t:5.2f} z={r[2]:7.1f} x={r[0]:7.1f} y={r[1]:6.1f} vz={v[2]:7.2f} vx={v[0]:6.2f} "
          f"T={np.linalg.norm(T)/1e6:5.2f}MN n={n} tilt={tilt:4.1f} m={m/1e3:6.1f}t")
print(res.caught, res.reason, res.capture)
