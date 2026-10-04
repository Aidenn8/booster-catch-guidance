"""Nominal-flight fuel for each guidance setting: what does each robustness feature cost?

    python scripts/fuel_ablation.py
"""
import json
import sys
import warnings
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
warnings.filterwarnings("ignore")

import numpy as np  # noqa: E402

from boostercatch import ROOT, controllers as C, guidance as G, sim  # noqa: E402
from boostercatch.vehicle import NOMINAL, SUPER_HEAVY, TOWER  # noqa: E402

veh, tw, sc = SUPER_HEAVY, TOWER, NOMINAL
r0, v0 = np.array(sc.r0), np.array(sc.v0)
rows = []

print("open-loop plans to the gate (no disturbances):")
for label, kw in [("fuel-optimal, no margins", dict(margin=0.0, approach_cap=False)),
                  ("+ approach cap 10 m/s", dict(margin=0.0)),
                  ("+ 8% thrust margin", dict(margin=0.08)),
                  ("8% margin, no approach cap", dict(margin=0.08, approach_cap=False))]:
    p = G.solve_earliest(veh, tw, r0, v0, veh.wet_mass, switch_time=sc.switch_time, **kw)
    print(f"  {label:32s} tf={p.tf:5.2f}s fuel to gate {p.fuel / 1e3:5.2f} t  lossless gap {p.lossless_gap:.1e}")
    rows.append({"kind": "plan", "label": label, "tf": p.tf, "fuel_t": p.fuel / 1e3})

print("closed loop, nominal flight (propellant left at the arms):")


class NoCap(C.ConvexMPC):
    def _solve(self, t, r, v, tf, margin, gate_speed_max=None):
        e = self.engines_for_planning(t)
        if self.engine_out and t >= self.scen.switch_time:
            e = (e[1], e[1])
        return G.solve_with_drag(self.veh, self.tower, r, v, self.m_eff, tf, passes=1, n_nodes=self.n_nodes,
                                 switch_time=self.scen.switch_time, engines=e, elapsed=t, total_tf=t + tf,
                                 margin=margin, extra_accel=self._residual(), gate_speed_max=gate_speed_max,
                                 approach_cap=False)


for label, ctl in [("MPC (default)", C.ConvexMPC()),
                   ("MPC, no approach cap", NoCap()),
                   ("MPC, no extra time", C.ConvexMPC(extra_time=0.0)),
                   ("MPC, 4% margin", C.ConvexMPC(margin=0.04)),
                   ("Baseline a2=6", C.Baseline(a2=6.0)),
                   ("Baseline a2=8 (default)", C.Baseline(a2=8.0)),
                   ("Baseline a2=10", C.Baseline(a2=10.0))]:
    res = sim.simulate(ctl, veh, tw, sc)
    left = res.capture.get("propellant_left", float("nan")) / 1e3
    print(f"  {label:28s} {res.reason:24s} propellant left {left:5.2f} t")
    rows.append({"kind": "closed", "label": label, "reason": res.reason, "prop_left_t": left})
(ROOT / "results").mkdir(exist_ok=True)
json.dump(rows, open(ROOT / "results" / "fuel_ablation.json", "w"), indent=1)
