"""Fly the showcase flights and export them for the 3D replay page (docs/replay/flights.json).

    python scripts/export_replay.py
"""
import json
import sys
import warnings
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
warnings.filterwarnings("ignore")

import numpy as np  # noqa: E402

from boostercatch import ROOT, controllers as C, sim  # noqa: E402
from boostercatch.vehicle import NOMINAL, SUPER_HEAVY, TOWER  # noqa: E402

OUT = ROOT / "docs" / "replay"
OUT.mkdir(parents=True, exist_ok=True)


def wind(speed, deg):
    a = np.radians(deg)
    return (float(speed * np.cos(a)), float(speed * np.sin(a)))


# (id, title, one-line story, Dispersion kwargs); each is flown by both controllers
SHOWCASE = [
    ("nominal", "Nominal", "Calm air, everything as planned.", {}),
    ("crosswind", "Crosswind 15 m/s", "Steady 15 m/s wind from the north with gusts, pushing on the booster's 71 m side.",
     dict(wind_mean=wind(15, -90), gust_sigma=2.75, seed=5)),
    ("late", "Late ignition", "The landing burn starts 200 m lower than planned: less room to brake.",
     dict(dr0=(0.0, 0.0, -200.0))),
    ("long", "Overshoot", "The booster arrives 160 m further downrange than planned, already closer to the tower.",
     dict(dr0=(160.0, 0.0, 0.0))),
    ("engine5", "Engine out at 5 s", "One of the three centre engines fails 5 s into the burn.",
     dict(engine_out_time=5.0)),
    ("engine10", "Engine out at 10 s", "One of the three centre engines fails 10 s into the burn, inside the commit window.",
     dict(engine_out_time=10.0)),
]


def phase_of(t, r, ctl, res):
    return None


def export_flight(key, ctl, disp):
    plan_paths = []
    if isinstance(ctl, C.ConvexMPC):
        orig = ctl._replan

        def rec(t, r, v):
            before = ctl.plan
            orig(t, r, v)
            if ctl.plan is not None and ctl.plan is not before:
                plan_paths.append({"t0": round(t, 2), "r": np.round(ctl.plan.r, 1).tolist()})
        ctl._replan = rec
    # record the controller's mode every step
    modes = []
    orig_cmd = ctl.command

    def cmd(t, r, v, n):
        out = orig_cmd(t, r, v, n)
        modes.append("divert" if ctl.abort_reason else ("terminal" if ctl.in_terminal else "approach"))
        return out
    ctl.command = cmd
    res = sim.simulate(ctl, SUPER_HEAVY, TOWER, NOMINAL, disp, record_every=5)
    every = 5
    mode_at = [modes[min(i * every, len(modes) - 1)] for i in range(len(res.t))]
    dry = SUPER_HEAVY.dry_mass * disp.dry_mass_scale
    return {
        "controller": key,
        "outcome": "caught" if res.caught else ("safe miss" if res.pad_safe else "pad strike"),
        "reason": res.reason,
        "abort": res.abort_reason, "abort_time": res.abort_time,
        "capture": {k: round(v, 3) for k, v in res.capture.items()},
        "t": np.round(res.t, 2).tolist(),
        "r": np.round(res.r, 2).tolist(),
        "v": np.round(res.v, 2).tolist(),
        "T": np.round(res.thrust / 1e6, 3).tolist(),          # MN
        "n": res.n_engines.astype(int).tolist(),
        "prop": np.round((res.m - dry) / 1e3, 2).tolist(),     # t
        "mode": mode_at,
        "plans": plan_paths,
    }


def main():
    flights = []
    for fid, title, story, kw in SHOWCASE:
        disp = sim.Dispersion(**kw)
        entry = {"id": fid, "title": title, "story": story,
                 "wind": list(disp.wind_mean), "engine_out_time": disp.engine_out_time, "runs": {}}
        for key, ctl in (("mpc", C.ConvexMPC()), ("baseline", C.Baseline())):
            run = export_flight(key, ctl, disp)
            entry["runs"][key] = run
            print(f"{fid:10s} {key:8s} {run['outcome']:10s} {run['reason']}")
        flights.append(entry)
    meta = {
        "tower": {"catch_point": list(TOWER.catch_point), "gate_height": TOWER.gate_height,
                  "standoff": TOWER.standoff, "pad_radius": sim.PAD_RADIUS},
        "vehicle": {"length": SUPER_HEAVY.length, "diameter": SUPER_HEAVY.diameter},
        "switch_time": NOMINAL.switch_time,
    }
    json.dump({"meta": meta, "flights": flights}, open(OUT / "flights.json", "w"), separators=(",", ":"))
    print("wrote", OUT / "flights.json", f"{(OUT / 'flights.json').stat().st_size / 1e3:.0f} kB")


if __name__ == "__main__":
    main()
