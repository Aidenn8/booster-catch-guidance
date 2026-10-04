"""Re-fly cases from a results file in-process (quick check after a controller change).

    python scripts/rerun_cases.py results/mc_pilot.jsonl baseline            # all cases
    python scripts/rerun_cases.py results/mc_pilot.jsonl mpc 8 9 13 14       # some cases
"""
import json
import sys
import time
import warnings
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
warnings.filterwarnings("ignore")

from boostercatch import montecarlo as MC, sim  # noqa: E402
from boostercatch.vehicle import NOMINAL, SUPER_HEAVY, TOWER  # noqa: E402

path, key = sys.argv[1], sys.argv[2]
want = {int(x) for x in sys.argv[3:]}
cases = {}
for line in open(path):
    d = json.loads(line)
    cases[d["case_id"]] = sim.Dispersion(**{k[2:]: v for k, v in d.items() if k.startswith("d_")})
n_ok = n = 0
for i, disp in sorted(cases.items()):
    if want and i not in want:
        continue
    ctl = MC.make_controller(key)
    t0 = time.time()
    res = sim.simulate(ctl, SUPER_HEAVY, TOWER, NOMINAL, disp)
    n += 1
    n_ok += res.caught
    cap = {k: round(v, 2) for k, v in res.capture.items()}
    print(f"{i:3d} {key} {res.reason:32s} safe={res.pad_safe!s:5s} eo={disp.engine_out_time} {cap} "
          f"{time.time() - t0:.0f}s fails={getattr(ctl, 'fail_times', '')}", flush=True)
print(f"{key}: caught {n_ok}/{n}")
