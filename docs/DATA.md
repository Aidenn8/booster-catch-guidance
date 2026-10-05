# Data formats

Every number in the report is computed from the files in `results/`. This page documents
each file and field so the results can be analysed without running the simulator.

- [Campaign files](#campaign-files-resultsmc_jsonl)
- [Fields in every flight record](#fields-in-every-flight-record)
- [Campaign-specific fields](#campaign-specific-fields)
- [validation.json](#validationjson)
- [fuel_ablation.json](#fuel_ablationjson)
- [Replay data](#replay-data-docsreplayflightsjson)
- [Loading the data](#loading-the-data)

## Campaign files (`results/mc_*.jsonl`)

JSON Lines: one JSON object per simulated flight, appended as flights finish. Each
`(controller, case_id)` pair appears once; every controller in a file flew the same cases.

| File | Campaign | Cases | Controllers | Generator |
|---|---|---|---|---|
| `mc_tune_v4.jsonl` | Tuning set: full dispersions, seed 99 | 40 per setting | 5 baseline braking levels, 6 MPC settings | `run_mc.py tune --n 40 --tag _v4` |
| `mc_random.jsonl` | Main Monte Carlo: full dispersions, seed 2026 | 300 | `mpc`, `mpc[approach_cap=1]`, `mpc[eo_reserve=1]`, `baseline` | `run_mc.py random --n 300` |
| `mc_engineout.jsonl` | One engine fails at 0, 0.5, …, 16 s (plus a no-failure case); calm air, no other errors | 34 | as above | `run_mc.py engineout` |
| `mc_wind.jsonl` | Mean wind 0–30 m/s in 5 m/s steps, random direction, gusts, small errors, no engine failures | 12 per speed | `mpc`, `baseline` | `run_mc.py wind` |
| `mc_basin.jsonl` | Ignition point shifted −200…200 m (40 m steps) towards the tower and −300…300 m (50 m steps) in altitude; calm air | 143 | `mpc`, `baseline` | `run_mc.py basin` |
| `mc_eoprop.jsonl` | The same 40 dispersed flights (20 with a random engine failure) at 45, 50 and 55 t of landing propellant | 120 | `mpc[eo_reserve=1]`, `mpc` | `run_mc.py eoprop` |

Controller keys are the specification strings accepted by `montecarlo.make_controller`:
`mpc`, `baseline`, or a variant like `mpc[margin=0.08,approach_cap=0]` or `baseline[a2=8]`.
Defaults: MPC margin 0.08, tilt margin 2°, no approach cap, no engine-out reserve; baseline
`a2 = 8` m/s².

> **Note on `mc_tune_v4.jsonl`.** The tuning set was flown when the approach cap was on by
> default. In that file, `mpc[margin=…]` keys **without** `approach_cap=0` are the *capped*
> settings, and keys with `approach_cap=0` are uncapped. The selected setting,
> `mpc[margin=0.08,approach_cap=0]`, became the default `mpc` used in every later campaign.
> `scripts/make_figures.py` (`fig_tradeoff`) reads the file this way.

## Fields in every flight record

**Outcome**

| Field | Type | Meaning |
|---|---|---|
| `controller` | string | Controller specification (see above) |
| `case_id` | int | Case number, unique per controller within a file |
| `caught` | bool | Captured within all tolerances |
| `reason` | string | `caught`, or why not, e.g. `aborted (no feasible trajectory)`, `aborted (braking out of reach)`, `aborted (no-go at the gate)`, `out of propellant`, `missed: lateral miss, too fast vertically`, `hit the tower`. After an abort, the abort comes first, then what happened next, separated by `;` |
| `pad_safe` | bool | `true` if caught, or if a failed booster came down more than 40 m from the catch point. `false` is a *pad strike* |
| `impact` | [x, y] or null | Where a failed booster came down (m); null for catches and tower strikes |
| `abort_reason` | string or null | Why the controller gave up (`no feasible trajectory`, `braking out of reach`, `no-go at the gate`, `tower keep-out`, `missed the arms`, `stalled above the arms`) |
| `abort_time` | float or null | Seconds after ignition |
| `t_end` | float | Last recorded time (s) |
| `min_tower_clearance` | float | Closest approach of the booster's skin to the tower face while below the tower top (m); negative means contact |

The outcome categories used in the report are `caught` (`caught` is true), *safe miss*
(`caught` false, `pad_safe` true) and *pad strike* (`pad_safe` false).
`scripts/summarize.py` defines `outcome(record)`.

**State at the catch height** (present whenever the centre of mass crossed 65 m, including
misses; absent if it never got there)

| Field | Unit | Meaning |
|---|---|---|
| `lateral_error` | m | Horizontal distance from the catch point |
| `vz` | m/s | Vertical velocity (negative is downwards) |
| `vh` | m/s | Horizontal speed |
| `tilt_deg` | deg | Thrust tilt from vertical |
| `propellant_left` | kg | Propellant remaining |
| `time` | s | Time of the crossing |

**MPC diagnostics** (null for the baseline)

| Field | Meaning |
|---|---|
| `n_solves` | Re-plans attempted |
| `failed_replans` | Re-plans for which no plan fitted within the normal margins |
| `recoveries` | Failed re-plans rescued by the faster-gate recovery plan |
| `fail_times` | List of `[time, seconds to the gate]` for each failed re-plan |

**Run information**

| Field | Meaning |
|---|---|
| `wall_s` | Wall-clock seconds the flight took to simulate |

**Dispersion** (the `sim.Dispersion` the flight was flown with, prefixed `d_`)

| Field | Unit | Meaning |
|---|---|---|
| `d_dr0` | m, [x, y, z] | Ignition position offset |
| `d_dv0` | m/s, [x, y, z] | Ignition velocity offset |
| `d_wind_mean` | m/s, [x, y] | Mean wind |
| `d_gust_sigma` | m/s | Gust intensity (Ornstein–Uhlenbeck, 3 s correlation) |
| `d_cd_scale` | – | Drag coefficient multiplier |
| `d_thrust_bias` | fraction | Thrust error (0.02 = 2% more thrust than commanded) |
| `d_isp_scale` | – | Specific-impulse multiplier |
| `d_dry_mass_scale` | – | Dry-mass multiplier |
| `d_engine_out_time` | s or null | When one engine fails |
| `d_nav_sigma_r`, `d_nav_sigma_v` | m, m/s | Navigation noise |
| `d_tau_lag` | s | Engine response time constant |
| `d_seed` | int | Seed for gusts and navigation noise |

A flight can be re-flown exactly from these fields:
`sim.simulate(make_controller(rec["controller"]), SUPER_HEAVY, TOWER, NOMINAL,
sim.Dispersion(**{k[2:]: v for k, v in rec.items() if k.startswith("d_")}))`
(`scripts/rerun_cases.py` does this).

## Campaign-specific fields

| File | Extra fields |
|---|---|
| `mc_engineout.jsonl` | `engine_out_time` (s, null for the no-failure reference case) |
| `mc_wind.jsonl` | `wind_speed` (m/s, mean) |
| `mc_basin.jsonl` | `dx`, `dz` (m): ignition shift towards the tower and in altitude |
| `mc_eoprop.jsonl` | `landing_propellant` (kg), `flight` (index of the underlying dispersed flight, 0–39) |

## validation.json

Written by `scripts/validate_guidance.py`; the tests assert the same quantities.

| Key | Meaning |
|---|---|
| `nominal_solve_s` | Solve time of the nominal plan |
| `nominal_rel_lossless_gap` | Largest relative (σ − ‖u‖)/σ over the plan |
| `vertical.*` | 1-D descent: convex vs shooting propellant (kg) and flight time (s), and their relative difference |
| `nlp.convex_*` | Convex free-time optimum: flight time, propellant, solve time |
| `nlp.slsqp.<start>` | SLSQP on the non-convex problem from three starting guesses: flight time, propellant, iterations, success, time, relative difference |
| `sim_vs_plan.*` | The nominal plan replayed open loop in the simulator: final position (m), velocity (m/s) and mass (kg) error |

The integrator check against the exact rocket-equation solution is asserted in
`tests/test_sim.py` rather than stored here.

## fuel_ablation.json

Written by `scripts/fuel_ablation.py`. A list of entries:

* `kind: "plan"`: the propellant (`fuel_t`, tonnes) and flight time (`tf`) of the nominal
  plan with each robustness feature added in turn: no margins, the thrust margin, the tilt
  margin, the approach cap.
* `kind: "closed"`: the nominal closed-loop flight of each controller setting: `reason`,
  `prop_left_t`.

## Replay data (`docs/replay/flights.json`)

Written by `scripts/export_replay.py` and embedded into the replay page by
`scripts/build_replay.py`.

```text
{
  "meta": {
    "tower":   {"catch_point": [x, y, z], "gate_height", "standoff", "pad_radius"},
    "vehicle": {"length", "diameter"},
    "switch_time": 3.0
  },
  "flights": [                                  six showcase scenarios
    {
      "id", "title", "story",                   scenario name and one-line description
      "wind": [wx, wy], "engine_out_time",      the scenario's disturbances
      "runs": {
        "mpc" | "baseline" | "mpc_eo": {        mpc_eo only in engine-failure scenarios
          "controller", "outcome", "reason", "abort", "abort_time",
          "capture": {...},                     as in the campaign records
          "t":    [s, ...],                     every 0.1 s
          "r":    [[x, y, z], ...],             m
          "v":    [[vx, vy, vz], ...],          m/s
          "T":    [[Tx, Ty, Tz], ...],          MN
          "n":    [engines, ...],
          "prop": [t, ...],                     propellant left, tonnes
          "mode": ["approach" | "terminal" | "divert", ...],
          "plans": [{"t0": s, "r": [[x, y, z], ...]}, ...]   the MPC's plans, for drawing
        }
      }
    }
  ]
}
```

## Loading the data

```python
import json
from collections import Counter

rows = [json.loads(line) for line in open("results/mc_random.jsonl")]
outcome = lambda d: "caught" if d["caught"] else ("safe miss" if d["pad_safe"] else "pad strike")
table = Counter((d["controller"], d["d_engine_out_time"] is not None, outcome(d)) for d in rows)
for (ctl, failure, out), n in sorted(table.items()):
    print(f"{ctl:22s} {'engine failure' if failure else 'no failure':15s} {out:11s} {n}")
```

Or use the scripts: `python scripts/summarize.py results/mc_random.jsonl` and
`python scripts/split_summary.py results/mc_random.jsonl`.
