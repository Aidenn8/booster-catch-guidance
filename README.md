# booster-catch

**Fuel-optimal guidance for catching Super Heavy, stress-tested against a braking
heuristic in thousands of simulated landing burns.**

The last ~16 seconds of a Super Heavy tower catch, simulated from ignition (1.25 km up,
270 m/s) to the chopsticks. Two guidance algorithms fly the same dispersed flights with
the same sensors, engines, estimator, terminal descent and abort rules:

* **Convex MPC**: every second, solve a minimum-fuel trajectory to the arms as a
  second-order cone program (lossless convexification, as in JPL's G-FOLD), with the
  13 -> 3 engine switch, thrust and tilt margins, an offshore commit window, a glide-slope
  cone, drag and estimated disturbances; abort when the solver proves no catch exists.
* **Baseline**: an engineered constant-deceleration braking profile with an offshore
  waiting point, the kind of logic a careful engineer writes by hand.

Plus an optional **engine-out braking reserve**: one convex constraint that keeps enough
braking in hand to survive losing an engine at any moment, and a measurement of what it
costs in propellant.

### Results (300 dispersed flights per controller, same flights for all)

| | convex MPC | MPC + approach cap | baseline |
|---|---|---|---|
| caught, all flights (95 % CI) | **86.3 %** (82-90) | **86.3 %** (82-90) | 73.0 % (68-78) |
| caught, no engine failure | 243 / 244 | **244 / 244** | 217 / 244 |
| median propellant left at the catch | **5.2 t** | 3.7 t | 3.9 t |
| median distance from the catch point | **4 cm** | **4 cm** | 38 cm |
| boosters saved after an engine failure | 16 / 56 | 15 / 56 | 2 / 56 |
| pad strikes (all after engine failures) | 20 | **5** | 8 |

* **The catch happens at the throttle floor**: three Raptors at 40 % almost exactly hover a
  near-empty booster, so over-braking near the arms is a trap. Both controllers end with a
  gentle decelerating descent from a gate 12 m above the arms.
* **Fuel-optimal guidance wins on accuracy, robustness and fuel**, catches from far more
  ignition points (127/143 vs 86/143) and in stronger wind (100 % vs 67 % at 20 m/s).
* **Its weak spot is the mid-burn engine failure**: braking late saves fuel but leaves less
  room to divert. A 10 m/s approach cap fixes most of it for 1.5 t of propellant.
* **Engine-out tolerance = one convex constraint + ~5 t of propellant**: a braking reserve
  that keeps two engines able to stop the booster survives 19 of 20 engine failures under
  full dispersions with 50 t of landing propellant (45 t baseline), with no pad strikes.
* **Verified independently**: the convex solution matches a shooting solution (0.03 %)
  and SciPy SLSQP on the original non-convex problem from three starts (0.02 %).

**[Interactive 3D replay](docs/replay/index.html)** of six flights, all controllers side by
side · **[Full report](REPORT.md)** · **[Plain-language explainer](EXPLAINER.md)** ·
**[Reproduce](docs/REPRODUCE.md)**

![Fuel vs robustness](docs/images/tradeoff.png)
![Engine failure timing](docs/images/engineout.png)
![Capture basin](docs/images/basin.png)
![Engine-out tolerance vs propellant](docs/images/eoprop.png)

## What is in here

```
boostercatch/
  vehicle.py       booster, tower and scenario parameters (public estimates, one place)
  guidance.py      lossless-convexification SOCP (CVXPY + Clarabel), drag passes, gate timing
  controllers.py   ConvexMPC, Baseline, shared estimator / terminal descent / abort / divert
  sim.py           3-DoF RK4 simulator: drag, gusty wind, thrust & mass errors, lag, noise,
                   engine failures; classifies every miss as safe or a pad strike
  montecarlo.py    dispersion model and parallel campaign runner
scripts/           campaigns, summaries, figures, validation, replay export, debugging tools
tests/             14 tests: convex vs shooting vs SLSQP, simulator vs rocket equation, ...
results/           one JSON line per simulated flight
docs/images/       figures;  docs/replay/  the interactive replay page
tools/thermal/     thermal guard used to run the campaigns on a fanless laptop
```

## Honest limits

A point-mass model (thrust direction stands in for attitude; no rotational dynamics, grid
fins or slosh), public-estimate vehicle numbers, an assumed catch tolerance and tower
geometry, and Python solve times (~0.1-0.3 s per plan; flight code would use an embedded
solver). The value is in the comparisons, the failure mechanisms and the method, not in
the absolute percentages. Details in [REPORT.md](REPORT.md).
