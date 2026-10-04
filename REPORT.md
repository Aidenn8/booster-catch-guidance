# Catching Super Heavy: fuel-optimal guidance versus a braking heuristic

A simulation study of the last ~16 seconds of a Super Heavy tower catch: a convex,
fuel-optimal model-predictive controller and an engineered braking-profile controller fly
the same dispersed landing burns, with the same sensors, engines, estimator, terminal
descent and abort rules. Every number below comes from code in this repository; all
vehicle data are public estimates, not SpaceX data.

<!-- RESULTS-SUMMARY -->

## 1. Setup

**Vehicle** (`boostercatch/vehicle.py`). Raptor 2 sea-level thrust 2.256 MN, throttle
40-100 %, Isp 327 s; booster empty mass 275 t, 45 t of landing propellant, 9 m x 71 m.
The landing burn starts on 13 engines and switches to the 3 centre engines at 3 s.

**Scenario.** Ignition at 1,250 m altitude, 280 m offshore and 60 m north of the tower,
falling at 270 m/s with 30 m/s of drift towards the tower. Frame: x points from offshore
towards the tower, z up; the catch point is the booster's centre of mass at capture, 65 m
above the tower base.

**Catch.** The booster's centre of mass must pass the catch height within 1 m of the
catch point, descending at under 1.5 m/s, moving sideways at under 0.5 m/s, with thrust
tilt under 3°. A booster that misses keeps flying (diverting offshore) until it reaches
the ground; it is a *safe miss* if it comes down more than 40 m from the tower, otherwise a
*pad strike*. Touching the tower structure is always a pad strike.

**The throttle floor.** Three Raptors at 40 % give 2.71 MN; a near-empty booster weighs
about 2.75 MN. At minimum throttle the booster almost exactly hovers: it can brake at up
to ~13 m/s² but can accelerate its descent by only a few tenths of a m/s². Over-braking
near the arms is therefore a trap (an early version stalled 12 m above the arms and hovered
until its propellant ran out), and a constant-speed final descent would sit on the floor
with no authority to correct downwards. Both controllers therefore hand over at a
**terminal gate** 12 m above the arms to the same law: a constant 1 m/s² deceleration from
5 m/s to 1 m/s, which keeps thrust ~10 % above the floor.

## 2. The two controllers (`boostercatch/controllers.py`)

**Convex MPC.** Every second (0.5 s after ignition, then every 1 s, frozen in the last
2.5 s before the gate) it solves a minimum-fuel trajectory from the current estimated
state to the gate, then tracks it with a PD correction. The plan respects: thrust in
[40 %, 100 % - margin] of the live engines (13, then 3), thrust tilt ≤ 15° - tilt margin
(≤ 3° in the 2 s before the gate), an offshore half-space until the commit window (6 s
before the gate) and an approach-speed cap inside it, a 20° glide-slope cone over the
catch point, drag (two-pass profile), the estimated disturbance, and the expected thrust
tail-off at the engine switch. If no plan exists with the margins it relaxes them in tiers,
then allows a faster gate arrival (up to 12 m/s, which the terminal descent can absorb);
if still nothing exists and the gate is more than 3 s away, the infeasibility is taken as
a certificate that the catch is lost and it aborts. An engine failure triggers an
immediate re-plan with the reduced engine count.

**Baseline.** Vertical: during the 13-engine phase, the constant acceleration that lands
the booster on a constant-deceleration speed profile at the engine switch; then that
profile (deceleration a2, tuned) down to the gate. Sideways: a braking profile (3 m/s²,
capped at 45 m/s) to a waiting point offshore of the tower, switching to the catch point
when the commit window opens. On an engine failure it rescales a2 to the remaining
braking capability; if the braking needed to reach the gate exceeds 95 % of what the
engines can give, it aborts.

**Shared, so the comparison isolates the guidance:**
* a thrust model (commands through the nominal engine limits and 0.25 s lag) and a
  mass estimate integrated from it;
* an observer that explains the unexplained acceleration as a thrust-effectiveness
  factor k (along the thrust: thrust bias and mass error) plus a residual (across it:
  mostly wind); commands are scaled by 1/k and the residual is fed forward with the drag
  model;
* the terminal descent from the gate, entered one throttle lag early;
* a go/no-go check at the gate (predicted sideways stopping point within 1.5 m), a
  tower keep-out watch, and an offshore divert (up to 30° tilt) after any abort.

## 3. Guidance: lossless convexification (`boostercatch/guidance.py`)

Following Açıkmeşe & Ploen (2007) and Açıkmeşe, Carson & Blackmore (2013): with
u = T/m, σ = Γ/m and z = ln m, the non-convex thrust ring T_min ≤ ‖T‖ ≤ T_max becomes
‖u‖ ≤ σ with σ bounded by Taylor expansions of T_min e^{-z} (second order, below) and
T_max e^{-z} (first order, above) about the lightest reachable mass, and the mass dynamics
become linear: ż = -σ/(Isp g0). The problem is a second-order cone program (CVXPY +
Clarabel, 32 nodes, ~0.1-0.3 s per solve). The relaxation is lossless (‖u‖ = σ at the
optimum); the code rejects any plan that would ask for less than minimum throttle.

The time grid has nodes exactly at every rule boundary (engine switch, commit window,
final alignment). Without this, a constraint checked only at nodes can be violated
between them and the next re-plan, on a different grid, finds the same gate time
infeasible and must slip it, at a cost of ~0.5 t of propellant per half second.

### Validation (`tests/test_guidance.py`, `scripts/validate_guidance.py`)

| check | result |
|---|---|
| lossless gap on the nominal plan (max σ - ‖u‖, relative) | 2 x 10⁻⁷ |
| planned thrust inside the real 40-100 % range | every interval |
| vertical descent vs independent bang-bang shooting solution | fuel 16,524.6 vs 16,520.2 kg (0.03 %), flight time 10.261 vs 10.256 s |
| SLSQP on the original non-convex problem, free final time, 3 different starting guesses | all converge to 11.233 s, 18,170.2 kg; convex: 11.221 s, 18,178.5 kg (+0.05 %, the conservative Taylor bounds) |
| plan replayed open loop in the simulator (14 s from 270 m/s) | 3.7 m, 0.17 m/s, 36 kg off |
| simulator integrator vs exact rocket-equation solution | < 1 mm, < 1 mm/s |

A note on the SLSQP check: at a *fixed* flight time, SLSQP from a smooth start converges
in 3 iterations to a smooth thrust ramp with the same fuel as the convex max-min-max
profile (19,049.9 vs 19,048.0 kg). The fuel is nearly independent of the thrust profile at
fixed time, because the boundary conditions fix the vertical impulse; the real fuel lever is
the flight time (gravity loss). The free-final-time comparison above is the discriminating
test.

## 4. Simulator and dispersions (`boostercatch/sim.py`, `boostercatch/montecarlo.py`)

3-DoF point mass, RK4 at 50 Hz. Thrust direction stands in for attitude (no rotational
dynamics). Truth model and Monte Carlo dispersions (1σ unless stated):

| quantity | model / dispersion |
|---|---|
| drag | crossflow model: vertical airflow on the 64 m² base (Cd 0.9), horizontal on the 639 m² side (Cd 0.7); Cd x U(0.8, 1.2) |
| wind | mean U(0, 15) m/s, random direction; gusts: Ornstein-Uhlenbeck, σ = 0.5 + 0.15 x mean, 3 s correlation |
| engines | thrust bias N(0, 2 %) clipped at ±5 %, Isp x N(1, 0.5 %), first-order lag 0.25 s |
| mass | dry mass x N(1, 2 %) |
| ignition state | position 20 m horizontal / 15 m vertical, velocity 3 m/s horizontal / 5 m/s vertical |
| engine failure | probability 0.2, at a uniform time in [0, 15] s; one engine lost for the rest of the burn |
| navigation | white noise 0.3 m, 0.1 m/s |

A 15 m/s crosswind pushes the booster sideways at 0.22 m/s²; the terminal descent's tilt
limit (2.7°) allows about 0.5 m/s² of correction, which is why wind is a real limit here.

<!-- RESULTS -->
