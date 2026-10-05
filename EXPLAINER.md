# The booster catch, explained from scratch

This is the plain-language version of the project: what problem it solves, how the two
"pilots" work, and what the experiments found. No equations needed.

## The problem

SpaceX's Super Heavy booster is about 71 m tall and weighs about 275 tonnes empty. After
it pushes Starship up, it flies back to the launch site and, instead of landing on legs,
it is caught in mid-air by two giant arms on the launch tower (people call them the
"chopsticks"). The booster has to arrive in a box roughly 1 m wide, moving down at walking
speed, nearly upright, next to a tower it must not touch.

It starts the landing burn about 1.25 km up, falling at about 270 m/s (~970 km/h). It has
about 16 seconds and roughly 45 tonnes of propellant to stop. First 13 engines fire, then
only the 3 centre engines finish the job.

The question this project asks: **how should the computer steer those last 16 seconds?**
I built two different "pilots" (guidance algorithms), a physics simulator full of the
things that go wrong in real life, and flew both pilots through thousands of simulated
landings to see where each one succeeds and fails.

## A surprising piece of physics: the throttle floor

A Raptor engine cannot throttle below about 40%. With three engines at 40%, the thrust
is about 2.7 MN; a nearly empty booster weighs about 2.75 MN. So at minimum throttle the
booster **almost exactly hovers**.

That sounds harmless but it shapes everything:

* The booster can brake hard (full throttle is 2.5x its weight) but can barely speed
  up its descent. If it ever brakes too much and stops a few metres above the arms, it
  is stuck: it cannot come down without shutting off an engine, and it slowly burns its
  fuel hovering. I hit exactly this failure in early versions.
* So the final approach cannot be "coast down slowly at a constant speed" (that needs
  thrust equal to weight, which is right on the floor, leaving no room to correct
  downwards). Instead both pilots end with a **gentle, steady deceleration** from a point
  12 m above the arms (the "gate"), which keeps the engines a bit above their floor so
  they can push both ways.

## Pilot 1: the baseline (a sensible hand-written rule)

This is the kind of logic a good engineer would write by hand:

* **Up/down:** pick a constant deceleration (8 m/s² by default) and compute, for every
  height, how fast the booster should be falling so that it reaches the gate at the right
  speed. Then just follow that speed-versus-height curve.
* **Sideways:** fly to a waiting point offshore of the tower; when the "commit window"
  opens (the last ~10 s), fly over to the arms, braking smoothly so it arrives with no
  sideways speed.

It is simple, fast (a fraction of a millisecond per decision) and easy to trust.

## Pilot 2: convex model-predictive control (the optimizer)

Every second, this pilot solves a full math problem: *"from exactly where I am now, what
is the thrust plan for the rest of the flight that uses the least fuel, reaches the gate
exactly, never asks the engines for more than they can do, never tilts too far, stays
offshore until the commit window, and never dips below a safe approach cone?"*

Problems like this are normally very hard to solve reliably, because "an engine can be
at 40% or at 100% but never at 0%" is a nasty, non-convex constraint. A 2007 result by
Açıkmeşe and Ploen (the method behind NASA JPL's "G-FOLD" Mars landing tests) showed a
trick: rewrite the problem in slightly different variables and relax that constraint,
and the relaxed problem is *convex* (a bowl shape with one bottom), which a solver can
crack in a fraction of a second, every time, with a guarantee that the answer is also
valid for the real engines. This is called **lossless convexification**. Here it takes
about 0.1-0.3 s per plan.

Between re-plans, the pilot follows the plan and adds small corrections for errors.

One extra superpower: if the optimizer reports that **no** valid plan exists, that is a
mathematical proof (under its model) that the catch can no longer be made. The pilot then
aborts immediately and steers away from the tower, instead of chasing a hopeless catch.

## What both pilots share (so the comparison is fair)

* The same noisy sensors, engines and physics.
* The same **estimator**: both learn, in flight, how much weaker or stronger the engines
  really are than expected and how hard the wind is pushing, and correct for it.
* The same **final 12 m**: from the gate down, both use the identical descent law.
* The same **safety rules**: a go/no-go check at the gate, a "keep out of the tower"
  watch, and an offshore divert if the catch is lost. A booster that cannot be caught
  is supposed to splash down offshore; a crash near the tower is the worst outcome.

## What the simulator throws at them

Each simulated flight randomly varies: where the burn starts (tens of metres), how fast
it is going, mean wind (up to 15 m/s) plus gusts, the drag of the booster (±20%), engine
thrust (±2% typical), fuel efficiency, the booster's empty mass (±2%), sensor noise, a
quarter-second engine response lag, and in 20% of flights one engine fails at a random
moment.

## How I checked the math is right

* **Valid for real engines:** every planned thrust lands inside the true 40-100% range.
* **Matches an independent answer:** for a straight-down descent the textbook optimal
  strategy is known (coast at minimum thrust, then full thrust); solving it a completely
  different way gives the same fuel to within 0.03%.
* **Matches a general-purpose optimizer:** SciPy's SLSQP, solving the original hard
  problem from three different starting guesses, lands on the same flight time and fuel
  (within 0.05%) as the convex method.
* **Matches the simulator:** replaying a plan open-loop in the physics simulator lands
  within a few metres of where the plan said it would.

## Lessons the experiments taught me

These came out of debugging, and each one is a real property of the problem:

1. **"Optimal" means "on the edge".** A fuel-optimal plan brakes as late as possible and
   rides every limit at once: full thrust *and* full tilt. Any small error then makes the
   plan impossible. The fix is margins on both thrust (8%) and tilt (2 degrees), kept for
   corrections.
2. **A quarter-second matters.** The engines take ~0.25 s to change thrust. Optimal plans
   switch thrust abruptly, so every switch arrived late and lost 1-3 m/s of braking
   (always in the dangerous direction). Commanding the plan a quarter-second early fixed it.
3. **Clipping the right way.** When a command asks for more sideways push than the tilt
   limit allows, you must trim the sideways part, not rotate the whole thrust vector
   (rotation turns the excess into extra braking, which stalls the booster at the floor).
4. **Know which number you are fooling.** Treating "engines 4% weak" as "booster 4%
   heavier" gives the right accelerations but makes the planner miscount the remaining
   fuel by 10 tonnes, so it falsely declared catches impossible.
5. **A planner must agree with itself.** The test that found the biggest bug was simple:
   take a plan, jump to a point along it, and ask the planner to plan again from there
   with the same rules. It said "impossible". Air drag was added in a second pass, and
   the first pass ignored it; drag supplies a lot of braking early on, so the drag-free
   first pass failed and the planner gave up before ever adding drag. Starting the first
   pass with a drag estimate fixed it, and the optimizer stopped wasting fuel on
   needless re-timing.
6. **Rules checked only at a few instants can be broken in between.** The planner checks
   "stay offshore until the commit window" at its 32 time points. Between two points the
   plan could drift slightly past the line, and the next plan (with different time points)
   then found the same timing impossible. Putting a time point exactly on every boundary
   where a rule changes fixed it.

## Results

All pilots flew the same 300 randomly disturbed landings (56 of them with an engine
failure). The settings for each pilot were chosen beforehand on a separate set of 40 flights.

* **Normal days (no engine failure, 244 flights):** the optimizer caught 243 (the
  approach-capped version all 244); the hand-written baseline caught 217. The optimizer
  also landed with more spare propellant (5.2 t vs 3.9 t, typical) and much closer to the
  bullseye (4 cm vs 38 cm from the catch point).
* **Odd starts:** shifting where the burn begins by up to 200 m sideways and 300 m up or
  down, the optimizer still caught 127 of 143 starting points; the baseline 86. The
  optimizer never hit the pad; the baseline did 25 times.
* **Wind:** both work up to about 15 m/s; at 20 m/s the optimizer caught every flight and
  the baseline two thirds. Past 25 m/s neither can: the wind pushes the tall booster
  sideways harder than a nearly upright booster can correct.
* **Engine failures:** this is where it gets interesting. The optimizer saves far more
  boosters after an engine failure (16 of 56, versus 2), but because it brakes late to save
  fuel, when an engine fails mid-burn it is lower and faster, and some of its aborts land
  too close to the tower (20 of 56). Adding one rule (approach the tower no faster than
  10 m/s) cuts that to 5, the fewest of all, at the cost of 1.5 t of propellant.
* **Surviving any engine failure:** one more mathematical rule, "always keep enough braking
  that two engines could still stop you", lets the optimizer survive an engine failure at
  almost any moment (32 of 33 moments in calm air). With the normal 45 t of landing
  propellant that leaves too little spare for real-world variation; with **50 t (about
  5 t more)** it survived 19 of 20 engine failures in disturbed flights and never hit the
  pad. That is a concrete answer to "what does engine-out tolerance cost?".

The honest limits: this is a simplified model (the booster is treated as a point, not a
spinning body), all vehicle numbers are public estimates, and the tolerances are my
assumptions. The comparisons and the reasons behind them are the result, not the exact
percentages.

## Where to go next

* **[Watch it](https://aidenn8.github.io/booster-catch-guidance/):** the interactive report with a
  3D replay of six landings, both pilots flying side by side.
* **[The technical report](REPORT.md):** the same story with the equations, every table and the
  seven things that broke along the way.
* **[How the code works](docs/ARCHITECTURE.md)** and **[how to re-run everything](docs/REPRODUCE.md)**.
