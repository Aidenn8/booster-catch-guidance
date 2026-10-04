"""Super Heavy booster catch: fuel-optimal guidance, closed-loop simulation, Monte Carlo.

    vehicle     booster + tower parameters (public estimates, all in one place)
    guidance    convex powered-descent guidance (lossless convexification)
    sim         3-DoF flight simulator: drag, wind, engine failures, sensor noise
    controllers the convex-MPC guidance and a constant-deceleration baseline
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
WORK = ROOT / "work"
