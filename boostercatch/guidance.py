"""Fuel-optimal powered-descent guidance by lossless convexification.

The real problem is non-convex: thrust magnitude must lie in [T_min, T_max] (a ring,
because engines cannot throttle to zero), and mass changes as fuel burns. Acikmese &
Ploen (2007) and Acikmese, Carson & Blackmore (2013) showed it can be solved exactly
as a convex second-order cone program:

  * change variables u = T/m, sigma = Gamma/m, z = ln m
  * relax ||T|| in [T_min, T_max] to ||u|| <= sigma with sigma bounded by the
    (approximated) mass-dependent limits. At the optimum ||u|| = sigma ("lossless").
  * the exponential bounds rho*exp(-z) are approximated by a Taylor expansion about
    the lightest mass reachable at each time (second order below, first order above)

Additions for a tower catch (all convex):
  * thrust limits change with time: 13 engines, then 3 (two throttle ranges)
  * thrust pointing cone, tighter in the final seconds (booster vertical for capture)
  * the plan ends at a terminal gate above the arms (a shared terminal descent law
    flies the last few seconds)
  * stay offshore of a plane until `commit_time` before the gate (half-space), then
    close on the tower no faster than `approach_speed_max`
  * a glide-slope cone above the catch point
  * aerodynamic drag enters as a known acceleration profile, updated from the previous
    solution (two passes, a light form of successive convexification)

Free final time: fuel is (nearly) unimodal in flight time, so a golden-section search
over t_f wraps the fixed-time convex solve.
"""
from dataclasses import dataclass

import cvxpy as cp
import numpy as np

from .vehicle import G0

GRAV = np.array([0.0, 0.0, -G0])
RHO_AIR = 1.225
GLIDE_SLOPE_DEG = 20.0
FINAL_ALIGN_TIME = 2.0   # s before the gate: thrust within the final tilt limit


@dataclass
class Plan:
    t: np.ndarray        # (N+1,) node times
    r: np.ndarray        # (N+1, 3) position
    v: np.ndarray        # (N+1, 3) velocity
    m: np.ndarray        # (N+1,) mass
    thrust: np.ndarray   # (N, 3) thrust vector per interval [N]
    accel: np.ndarray    # (N, 3) commanded thrust acceleration u per interval
    sigma: np.ndarray    # (N,)
    fuel: float
    status: str
    tf: float
    lossless_gap: float  # max over intervals of sigma - ||u|| (should be ~0)

    def thrust_at(self, tau):
        k = int(np.clip(np.searchsorted(self.t, tau, side="right") - 1, 0, len(self.thrust) - 1))
        return self.thrust[k]


def drag_accel(v, m, wind, veh):
    """Drag acceleration (m/s^2) on the near-upright booster for velocity v relative to air
    moving at `wind` (crossflow principle): the vertical component of the airflow pushes on
    the 9 m base, the horizontal component on the 71 m x 9 m side, ten times the area, which
    is why crosswind matters so much near the tower."""
    vr = np.asarray(v, float) - np.asarray(wind, float)
    m = np.asarray(m, float)[..., None]
    vh = vr[..., :2]
    side = -0.5 * RHO_AIR * veh.cd_side * veh.side_area * np.linalg.norm(vh, axis=-1, keepdims=True) * vh
    axial = -0.5 * RHO_AIR * veh.cd * veh.area * np.abs(vr[..., 2:]) * vr[..., 2:]
    return np.concatenate([side, axial], axis=-1) / m


def time_grid(tf, N, breaks):
    """N intervals over [0, tf], with nodes exactly at each break inside (0, tf)."""
    bs = sorted({float(b) for b in breaks if b is not None and 1e-3 < b < tf - 1e-3})
    edges = np.array([0.0] + bs + [tf])
    lens = np.diff(edges)
    n = np.maximum(1, np.round(N * lens / tf).astype(int))
    while n.sum() > N:
        n[np.argmax(np.where(n > 1, n, -1))] -= 1     # take from the most finely split segment
    while n.sum() < N:
        n[np.argmax(lens / n)] += 1
    t = [0.0]
    for a, b, k in zip(edges[:-1], edges[1:], n):
        t.extend(np.linspace(a, b, k + 1)[1:])
    return np.array(t)


def solve_fixed(veh, tower, r0, v0, m0, tf, n_nodes=40, switch_time=None, engines=(13, 3),
                wind=(0.0, 0.0, 0.0), drag_profile=None, elapsed=0.0, total_tf=None,
                use_offshore=True, use_glideslope=True, margin=0.0, extra_accel=None, gate_speed_max=None,
                final_align=True, check_floor=True, approach_cap=True, tilt_margin_deg=0.0, switch_lag=0.0,
                v_guess=None, eo_reserve=False,
                solver="CLARABEL"):
    """Minimum-fuel trajectory reaching the terminal gate at time tf (from now): centred
    above the catch point, `gate_height` up, moving straight down at the gate speed.

    gate_speed_max relaxes the gate's descent rate to a range (used for recovery: the
    terminal descent can absorb a faster arrival at the gate).

    elapsed/total_tf let a replanning controller keep the original phase timing
    (engine switch, commit window) while the horizon shrinks. `margin` keeps the plan
    a fraction below maximum thrust, so the controller has braking authority left to
    reject disturbances (a plan riding the limit cannot correct anything). It is not
    applied at minimum thrust: near empty, three engines at minimum throttle barely
    balance the booster's weight, and raising that floor would forbid a slow descent.
    `tilt_margin_deg` does the same for the thrust-pointing cone: fuel-optimal plans also
    ride the tilt limit while they cancel sideways speed.
    """
    N = n_nodes
    total_tf = total_tf if total_tf is not None else tf + elapsed
    # time grid with a node exactly on every boundary where the rules change: the engine
    # switch (no interval mixes the 13- and 3-engine thrust ranges), the start of the commit
    # window and of the final alignment. Otherwise a constraint checked only at nodes can
    # be violated between them, and the next re-plan (on a different grid) finds the same
    # gate time infeasible and has to slip it.
    ts = None if switch_time is None else switch_time - elapsed
    t = time_grid(tf, N, [ts, total_tf - elapsed - tower.commit_time, total_tf - elapsed - FINAL_ALIGN_TIME])
    dt = np.diff(t)
    tt = t[:-1] + elapsed                      # absolute time since ignition, per interval
    n_eng = np.where(tt < (switch_time - 1e-9 if switch_time is not None else -1), engines[0], engines[1])
    tmin = n_eng * veh.engine_thrust * veh.throttle_min
    tmax = n_eng * veh.engine_thrust * veh.throttle_max * (1 - margin)
    alpha = veh.alpha
    # lightest possible mass at each time (burning at max thrust the whole way): expansion point
    burn_max = np.concatenate([[0.0], np.cumsum(alpha * tmax * dt)])
    m_low = np.maximum(m0 - burn_max, veh.dry_mass * 0.95)
    z0 = np.log(m_low[:-1])
    mu1 = tmin * np.exp(-z0)
    mu2 = tmax * np.exp(-z0)
    if drag_profile is not None:
        a_drag = drag_profile
    elif v_guess is not None:          # drag evaluated on a velocity guess (function of time from now)
        tm = 0.5 * (t[:-1] + t[1:])
        a_drag = drag_accel(np.array([v_guess(x) for x in tm]), np.full(N, m0), wind, veh)
    else:
        a_drag = np.zeros((N, 3))
    if extra_accel is not None:          # e.g. an estimated constant disturbance
        a_drag = a_drag + np.asarray(extra_accel)
    if switch_lag > 0 and ts is not None and 0.0 < ts < tf:
        # Engine-switch tail-off: thrust lags commands by ~switch_lag, so when the outer
        # engines shut down the booster keeps ~switch_lag x (13-engine minimum - planned
        # 3-engine thrust) of extra braking impulse. Approximate the planned 3-engine thrust
        # by its (margined) maximum and spread the impulse over the first interval after
        # the switch as a known upward acceleration.
        k_sw = int(np.searchsorted(t, ts - 1e-9))
        if k_sw < N:
            lo1 = engines[0] * veh.engine_thrust * veh.throttle_min
            hi2 = engines[1] * veh.engine_thrust * veh.throttle_max * (1 - margin)
            impulse = switch_lag * max(lo1 - hi2, 0.0)
            a_drag = a_drag.copy()
            a_drag[k_sw, 2] += impulse / (m0 * dt[k_sw])

    r = cp.Variable((N + 1, 3))
    v = cp.Variable((N + 1, 3))
    z = cp.Variable(N + 1)
    u = cp.Variable((N, 3))
    s = cp.Variable(N)
    cons = [r[0] == r0, v[0] == v0, z[0] == np.log(m0)]
    target = np.array(tower.catch_point)
    if gate_speed_max is None:
        cons += [r[N] == tower.gate_point, v[N] == np.array([0.0, 0.0, -tower.gate_speed])]
    else:   # recovery: reach the gate centred, but allow a faster descent through it
        cons += [r[N] == tower.gate_point, v[N][:2] == 0,
                 v[N][2] >= -gate_speed_max, v[N][2] <= -tower.gate_speed]
    cos_max = np.cos(np.radians(veh.max_tilt_deg - tilt_margin_deg))
    cos_fin = np.cos(np.radians(veh.final_tilt_deg))
    for k in range(N):
        acc = u[k] + GRAV + a_drag[k]
        cons += [v[k + 1] == v[k] + acc * dt[k],
                 r[k + 1] == r[k] + v[k] * dt[k] + 0.5 * acc * dt[k] ** 2,
                 z[k + 1] == z[k] - alpha * s[k] * dt[k],
                 cp.norm(u[k]) <= s[k]]
        dz = z[k] - z0[k]
        cons += [s[k] >= mu1[k] * (1 - dz + cp.square(dz) / 2),
                 s[k] <= mu2[k] * (1 - dz)]
        in_final = final_align and (total_tf - tt[k]) <= FINAL_ALIGN_TIME + 1e-6   # upright for the hand-over
        cons += [u[k][2] >= (cos_fin if in_final else cos_max) * s[k]]
    # mass cannot drop below dry mass
    cons += [z >= np.log(veh.dry_mass)]
    if use_offshore:
        for k in range(N + 1):
            if (total_tf - (t[k] + elapsed)) > tower.commit_time + 1e-6:
                cons += [r[k][0] <= target[0] - tower.standoff]
            elif approach_cap:
                # inside the commit window, close on the tower no faster than this, so a
                # booster that has to abort late still has room to divert away from it
                cons += [v[k][0] <= tower.approach_speed_max]
    if eo_reserve:
        # Engine-out braking reserve (convex: a square below an affine function). During the
        # 3-engine phase the booster must always be slow enough that the engines left after
        # losing one (at the heaviest possible mass, i.e. now) could still bring it down to
        # the gate speed at the gate: v_z^2 <= v_gate^2 + 2 a_eo (z - z_gate).
        n_left = engines[1] - 1
        a_eo = n_left * veh.engine_thrust * veh.throttle_max / m0 - G0
        zg = tower.gate_point[2]
        for k in range(1, N + 1):
            if t[k] + elapsed >= (switch_time if switch_time is not None else 0.0) - 1e-9 and a_eo > 0:
                cons += [cp.square(v[k][2]) <= tower.gate_speed ** 2 + 2 * a_eo * (r[k][2] - zg)]
    if use_glideslope:
        # stay above a cone rising at GLIDE_SLOPE_DEG from the catch point: no low,
        # flat approach skimming the ground or the launch mount
        tan_gs = np.tan(np.radians(GLIDE_SLOPE_DEG))
        for k in range(N + 1):
            cons += [tan_gs * cp.norm(r[k][:2] - target[:2]) <= r[k][2] - target[2]]
    prob = cp.Problem(cp.Maximize(z[N]), cons)
    try:
        prob.solve(solver=solver)
    except cp.SolverError:
        return None
    if prob.status not in ("optimal", "optimal_inaccurate") or z.value is None:
        return None
    m = np.exp(z.value)
    uu, ss = u.value, s.value
    thrust = uu * m[:-1, None]
    gap = float(np.max(ss - np.linalg.norm(uu, axis=1)))
    # The relaxation is lossless when ||u|| = sigma. If it is not (possible when the problem
    # is barely feasible), a step may ask for less than minimum throttle, which no engine
    # can deliver: reject such plans rather than fly them.
    if check_floor and np.any(np.linalg.norm(thrust, axis=1) < 0.995 * tmin):
        return None
    return Plan(t=t, r=r.value, v=v.value, m=m, thrust=thrust, accel=uu, sigma=ss,
                fuel=float(m0 - m[-1]), status=prob.status, tf=tf, lossless_gap=gap)


def solve_with_drag(veh, tower, r0, v0, m0, tf, wind=(0.0, 0.0, 0.0), passes=2, v_guess=None, **kw):
    """Drag by successive passes: plan with drag evaluated on a velocity guess, then re-plan
    with drag evaluated on that plan.

    The first pass must include drag. Drag supplies up to ~8 m/s^2 of braking early in the
    burn, so a drag-free first pass is infeasible for any plan that uses its margins, and
    re-planning from a plan's own state at the same margin would then fail: the planner
    would contradict itself. The guess is the previous plan's velocities when re-planning
    (`v_guess`, a function of time from now), else a straight line to the gate velocity."""
    if v_guess is None:
        v_end = np.array([0.0, 0.0, -tower.gate_speed])
        v0a = np.asarray(v0, float)
        v_guess = lambda x: v0a + (v_end - v0a) * min(x / tf, 1.0)  # noqa: E731
    plan = solve_fixed(veh, tower, r0, v0, m0, tf, wind=wind, v_guess=v_guess, **kw)
    for _ in range(passes):
        if plan is None:
            return None
        vmid = 0.5 * (plan.v[:-1] + plan.v[1:])
        prof = drag_accel(vmid, plan.m[:-1], wind, veh)
        plan = solve_fixed(veh, tower, r0, v0, m0, tf, wind=wind, drag_profile=prof, **kw)
    return plan


def solve_earliest(veh, tower, r0, v0, m0, tf_lo=4.0, tf_hi=40.0, tol=0.1, extra=0.0, drag=True, **kw):
    """Plan with the earliest feasible flight time (plus `extra` seconds of slack).

    For this vehicle fuel rises monotonically with flight time inside the feasible
    window (the booster cannot hover: every extra second is gravity loss), so the
    earliest feasible time is the fuel optimum; a bisection on feasibility finds it
    with ~6 solves instead of a full search. `extra` trades a little fuel for timing slack.
    """
    solve = solve_with_drag if drag else solve_fixed
    hi_plan = solve(veh, tower, r0, v0, m0, tf_hi, **kw)
    if hi_plan is None:
        # the window may lie below tf_hi; scan down for any feasible time
        for tf in np.linspace(tf_hi, tf_lo, 15)[1:]:
            hi_plan = solve(veh, tower, r0, v0, m0, tf, **kw)
            if hi_plan is not None:
                tf_hi = tf
                break
        if hi_plan is None:
            return None
    lo, hi = tf_lo, tf_hi
    while hi - lo > tol:
        mid = 0.5 * (lo + hi)
        if solve(veh, tower, r0, v0, m0, mid, **kw) is not None:
            hi = mid
        else:
            lo = mid
    plan = solve(veh, tower, r0, v0, m0, hi + extra, **kw)
    return plan if plan is not None else solve(veh, tower, r0, v0, m0, hi, **kw)


def solve_free_time(veh, tower, r0, v0, m0, tf_lo=10.0, tf_hi=60.0, tol=0.1, drag=True, **kw):
    """Golden-section search on the flight time for the minimum-fuel feasible plan.

    Infeasible flight times score +inf; the bracket is first narrowed to feasible
    times by a coarse scan so the search starts inside the feasible region.
    """
    solve = solve_with_drag if drag else solve_fixed
    grid = np.linspace(tf_lo, tf_hi, 11)
    scored = []
    for tf in grid:
        p = solve(veh, tower, r0, v0, m0, tf, **kw)
        scored.append((p.fuel if p else np.inf, tf, p))
    feas = [x for x in scored if np.isfinite(x[0])]
    if not feas:
        return None
    best = min(feas, key=lambda x: x[0])
    i = list(grid).index(best[1])
    a, b = grid[max(i - 1, 0)], grid[min(i + 1, len(grid) - 1)]
    gr = (np.sqrt(5) - 1) / 2
    cache = {}

    def f(tf):
        if tf not in cache:
            p = solve(veh, tower, r0, v0, m0, tf, **kw)
            cache[tf] = (p.fuel if p else np.inf, p)
        return cache[tf][0]

    c, d = b - gr * (b - a), a + gr * (b - a)
    while b - a > tol:
        if f(c) < f(d):
            b, d = d, c
            c = b - gr * (b - a)
        else:
            a, c = c, d
            d = a + gr * (b - a)
    cand = [best[2]] + [p for (fu, p) in cache.values() if p is not None]
    return min(cand, key=lambda p: p.fuel)
