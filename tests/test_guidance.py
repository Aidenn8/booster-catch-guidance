"""Checks that the convex guidance solves the problem it claims to solve.

1. lossless: the relaxed problem's solution is physically valid (||u|| = sigma, thrust
   inside [T_min, T_max] at every step)
2. 1-D optimum: for a purely vertical descent the convex optimum matches an independent
   shooting solution of the known bang-bang structure (minimum thrust, then maximum)
3. non-convex cross-check: a general nonlinear optimiser (SLSQP) on the original,
   non-convex problem, started from a different guess, finds the same fuel
4. the plan's discrete dynamics agree with the flight simulator's integration
"""
import sys
import warnings
from pathlib import Path

import numpy as np
import pytest
from scipy.optimize import brentq, minimize

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
warnings.filterwarnings("ignore")

from boostercatch import guidance as G, sim  # noqa: E402
from boostercatch.vehicle import G0, NOMINAL, SUPER_HEAVY, TOWER, Tower  # noqa: E402

veh, tw, sc = SUPER_HEAVY, TOWER, NOMINAL


def test_nominal_plan_is_lossless_and_within_engine_limits():
    p = G.solve_earliest(veh, tw, np.array(sc.r0), np.array(sc.v0), veh.wet_mass, switch_time=sc.switch_time)
    assert p is not None
    tmag = np.linalg.norm(p.thrust, axis=1)
    n = np.where(p.t[:-1] < sc.switch_time - 1e-9, sc.engines_phase1, sc.engines_phase2)
    lo, hi = n * veh.engine_thrust * veh.throttle_min, n * veh.engine_thrust * veh.throttle_max
    assert np.all(tmag >= lo * 0.995) and np.all(tmag <= hi * 1.001)
    assert p.lossless_gap / np.max(p.sigma) < 1e-3
    # ends at the gate, moving straight down at the gate speed
    assert np.allclose(p.r[-1], tw.gate_point, atol=1e-3)
    assert np.allclose(p.v[-1], [0, 0, -tw.gate_speed], atol=1e-3)
    # thrust tilt within the cone
    tilt = np.degrees(np.arctan2(np.linalg.norm(p.thrust[:, :2], axis=1), p.thrust[:, 2]))
    assert tilt.max() <= veh.max_tilt_deg + 0.05
    # fuel-optimal structure: the 3-engine phase is a min-max (or max-min-max) profile
    frac = (tmag - lo) / (hi - lo)
    phase2 = frac[p.t[:-1] >= sc.switch_time]
    assert np.mean((phase2 < 0.05) | (phase2 > 0.95)) > 0.8


def _vertical_problem():
    tower = Tower(catch_point=(0.0, 0.0, 0.0), gate_height=0.0, standoff=0.0, commit_time=0.0,
                  final_descent_rate=2.0, terminal_decel=1.0)
    r0, v0 = np.array([0.0, 0.0, 600.0]), np.array([0.0, 0.0, -80.0])
    return tower, r0, v0


def _shoot(veh, r0z, v0z, m0, t_s, n, dt=1e-3):
    """Min thrust until t_s, then max; integrate until speed falls to the gate speed."""
    lo, hi = veh.thrust_bounds(n)
    h, v, m, t = r0z, v0z, m0, 0.0
    while True:
        T = lo if t < t_s else hi
        a = T / m - G0
        v_new = v + a * dt
        if t > t_s and v_new >= -2.0:          # reached the final descent rate
            f = (-2.0 - v) / (v_new - v)
            return h + v * f * dt, m - T * f * dt / (veh.isp * G0), t + f * dt
        h += v * dt + 0.5 * a * dt ** 2
        v = v_new
        m -= T * dt / (veh.isp * G0)
        t += dt
        if t > 60:
            return None


def test_vertical_descent_matches_bang_bang_shooting_solution():
    tower, r0, v0 = _vertical_problem()
    m0 = 305e3   # 30 t of propellant: enough for this descent
    kw = dict(switch_time=None, engines=(3, 3), use_offshore=False, use_glideslope=False, final_align=False,
              n_nodes=120)
    p = G.solve_earliest(veh, tower, r0, v0, m0, tf_lo=2.0, tf_hi=30.0, tol=0.02, drag=False, **kw)
    assert p is not None
    # independent solution: find the switch time at which the max-thrust burn ends at h = 0
    t_s = brentq(lambda ts: _shoot(veh, r0[2], v0[2], m0, ts, 3)[0], 0.0, 5.0, xtol=1e-6)
    h_end, m_end, t_end = _shoot(veh, r0[2], v0[2], m0, t_s, 3)
    fuel_shoot = m0 - m_end
    assert abs(p.fuel - fuel_shoot) / fuel_shoot < 0.01
    assert abs(p.tf - t_end) < 0.15


def nlp_free_time(r0, v0, m0, N, x0, tf0):
    """SLSQP on the original non-convex problem (thrust magnitude in a ring, exact mass),
    3 engines, no drag, with the flight time as a decision variable."""
    tower = Tower(catch_point=(0.0, 0.0, 0.0), gate_height=0.0, standoff=0.0, commit_time=0.0,
                  final_descent_rate=2.0, terminal_decel=1.0)
    lo, hi = veh.thrust_bounds(3)
    alpha, cmax = veh.alpha, np.cos(np.radians(veh.max_tilt_deg))

    def rollout(x):
        u, tf = x[:-1].reshape(N, 3), x[-1]
        dt = tf / N
        r, v, z = r0.copy(), v0.copy(), np.log(m0)
        zs = [z]
        for k in range(N):
            acc = u[k] + np.array([0, 0, -G0])
            r = r + v * dt + 0.5 * acc * dt ** 2
            v = v + acc * dt
            z = z - alpha * np.linalg.norm(u[k]) * dt
            zs.append(z)
        return r, v, np.array(zs)

    def eq(x):
        r, v, _ = rollout(x)
        return np.concatenate([r - tower.gate_point, v - np.array([0, 0, -2.0])])

    def ineq(x):
        u = x[:-1].reshape(N, 3)
        _, _, zs = rollout(x)
        un = np.linalg.norm(u, axis=1)
        return np.concatenate([un * np.exp(zs[1:]) - lo, hi - un * np.exp(zs[:-1]), u[:, 2] - cmax * un]) / 1e5

    res = minimize(lambda x: m0 - np.exp(rollout(x)[2][-1]), np.r_[x0, tf0], method="SLSQP",
                   bounds=[(None, None)] * (3 * N) + [(2.0, 40.0)],
                   constraints=[{"type": "eq", "fun": eq}, {"type": "ineq", "fun": ineq}],
                   options={"maxiter": 1000, "ftol": 1e-10})
    return res, res.x[-1], m0 - np.exp(rollout(res.x)[2][-1]), tower


def test_convex_solution_matches_nonconvex_nlp():
    """Free final time: the convex method's earliest feasible time and fuel must match a
    general nonlinear optimiser on the original, non-convex problem started elsewhere.
    (At a fixed flight time this check is weak: the fuel is almost independent of the
    thrust profile there, since the vertical impulse is fixed by the boundary conditions.)"""
    r0, v0, m0, N = np.array([-60.0, 20.0, 700.0]), np.array([12.0, -3.0, -85.0]), 310e3, 16
    res, tf_nlp, fuel_nlp, tower = nlp_free_time(r0, v0, m0, N, np.tile([0.0, 0.0, 1.2 * G0], N), 14.0)
    assert res.success, res.message
    kw = dict(switch_time=None, engines=(3, 3), use_offshore=False, use_glideslope=False, final_align=False)
    p = G.solve_earliest(veh, tower, r0, v0, m0, tf_lo=2.0, tf_hi=40.0, tol=0.005, drag=False, n_nodes=N, **kw)
    assert abs(p.tf - tf_nlp) < 0.05
    # the convex bounds are a conservative Taylor approximation: never better than the NLP
    # by more than numerical noise, and within a fraction of a percent of it
    assert p.fuel >= fuel_nlp * (1 - 1e-3)
    assert (p.fuel - fuel_nlp) / fuel_nlp < 0.005


class _OpenLoop:
    """Replays a plan's thrust acceleration open loop, times the plan's own mass."""

    def __init__(self, plan):
        self.p = plan

    def reset(self, *a):
        pass

    def command(self, t, r, v, n):
        k = int(np.clip(np.searchsorted(self.p.t, t, side="right") - 1, 0, len(self.p.accel) - 1))
        m = np.interp(t, self.p.t, self.p.m)
        return self.p.accel[k] * m


def test_plan_dynamics_agree_with_simulator():
    r0, v0 = np.array(sc.r0), np.array(sc.v0)
    tf = G.solve_earliest(veh, tw, r0, v0, veh.wet_mass, switch_time=sc.switch_time).tf + 0.5
    p = G.solve_with_drag(veh, tw, r0, v0, veh.wet_mass, tf, passes=3, n_nodes=80, switch_time=sc.switch_time)
    assert p is not None
    res = sim.simulate(_OpenLoop(p), veh, tw, sc, sim.Dispersion(tau_lag=0.0), t_max=p.tf, record_every=1)
    i = int(np.argmin(np.abs(res.t - p.tf)))
    # open loop over ~14 s from 270 m/s: the remaining gap is discretisation of drag and thrust
    assert np.linalg.norm(res.r[i] - p.r[-1]) < 5.0
    assert np.linalg.norm(res.v[i] - p.v[-1]) < 1.5
    assert abs(res.m[i] - p.m[-1]) < 200.0


def test_drag_model_crossflow():
    a = G.drag_accel(np.array([0.0, 0.0, -100.0]), 300e3, np.zeros(3), veh)
    assert a[2] > 0 and abs(a[0]) < 1e-12
    # 15 m/s crosswind on the 71 m x 9 m side: ~0.22 m/s^2 at 280 t
    w = G.drag_accel(np.zeros(3), 280e3, np.array([15.0, 0.0, 0.0]), veh)
    assert 0.18 < w[0] < 0.26


@pytest.mark.parametrize("u", [[3.0, 4.0, 10.0], [0.1, 0.0, 10.0], [-8.0, 2.0, 5.0]])
def test_limit_tilt_keeps_vertical_component(u):
    from boostercatch.controllers import limit_tilt
    out = limit_tilt(np.array(u), 10.0)
    assert out[2] == pytest.approx(u[2])
    assert np.degrees(np.arctan2(np.linalg.norm(out[:2]), out[2])) <= 10.0 + 1e-9
