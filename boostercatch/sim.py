"""3-DoF flight simulator for the landing burn, with the things that go wrong in reality.

Truth model (what the controllers do not know exactly):
  * aerodynamic drag with an uncertain drag coefficient, relative to a gusty wind
    (mean wind + Ornstein-Uhlenbeck gusts)
  * thrust magnitude bias, throttle response lag (first order), Isp error
  * dry-mass error (the controller's mass estimate drifts from the truth)
  * engine failures at a random time (13 -> 12 engines, or 3 -> 2)
  * navigation noise on the position and velocity the controller sees

The booster is a point mass (no attitude dynamics); thrust direction stands in for
attitude, and its tilt from vertical is limited and checked at capture.

Capture is judged when the centre of mass descends through the catch height:
lateral error, horizontal and vertical speed and thrust tilt must all be inside the
arms' tolerances. A booster that misses (or whose controller aborts earlier) keeps
flying, diverting offshore, until it reaches the ground; every failure is then
classified by where it comes down: clear of the pad (a lost booster) or within
PAD_RADIUS of the tower (a pad strike: damage to the tower or launch mount, the
outcome that matters most). Hitting the tower structure is always a pad strike.
"""
from dataclasses import dataclass, field

import numpy as np

from .guidance import GRAV, drag_accel
from .vehicle import G0

DT = 0.02  # s, integration step
PAD_RADIUS = 40.0   # m around the catch point: tower, chopsticks and launch mount
GROUND_Z = 35.5     # m: centre-of-mass height when the booster's base reaches the ground


@dataclass
class Dispersion:
    """One Monte Carlo case. All zeros/defaults = the nominal flight."""
    dr0: tuple = (0.0, 0.0, 0.0)
    dv0: tuple = (0.0, 0.0, 0.0)
    wind_mean: tuple = (0.0, 0.0)     # m/s horizontal
    gust_sigma: float = 0.0           # m/s
    cd_scale: float = 1.0
    thrust_bias: float = 0.0          # fraction
    isp_scale: float = 1.0
    dry_mass_scale: float = 1.0
    engine_out_time: float = None     # s after ignition, or None
    nav_sigma_r: float = 0.0          # m
    nav_sigma_v: float = 0.0          # m/s
    tau_lag: float = 0.25             # s, throttle response time constant
    seed: int = 0


@dataclass
class Result:
    caught: bool
    reason: str
    t: np.ndarray
    r: np.ndarray
    v: np.ndarray
    m: np.ndarray
    thrust: np.ndarray
    n_engines: np.ndarray
    capture: dict = field(default_factory=dict)
    pad_safe: bool = True
    impact: tuple = None            # (x, y) where a failed booster comes down
    abort_reason: str = None
    abort_time: float = None


def tower_hit(r, radius=4.5):
    """The tower column sits behind the catch point: its front face at x = 9 m (4.5 m of
    clearance from the booster's skin at capture), 12 m deep, 20 m wide, ~146 m tall."""
    return (r[2] < 146.0 and r[0] + radius > 9.0 and r[0] - radius < 21.0 and abs(r[1]) < 10.0 + radius)


def ballistic_impact(r, v):
    """Where an engine-less booster at (r, v) reaches the ground (no drag: conservative
    on distance travelled)."""
    h = r[2] - GROUND_Z
    if h <= 0:
        return r[:2]
    t = (v[2] + np.sqrt(v[2] ** 2 + 2 * G0 * h)) / G0
    return r[:2] + v[:2] * t


def simulate(controller, veh, tower, scen, disp=Dispersion(), t_max=60.0, record_every=5):
    rng = np.random.default_rng(disp.seed)
    r = np.array(scen.r0, float) + np.array(disp.dr0)
    v = np.array(scen.v0, float) + np.array(disp.dv0)
    m = veh.wet_mass + (disp.dry_mass_scale - 1.0) * veh.dry_mass
    dry_true = veh.dry_mass * disp.dry_mass_scale
    gust = np.zeros(3)
    wind_mean = np.array([disp.wind_mean[0], disp.wind_mean[1], 0.0])
    thrust_actual = np.zeros(3)
    target = np.array(tower.catch_point)
    tau_lag = disp.tau_lag
    failed = False
    missed = None
    capture = {}
    controller.reset(veh, tower, scen)
    log_t, log_r, log_v, log_m, log_T, log_n = [], [], [], [], [], []
    t = 0.0
    step = 0
    while t < t_max:
        n_eng = scen.engines_phase1 if t < scen.switch_time else scen.engines_phase2
        if disp.engine_out_time is not None and t >= disp.engine_out_time:
            n_eng -= 1
            if not failed:
                failed = True
                controller.notify_engine_out(t)
        # what the controller sees
        r_est = r + rng.normal(0, disp.nav_sigma_r, 3) if disp.nav_sigma_r else r.copy()
        v_est = v + rng.normal(0, disp.nav_sigma_v, 3) if disp.nav_sigma_v else v.copy()
        cmd = controller.command(t, r_est, v_est, n_eng)
        # engines: magnitude within the live engines' range, tilt within limits, lag
        lo = n_eng * veh.engine_thrust * veh.throttle_min
        hi = n_eng * veh.engine_thrust * veh.throttle_max
        mag = np.linalg.norm(cmd)
        direc = cmd / mag if mag > 0 else np.array([0, 0, 1.0])
        max_tilt = np.radians(veh.divert_tilt_deg)   # physical limit; controllers enforce their own
        if np.arccos(np.clip(direc[2], -1, 1)) > max_tilt:
            h = direc[:2] / (np.linalg.norm(direc[:2]) + 1e-12)
            direc = np.array([h[0] * np.sin(max_tilt), h[1] * np.sin(max_tilt), np.cos(max_tilt)])
        target_thrust = direc * np.clip(mag, lo, hi) * (1 + disp.thrust_bias)
        thrust_actual += (target_thrust - thrust_actual) * (min(1.0, DT / tau_lag) if tau_lag > 0 else 1.0)
        # wind gusts (OU process, 3 s correlation)
        if disp.gust_sigma:
            gust += -gust * DT / 3.0 + disp.gust_sigma * np.sqrt(2 * DT / 3.0) * rng.normal(0, 1, 3) * np.array([1, 1, 0.3])
        wind = wind_mean + gust

        def deriv(rr, vv, mm):
            acc = thrust_actual / mm + GRAV + drag_accel(vv, mm, wind, veh) * disp.cd_scale
            mdot = -np.linalg.norm(thrust_actual) / (veh.isp * disp.isp_scale * G0)
            return vv, acc, mdot

        k1 = deriv(r, v, m)
        k2 = deriv(r + 0.5 * DT * k1[0], v + 0.5 * DT * k1[1], m + 0.5 * DT * k1[2])
        k3 = deriv(r + 0.5 * DT * k2[0], v + 0.5 * DT * k2[1], m + 0.5 * DT * k2[2])
        k4 = deriv(r + DT * k3[0], v + DT * k3[1], m + DT * k3[2])
        r_new = r + DT / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
        v_new = v + DT / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])
        m = m + DT / 6 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2])
        if step % record_every == 0:
            log_t.append(t); log_r.append(r.copy()); log_v.append(v.copy()); log_m.append(m)
            log_T.append(thrust_actual.copy()); log_n.append(n_eng)

        def finish(caught, reason, cap=None, impact=None):
            ar, at = getattr(controller, "abort_reason", None), getattr(controller, "abort_time", None)
            if not caught and ar is not None and ar not in ("missed the arms", "stalled above the arms"):
                reason = f"aborted ({ar})" + ("" if reason == "hit the ground" else f"; {reason}")
            if caught:
                safe = True
            elif impact is None:   # tower strike
                safe = False
            else:
                safe = bool(np.hypot(impact[0] - target[0], impact[1] - target[1]) > PAD_RADIUS)
            return Result(caught, reason, np.array(log_t), np.array(log_r), np.array(log_v), np.array(log_m),
                          np.array(log_T), np.array(log_n), cap or capture, safe,
                          None if impact is None else (float(impact[0]), float(impact[1])), ar, at)

        if m <= dry_true:
            return finish(False, missed or "out of propellant", impact=ballistic_impact(r, v))
        if tower_hit(r_new) and r_new[2] > 0:
            return finish(False, "hit the tower")
        if r_new[2] <= GROUND_Z:
            return finish(False, missed or "hit the ground", impact=r_new[:2])
        if missed is None and t > 1.0 and r[2] >= target[2] > r_new[2]:
            # crossing the catch height: interpolate the state at the crossing
            f = (r[2] - target[2]) / (r[2] - r_new[2])
            rc, vc = r + f * (r_new - r), v + f * (v_new - v)
            tilt = np.degrees(np.arccos(np.clip(thrust_actual[2] / (np.linalg.norm(thrust_actual) + 1e-9), -1, 1)))
            capture = {"lateral_error": float(np.linalg.norm(rc[:2] - target[:2])),
                       "vz": float(vc[2]), "vh": float(np.linalg.norm(vc[:2])), "tilt_deg": float(tilt),
                       "propellant_left": float(m - dry_true), "time": float(t)}
            ok = (capture["lateral_error"] <= tower.tol_lateral and abs(capture["vz"]) <= tower.tol_vertical_speed
                  and capture["vh"] <= tower.tol_lateral_speed and capture["tilt_deg"] <= tower.tol_tilt_deg)
            if ok and getattr(controller, "abort_reason", None) is None:
                log_t.append(t); log_r.append(rc); log_v.append(vc); log_m.append(m)
                log_T.append(thrust_actual.copy()); log_n.append(n_eng)
                return finish(True, "caught", capture)
            reasons = [k for k, bad in (("lateral miss", capture["lateral_error"] > tower.tol_lateral),
                                        ("too fast vertically", abs(capture["vz"]) > tower.tol_vertical_speed),
                                        ("drifting sideways", capture["vh"] > tower.tol_lateral_speed),
                                        ("tilted", capture["tilt_deg"] > tower.tol_tilt_deg)) if bad]
            missed = "missed: " + (", ".join(reasons) or "diverting")
            if hasattr(controller, "abort"):
                controller.abort(t, "missed the arms")
        if missed is None and v_new[2] > 5.0 and t > 3.0:
            # braked too hard and is climbing: at minimum throttle the engines can roughly
            # hover the booster, so it may never come back down to the arms
            missed = "stalled above the arms"
            if hasattr(controller, "abort"):
                controller.abort(t, "stalled above the arms")
        r, v = r_new, v_new
        t += DT
        step += 1
    safe = bool(np.hypot(r[0] - target[0], r[1] - target[1]) > PAD_RADIUS)
    return Result(False, missed or "timeout", np.array(log_t), np.array(log_r), np.array(log_v), np.array(log_m),
                  np.array(log_T), np.array(log_n), capture, safe, (float(r[0]), float(r[1])),
                  getattr(controller, "abort_reason", None), getattr(controller, "abort_time", None))
